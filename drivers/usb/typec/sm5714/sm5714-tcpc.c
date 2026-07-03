// SPDX-License-Identifier: GPL-2.0
/*
 * Silicon Mitus SM5714 USB Type-C Port Controller shim for the kernel's USB-PD
 * Power Delivery policy engine (drivers/usb/typec/tcpm).
 *
 * The SM5714 PDIC (I2C 0x33) is an autonomous Type-C/PD controller with a
 * vendor-proprietary register layout -- not a standard TCPCI part -- so it is
 * bound to tcpm the way Qualcomm's PMIC Type-C is: this driver fills in a
 * struct tcpc_dev with callbacks tcpm calls "down", and feeds tcpm's state
 * machine by calling the tcpm_*() event functions "up" from the chip interrupt.
 *
 * This is the SINK-ONLY first slice of the tcpm port (the bespoke role driver in
 * drivers/usb/misc/sm5714-typec.c carried the whole USB-PD policy in software;
 * tcpm replaces that policy, and the shim keeps only the chip transport and CC/
 * role control).  A live on-device experiment confirmed the autonomous chip can
 * be held as a full-time passive sink under external control: forcing manual
 * sink (CC_CNTL1=0x45) detected attach/detach, resolved orientation, and ran the
 * full PD message FIFO -- the primitives tcpm needs.  Dual-role toggling, source/
 * Rp and OTG-VBUS sourcing are a later staged increment; this slice deliberately
 * regresses them so tcpm can drive a sink contract end-to-end and what breaks
 * under tcpm policy can be catalogued.
 *
 * Every register value is transcribed from the device's own downstream driver
 * (drivers/usb/typec/sm/sm5714/{sm5714_typec.c,sm5714_pd.c}) -- the authority for
 * this exact hardware -- and was confirmed live on the SM-X910 (Galaxy Tab S9
 * Ultra).
 */

#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec.h>

/* The SM5714 VBUS host/OTG + Samsung-AFC inhibit interface lives with the bespoke
 * cluster for now; the in-tree landing relocates this header (a mechanical move). */
#include "../../misc/sm5714-usb-vbus.h"

/*
 * Role / CC window (disjoint from the PD window below; one chip, two register
 * sets).  STATUS1 is the live status mirror; INT1 is its latched, read-to-clear
 * twin.  Bit assignments from the device's own sm5714_typec.h.
 */
#define SM5714_REG_INT1			0x01
#define SM5714_INT1_VBUSPOK		BIT(0)	/* VBUS present (assert edge) */
#define SM5714_INT1_ATTACH		BIT(3)
#define SM5714_INT1_DETACH		BIT(4)
#define SM5714_REG_INT_MASK1		0x06	/* a SET bit MASKS (disables) the source */
/* Enable attach + detach + VBUS-present; mask the rest: ~(BIT0|BIT3|BIT4). */
#define SM5714_INT_MASK1_SINK		((u8)~(SM5714_INT1_VBUSPOK | \
					       SM5714_INT1_ATTACH | \
					       SM5714_INT1_DETACH))

/*
 * INT2 (index 1 in the block read).  SRC_ADV_CHG fires when the attached source
 * changes its advertised Rp current -- i.e. the PD-3.0 collision-avoidance
 * SinkTxOk/SinkTxNG signal.  tcpm gates every sink-initiated AMS on its cached
 * CC value, so this edge MUST refresh that view (-> tcpm_cc_change); without it a
 * stale SinkTxNG livelocks tcpm's VDM discovery ("Sink TX No Go") even though the
 * source is steady at SinkTxOk.  VBUS_0V is the VBUS-deassert edge.
 */
#define SM5714_REG_INT2			0x02
#define SM5714_INT2_SRC_ADV_CHG		BIT(4)	/* source's advertised Rp changed */
#define SM5714_INT2_VBUS_0V		BIT(5)	/* VBUS fell to 0 V */
#define SM5714_REG_INT_MASK2		0x07
/* Enable SRC_ADV_CHG + VBUS_0V; mask the rest: ~(BIT4|BIT5). */
#define SM5714_INT_MASK2_SINK		((u8)~(SM5714_INT2_SRC_ADV_CHG | \
					       SM5714_INT2_VBUS_0V))

#define SM5714_REG_STATUS1		0x0b
#define SM5714_STATUS1_VBUSPOK		BIT(0)	/* live VBUS-present (get_vbus) */
#define SM5714_STATUS1_ATTACH		BIT(3)	/* live attach (CC resolved) */

#define SM5714_REG_CC_STATUS		0x28
#define SM5714_CC_ATTACH_MASK		0x07
#define SM5714_CC_ATTACH_SRC		0x01	/* a source is on the cable -> we sink */
#define SM5714_CC_ATTACH_SNK		0x02	/* a sink is on the cable   -> we source */
#define SM5714_CC_CABLE_FLIP		BIT(5)	/* 0 = CC1, 1 = CC2 */
#define SM5714_CC_ADV_CURR		0x18	/* source's advertised Rp current */
#define SM5714_CC_ADV_CURR_DEF		0x00	/* Rp-default (0.5 A) */
#define SM5714_CC_ADV_CURR_1_5A		0x08	/* Rp-1.5 A */
#define SM5714_CC_ADV_CURR_3_0A		0x10	/* Rp-3.0 A */

/*
 * CC role control.  CORRECTION baked in from the device's own
 * sm5714_rprd_mode_change: 0x45 = manual SINK (present Rd), 0x49 = manual SOURCE
 * (present Rp), 0x40 = autonomous DRP.  CC_CNTL3 is the paired base register.
 */
#define SM5714_REG_CC_CNTL1		0x29
#define SM5714_CC_CNTL1_DRP		0x40	/* autonomous dual-role toggling */
#define SM5714_CC_CNTL1_SINK		0x45	/* present Rd (manual sink) */
#define SM5714_CC_CNTL1_SOURCE		0x49	/* present Rp (manual source; phase-2) */
#define SM5714_REG_CC_CNTL3		0x2b
#define SM5714_CC_CNTL3_BASE		0x80	/* the base the manual modes pair with */

/*
 * USB-PD protocol-layer window.  The PHY does CRC32 / GoodCRC / retry / message-
 * ID stamping in hardware; software only moves the message FIFOs and reacts to
 * the INT4 protocol-layer interrupts.  INT4 is read-to-clear (latched); the live
 * INT is unmasked via INT_MASK4 (active-low: a SET bit masks).
 */
#define SM5714_REG_INT4			0x04	/* index 3 in the INT1..INT5 block read */
#define SM5714_INT4_RX_DONE		BIT(0)	/* a PD message was received */
#define SM5714_INT4_TX_DONE		BIT(1)	/* our TX got the partner's GoodCRC */
#define SM5714_INT4_TX_SOP_ERR		BIT(2)	/* TX failed after HW retries */
#define SM5714_INT4_HRST_RCVED		BIT(5)	/* a hard reset was received */
#define SM5714_INT4_TX_DISCARD		BIT(7)	/* TX pre-empted by an incoming message */
#define SM5714_REG_INT_MASK4		0x09
/* Unmask only the INT4 bits we handle: ~(RX|TX|TXERR|HRST|DISCARD) == 0x58. */
#define SM5714_INT_MASK4_PD		((u8)~(SM5714_INT4_RX_DONE | \
					       SM5714_INT4_TX_DONE | \
					       SM5714_INT4_TX_SOP_ERR | \
					       SM5714_INT4_HRST_RCVED | \
					       SM5714_INT4_TX_DISCARD))
#define SM5714_INT_MASK4_ALL		0xff

#define SM5714_REG_PD_CNTL1		0x38
#define SM5714_PD_CNTL1_ENABLE		0x08	/* enable the PD protocol layer (SOP) */
#define SM5714_PD_CNTL1_DISABLE		0x00
#define SM5714_REG_PD_CNTL2		0x39	/* bit0 = DFP, bit1 = source */
#define SM5714_PD_CNTL2_DFP		BIT(0)
#define SM5714_PD_CNTL2_SRC		BIT(1)
#define SM5714_REG_PD_CNTL4		0x3b	/* protocol/hard/cable-reset control */
#define SM5714_PD_CNTL4_CABLE_RESET	0x02	/* send a Cable Reset (SM5714_ATTACH_SOURCE<<1) */
#define SM5714_PD_CNTL4_HARD_RESET	0x04	/* send a Hard Reset  (SM5714_ATTACH_SOURCE<<2) */
#define SM5714_PD_CNTL4_PRL_RESET	0x08	/* reset the protocol layer (<<3) */

#define SM5714_REG_RX_HEADER_00		0x42	/* 2-byte block: PD message header */
#define SM5714_REG_RX_PAYLOAD		0x44	/* N*4-byte block: data objects */
#define SM5714_REG_RX_BUF		0x5e	/* write 0x80 = "RX read done" (mandatory) */
#define SM5714_RX_BUF_READ_DONE		0x80
#define SM5714_REG_RX_BUF_ST		0x5f	/* write 0x10 = flush RX buffer */
#define SM5714_RX_BUF_FLUSH		0x10
#define SM5714_REG_TX_HEADER_00		0x60	/* 2-byte block: PD message header */
#define SM5714_REG_TX_PAYLOAD		0x62	/* N*4-byte block: data objects */
#define SM5714_REG_TX_REQ		0x7e	/* write 0x07 = send queued SOP message */
#define SM5714_TX_REQ_SOP		0x07

struct sm5714_tcpc {
	struct i2c_client *client;
	struct tcpm_port *port;
	struct tcpc_dev tcpc;
	struct fwnode_handle *fwnode;	/* the synthetic sink-caps connector node */
	struct mutex lock;		/* serializes the i2c register sequences */
	bool pd_rx_enabled;		/* set_pd_rx state; gates INT4 handling */
};

#define tcpc_to_sm5714(p) container_of(p, struct sm5714_tcpc, tcpc)

static int sm5714_read(struct sm5714_tcpc *st, u8 reg)
{
	return i2c_smbus_read_byte_data(st->client, reg);
}

static int sm5714_write(struct sm5714_tcpc *st, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(st->client, reg, val);
}

/* ---- tcpc_dev callbacks (tcpm calls these "down") ---------------------------- */

/*
 * Establish the CC_CNTL3 base that the manual-CC modes pair with.  The on-device
 * experiment held set_cc(Rd)=0x45 as a single write only because CC_CNTL3 was at
 * 0x80; the chip is not assumed to power up there, so init writes it explicitly.
 */
static int sm5714_tcpc_init(struct tcpc_dev *tcpc)
{
	struct sm5714_tcpc *st = tcpc_to_sm5714(tcpc);
	int ret;

	mutex_lock(&st->lock);
	ret = sm5714_write(st, SM5714_REG_CC_CNTL3, SM5714_CC_CNTL3_BASE);
	mutex_unlock(&st->lock);

	return ret;
}

/* VBUS-present is STATUS1 bit0 (VBUSPOK) -- the same register read for attach. */
static int sm5714_tcpc_get_vbus(struct tcpc_dev *tcpc)
{
	struct sm5714_tcpc *st = tcpc_to_sm5714(tcpc);
	int status1;

	mutex_lock(&st->lock);
	status1 = sm5714_read(st, SM5714_REG_STATUS1);
	mutex_unlock(&st->lock);

	if (status1 < 0)
		return status1;

	return !!(status1 & SM5714_STATUS1_VBUSPOK);
}

/*
 * Report the CC pin states tcpm needs.  As a sink we see the source's Rp on the
 * active CC line (decoded from CC_STATUS) and Open on the other; orientation
 * comes from the cable-flip bit.  Returns Open/Open when nothing is attached.
 */
static int sm5714_tcpc_get_cc(struct tcpc_dev *tcpc,
			      enum typec_cc_status *cc1,
			      enum typec_cc_status *cc2)
{
	struct sm5714_tcpc *st = tcpc_to_sm5714(tcpc);
	enum typec_cc_status active = TYPEC_CC_OPEN;
	int status1, cc;

	*cc1 = TYPEC_CC_OPEN;
	*cc2 = TYPEC_CC_OPEN;

	mutex_lock(&st->lock);
	status1 = sm5714_read(st, SM5714_REG_STATUS1);
	if (status1 < 0) {
		mutex_unlock(&st->lock);
		return status1;
	}
	if (!(status1 & SM5714_STATUS1_ATTACH)) {
		mutex_unlock(&st->lock);
		return 0;		/* detached -> Open/Open */
	}
	cc = sm5714_read(st, SM5714_REG_CC_STATUS);
	mutex_unlock(&st->lock);

	if (cc < 0)
		return cc;

	switch (cc & SM5714_CC_ATTACH_MASK) {
	case SM5714_CC_ATTACH_SRC:		/* a source is on the cable: report its Rp */
		switch (cc & SM5714_CC_ADV_CURR) {
		case SM5714_CC_ADV_CURR_1_5A:
			active = TYPEC_CC_RP_1_5;
			break;
		case SM5714_CC_ADV_CURR_3_0A:
			active = TYPEC_CC_RP_3_0;
			break;
		default:
			active = TYPEC_CC_RP_DEF;
			break;
		}
		break;
	case SM5714_CC_ATTACH_SNK:		/* a sink is on the cable (phase-2 source) */
		active = TYPEC_CC_RD;
		break;
	default:				/* audio / unknown -> leave Open */
		return 0;
	}

	if (cc & SM5714_CC_CABLE_FLIP)
		*cc2 = active;
	else
		*cc1 = active;

	return 0;
}

/* Present Rd / Rp / release, per tcpm's requested role. */
static int sm5714_tcpc_set_cc(struct tcpc_dev *tcpc, enum typec_cc_status cc)
{
	struct sm5714_tcpc *st = tcpc_to_sm5714(tcpc);
	u8 cntl1;
	int ret;

	switch (cc) {
	case TYPEC_CC_RD:			/* sink: present Rd (the v1 path) */
		cntl1 = SM5714_CC_CNTL1_SINK;
		break;
	case TYPEC_CC_RP_DEF:
	case TYPEC_CC_RP_1_5:
	case TYPEC_CC_RP_3_0:			/* source: present Rp (phase-2) */
		cntl1 = SM5714_CC_CNTL1_SOURCE;
		break;
	case TYPEC_CC_OPEN:
	default:				/* release to the chip's neutral DRP idle */
		cntl1 = SM5714_CC_CNTL1_DRP;
		break;
	}

	mutex_lock(&st->lock);
	ret = sm5714_write(st, SM5714_REG_CC_CNTL1, cntl1);
	mutex_unlock(&st->lock);

	return ret;
}

/*
 * Sink-only v1: present Rd and let the chip self-detect the attach -- it holds
 * the commanded CC mode across the whole plug cycle (no per-detach re-assert), as
 * proven on-device.  A dual-role / source port is phase-2.
 *
 * The autonomous chip latches an attach only on a physical CC edge (INT1_ATTACH)
 * and then HOLDS that state for the whole plug cycle -- it does NOT re-emit the
 * edge when tcpm merely restarts toggling.  So whenever tcpm (re)enters TOGGLING
 * while a partner is already physically attached -- at boot with a charger already
 * inserted, or on the SNK_UNATTACHED->TOGGLING leg of a hard-reset / port-reset /
 * error-recovery -- no fresh edge ever arrives and tcpm would starve in TOGGLING
 * at online=0 until a physical replug (charging silently degrades to the AFC/buck
 * fallback).  Synthesize the missing edge from the chip's held attach status so
 * tcpm re-reads CC (get_cc) and proceeds to attach.  tcpm_cc_change() only queues
 * an event (leaf spinlock, no port lock), so it is safe to call here.
 */
static int sm5714_tcpc_start_toggling(struct tcpc_dev *tcpc,
				      enum typec_port_type port_type,
				      enum typec_cc_status cc)
{
	struct sm5714_tcpc *st = tcpc_to_sm5714(tcpc);
	int ret, status1;

	ret = sm5714_tcpc_set_cc(tcpc, port_type == TYPEC_PORT_SRC ?
					TYPEC_CC_RP_DEF : TYPEC_CC_RD);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	status1 = sm5714_read(st, SM5714_REG_STATUS1);
	mutex_unlock(&st->lock);
	if (st->port && status1 >= 0 && (status1 & SM5714_STATUS1_ATTACH))
		tcpm_cc_change(st->port);

	return 0;
}

/* Orientation is resolved in hardware (CC_STATUS flip bit); nothing to set. */
static int sm5714_tcpc_set_polarity(struct tcpc_dev *tcpc,
				    enum typec_cc_polarity polarity)
{
	return 0;
}

/* VCONN sourcing is a source-role concern; no-op for the sink-only slice. */
static int sm5714_tcpc_set_vconn(struct tcpc_dev *tcpc, bool on)
{
	return 0;
}

/*
 * OTG VBUS sourcing is owned by the SM5714 charger (host role).  The sink slice
 * only ever sees set_vbus(false); route to the proven OTG path defensively.
 */
static int sm5714_tcpc_set_vbus(struct tcpc_dev *tcpc, bool on, bool charge)
{
	return sm5714_usb_vbus_set_host(on);
}

/* Set the data/power role bits the PHY stamps into outgoing messages. */
static int sm5714_tcpc_set_roles(struct tcpc_dev *tcpc, bool attached,
				 enum typec_role role,
				 enum typec_data_role data)
{
	struct sm5714_tcpc *st = tcpc_to_sm5714(tcpc);
	int cntl2, ret;

	mutex_lock(&st->lock);
	cntl2 = sm5714_read(st, SM5714_REG_PD_CNTL2);
	if (cntl2 < 0) {
		mutex_unlock(&st->lock);
		return cntl2;
	}
	cntl2 &= ~(SM5714_PD_CNTL2_DFP | SM5714_PD_CNTL2_SRC);
	if (data == TYPEC_HOST)
		cntl2 |= SM5714_PD_CNTL2_DFP;
	if (role == TYPEC_SOURCE)
		cntl2 |= SM5714_PD_CNTL2_SRC;
	ret = sm5714_write(st, SM5714_REG_PD_CNTL2, cntl2);
	mutex_unlock(&st->lock);

	return ret;
}

/*
 * Enable / disable reception of PD messages.  Enabling resets the protocol layer
 * (flush RX + PRL reset, re-zeroing the HW message-ID, as the device's own
 * PE_SNK_Startup does), unmasks the INT4 PD interrupts, and enables the layer.
 *
 * Inhibiting Samsung-AFC while PD is up is NOT optional and NOT a deferrable
 * edge-case: AFC is proprietary D+/D- signalling on the charger/MUIC subsystem
 * that tcpm has no concept of, so without this gate the charging worker could
 * fire an AFC handshake into the middle of tcpm's negotiation and break it on the
 * shared cable.  Mirrors what the bespoke driver did at PD bring-up.
 */
static int sm5714_tcpc_set_pd_rx(struct tcpc_dev *tcpc, bool on)
{
	struct sm5714_tcpc *st = tcpc_to_sm5714(tcpc);
	int ret;

	mutex_lock(&st->lock);
	if (on) {
		sm5714_write(st, SM5714_REG_RX_BUF_ST, SM5714_RX_BUF_FLUSH);
		sm5714_write(st, SM5714_REG_PD_CNTL4, SM5714_PD_CNTL4_PRL_RESET);
		sm5714_write(st, SM5714_REG_INT_MASK4, SM5714_INT_MASK4_PD);
		ret = sm5714_write(st, SM5714_REG_PD_CNTL1, SM5714_PD_CNTL1_ENABLE);
		st->pd_rx_enabled = (ret == 0);
	} else {
		ret = sm5714_write(st, SM5714_REG_PD_CNTL1, SM5714_PD_CNTL1_DISABLE);
		sm5714_write(st, SM5714_REG_INT_MASK4, SM5714_INT_MASK4_ALL);
		st->pd_rx_enabled = false;
	}
	mutex_unlock(&st->lock);

	sm5714_usb_vbus_inhibit_afc(on);

	return ret;
}

/*
 * Transmit a PD message (or a hard/cable-reset signal).  For a SOP message the
 * PHY does CRC/GoodCRC/retry and stamps the message-ID, so this just loads the
 * FIFO and fires TX_REQ; the result is reported asynchronously to tcpm via
 * tcpm_pd_transmit_complete() on the INT4 TX_DONE / TX_SOP_ERR interrupt.  A
 * hard/cable reset is a single read-modify-write of PD_CNTL4 (the device's own
 * sm5714_hard_reset).  pd_message is byte-identical to the on-wire FIFO format.
 */
static int sm5714_tcpc_pd_transmit(struct tcpc_dev *tcpc,
				   enum tcpm_transmit_type type,
				   const struct pd_message *msg,
				   unsigned int negotiated_rev)
{
	struct sm5714_tcpc *st = tcpc_to_sm5714(tcpc);
	int ret, val, cnt;

	mutex_lock(&st->lock);

	switch (type) {
	case TCPC_TX_HARD_RESET:
	case TCPC_TX_CABLE_RESET:
		val = sm5714_read(st, SM5714_REG_PD_CNTL4);
		if (val < 0) {
			ret = val;
			break;
		}
		val |= (type == TCPC_TX_HARD_RESET) ?
			SM5714_PD_CNTL4_HARD_RESET : SM5714_PD_CNTL4_CABLE_RESET;
		ret = sm5714_write(st, SM5714_REG_PD_CNTL4, val);
		break;
	default:				/* SOP* message */
		if (!msg) {
			ret = -EINVAL;
			break;
		}
		cnt = pd_header_cnt_le(msg->header);
		ret = i2c_smbus_write_i2c_block_data(st->client,
						     SM5714_REG_TX_HEADER_00, 2,
						     (const u8 *)&msg->header);
		if (ret)
			break;
		if (cnt > 0) {
			ret = i2c_smbus_write_i2c_block_data(st->client,
							     SM5714_REG_TX_PAYLOAD,
							     cnt * 4,
							     (const u8 *)msg->payload);
			if (ret)
				break;
		}
		ret = sm5714_write(st, SM5714_REG_TX_REQ, SM5714_TX_REQ_SOP);
		break;
	}

	mutex_unlock(&st->lock);

	return ret;
}

/* ---- chip interrupt -> tcpm event feed (the driver calls tcpm "up") --------- */

/*
 * Pull one received PD message out of the RX FIFO into a struct pd_message.  The
 * final RX_BUF write is mandatory -- without it the controller will not release
 * the buffer for the next message.  Caller holds the lock.
 */
static int sm5714_tcpc_read_message(struct sm5714_tcpc *st, struct pd_message *msg)
{
	int ret, cnt;

	ret = i2c_smbus_read_i2c_block_data(st->client, SM5714_REG_RX_HEADER_00,
					    2, (u8 *)&msg->header);
	if (ret < 0)
		return ret;

	cnt = pd_header_cnt_le(msg->header);
	if (cnt > PD_MAX_PAYLOAD)
		cnt = PD_MAX_PAYLOAD;
	if (cnt > 0) {
		ret = i2c_smbus_read_i2c_block_data(st->client,
						    SM5714_REG_RX_PAYLOAD,
						    cnt * 4, (u8 *)msg->payload);
		if (ret < 0)
			return ret;
	}

	i2c_smbus_write_byte_data(st->client, SM5714_REG_RX_BUF,
				  SM5714_RX_BUF_READ_DONE);
	return 0;
}

static irqreturn_t sm5714_tcpc_irq(int irq, void *data)
{
	struct sm5714_tcpc *st = data;
	struct pd_message msg = { };
	u8 intr[5];
	int ret;
	bool cc_change = false, vbus_change = false, rx = false;
	bool tx_ok = false, tx_fail = false, tx_disc = false, hrst = false;

	mutex_lock(&st->lock);

	/*
	 * One block read of the five latched INT registers (read-to-clear).  The
	 * line is level-low / ONESHOT, so every unmasked-but-unread bit would hold
	 * it asserted; reading the whole bank each time clears them, as the
	 * device's own driver does.  Fall back to clearing INT1 alone.
	 */
	ret = i2c_smbus_read_i2c_block_data(st->client, SM5714_REG_INT1, 5, intr);
	if (ret < 0) {
		i2c_smbus_read_byte_data(st->client, SM5714_REG_INT1);
		intr[0] = 0;
		intr[1] = 0;
		intr[3] = 0;
	}

	/* INT1 (intr[0]): attach/detach -> CC change; VBUS edge -> VBUS change. */
	if (intr[0] & (SM5714_INT1_ATTACH | SM5714_INT1_DETACH))
		cc_change = true;
	if (intr[0] & (SM5714_INT1_ATTACH | SM5714_INT1_DETACH | SM5714_INT1_VBUSPOK))
		vbus_change = true;

	/* INT2 (intr[1]): a source Rp change (SinkTxOk/NG) must refresh tcpm's CC
	 * view; VBUS_0V is the deassert edge. */
	if (intr[1] & SM5714_INT2_SRC_ADV_CHG)
		cc_change = true;
	if (intr[1] & SM5714_INT2_VBUS_0V)
		vbus_change = true;

	/* INT4 (intr[3]): the PD protocol layer, only once RX is enabled. */
	if (st->pd_rx_enabled) {
		if (intr[3] & SM5714_INT4_RX_DONE)
			rx = true;
		if (intr[3] & SM5714_INT4_TX_DONE)
			tx_ok = true;
		if (intr[3] & SM5714_INT4_TX_SOP_ERR)
			tx_fail = true;
		if (intr[3] & SM5714_INT4_TX_DISCARD)
			tx_disc = true;
		if (intr[3] & SM5714_INT4_HRST_RCVED)
			hrst = true;
	}

	/* Read the message out of the FIFO under the lock; hand it up after. */
	if (rx && sm5714_tcpc_read_message(st, &msg) < 0)
		rx = false;

	mutex_unlock(&st->lock);

	/*
	 * All tcpm_*() up-calls happen with the lock dropped: they queue into
	 * tcpm's state machine and take tcpm-internal locks, so holding ours across
	 * them would invite a lock-order inversion (mirrors qcom's port.c).
	 */
	if (cc_change)
		tcpm_cc_change(st->port);
	if (vbus_change)
		tcpm_vbus_change(st->port);
	if (rx)
		tcpm_pd_receive(st->port, &msg, TCPC_TX_SOP);
	if (tx_ok)
		tcpm_pd_transmit_complete(st->port, TCPC_TX_SUCCESS);
	if (tx_fail)
		tcpm_pd_transmit_complete(st->port, TCPC_TX_FAILED);
	if (tx_disc)
		tcpm_pd_transmit_complete(st->port, TCPC_TX_DISCARDED);
	if (hrst)
		tcpm_pd_hard_reset(st->port);

	return IRQ_HANDLED;
}

/* ---- probe / remove --------------------------------------------------------- */

/*
 * The synthetic connector node tcpm reads its capabilities from.  Sink-only for
 * v1 (power-role = sink) so tcpm runs the sink state machine rather than DRP.
 * The sink advertises 5 V and 9 V fixed (PDO1 must be vSafe5V per spec) so tcpm
 * climbs to a 9 V fixed contract -- the charge-pump's input rail -- plus a PPS
 * APDO covering the fast-charge window (the SM5440 pump steps it via the tcpm
 * source-psy).  A software_node lets tcpm register without the dual-role DT
 * connector graph -- tcpm tolerates a missing usb_role_switch (data-role muxing
 * to dwc3 is phase-2).
 *
 * The PPS APDO advertises 5 A: the 2:1 charge-pump draws >3 A on the input at
 * the ~37 W rate, so a 3 A sink APDO would be spec-noncompliant for the real
 * draw.  This does not itself cap the request -- tcpm bounds a PPS step by the
 * SOURCE APDO max, never the sink APDO -- but it keeps the sink advertisement
 * honest for upstream.
 */
static const u32 sm5714_snk_pdos[] = {
	PDO_FIXED(5000, 3000, PDO_FIXED_USB_COMM | PDO_FIXED_DATA_SWAP),
	PDO_FIXED(9000, 3000, 0),
	PDO_PPS_APDO(3300, 11000, 5000),
};

static const struct property_entry sm5714_connector_props[] = {
	PROPERTY_ENTRY_STRING("data-role", "device"),
	PROPERTY_ENTRY_STRING("power-role", "sink"),
	PROPERTY_ENTRY_U32_ARRAY("sink-pdos", sm5714_snk_pdos),
	PROPERTY_ENTRY_U32("op-sink-microwatt", 2500000),
	{ }
};

static int sm5714_tcpc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sm5714_tcpc *st;
	int ret;

	if (!client->irq)
		return dev_err_probe(dev, -EINVAL, "no IRQ\n");

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->client = client;
	mutex_init(&st->lock);
	i2c_set_clientdata(client, st);

	st->fwnode = fwnode_create_software_node(sm5714_connector_props, NULL);
	if (IS_ERR(st->fwnode))
		return dev_err_probe(dev, PTR_ERR(st->fwnode),
				     "failed to create connector node\n");

	st->tcpc.fwnode = st->fwnode;
	st->tcpc.init = sm5714_tcpc_init;
	st->tcpc.get_vbus = sm5714_tcpc_get_vbus;
	st->tcpc.get_cc = sm5714_tcpc_get_cc;
	st->tcpc.set_cc = sm5714_tcpc_set_cc;
	st->tcpc.set_polarity = sm5714_tcpc_set_polarity;
	st->tcpc.set_vconn = sm5714_tcpc_set_vconn;
	st->tcpc.set_vbus = sm5714_tcpc_set_vbus;
	st->tcpc.set_roles = sm5714_tcpc_set_roles;
	st->tcpc.set_pd_rx = sm5714_tcpc_set_pd_rx;
	st->tcpc.start_toggling = sm5714_tcpc_start_toggling;
	st->tcpc.pd_transmit = sm5714_tcpc_pd_transmit;

	/*
	 * Unmask attach/detach + VBUS in INT1, mask the whole PD layer (INT4) until
	 * set_pd_rx enables it, and clear any latched state.
	 */
	mutex_lock(&st->lock);
	sm5714_write(st, SM5714_REG_INT_MASK1, SM5714_INT_MASK1_SINK);
	sm5714_write(st, SM5714_REG_INT_MASK2, SM5714_INT_MASK2_SINK);
	sm5714_write(st, SM5714_REG_INT_MASK4, SM5714_INT_MASK4_ALL);
	sm5714_read(st, SM5714_REG_INT1);
	mutex_unlock(&st->lock);

	/*
	 * Request the IRQ disabled (NO_AUTOEN) and enable it only after the port is
	 * registered, so an attach interrupt can never reach tcpm before it exists.
	 */
	ret = devm_request_threaded_irq(dev, client->irq, NULL, sm5714_tcpc_irq,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT |
					IRQF_NO_AUTOEN, "sm5714-tcpc", st);
	if (ret) {
		dev_err_probe(dev, ret, "failed to request IRQ\n");
		goto err_fwnode;
	}

	st->port = tcpm_register_port(dev, &st->tcpc);
	if (IS_ERR(st->port)) {
		ret = dev_err_probe(dev, PTR_ERR(st->port),
				    "failed to register tcpm port\n");
		goto err_fwnode;
	}

	enable_irq(client->irq);

	dev_info(dev, "SM5714 tcpm shim ready (sink-only first slice)\n");
	return 0;

err_fwnode:
	fwnode_remove_software_node(st->fwnode);
	return ret;
}

static void sm5714_tcpc_remove(struct i2c_client *client)
{
	struct sm5714_tcpc *st = i2c_get_clientdata(client);

	disable_irq(client->irq);
	tcpm_unregister_port(st->port);

	/* Drop PD, return CC to autonomous DRP, and release the AFC inhibit so a
	 * re-bind (or the bespoke driver) starts from the chip's neutral state. */
	mutex_lock(&st->lock);
	sm5714_write(st, SM5714_REG_PD_CNTL1, SM5714_PD_CNTL1_DISABLE);
	sm5714_write(st, SM5714_REG_CC_CNTL1, SM5714_CC_CNTL1_DRP);
	mutex_unlock(&st->lock);
	sm5714_usb_vbus_inhibit_afc(false);

	fwnode_remove_software_node(st->fwnode);
}

/*
 * Shares the "siliconmitus,sm5714-typec" compatible with the bespoke role driver
 * (drivers/usb/misc/sm5714-typec.c).  During bring-up this shim is a module that
 * is hand-bound after the bespoke driver is unbound, so only one ever owns the
 * device.  The eventual in-tree landing must disable CONFIG_USB_SM5714_TYPEC (the
 * bespoke driver) -- two drivers cannot both be built-in for one compatible.
 */
static const struct of_device_id sm5714_tcpc_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-typec" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_tcpc_of_match);

static struct i2c_driver sm5714_tcpc_driver = {
	.driver = {
		.name = "sm5714-tcpc",
		.of_match_table = sm5714_tcpc_of_match,
	},
	.probe = sm5714_tcpc_probe,
	.remove = sm5714_tcpc_remove,
};
module_i2c_driver(sm5714_tcpc_driver);

MODULE_DESCRIPTION("SM5714 USB Type-C Port Controller shim for tcpm (sink-only)");
MODULE_AUTHOR("ubuntu-tab project");
MODULE_LICENSE("GPL");
