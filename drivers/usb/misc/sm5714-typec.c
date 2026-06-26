// SPDX-License-Identifier: GPL-2.0
/*
 * Silicon Mitus SM5714 USB Type-C role (PDIC) driver
 *
 * The SM5714 combined PMIC includes a Type-C/PD controller (the "PDIC") reached
 * over I2C at 7-bit address 0x33 -- on a separate I2C-master-hub serial engine
 * from the charger/fuel-gauge (e.g. Samsung Galaxy Tab S9 Ultra, SM-X910, where
 * the PDIC sits on i2c_hub_9 and the charger 0x49 / fuel-gauge 0x71 on
 * i2c_hub_8).  This driver makes USB-C role detection automatic:
 *
 *   - it programs the controller for autonomous Dual-Role (DRP) toggling;
 *   - on each attach/detach it reads CC_STATUS and decodes the cable: a source
 *     on the cable (a charger or host PC) means we are the DEVICE/sink; a sink
 *     on the cable (a USB peripheral) means we are the HOST/source;
 *   - it drives the standard mainline usb_role_switch so dwc3 flips its data
 *     role (host <-> gadget) accordingly, and asks the SM5714 VBUS driver to
 *     source 5 V (host) or stay off (device/disconnected);
 *   - it registers a USB Type-C port (/sys/class/typec/portN) that reports the
 *     hardware-decided data/power role, the attached partner, and the plug
 *     orientation -- the standard userspace surface and the foundation for
 *     DP-altmode.
 *
 * This replaces the manual "host_vbus" sysfs toggle with automatic, cable-driven
 * role switching.  All register values are transcribed from the device's own
 * downstream driver (drivers/usb/typec/sm/sm5714/sm5714_typec.c) and confirmed
 * live on the hardware (CC_STATUS reads 0x01 with a charger, 0x02 with a
 * peripheral, 0x00 unplugged).
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>
#include <linux/workqueue.h>
#include <linux/completion.h>

#include "sm5714-usb-vbus.h"
#include "sm5714-typec.h"

/* Interrupt / status registers (latched INT read-to-clear; STATUS is live). */
#define SM5714_TYPEC_REG_INT1		0x01
#define SM5714_TYPEC_REG_INT_MASK1	0x06
#define SM5714_TYPEC_REG_STATUS1	0x0b
#define SM5714_TYPEC_INT1_ATTACH	BIT(3)
#define SM5714_TYPEC_INT1_DETACH	BIT(4)
/* STATUS1 is the live mirror of INT1: same bit layout. */
#define SM5714_TYPEC_STATUS1_ATTACH	BIT(3)
/* INT_MASK1: a set bit masks (disables) the source; clear to enable.  Enable
 * only ATTACH+DETACH (0xff & ~(BIT(3)|BIT(4)) == 0xe7). */
#define SM5714_TYPEC_INT_MASK1_ATTDET	0xe7

/* CC status + role control. */
#define SM5714_TYPEC_REG_CC_STATUS	0x28
#define SM5714_TYPEC_CC_ATTACH_MASK	0x07
#define SM5714_TYPEC_CC_ATTACH_SRC	0x01	/* source on cable -> we DEVICE */
#define SM5714_TYPEC_CC_ATTACH_SNK	0x02	/* sink on cable   -> we HOST   */
#define SM5714_TYPEC_CC_ATTACH_AUDIO	0x03	/* audio accessory             */
#define SM5714_TYPEC_CC_CABLE_FLIP	BIT(5)	/* 0=CC1 (normal), 1=CC2 (reverse) */
/*
 * The source's advertised Rp current (CC_STATUS bits 3-4) doubles as the PD-3.0
 * collision-avoidance signal: Rp-3.0A means "SinkTxOk" (the sink may start an AMS,
 * e.g. our keepalive re-Request); Rp-1.5A means "SinkTxNG" (defer).  Values from
 * the device's own driver (sm5714_typec.c sm5714_notify_rp_current_level): 0x00 =
 * 0.5A default, 0x08 = 1.5A, 0x10 = 3.0A.
 */
#define SM5714_TYPEC_CC_ADV_CURR	0x18	/* advertised-Rp-current field mask */
#define SM5714_TYPEC_CC_ADV_CURR_3A	0x10	/* Rp-3.0A == PD-3.0 SinkTxOk */
#define SM5714_TYPEC_REG_CC_CNTL1	0x29
#define SM5714_TYPEC_CC_CNTL1_DRP	0x40	/* autonomous Dual-Role toggling */

/*
 * USB Power Delivery (PD) protocol-layer window.  This is a register window on
 * the SAME 0x33 PDIC client, fully disjoint from the role/CC window above (the
 * SM5714 is one chip with two non-intersecting register sets).  The PD PHY does
 * the PD protocol layer -- CRC32, the GoodCRC handshake, and message-ID stamping
 * -- in HARDWARE; software only writes/reads the message FIFOs and reacts to the
 * INT4 protocol-layer interrupts.  All values transcribed from the device's own
 * downstream driver (drivers/usb/typec/sm/sm5714/{sm5714_typec.c,sm5714_pd.c}).
 */
#define SM5714_TYPEC_REG_INT4		0x04	/* PD protocol-layer interrupt (latched) */
#define SM5714_TYPEC_REG_INT_MASK4	0x09	/* active-low: a SET bit MASKS the source */
#define SM5714_TYPEC_INT4_RX_DONE	BIT(0)	/* a PD message was received */
#define SM5714_TYPEC_INT4_TX_DONE	BIT(1)	/* our TX got the partner's GoodCRC */
#define SM5714_TYPEC_INT4_TX_SOP_ERR	BIT(2)	/* TX failed after HW retries */
#define SM5714_TYPEC_INT4_HRST_RCVED	BIT(5)	/* hard reset received */
#define SM5714_TYPEC_INT4_TX_DISCARD	BIT(7)	/* TX aborted (pre-empted by an RX) */
/* Unmask only the INT4 bits we read+handle (0xff & ~(RX|TX|TXERR|HRST|DISCARD)). */
#define SM5714_TYPEC_INT_MASK4_PD \
	((u8)~(SM5714_TYPEC_INT4_RX_DONE | SM5714_TYPEC_INT4_TX_DONE | \
	       SM5714_TYPEC_INT4_TX_SOP_ERR | SM5714_TYPEC_INT4_HRST_RCVED | \
	       SM5714_TYPEC_INT4_TX_DISCARD))

#define SM5714_TYPEC_REG_PD_CNTL1	0x38
#define SM5714_TYPEC_PD_CNTL1_ENABLE	0x08	/* enable PD protocol layer for SOP */
#define SM5714_TYPEC_PD_CNTL1_DISABLE	0x00
#define SM5714_TYPEC_REG_PD_CNTL2	0x39	/* bit0=DFP(set)/UFP(clr), bit1=src(set)/snk(clr) */
#define SM5714_TYPEC_PD_CNTL2_DFP	BIT(0)
#define SM5714_TYPEC_PD_CNTL2_SRC	BIT(1)
#define SM5714_TYPEC_REG_PD_CNTL4	0x3b	/* protocol-layer reset / AMS control */
#define SM5714_TYPEC_PD_CNTL4_PRL_RESET	0x08	/* reset the PD protocol layer */
#define SM5714_TYPEC_REG_RX_SRC		0x41	/* low nibble = SOP type (0 = SOP) */
#define SM5714_TYPEC_REG_RX_HEADER_00	0x42	/* 2-byte block: PD message header */
#define SM5714_TYPEC_REG_RX_PAYLOAD	0x44	/* N*4-byte block: data objects */
#define SM5714_TYPEC_REG_RX_BUF		0x5e	/* write 0x80 = "RX read done" (mandatory) */
#define SM5714_TYPEC_RX_BUF_READ_DONE	0x80
#define SM5714_TYPEC_REG_RX_BUF_ST	0x5f	/* write 0x10 = flush RX buffer (protocol reset) */
#define SM5714_TYPEC_RX_BUF_FLUSH	0x10
#define SM5714_TYPEC_REG_TX_HEADER_00	0x60	/* 2-byte block: PD message header */
#define SM5714_TYPEC_REG_TX_PAYLOAD	0x62	/* N*4-byte block: data objects */
#define SM5714_TYPEC_REG_TX_REQ		0x7e	/* write 0x07 = "send queued SOP message" */
#define SM5714_TYPEC_TX_REQ_SOP		0x07

/* PD message-type codes (control: num_data_objs==0; data: num_data_objs 1-7). */
#define SM5714_PD_CTRL_GOODCRC		0x1
#define SM5714_PD_CTRL_ACCEPT		0x3
#define SM5714_PD_CTRL_REJECT		0x4
#define SM5714_PD_CTRL_PS_RDY		0x6
#define SM5714_PD_CTRL_GET_SOURCE_CAP	0x7
#define SM5714_PD_CTRL_WAIT		0xc
#define SM5714_PD_CTRL_SOFT_RESET	0xd
#define SM5714_PD_DATA_SOURCE_CAP	0x1
#define SM5714_PD_DATA_REQUEST		0x2
/* PD header field values. */
#define SM5714_PD_DATA_ROLE_UFP		0
#define SM5714_PD_POWER_ROLE_SINK	0
#define SM5714_PD_SPEC_REV_30		2	/* PPS requires PD revision 3.0 */
/* data_obj supply_type (bits 30-31). */
#define SM5714_PD_SUPPLY_FIXED		0
#define SM5714_PD_SUPPLY_BATTERY	1
#define SM5714_PD_SUPPLY_VARIABLE	2
#define SM5714_PD_SUPPLY_APDO		3	/* augmented; PPS when pps_supply==0 */

#define SM5714_PD_MAX_OBJ		7	/* max data objects in a PD message */

/* Control-message events the IRQ raises for the Request work item.  Sticky in
 * pd_evt so an event arriving before the work waits for it is never lost. */
#define SM5714_PD_EVT_ACCEPT		BIT(0)
#define SM5714_PD_EVT_REJECT		BIT(1)
#define SM5714_PD_EVT_WAIT		BIT(2)
#define SM5714_PD_EVT_PS_RDY		BIT(3)

/*
 * PPS Request target.  9 V is also reachable by Samsung AFC, so to make a
 * successful PD contract electrically falsifiable (not merely visible in dmesg)
 * we request a voltage AFC physically cannot produce -- AFC is 5 V/9 V quantized
 * -- yet inside PDO[5]'s 5-11 V PPS window and under the charger's fixed-PDO OVP.
 * Success is then VBUS rising 5 V -> ~10 V on the SM5440 telemetry, an outcome no
 * AFC negotiation can fake.  The op-current is only a ceiling; the buck draws
 * what it needs.
 */
#define SM5714_PD_REQ_MV		10000
#define SM5714_PD_REQ_MA		3000
#define SM5714_PD_T_ACCEPT_MS		400	/* tSenderResponse ~27 ms; generous */
#define SM5714_PD_T_PSRDY_MS		600	/* tPSTransition ~450-550 ms */
/*
 * Keepalive: once a PPS contract is established, the source expects a fresh
 * Request within tPPSTimeout (~12-15 s) or it Hard-Resets the contract back to
 * 5 V.  Re-Request well inside that window to hold the contract.  1 s leaves a
 * huge margin even after SinkTxNG deferrals stack up.
 */
#define SM5714_PD_KEEPALIVE_MS		1000	/* re-Request period (<< tPPSTimeout) */
#define SM5714_PD_SINKTX_RETRY_MS	200	/* re-check cadence while SinkTxNG */

/*
 * PD message header (16 bits) and data object (32 bits).  Bitfields transcribed
 * verbatim from the device's own headers (common/pdic_core.h msg_header_type and
 * sm/sm5714/sm5714_pd.h data_obj_type) so the on-wire packing matches byte-for-
 * byte; they are LSB-first, which is correct for this little-endian arm64 target.
 */
union sm5714_pd_header {
	u16 word;
	u8 byte[2];
	struct {
		unsigned msg_type:5;
		unsigned port_data_role:1;
		unsigned spec_revision:2;
		unsigned port_power_role:1;
		unsigned msg_id:3;		/* HW-stamped on TX; leave 0 */
		unsigned num_data_objs:3;
		unsigned extended:1;
	};
};

union sm5714_pd_obj {
	u32 object;
	u8 byte[4];
	struct {
		unsigned :30;
		unsigned supply_type:2;
	} supply;
	struct {				/* fixed supply PDO (source) */
		unsigned max_current:10;	/* 10 mA units */
		unsigned voltage:10;		/* 50 mV units */
		unsigned :10;
		unsigned supply_type:2;
	} fixed;
	struct {				/* augmented PDO (PPS) */
		unsigned max_current:7;		/* 50 mA units */
		unsigned :1;
		unsigned min_voltage:8;		/* 100 mV units */
		unsigned :1;
		unsigned max_voltage:8;		/* 100 mV units */
		unsigned :2;
		unsigned pps_power_limited:1;
		unsigned pps_supply:2;		/* 0 = PPS */
		unsigned supply_type:2;
	} apdo;
	struct {				/* programmable request data object (PPS) */
		unsigned op_current:7;		/* 50 mA units */
		unsigned :2;
		unsigned output_voltage:11;	/* 20 mV units */
		unsigned :3;
		unsigned unchunked_ext_msg_support:1;
		unsigned no_usb_suspend:1;
		unsigned usb_comm_capable:1;
		unsigned capability_mismatch:1;
		unsigned :1;
		unsigned object_position:3;	/* 1-based PDO index */
		unsigned :1;
	} rdo_pps;
};

/* No edge can be missed without leaving VBUS mis-sourced, so re-sync slowly. */
#define SM5714_TYPEC_POLL_MS		2000

struct sm5714_typec {
	struct i2c_client *client;
	struct usb_role_switch *role_sw;
	struct typec_port *port;
	struct typec_partner *partner;
	struct typec_capability cap;
	struct mutex lock;
	struct delayed_work resync;
	enum usb_role role;
	bool pd_enabled;		/* PD protocol layer brought up (sink) */
	bool pd_arm;			/* enable PD on the next sink-attach edge */
	bool pd_do_request;		/* negotiate (not just capture) on this attach */
	bool pd_negotiating;		/* a Request flow is in flight (work scheduled) */
	bool pd_contract_lost;		/* IRQ saw HRST/TX_SOP_ERR: contract genuinely gone */
	struct work_struct pd_req_work;	/* initial PD negotiation; runs off-IRQ (it blocks) */
	struct delayed_work pd_keepalive; /* re-Request loop that sustains the contract */
	struct completion pd_reply;	/* IRQ wakes the work on a control message */
	u32 pd_evt;			/* sticky SM5714_PD_EVT_* control-msg events */
	union sm5714_pd_obj pd_pdo[SM5714_PD_MAX_OBJ];	/* last captured source caps */
	int pd_npdo;
	u32 pd_target_mv;		/* commanded PPS request voltage; the keepalive
					 * carries it.  Seeded to SM5714_PD_REQ_MV; the
					 * SM5440 pump loop steps it via
					 * sm5714_pd_request_voltage(). */
	u32 pd_target_ma;		/* commanded PPS request current ceiling */
};

/*
 * The single bound PDIC instance, for the cross-driver PPS hooks the SM5440
 * charge-pump driver calls.  Read with READ_ONCE; set at the end of probe,
 * cleared at the start of remove.
 */
static struct sm5714_typec *sm5714_typec_instance;

/* PD bring-up/teardown, defined below; driven from the role-apply path. */
static int sm5714_pd_enable(struct sm5714_typec *t);
static void sm5714_pd_disable(struct sm5714_typec *t);
/* Request build + send, defined below; the RX handler re-Requests inline when a
 * PPS source re-advertises its caps mid-contract. */
static int sm5714_pd_build_request(struct sm5714_typec *t,
				   union sm5714_pd_header *h,
				   union sm5714_pd_obj *rdo);
static int sm5714_pd_send(struct sm5714_typec *t,
			  const union sm5714_pd_header *h,
			  const union sm5714_pd_obj *objs, int n);

/* Apply a decoded role: VBUS first toward the fail-safe, then the data role. */
static void sm5714_typec_apply(struct sm5714_typec *t, enum usb_role role,
			       enum typec_orientation orientation)
{
	bool host = (role == USB_ROLE_HOST);
	int ret;

	if (role == t->role)
		return;

	/*
	 * VBUS is sourced ONLY for HOST.  Drive it before the data role so that
	 * on any transition away from HOST the 5 V boost is cut first (fail-safe
	 * against back-feeding a newly attached source).
	 */
	ret = sm5714_usb_vbus_set_host(host);

	/*
	 * A failed VBUS *cut* is safety-critical: the boost may still be sourcing
	 * 5 V.  Do not commit the new role, so the periodic resync re-enters here
	 * (role still != t->role) and retries the cut instead of early-outing on an
	 * unchanged role.  -ENODEV means the VBUS driver has not probed, so there is
	 * no boost to cut -- not a failure.  A failed *enable* sources no VBUS, so
	 * it does not block adopting the (non-host) role.
	 */
	if (ret && ret != -ENODEV && !host) {
		dev_err(&t->client->dev,
			"failed to cut OTG VBUS (%d); will retry\n", ret);
		return;
	}

	if (t->role_sw)
		usb_role_switch_set_role(t->role_sw, role);

	/*
	 * Report the new connection to the Type-C class (only if the port
	 * registered; the role switching above is independent of it).  The
	 * partner is unconditionally re-registered on every change -- mirroring
	 * the in-tree standalone port controllers (e.g. wusb3801) -- so a
	 * DEVICE<->HOST transition that skips the disconnected (NONE) state
	 * (a DETACH edge seen only by the periodic resync) cannot leave a stale
	 * partner of the wrong role attached.  We are SINK+DEVICE when a source
	 * is on the cable and SOURCE+HOST when a sink is, matching the device's
	 * own downstream driver.
	 */
	if (t->port) {
		if (t->partner) {
			typec_unregister_partner(t->partner);
			t->partner = NULL;
		}
		if (role == USB_ROLE_NONE) {
			typec_set_pwr_role(t->port, TYPEC_SINK);
			typec_set_data_role(t->port, TYPEC_DEVICE);
			typec_set_orientation(t->port, TYPEC_ORIENTATION_NONE);
		} else {
			struct typec_partner_desc desc = {};

			t->partner = typec_register_partner(t->port, &desc);
			if (IS_ERR(t->partner)) {
				dev_warn(&t->client->dev,
					 "failed to register partner (%ld)\n",
					 PTR_ERR(t->partner));
				t->partner = NULL;
			}
			typec_set_pwr_role(t->port,
					   host ? TYPEC_SOURCE : TYPEC_SINK);
			typec_set_data_role(t->port,
					    host ? TYPEC_HOST : TYPEC_DEVICE);
			typec_set_orientation(t->port, orientation);
		}
	}

	t->role = role;
	dev_info(&t->client->dev, "USB-C role -> %s\n", usb_role_string(role));

	/*
	 * Bring the PD protocol layer up at the sink-attach edge (when armed via
	 * the pd_caps trigger) so it is live the moment the source sends its
	 * initial Source_Capabilities -- a PD source only advertises caps early
	 * in attach, so enabling here (not seconds later) is what lets us catch
	 * them.  Tear PD down on any move away from sink.  One-shot: the arm is
	 * consumed so a later Hard-Reset re-attach does not re-loop.
	 */
	if (role == USB_ROLE_DEVICE) {
		if (t->pd_arm && !t->pd_enabled) {
			t->pd_arm = false;
			if (sm5714_pd_enable(t))
				dev_warn(&t->client->dev, "PD enable failed\n");
		}
	} else if (t->pd_enabled) {
		sm5714_pd_disable(t);
	}
}

/* Read CC_STATUS and map the cable to a USB role; fail safe to NONE on error. */
static void sm5714_typec_update(struct sm5714_typec *t)
{
	enum typec_orientation orientation;
	enum usb_role role;
	int status, cc;

	/*
	 * Trust CC_STATUS only when the controller reports a resolved attachment
	 * (STATUS1 ATTACH, bit3).  CC_STATUS can read a transient orientation while
	 * the DRP state machine toggles Rp/Rd with nothing attached, and this path
	 * also runs from the blind periodic resync -- so without this gate a
	 * spurious 0x02 could wrongly source host VBUS.  Not attached -> NONE.
	 */
	status = i2c_smbus_read_byte_data(t->client, SM5714_TYPEC_REG_STATUS1);
	if (status < 0) {
		dev_warn(&t->client->dev, "STATUS1 read failed (%d); role->none\n",
			 status);
		sm5714_typec_apply(t, USB_ROLE_NONE, TYPEC_ORIENTATION_NONE);
		return;
	}
	if (!(status & SM5714_TYPEC_STATUS1_ATTACH)) {
		sm5714_typec_apply(t, USB_ROLE_NONE, TYPEC_ORIENTATION_NONE);
		return;
	}

	cc = i2c_smbus_read_byte_data(t->client, SM5714_TYPEC_REG_CC_STATUS);
	if (cc < 0) {
		dev_warn(&t->client->dev, "CC_STATUS read failed (%d); role->none\n",
			 cc);
		sm5714_typec_apply(t, USB_ROLE_NONE, TYPEC_ORIENTATION_NONE);
		return;
	}

	/* CC_STATUS bit5 resolves the plug orientation (CC1 normal / CC2 reverse). */
	orientation = (cc & SM5714_TYPEC_CC_CABLE_FLIP) ?
		TYPEC_ORIENTATION_REVERSE : TYPEC_ORIENTATION_NORMAL;

	switch (cc & SM5714_TYPEC_CC_ATTACH_MASK) {
	case SM5714_TYPEC_CC_ATTACH_SNK:
		role = USB_ROLE_HOST;		/* a peripheral is attached */
		break;
	case SM5714_TYPEC_CC_ATTACH_SRC:
		role = USB_ROLE_DEVICE;		/* a charger / host PC is attached */
		break;
	default:				/* 0x03 audio accessory */
		role = USB_ROLE_NONE;
		break;
	}

	sm5714_typec_apply(t, role, orientation);
}

/* Decode and log a received Source_Capabilities PDO array (one line per PDO). */
static void sm5714_pd_dump_source_caps(struct sm5714_typec *t,
				       union sm5714_pd_obj *objs, int n)
{
	struct device *dev = &t->client->dev;
	int i;

	for (i = 0; i < n; i++) {
		union sm5714_pd_obj o = objs[i];

		switch (o.supply.supply_type) {
		case SM5714_PD_SUPPLY_FIXED:
			dev_info(dev, "  PDO[%d] FIXED %u mV %u mA\n", i + 1,
				 o.fixed.voltage * 50, o.fixed.max_current * 10);
			break;
		case SM5714_PD_SUPPLY_APDO:
			if (o.apdo.pps_supply == 0)
				dev_info(dev,
					 "  PDO[%d] PPS %u-%u mV %u mA (pos %d)\n",
					 i + 1, o.apdo.min_voltage * 100,
					 o.apdo.max_voltage * 100,
					 o.apdo.max_current * 50, i + 1);
			else
				dev_info(dev, "  PDO[%d] APDO non-PPS raw=0x%08x\n",
					 i + 1, o.object);
			break;
		default:
			dev_info(dev, "  PDO[%d] type=%u raw=0x%08x\n", i + 1,
				 o.supply.supply_type, o.object);
			break;
		}
	}
}

/*
 * Read one received PD message out of the RX FIFO and (if it is a
 * Source_Capabilities) dump the advertised PDOs.  Runs in the IRQ thread; it
 * does not block, so there is no risk of waiting for an interrupt while holding
 * one off.  The final RX_BUF write is mandatory -- without it the controller
 * will not release the buffer for the next message.
 */
static void sm5714_pd_rx(struct sm5714_typec *t)
{
	union sm5714_pd_header hdr = { };
	union sm5714_pd_obj objs[SM5714_PD_MAX_OBJ] = { };
	struct device *dev = &t->client->dev;
	int ret, n, src;

	ret = i2c_smbus_read_i2c_block_data(t->client,
					    SM5714_TYPEC_REG_RX_HEADER_00, 2,
					    hdr.byte);
	if (ret < 0) {
		dev_warn(dev, "PD RX header read failed (%d)\n", ret);
		return;
	}

	n = hdr.num_data_objs;
	if (n > SM5714_PD_MAX_OBJ)
		n = SM5714_PD_MAX_OBJ;
	if (n > 0) {
		ret = i2c_smbus_read_i2c_block_data(t->client,
						    SM5714_TYPEC_REG_RX_PAYLOAD,
						    n * 4, (u8 *)objs);
		if (ret < 0) {
			dev_warn(dev, "PD RX payload read failed (%d)\n", ret);
			n = 0;
		}
	}

	src = i2c_smbus_read_byte_data(t->client, SM5714_TYPEC_REG_RX_SRC);

	/* Mandatory: acknowledge the read so the HW frees the RX buffer. */
	i2c_smbus_write_byte_data(t->client, SM5714_TYPEC_REG_RX_BUF,
				  SM5714_TYPEC_RX_BUF_READ_DONE);

	dev_info(dev, "PD RX: msg_type=%u objs=%u rev=%u src=0x%02x (hdr=0x%04x)\n",
		 hdr.msg_type, hdr.num_data_objs, hdr.spec_revision,
		 src < 0 ? 0xff : src, hdr.word);

	if (hdr.num_data_objs > 0) {
		if (hdr.msg_type == SM5714_PD_DATA_SOURCE_CAP) {
			sm5714_pd_dump_source_caps(t, objs, n);
			if (t->pd_do_request) {
				/*
				 * Negotiate: hand the caps to the work item,
				 * which sends a Request and blocks for the reply
				 * -- which cannot happen here, as this IRQ is
				 * ONESHOT-masked.  PD stays up; pd_negotiating
				 * stops a re-sent caps re-scheduling the work.
				 */
				if (!t->pd_negotiating) {
					t->pd_negotiating = true;
					memcpy(t->pd_pdo, objs,
					       n * sizeof(objs[0]));
					t->pd_npdo = n;
					schedule_work(&t->pd_req_work);
				} else {
					/*
					 * Mid-contract re-advertisement: a PPS
					 * source periodically re-offers its caps (a
					 * source-initiated AMS) and Hard-Resets if
					 * the sink does not Request again within
					 * ~tSenderResponse.  Re-Request inline -- we
					 * must answer now, and sm5714_pd_send is
					 * non-blocking (FIFO write + TX_REQ; the
					 * reply lands on a later IRQ), so it is safe
					 * in this handler (unlike the blocking
					 * Accept/PS_RDY waits, which stay in the work
					 * item).  No SinkTxOk gate: this is the
					 * source's AMS, not a sink-initiated one.
					 * Refresh the stored PDOs first so the
					 * keepalive stays consistent.
					 */
					union sm5714_pd_header rh = { };
					union sm5714_pd_obj rrdo = { };

					memcpy(t->pd_pdo, objs,
					       n * sizeof(objs[0]));
					t->pd_npdo = n;
					if (sm5714_pd_build_request(t, &rh,
								    &rrdo) > 0 &&
					    !sm5714_pd_send(t, &rh, &rrdo, 1))
						dev_info(dev,
							 "PD source re-advertised caps -> re-Request (renegotiate)\n");
				}
			} else {
				/*
				 * Receive-only bring-up: caps captured, drop PD
				 * before the source's no-Request timer
				 * (tSenderResponse ~27 ms) fires a Hard Reset
				 * that would cycle VBUS.
				 */
				sm5714_pd_disable(t);
			}
		}
		return;
	}

	/*
	 * Control message (no data objects): record it for the negotiation work
	 * item and wake it.  pd_evt is sticky so an event that arrives before
	 * the work starts waiting is not lost; the completion only cuts latency.
	 */
	switch (hdr.msg_type) {
	case SM5714_PD_CTRL_ACCEPT:
		t->pd_evt |= SM5714_PD_EVT_ACCEPT;
		break;
	case SM5714_PD_CTRL_REJECT:
		t->pd_evt |= SM5714_PD_EVT_REJECT;
		break;
	case SM5714_PD_CTRL_WAIT:
		t->pd_evt |= SM5714_PD_EVT_WAIT;
		break;
	case SM5714_PD_CTRL_PS_RDY:
		t->pd_evt |= SM5714_PD_EVT_PS_RDY;
		break;
	case SM5714_PD_CTRL_SOFT_RESET: {
		/*
		 * Soft_Reset is a recoverable message-ID resync the source
		 * initiates on a protocol hiccup; ignoring it makes the source
		 * escalate to a Hard Reset that drops the contract (observed:
		 * msg_type=13 -> hard reset ~29 ms later -> charge-pump ENOTCONN).
		 * Respond exactly as the device's own PE_SNK_Soft_Reset
		 * (sm5714_policy.c sm5714_usbpd_policy_snk_soft_reset): reset the
		 * protocol layer -- RX-flush + PD_CNTL4 PRL reset, byte-identical
		 * to the downstream sm5714_protocol_layer_reset(), which re-zeroes
		 * the HW-stamped message-ID -- then Accept.  The source then
		 * re-advertises Source_Capabilities and the mid-contract
		 * re-advertise path above re-Requests, completing the AMS; the
		 * contract SURVIVES, so pd_contract_lost stays clear.  All
		 * non-blocking i2c writes -- safe in this ONESHOT IRQ thread (like
		 * the inline re-Request above) and lower-latency than a work item
		 * against the source's ~30 ms response window.
		 */
		union sm5714_pd_header ah = { };

		i2c_smbus_write_byte_data(t->client, SM5714_TYPEC_REG_RX_BUF_ST,
					  SM5714_TYPEC_RX_BUF_FLUSH);
		i2c_smbus_write_byte_data(t->client, SM5714_TYPEC_REG_PD_CNTL4,
					  SM5714_TYPEC_PD_CNTL4_PRL_RESET);

		ah.msg_type = SM5714_PD_CTRL_ACCEPT;
		ah.spec_revision = SM5714_PD_SPEC_REV_30;
		ah.port_data_role = SM5714_PD_DATA_ROLE_UFP;
		ah.port_power_role = SM5714_PD_POWER_ROLE_SINK;
		if (sm5714_pd_send(t, &ah, NULL, 0))
			dev_warn(dev,
				 "PD Soft_Reset: Accept send failed (source will Hard-Reset)\n");
		else
			dev_info(dev,
				 "PD Soft_Reset received -> protocol reset + Accept (hdr=0x%04x, contract preserved)\n",
				 ah.word);
		return;
	}
	default:
		return;
	}
	complete(&t->pd_reply);
}

/*
 * Bring up the PD protocol layer for a sink, mirroring the device's own attach
 * sequence: select UFP + sink roles, reset the protocol layer (flush the RX
 * buffer + PD_CNTL4 reset, as the downstream driver does at PE_SNK_Startup so
 * the PHY starts from a clean state), unmask the PD interrupts we handle, then
 * enable the protocol layer.  Caller holds the lock.  The HW does
 * CRC/GoodCRC/retry and stamps the outgoing message-ID, so from here software
 * only writes/reads the FIFOs.
 */
static int sm5714_pd_enable(struct sm5714_typec *t)
{
	struct i2c_client *c = t->client;
	int cntl2, ret;

	cntl2 = i2c_smbus_read_byte_data(c, SM5714_TYPEC_REG_PD_CNTL2);
	if (cntl2 < 0)
		return cntl2;
	cntl2 &= ~(SM5714_TYPEC_PD_CNTL2_DFP | SM5714_TYPEC_PD_CNTL2_SRC);
	ret = i2c_smbus_write_byte_data(c, SM5714_TYPEC_REG_PD_CNTL2, cntl2);
	if (ret)
		return ret;

	/* Protocol-layer reset: flush RX buffer, then reset the protocol layer. */
	i2c_smbus_write_byte_data(c, SM5714_TYPEC_REG_RX_BUF_ST,
				  SM5714_TYPEC_RX_BUF_FLUSH);
	i2c_smbus_write_byte_data(c, SM5714_TYPEC_REG_PD_CNTL4,
				  SM5714_TYPEC_PD_CNTL4_PRL_RESET);

	ret = i2c_smbus_write_byte_data(c, SM5714_TYPEC_REG_INT_MASK4,
					SM5714_TYPEC_INT_MASK4_PD);
	if (ret)
		return ret;

	ret = i2c_smbus_write_byte_data(c, SM5714_TYPEC_REG_PD_CNTL1,
					SM5714_TYPEC_PD_CNTL1_ENABLE);
	if (ret)
		return ret;

	t->pd_contract_lost = false;
	t->pd_enabled = true;
	/* Keep the charging worker's AFC handshake off the shared cable while PD
	 * owns it -- an AFC ping mid-contract perturbs VBUS and breaks PPS. */
	sm5714_usb_vbus_inhibit_afc(true);
	dev_info(&c->dev, "PD protocol layer enabled (sink)\n");
	return 0;
}

/* Tear the PD protocol layer back down (disable PD; the layer's INT4 events
 * stop mattering once pd_enabled is clear). */
static void sm5714_pd_disable(struct sm5714_typec *t)
{
	/*
	 * Stop the keepalive.  The non-sync cancel only drops a pending timer and
	 * never sleeps, so it is safe to call under t->lock (the detach/IRQ callers
	 * hold it); a keepalive already running re-checks pd_enabled below and
	 * self-exits.  The sleeping sync-cancel lives in remove() (no lock held).
	 */
	cancel_delayed_work(&t->pd_keepalive);
	i2c_smbus_write_byte_data(t->client, SM5714_TYPEC_REG_PD_CNTL1,
				  SM5714_TYPEC_PD_CNTL1_DISABLE);
	t->pd_enabled = false;
	t->pd_do_request = false;	/* one-shot: a fresh attach must re-arm */
	t->pd_negotiating = false;
	t->pd_contract_lost = false;
	/* PD gone: re-allow AFC as the high-voltage fallback (the worker re-AFCs). */
	sm5714_usb_vbus_inhibit_afc(false);
	dev_info(&t->client->dev, "PD protocol layer disabled\n");
}

/*
 * Queue one PD message into the TX FIFO and fire it: header (2 bytes) + any data
 * objects (n*4 bytes) + TX_REQ.  The PHY does CRC/GoodCRC/retry and stamps the
 * message-ID, so this is the whole send.  Caller holds the lock.
 */
static int sm5714_pd_send(struct sm5714_typec *t,
			  const union sm5714_pd_header *h,
			  const union sm5714_pd_obj *objs, int n)
{
	struct i2c_client *c = t->client;
	int ret;

	ret = i2c_smbus_write_i2c_block_data(c, SM5714_TYPEC_REG_TX_HEADER_00,
					     2, h->byte);
	if (ret)
		return ret;
	if (n > 0) {
		ret = i2c_smbus_write_i2c_block_data(c,
						     SM5714_TYPEC_REG_TX_PAYLOAD,
						     n * 4, (const u8 *)objs);
		if (ret)
			return ret;
	}
	return i2c_smbus_write_byte_data(c, SM5714_TYPEC_REG_TX_REQ,
					 SM5714_TYPEC_TX_REQ_SOP);
}

/*
 * Find a PPS APDO in the captured caps whose programmable range covers
 * target_mv; return its 1-based object position (PD Request object_position) or
 * 0 if none.  Caller holds the lock (reads pd_pdo/pd_npdo).
 */
static int sm5714_pd_pick_pps(struct sm5714_typec *t, unsigned int target_mv)
{
	int i;

	for (i = 0; i < t->pd_npdo; i++) {
		union sm5714_pd_obj o = t->pd_pdo[i];

		if (o.supply.supply_type == SM5714_PD_SUPPLY_APDO &&
		    o.apdo.pps_supply == 0 &&
		    target_mv >= o.apdo.min_voltage * 100u &&
		    target_mv <= o.apdo.max_voltage * 100u)
			return i + 1;
	}
	return 0;
}

/*
 * Wait until any bit in mask is set in pd_evt, or timeout.  pd_evt is read under
 * the lock; the wait is done WITHOUT the lock so the IRQ can take it to signal
 * (waiting under the lock would deadlock against the very handler that sets the
 * event).  The completion is just a wakeup; the sticky pd_evt is the truth, so a
 * lost/extra completion at most costs one non-blocking re-check.
 */
static u32 sm5714_pd_wait(struct sm5714_typec *t, u32 mask,
			  unsigned int timeout_ms)
{
	unsigned long left = msecs_to_jiffies(timeout_ms);
	u32 hit;

	for (;;) {
		mutex_lock(&t->lock);
		hit = t->pd_evt & mask;
		mutex_unlock(&t->lock);
		if (hit || !left)
			return hit;
		left = wait_for_completion_timeout(&t->pd_reply, left);
	}
}

/*
 * Build the PPS Request (header + RDO) for the 10 V target from the captured
 * source caps.  Returns the chosen APDO's 1-based object position, or 0 if no
 * PPS APDO covers the target.  Shared by the initial negotiation and the
 * keepalive so both emit a byte-identical Request.  Caller holds the lock.
 */
static int sm5714_pd_build_request(struct sm5714_typec *t,
				   union sm5714_pd_header *h,
				   union sm5714_pd_obj *rdo)
{
	unsigned int op_ma, pdo_max_ma;
	int pos = sm5714_pd_pick_pps(t, t->pd_target_mv);

	if (!pos)
		return 0;

	/* Never request above the chosen APDO's advertised max current. */
	pdo_max_ma = t->pd_pdo[pos - 1].apdo.max_current * 50;
	op_ma = t->pd_target_ma;
	if (op_ma > pdo_max_ma)
		op_ma = pdo_max_ma;

	/* PPS Request data object: output_voltage in 20 mV, op_current in 50 mA. */
	rdo->object = 0;
	rdo->rdo_pps.output_voltage = t->pd_target_mv / 20;
	rdo->rdo_pps.op_current = op_ma / 50;
	rdo->rdo_pps.object_position = pos;
	rdo->rdo_pps.no_usb_suspend = 1;
	rdo->rdo_pps.usb_comm_capable = 1;

	/* Request: one data object, PD rev 3.0 (PPS), UFP + sink roles. */
	h->word = 0;
	h->msg_type = SM5714_PD_DATA_REQUEST;
	h->num_data_objs = 1;
	h->spec_revision = SM5714_PD_SPEC_REV_30;
	h->port_data_role = SM5714_PD_DATA_ROLE_UFP;
	h->port_power_role = SM5714_PD_POWER_ROLE_SINK;

	return pos;
}

/*
 * The PD sink Policy Engine's INITIAL negotiation: build a PPS Request for the
 * chosen APDO, send it, wait for Accept then PS_RDY, and -- on success -- arm
 * the keepalive that sustains the contract.  Runs in its own work item, NOT the
 * IRQ thread: it blocks waiting for the source's replies, which the IRQ delivers
 * (an inline wait would mask the line that carries the reply).  Any failure here
 * means no contract was established, so PD is torn down; the source's collision
 * toggles and tPPSTimeout only start mattering once a contract is held, which
 * the keepalive then maintains.
 */
static void sm5714_pd_req_work(struct work_struct *work)
{
	struct sm5714_typec *t = container_of(work, struct sm5714_typec,
					      pd_req_work);
	struct device *dev = &t->client->dev;
	union sm5714_pd_header h = { };
	union sm5714_pd_obj rdo = { };
	int pos, ret;
	u32 evt;

	mutex_lock(&t->lock);
	if (!t->pd_enabled) {		/* attach went away before we ran */
		mutex_unlock(&t->lock);
		return;
	}

	pos = sm5714_pd_build_request(t, &h, &rdo);
	if (!pos) {
		dev_warn(dev, "PD: no PPS APDO covers %u mV; not requesting\n",
			 t->pd_target_mv);
		mutex_unlock(&t->lock);
		return;
	}

	/* Arm the reply path BEFORE sending: clear stale events, reinit the
	 * completion -- otherwise a fast Accept could be missed or a stale one
	 * could false-fire. */
	t->pd_evt = 0;
	reinit_completion(&t->pd_reply);

	ret = sm5714_pd_send(t, &h, &rdo, 1);
	mutex_unlock(&t->lock);

	if (ret) {
		dev_warn(dev, "PD Request send failed (%d)\n", ret);
		goto teardown;
	}
	/* Log the exact packed objects so the bitfield transcription is
	 * verifiable on-device (expect rdo=0x5303e83c hdr=0x1082 for the
	 * 10 V/3 A pos-5 Request), independent of whether the source accepts. */
	dev_info(dev,
		 "PD Request sent: PPS PDO[%d] %u mV / %u mA (rdo=0x%08x hdr=0x%04x)\n",
		 pos, t->pd_target_mv, rdo.rdo_pps.op_current * 50, rdo.object,
		 h.word);

	evt = sm5714_pd_wait(t, SM5714_PD_EVT_ACCEPT | SM5714_PD_EVT_REJECT |
				SM5714_PD_EVT_WAIT, SM5714_PD_T_ACCEPT_MS);
	if (!(evt & SM5714_PD_EVT_ACCEPT)) {
		dev_warn(dev, "PD Request not accepted (evt=0x%x)\n", evt);
		goto teardown;
	}
	dev_info(dev, "PD Request ACCEPTED -- waiting for PS_RDY\n");

	evt = sm5714_pd_wait(t, SM5714_PD_EVT_PS_RDY, SM5714_PD_T_PSRDY_MS);
	if (!(evt & SM5714_PD_EVT_PS_RDY)) {
		dev_warn(dev, "PD: Accept but no PS_RDY (evt=0x%x)\n", evt);
		goto teardown;
	}
	dev_info(dev,
		 "PD PPS contract ESTABLISHED: %u mV requested -- read VBUS on the SM5440 now\n",
		 t->pd_target_mv);

	/*
	 * Contract is up.  Instead of the one-shot hold-then-drop, arm the
	 * keepalive so the source holds the contract past tPPSTimeout.  Re-check
	 * pd_enabled under the lock first -- a detach could have torn PD down while
	 * we waited for the replies above.
	 */
	mutex_lock(&t->lock);
	if (t->pd_enabled) {
		t->pd_contract_lost = false;
		schedule_delayed_work(&t->pd_keepalive,
				      msecs_to_jiffies(SM5714_PD_KEEPALIVE_MS));
		dev_info(dev, "PD keepalive armed (%u ms) -- sustaining contract\n",
			 SM5714_PD_KEEPALIVE_MS);
	}
	mutex_unlock(&t->lock);
	return;

teardown:
	mutex_lock(&t->lock);
	if (t->pd_enabled)
		sm5714_pd_disable(t);
	mutex_unlock(&t->lock);
}

/*
 * Keepalive: once the contract is established, periodically re-Request the same
 * PPS voltage so the source does not Hard-Reset it on tPPSTimeout.  Self-
 * rescheduling delayed work (the device's own sm_dc loop uses the same shape).
 * Each keepalive completes a full AMS: send the re-Request, then WAIT for the
 * source's PS_RDY before returning.  This serializes the contract's Requests.
 * The engine steps the PPS voltage via sm5714_pd_request_voltage(), which kicks
 * this same work item immediately (mod_delayed_work .. 0); without waiting for
 * PS_RDY that stepped re-Request can fire mid-AMS -- a fresh Request arriving
 * before the source's prior PS_RDY, i.e. two overlapping Requests in one
 * contract, which a spec-correct source answers with a Hard Reset (observed:
 * a 9982 mV step fired 16 ms after a 9000 mV Accept, ~53 ms before its PS_RDY,
 * -> HRST).  Genuine loss is detected out-of-band -- the IRQ sets
 * pd_contract_lost on HRST or TX_SOP_ERR -- and checked here; VBUS on the SM5440
 * is the electrical health signal.
 */
static void sm5714_pd_keepalive_work(struct work_struct *work)
{
	struct sm5714_typec *t = container_of(work, struct sm5714_typec,
					      pd_keepalive.work);
	struct device *dev = &t->client->dev;
	union sm5714_pd_header h = { };
	union sm5714_pd_obj rdo = { };
	int pos, cc, ret;
	u32 evt;

	mutex_lock(&t->lock);

	if (!t->pd_enabled) {		/* detached: nothing to sustain or tear down */
		mutex_unlock(&t->lock);
		return;
	}
	if (t->pd_contract_lost) {	/* HRST / TX_SOP_ERR: the source dropped us */
		dev_info(dev,
			 "PD keepalive: contract lost -- tearing down (charger re-AFCs)\n");
		sm5714_pd_disable(t);
		mutex_unlock(&t->lock);
		return;
	}

	/*
	 * PD-3.0 collision avoidance: our re-Request is a sink-initiated AMS, so it
	 * may only start when the source advertises Rp-3.0A (SinkTxOk).  If it is
	 * SinkTxNG, defer and re-check shortly rather than colliding; the deferrals
	 * stay far inside tPPSTimeout.
	 */
	cc = i2c_smbus_read_byte_data(t->client, SM5714_TYPEC_REG_CC_STATUS);
	if (cc < 0 ||
	    (cc & SM5714_TYPEC_CC_ADV_CURR) != SM5714_TYPEC_CC_ADV_CURR_3A) {
		/* Diagnostic: silent when the source idles at SinkTxOk (no defer);
		 * if deferrals pile up it shows the Rp the source is advertising. */
		dev_info(dev, "PD keepalive: SinkTxNG (CC_STATUS=0x%02x) -- deferring\n",
			 cc);
		mutex_unlock(&t->lock);
		schedule_delayed_work(&t->pd_keepalive,
				      msecs_to_jiffies(SM5714_PD_SINKTX_RETRY_MS));
		return;
	}

	pos = sm5714_pd_build_request(t, &h, &rdo);
	if (!pos) {			/* caps should still be valid; reschedule */
		mutex_unlock(&t->lock);
		schedule_delayed_work(&t->pd_keepalive,
				      msecs_to_jiffies(SM5714_PD_KEEPALIVE_MS));
		return;
	}
	/*
	 * Arm for THIS Request's PS_RDY before sending.  The clear and the send
	 * are both under the lock so the IRQ -- which sets PS_RDY -- cannot
	 * interleave between them and leave us waiting on a stale event.
	 */
	t->pd_evt &= ~SM5714_PD_EVT_PS_RDY;
	ret = sm5714_pd_send(t, &h, &rdo, 1);
	mutex_unlock(&t->lock);

	if (ret) {
		dev_warn(dev, "PD keepalive re-Request failed (%d)\n", ret);
	} else {
		dev_info(dev, "PD keepalive re-Request: %u mV (rdo=0x%08x)\n",
			 t->pd_target_mv, rdo.object);
		/*
		 * Serialize the AMS: block until this Request's PS_RDY arrives so the
		 * next Request (this work re-queued by the engine's voltage step, or
		 * the next periodic tick) cannot collide with an in-flight AMS.  This
		 * work item is single-threaded, so a mod_delayed_work(.., 0) landing
		 * during the wait simply re-runs us afterwards -- the step is requested
		 * cleanly on the next pass.  Times out at tPSTransition (rare: a HRST
		 * during the wait leaves PS_RDY unset; proceed, the next tick tears
		 * down on pd_contract_lost).
		 */
		evt = sm5714_pd_wait(t, SM5714_PD_EVT_PS_RDY, SM5714_PD_T_PSRDY_MS);
		if (!(evt & SM5714_PD_EVT_PS_RDY))
			dev_warn(dev,
				 "PD keepalive: no PS_RDY within %d ms (evt=0x%x) -- proceeding\n",
				 SM5714_PD_T_PSRDY_MS, evt);
	}

	schedule_delayed_work(&t->pd_keepalive,
			      msecs_to_jiffies(SM5714_PD_KEEPALIVE_MS));
}

/*
 * Cross-driver hooks for the SM5440 charge-pump loop (declared in
 * sm5714-typec.h).
 *
 * sm5714_pd_request_voltage(): command the sustained PPS contract to a new
 * voltage / current ceiling.  The pump's CC/CV loop calls this to step the PPS
 * input as it regulates; the keepalive then re-Requests the new target.  We kick
 * the keepalive to fire immediately (mod_delayed_work .. 0) so the stepped
 * voltage is requested promptly rather than up to one keepalive period later.
 * Returns -ENODEV (no bound instance) or -ENOTCONN (no PPS contract held).
 */
int sm5714_pd_request_voltage(unsigned int mv, unsigned int ma)
{
	struct sm5714_typec *t = READ_ONCE(sm5714_typec_instance);
	bool active;

	if (!t)
		return -ENODEV;

	mutex_lock(&t->lock);
	active = t->pd_enabled && !t->pd_contract_lost;
	if (active) {
		t->pd_target_mv = mv;
		t->pd_target_ma = ma;
	}
	mutex_unlock(&t->lock);

	if (!active)
		return -ENOTCONN;

	mod_delayed_work(system_wq, &t->pd_keepalive, 0);
	return 0;
}
EXPORT_SYMBOL_GPL(sm5714_pd_request_voltage);

/* True while a sink PPS contract is held (PD enabled and not lost). */
bool sm5714_pd_contract_active(void)
{
	struct sm5714_typec *t = READ_ONCE(sm5714_typec_instance);
	bool active;

	if (!t)
		return false;

	mutex_lock(&t->lock);
	active = t->pd_enabled && !t->pd_contract_lost;
	mutex_unlock(&t->lock);
	return active;
}
EXPORT_SYMBOL_GPL(sm5714_pd_contract_active);

static irqreturn_t sm5714_typec_irq(int irq, void *data)
{
	struct sm5714_typec *t = data;
	u8 intr[5];
	int ret;

	mutex_lock(&t->lock);

	/*
	 * Read all five latched INT registers in one block.  They are read-to-
	 * clear and the IRQ is level-low / ONESHOT, so once INT4 (the PD layer)
	 * is unmasked, any unmasked-but-unread bit would hold the line asserted.
	 * Reading the whole bank each time -- as the device's own driver does --
	 * prevents that.  Fall back to clearing INT1 alone if the block read
	 * fails, so the role-switching path keeps working regardless.
	 */
	ret = i2c_smbus_read_i2c_block_data(t->client, SM5714_TYPEC_REG_INT1,
					    5, intr);
	if (ret < 0) {
		i2c_smbus_read_byte_data(t->client, SM5714_TYPEC_REG_INT1);
		intr[3] = 0;
	}

	/* INT1 attach/detach is handled by re-reading CC/STATUS. */
	sm5714_typec_update(t);

	/* INT4 is the PD protocol layer (only meaningful once PD is enabled). */
	if (t->pd_enabled) {
		if (intr[3] & SM5714_TYPEC_INT4_RX_DONE)
			sm5714_pd_rx(t);
		if (intr[3] & SM5714_TYPEC_INT4_TX_DONE)
			dev_info(&t->client->dev, "PD TX delivered (GoodCRC)\n");
		/*
		 * TX_SOP_ERR = transmitted but no partner GoodCRC after the HW
		 * retries -> the link is gone.  HRST = the source reset the
		 * contract.  Either means the contract is genuinely lost; flag it
		 * so the keepalive tears PD down -- as opposed to a same-voltage
		 * re-Request the source simply answers without a fresh PS_RDY,
		 * which is benign and must NOT trigger a teardown.  TX_DISCARD =
		 * our TX was pre-empted by an incoming message (a collision) ->
		 * the source is alive, so treat it as a transient and let the next
		 * keepalive retry.
		 */
		if (intr[3] & SM5714_TYPEC_INT4_TX_SOP_ERR) {
			dev_warn(&t->client->dev,
				 "PD TX failed (no GoodCRC) -- contract lost\n");
			t->pd_contract_lost = true;
		}
		if (intr[3] & SM5714_TYPEC_INT4_TX_DISCARD)
			dev_info(&t->client->dev,
				 "PD TX discarded (collision; transient)\n");
		if (intr[3] & SM5714_TYPEC_INT4_HRST_RCVED) {
			dev_info(&t->client->dev,
				 "PD hard reset received -- contract lost\n");
			t->pd_contract_lost = true;
		}
	}

	mutex_unlock(&t->lock);

	return IRQ_HANDLED;
}

/* Slow safety re-sync in case a transition's edge was ever missed. */
static void sm5714_typec_resync_work(struct work_struct *work)
{
	struct sm5714_typec *t = container_of(work, struct sm5714_typec,
					      resync.work);

	mutex_lock(&t->lock);
	sm5714_typec_update(t);
	mutex_unlock(&t->lock);

	schedule_delayed_work(&t->resync, msecs_to_jiffies(SM5714_TYPEC_POLL_MS));
}

/*
 * Debug/bring-up trigger (write-only sysfs): ARM PD for the next sink attach.
 * A PD source only advertises Source_Capabilities early in attach and a charger
 * that has already settled into a non-PD high-voltage mode (e.g. AFC) will not
 * answer PD on the existing attach -- so PD must be live at the attach edge.
 * Writing here arms a one-shot enable that fires when the cable next resolves to
 * sink; the operator then replugs the charger.  The captured caps are logged and
 * PD is dropped again (receive-only bring-up).
 */
static ssize_t pd_caps_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct sm5714_typec *t = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&t->lock);
	t->pd_arm = true;
	mutex_unlock(&t->lock);
	dev_info(dev,
		 "PD armed: will enable PD + capture Source_Capabilities on the next sink attach -- replug the charger\n");
	return len;
}
static DEVICE_ATTR_WO(pd_caps);

/*
 * Debug/bring-up trigger (write-only sysfs): ARM a full PPS negotiation for the
 * next sink attach.  Same attach-edge timing as pd_caps, but instead of dropping
 * PD after capturing the caps, the driver sends a PPS Request (~10 V), waits for
 * Accept + PS_RDY, and then SUSTAINS the contract via the keepalive (re-Request
 * loop) until the cable is unplugged -- the operator reads VBUS on the SM5440 to
 * confirm it holds ~10 V past tPPSTimeout.  One-shot arm; the operator replugs.
 */
static ssize_t pd_request_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct sm5714_typec *t = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&t->lock);
	t->pd_arm = true;
	t->pd_do_request = true;
	mutex_unlock(&t->lock);
	dev_info(dev,
		 "PD armed: will negotiate + sustain a %u mV PPS contract on the next sink attach -- replug the charger\n",
		 SM5714_PD_REQ_MV);
	return len;
}
static DEVICE_ATTR_WO(pd_request);

/*
 * Register a Type-C port so the connector shows up under /sys/class/typec
 * (data/power role, partner, orientation) and to provide the foundation for
 * DP-altmode.  The capability (DRP / DRD / try-sink) is read straight from the
 * "usb-c-connector" DT node via typec_get_fw_cap() -- the device's own
 * description is the source of truth.  No typec_operations are supplied: the
 * SM5714 runs autonomous DRP and this driver carries no USB-PD policy engine,
 * so a userspace-requested role swap cannot be honoured (the Type-C class
 * returns -EOPNOTSUPP); the port only reports the hardware-decided state.
 *
 * Non-fatal by design: the port is an addition to the already-working role
 * switching + VBUS, so a failure here must leave t->port NULL and carry on
 * rather than take the working driver down (the reporting paths guard on it).
 */
static void sm5714_typec_register_port(struct sm5714_typec *t,
				       struct fwnode_handle *connector)
{
	struct device *dev = &t->client->dev;
	struct typec_port *port;
	int ret;

	ret = typec_get_fw_cap(&t->cap, connector);
	if (ret) {
		dev_warn(dev, "no Type-C port: bad connector capabilities (%d)\n",
			 ret);
		return;
	}

	t->cap.revision = USB_TYPEC_REV_1_2;
	t->cap.orientation_aware = true;
	t->cap.driver_data = t;
	t->cap.ops = NULL;

	port = typec_register_port(dev, &t->cap);
	if (IS_ERR(port)) {
		dev_warn(dev,
			 "failed to register Type-C port (%ld); role switching still active\n",
			 PTR_ERR(port));
		return;
	}
	t->port = port;
}

static int sm5714_typec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *connector;
	struct sm5714_typec *t;
	int ret;

	t = devm_kzalloc(dev, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	t->client = client;
	t->role = USB_ROLE_NONE;
	t->pd_target_mv = SM5714_PD_REQ_MV;
	t->pd_target_ma = SM5714_PD_REQ_MA;
	mutex_init(&t->lock);
	INIT_DELAYED_WORK(&t->resync, sm5714_typec_resync_work);
	INIT_WORK(&t->pd_req_work, sm5714_pd_req_work);
	INIT_DELAYED_WORK(&t->pd_keepalive, sm5714_pd_keepalive_work);
	init_completion(&t->pd_reply);
	i2c_set_clientdata(client, t);

	/*
	 * dwc3 (dr_mode=otg, usb-role-switch) registers the role switch; we are
	 * the provider that drives it.  The link is the OF graph from our
	 * "usb-c-connector" child node to the dwc3 HS port; that same node also
	 * describes the Type-C port capability.  Fall back to a direct lookup
	 * from our own node (and no Type-C port) if no connector child exists.
	 */
	connector = device_get_named_child_node(dev, "connector");
	if (connector)
		t->role_sw = fwnode_usb_role_switch_get(connector);
	else
		t->role_sw = usb_role_switch_get(dev);
	if (IS_ERR(t->role_sw)) {
		ret = dev_err_probe(dev, PTR_ERR(t->role_sw),
				    "failed to get usb_role_switch\n");
		goto err_put_connector;
	}

	/* Register the Type-C port before the first update() reflects state. */
	if (connector)
		sm5714_typec_register_port(t, connector);

	/* Program autonomous DRP so the controller toggles Rp/Rd on its own. */
	ret = i2c_smbus_write_byte_data(client, SM5714_TYPEC_REG_CC_CNTL1,
					SM5714_TYPEC_CC_CNTL1_DRP);
	if (ret) {
		dev_err_probe(dev, ret, "failed to enable DRP\n");
		goto err_unregister_port;
	}

	/* Unmask only attach/detach, then clear any latched state. */
	i2c_smbus_write_byte_data(client, SM5714_TYPEC_REG_INT_MASK1,
				  SM5714_TYPEC_INT_MASK1_ATTDET);
	i2c_smbus_read_byte_data(client, SM5714_TYPEC_REG_INT1);

	/* Reflect the cable state present at boot before arming the IRQ. */
	mutex_lock(&t->lock);
	sm5714_typec_update(t);
	mutex_unlock(&t->lock);

	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sm5714_typec_irq,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						"sm5714-typec", t);
		if (ret) {
			dev_err_probe(dev, ret, "failed to request IRQ\n");
			goto err_unregister_port;
		}
	} else {
		dev_warn(dev, "no IRQ; relying on %d ms poll\n",
			 SM5714_TYPEC_POLL_MS);
	}

	/* Slow re-sync backstop regardless of IRQ. */
	schedule_delayed_work(&t->resync, msecs_to_jiffies(SM5714_TYPEC_POLL_MS));

	/* PD bring-up triggers (non-fatal additions): capture-only + negotiate. */
	if (device_create_file(dev, &dev_attr_pd_caps))
		dev_warn(dev, "could not create pd_caps sysfs attribute\n");
	if (device_create_file(dev, &dev_attr_pd_request))
		dev_warn(dev, "could not create pd_request sysfs attribute\n");

	dev_info(dev, "SM5714 Type-C role controller ready (role=%s)\n",
		 usb_role_string(t->role));
	/* Fully initialised: publish the instance for the cross-driver PPS hooks. */
	WRITE_ONCE(sm5714_typec_instance, t);
	fwnode_handle_put(connector);
	return 0;

err_unregister_port:
	if (t->partner)
		typec_unregister_partner(t->partner);
	if (t->port)
		typec_unregister_port(t->port);
	usb_role_switch_put(t->role_sw);
err_put_connector:
	fwnode_handle_put(connector);
	return ret;
}

static void sm5714_typec_remove(struct i2c_client *client)
{
	struct sm5714_typec *t = i2c_get_clientdata(client);

	/* Stop the cross-driver PPS hooks finding us before we tear down. */
	WRITE_ONCE(sm5714_typec_instance, NULL);
	device_remove_file(&client->dev, &dev_attr_pd_caps);
	device_remove_file(&client->dev, &dev_attr_pd_request);
	cancel_delayed_work_sync(&t->resync);
	cancel_work_sync(&t->pd_req_work);
	cancel_delayed_work_sync(&t->pd_keepalive);
	/* Leave the port disconnected + VBUS off (also unregisters the partner). */
	mutex_lock(&t->lock);
	sm5714_typec_apply(t, USB_ROLE_NONE, TYPEC_ORIENTATION_NONE);
	mutex_unlock(&t->lock);
	if (t->port)
		typec_unregister_port(t->port);
	usb_role_switch_put(t->role_sw);
}

static const struct of_device_id sm5714_typec_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-typec" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_typec_of_match);

static struct i2c_driver sm5714_typec_driver = {
	.driver = {
		.name = "sm5714-typec",
		.of_match_table = sm5714_typec_of_match,
	},
	.probe = sm5714_typec_probe,
	.remove = sm5714_typec_remove,
};
module_i2c_driver(sm5714_typec_driver);

MODULE_DESCRIPTION("SM5714 USB Type-C role (PDIC) driver");
MODULE_AUTHOR("ubuntu-tab project");
MODULE_LICENSE("GPL");
