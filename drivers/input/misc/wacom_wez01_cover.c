// SPDX-License-Identifier: GPL-2.0-only
/*
 * Book-cover open/close switch carried by the Wacom WEZ01 EMR digitiser.
 *
 * On the Samsung Galaxy Tab S9 Ultra (SM-X910) the book cover's state is not a
 * GPIO. There is a hall sensor on the board -- and it is real -- but the Book
 * Cover Keyboard Slim fitted to this tablet does not actuate it. The state that
 * actually tracks the cover arrives as ONE BIT inside a Wacom notification
 * packet on i2c: packet id 13 (NOTI) in the low nibble of byte 0, sub-id 10
 * (COVER_DETECT) in byte 1, and the state in bit 7 of byte 3.
 *
 * The driver began as only the cover switch and has grown two things the same
 * bus carries: pen coordinates, and the garage charging coil. It still does NOT
 * do firmware update, and must not -- see the query-descriptor note below.
 *
 * The garage sense pin is not here either. It is a plain GPIO that no i2c
 * packet carries, so gpio-keys reports it as SW_PEN_INSERTED from the device
 * tree, which is how upstream boards report a pen garage. That split is why
 * charging POLICY cannot live in this driver: the trigger it needs is on
 * another input device entirely. What lives here is the mechanism -- the
 * commands, the state read, and the one safety rule that must not depend on a
 * userspace process still being alive.
 *
 * PROTOCOL, transcribed from the device's own downstream driver
 * (drivers/input/wacom/{wacom_i2c.c,wacom_reg.h,wacom_dev.h}) and then
 * confirmed on this silicon from userspace before a line of this was written:
 *
 *   - reads are a plain 17-byte i2c_master_recv with NO register address.
 *     17 is the vendor's COM_COORD_NUM + 1; every packet class is that size.
 *   - the IC signals "a packet is waiting" by pulling its interrupt line LOW,
 *     and RELEASES IT WHEN THE PACKET IS READ. That is level semantics, and it
 *     is why the interrupt is requested IRQ_TYPE_LEVEL_LOW rather than on an
 *     edge. With an edge trigger, a second notification arriving while the line
 *     is already low produces no new edge, the line stays low forever, and the
 *     driver goes permanently deaf in a way that looks like dead hardware.
 *     The vendor requests IRQF_ONESHOT | IRQF_TRIGGER_LOW for the same reason.
 *   - the steady state is COM_SURVEY_GARAGE_ONLY (0x3b), which is what the
 *     vendor's EPEN_SURVEY_MODE_COVER_DETECTION_ONLY sends. Measured on this
 *     part against a full-scan control arm: 8 cover transitions in survey mode
 *     versus 10 in full scan over equal windows, every falling edge yielding
 *     exactly one cover packet in both. Survey mode loses nothing and is not
 *     the part's highest-power state, so there is no reason to hold full scan.
 *
 * POLARITY is not a guess. The vendor's cover handler feeds change_status == 1
 * to its FLIP_CLOSE notifier, and linux/input-event-codes.h defines SW_LID as
 * "set = lid shut". The bit therefore maps onto SW_LID with no inversion.
 *
 * WHY SW_LID AND NOT SW_MACHINE_COVER. Samsung reports this as SW_FLIP, which
 * its own header puts at 0x10 -- numerically identical to mainline's
 * SW_MACHINE_COVER. That match proves the ENCODING and nothing else. Nothing in
 * this system's userspace consumes 0x10: systemd's logind implements
 * HandleLidSwitch on SW_LID alone. A driver emitting SW_MACHINE_COVER would
 * show the bit flipping in evtest, pass every inspection, and never once
 * suspend the machine. Matching stock's BEHAVIOUR means diverging from its
 * literal event number.
 *
 * The digitiser's FWE line (firmware-write-enable) is deliberately NOT claimed
 * here. It is held low by a gpio-hog applied at TLMM probe, which is earlier
 * and more certain than any driver bind. The IC's firmware is the one thing on
 * this device that re-flashing a partition cannot recover, so its mode must not
 * become contingent on this driver probing successfully.
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sysfs.h>

/* Commands. Names and values are the vendor's, from wacom_reg.h. */
#define WACOM_CMD_SURVEY_EXIT		0x2d	/* COM_SURVEY_EXIT */
#define WACOM_CMD_SAMPLERATE_START	0x31	/* COM_SAMPLERATE_START */
#define WACOM_CMD_SURVEY_COVER_ONLY	0x3b	/* COM_SURVEY_GARAGE_ONLY */
#define WACOM_CMD_COVER_CHECK_STATUS	0x88	/* COM_KBDCOVER_CHECK_STATUS */

/*
 * GARAGE CHARGING. The slot that holds the pen is a charging coil, and the
 * digitiser drives it on command -- nothing happens on its own. Measured on
 * this part before any of this was written: a fresh re-dock sat in
 * BLE_C_OFF_KEEP_2 for 95 consecutive samples over 2m12s and never left the
 * OFF family. A docked pen on this system was simply never charged.
 *
 * The commands are the vendor's, from wacom_reg.h. Only four of its nine are
 * named here, and the omissions are the point:
 *
 *   0xE8 DISABLE and 0xE9 ENABLE are policy flags, not charge commands, and
 *        the vendor's own shipping path rewrites DISABLE into KEEP_OFF before
 *        it can reach the bus (wacom_i2c_sec.c:1319-1323).
 *   0xEA RESET is redundant with START here and untested on this part.
 *   0xEF FORCE_RESET resets the pen's DSP. 0xF3 FULL is documented "depend on
 *        fw". 0xFF COM_FLASH puts the IC into firmware-flash mode, and this
 *        digitiser has no reset GPIO and an always-on rail, so a mis-flash is
 *        not recoverable by re-flashing a partition.
 *
 * Those bytes are not defined in this file and no code path can construct
 * one. The attribute takes WORDS, never an index: the vendor's sysfs surface
 * takes an integer and range-checks it, which means a userspace typo of "7"
 * emits FORCE_RESET. There is no integer here to mistype.
 */
#define WACOM_CMD_BLE_START		0xeb	/* COM_BLE_C_START */
#define WACOM_CMD_BLE_KEEP_ON		0xec	/* COM_BLE_C_KEEP_ON */
#define WACOM_CMD_BLE_KEEP_OFF		0xed	/* COM_BLE_C_KEEP_OFF */
#define WACOM_CMD_BLE_MODE_RETURN	0xee	/* COM_BLE_C_MODE_RETURN */

/*
 * A charge bout is KEEP_OFF, then START, and it is one operation rather than
 * two writes a caller sequences itself.
 *
 * WHY THE PAIR. START's own vendor comment reads "make start patter[n] + 1m
 * charge": the pen is woken by a PATTERN driven onto the coil, which needs a
 * de-energised coil to start from. Every sequence captured from stock on this
 * hardware is the pair -- 05,ED then 03,EB, twice in one capture -- and stock
 * never sends a bare START. A bare START does work from an idle part, which is
 * why probing found one sufficient; it is exactly when the coil is already on
 * that it would quietly do nothing, and that is the state a re-arm hits.
 *
 * The 600 ms floor is stock's, measured on the wire at 602 ms between the two
 * writes. It lives here rather than in the caller so that no caller can skip
 * it.
 */
#define WACOM_CHARGE_FLOOR_MS		600

/*
 * How long a bout can still be running. The part expires a bare START on its
 * own, measured on this hardware at t+91 s -- charging at t+89, off at t+91 --
 * so beyond this the coil is off whatever the host believes. The margin is
 * deliberate and one-sided: erring long costs a redundant stop command, erring
 * short would skip a stop that was needed.
 */
#define WACOM_CHARGE_BOUT_MS		120000

/*
 * Packet classes carrying charge information. From the vendor's PACKET_ID,
 * NOTI_SUB_ID and REPLY_SUB_ID enums in wacom_dev.h.
 *
 * REPLY(14)/GARAGE_CHARGE(6) is the solicited answer to MODE_RETURN, and its
 * byte 2 low nibble is the charge state. NOTI(13)/OOK(4) is accepted as an
 * equivalent answer because the vendor accepts either (wacom_i2c_sec.c:1237).
 *
 * NOTI(13)/CHARGE_FINISHED(7) is the vendor's GCF_PACKET, "Garage Charging
 * Finished". It is UNSOLICITED, it arrives on i2c, and the vendor's driver
 * clears its charging state on it (wacom_i2c.c:1193-1198). It is decoded here
 * only to record that it happened: whether this part's firmware emits it at
 * all has never been observed on this system, and a policy may not be built on
 * a packet nobody has seen. Recording it is how that changes.
 */
#define WACOM_PACKET_ID_REPLY		14	/* REPLY_PACKET */
#define WACOM_REPLY_GARAGE_CHARGE	6	/* GARAGE_CHARGE_PACKET */
#define WACOM_NOTI_OOK			4	/* OOK_PACKET */
#define WACOM_NOTI_CHARGE_FINISHED	7	/* GCF_PACKET */
#define WACOM_OOK_FAIL			BIT(7)	/* in byte 4 */
#define WACOM_CHARGE_STATE_MASK		0x0f	/* in byte 2 */

/*
 * The charge-state enum, from the vendor's epen_ble_charge_state. The vendor
 * groups it into two verdicts and one error (wacom_i2c_sec.c:1254-1271); the
 * grouping is transcribed with it, because the individual state names are
 * ambiguous in a way the grouping is not -- AFTER_RESET is a CHARGING state
 * despite reading like a stopped one, and this part answers AFTER_RESET to a
 * START rather than the AFTER_START its name would suggest.
 */
enum wacom_charge_state {
	WACOM_BLE_C_OFF		= 0,
	WACOM_BLE_C_START,
	WACOM_BLE_C_TRANSIT,
	WACOM_BLE_C_RESET,
	WACOM_BLE_C_AFTER_START,
	WACOM_BLE_C_AFTER_RESET,
	WACOM_BLE_C_ON_KEEP_1,
	WACOM_BLE_C_OFF_KEEP_1,
	WACOM_BLE_C_ON_KEEP_2,
	WACOM_BLE_C_OFF_KEEP_2,
	WACOM_BLE_C_FULL,
};

/* Ring of what was actually SENT and what actually ARRIVED, newest last. */
#define WACOM_CHARGE_LOG_ENTRIES	16

/* Packet layout. From wacom_dev.h's PACKET_ID / NOTI_SUB_ID enums. */
#define WACOM_PACKET_LEN		17	/* COM_COORD_NUM + 1 */
/*
 * The low nibble of byte 0 is the packet id. The high nibble is NOT a scan
 * counter: in a COORD packet those bits are status -- 0x80 rdy (in range),
 * 0x40 eraser, 0x20 side button, 0x10 tip down (wacom_i2c.c:1398-1424 in the
 * device's own downstream driver). The scan sequence is the high nibble of
 * byte 2, and only in a NOTI packet (wacom_i2c.c:1595). Masking byte 0 with
 * 0x0f is still correct for extracting the id; only the explanation was wrong.
 */
#define WACOM_PACKET_ID_MASK		0x0f
#define WACOM_PACKET_ID_NOTI		13	/* NOTI_PACKET */
#define WACOM_NOTI_COVER_DETECT		10	/* COVER_DETECT_PACKET */
#define WACOM_COVER_CLOSED		BIT(7)	/* in byte 3 */

/*
 * Settle times. The vendor sleeps 300 ms after entering a survey mode and 50 ms
 * after a start/stop command; these are the delays the userspace sequence used
 * when it was proven on this part, so they are transcribed rather than tuned.
 */
#define WACOM_SURVEY_SETTLE_MS		300
#define WACOM_CMD_SETTLE_MS		50

/*
 * Query descriptor. The part volunteers sixteen bytes of geometry as bytes
 * 16..31 of a plain 32-byte read: no register address, and no command. The
 * vendor defines an opcode for it (COM_QUERY 0x2a) but marks it "not use" and
 * never sends it, so this asks for a longer read and nothing else.
 *
 * WHY THIS IS READ-ONLY AND MUST STAY THAT WAY. The vendor's driver reaches a
 * firmware flash from a FAILED query: wacom_i2c_query() leaves the version
 * array zeroed, and wacom_i2c.c:2425 then does
 *     if (img_version_of_ic[2] == 0 && img_version_of_ic[3] == 0) goto fw_update;
 * which jumps PAST the MPU/PROJ_ID wrong-image guard at :2429. None of that is
 * ported here and none of it may be: this device's digitiser has no reset GPIO
 * (the vendor's reset is a bare AVDD cycle) and its rail is regulator-always-on,
 * so a mis-flash is not recoverable by re-flashing a partition. A failed query
 * here logs and returns; it never writes anything, ever.
 *
 * Only max_x and max_y come from the descriptor. Pressure, tilt and height are
 * literal constants in the device's own device tree (wacom,max_pressure 0xfff,
 * wacom,max_tilt <0x3f 0x3f>, wacom,max_height 0xff), so there is no reason to
 * decode them from a read that can fail.
 */
#define WACOM_QUERY_LEN			32
#define WACOM_QUERY_POS			16	/* COM_QUERY_POS */
#define WACOM_QUERY_HEADER		0x0f
#define WACOM_START_SETTLE_MS		100

#define WACOM_READ_RETRIES		3
#define WACOM_READ_RETRY_MS		10
#define WACOM_BACKOFF_MS		50
#define WACOM_RESUME_TIMEOUT_MS		500

/*
 * How long to hold the system awake after reporting a cover change. Closing the
 * cover makes logind suspend; opening it must therefore both wake the machine
 * AND get the SW_LID=0 report in before anything decides to suspend again.
 * Without this the open report can lose a race with a re-suspend and the tablet
 * bounces straight back down with the switch still reading closed.
 */
#define WACOM_WAKE_HOLD_MS		2000

/*
 * Coordinate packet. Layout transcribed from wacom_i2c.c:1398-1424 in the
 * device's own downstream driver and checked against it field by field.
 *
 * The vendor's local names for two of the status bits describe neither what
 * they read nor what they drive, and must not be copied: what it calls "prox"
 * is bit 4 and it drives pen_pressed -> BTN_TOUCH, i.e. the TIP; what it calls
 * "stylus" is bit 5 and it drives side_pressed -> BTN_STYLUS, i.e. the SIDE
 * BUTTON. The names below say what the bits do.
 */
#define WACOM_PACKET_ID_COORD		1	/* COORD_PACKET */
#define WACOM_COORD_IN_RANGE		BIT(7)	/* the vendor's "rdy" */
#define WACOM_COORD_ERASER		BIT(6)
#define WACOM_COORD_SIDE_BUTTON		BIT(5)
#define WACOM_COORD_TIP_DOWN		BIT(4)
#define WACOM_COORD_PRESSURE_MASK	0x0f	/* high nibble, in byte 5 */

/*
 * Maxima that are literals in this device's own dtbo (overlay0.dts:10864):
 * wacom,max_pressure = <0xfff>, wacom,max_tilt = <0x3f 0x3f>,
 * wacom,max_height = <0xff>. Only max_x/max_y are absent from the device tree,
 * which is the whole reason the geometry query exists.
 */
#define WACOM_MAX_PRESSURE		0xfff
#define WACOM_MAX_TILT			0x3f
#define WACOM_MAX_HEIGHT		0xff

/*
 * Sensor resolution, in units per millimetre. NOT a guess and not derived from
 * the panel at runtime: the queried maxima divided by the display's active area
 * (2960x1848 over 14.6in => 314.6 x 196.4 mm) give 99.7 on BOTH axes, and
 * 99.7 x 25.4 = 2532, i.e. 2540 LPI -- the standard Wacom EMR resolution. Two
 * axes agreeing on a standard value is what makes this a transcription rather
 * than a fit.
 *
 * input_abs_set_res() is mandatory, not decorative: libinput rejects a tablet
 * whose axes carry no resolution, and there is no vendor value to copy.
 */
#define WACOM_RESOLUTION		100	/* units/mm */

struct wacom_cover {
	struct i2c_client *client;
	struct input_dev *input;
	/*
	 * A SECOND input device for the pen. The vendor reports the cover as
	 * SW_FLIP on the same node as the pen; we deliberately do not. logind
	 * watches SW_LID, and libinput mishandles a tablet that also carries a
	 * switch, so the cover keeps its own device and this one is added
	 * beside it. NULL when the geometry query did not answer.
	 */
	struct input_dev *pen;
	u16 max_x, max_y;
	bool pen_in_range;
	/*
	 * The scan-mode knob. false is SURVEY_COVER_ONLY, which is exactly what
	 * shipped before this existed, so an untouched knob leaves the driver
	 * behaviourally identical to the deployed one.
	 *
	 * It exists because the part reports only what its survey mode surveys
	 * for: in cover-only it emits NOTHING for the pen -- measured, not
	 * assumed, with a pen in range and the cover cycle as the positive
	 * control on the same interrupt line. Reaching the pen therefore means
	 * leaving cover-only, and that is precisely the change that risks the
	 * shipped cover suspend/wake path. A runtime knob lets both arms of
	 * that experiment share ONE binary instead of two flashes.
	 */
	bool full_scan;
	struct mutex mode_lock;
	/*
	 * Held un-completed only while the system is suspended. A wake
	 * interrupt can reach the threaded handler before the QUP i2c
	 * controller has resumed, and the transfer would simply fail. Since a
	 * lost report leaves SW_LID reading closed, that failure would put the
	 * machine straight back to sleep. complete_all() is required here, not
	 * complete(): it pins the completion so every later interrupt passes.
	 */
	struct completion resumed;

	/*
	 * Charge command serialisation. Deliberately NOT mode_lock: a charge
	 * bout holds its lock across the 600 ms floor, and mode_lock is held
	 * across a 300 ms survey settle, so sharing one would make each wait on
	 * the other for no reason. The i2c core already serialises the bus
	 * itself, so the two never corrupt each other -- this only keeps two
	 * concurrent bouts from interleaving their KEEP_OFF and START.
	 */
	struct mutex charge_lock;
	/*
	 * Whether a bout has been STARTED and not stopped. Deliberately not
	 * "the last byte written": the state read writes MODE_RETURN, so a
	 * mere read of this device's charge state would otherwise erase the
	 * record that a bout was running and silently defeat the suspend rule
	 * below. Only the commands that arm or disarm the coil touch it.
	 */
	bool charge_armed;
	/*
	 * Whether the running bout was LATCHED with KEEP_ON, which has no
	 * expiry of its own, and the boottime at which the bout was armed.
	 *
	 * WHY THE TIMESTAMP EXISTS. A bare START is expired by the part itself
	 * at about t+91 s, and nothing tells the host that it happened. Without
	 * a window, charge_armed stays set for the rest of uptime after the
	 * first top-up, and every later suspend -- on a machine that suspends
	 * constantly -- sends a pointless KEEP_OFF and writes a ring entry. The
	 * ring is sixteen deep, so within an hour the execution record would
	 * contain nothing but suspend noise, and the instrument built to show
	 * what an unattended policy actually did would have destroyed exactly
	 * that.
	 */
	bool charge_latched;
	u64 charge_started_ms;

	/*
	 * The execution record. Stock's policy layer and this hardware can
	 * disagree silently: a capture on stock showed a full command sequence
	 * logged by the policy that never reached the bus at all. A log of
	 * INTENT is not evidence about a coil. This ring is what was SENT, with
	 * its return code, plus every charging-finished notification that
	 * ARRIVED -- so an unattended policy can be audited against the wire
	 * rather than against its own opinion of itself.
	 *
	 * Timestamps are ktime_get_boottime(), which INCLUDES suspended time.
	 * The vendor uses local_clock(), which does not, and correlating one of
	 * those against wall time is wrong by the whole sleep -- on this device
	 * that has already meant an eighteen-hour error.
	 */
	spinlock_t log_lock;
	struct {
		u64 stamp_ms;
		u8 byte;
		s16 ret;
		bool received;
	} log[WACOM_CHARGE_LOG_ENTRIES];
	unsigned int log_head;
	unsigned int log_count;
};

static int wacom_cover_send(struct wacom_cover *wc, u8 cmd, unsigned int settle_ms)
{
	int ret;

	ret = i2c_master_send(wc->client, &cmd, 1);
	if (ret != 1) {
		dev_err(&wc->client->dev, "command 0x%02x failed: %d\n", cmd, ret);
		return ret < 0 ? ret : -EIO;
	}

	msleep(settle_ms);
	return 0;
}

static void wacom_charge_log(struct wacom_cover *wc, u8 byte, int ret, bool received)
{
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&wc->log_lock, flags);
	i = wc->log_head;
	wc->log[i].stamp_ms = ktime_to_ms(ktime_get_boottime());
	wc->log[i].byte = byte;
	wc->log[i].ret = ret;
	wc->log[i].received = received;
	wc->log_head = (i + 1) % WACOM_CHARGE_LOG_ENTRIES;
	if (wc->log_count < WACOM_CHARGE_LOG_ENTRIES)
		wc->log_count++;
	spin_unlock_irqrestore(&wc->log_lock, flags);
}

/* One charge command byte, recorded whether it succeeded or not. */
static int wacom_charge_send(struct wacom_cover *wc, u8 cmd)
{
	int ret;

	ret = i2c_master_send(wc->client, &cmd, 1);
	if (ret == 1)
		ret = 0;
	else if (ret >= 0)
		ret = -EIO;

	wacom_charge_log(wc, cmd, ret, false);
	if (ret)
		dev_err(&wc->client->dev, "charge command 0x%02x failed: %d\n",
			cmd, ret);
	return ret;
}

/*
 * Start one bounded charge bout, or stop one.
 *
 * THE STOP IS THE HARDWARE'S, and that is the whole reason this is the shipped
 * form. A bare START energises the coil and the part expires it on its own:
 * measured on this hardware at t+91 s, with a control arm that stayed latched
 * past t+146 s when KEEP_ON was sent, so the expiry is the part's behaviour and
 * not a timeout of ours. Nothing here has to remember to stop it, no timer has
 * to survive a suspend, and a policy daemon that dies mid-bout leaves a coil
 * that switches itself off.
 *
 * That matters more than it sounds. An energised coil costs 52 mA, measured;
 * this device draws about 6 mA asleep on the vendor's own software. A latch
 * that outlived its owner would cost roughly nine times the entire idle budget
 * of the machine, indefinitely.
 */
static int wacom_charge_start(struct wacom_cover *wc)
{
	int ret;

	if (!wait_for_completion_timeout(&wc->resumed,
					 msecs_to_jiffies(WACOM_RESUME_TIMEOUT_MS)))
		return -EAGAIN;

	mutex_lock(&wc->charge_lock);
	ret = wacom_charge_send(wc, WACOM_CMD_BLE_KEEP_OFF);
	if (!ret) {
		msleep(WACOM_CHARGE_FLOOR_MS);
		ret = wacom_charge_send(wc, WACOM_CMD_BLE_START);
		if (!ret) {
			wc->charge_armed = true;
			wc->charge_latched = false;
			wc->charge_started_ms = ktime_to_ms(ktime_get_boottime());
		}
	}
	mutex_unlock(&wc->charge_lock);

	return ret;
}

static int wacom_charge_stop(struct wacom_cover *wc)
{
	int ret;

	if (!wait_for_completion_timeout(&wc->resumed,
					 msecs_to_jiffies(WACOM_RESUME_TIMEOUT_MS)))
		return -EAGAIN;

	mutex_lock(&wc->charge_lock);
	ret = wacom_charge_send(wc, WACOM_CMD_BLE_KEEP_OFF);
	/*
	 * Cleared only on a write that reached the bus. A failed stop leaves
	 * the coil in an unknown state, so the flag stays set and the next
	 * suspend tries again.
	 */
	if (!ret) {
		wc->charge_armed = false;
		wc->charge_latched = false;
	}
	mutex_unlock(&wc->charge_lock);

	return ret;
}

/*
 * Hold a bout open past the part's own expiry.
 *
 * NOT USED BY ANY POLICY HERE, and exposed only so the one open question about
 * this hardware can be answered: the vendor's driver acts on a "garage charging
 * finished" notification that nothing on this system has ever seen, and a
 * latched bout is the state in which it would have to arrive. Until it is
 * observed, a latch has no stop that survives losing its owner, so nothing
 * arms one automatically -- and suspend forces it off regardless.
 */
static int wacom_charge_hold(struct wacom_cover *wc)
{
	int ret;

	if (!wait_for_completion_timeout(&wc->resumed,
					 msecs_to_jiffies(WACOM_RESUME_TIMEOUT_MS)))
		return -EAGAIN;

	mutex_lock(&wc->charge_lock);
	ret = wacom_charge_send(wc, WACOM_CMD_BLE_KEEP_ON);
	if (!ret) {
		wc->charge_armed = true;
		wc->charge_latched = true;
		wc->charge_started_ms = ktime_to_ms(ktime_get_boottime());
	}
	mutex_unlock(&wc->charge_lock);

	return ret;
}

/*
 * Bring the IC up in the survey mode this driver is in. The sequence is the one
 * that was validated from userspace on this part: leave whatever survey mode it
 * was in, start sampling, then select a mode. Shorter sequences may well work;
 * none has been demonstrated, so none is shipped.
 *
 * That last command is the mode in force rather than a literal, because resume
 * calls this as a fallback when a status request fails. A literal there would
 * leave the hardware in cover-only while the scan-mode attribute still read
 * full, and in cover-only the part reports nothing for the pen: the attribute
 * would disagree with the hardware and the pen would go quiet with nothing
 * logged and no error returned anywhere. The mode is re-selected unconditionally
 * rather than skipped when it is already correct, because whether the sample-rate
 * command above perturbs the survey mode has not been established.
 *
 * The flag is read without the mode lock. These are system-sleep pm ops --
 * DEFINE_SIMPLE_DEV_PM_OPS leaves every runtime slot NULL -- and a resume
 * callback completes in dpm_resume_end() before suspend_finish() thaws
 * processes, so no writer of the attribute can be running. Probe is the only
 * other caller and runs before the attribute exists.
 */
static int wacom_cover_configure(struct wacom_cover *wc)
{
	int ret;

	ret = wacom_cover_send(wc, WACOM_CMD_SURVEY_EXIT, WACOM_SURVEY_SETTLE_MS);
	if (ret)
		return ret;

	ret = wacom_cover_send(wc, WACOM_CMD_SAMPLERATE_START, WACOM_START_SETTLE_MS);
	if (ret)
		return ret;

	return wacom_cover_send(wc, wc->full_scan ? WACOM_CMD_SURVEY_EXIT
						  : WACOM_CMD_SURVEY_COVER_ONLY,
				WACOM_SURVEY_SETTLE_MS);
}

/*
 * Read the geometry descriptor and report it. Advisory only: the cover switch
 * works without it, so a failure is logged and probe continues. Called before
 * any command byte goes out, which is the closest this driver gets to the
 * post-reset state the vendor queries in.
 */
static void wacom_query_geometry(struct wacom_cover *wc)
{
	u8 buf[WACOM_QUERY_LEN];
	const u8 *q = buf + WACOM_QUERY_POS;
	int ret, i;

	for (i = 0; i < WACOM_READ_RETRIES; i++) {
		ret = i2c_master_recv(wc->client, buf, sizeof(buf));
		if (ret == sizeof(buf) && q[0] == WACOM_QUERY_HEADER)
			break;
		if (i + 1 < WACOM_READ_RETRIES)
			msleep(WACOM_READ_RETRY_MS);
	}

	if (i == WACOM_READ_RETRIES) {
		/*
		 * Deliberately not an error. Nothing downstream depends on the
		 * geometry yet, and this must never become a reason to do
		 * anything other than carry on.
		 */
		dev_info(&wc->client->dev,
			 "geometry query did not answer (ret %d); cover switch is unaffected\n",
			 ret);
		return;
	}

	wc->max_x = ((u16)q[1] << 8) | q[2];
	wc->max_y = ((u16)q[3] << 8) | q[4];

	dev_info(&wc->client->dev,
		 "geometry: max_x=%u max_y=%u fw=%02x%02x mpu=%02x proj=%02x\n",
		 wc->max_x, wc->max_y, q[7], q[8], q[9], q[15]);
}

/*
 * Register the pen. Skipped entirely, and without failing probe, when the
 * geometry query did not answer: the vendor's own coordinate validation drops
 * every packet once the maxima are zero, and libinput would reject the device
 * anyway. The cover switch -- the function that actually ships -- is unaffected
 * either way, and must never be taken down by the pen.
 */
static int wacom_pen_register(struct wacom_cover *wc)
{
	struct input_dev *pen;
	int ret;

	if (!wc->max_x || !wc->max_y) {
		dev_info(&wc->client->dev,
			 "no geometry, pen input not registered (cover unaffected)\n");
		return 0;
	}

	pen = devm_input_allocate_device(&wc->client->dev);
	if (!pen)
		return -ENOMEM;

	pen->name = "Wacom WEZ01 Pen";
	pen->id.bustype = BUS_I2C;

	input_set_capability(pen, EV_KEY, BTN_TOOL_PEN);
	input_set_capability(pen, EV_KEY, BTN_TOOL_RUBBER);
	input_set_capability(pen, EV_KEY, BTN_TOUCH);
	input_set_capability(pen, EV_KEY, BTN_STYLUS);

	/*
	 * ROTATED IN THE DRIVER, and deliberately not in userspace.
	 *
	 * The sensor's x spans the panel's SHORT side and its y the long one,
	 * so raw coordinates arrive 90 degrees off a landscape desktop. The
	 * obvious fix is a libinput calibration matrix of "0 1 0 -1 0 1" --
	 * and on this stack that matrix is BROKEN: KWin 6 / libinput-1.31.3
	 * drop the off-diagonal cross-term and collapse the rotated X axis to
	 * zero. The touchscreen hit this first and its driver now rotates
	 * in-kernel for exactly this reason (goodix_berlin_report_state():
	 * rx = y, ry = RAW_MAX_X - x); see 61-goodix-rotate.rules, which
	 * pointedly installs an IDENTITY matrix. This reproduces that
	 * transform so the compositor has to apply no rotation at all.
	 *
	 * Hence the swapped ranges: ABS_X spans the sensor's y and vice versa.
	 * Verified against a pen held at the panel's top-left corner, which
	 * read raw x=19387 (of 19589) y=280 (of 31376) and maps to (0.009,
	 * 0.010) -- top-left.
	 */
	input_set_abs_params(pen, ABS_X, 0, wc->max_y, 0, 0);
	input_set_abs_params(pen, ABS_Y, 0, wc->max_x, 0, 0);
	input_set_abs_params(pen, ABS_PRESSURE, 0, WACOM_MAX_PRESSURE, 0, 0);
	input_set_abs_params(pen, ABS_DISTANCE, 0, WACOM_MAX_HEIGHT, 0, 0);
	input_set_abs_params(pen, ABS_TILT_X, -WACOM_MAX_TILT, WACOM_MAX_TILT, 0, 0);
	input_set_abs_params(pen, ABS_TILT_Y, -WACOM_MAX_TILT, WACOM_MAX_TILT, 0, 0);

	input_abs_set_res(pen, ABS_X, WACOM_RESOLUTION);
	input_abs_set_res(pen, ABS_Y, WACOM_RESOLUTION);

	ret = input_register_device(pen);
	if (ret)
		return ret;

	wc->pen = pen;
	dev_info(&wc->client->dev,
		 "pen input registered: %ux%u at %u units/mm\n",
		 wc->max_x, wc->max_y, WACOM_RESOLUTION);
	return 0;
}

/*
 * Decode and report one coordinate packet. Never called for a cover
 * notification, and never touches the cover input device.
 */
static void wacom_pen_report(struct wacom_cover *wc, const u8 *buf)
{
	u16 x, y, pressure;

	if (!wc->pen)
		return;

	if (!(buf[0] & WACOM_COORD_IN_RANGE)) {
		/* Out of range: retract everything exactly once. */
		if (wc->pen_in_range) {
			input_report_abs(wc->pen, ABS_PRESSURE, 0);
			input_report_abs(wc->pen, ABS_DISTANCE, 0);
			input_report_key(wc->pen, BTN_TOUCH, 0);
			input_report_key(wc->pen, BTN_STYLUS, 0);
			input_report_key(wc->pen, BTN_TOOL_PEN, 0);
			input_report_key(wc->pen, BTN_TOOL_RUBBER, 0);
			input_sync(wc->pen);
			wc->pen_in_range = false;
		}
		return;
	}

	x = ((u16)buf[1] << 8) | buf[2];
	y = ((u16)buf[3] << 8) | buf[4];

	/*
	 * The vendor validates against the maxima and DROPS the packet rather
	 * than clamping (wacom_i2c.c:1414). Same here, and for the same reason:
	 * a coordinate outside the sensor is a corrupt read, and clamping would
	 * launder it into a plausible stroke at the edge of the screen.
	 */
	if (x > wc->max_x || y > wc->max_y) {
		dev_dbg(&wc->client->dev, "coord out of range x=%u y=%u\n", x, y);
		return;
	}

	pressure = ((u16)(buf[5] & WACOM_COORD_PRESSURE_MASK) << 8) | buf[6];

	input_report_key(wc->pen, (buf[0] & WACOM_COORD_ERASER) ?
			 BTN_TOOL_RUBBER : BTN_TOOL_PEN, 1);
	/* The 90-degree rotation described at wacom_pen_register(). */
	input_report_abs(wc->pen, ABS_X, y);
	input_report_abs(wc->pen, ABS_Y, wc->max_x - x);
	input_report_abs(wc->pen, ABS_PRESSURE, pressure);
	input_report_abs(wc->pen, ABS_DISTANCE, buf[7]);
	/* Tilt rides the same rotation as the coordinates it describes. */
	input_report_abs(wc->pen, ABS_TILT_X, (s8)buf[9]);
	input_report_abs(wc->pen, ABS_TILT_Y, -(s8)buf[8]);
	input_report_key(wc->pen, BTN_TOUCH,  !!(buf[0] & WACOM_COORD_TIP_DOWN));
	input_report_key(wc->pen, BTN_STYLUS, !!(buf[0] & WACOM_COORD_SIDE_BUTTON));
	input_sync(wc->pen);
	wc->pen_in_range = true;
}

static int wacom_cover_read(struct wacom_cover *wc, u8 *buf)
{
	int ret, i;

	for (i = 0; i < WACOM_READ_RETRIES; i++) {
		ret = i2c_master_recv(wc->client, buf, WACOM_PACKET_LEN);
		if (ret == WACOM_PACKET_LEN)
			return 0;
		if (i + 1 < WACOM_READ_RETRIES)
			msleep(WACOM_READ_RETRY_MS);
	}

	return ret < 0 ? ret : -EIO;
}

/*
 * Decode one packet. Returns true and sets *closed only for a cover
 * notification; anything else is logged at debug so that the question "do other
 * packet classes interleave once a pen is in range?" is answerable from dmesg
 * without rebuilding anything.
 */
static bool wacom_cover_decode(struct wacom_cover *wc, const u8 *buf, bool *closed)
{
	u8 id = buf[0] & WACOM_PACKET_ID_MASK;

	if (id != WACOM_PACKET_ID_NOTI || buf[1] != WACOM_NOTI_COVER_DETECT) {
		/*
		 * One notification class is recorded rather than merely logged.
		 * The vendor's driver treats NOTI sub 7 as "garage charging
		 * finished" and clears its charging state on it, which would be
		 * a stop condition arriving on i2c with no radio involved. It
		 * has never been seen on this system -- and it never could have
		 * been, because nothing on this system had ever started a charge
		 * for it to finish. Now that something does, this is the only
		 * place the answer can appear.
		 */
		if (id == WACOM_PACKET_ID_NOTI &&
		    buf[1] == WACOM_NOTI_CHARGE_FINISHED) {
			wacom_charge_log(wc, buf[1], 0, true);
			dev_info(&wc->client->dev,
				 "garage charging finished (%*ph)\n",
				 WACOM_PACKET_LEN, buf);
			return false;
		}

		dev_dbg(&wc->client->dev,
			"ignoring packet id %u sub %u (%*ph)\n",
			id, buf[1], WACOM_PACKET_LEN, buf);
		return false;
	}

	*closed = buf[3] & WACOM_COVER_CLOSED;
	return true;
}

static void wacom_cover_report(struct wacom_cover *wc, bool closed)
{
	dev_dbg(&wc->client->dev, "cover %s\n", closed ? "closed" : "open");

	input_report_switch(wc->input, SW_LID, closed);
	input_sync(wc->input);

	pm_wakeup_event(&wc->client->dev, WACOM_WAKE_HOLD_MS);
}

/* Ask the IC for the current state rather than waiting for it to change. */
static int wacom_cover_sync_state(struct wacom_cover *wc)
{
	u8 buf[WACOM_PACKET_LEN];
	bool closed;
	int ret, i;

	ret = wacom_cover_send(wc, WACOM_CMD_COVER_CHECK_STATUS, WACOM_CMD_SETTLE_MS);
	if (ret)
		return ret;

	/*
	 * The solicited reply is not guaranteed to be the very next packet, so
	 * read a few and take the first cover notification.
	 */
	for (i = 0; i < WACOM_READ_RETRIES; i++) {
		ret = wacom_cover_read(wc, buf);
		if (ret)
			return ret;
		if (wacom_cover_decode(wc, buf, &closed)) {
			wacom_cover_report(wc, closed);
			return 0;
		}
		msleep(WACOM_READ_RETRY_MS);
	}

	dev_warn(&wc->client->dev, "no cover packet in reply to a status request\n");
	return -ENODATA;
}

static irqreturn_t wacom_cover_irq(int irq, void *data)
{
	struct wacom_cover *wc = data;
	u8 buf[WACOM_PACKET_LEN];
	bool closed;

	/*
	 * Every early return below leaves the packet unread, which leaves the
	 * interrupt line asserted -- and because the line is level triggered we
	 * are simply called again. The short sleeps keep that retry from
	 * becoming a spin; no timer or workqueue is needed.
	 */
	if (!wait_for_completion_timeout(&wc->resumed,
					 msecs_to_jiffies(WACOM_RESUME_TIMEOUT_MS))) {
		dev_warn_ratelimited(&wc->client->dev,
				     "i2c not resumed yet, deferring cover read\n");
		msleep(WACOM_BACKOFF_MS);
		return IRQ_HANDLED;
	}

	if (wacom_cover_read(wc, buf)) {
		dev_warn_ratelimited(&wc->client->dev, "cover packet read failed\n");
		msleep(WACOM_BACKOFF_MS);
		return IRQ_HANDLED;
	}

	/*
	 * The cover branch is evaluated first and is untouched. The pen branch
	 * is strictly additive: it runs only for packets the cover decode has
	 * already declined, and it reports on a different input device, so it
	 * cannot alter the behaviour of the path that ships.
	 */
	if (wacom_cover_decode(wc, buf, &closed))
		wacom_cover_report(wc, closed);
	else if ((buf[0] & WACOM_PACKET_ID_MASK) == WACOM_PACKET_ID_COORD)
		wacom_pen_report(wc, buf);

	return IRQ_HANDLED;
}

/*
 * Switch scan mode. SURVEY_EXIT (0x2d) is the vendor's EPEN_SURVEY_MODE_NONE,
 * which its own log calls "normal mode" and which it selects whenever the
 * screen is on with the pen out; SURVEY_COVER_ONLY (0x3b) is what we ship.
 */
static int wacom_set_scan_mode(struct wacom_cover *wc, bool full)
{
	int ret;

	mutex_lock(&wc->mode_lock);
	ret = wacom_cover_send(wc, full ? WACOM_CMD_SURVEY_EXIT
					: WACOM_CMD_SURVEY_COVER_ONLY,
			       WACOM_SURVEY_SETTLE_MS);
	if (!ret)
		wc->full_scan = full;
	mutex_unlock(&wc->mode_lock);

	dev_info(&wc->client->dev, "scan mode -> %s (%d)\n",
		 full ? "full" : "cover-only", ret);
	return ret;
}

static ssize_t full_scan_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct wacom_cover *wc = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "%d\n", wc->full_scan ? 1 : 0);
}

static ssize_t full_scan_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct wacom_cover *wc = i2c_get_clientdata(to_i2c_client(dev));
	bool full;
	int ret;

	ret = kstrtobool(buf, &full);
	if (ret)
		return ret;

	ret = wacom_set_scan_mode(wc, full);
	return ret ? ret : count;
}
/*
 * Ask the IC what the coil is actually doing.
 *
 * This is a real hardware read, not a replay of what was last written, and the
 * distinction is the point: a command that was accepted by the bus and did
 * nothing at the coil is a failure mode this device has already produced. The
 * round trip is the vendor's own (wacom_i2c_sec.c:1206-1271) -- send
 * MODE_RETURN, read a packet, accept either REPLY/GARAGE_CHARGE or NOTI/OOK as
 * the answer, and take the charge state from byte 2's low nibble.
 *
 * The interrupt is disabled across it because the reply is SOLICITED and the
 * threaded handler would otherwise consume it, leaving this read to time out
 * against a packet that arrived and was thrown away. The vendor disables its
 * interrupt here for the same reason.
 *
 * One transcription is deliberately not literal: the vendor rejects a reply on
 * `buff[4] & 80` -- decimal 80, i.e. 0x50, where every other use of that byte
 * in its own tree tests 0x80 for an OOK failure. That is a missing 0x, so the
 * bit it means is used rather than the bit it wrote.
 */
static int wacom_charge_state_read(struct wacom_cover *wc)
{
	u8 buf[WACOM_PACKET_LEN];
	bool covered;
	int ret, i;
	u8 id;

	if (!wait_for_completion_timeout(&wc->resumed,
					 msecs_to_jiffies(WACOM_RESUME_TIMEOUT_MS)))
		return -EAGAIN;

	mutex_lock(&wc->charge_lock);
	disable_irq(wc->client->irq);

	ret = wacom_charge_send(wc, WACOM_CMD_BLE_MODE_RETURN);
	if (ret)
		goto out;

	for (i = 0; i < WACOM_READ_RETRIES; i++) {
		ret = wacom_cover_read(wc, buf);
		if (ret)
			continue;

		id = buf[0] & WACOM_PACKET_ID_MASK;
		if (!((id == WACOM_PACKET_ID_REPLY &&
		       buf[1] == WACOM_REPLY_GARAGE_CHARGE) ||
		      (id == WACOM_PACKET_ID_NOTI &&
		       buf[1] == WACOM_NOTI_OOK))) {
			/*
			 * NOT the reply we asked for -- so it is a packet the
			 * interrupt handler would have processed, and this read
			 * has just taken it with the interrupt disabled.
			 *
			 * DISCARDING IT IS NOT HARMLESS. The IC releases its
			 * interrupt line when a packet is READ, so a cover
			 * notification consumed here and thrown away produces no
			 * further edge: SW_LID stays stale, and this driver
			 * already documents what that costs -- logind acts on a
			 * stale "closed" and puts the machine straight back to
			 * sleep. The policy reads this attribute after every
			 * top-up, so the window is routine rather than exotic.
			 *
			 * Hand it to the same decode the handler would have run.
			 */
			if (wacom_cover_decode(wc, buf, &covered))
				wacom_cover_report(wc, covered);
			msleep(WACOM_READ_RETRY_MS);
			continue;
		}

		if (buf[4] & WACOM_OOK_FAIL) {
			dev_warn(&wc->client->dev,
				 "charge state reply flags OOK failure (0x%02x)\n",
				 buf[4]);
			msleep(WACOM_READ_RETRY_MS);
			continue;
		}

		ret = buf[2] & WACOM_CHARGE_STATE_MASK;
		goto out;
	}

	ret = -ENODATA;
out:
	enable_irq(wc->client->irq);
	mutex_unlock(&wc->charge_lock);
	return ret;
}

/*
 * The vendor's own grouping of the state enum, not a per-state name. Its state
 * names mislead -- this part answers AFTER_RESET (5) to a START, and every
 * KEEP-suffixed value has both an ON and an OFF member -- so what is reported
 * is the verdict the vendor derives, which is unambiguous.
 */
static const char *wacom_charge_state_name(int state)
{
	switch (state) {
	case WACOM_BLE_C_AFTER_START:
	case WACOM_BLE_C_AFTER_RESET:
	case WACOM_BLE_C_ON_KEEP_1:
	case WACOM_BLE_C_ON_KEEP_2:
	case WACOM_BLE_C_FULL:
		return "charging";
	case WACOM_BLE_C_OFF:
	case WACOM_BLE_C_OFF_KEEP_1:
	case WACOM_BLE_C_OFF_KEEP_2:
		return "idle";
	default:
		return "transient";
	}
}

static ssize_t pen_charge_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct wacom_cover *wc = i2c_get_clientdata(to_i2c_client(dev));
	int state;

	state = wacom_charge_state_read(wc);
	if (state < 0)
		return state;

	return sysfs_emit(buf, "%s %d\n", wacom_charge_state_name(state), state);
}

/*
 * WORDS, NOT INDICES, and that is a safety property rather than a style
 * choice. The vendor's equivalent attribute takes an integer into a command
 * table whose entry 7 is FORCE_RESET and whose entry 8 is a firmware-dependent
 * full-charge command; a single mistyped digit from userspace reaches the pen's
 * DSP. No integer is parsed here, and the bytes those indices select are not
 * defined in this file at all.
 *
 * "top-up" is the only one a policy should ever need. "hold" and "off" exist
 * for the charging-finished question described above, and hold is never armed
 * by anything automatic.
 *
 * There is no docked-pen check here, and there cannot be: the garage sense pin
 * is claimed by gpio-keys, which reports it as SW_PEN_INSERTED on its own input
 * device, so this driver cannot read it. Energising an empty slot wastes
 * current and harms nothing, and the check belongs with the policy that can
 * actually see the switch.
 */
static ssize_t pen_charge_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct wacom_cover *wc = i2c_get_clientdata(to_i2c_client(dev));
	int ret;

	if (sysfs_streq(buf, "top-up"))
		ret = wacom_charge_start(wc);
	else if (sysfs_streq(buf, "hold"))
		ret = wacom_charge_hold(wc);
	else if (sysfs_streq(buf, "off"))
		ret = wacom_charge_stop(wc);
	else
		return -EINVAL;

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(pen_charge);

/* What reached the bus, and what came back. Newest last. */
static ssize_t pen_charge_log_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct wacom_cover *wc = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long flags;
	unsigned int i, n, idx;
	int len = 0;

	spin_lock_irqsave(&wc->log_lock, flags);
	n = wc->log_count;
	for (i = 0; i < n; i++) {
		idx = (wc->log_head + WACOM_CHARGE_LOG_ENTRIES - n + i) %
			WACOM_CHARGE_LOG_ENTRIES;
		len += sysfs_emit_at(buf, len, "%llu.%03llu %s %02x %d\n",
				     wc->log[idx].stamp_ms / 1000,
				     wc->log[idx].stamp_ms % 1000,
				     wc->log[idx].received ? "recv" : "sent",
				     wc->log[idx].byte, wc->log[idx].ret);
	}
	spin_unlock_irqrestore(&wc->log_lock, flags);

	return len;
}
static DEVICE_ATTR_RO(pen_charge_log);

static DEVICE_ATTR_RW(full_scan);

static struct attribute *wacom_cover_attrs[] = {
	&dev_attr_full_scan.attr,
	&dev_attr_pen_charge.attr,
	&dev_attr_pen_charge_log.attr,
	NULL,
};
ATTRIBUTE_GROUPS(wacom_cover);

static int wacom_cover_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct wacom_cover *wc;
	int ret;

	if (!client->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt specified\n");

	wc = devm_kzalloc(dev, sizeof(*wc), GFP_KERNEL);
	if (!wc)
		return -ENOMEM;

	wc->client = client;
	i2c_set_clientdata(client, wc);

	init_completion(&wc->resumed);
	complete_all(&wc->resumed);
	mutex_init(&wc->mode_lock);
	mutex_init(&wc->charge_lock);
	spin_lock_init(&wc->log_lock);

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable vdd\n");

	/* Before any command byte is sent, and before the interrupt is live. */
	wacom_query_geometry(wc);

	wc->input = devm_input_allocate_device(dev);
	if (!wc->input)
		return -ENOMEM;

	wc->input->name = "Wacom Book Cover Switch";
	wc->input->id.bustype = BUS_I2C;
	input_set_capability(wc->input, EV_SW, SW_LID);

	ret = input_register_device(wc->input);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register input device\n");

	/*
	 * Registered after the cover device and before the interrupt is live.
	 * A failure here is NOT fatal: the cover switch is the shipped
	 * function and must survive anything the pen does.
	 */
	ret = wacom_pen_register(wc);
	if (ret)
		dev_warn(dev, "pen input unavailable: %d\n", ret);

	ret = wacom_cover_configure(wc);
	if (ret)
		return dev_err_probe(dev, ret, "cannot configure the digitiser\n");

	/*
	 * Seed the switch before the interrupt is live, so that the first
	 * reader of EVIOCGSW -- logind -- sees the real state rather than a
	 * default-open switch that only becomes true on the next movement.
	 */
	ret = wacom_cover_sync_state(wc);
	if (ret)
		dev_warn(dev, "initial cover state unknown: %d\n", ret);

	/*
	 * Requested last, and requested as a level. If a notification is
	 * already pending the line is low right now, and a level-triggered
	 * interrupt fires the moment it is requested -- which drains it. An
	 * edge-triggered one would wait for a transition that has already
	 * happened.
	 */
	ret = devm_request_threaded_irq(dev, client->irq, NULL, wacom_cover_irq,
					IRQF_ONESHOT, "wacom-cover", wc);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request irq %d\n", client->irq);

	device_init_wakeup(dev, true);

	dev_info(dev, "wacom cover switch ready on irq %d\n", client->irq);
	return 0;
}

static int wacom_cover_suspend(struct device *dev)
{
	struct wacom_cover *wc = dev_get_drvdata(dev);
	int ret;

	/*
	 * NEVER CARRY AN ENERGISED COIL INTO SUSPEND. This is the only hard
	 * safety rule the charging path has, and it is enforced here rather
	 * than by the policy that started the bout, because the whole hazard is
	 * a policy that is no longer running.
	 *
	 * The part's own expiry cannot be relied on across a sleep: it is the
	 * digitiser's timer, on an always-on rail, and it keeps its state while
	 * the machine is frozen -- but a HELD bout has no expiry at all, and a
	 * held coil costs 52 mA against a device that should be drawing about
	 * 6 mA asleep. Every path out of here that suspends with the coil on is
	 * an order-of-magnitude battery regression that nothing would report.
	 *
	 * Sent unconditionally when the last command started or held a bout,
	 * before the gate closes, so it goes out while the bus is still up. The
	 * cost when a bout was legitimately running is the remainder of a 90 s
	 * top-up, which the next trigger repeats.
	 */
	if (wc->charge_armed &&
	    (wc->charge_latched ||
	     ktime_to_ms(ktime_get_boottime()) - wc->charge_started_ms <
			WACOM_CHARGE_BOUT_MS)) {
		dev_dbg(dev, "stopping the charge coil before suspend\n");
		wacom_charge_stop(wc);
	} else {
		/*
		 * Outside the window and not latched: the part has already
		 * expired the bout. Forget it rather than stopping it, so the
		 * next suspend does not do this again.
		 */
		wc->charge_armed = false;
	}

	/*
	 * Close the gate before the i2c controller goes down, so a wake
	 * interrupt waits for resume instead of failing a transfer.
	 */
	reinit_completion(&wc->resumed);

	if (device_may_wakeup(dev)) {
		ret = enable_irq_wake(wc->client->irq);
		if (ret)
			dev_warn(dev, "cannot arm irq %d as a wake source: %d\n",
				 wc->client->irq, ret);
		else
			dev_dbg(dev, "irq %d armed as a wake source\n",
				wc->client->irq);
	}

	return 0;
}

static int wacom_cover_resume(struct device *dev)
{
	struct wacom_cover *wc = dev_get_drvdata(dev);

	complete_all(&wc->resumed);

	if (device_may_wakeup(dev))
		disable_irq_wake(wc->client->irq);

	/*
	 * Re-read the state rather than trusting the wake packet to arrive.
	 * The cover can also move while the machine is frozen. Getting this
	 * wrong is not cosmetic: a stale "closed" is what logind acts on, so
	 * the machine would suspend again immediately. The IC keeps its mode
	 * across suspend because its supply is always-on, so only a status
	 * request is needed -- but if that fails, the mode is re-applied.
	 */
	if (wacom_cover_sync_state(wc)) {
		dev_warn(dev, "re-configuring the digitiser after resume\n");
		if (!wacom_cover_configure(wc))
			wacom_cover_sync_state(wc);
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(wacom_cover_pm_ops,
				wacom_cover_suspend, wacom_cover_resume);

static const struct of_device_id wacom_cover_of_match[] = {
	{ .compatible = "wacom,wez01" },
	{ }
};
MODULE_DEVICE_TABLE(of, wacom_cover_of_match);

static const struct i2c_device_id wacom_cover_id[] = {
	{ "wez01-cover" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wacom_cover_id);

static struct i2c_driver wacom_cover_driver = {
	.driver = {
		.name = "wacom-wez01-cover",
		.dev_groups = wacom_cover_groups,
		.of_match_table = wacom_cover_of_match,
		.pm = pm_sleep_ptr(&wacom_cover_pm_ops),
	},
	.probe = wacom_cover_probe,
	.id_table = wacom_cover_id,
};
module_i2c_driver(wacom_cover_driver);

MODULE_DESCRIPTION("Book-cover switch on the Wacom WEZ01 EMR digitiser");
MODULE_LICENSE("GPL");
