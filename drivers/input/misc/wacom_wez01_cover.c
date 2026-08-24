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
 * This driver is deliberately only the cover switch. It does not do pen
 * coordinates, pressure, tilt, the garage, or firmware update. Those need the
 * query handshake and the panel-data tables; the cover bit needs none of it.
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
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>

/* Commands. Names and values are the vendor's, from wacom_reg.h. */
#define WACOM_CMD_SURVEY_EXIT		0x2d	/* COM_SURVEY_EXIT */
#define WACOM_CMD_SAMPLERATE_START	0x31	/* COM_SAMPLERATE_START */
#define WACOM_CMD_SURVEY_COVER_ONLY	0x3b	/* COM_SURVEY_GARAGE_ONLY */
#define WACOM_CMD_COVER_CHECK_STATUS	0x88	/* COM_KBDCOVER_CHECK_STATUS */

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

/*
 * Put the IC into cover-detection-only. This is the exact sequence that was
 * validated from userspace on this part: leave whatever survey mode it was in,
 * start sampling, then drop to survey/cover-only. Shorter sequences may well
 * work; none has been demonstrated, so none is shipped.
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

	return wacom_cover_send(wc, WACOM_CMD_SURVEY_COVER_ONLY, WACOM_SURVEY_SETTLE_MS);
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
static DEVICE_ATTR_RW(full_scan);

static struct attribute *wacom_cover_attrs[] = {
	&dev_attr_full_scan.attr,
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
