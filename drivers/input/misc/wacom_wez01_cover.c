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
#define WACOM_PACKET_ID_MASK		0x0f	/* high nibble is a scan counter */
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

struct wacom_cover {
	struct i2c_client *client;
	struct input_dev *input;
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

	if (wacom_cover_decode(wc, buf, &closed))
		wacom_cover_report(wc, closed);

	return IRQ_HANDLED;
}

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

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable vdd\n");

	wc->input = devm_input_allocate_device(dev);
	if (!wc->input)
		return -ENOMEM;

	wc->input->name = "Wacom Book Cover Switch";
	wc->input->id.bustype = BUS_I2C;
	input_set_capability(wc->input, EV_SW, SW_LID);

	ret = input_register_device(wc->input);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register input device\n");

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
		.of_match_table = wacom_cover_of_match,
		.pm = pm_sleep_ptr(&wacom_cover_pm_ops),
	},
	.probe = wacom_cover_probe,
	.id_table = wacom_cover_id,
};
module_i2c_driver(wacom_cover_driver);

MODULE_DESCRIPTION("Book-cover switch on the Wacom WEZ01 EMR digitiser");
MODULE_LICENSE("GPL");
