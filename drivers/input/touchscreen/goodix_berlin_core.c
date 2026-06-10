// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Goodix "Berlin" Touchscreen IC driver
 * Copyright (C) 2020 - 2021 Goodix, Inc.
 * Copyright (C) 2023 Linaro Ltd.
 *
 * Based on goodix_ts_berlin driver.
 *
 * This driver is distinct from goodix.c since hardware interface
 * is different enough to require a new driver.
 * None of the register address or data structure are close enough
 * to the previous generations.
 *
 * Currently the driver only handles Multitouch events with already
 * programmed firmware and "config" for "Revision A/D" Berlin IC.
 *
 * Support is missing for:
 * - ESD Management
 * - Firmware update/flashing
 * - "Config" update/flashing
 * - Stylus Events
 * - Gesture Events
 * - Support for revision B
 */

#include <linux/bitfield.h>
#include <linux/export.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>

#include "goodix_berlin.h"

#define GOODIX_BERLIN_MAX_TOUCH			10

/*
 * GTS9U panel-native multitouch range. The gt6936_gts9u firmware reports
 * contacts in portrait panel units, 0..GOODIX_GTS9U_RAW_MAX_X by
 * 0..GOODIX_GTS9U_RAW_MAX_Y, matching the downstream sec_input max_coords of
 * 1848 x 2960 (inclusive maxima are one less). goodix_berlin_report_state()
 * rotates each contact into the landscape desktop frame before reporting, so
 * GOODIX_GTS9U_RAW_MAX_X is also the inversion pivot for the rotated Y axis.
 */
#define GOODIX_GTS9U_RAW_MAX_X			1847
#define GOODIX_GTS9U_RAW_MAX_Y			2959

#define GOODIX_BERLIN_NORMAL_RESET_DELAY_MS	100

#define GOODIX_BERLIN_TOUCH_EVENT		BIT(7)
#define GOODIX_BERLIN_REQUEST_EVENT		BIT(6)
#define GOODIX_BERLIN_TOUCH_COUNT_MASK		GENMASK(3, 0)

#define GOODIX_BERLIN_REQUEST_CODE_RESET	3

#define GOODIX_BERLIN_POINT_TYPE_MASK		GENMASK(3, 0)
#define GOODIX_BERLIN_POINT_TYPE_STYLUS_HOVER	1
#define GOODIX_BERLIN_POINT_TYPE_STYLUS		3

#define GOODIX_BERLIN_TOUCH_ID_MASK		GENMASK(7, 4)

#define GOODIX_BERLIN_DEV_CONFIRM_VAL		0xAA
#define GOODIX_BERLIN_BOOTOPTION_ADDR		0x10000

#define GOODIX_BERLIN_IC_INFO_MAX_LEN		SZ_1K

#define GOODIX_BERLIN_CHECKSUM_SIZE		sizeof(u16)

struct goodix_berlin_fw_version {
	u8 rom_pid[6];
	u8 rom_vid[3];
	u8 rom_vid_reserved;
	u8 patch_pid[8];
	u8 patch_vid[4];
	u8 patch_vid_reserved;
	u8 sensor_id;
	u8 reserved[2];
	__le16 checksum;
};

struct goodix_berlin_ic_info_version {
	u8 info_customer_id;
	u8 info_version_id;
	u8 ic_die_id;
	u8 ic_version_id;
	__le32 config_id;
	u8 config_version;
	u8 frame_data_customer_id;
	u8 frame_data_version_id;
	u8 touch_data_customer_id;
	u8 touch_data_version_id;
	u8 reserved[3];
} __packed;

struct goodix_berlin_ic_info_feature {
	__le16 freqhop_feature;
	__le16 calibration_feature;
	__le16 gesture_feature;
	__le16 side_touch_feature;
	__le16 stylus_feature;
} __packed;

struct goodix_berlin_ic_info_misc {
	__le32 cmd_addr;
	__le16 cmd_max_len;
	__le32 cmd_reply_addr;
	__le16 cmd_reply_len;
	__le32 fw_state_addr;
	__le16 fw_state_len;
	__le32 fw_buffer_addr;
	__le16 fw_buffer_max_len;
	__le32 frame_data_addr;
	__le16 frame_data_head_len;
	__le16 fw_attr_len;
	__le16 fw_log_len;
	u8 pack_max_num;
	u8 pack_compress_version;
	__le16 stylus_struct_len;
	__le16 mutual_struct_len;
	__le16 self_struct_len;
	__le16 noise_struct_len;
	__le32 touch_data_addr;
	__le16 touch_data_head_len;
	__le16 point_struct_len;
	__le16 reserved1;
	__le16 reserved2;
	__le32 mutual_rawdata_addr;
	__le32 mutual_diffdata_addr;
	__le32 mutual_refdata_addr;
	__le32 self_rawdata_addr;
	__le32 self_diffdata_addr;
	__le32 self_refdata_addr;
	__le32 iq_rawdata_addr;
	__le32 iq_refdata_addr;
	__le32 im_rawdata_addr;
	__le16 im_readata_len;
	__le32 noise_rawdata_addr;
	__le16 noise_rawdata_len;
	__le32 stylus_rawdata_addr;
	__le16 stylus_rawdata_len;
	__le32 noise_data_addr;
	__le32 esd_addr;
} __packed;

/*
 * GTS9U: the gt9916 "gt6936_gts9u" firmware uses a 16-byte event record, NOT
 * the 8-byte upstream {status, reserved, le16 x/y/w}. Confirmed by live raw
 * capture: with two fingers down, finger 1's record begins at report offset 24
 * (= header 8 + one 16-byte record), and the report checksum trails touch_num
 * 16-byte records. The first 8 bytes of each record carry the contact
 * (action/id/eid + nibble-packed 12-bit x/y + major/minor/z); the trailing 8
 * bytes carry aux data (noise/strength/hover/freq/pad) the coordinate path
 * ignores. Sizing this struct at 16 bytes makes GOODIX_BERLIN_TOUCH_SIZE -- and
 * thus the IRQ read length, the per-record stride, get_remaining_contacts() and
 * the checksum span -- all use the correct record size.
 */
struct goodix_berlin_touch {
	u8 status;	/* [action:7-6][tid:5-2][eid:1-0] */
	u8 x_hi;	/* X[11:4] */
	u8 y_hi;	/* Y[11:4] */
	u8 xy_lo;	/* [X[3:0]:7-4][Y[3:0]:3-0] */
	u8 major;
	u8 minor;
	u8 z_hi;	/* [touch_type hi:7-6][z:5-0] */
	u8 z_lo;
	u8 aux[8];	/* noise_level, max_strength, hover_id_num, freq_id, pad */
};
#define GOODIX_BERLIN_TOUCH_SIZE	sizeof(struct goodix_berlin_touch)

struct goodix_berlin_header {
	u8 status;
	u8 reserved1;
	u8 request_type;
	u8 reserved2[3];
	__le16 checksum;
};
#define GOODIX_BERLIN_HEADER_SIZE	sizeof(struct goodix_berlin_header)

struct goodix_berlin_event {
	struct goodix_berlin_header hdr;
	/* The data below is u16/__le16 aligned */
	u8 data[GOODIX_BERLIN_TOUCH_SIZE * GOODIX_BERLIN_MAX_TOUCH +
		GOODIX_BERLIN_CHECKSUM_SIZE];
};

struct goodix_berlin_core {
	struct device *dev;
	struct regmap *regmap;
	struct regulator *avdd;
	struct regulator *vddio;
	struct gpio_desc *reset_gpio;
	struct touchscreen_properties props;
	struct goodix_berlin_fw_version fw_version;
	struct input_dev *input_dev;
	int irq;

	/* Runtime parameters extracted from IC_INFO buffer  */
	u32 touch_data_addr;
	u16 touch_data_head_len;	/* GTS9U-DBG: chip-reported, currently UNUSED upstream */
	u16 point_struct_len;		/* GTS9U-DBG: chip-reported, currently UNUSED upstream */

	const struct goodix_berlin_ic_data *ic_data;

	struct goodix_berlin_event event;
};

static bool goodix_berlin_checksum_valid(const u8 *data, int size)
{
	u32 cal_checksum = 0;
	u16 r_checksum;
	int i;

	if (size < GOODIX_BERLIN_CHECKSUM_SIZE)
		return false;

	for (i = 0; i < size - GOODIX_BERLIN_CHECKSUM_SIZE; i++)
		cal_checksum += data[i];

	r_checksum = get_unaligned_le16(&data[i]);

	return (u16)cal_checksum == r_checksum;
}

static bool goodix_berlin_is_dummy_data(struct goodix_berlin_core *cd,
					const u8 *data, int size)
{
	int i;

	/*
	 * If the device is missing or doesn't respond the buffer
	 * could be filled with bus default line state, 0x00 or 0xff,
	 * so declare success the first time we encounter neither.
	 */
	for (i = 0; i < size; i++)
		if (data[i] > 0 && data[i] < 0xff)
			return false;

	return true;
}

static int goodix_berlin_dev_confirm(struct goodix_berlin_core *cd)
{
	u8 tx_buf[8], rx_buf[8];
	int retry = 3;
	int error;

	memset(tx_buf, GOODIX_BERLIN_DEV_CONFIRM_VAL, sizeof(tx_buf));
	while (retry--) {
		error = regmap_raw_write(cd->regmap,
					 GOODIX_BERLIN_BOOTOPTION_ADDR,
					 tx_buf, sizeof(tx_buf));
		if (error)
			return error;

		error = regmap_raw_read(cd->regmap,
					GOODIX_BERLIN_BOOTOPTION_ADDR,
					rx_buf, sizeof(rx_buf));
		if (error)
			return error;

		if (!memcmp(tx_buf, rx_buf, sizeof(tx_buf)))
			return 0;

		usleep_range(5000, 5100);
	}

	dev_err(cd->dev, "device confirm failed, rx_buf: %*ph\n",
		(int)sizeof(rx_buf), rx_buf);

	return -EINVAL;
}

static int goodix_berlin_power_on(struct goodix_berlin_core *cd)
{
	int error;

	error = regulator_enable(cd->vddio);
	if (error) {
		dev_err(cd->dev, "Failed to enable vddio: %d\n", error);
		return error;
	}

	/* Vendor waits 3ms for VDDIO to settle */
	usleep_range(3000, 3100);

	error = regulator_enable(cd->avdd);
	if (error) {
		dev_err(cd->dev, "Failed to enable avdd: %d\n", error);
		goto err_vddio_disable;
	}

	/* Vendor waits 15ms for AVDD to settle */
	usleep_range(15000, 15100);

	gpiod_set_value_cansleep(cd->reset_gpio, 0);

	/* Vendor waits 4ms for Firmware to initialize */
	usleep_range(4000, 4100);

	error = goodix_berlin_dev_confirm(cd);
	if (error)
		goto err_dev_reset;

	/* Vendor waits 100ms for Firmware to fully boot */
	msleep(GOODIX_BERLIN_NORMAL_RESET_DELAY_MS);

	return 0;

err_dev_reset:
	gpiod_set_value_cansleep(cd->reset_gpio, 1);
	regulator_disable(cd->avdd);
err_vddio_disable:
	regulator_disable(cd->vddio);
	return error;
}

static void goodix_berlin_power_off(struct goodix_berlin_core *cd)
{
	gpiod_set_value_cansleep(cd->reset_gpio, 1);
	regulator_disable(cd->avdd);
	regulator_disable(cd->vddio);
}

static int goodix_berlin_read_version(struct goodix_berlin_core *cd)
{
	int error;

	error = regmap_raw_read(cd->regmap, cd->ic_data->fw_version_info_addr,
				&cd->fw_version, sizeof(cd->fw_version));
	if (error) {
		dev_err(cd->dev, "error reading fw version, %d\n", error);
		return error;
	}

	if (!goodix_berlin_checksum_valid((u8 *)&cd->fw_version,
					  sizeof(cd->fw_version))) {
		dev_err(cd->dev, "invalid fw version: checksum error\n");
		return -EINVAL;
	}

	return 0;
}

/* Only extract necessary data for runtime */
static int goodix_berlin_parse_ic_info(struct goodix_berlin_core *cd,
				       const u8 *data, u16 length)
{
	struct goodix_berlin_ic_info_misc *misc;
	unsigned int offset = 0;

	offset += sizeof(__le16); /* length */
	offset += sizeof(struct goodix_berlin_ic_info_version);
	offset += sizeof(struct goodix_berlin_ic_info_feature);

	/* IC_INFO Parameters, variable width structure */
	offset += 4 * sizeof(u8); /* drv_num, sen_num, button_num, force_num */
	if (offset >= length)
		goto invalid_offset;

#define ADVANCE_LE16_PARAMS()				\
	do {						\
		u8 param_num = data[offset++];		\
		offset += param_num * sizeof(__le16);	\
		if (offset >= length)			\
			goto invalid_offset;		\
	} while (0)
	ADVANCE_LE16_PARAMS(); /* active_scan_rate_num */
	ADVANCE_LE16_PARAMS(); /* mutual_freq_num*/
	ADVANCE_LE16_PARAMS(); /* self_tx_freq_num */
	ADVANCE_LE16_PARAMS(); /* self_rx_freq_num */
	ADVANCE_LE16_PARAMS(); /* stylus_freq_num */
#undef ADVANCE_LE16_PARAMS

	misc = (struct goodix_berlin_ic_info_misc *)&data[offset];
	cd->touch_data_addr = le32_to_cpu(misc->touch_data_addr);
	cd->touch_data_head_len = le16_to_cpu(misc->touch_data_head_len);
	cd->point_struct_len = le16_to_cpu(misc->point_struct_len);

	/* GTS9U-DBG: info_version_id is the first byte after the 2-byte length */
	dev_info(cd->dev,
		 "GTS9U-DBG ic_info: info_ver=0x%02x touch_data_addr=%#x head_len=%u point_struct_len=%u\n",
		 data[sizeof(__le16)], cd->touch_data_addr,
		 cd->touch_data_head_len, cd->point_struct_len);

	return 0;

invalid_offset:
	dev_err(cd->dev, "ic_info length is invalid (offset %d length %d)\n",
		offset, length);
	return -EINVAL;
}

static int goodix_berlin_get_ic_info(struct goodix_berlin_core *cd)
{
	u8 *afe_data __free(kfree) = NULL;
	__le16 length_raw;
	u16 length;
	int error;

	afe_data = kzalloc(GOODIX_BERLIN_IC_INFO_MAX_LEN, GFP_KERNEL);
	if (!afe_data)
		return -ENOMEM;

	error = regmap_raw_read(cd->regmap, cd->ic_data->ic_info_addr,
				&length_raw, sizeof(length_raw));
	if (error) {
		dev_err(cd->dev, "failed get ic info length, %d\n", error);
		return error;
	}

	length = le16_to_cpu(length_raw);
	if (length >= GOODIX_BERLIN_IC_INFO_MAX_LEN) {
		dev_err(cd->dev, "invalid ic info length %d\n", length);
		return -EINVAL;
	}

	error = regmap_raw_read(cd->regmap, cd->ic_data->ic_info_addr, afe_data,
				length);
	if (error) {
		dev_err(cd->dev, "failed get ic info data, %d\n", error);
		return error;
	}

	/* check whether the data is valid (ex. bus default values) */
	if (goodix_berlin_is_dummy_data(cd, afe_data, length)) {
		dev_err(cd->dev, "fw info data invalid\n");
		return -EINVAL;
	}

	if (!goodix_berlin_checksum_valid(afe_data, length)) {
		dev_err(cd->dev, "fw info checksum error\n");
		return -EINVAL;
	}

	error = goodix_berlin_parse_ic_info(cd, afe_data, length);
	if (error)
		return error;

	/* check some key info */
	if (!cd->touch_data_addr) {
		dev_err(cd->dev, "touch_data_addr is null\n");
		return -EINVAL;
	}

	return 0;
}

static int goodix_berlin_get_remaining_contacts(struct goodix_berlin_core *cd,
						int n)
{
	size_t offset = 2 * GOODIX_BERLIN_TOUCH_SIZE +
				GOODIX_BERLIN_CHECKSUM_SIZE;
	u32 addr = cd->touch_data_addr + GOODIX_BERLIN_HEADER_SIZE + offset;
	int error;

	error = regmap_raw_read(cd->regmap, addr,
				&cd->event.data[offset],
				(n - 2) * GOODIX_BERLIN_TOUCH_SIZE);
	if (error) {
		dev_err_ratelimited(cd->dev, "failed to get touch data, %d\n",
				    error);
		return error;
	}

	return 0;
}

/*
 * GTS9U: this device's gt9916 ships Samsung "gt6936_gts9u" firmware whose event
 * records are 16 bytes each (GOODIX_BERLIN_TOUCH_SIZE) and do NOT use the
 * upstream {status, reserved, le16 x/y/w} layout. It uses the Samsung sec_input
 * packing (cf. downstream goodix_brl_hw.c goodix_parse_finger, BYTES_PER_POINT=16):
 *   byte0: [7:6]=action(1=press,2=move,3=release) [5:2]=finger id [1:0]=eid
 *   byte1: x[11:4]   byte2: y[11:4]   byte3: [7:4]=x[3:0] [3:0]=y[3:0]
 *   byte4: major     byte5: minor     byte6: [5:0]=pressure
 *   bytes 8-15: aux (noise/strength/hover/freq/pad) -- ignored here
 * eid (byte0 bits 1:0) selects record type: 0=coordinate, 1=status, 2=gesture,
 * 3=empty; only eid==0 carries a contact. Decoding the upstream 8-byte-stride
 * way reads record-0's aux tail as a phantom "finger 1" (garbage ~x=500,y=0) and
 * truncates the read before the real 2nd contact; the 16-byte stride + nibble-pack
 * below match the live raw captures (finger 1 begins at report offset 24).
 */
static void goodix_berlin_report_state(struct goodix_berlin_core *cd, int n)
{
	u8 *data = cd->event.data;
	int i;

	for (i = 0; i < n; i++) {
		u8 *p = data + i * GOODIX_BERLIN_TOUCH_SIZE;
		unsigned int eid = p[0] & 0x03;
		unsigned int action = (p[0] & 0xC0) >> 6;
		unsigned int id = (p[0] & 0x3C) >> 2;
		unsigned int x = ((unsigned int)p[1] << 4) | ((p[3] & 0xF0) >> 4);
		unsigned int y = ((unsigned int)p[2] << 4) | (p[3] & 0x0F);
		bool active = (action != 3);	/* 3 = release */

		/*
		 * eid (p[0] bits 1:0): 0 = coordinate, 1 = status, 2 = gesture,
		 * 3 = empty. Only coordinate records carry contact data; skip the
		 * rest so a status/gesture event is not reported as a phantom finger.
		 */
		if (eid != 0)
			continue;

		if (id >= GOODIX_BERLIN_MAX_TOUCH) {
			dev_warn_ratelimited(cd->dev, "invalid finger id %d\n", id);
			continue;
		}

		input_mt_slot(cd->input_dev, id);
		input_mt_report_slot_state(cd->input_dev, MT_TOOL_FINGER, active);

		if (active) {
			/*
			 * The panel is mounted portrait (raw 0..MAX_X by
			 * 0..MAX_Y) but the desktop runs landscape. Rotate the
			 * contact 270 degrees here so the compositor needs only
			 * an identity calibration matrix: rx = y, ry = MAX_X - x.
			 * This reproduces the proven-correct "0 1 0 -1 0 1"
			 * libinput matrix in-driver, sidestepping the KWin 6
			 * off-diagonal-matrix mapping bug that collapses a
			 * rotated touch X to zero.
			 */
			unsigned int rx = y;
			unsigned int ry = GOODIX_GTS9U_RAW_MAX_X - x;

			touchscreen_report_pos(cd->input_dev, &cd->props,
					       rx, ry, true);
			input_report_abs(cd->input_dev, ABS_MT_TOUCH_MAJOR, p[4]);
		}
	}

	input_mt_sync_frame(cd->input_dev);
	input_sync(cd->input_dev);
}

static void goodix_berlin_touch_handler(struct goodix_berlin_core *cd)
{
	u8 touch_num;
	int error;

	touch_num = FIELD_GET(GOODIX_BERLIN_TOUCH_COUNT_MASK,
			      cd->event.hdr.request_type);
	if (touch_num > GOODIX_BERLIN_MAX_TOUCH) {
		dev_warn(cd->dev, "invalid touch num %d\n", touch_num);
		return;
	}

	if (touch_num > 2) {
		/* read additional contact data if more than 2 touch events */
		error = goodix_berlin_get_remaining_contacts(cd, touch_num);
		if (error)
			return;
	}

	if (touch_num) {
		/*
		 * GTS9U: each event record is GOODIX_BERLIN_TOUCH_SIZE (16) bytes,
		 * so the report checksum trails touch_num records: span
		 * touch_num * 16 + CHECKSUM_SIZE. (The earlier 8-byte stride read
		 * record-0's 8-byte aux tail as a phantom "finger 1" and mistook
		 * the first 2 bytes of finger-1's real record for the checksum --
		 * both fixed by the 16-byte record size.) Kept non-fatal: a
		 * transient mismatch must not drop a frame, since each contact's
		 * coordinates decode independently of the checksum.
		 */
		int len = touch_num * GOODIX_BERLIN_TOUCH_SIZE +
			  GOODIX_BERLIN_CHECKSUM_SIZE;

		if (!goodix_berlin_checksum_valid(cd->event.data, len))
			dev_warn_ratelimited(cd->dev,
				"touch data checksum mismatch (non-fatal): %*ph\n",
				len, cd->event.data);
	}

	goodix_berlin_report_state(cd, touch_num);
}

static int goodix_berlin_request_handle_reset(struct goodix_berlin_core *cd)
{
	gpiod_set_value_cansleep(cd->reset_gpio, 1);
	usleep_range(2000, 2100);
	gpiod_set_value_cansleep(cd->reset_gpio, 0);

	msleep(GOODIX_BERLIN_NORMAL_RESET_DELAY_MS);

	return 0;
}

static irqreturn_t goodix_berlin_irq(int irq, void *data)
{
	struct goodix_berlin_core *cd = data;
	int error;

	/*
	 * First read: header + space for 2 event records + checksum:
	 * - GOODIX_BERLIN_HEADER_SIZE = 8 bytes
	 * - GOODIX_BERLIN_TOUCH_SIZE * 2 = 32 bytes (each record is 16 bytes:
	 *   an 8-byte contact followed by 8 bytes of aux data)
	 * - GOODIX_BERLIN_CHECKSUM_SIZE = 2 bytes
	 * For a total of 42 bytes.
	 *
	 * Layout when up to 2 events are reported:
	 * - bytes 0-7:   Header
	 * - bytes 8-23:  Event record 0 (contact bytes 0-7, aux bytes 8-15)
	 * - bytes 24-39: Event record 1
	 * - bytes 40-41: Checksum (immediately after touch_num records)
	 *
	 * If more than 2 events are reported, bytes 40-41 instead hold the
	 * first 2 bytes of record 2, and goodix_berlin_get_remaining_contacts()
	 * reads the rest contiguously, including the trailing checksum.
	 */
	error = regmap_raw_read(cd->regmap, cd->touch_data_addr,
				&cd->event,
				GOODIX_BERLIN_HEADER_SIZE +
					2 * GOODIX_BERLIN_TOUCH_SIZE +
					GOODIX_BERLIN_CHECKSUM_SIZE);
	if (error) {
		dev_warn_ratelimited(cd->dev,
				     "failed get event head data: %d\n", error);
		goto out;
	}

	if (cd->event.hdr.status == 0)
		goto out;

	/* GTS9U-DBG: dump the raw report (header + two 16-byte records + checksum) */
	dev_info_ratelimited(cd->dev, "GTS9U-DBG raw[42]: %42ph\n",
			     (u8 *)&cd->event);

	if (!goodix_berlin_checksum_valid((u8 *)&cd->event.hdr,
					  GOODIX_BERLIN_HEADER_SIZE)) {
		dev_warn_ratelimited(cd->dev,
				     "touch head checksum error: %*ph\n",
				     (int)GOODIX_BERLIN_HEADER_SIZE,
				     &cd->event.hdr);
		goto out_clear;
	}

	if (cd->event.hdr.status & GOODIX_BERLIN_TOUCH_EVENT)
		goodix_berlin_touch_handler(cd);

	if (cd->event.hdr.status & GOODIX_BERLIN_REQUEST_EVENT) {
		switch (cd->event.hdr.request_type) {
		case GOODIX_BERLIN_REQUEST_CODE_RESET:
			if (cd->reset_gpio)
				goodix_berlin_request_handle_reset(cd);
			break;

		default:
			dev_warn(cd->dev, "unsupported request code 0x%x\n",
				 cd->event.hdr.request_type);
		}
	}


out_clear:
	/* Clear up status field */
	regmap_write(cd->regmap, cd->touch_data_addr, 0);

out:
	return IRQ_HANDLED;
}

static int goodix_berlin_input_dev_config(struct goodix_berlin_core *cd,
					  const struct input_id *id)
{
	struct input_dev *input_dev;
	int error;

	input_dev = devm_input_allocate_device(cd->dev);
	if (!input_dev)
		return -ENOMEM;

	cd->input_dev = input_dev;
	input_set_drvdata(input_dev, cd);

	input_dev->name = "Goodix Berlin Capacitive TouchScreen";
	input_dev->phys = "input/ts";

	input_dev->id = *id;

	/*
	 * GTS9U: the gt6936_gts9u firmware reports coordinates in portrait
	 * panel-native units (max_coords 1848 x 2960, from the downstream
	 * sec_input node), not the upstream SZ_64K logical range.
	 * goodix_berlin_report_state() rotates each contact 270 degrees into
	 * the landscape desktop frame, so the *reported* axes are swapped
	 * relative to the panel: X spans the panel's long axis and Y its short
	 * axis. Advertise the post-rotation range here.
	 */
	input_set_abs_params(cd->input_dev, ABS_MT_POSITION_X,
			     0, GOODIX_GTS9U_RAW_MAX_Y, 0, 0);
	input_set_abs_params(cd->input_dev, ABS_MT_POSITION_Y,
			     0, GOODIX_GTS9U_RAW_MAX_X, 0, 0);
	input_set_abs_params(cd->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	touchscreen_parse_properties(cd->input_dev, true, &cd->props);

	error = input_mt_init_slots(cd->input_dev, GOODIX_BERLIN_MAX_TOUCH,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;

	error = input_register_device(cd->input_dev);
	if (error)
		return error;

	return 0;
}

static int goodix_berlin_suspend(struct device *dev)
{
	struct goodix_berlin_core *cd = dev_get_drvdata(dev);

	disable_irq(cd->irq);
	goodix_berlin_power_off(cd);

	return 0;
}

static int goodix_berlin_resume(struct device *dev)
{
	struct goodix_berlin_core *cd = dev_get_drvdata(dev);
	int error;

	error = goodix_berlin_power_on(cd);
	if (error)
		return error;

	enable_irq(cd->irq);

	return 0;
}

EXPORT_GPL_SIMPLE_DEV_PM_OPS(goodix_berlin_pm_ops,
			     goodix_berlin_suspend, goodix_berlin_resume);

static void goodix_berlin_power_off_act(void *data)
{
	struct goodix_berlin_core *cd = data;

	goodix_berlin_power_off(cd);
}

static ssize_t registers_read(struct file *filp, struct kobject *kobj,
			      const struct bin_attribute *bin_attr,
			      char *buf, loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct goodix_berlin_core *cd = dev_get_drvdata(dev);
	int error;

	error = regmap_raw_read(cd->regmap, off, buf, count);

	return error ? error : count;
}

static ssize_t registers_write(struct file *filp, struct kobject *kobj,
			       const struct bin_attribute *bin_attr,
			       char *buf, loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct goodix_berlin_core *cd = dev_get_drvdata(dev);
	int error;

	error = regmap_raw_write(cd->regmap, off, buf, count);

	return error ? error : count;
}

static const BIN_ATTR_ADMIN_RW(registers, 0);

static const struct bin_attribute *const goodix_berlin_bin_attrs[] = {
	&bin_attr_registers,
	NULL,
};

static const struct attribute_group goodix_berlin_attr_group = {
	.bin_attrs = goodix_berlin_bin_attrs,
};

const struct attribute_group *goodix_berlin_groups[] = {
	&goodix_berlin_attr_group,
	NULL,
};
EXPORT_SYMBOL_GPL(goodix_berlin_groups);

int goodix_berlin_probe(struct device *dev, int irq, const struct input_id *id,
			struct regmap *regmap,
			const struct goodix_berlin_ic_data *ic_data)
{
	struct goodix_berlin_core *cd;
	int error;

	if (irq <= 0) {
		dev_err(dev, "Missing interrupt number\n");
		return -EINVAL;
	}

	cd = devm_kzalloc(dev, sizeof(*cd), GFP_KERNEL);
	if (!cd)
		return -ENOMEM;

	cd->dev = dev;
	cd->regmap = regmap;
	cd->irq = irq;
	cd->ic_data = ic_data;

	cd->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(cd->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(cd->reset_gpio),
				     "Failed to request reset gpio\n");

	cd->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(cd->avdd))
		return dev_err_probe(dev, PTR_ERR(cd->avdd),
				     "Failed to request avdd regulator\n");

	cd->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(cd->vddio))
		return dev_err_probe(dev, PTR_ERR(cd->vddio),
				     "Failed to request vddio regulator\n");

	error = goodix_berlin_power_on(cd);
	if (error) {
		dev_err(dev, "failed power on");
		return error;
	}

	error = devm_add_action_or_reset(dev, goodix_berlin_power_off_act, cd);
	if (error)
		return error;

	error = goodix_berlin_read_version(cd);
	if (error) {
		dev_err(dev, "failed to get version info");
		return error;
	}

	error = goodix_berlin_get_ic_info(cd);
	if (error) {
		dev_err(dev, "invalid ic info, abort");
		return error;
	}

	error = goodix_berlin_input_dev_config(cd, id);
	if (error) {
		dev_err(dev, "failed set input device");
		return error;
	}

	error = devm_request_threaded_irq(dev, cd->irq, NULL, goodix_berlin_irq,
					  IRQF_ONESHOT, "goodix-berlin", cd);
	if (error) {
		dev_err(dev, "request threaded irq failed: %d\n", error);
		return error;
	}

	dev_set_drvdata(dev, cd);

	dev_dbg(dev, "Goodix Berlin %s Touchscreen Controller",
		cd->fw_version.patch_pid);

	return 0;
}
EXPORT_SYMBOL_GPL(goodix_berlin_probe);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Goodix Berlin Core Touchscreen driver");
MODULE_AUTHOR("Neil Armstrong <neil.armstrong@linaro.org>");
