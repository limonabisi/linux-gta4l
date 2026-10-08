// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech FT8201 SPI touchscreen driver (TDDI, host firmware download)
 *
 * The touch controller shares its silicon with the display driver and has
 * no flash of its own: the host has to load the touch firmware into the
 * controller's program RAM over SPI every time the panel has been reset.
 *
 * Protocol and register layout taken from the downstream "focaltech_touch"
 * driver, Copyright (c) 2012-2020, FocalTech Systems, Ltd.
 */

#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <drm/drm_panel.h>

/*
 * Every SPI frame starts with a four byte header: command, control, and the
 * big endian payload length. Payloads follow three dummy bytes. Reads ask for
 * a CRC16 after the payload. The controller reports its status in the fourth
 * received byte.
 */
#define FTS_SPI_HDR_LEN			4
#define FTS_SPI_DUMMY_BYTES		3
#define FTS_SPI_CRC_LEN			2
#define FTS_SPI_CTRL_WRITE		0x00
#define FTS_SPI_CTRL_READ		0xa0	/* read | data CRC enable */
#define FTS_SPI_STATUS_ERR		0xa0
#define FTS_SPI_RETRIES			3
#define FTS_SPI_CS_HIGH_US		150
#define FTS_SPI_MAX_HZ			6000000

/* Largest payload of a single program RAM write, as used by the vendor */
#define FTS_PACKET_LEN			(32 * 1024 - 16)
#define FTS_READ_MAX_LEN		128
#define FTS_XFER_BUF_LEN		(FTS_SPI_HDR_LEN + FTS_SPI_DUMMY_BYTES + \
					 FTS_PACKET_LEN + FTS_SPI_CRC_LEN)

/* ROM boot loader commands */
#define FTS_CMD_START_APP		0x08
#define FTS_CMD_START1			0x55
#define FTS_CMD_READ_ID			0x90
#define FTS_CMD_SET_PRAM_ADDR		0xad
#define FTS_CMD_WRITE_PRAM		0xae
#define FTS_CMD_ECC			0xcc
#define FTS_CMD_ECC_READ		0xcd
#define FTS_CMD_ECC_FINISH		0xce
#define FTS_ECC_FINISH_OK		0xa5
#define FTS_ECC_FINISH_TRIES		100
#define FTS_ECC_POLY			0x8408
#define FTS_ECC_LEN_MAX			(128 * 1024)

/* Firmware registers */
#define FTS_REG_POINT_DATA		0x01
#define FTS_REG_CHIP_ID			0xa3
#define FTS_REG_POWER_MODE		0xa5
#define FTS_POWER_MODE_SLEEP		0x03
#define FTS_REG_FW_VER			0xa6
#define FTS_REG_BOOT_STATE		0xd0
#define FTS_BOOT_STATE_ROM		0x01

/* FT8201AB: ID of the ROM boot loader and of the running firmware */
#define FTS_ROM_ID_H			0x82
#define FTS_ROM_ID_L			0x01
#define FTS_CHIP_ID_H			0x82

/* Firmware image: application length (in 16 bit words) at 0x100 */
#define FTS_APP_INFO_OFF		0x100
#define FTS_FW_MIN_LEN			0x120
#define FTS_APP_MAX_LEN			(96 * 1024)

#define FTS_BOOT_LOOPS			30
#define FTS_BOOT_ID_TRIES		3
#define FTS_BOOT_DELAY_US		8000
#define FTS_DOWNLOAD_TRIES		3
#define FTS_VALID_TRIES			5
#define FTS_VALID_INTERVAL_MS		200

/* Touch report, read from FTS_REG_POINT_DATA */
#define FTS_MAX_FINGERS			10
#define FTS_MAX_ID			0x0a
#define FTS_FINGER_LEN			6
#define FTS_GESTURE_DATA_LEN		28
#define FTS_POINT_DATA_LEN		(2 + FTS_MAX_FINGERS * FTS_FINGER_LEN + \
					 FTS_GESTURE_DATA_LEN)
#define FTS_EVENT_DOWN			0
#define FTS_EVENT_CONTACT		2
#define FTS_DEFAULT_PRESSURE		0x3f
#define FTS_DEFAULT_AREA		0x09
#define FTS_RECOVERY_PATTERN		0xef

#define FTS_MAX_X			1200
#define FTS_MAX_Y			2000

#define FTS_FW_LOAD_TRIES		20
#define FTS_FW_RETRY_MS			3000
#define FTS_FW_PANEL_DELAY_MS		100

static char *panel;
module_param(panel, charp, 0444);
MODULE_PARM_DESC(panel,
		 "Panel to assume instead of the one named on the kernel command line, e.g. ft8201ab_lide_hsd_video");

struct fts_ts {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct gpio_desc *reset_gpio;

	/* Serialises all SPI traffic and the firmware state below */
	struct mutex lock;
	u8 *tx_buf;
	u8 *rx_buf;

	struct drm_panel_follower follower;
	bool is_follower;
	struct delayed_work fw_work;

	const char *fw_name;
	const struct firmware *fw;
	u32 app_len;
	unsigned int fw_tries;

	/* The panel is up and the controller is supposed to be working */
	bool enabled;
	/* Firmware is loaded and the interrupt is enabled */
	bool running;
};

/* ---------------------------------------------------------------------- */
/* SPI access                                                             */
/* ---------------------------------------------------------------------- */

static int fts_spi_xfer(struct fts_ts *ts, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->tx_buf,
		.rx_buf = ts->rx_buf,
		.len = len,
		.speed_hz = FTS_SPI_MAX_HZ,
	};
	int ret;

	ret = spi_sync_transfer(ts->spi, &xfer, 1);

	/* Chip select has to stay high for a while between frames */
	fsleep(FTS_SPI_CS_HIGH_US);

	return ret;
}

static void fts_spi_header(struct fts_ts *ts, u8 cmd, u8 ctrl, size_t len)
{
	ts->tx_buf[0] = cmd;
	ts->tx_buf[1] = ctrl;
	put_unaligned_be16(len, &ts->tx_buf[2]);
}

static int fts_write(struct fts_ts *ts, u8 cmd, const u8 *data, size_t len)
{
	size_t xfer_len = FTS_SPI_HDR_LEN;
	int i, ret = -EIO;

	if (len > FTS_PACKET_LEN)
		return -EINVAL;

	fts_spi_header(ts, cmd, FTS_SPI_CTRL_WRITE, len);
	if (len) {
		/* The dummy bytes are only sent when there is a payload */
		memset(&ts->tx_buf[FTS_SPI_HDR_LEN], 0, FTS_SPI_DUMMY_BYTES);
		xfer_len += FTS_SPI_DUMMY_BYTES;
		memcpy(&ts->tx_buf[xfer_len], data, len);
		xfer_len += len;
	}

	for (i = 0; i < FTS_SPI_RETRIES; i++) {
		ret = fts_spi_xfer(ts, xfer_len);
		if (!ret && !(ts->rx_buf[3] & FTS_SPI_STATUS_ERR))
			return 0;

		ret = -EIO;
		fsleep(FTS_SPI_CS_HIGH_US);
	}

	return ret;
}

static int fts_write_u8(struct fts_ts *ts, u8 cmd, u8 val)
{
	return fts_write(ts, cmd, &val, 1);
}

/*
 * Returns -EBADMSG when the data was received but failed its CRC check; the
 * (unverified) data is still copied in that case, as the controller signals
 * a reset that way.
 */
static int fts_read(struct fts_ts *ts, u8 cmd, u8 *data, size_t len)
{
	const size_t pos = FTS_SPI_HDR_LEN + FTS_SPI_DUMMY_BYTES;
	const size_t xfer_len = pos + len + FTS_SPI_CRC_LEN;
	int i, ret = -EIO;

	if (!len || len > FTS_READ_MAX_LEN)
		return -EINVAL;

	memset(ts->tx_buf, 0, xfer_len);
	fts_spi_header(ts, cmd, FTS_SPI_CTRL_READ, len);

	for (i = 0; i < FTS_SPI_RETRIES; i++) {
		ret = fts_spi_xfer(ts, xfer_len);
		if (ret || (ts->rx_buf[3] & FTS_SPI_STATUS_ERR)) {
			ret = -EIO;
			fsleep(FTS_SPI_CS_HIGH_US);
			continue;
		}

		memcpy(data, &ts->rx_buf[pos], len);
		if (crc_ccitt(0xffff, &ts->rx_buf[pos], len) ==
		    get_unaligned_le16(&ts->rx_buf[pos + len]))
			return 0;

		ret = -EBADMSG;
		fsleep(FTS_SPI_CS_HIGH_US);
	}

	return ret;
}

static int fts_read_u8(struct fts_ts *ts, u8 cmd, u8 *val)
{
	return fts_read(ts, cmd, val, 1);
}

/* ---------------------------------------------------------------------- */
/* ROM boot loader                                                        */
/* ---------------------------------------------------------------------- */

static void fts_hw_reset(struct fts_ts *ts)
{
	if (!ts->reset_gpio)
		return;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
}

static int fts_check_boot_id(struct fts_ts *ts, u8 *id)
{
	int ret;

	ret = fts_read(ts, FTS_CMD_READ_ID, id, 2);
	if (ret)
		return ret;

	return id[0] == FTS_ROM_ID_H && id[1] == FTS_ROM_ID_L ? 0 : -ENODEV;
}

/* Reset the controller and wait until its ROM boot loader answers */
static int fts_enter_boot(struct fts_ts *ts)
{
	u8 id[2] = { };
	int i, j, ret = -EIO;

	for (i = 0; i < FTS_BOOT_LOOPS; i++) {
		fts_hw_reset(ts);
		usleep_range(FTS_BOOT_DELAY_US, FTS_BOOT_DELAY_US + 1000);

		for (j = 0; j < FTS_BOOT_ID_TRIES; j++) {
			ret = fts_write(ts, FTS_CMD_START1, NULL, 0);
			if (ret)
				continue;

			usleep_range(FTS_BOOT_DELAY_US, FTS_BOOT_DELAY_US + 1000);
			ret = fts_check_boot_id(ts, id);
			if (!ret)
				return 0;
		}
	}

	dev_err(&ts->spi->dev, "boot loader did not answer: %d, ID %02x %02x\n",
		ret, id[0], id[1]);

	return ret;
}

/* True if the controller has fallen back to its ROM boot loader */
static bool fts_in_rom_boot(struct fts_ts *ts)
{
	u8 id[2], state;

	if (fts_check_boot_id(ts, id))
		return false;

	if (fts_read_u8(ts, FTS_REG_BOOT_STATE, &state))
		return false;

	return state == FTS_BOOT_STATE_ROM;
}

/* ---------------------------------------------------------------------- */
/* Firmware download                                                      */
/* ---------------------------------------------------------------------- */

static int fts_fw_parse(struct fts_ts *ts)
{
	const struct firmware *fw = ts->fw;
	u16 len, len_n;

	if (fw->size < FTS_FW_MIN_LEN) {
		dev_err(&ts->spi->dev, "firmware too short (%zu bytes)\n", fw->size);
		return -EINVAL;
	}

	/* Length of the application in 16 bit words, followed by its complement */
	len = get_unaligned_be16(&fw->data[FTS_APP_INFO_OFF]);
	len_n = get_unaligned_be16(&fw->data[FTS_APP_INFO_OFF + 2]);
	if ((u16)(len + len_n) != 0xffff) {
		dev_err(&ts->spi->dev, "bad application length %04x/%04x\n", len, len_n);
		return -EINVAL;
	}

	ts->app_len = len * 2;
	if (ts->app_len < FTS_FW_MIN_LEN || ts->app_len > FTS_APP_MAX_LEN ||
	    ts->app_len > fw->size) {
		dev_err(&ts->spi->dev, "bad application length %u (image %zu bytes)\n",
			ts->app_len, fw->size);
		return -EINVAL;
	}

	dev_info(&ts->spi->dev, "firmware %s: %zu bytes, application %u bytes\n",
		 ts->fw_name, fw->size, ts->app_len);

	return 0;
}

static void fts_fw_release(struct fts_ts *ts)
{
	release_firmware(ts->fw);
	ts->fw = NULL;
	ts->app_len = 0;
}

static int fts_write_pram(struct fts_ts *ts)
{
	const u8 *data = ts->fw->data;
	u32 offset, len;
	u8 addr[3];
	int ret;

	for (offset = 0; offset < ts->app_len; offset += len) {
		len = min_t(u32, ts->app_len - offset, FTS_PACKET_LEN);

		put_unaligned_be24(offset, addr);
		ret = fts_write(ts, FTS_CMD_SET_PRAM_ADDR, addr, sizeof(addr));
		if (ret)
			return ret;

		ret = fts_write(ts, FTS_CMD_WRITE_PRAM, &data[offset], len);
		if (ret)
			return ret;
	}

	return 0;
}

static u16 fts_ecc_host(const u8 *data, u32 len)
{
	u16 ecc = 0;
	u32 i;
	int j;

	for (i = 0; i < len; i += 2) {
		ecc ^= get_unaligned_be16(&data[i]);
		for (j = 0; j < 16; j++)
			ecc = (ecc & 1) ? (ecc >> 1) ^ FTS_ECC_POLY : ecc >> 1;
	}

	return ecc;
}

static int fts_ecc_tp(struct fts_ts *ts, u32 addr, u32 len, u16 *ecc)
{
	u8 cmd[6], val[2];
	int i, ret;

	put_unaligned_be24(addr, &cmd[0]);
	put_unaligned_be24(len, &cmd[3]);
	ret = fts_write(ts, FTS_CMD_ECC, cmd, sizeof(cmd));
	if (ret)
		return ret;

	usleep_range(2000, 3000);

	for (i = 0; i < FTS_ECC_FINISH_TRIES; i++) {
		ret = fts_read_u8(ts, FTS_CMD_ECC_FINISH, val);
		if (ret)
			return ret;
		if (val[0] == FTS_ECC_FINISH_OK)
			break;

		usleep_range(1000, 1100);
	}
	if (i == FTS_ECC_FINISH_TRIES)
		return -ETIMEDOUT;

	ret = fts_read(ts, FTS_CMD_ECC_READ, val, sizeof(val));
	if (ret)
		return ret;

	*ecc = get_unaligned_be16(val);

	return 0;
}

static int fts_ecc_check(struct fts_ts *ts)
{
	const u8 *data = ts->fw->data;
	u32 offset, len;
	u16 host, tp;
	int ret;

	for (offset = 0; offset < ts->app_len; offset += len) {
		len = min_t(u32, ts->app_len - offset, FTS_ECC_LEN_MAX);

		host = fts_ecc_host(&data[offset], len);
		ret = fts_ecc_tp(ts, offset, len, &tp);
		if (ret)
			return ret;

		if (host != tp) {
			dev_err(&ts->spi->dev, "ECC mismatch at 0x%x: host %04x, controller %04x\n",
				offset, host, tp);
			return -EIO;
		}
	}

	return 0;
}

static int fts_download_firmware_once(struct fts_ts *ts)
{
	int ret;

	ret = fts_enter_boot(ts);
	if (ret)
		return ret;

	ret = fts_write_pram(ts);
	if (ret) {
		dev_err(&ts->spi->dev, "writing program RAM failed: %d\n", ret);
		return ret;
	}

	ret = fts_ecc_check(ts);
	if (ret)
		return ret;

	/* Remap the program RAM and run the firmware */
	ret = fts_write(ts, FTS_CMD_START_APP, NULL, 0);
	if (ret)
		return ret;

	usleep_range(10000, 11000);

	return 0;
}

static int fts_download_firmware(struct fts_ts *ts)
{
	int retry, ret;

	for (retry = 0; retry < FTS_DOWNLOAD_TRIES; retry++) {
		ret = fts_download_firmware_once(ts);
		if (!ret)
			return 0;

		dev_warn(&ts->spi->dev, "firmware download attempt %d failed: %d\n",
			 retry + 1, ret);
	}

	return ret;
}

/* Wait for the firmware to come up; the vendor carries on regardless */
static void fts_wait_fw_valid(struct fts_ts *ts)
{
	u8 id = 0, ver = 0;
	int i;

	msleep(50);

	for (i = 0; i < FTS_VALID_TRIES; i++) {
		if (!fts_read_u8(ts, FTS_REG_CHIP_ID, &id) && id == FTS_CHIP_ID_H)
			break;

		msleep(FTS_VALID_INTERVAL_MS);
	}

	if (i == FTS_VALID_TRIES) {
		dev_warn(&ts->spi->dev, "firmware not ready, chip ID 0x%02x\n", id);
		return;
	}

	fts_read_u8(ts, FTS_REG_FW_VER, &ver);
	dev_info(&ts->spi->dev, "firmware running, chip ID 0x%02x, version 0x%02x\n",
		 id, ver);
}

/* ---------------------------------------------------------------------- */
/* Start / stop                                                           */
/* ---------------------------------------------------------------------- */

static void fts_release_fingers(struct fts_ts *ts)
{
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static void fts_fw_retry(struct fts_ts *ts, const char *what, int err)
{
	if (++ts->fw_tries >= FTS_FW_LOAD_TRIES) {
		dev_err(&ts->spi->dev, "%s failed (%d), giving up\n", what, err);
		return;
	}

	if (ts->fw_tries == 1)
		dev_warn(&ts->spi->dev, "%s failed (%d), will retry\n", what, err);

	schedule_delayed_work(&ts->fw_work, msecs_to_jiffies(FTS_FW_RETRY_MS));
}

static void fts_fw_work(struct work_struct *work)
{
	struct fts_ts *ts = container_of(work, struct fts_ts, fw_work.work);
	ktime_t start;
	int ret;

	guard(mutex)(&ts->lock);

	if (!ts->enabled || ts->running)
		return;

	if (!ts->fw) {
		ret = firmware_request_nowarn(&ts->fw, ts->fw_name, &ts->spi->dev);
		if (ret) {
			ts->fw = NULL;
			/* The image may live on a filesystem that is not there yet */
			fts_fw_retry(ts, "loading the firmware image", ret);
			return;
		}

		ret = fts_fw_parse(ts);
		if (ret) {
			fts_fw_release(ts);
			return;
		}
	}

	start = ktime_get();
	ret = fts_download_firmware(ts);
	if (ret) {
		fts_fw_retry(ts, "firmware download", ret);
		return;
	}

	dev_info(&ts->spi->dev, "firmware downloaded in %lld ms\n",
		 ktime_ms_delta(ktime_get(), start));

	fts_wait_fw_valid(ts);

	ts->fw_tries = 0;
	ts->running = true;
	enable_irq(ts->spi->irq);
}

static void fts_start(struct fts_ts *ts, unsigned int delay_ms)
{
	scoped_guard(mutex, &ts->lock) {
		ts->enabled = true;
		ts->fw_tries = 0;
	}

	schedule_delayed_work(&ts->fw_work, msecs_to_jiffies(delay_ms));
}

static void fts_stop(struct fts_ts *ts)
{
	bool was_running;

	scoped_guard(mutex, &ts->lock) {
		ts->enabled = false;
		was_running = ts->running;
		ts->running = false;
		if (was_running)
			disable_irq_nosync(ts->spi->irq);
	}

	cancel_delayed_work_sync(&ts->fw_work);

	if (!was_running)
		return;

	/* Wait for a report that is being handled right now */
	synchronize_irq(ts->spi->irq);

	guard(mutex)(&ts->lock);
	fts_write_u8(ts, FTS_REG_POWER_MODE, FTS_POWER_MODE_SLEEP);
	fts_release_fingers(ts);
}

/* ---------------------------------------------------------------------- */
/* Touch reports                                                          */
/* ---------------------------------------------------------------------- */

static bool fts_is_recovery_pattern(const u8 *data, int ret)
{
	if (data[1] == FTS_RECOVERY_PATTERN && data[2] == FTS_RECOVERY_PATTERN &&
	    data[3] == FTS_RECOVERY_PATTERN)
		return true;

	return ret && data[0] == FTS_RECOVERY_PATTERN;
}

static bool fts_is_blank_report(const u8 *data)
{
	int i;

	for (i = 1; i <= 5; i++) {
		if (data[i] != 0xff)
			return false;
	}

	return true;
}

static void fts_report_fingers(struct fts_ts *ts, const u8 *data)
{
	unsigned int num_points = data[1] & 0x0f;
	unsigned int i, n = 0;

	if (num_points > FTS_MAX_FINGERS)
		return;

	/* Validate the whole report before passing anything on */
	for (i = 0; i < FTS_MAX_FINGERS; i++) {
		const u8 *p = &data[2 + i * FTS_FINGER_LEN];
		unsigned int event = p[0] >> 6;

		if ((p[2] >> 4) >= FTS_MAX_ID)
			break;
		if ((event == FTS_EVENT_DOWN || event == FTS_EVENT_CONTACT) && !num_points)
			return;
		n++;
	}

	for (i = 0; i < n; i++) {
		const u8 *p = &data[2 + i * FTS_FINGER_LEN];
		unsigned int event = p[0] >> 6;
		unsigned int x, y, pressure, area;

		if (event != FTS_EVENT_DOWN && event != FTS_EVENT_CONTACT)
			continue;

		x = min_t(unsigned int, ((p[0] & 0x0f) << 8) | p[1], ts->prop.max_x);
		y = min_t(unsigned int, ((p[2] & 0x0f) << 8) | p[3], ts->prop.max_y);
		pressure = p[4] ?: FTS_DEFAULT_PRESSURE;
		area = (p[5] >> 4) ?: FTS_DEFAULT_AREA;

		input_mt_slot(ts->input, p[2] >> 4);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, area);
		input_report_abs(ts->input, ABS_MT_PRESSURE, pressure);
	}

	/* Fingers that are no longer reported as down are released here */
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static irqreturn_t fts_irq_thread(int irq, void *dev_id)
{
	struct fts_ts *ts = dev_id;
	u8 data[FTS_POINT_DATA_LEN];
	int ret;

	guard(mutex)(&ts->lock);

	if (!ts->running)
		return IRQ_HANDLED;

	memset(data, 0xff, sizeof(data));
	ret = fts_read(ts, FTS_REG_POINT_DATA, data, sizeof(data));

	/* The controller lost its firmware (ESD, panel reset): load it again */
	if (fts_is_recovery_pattern(data, ret)) {
		fts_release_fingers(ts);
		if (fts_in_rom_boot(ts)) {
			dev_warn(&ts->spi->dev, "controller was reset, reloading firmware\n");
			ts->running = false;
			disable_irq_nosync(irq);
			ts->fw_tries = 0;
			schedule_delayed_work(&ts->fw_work, 0);
		}
		return IRQ_HANDLED;
	}

	if (ret) {
		dev_err_ratelimited(&ts->spi->dev, "failed to read touch data: %d\n", ret);
		return IRQ_HANDLED;
	}

	/* An in-cell controller returns all ones while it is busy */
	if ((data[1] & 0x0f) == 0x0f && fts_is_blank_report(data)) {
		fts_release_fingers(ts);
		return IRQ_HANDLED;
	}

	fts_report_fingers(ts, data);

	return IRQ_HANDLED;
}

/* ---------------------------------------------------------------------- */
/* Panel follower and power management                                    */
/* ---------------------------------------------------------------------- */

static int fts_panel_prepared(struct drm_panel_follower *follower)
{
	struct fts_ts *ts = container_of(follower, struct fts_ts, follower);

	/* The panel reset wiped the touch firmware; load it once the panel is up */
	fts_start(ts, FTS_FW_PANEL_DELAY_MS);

	return 0;
}

static int fts_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct fts_ts *ts = container_of(follower, struct fts_ts, follower);

	fts_stop(ts);

	return 0;
}

static const struct drm_panel_follower_funcs fts_panel_follower_funcs = {
	.panel_prepared = fts_panel_prepared,
	.panel_unpreparing = fts_panel_unpreparing,
};

static int fts_suspend(struct device *dev)
{
	struct fts_ts *ts = dev_get_drvdata(dev);

	if (!ts->is_follower)
		fts_stop(ts);

	return 0;
}

static int fts_resume(struct device *dev)
{
	struct fts_ts *ts = dev_get_drvdata(dev);

	if (!ts->is_follower)
		fts_start(ts, FTS_FW_PANEL_DELAY_MS);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(fts_pm_ops, fts_suspend, fts_resume);

/* ---------------------------------------------------------------------- */
/* Probe                                                                  */
/* ---------------------------------------------------------------------- */

static void fts_cleanup(void *data)
{
	struct fts_ts *ts = data;

	fts_stop(ts);
	fts_fw_release(ts);
}

/*
 * The touch node is shared with the drivers for the other touch controllers
 * this tablet was built with. The bootloader names the fitted panel on the
 * command line (msm_drm.dsi_display0=qcom,mdss_dsi_ft8201ab_<panel>_video:);
 * only bind when that is an FT8201 panel. Returns the panel name or NULL.
 */
static const char *fts_find_panel(struct device *dev)
{
	static const char key[] = "mdss_dsi_ft8201";
	const char *args, *s;

	if (panel && *panel)
		return strstr(panel, "ft8201") ? panel : NULL;

	if (!of_chosen || of_property_read_string(of_chosen, "bootargs", &args))
		return NULL;

	s = strstr(args, key);
	if (!s)
		return NULL;

	s += sizeof("mdss_dsi_") - 1;

	return devm_kasprintf(dev, GFP_KERNEL, "%.*s", (int)strcspn(s, ": "), s);
}

/* Firmware file per panel, as chosen by the vendor driver */
static const char *fts_fw_name_for_panel(const char *name)
{
	static const struct {
		const char *panel;
		const char *fw_name;
	} map[] = {
		{ "lide_hsd", "focal_ts_fw_lide_hsd.bin" },
		{ "tianma_tianma", "focal_ts_fw_tianma_tianma.bin" },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(map); i++) {
		if (strstr(name, map[i].panel))
			return map[i].fw_name;
	}

	return "focal_ts_fw.bin";
}

static int fts_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct input_dev *input;
	const char *panel_name;
	struct fts_ts *ts;
	int ret;

	/* Decide before touching any hardware: another driver may own it */
	panel_name = fts_find_panel(dev);
	if (!panel_name) {
		dev_dbg(dev, "no FT8201 panel fitted\n");
		return -ENODEV;
	}

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;

	if (!spi->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	ts->tx_buf = devm_kmalloc(dev, FTS_XFER_BUF_LEN, GFP_KERNEL);
	ts->rx_buf = devm_kmalloc(dev, FTS_XFER_BUF_LEN, GFP_KERNEL);
	if (!ts->tx_buf || !ts->rx_buf)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &ts->lock);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&ts->fw_work, fts_fw_work);
	spi_set_drvdata(spi, ts);

	/* Reset is active low; keep the controller running */
	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset GPIO\n");

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "SPI setup failed\n");

	ts->fw_name = fts_fw_name_for_panel(panel_name);
	dev_info(dev, "panel %s, firmware %s\n", panel_name, ts->fw_name);

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	ts->input = input;
	input->name = "FocalTech FT8201 Touchscreen";
	input->id.bustype = BUS_SPI;

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, FTS_MAX_X - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, FTS_MAX_Y - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	touchscreen_parse_properties(input, true, &ts->prop);

	ret = input_mt_init_slots(input, FTS_MAX_FINGERS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	/*
	 * The interrupt line is shared with the other controllers' nodes,
	 * which use a different polarity: the FocalTech firmware pulses it low.
	 */
	ret = devm_request_threaded_irq(dev, spi->irq, NULL, fts_irq_thread,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
					IRQF_NO_AUTOEN,
					dev_name(dev), ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request interrupt\n");

	ret = input_register_device(input);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, fts_cleanup, ts);
	if (ret)
		return ret;

	ts->is_follower = drm_is_panel_follower(dev);
	if (ts->is_follower) {
		ts->follower.funcs = &fts_panel_follower_funcs;
		ret = devm_drm_panel_add_follower(dev, &ts->follower);
		if (ret)
			return dev_err_probe(dev, ret, "failed to follow the panel\n");
	} else {
		dev_warn(dev, "no panel to follow, assuming the display is already on\n");
		fts_start(ts, 0);
	}

	return 0;
}

static const struct of_device_id fts_of_match[] = {
	{ .compatible = "focaltech,ft8201-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, fts_of_match);

static const struct spi_device_id fts_spi_ids[] = {
	{ "ft8201-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, fts_spi_ids);

static struct spi_driver fts_spi_driver = {
	.driver = {
		.name = "focaltech-ft8201-spi",
		.of_match_table = fts_of_match,
		.pm = pm_sleep_ptr(&fts_pm_ops),
	},
	.probe = fts_probe,
	.id_table = fts_spi_ids,
};
module_spi_driver(fts_spi_driver);

MODULE_DESCRIPTION("FocalTech FT8201 SPI touchscreen driver");
MODULE_LICENSE("GPL");
