// SPDX-License-Identifier: GPL-2.0-only
/*
 * Himax HX83102E SPI touchscreen driver (in-cell TDDI, "zero flash")
 *
 * The touch controller shares its silicon with the display driver and has
 * no flash of its own: the host has to load the touch firmware into the
 * controller's SRAM over SPI every time the panel has been reset.
 *
 * Protocol and register layout taken from the downstream "hxchipset" driver,
 * Copyright (C) 2019 Himax Corporation.
 */

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

#define HX_SPI_WRITE			0xf2
#define HX_SPI_READ			0xf3

/* Bus registers, accessed directly with HX_SPI_WRITE / HX_SPI_READ */
#define HX_REG_AHB_ADDR			0x00
#define HX_REG_AHB_RDATA		0x08
#define HX_REG_AHB_DIR			0x0c
#define HX_REG_INCR4			0x0d
#define HX_REG_AHB_BURST		0x11
#define HX_REG_CONTI			0x13
#define HX_REG_EVENT			0x30
#define HX_REG_PSW_LB			0x31
#define HX_REG_PSW_UB			0x32

#define HX_AHB_DIR_READ			0x00
#define HX_CONTI_EN			0x31
#define HX_INCR4_EN			0x10
#define HX_INCR4_AUTO			0x01
#define HX_PSW_LB_SAFE			0x27
#define HX_PSW_UB_SAFE			0x95

/* Registers on the internal AHB bus */
#define HX_ADDR_SYSTEM_RESET		0x90000018
#define HX_ADDR_ACTIVE_RELOAD		0x90000048
#define HX_ADDR_CTRL_FW			0x9000005c
#define HX_ADDR_CS_CENTRAL_STATE	0x900000a8
#define HX_ADDR_ICID			0x900000d0
#define HX_ADDR_RESET_EVENT		0x900000e4
#define HX_ADDR_TCON_RST		0x80020020
#define HX_ADDR_ADC_RST			0x80020094
#define HX_ADDR_RELOAD_STATUS		0x80050000
#define HX_ADDR_RELOAD_CRC32		0x80050018
#define HX_ADDR_RELOAD_FROM		0x80050020
#define HX_ADDR_RELOAD_CMD		0x80050028
#define HX_ADDR_FLASH_RELOAD		0x10007f00
#define HX_ADDR_RELOAD_DONE		0x100072c0
#define HX_ADDR_SET_FRAME		0x10007294

#define HX_DATA_SYSTEM_RESET		0x55
#define HX_DATA_FW_STOP			0xa5
#define HX_DATA_ACTIVE_RELOAD		0xec
#define HX_ACTIVE_RELOAD_ACK		0x01ec
#define HX_DATA_DIS_FLASH_RELOAD	0x9aa9
#define HX_RELOAD_DONE			0x72c0
#define HX_RESET_EVENT_DONE		0x0200
#define HX_RELOAD_CMD_CRC		0x00990000

#define HX_STATE_FW_RUNNING		0x05
#define HX_STATE_SAFE_MODE		0x0c

/* One AHB write carries at most this much data, as in the vendor driver */
#define HX_AHB_CHUNK			0x10000
#define HX_TX_BUF_LEN			(2 + 4 + HX_AHB_CHUNK)
#define HX_MAX_READ_LEN			256

/* Zero flash image layout */
#define HX_FW_CFG_TABLE			0x10000
#define HX_FW_PART_ENTRY_LEN		16
#define HX_FW_MAIN_LEN			0xc000
#define HX_FW_FIXED_MIN_LEN		0xe600
#define HX_DSRAM_SIZE			0x8000
#define HX_CRC_POLY			0x82f63b78

/* Touch report: 10 fingers, see the vendor's himax_report_data_init() */
#define HX_MAX_FINGERS			10
#define HX_X_RES			1200
#define HX_Y_RES			2000
#define HX_COORD_LEN			(HX_MAX_FINGERS * 4)
#define HX_AREA_LEN			(DIV_ROUND_UP(HX_MAX_FINGERS, 4) * 4)
#define HX_POINT_CNT_POS		(HX_COORD_LEN + HX_AREA_LEN)
#define HX_TOUCH_INFO_LEN		(HX_POINT_CNT_POS + 4)
#define HX_EVENT_STACK_LEN		128

#define HX_EXCP_ED_COUNT		5

#define HX_FW_LOAD_TRIES		20
#define HX_FW_RETRY_MS			3000
#define HX_FW_PANEL_DELAY_MS		100

struct hx_sense_off_cfg {
	unsigned int stop_delay_us;
	unsigned int running_delay_us;
	unsigned int tries;
	unsigned int retry_delay_us;
	bool tcon_release;
	bool adc_reset;
};

struct hx_chip_data {
	const char *name;
	u8 id;
	/* SRAM targets of the fixed (<= 64 KiB) image layout */
	u32 sram_start;
	u32 adc_cfg_1;
	u32 adc_cfg_2;
	/* Version bytes inside the firmware image */
	u32 fw_ver_offset;
	u32 cfg_ver_offset;
	struct hx_sense_off_cfg sense_off;
	/* Switch AHB burst reads off around event stack reads */
	bool event_burst_off;
};

struct hx_ts {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct gpio_desc *reset_gpio;
	const struct hx_chip_data *chip;

	/* Serialises all SPI traffic and the firmware state below */
	struct mutex lock;
	u8 *tx_buf;
	u8 *rx_buf;

	struct drm_panel_follower follower;
	bool is_follower;
	struct delayed_work fw_work;

	const char *fw_name;
	const struct firmware *fw;
	/* Image with a partition table (> 64 KiB): main code plus DSRAM config */
	bool fw_partitioned;
	u32 main_sram;
	u32 main_offset;
	u32 main_len;
	u8 *cfg_buf;
	u32 cfg_sram;
	u32 cfg_len;
	u32 cfg_crc;
	unsigned int fw_tries;

	/* The panel is up and the controller is supposed to be working */
	bool enabled;
	/* Firmware is loaded and the interrupt is enabled */
	bool running;
	unsigned int ed_count;

	u8 scratch[HX_EVENT_STACK_LEN];
};

static char *panel;
module_param(panel, charp, 0444);
MODULE_PARM_DESC(panel, "Panel name to use instead of the one on the kernel command line, e.g. hx83102e_txd_auo");

/* ---------------------------------------------------------------------- */
/* SPI access                                                             */
/* ---------------------------------------------------------------------- */

static int hx_bus_write(struct hx_ts *ts, u8 reg, const u8 *data, size_t len)
{
	if (len > HX_TX_BUF_LEN - 2)
		return -EINVAL;

	ts->tx_buf[0] = HX_SPI_WRITE;
	ts->tx_buf[1] = reg;
	memcpy(&ts->tx_buf[2], data, len);

	return spi_write(ts->spi, ts->tx_buf, len + 2);
}

static int hx_bus_write_u8(struct hx_ts *ts, u8 reg, u8 val)
{
	return hx_bus_write(ts, reg, &val, 1);
}

static int hx_bus_read(struct hx_ts *ts, u8 reg, u8 *data, size_t len)
{
	struct spi_transfer xfers[2] = {
		{ .tx_buf = ts->tx_buf, .len = 3 },
		{ .rx_buf = ts->rx_buf, .len = len },
	};
	int ret;

	if (!len || len > HX_MAX_READ_LEN)
		return -EINVAL;

	ts->tx_buf[0] = HX_SPI_READ;
	ts->tx_buf[1] = reg;
	ts->tx_buf[2] = 0x00;

	ret = spi_sync_transfer(ts->spi, xfers, ARRAY_SIZE(xfers));
	if (ret)
		return ret;

	memcpy(data, ts->rx_buf, len);

	return 0;
}

static int hx_burst_enable(struct hx_ts *ts, bool auto_incr)
{
	int ret;

	ret = hx_bus_write_u8(ts, HX_REG_CONTI, HX_CONTI_EN);
	if (ret)
		return ret;

	return hx_bus_write_u8(ts, HX_REG_INCR4,
			       HX_INCR4_EN | (auto_incr ? HX_INCR4_AUTO : 0));
}

static int hx_reg_read(struct hx_ts *ts, u32 addr, u8 *data, size_t len)
{
	u8 buf[4];
	int ret;

	if (len > HX_MAX_READ_LEN)
		return -EINVAL;

	ret = hx_burst_enable(ts, len > 4);
	if (ret)
		return ret;

	put_unaligned_le32(addr, buf);
	ret = hx_bus_write(ts, HX_REG_AHB_ADDR, buf, sizeof(buf));
	if (ret)
		return ret;

	ret = hx_bus_write_u8(ts, HX_REG_AHB_DIR, HX_AHB_DIR_READ);
	if (ret)
		return ret;

	ret = hx_bus_read(ts, HX_REG_AHB_RDATA, data, len);
	if (ret)
		return ret;

	if (len > 4)
		ret = hx_burst_enable(ts, false);

	return ret;
}

static int hx_reg_read32(struct hx_ts *ts, u32 addr, u32 *val)
{
	u8 buf[4];
	int ret;

	ret = hx_reg_read(ts, addr, buf, sizeof(buf));
	if (ret)
		return ret;

	*val = get_unaligned_le32(buf);

	return 0;
}

static int hx_reg_write(struct hx_ts *ts, u32 addr, const u8 *data, size_t len)
{
	int ret;

	ret = hx_burst_enable(ts, len > 4);
	if (ret)
		return ret;

	while (len) {
		size_t n = min_t(size_t, len, HX_AHB_CHUNK);

		ts->tx_buf[0] = HX_SPI_WRITE;
		ts->tx_buf[1] = HX_REG_AHB_ADDR;
		put_unaligned_le32(addr, &ts->tx_buf[2]);
		memcpy(&ts->tx_buf[6], data, n);

		ret = spi_write(ts->spi, ts->tx_buf, n + 6);
		if (ret)
			return ret;

		addr += n;
		data += n;
		len -= n;

		if (len)
			usleep_range(100, 200);
	}

	return 0;
}

static int hx_reg_write32(struct hx_ts *ts, u32 addr, u32 val)
{
	u8 buf[4];

	put_unaligned_le32(val, buf);

	return hx_reg_write(ts, addr, buf, sizeof(buf));
}

/* ---------------------------------------------------------------------- */
/* Controller state                                                       */
/* ---------------------------------------------------------------------- */

static void hx_pin_reset(struct hx_ts *ts, unsigned int low_ms, unsigned int high_ms)
{
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(low_ms);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(high_ms);
}

/* Wake up the bus interface and switch on burst mode */
static int hx_interface_on(struct hx_ts *ts)
{
	u8 conti, incr4, dummy[4];
	int retry, ret;

	ret = hx_bus_read(ts, HX_REG_AHB_RDATA, dummy, sizeof(dummy));
	if (ret)
		return ret;

	for (retry = 0; retry < 10; retry++) {
		ret = hx_bus_write_u8(ts, HX_REG_CONTI, HX_CONTI_EN);
		if (!ret)
			ret = hx_bus_write_u8(ts, HX_REG_INCR4, HX_INCR4_EN);
		if (!ret)
			ret = hx_bus_read(ts, HX_REG_CONTI, &conti, 1);
		if (!ret)
			ret = hx_bus_read(ts, HX_REG_INCR4, &incr4, 1);
		if (ret)
			return ret;

		if (conti == HX_CONTI_EN && incr4 == HX_INCR4_EN)
			return 0;

		usleep_range(1000, 1100);
	}

	dev_dbg(&ts->spi->dev, "burst mode did not stick (%02x %02x)\n", conti, incr4);

	return 0;
}

/* Software reset, used by the vendor when there is no reset GPIO */
static int hx_system_reset(struct hx_ts *ts)
{
	int retry, ret;
	u32 val;

	ret = hx_interface_on(ts);
	if (ret)
		return ret;

	ret = hx_reg_write32(ts, HX_ADDR_CTRL_FW, 0);
	if (ret)
		return ret;

	for (retry = 0; retry < 6; retry++) {
		hx_bus_write_u8(ts, HX_REG_PSW_LB, HX_PSW_LB_SAFE);
		hx_bus_write_u8(ts, HX_REG_PSW_UB, HX_PSW_UB_SAFE);
		hx_bus_write_u8(ts, HX_REG_PSW_LB, 0x00);
		usleep_range(10000, 11000);

		ret = hx_reg_read32(ts, HX_ADDR_RESET_EVENT, &val);
		if (ret)
			return ret;
		if ((val & 0xffff) == HX_RESET_EVENT_DONE)
			return 0;
	}

	return -ETIMEDOUT;
}

static int hx_ic_reset(struct hx_ts *ts)
{
	int ret;

	if (ts->reset_gpio) {
		hx_pin_reset(ts, 5, 5);
		return 0;
	}

	ret = hx_system_reset(ts);
	if (ret == -ETIMEDOUT) {
		dev_warn(&ts->spi->dev, "software reset not acknowledged\n");
		ret = 0;
	}

	return ret;
}

/* Stop the firmware and put the MCU into safe mode for SRAM access */
static int hx_sense_off(struct hx_ts *ts, const struct hx_sense_off_cfg *cfg)
{
	unsigned int i;
	u32 state;
	int ret;

	ret = hx_reg_write32(ts, HX_ADDR_CTRL_FW, HX_DATA_FW_STOP);
	if (ret)
		return ret;
	usleep_range(cfg->stop_delay_us, cfg->stop_delay_us + 100);

	ret = hx_reg_read32(ts, HX_ADDR_CS_CENTRAL_STATE, &state);
	if (ret)
		return ret;
	if ((state & 0xff) == HX_STATE_FW_RUNNING && cfg->running_delay_us)
		usleep_range(cfg->running_delay_us, cfg->running_delay_us + 100);

	for (i = 0; i < cfg->tries; i++) {
		ret = hx_bus_write_u8(ts, HX_REG_PSW_LB, HX_PSW_LB_SAFE);
		if (!ret)
			ret = hx_bus_write_u8(ts, HX_REG_PSW_UB, HX_PSW_UB_SAFE);
		if (!ret)
			ret = hx_reg_read32(ts, HX_ADDR_CS_CENTRAL_STATE, &state);
		if (ret)
			return ret;

		if ((state & 0xff) == HX_STATE_SAFE_MODE) {
			/* Reset TCON and, on some chips, the ADC */
			ret = hx_reg_write32(ts, HX_ADDR_TCON_RST, 0);
			if (ret)
				return ret;
			usleep_range(1000, 1100);

			if (cfg->tcon_release) {
				ret = hx_reg_write32(ts, HX_ADDR_TCON_RST, 1);
				if (ret)
					return ret;
			}

			if (cfg->adc_reset) {
				ret = hx_reg_write32(ts, HX_ADDR_ADC_RST, 0);
				if (ret)
					return ret;
				usleep_range(1000, 1100);
				ret = hx_reg_write32(ts, HX_ADDR_ADC_RST, 1);
			}

			return ret;
		}

		usleep_range(cfg->retry_delay_us, cfg->retry_delay_us + 100);
		if (ts->reset_gpio)
			hx_pin_reset(ts, 20, 50);
	}

	dev_err(&ts->spi->dev, "controller did not enter safe mode, state %08x\n", state);

	return -ETIMEDOUT;
}

/* Ask the boot code to (re)start the firmware that is in SRAM */
static int hx_reload_to_active(struct hx_ts *ts)
{
	int retry, ret;
	u32 val;

	for (retry = 0; retry < 5; retry++) {
		ret = hx_reg_write32(ts, HX_ADDR_ACTIVE_RELOAD, HX_DATA_ACTIVE_RELOAD);
		if (ret)
			return ret;
		usleep_range(1000, 1100);

		ret = hx_reg_read32(ts, HX_ADDR_ACTIVE_RELOAD, &val);
		if (ret)
			return ret;
		if ((val & 0xffff) == HX_ACTIVE_RELOAD_ACK)
			return 0;
	}

	dev_warn(&ts->spi->dev, "reload to active not acknowledged (%08x)\n", val);

	return 0;
}

static int hx_sense_on(struct hx_ts *ts)
{
	int ret;

	ret = hx_interface_on(ts);
	if (ret)
		return ret;

	ret = hx_reg_write32(ts, HX_ADDR_CTRL_FW, 0);
	if (ret)
		return ret;
	usleep_range(10000, 11000);

	ret = hx_ic_reset(ts);
	if (ret)
		return ret;

	return hx_reload_to_active(ts);
}

static const struct hx_chip_data hx83102d_data = {
	.name			= "HX83102D",
	.id			= 0x2d,
	.sram_start		= 0x20000000,
	.adc_cfg_1		= 0x10007b00,
	.adc_cfg_2		= 0x10006a00,
	.fw_ver_offset		= 0xc005,
	.cfg_ver_offset		= 0xc100,
	.sense_off = {
		.stop_delay_us	= 10000,
		.tries		= 6,
		.retry_delay_us	= 5000,
		.adc_reset	= true,
	},
	.event_burst_off	= true,
};

static const struct hx_chip_data hx83102e_data = {
	.name			= "HX83102E",
	.id			= 0x2e,
	.sram_start		= 0x08000000,
	.adc_cfg_1		= 0x10006a00,
	.adc_cfg_2		= 0x10007b28,
	.fw_ver_offset		= 0xe805,
	.cfg_ver_offset		= 0xe900,
	.sense_off = {
		.stop_delay_us	= 1000,
		.running_delay_us = 10000,
		.tries		= 6,
		.retry_delay_us	= 5000,
	},
};

/* Used before the chip is known, as in the vendor's hx83102_chip_detect() */
static const struct hx_sense_off_cfg hx_detect_sense_off = {
	.stop_delay_us	= 20000,
	.tries		= 16,
	.retry_delay_us	= 10000,
	.tcon_release	= true,
	.adc_reset	= true,
};

static int hx_identify_chip(struct hx_ts *ts)
{
	static const struct hx_chip_data * const chips[] = {
		&hx83102d_data, &hx83102e_data,
	};
	u8 id[4] = { }, dummy;
	int retry, i, ret;

	if (ts->reset_gpio)
		hx_pin_reset(ts, 20, 50);

	ret = hx_bus_read(ts, HX_REG_CONTI, &dummy, 1);
	if (ret)
		return ret;

	ret = hx_sense_off(ts, &hx_detect_sense_off);
	if (ret)
		return ret;

	for (retry = 0; retry < 5; retry++) {
		ret = hx_reg_read(ts, HX_ADDR_ICID, id, sizeof(id));
		if (ret)
			return ret;

		if (id[3] != 0x83 || id[2] != 0x10)
			continue;

		for (i = 0; i < ARRAY_SIZE(chips); i++) {
			if (id[1] == chips[i]->id) {
				ts->chip = chips[i];
				dev_info(&ts->spi->dev, "found %s, chip ID %*ph\n",
					 ts->chip->name, (int)sizeof(id), id);
				return 0;
			}
		}

		dev_err(&ts->spi->dev, "unsupported HX8310%x\n", id[1] & 0xf);
		return -ENODEV;
	}

	dev_err(&ts->spi->dev, "unexpected chip ID %*ph\n", (int)sizeof(id), id);

	return -ENODEV;
}

/* ---------------------------------------------------------------------- */
/* Firmware image parsing                                                 */
/* ---------------------------------------------------------------------- */

/* CRC-32C over little endian words, no final inversion, as the hardware does */
static u32 hx_crc32(const u8 *data, size_t len)
{
	u32 crc = 0xffffffff;
	size_t i;
	int bit;

	for (i = 0; i + 4 <= len; i += 4) {
		crc ^= get_unaligned_le32(&data[i]);
		for (bit = 0; bit < 32; bit++)
			crc = (crc >> 1) ^ ((crc & 1) ? HX_CRC_POLY : 0);
	}

	return crc;
}

struct hx_fw_part {
	u32 sram;
	u32 len;
	u32 offset;
};

static void hx_fw_read_part(const u8 *entry, struct hx_fw_part *part)
{
	part->sram = get_unaligned_le32(&entry[0]);
	part->len = get_unaligned_le32(&entry[4]);
	part->offset = get_unaligned_le32(&entry[8]);
}

static bool hx_fw_part_valid(const struct firmware *fw, const struct hx_fw_part *part)
{
	return part->offset <= fw->size && part->len <= fw->size - part->offset;
}

/*
 * Images larger than 64 KiB carry a partition table: entry 0 is the code,
 * the others are pieces of the configuration, which are assembled into one
 * DSRAM block and written in a single go (vendor's himax_zf_part_info()).
 */
static int hx_fw_parse_partitions(struct hx_ts *ts)
{
	const struct firmware *fw = ts->fw;
	const u8 *table = &fw->data[HX_FW_CFG_TABLE];
	struct hx_fw_part main, part, max_part = { };
	u32 base = U32_MAX, max = 0, len, off;
	unsigned int num, i;

	num = table[12];
	if (!num || fw->size - HX_FW_CFG_TABLE < num * HX_FW_PART_ENTRY_LEN) {
		dev_err(&ts->spi->dev, "bad partition count %u\n", num);
		return -EINVAL;
	}

	hx_fw_read_part(table, &main);
	if (!hx_fw_part_valid(fw, &main) || !main.len) {
		dev_err(&ts->spi->dev, "bad code partition (0x%x + 0x%x)\n",
			main.offset, main.len);
		return -EINVAL;
	}

	ts->main_sram = main.sram;
	ts->main_offset = main.offset;
	ts->main_len = main.len;
	ts->fw_partitioned = true;

	dev_dbg(&ts->spi->dev, "[0] SRAM 0x%08x size 0x%x BIN 0x%x\n",
		main.sram, main.len, main.offset);

	if (num == 1)
		return 0;

	for (i = 1; i < num; i++) {
		hx_fw_read_part(&table[i * HX_FW_PART_ENTRY_LEN], &part);
		if (part.sram < base) {
			base = part.sram;
			ts->cfg_sram = part.sram;
		}
		if (part.sram > max) {
			max = part.sram;
			max_part = part;
		}
	}

	len = max - base + max_part.len;
	len = ALIGN(len, 16);
	if (len > HX_DSRAM_SIZE) {
		dev_err(&ts->spi->dev, "configuration too large (%u bytes)\n", len);
		return -EINVAL;
	}

	ts->cfg_buf = kzalloc(len, GFP_KERNEL);
	if (!ts->cfg_buf)
		return -ENOMEM;

	for (i = 1; i < num; i++) {
		hx_fw_read_part(&table[i * HX_FW_PART_ENTRY_LEN], &part);
		/* The vendor aligns each destination down to a word */
		off = (part.sram & ~3U) - base;

		dev_dbg(&ts->spi->dev, "[%u] SRAM 0x%08x size 0x%x BIN 0x%x\n",
			i, part.sram & ~3U, part.len, part.offset);

		if ((part.sram & ~3U) < base || off > len || part.len > len - off ||
		    !hx_fw_part_valid(fw, &part)) {
			dev_err(&ts->spi->dev, "bad configuration partition %u\n", i);
			kfree(ts->cfg_buf);
			ts->cfg_buf = NULL;
			return -EINVAL;
		}

		memcpy(&ts->cfg_buf[off], &fw->data[part.offset], part.len);
	}

	ts->cfg_len = len;
	ts->cfg_crc = hx_crc32(ts->cfg_buf, len);

	return 0;
}

static int hx_fw_parse(struct hx_ts *ts)
{
	const struct firmware *fw = ts->fw;
	int ret;

	ts->fw_partitioned = false;

	if (fw->size > HX_FW_CFG_TABLE) {
		ret = hx_fw_parse_partitions(ts);
		if (ret)
			return ret;
	} else if (fw->size < HX_FW_FIXED_MIN_LEN) {
		dev_err(&ts->spi->dev, "firmware image too small (%zu bytes)\n", fw->size);
		return -EINVAL;
	}

	dev_info(&ts->spi->dev, "firmware %s: %zu bytes, %s layout\n", ts->fw_name,
		 fw->size, ts->fw_partitioned ? "partitioned" : "fixed");

	return 0;
}

static void hx_fw_release(struct hx_ts *ts)
{
	kfree(ts->cfg_buf);
	ts->cfg_buf = NULL;
	ts->cfg_len = 0;
	release_firmware(ts->fw);
	ts->fw = NULL;
}

/* ---------------------------------------------------------------------- */
/* Firmware download                                                      */
/* ---------------------------------------------------------------------- */

/* Let the hardware compute the CRC of a block of SRAM */
static int hx_hw_crc(struct hx_ts *ts, u32 addr, u32 len, u32 *crc)
{
	int retry, ret;
	u32 val;

	ret = hx_reg_write32(ts, HX_ADDR_RELOAD_FROM, addr);
	if (ret)
		return ret;

	ret = hx_reg_write32(ts, HX_ADDR_RELOAD_CMD, HX_RELOAD_CMD_CRC | ((len / 4) & 0xffff));
	if (ret)
		return ret;

	for (retry = 0; retry < 100; retry++) {
		ret = hx_reg_read32(ts, HX_ADDR_RELOAD_STATUS, &val);
		if (ret)
			return ret;

		if (!(val & BIT(0)))
			return hx_reg_read32(ts, HX_ADDR_RELOAD_CRC32, crc);

		usleep_range(1000, 1100);
	}

	return -ETIMEDOUT;
}

/* Write a block whose CRC, including its embedded checksum, must be zero */
static int hx_write_sram_crc(struct hx_ts *ts, u32 addr, const u8 *data, u32 len)
{
	int retry, ret;
	u32 crc = 0;

	for (retry = 0; retry < 3; retry++) {
		ret = hx_reg_write(ts, addr, data, len);
		if (ret)
			return ret;

		ret = hx_hw_crc(ts, addr, len, &crc);
		if (ret)
			return ret;
		if (!crc)
			return 0;
	}

	dev_err(&ts->spi->dev, "HW CRC of 0x%08x + 0x%x is %08x\n", addr, len, crc);

	return -EIO;
}

static int hx_write_partitions(struct hx_ts *ts)
{
	int retry, ret;
	u32 crc = 0;

	ret = hx_write_sram_crc(ts, ts->main_sram, &ts->fw->data[ts->main_offset],
				ts->main_len);
	if (ret)
		return ret;

	if (!ts->cfg_buf)
		return 0;

	for (retry = 0; retry < 4; retry++) {
		ret = hx_reg_write(ts, ts->cfg_sram, ts->cfg_buf, ts->cfg_len);
		if (ret)
			return ret;

		ret = hx_hw_crc(ts, ts->cfg_sram, ts->cfg_len, &crc);
		if (ret)
			return ret;
		if (crc == ts->cfg_crc)
			return 0;
	}

	dev_err(&ts->spi->dev, "configuration CRC mismatch: HW %08x, SW %08x\n",
		crc, ts->cfg_crc);

	return -EIO;
}

/* Images of up to 64 KiB use a fixed layout (vendor's himax_mcu_firmware_update_0f()) */
static int hx_write_fixed(struct hx_ts *ts)
{
	const struct hx_chip_data *chip = ts->chip;
	const struct {
		u32 sram;
		u32 offset;
		u32 len;
	} parts[] = {
		{ 0x10007000,		0xc000, 128 },	/* config info */
		{ 0x10007084,		0xc0fe, 528 },	/* FW config 1 */
		{ 0x10007300,		0xca00, 128 },	/* FW config 3 */
		{ chip->adc_cfg_1,	0xd640, 1200 },	/* ADC config 1 */
		{ chip->adc_cfg_2,	0xd320, 800 },	/* ADC config 2 */
		{ 0x10007500,		0xe000, 1536 },	/* mapping table */
	};
	const u8 *data = ts->fw->data;
	unsigned int i;
	int ret;

	ret = hx_write_sram_crc(ts, chip->sram_start, data, HX_FW_MAIN_LEN);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(parts); i++) {
		ret = hx_write_sram_crc(ts, parts[i].sram, &data[parts[i].offset],
					parts[i].len);
		/* The vendor only logs CRC errors of the configuration blocks */
		if (ret == -EIO)
			continue;
		if (ret)
			return ret;
	}

	return 0;
}

static void hx_wait_reload_done(struct hx_ts *ts)
{
	int retry;
	u32 val = 0;

	for (retry = 0; retry < 30; retry++) {
		if (!hx_reg_read32(ts, HX_ADDR_RELOAD_DONE, &val) && val == HX_RELOAD_DONE)
			return;

		usleep_range(10000, 11000);
	}

	dev_warn(&ts->spi->dev, "firmware did not report reload done (%08x)\n", val);
}

static int hx_download_firmware_once(struct hx_ts *ts)
{
	int ret;

	ret = hx_reg_write32(ts, HX_ADDR_SYSTEM_RESET, HX_DATA_SYSTEM_RESET);
	if (ret)
		return ret;

	/* The vendor carries on regardless; the CRC checks below catch failures */
	ret = hx_sense_off(ts, &ts->chip->sense_off);
	if (ret && ret != -ETIMEDOUT)
		return ret;

	if (ts->fw_partitioned)
		ret = hx_write_partitions(ts);
	else
		ret = hx_write_fixed(ts);
	if (ret)
		return ret;

	/* Reset the frame count back to 1 for normal mode */
	ret = hx_reg_write32(ts, HX_ADDR_SET_FRAME, 1);
	if (ret)
		return ret;

	/* There is no flash to reload from */
	ret = hx_reg_write32(ts, HX_ADDR_FLASH_RELOAD, HX_DATA_DIS_FLASH_RELOAD);
	if (ret)
		return ret;

	ret = hx_reg_write32(ts, HX_ADDR_RELOAD_DONE, 0);
	if (ret)
		return ret;

	ret = hx_sense_on(ts);
	if (ret)
		return ret;

	hx_wait_reload_done(ts);

	return hx_reload_to_active(ts);
}

static int hx_download_firmware(struct hx_ts *ts)
{
	int retry, ret;

	for (retry = 0; retry < 3; retry++) {
		ret = hx_download_firmware_once(ts);
		if (!ret)
			return 0;

		dev_warn(&ts->spi->dev, "firmware download attempt %d failed: %d\n",
			 retry + 1, ret);
	}

	return ret;
}

/* ---------------------------------------------------------------------- */
/* Start / stop                                                           */
/* ---------------------------------------------------------------------- */

static int hx_read_event(struct hx_ts *ts, u8 *data, size_t len)
{
	int ret;

	if (ts->chip->event_burst_off) {
		ret = hx_bus_write_u8(ts, HX_REG_AHB_BURST, 0x00);
		if (ret)
			return ret;
	}

	ret = hx_bus_read(ts, HX_REG_EVENT, data, len);

	if (ts->chip->event_burst_off && !ret)
		ret = hx_bus_write_u8(ts, HX_REG_AHB_BURST, 0x01);

	return ret;
}

static void hx_release_fingers(struct hx_ts *ts)
{
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static void hx_fw_retry(struct hx_ts *ts, const char *what, int err)
{
	if (++ts->fw_tries >= HX_FW_LOAD_TRIES) {
		dev_err(&ts->spi->dev, "%s failed (%d), giving up\n", what, err);
		return;
	}

	if (ts->fw_tries == 1)
		dev_warn(&ts->spi->dev, "%s failed (%d), will retry\n", what, err);

	schedule_delayed_work(&ts->fw_work, msecs_to_jiffies(HX_FW_RETRY_MS));
}

static void hx_log_fw_version(struct hx_ts *ts)
{
	const struct firmware *fw = ts->fw;
	u32 fw_off = ts->chip->fw_ver_offset;
	u32 cfg_off = ts->chip->cfg_ver_offset;

	if (fw->size <= fw_off + 1 || fw->size <= cfg_off + 1)
		return;

	dev_info(&ts->spi->dev, "firmware version %02x%02x, config version %02x%02x\n",
		 fw->data[fw_off], fw->data[fw_off + 1],
		 fw->data[cfg_off], fw->data[cfg_off + 1]);
}

static void hx_fw_work(struct work_struct *work)
{
	struct hx_ts *ts = container_of(work, struct hx_ts, fw_work.work);
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
			hx_fw_retry(ts, "loading the firmware image", ret);
			return;
		}

		ret = hx_fw_parse(ts);
		if (ret) {
			hx_fw_release(ts);
			return;
		}
	}

	if (!ts->chip) {
		ret = hx_identify_chip(ts);
		if (ret) {
			hx_fw_retry(ts, "identifying the controller", ret);
			return;
		}
		hx_log_fw_version(ts);
	}

	start = ktime_get();
	ret = hx_download_firmware(ts);
	if (ret) {
		hx_fw_retry(ts, "firmware download", ret);
		return;
	}

	dev_info(&ts->spi->dev, "firmware downloaded in %lld ms\n",
		 ktime_ms_delta(ktime_get(), start));

	ts->fw_tries = 0;
	ts->ed_count = 0;
	ts->running = true;
	enable_irq(ts->spi->irq);

	/*
	 * The interrupt is edge triggered: if the controller raised it while
	 * it was still disabled, read the pending report so the line rises.
	 */
	hx_read_event(ts, ts->scratch, HX_TOUCH_INFO_LEN);
}

static void hx_start(struct hx_ts *ts, unsigned int delay_ms)
{
	scoped_guard(mutex, &ts->lock) {
		ts->enabled = true;
		ts->fw_tries = 0;
	}

	schedule_delayed_work(&ts->fw_work, msecs_to_jiffies(delay_ms));
}

static void hx_stop(struct hx_ts *ts)
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
	hx_release_fingers(ts);
}

/* ---------------------------------------------------------------------- */
/* Touch reports                                                          */
/* ---------------------------------------------------------------------- */

static void hx_reload_firmware(struct hx_ts *ts)
{
	dev_warn(&ts->spi->dev, "controller reported an exception, reloading firmware\n");
	ts->running = false;
	disable_irq_nosync(ts->spi->irq);
	hx_release_fingers(ts);
	ts->fw_tries = 0;
	ts->ed_count = 0;
	schedule_delayed_work(&ts->fw_work, 0);
}

static bool hx_all_bytes(const u8 *data, size_t len, u8 val)
{
	size_t i;

	for (i = 0; i < len; i++) {
		if (data[i] != val)
			return false;
	}

	return true;
}

/* A report with a bad checksum may be an exception event (vendor HX_EXCP_RECOVERY) */
static void hx_check_exception(struct hx_ts *ts, const u8 *data)
{
	if (hx_all_bytes(data, HX_TOUCH_INFO_LEN, 0xeb) ||
	    hx_all_bytes(data, HX_TOUCH_INFO_LEN, 0xec) ||
	    hx_all_bytes(data, HX_TOUCH_INFO_LEN, 0xee)) {
		hx_reload_firmware(ts);
	} else if (hx_all_bytes(data, HX_TOUCH_INFO_LEN, 0xed)) {
		/* Sent by the hardware after a reset of the touch MCU */
		hx_reload_to_active(ts);
		if (ts->ed_count++ > HX_EXCP_ED_COUNT)
			hx_reload_firmware(ts);
	} else {
		ts->ed_count = 0;
	}
}

static irqreturn_t hx_irq_thread(int irq, void *dev_id)
{
	struct hx_ts *ts = dev_id;
	u8 *data = ts->scratch;
	unsigned int i, points, raw_sel;
	u8 checksum = 0;
	int ret;

	guard(mutex)(&ts->lock);

	if (!ts->running)
		return IRQ_HANDLED;

	ret = hx_read_event(ts, data, HX_TOUCH_INFO_LEN);
	if (ret) {
		dev_err_ratelimited(&ts->spi->dev, "failed to read touch data: %d\n", ret);
		return IRQ_HANDLED;
	}

	for (i = 0; i < HX_TOUCH_INFO_LEN; i++)
		checksum += data[i];

	if (checksum || hx_all_bytes(data, HX_TOUCH_INFO_LEN, 0x00)) {
		hx_check_exception(ts, data);
		return IRQ_HANDLED;
	}
	ts->ed_count = 0;

	/* Diagnostic data is selected: drain the event stack and drop it */
	raw_sel = data[HX_POINT_CNT_POS] >> 4;
	if (raw_sel != 0x0f && raw_sel != 0x00) {
		hx_read_event(ts, data, HX_EVENT_STACK_LEN - HX_TOUCH_INFO_LEN);
		return IRQ_HANDLED;
	}

	points = data[HX_POINT_CNT_POS] == 0xff ? 0 : data[HX_POINT_CNT_POS] & 0x0f;

	for (i = 0; points && i < HX_MAX_FINGERS; i++) {
		unsigned int x = get_unaligned_be16(&data[i * 4]);
		unsigned int y = get_unaligned_be16(&data[i * 4 + 2]);

		/* Unused entries are 0xffff */
		if (x >= HX_X_RES || y >= HX_Y_RES)
			continue;

		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, data[HX_COORD_LEN + i]);
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

	return IRQ_HANDLED;
}

/* ---------------------------------------------------------------------- */
/* Panel follower and power management                                    */
/* ---------------------------------------------------------------------- */

static int hx_panel_prepared(struct drm_panel_follower *follower)
{
	struct hx_ts *ts = container_of(follower, struct hx_ts, follower);

	/* The panel reset wiped the touch firmware; load it once the panel is up */
	hx_start(ts, HX_FW_PANEL_DELAY_MS);

	return 0;
}

static int hx_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct hx_ts *ts = container_of(follower, struct hx_ts, follower);

	hx_stop(ts);

	return 0;
}

static const struct drm_panel_follower_funcs hx_panel_follower_funcs = {
	.panel_prepared = hx_panel_prepared,
	.panel_unpreparing = hx_panel_unpreparing,
};

static int hx_suspend(struct device *dev)
{
	struct hx_ts *ts = dev_get_drvdata(dev);

	if (!ts->is_follower)
		hx_stop(ts);

	return 0;
}

static int hx_resume(struct device *dev)
{
	struct hx_ts *ts = dev_get_drvdata(dev);

	if (!ts->is_follower)
		hx_start(ts, HX_FW_PANEL_DELAY_MS);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(hx_pm_ops, hx_suspend, hx_resume);

/* ---------------------------------------------------------------------- */
/* Probe                                                                  */
/* ---------------------------------------------------------------------- */

static void hx_cleanup(void *data)
{
	struct hx_ts *ts = data;

	hx_stop(ts);
	hx_fw_release(ts);
}

/*
 * The touch node is shared with the other controllers this tablet was built
 * with; the bootloader names the fitted display (and with it the touch
 * controller) on the command line: msm_drm.dsi_display0=qcom,mdss_dsi_<panel>_video.
 * Returns the panel name ("hx83102e_txd_auo_video") or NULL if not a Himax one.
 */
static const char *hx_find_panel(size_t *len)
{
	static const char key[] = "mdss_dsi_hx83102";
	const char *args, *s;

	if (panel && *panel) {
		*len = strlen(panel);
		return strstr(panel, "hx83102") ? panel : NULL;
	}

	if (!of_chosen || of_property_read_string(of_chosen, "bootargs", &args))
		return NULL;

	s = strstr(args, key);
	if (!s)
		return NULL;

	s += sizeof("mdss_dsi_") - 1;
	*len = strcspn(s, ": ");

	return s;
}

/* Same names, searched in the same order, as the vendor's himax_panel_info[] */
static const char *hx_fw_name_for_panel(struct device *dev, const char *name, size_t len)
{
	static const char * const panels[] = {
		"inx_fhd", "txd_inx", "hlt_auo", "txd_auo_al", "txd_auo",
		"lide_hsd", "tianma_tianma", "djn_jdi",
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(panels); i++) {
		if (strnstr(name, panels[i], len))
			return devm_kasprintf(dev, GFP_KERNEL, "Himax_firmware_%s.bin",
					      panels[i]);
	}

	dev_warn(dev, "unknown panel %.*s\n", (int)len, name);

	return "Himax_firmware.bin";
}

static int hx_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct input_dev *input;
	const char *panel_name;
	size_t panel_len;
	struct hx_ts *ts;
	int ret;

	/* Leave the hardware alone unless this tablet has a Himax panel */
	panel_name = hx_find_panel(&panel_len);
	if (!panel_name)
		return -ENODEV;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;

	if (!spi->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	ts->tx_buf = devm_kmalloc(dev, HX_TX_BUF_LEN, GFP_KERNEL);
	ts->rx_buf = devm_kmalloc(dev, HX_MAX_READ_LEN, GFP_KERNEL);
	if (!ts->tx_buf || !ts->rx_buf)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &ts->lock);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&ts->fw_work, hx_fw_work);
	spi_set_drvdata(spi, ts);

	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "SPI setup failed\n");

	ts->fw_name = hx_fw_name_for_panel(dev, panel_name, panel_len);
	if (!ts->fw_name)
		return -ENOMEM;

	dev_info(dev, "panel %.*s, firmware %s\n", (int)panel_len, panel_name, ts->fw_name);

	/* Deasserted: the controller is supposed to be running */
	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset GPIO\n");

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	ts->input = input;
	input->name = "Himax HX83102E Touchscreen";
	input->id.bustype = BUS_SPI;

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, HX_X_RES - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, HX_Y_RES - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	touchscreen_parse_properties(input, true, &ts->prop);

	ret = input_mt_init_slots(input, HX_MAX_FINGERS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	/*
	 * The vendor driver always uses a falling edge, whatever the DT says;
	 * the node is shared with controllers that want a different trigger.
	 */
	ret = devm_request_threaded_irq(dev, spi->irq, NULL, hx_irq_thread,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
					IRQF_NO_AUTOEN,
					dev_name(dev), ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request interrupt\n");

	ret = input_register_device(input);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, hx_cleanup, ts);
	if (ret)
		return ret;

	ts->is_follower = drm_is_panel_follower(dev);
	if (ts->is_follower) {
		ts->follower.funcs = &hx_panel_follower_funcs;
		ret = devm_drm_panel_add_follower(dev, &ts->follower);
		if (ret)
			return dev_err_probe(dev, ret, "failed to follow the panel\n");
	} else {
		dev_warn(dev, "no panel to follow, assuming the display is already on\n");
		hx_start(ts, 0);
	}

	return 0;
}

static const struct of_device_id hx_of_match[] = {
	{ .compatible = "himax,hx83102e-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, hx_of_match);

static const struct spi_device_id hx_spi_ids[] = {
	{ "hx83102e-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, hx_spi_ids);

static struct spi_driver hx_spi_driver = {
	.driver = {
		.name = "himax-hx83102e-spi",
		.of_match_table = hx_of_match,
		.pm = pm_sleep_ptr(&hx_pm_ops),
	},
	.probe = hx_probe,
	.id_table = hx_spi_ids,
};
module_spi_driver(hx_spi_driver);

MODULE_DESCRIPTION("Himax HX83102E SPI touchscreen driver");
MODULE_LICENSE("GPL");
