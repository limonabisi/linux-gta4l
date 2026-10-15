// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT36523 SPI touchscreen driver (TDDI, host firmware download)
 *
 * The touch controller shares its silicon with the display driver and has
 * no flash of its own: the host has to load the touch firmware into the
 * controller's RAM over SPI every time the panel has been reset.
 *
 * Protocol and register layout taken from the downstream "nt36xxx" driver,
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
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
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <drm/drm_panel.h>

#define NVT_SPI_WRITE_MASK		0x80
#define NVT_SPI_READ_MASK		0x7f
#define NVT_SPI_DUMMY_BYTES		1
#define NVT_REG_PAGE			0xff

/* Largest payload of a single firmware write, as used by the vendor */
#define NVT_TRANSFER_LEN		(63 * 1024)
#define NVT_XFER_BUF_LEN		(NVT_TRANSFER_LEN + 1 + NVT_SPI_DUMMY_BYTES)

#define NVT_ENG_RST_ADDR		0x7fff80
#define NVT_CHIP_VER_TRIM_ADDR		0x3f004
#define NVT_ID_LEN			6

/* Offsets inside the event buffer */
#define NVT_EVENT_HOST_CMD		0x50
#define NVT_EVENT_RESET_COMPLETE	0x60
#define NVT_EVENT_FWINFO		0x78
#define NVT_EVENT_PROJECTID		0x9a

#define NVT_CMD_FW_CRC_ENABLE		0xae
#define NVT_CMD_DEEP_SLEEP		0x11

#define NVT_RESET_STATE_INIT		0xa0
#define NVT_RESET_STATE_REK		0xa1
#define NVT_RESET_STATE_MAX		0xaf

#define NVT_MAX_FINGERS			10
#define NVT_FINGER_LEN			6
#define NVT_POINT_DATA_LEN		65
#define NVT_MAX_PRESSURE		1000
#define NVT_WDT_RECOVERY_COUNT		10

#define NVT_FLASH_SECTOR_SIZE		4096
#define NVT_FW_HEADER_MIN		0x30
#define NVT_FW_ILM_DLM_NUM		2
#define NVT_FW_MAX_PARTITIONS		64

#define NVT_FW_LOAD_TRIES		20
#define NVT_FW_RETRY_MS			3000
#define NVT_FW_PANEL_DELAY_MS		100

struct nvt_mem_map {
	u32 event_buf_addr;
	u32 boot_rdy_addr;
	u32 tx_auto_copy_en;
	u32 spi_dma_tx_info;
	u32 ilm_length_addr;
	u32 dlm_length_addr;
	u32 ilm_des_addr;
	u32 dlm_des_addr;
	u32 g_ilm_checksum_addr;
	u32 g_dlm_checksum_addr;
	u32 r_ilm_checksum_addr;
	u32 r_dlm_checksum_addr;
	u32 bld_crc_en_addr;
	u32 bld_ilm_dlm_crc_addr;
};

struct nvt_chip_data {
	const char *name;
	const struct nvt_mem_map *mmap;
	u32 swrst_n8_addr;
	u32 spi_rd_fast_addr;
	/* Chip ID as read from NVT_CHIP_VER_TRIM_ADDR, compared where mask is set */
	u8 id[NVT_ID_LEN];
	u8 id_mask[NVT_ID_LEN];
	unsigned int max_x;
	unsigned int max_y;
	const char *default_fw_name;
};

struct nvt_bin_map {
	char name[12];
	u32 bin_addr;
	u32 sram_addr;
	u32 size;
	u32 crc;
};

struct nvt_ts {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	const struct nvt_chip_data *chip;

	/* Serialises all SPI traffic and the firmware state below */
	struct mutex lock;
	u8 *tx_buf;
	u8 *rx_buf;

	struct drm_panel_follower follower;
	bool is_follower;
	struct delayed_work fw_work;

	const char *fw_name;
	const struct firmware *fw;
	struct nvt_bin_map *bin_map;
	unsigned int num_partitions;
	bool fw_cascade;
	unsigned int fw_tries;

	bool chip_identified;
	/* The panel is up and the controller is supposed to be working */
	bool enabled;
	/* Firmware is loaded and the interrupt is enabled */
	bool running;
	unsigned int wdt_count;

	u8 scratch[NVT_POINT_DATA_LEN];
	u8 fw_ver;
	u16 fw_max_x;
	u16 fw_max_y;
};

/* ---------------------------------------------------------------------- */
/* SPI access                                                             */
/* ---------------------------------------------------------------------- */

static int nvt_spi_write(struct nvt_ts *ts, u8 reg, const u8 *data, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->tx_buf,
		.len = len + 1,
	};

	if (len > NVT_TRANSFER_LEN)
		return -EINVAL;

	ts->tx_buf[0] = reg | NVT_SPI_WRITE_MASK;
	memcpy(&ts->tx_buf[1], data, len);

	return spi_sync_transfer(ts->spi, &xfer, 1);
}

static int nvt_spi_read(struct nvt_ts *ts, u8 reg, u8 *data, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->tx_buf,
		.rx_buf = ts->rx_buf,
		.len = len + 1 + NVT_SPI_DUMMY_BYTES,
	};
	int ret;

	if (len > NVT_TRANSFER_LEN)
		return -EINVAL;

	memset(ts->tx_buf, 0, xfer.len);
	ts->tx_buf[0] = reg & NVT_SPI_READ_MASK;

	ret = spi_sync_transfer(ts->spi, &xfer, 1);
	if (ret)
		return ret;

	memcpy(data, &ts->rx_buf[1 + NVT_SPI_DUMMY_BYTES], len);

	return 0;
}

static int nvt_set_page(struct nvt_ts *ts, u32 addr)
{
	u8 buf[2] = { (addr >> 15) & 0xff, (addr >> 7) & 0xff };

	return nvt_spi_write(ts, NVT_REG_PAGE, buf, sizeof(buf));
}

static int nvt_write_addr(struct nvt_ts *ts, u32 addr, const u8 *data, size_t len)
{
	int ret;

	ret = nvt_set_page(ts, addr);
	if (ret)
		return ret;

	return nvt_spi_write(ts, addr & 0x7f, data, len);
}

static int nvt_write_addr_u8(struct nvt_ts *ts, u32 addr, u8 val)
{
	return nvt_write_addr(ts, addr, &val, 1);
}

static int nvt_read_addr(struct nvt_ts *ts, u32 addr, u8 *data, size_t len)
{
	int ret;

	ret = nvt_set_page(ts, addr);
	if (ret)
		return ret;

	return nvt_spi_read(ts, addr & 0x7f, data, len);
}

/* ---------------------------------------------------------------------- */
/* Controller state                                                       */
/* ---------------------------------------------------------------------- */

static void nvt_eng_reset(struct nvt_ts *ts)
{
	nvt_write_addr_u8(ts, NVT_ENG_RST_ADDR, 0x5a);
	usleep_range(1000, 2000);
}

/* Reset the touch MCU and leave it waiting in the boot loader */
static int nvt_bootloader_reset(struct nvt_ts *ts)
{
	int ret;

	ret = nvt_write_addr_u8(ts, ts->chip->swrst_n8_addr, 0x69);
	if (ret)
		return ret;

	usleep_range(5000, 6000);

	if (ts->chip->spi_rd_fast_addr)
		ret = nvt_write_addr_u8(ts, ts->chip->spi_rd_fast_addr, 0x00);

	return ret;
}

static int nvt_identify_chip(struct nvt_ts *ts)
{
	const struct nvt_chip_data *chip = ts->chip;
	u8 id[NVT_ID_LEN] = { };
	int retry, i, ret;

	nvt_eng_reset(ts);
	msleep(10);

	for (retry = 0; retry < 5; retry++) {
		ret = nvt_bootloader_reset(ts);
		if (!ret)
			ret = nvt_read_addr(ts, NVT_CHIP_VER_TRIM_ADDR, id, sizeof(id));
		if (ret) {
			dev_err(&ts->spi->dev, "chip ID read failed: %d\n", ret);
			msleep(10);
			continue;
		}

		for (i = 0; i < NVT_ID_LEN; i++) {
			if (chip->id_mask[i] && id[i] != chip->id[i])
				break;
		}
		if (i == NVT_ID_LEN) {
			dev_info(&ts->spi->dev, "found %s, chip ID %*ph\n",
				 chip->name, NVT_ID_LEN, id);
			return 0;
		}

		msleep(10);
	}

	dev_err(&ts->spi->dev, "unexpected chip ID %*ph (wanted %*ph)\n",
		NVT_ID_LEN, id, NVT_ID_LEN, chip->id);

	return ret ?: -ENODEV;
}

static int nvt_check_fw_reset_state(struct nvt_ts *ts, u8 state)
{
	u32 addr = ts->chip->mmap->event_buf_addr | NVT_EVENT_RESET_COMPLETE;
	int retries = state == NVT_RESET_STATE_INIT ? 10 : 50;
	u8 buf[5] = { };
	int ret;

	do {
		ret = nvt_read_addr(ts, addr, buf, sizeof(buf));
		if (ret)
			return ret;

		if (buf[0] >= state && buf[0] <= NVT_RESET_STATE_MAX)
			return 0;

		usleep_range(10000, 11000);
	} while (retries--);

	dev_err(&ts->spi->dev, "firmware did not start, reset state %*ph\n",
		(int)sizeof(buf), buf);

	return -ETIMEDOUT;
}

static int nvt_get_fw_info(struct nvt_ts *ts)
{
	const struct nvt_mem_map *mmap = ts->chip->mmap;
	u8 buf[38];
	u8 pid[2] = { };
	int retry, ret;

	for (retry = 0; retry < 4; retry++) {
		ret = nvt_read_addr(ts, mmap->event_buf_addr | NVT_EVENT_FWINFO,
				    buf, sizeof(buf));
		if (ret)
			return ret;

		/* Version byte is followed by its complement */
		if ((u8)(buf[0] + buf[1]) == 0xff)
			break;

		ret = -EIO;
	}

	if (ret) {
		dev_warn(&ts->spi->dev, "firmware info is broken (%02x %02x)\n",
			 buf[0], buf[1]);
		ts->fw_ver = 0;
		ts->fw_max_x = ts->chip->max_x;
		ts->fw_max_y = ts->chip->max_y;
	} else {
		ts->fw_ver = buf[0];
		ts->fw_max_x = get_unaligned_be16(&buf[4]);
		ts->fw_max_y = get_unaligned_be16(&buf[6]);
		if (!ts->fw_max_x || !ts->fw_max_y) {
			ts->fw_max_x = ts->chip->max_x;
			ts->fw_max_y = ts->chip->max_y;
		}
	}

	nvt_read_addr(ts, mmap->event_buf_addr | NVT_EVENT_PROJECTID, pid, sizeof(pid));

	dev_info(&ts->spi->dev,
		 "firmware version 0x%02x, type 0x%02x, PID %04x, %ux%u sensors, resolution %ux%u\n",
		 ts->fw_ver, buf[13], get_unaligned_le16(pid), buf[2], buf[3],
		 ts->fw_max_x, ts->fw_max_y);

	return ret;
}

/* ---------------------------------------------------------------------- */
/* Firmware image parsing                                                 */
/* ---------------------------------------------------------------------- */

static int nvt_fw_need_write_size(const struct firmware *fw, size_t *size)
{
	size_t i;

	for (i = fw->size / NVT_FLASH_SECTOR_SIZE; i > 0; i--) {
		const u8 *flag = &fw->data[i * NVT_FLASH_SECTOR_SIZE - 3];

		if (!memcmp(flag, "NVT", 3) || !memcmp(flag, "MOD", 3)) {
			*size = i * NVT_FLASH_SECTOR_SIZE;
			return 0;
		}
	}

	return -EINVAL;
}

static int nvt_fw_read_entry(struct nvt_ts *ts, struct nvt_bin_map *map, u32 pos)
{
	const struct firmware *fw = ts->fw;

	if (pos > fw->size || fw->size - pos < 16)
		return -EINVAL;

	map->sram_addr = get_unaligned_le32(&fw->data[pos]);
	map->size = get_unaligned_le32(&fw->data[pos + 4]);
	map->bin_addr = get_unaligned_le32(&fw->data[pos + 8]);
	map->crc = get_unaligned_le32(&fw->data[pos + 12]);

	return 0;
}

static int nvt_fw_parse(struct nvt_ts *ts)
{
	const struct firmware *fw = ts->fw;
	const u8 *data = fw->data;
	struct nvt_bin_map *map;
	unsigned int info_sec_num = 0, ovly_sec_num, partitions, i;
	bool found_header = false;
	size_t need_size;
	u32 end, pos;
	int ret;

	if (fw->size < NVT_FLASH_SECTOR_SIZE)
		return -EINVAL;

	if (nvt_fw_need_write_size(fw, &need_size)) {
		dev_err(&ts->spi->dev, "firmware end flag not found\n");
		return -EINVAL;
	}

	/* Version byte and its complement live at the start of the last sector */
	pos = need_size - NVT_FLASH_SECTOR_SIZE;
	if ((u8)(data[pos] + data[pos + 1]) != 0xff) {
		dev_err(&ts->spi->dev, "firmware version check failed (%02x %02x)\n",
			data[pos], data[pos + 1]);
		return -ENOEXEC;
	}

	end = get_unaligned_le32(&data[0]);
	if (end < NVT_FW_HEADER_MIN || end > fw->size) {
		dev_err(&ts->spi->dev, "bad firmware header size 0x%x\n", end);
		return -EINVAL;
	}

	ts->fw_cascade = data[0x20] & BIT(1);

	if (ts->fw_cascade) {
		for (pos = 0x30; pos < end / 2; pos += 0x10)
			info_sec_num++;
		/* next header section */
		info_sec_num++;
	} else {
		for (pos = 0x30; pos < end; pos += 0x10)
			info_sec_num++;
	}

	ovly_sec_num = (data[0x28] & BIT(4)) ? data[0x28] & 0x0f : 0;

	partitions = NVT_FW_ILM_DLM_NUM + ovly_sec_num + info_sec_num;
	if (partitions > NVT_FW_MAX_PARTITIONS) {
		dev_err(&ts->spi->dev, "too many firmware partitions (%u)\n", partitions);
		return -EINVAL;
	}

	map = kcalloc(partitions, sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	pos = 0;
	for (i = 0; i < partitions; i++) {
		if (i < NVT_FW_ILM_DLM_NUM) {
			/* ILM and DLM: BIN address, SRAM address, size */
			map[i].bin_addr = get_unaligned_le32(&data[i * 12]);
			map[i].sram_addr = get_unaligned_le32(&data[4 + i * 12]);
			map[i].size = get_unaligned_le32(&data[8 + i * 12]);
			map[i].crc = get_unaligned_le32(&data[0x18 + i * 4]);
			strscpy(map[i].name, i ? "DLM" : "ILM");
		} else if (i < NVT_FW_ILM_DLM_NUM + info_sec_num) {
			/* Info sections: SRAM address, size, BIN address, CRC */
			if (!found_header)
				pos = 0x30 + 0x10 * (i - NVT_FW_ILM_DLM_NUM);
			else if (ts->fw_cascade)
				pos = end - 0x10;

			ret = nvt_fw_read_entry(ts, &map[i], pos);
			if (ret)
				goto err_free;

			if (map[i].bin_addr < end && map[i].size) {
				strscpy(map[i].name, "Header");
				found_header = true;
			} else {
				snprintf(map[i].name, sizeof(map[i].name), "Info-%u",
					 i - NVT_FW_ILM_DLM_NUM);
			}
		} else {
			/* Overlay sections are described at the start of the DLM */
			unsigned int n = i - NVT_FW_ILM_DLM_NUM - info_sec_num;

			ret = nvt_fw_read_entry(ts, &map[i], map[1].bin_addr + 0x10 * n);
			if (ret)
				goto err_free;

			snprintf(map[i].name, sizeof(map[i].name), "Overlay-%u", n);
		}

		if (map[i].bin_addr > fw->size ||
		    map[i].size > fw->size - map[i].bin_addr) {
			dev_err(&ts->spi->dev,
				"firmware partition %s (0x%x + 0x%x) is outside of the image\n",
				map[i].name, map[i].bin_addr, map[i].size);
			ret = -EINVAL;
			goto err_free;
		}

		dev_dbg(&ts->spi->dev, "[%u] %s: SRAM 0x%08x size 0x%08x BIN 0x%08x CRC 0x%08x\n",
			i, map[i].name, map[i].sram_addr, map[i].size,
			map[i].bin_addr, map[i].crc);
	}

	ts->bin_map = map;
	ts->num_partitions = partitions;

	dev_info(&ts->spi->dev, "firmware %s: version 0x%02x, %u partitions%s\n",
		 ts->fw_name, data[need_size - NVT_FLASH_SECTOR_SIZE], partitions,
		 ts->fw_cascade ? ", cascade" : "");

	return 0;

err_free:
	kfree(map);
	dev_err(&ts->spi->dev, "failed to parse firmware header: %d\n", ret);
	return ret;
}

static void nvt_fw_release(struct nvt_ts *ts)
{
	kfree(ts->bin_map);
	ts->bin_map = NULL;
	ts->num_partitions = 0;
	release_firmware(ts->fw);
	ts->fw = NULL;
}

/* ---------------------------------------------------------------------- */
/* Firmware download                                                      */
/* ---------------------------------------------------------------------- */

static int nvt_write_sram(struct nvt_ts *ts, u32 sram_addr, u32 bin_addr, u32 size)
{
	const struct firmware *fw = ts->fw;
	int ret;

	while (size) {
		u32 len = min_t(u32, size, NVT_TRANSFER_LEN);
		u32 avail = min_t(u32, len, fw->size - bin_addr);

		ret = nvt_set_page(ts, sram_addr);
		if (ret)
			return ret;

		ts->tx_buf[0] = (sram_addr & 0x7f) | NVT_SPI_WRITE_MASK;
		memcpy(&ts->tx_buf[1], &fw->data[bin_addr], avail);
		/* The vendor writes one byte past each partition; pad with zero */
		memset(&ts->tx_buf[1 + avail], 0, len - avail);

		ret = spi_write(ts->spi, ts->tx_buf, len + 1);
		if (ret)
			return ret;

		sram_addr += len;
		bin_addr += avail;
		size -= len;
	}

	return 0;
}

static int nvt_write_firmware(struct nvt_ts *ts)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ts->num_partitions; i++) {
		const struct nvt_bin_map *map = &ts->bin_map[i];

		/* Reserved partitions have no size */
		if (!map->size)
			continue;

		ret = nvt_write_sram(ts, map->sram_addr, map->bin_addr, map->size + 1);
		if (ret) {
			dev_err(&ts->spi->dev, "writing %s failed: %d\n", map->name, ret);
			return ret;
		}
	}

	return 0;
}

static int nvt_set_bld_crc_bank(struct nvt_ts *ts, u32 des_addr, u32 length_addr,
				u32 checksum_addr, const struct nvt_bin_map *map)
{
	u8 buf[4];
	int ret;

	/* All three registers share one page */
	put_unaligned_le24(map->sram_addr, buf);
	ret = nvt_write_addr(ts, des_addr, buf, 3);
	if (ret)
		return ret;

	buf[0] = map->size & 0xff;
	buf[1] = (map->size >> 8) & 0xff;
	buf[2] = (map->size >> 16) & 0x01;
	ret = nvt_spi_write(ts, length_addr & 0x7f, buf, 3);
	if (ret)
		return ret;

	put_unaligned_le32(map->crc, buf);

	return nvt_spi_write(ts, checksum_addr & 0x7f, buf, 4);
}

static int nvt_set_bld_hw_crc(struct nvt_ts *ts)
{
	const struct nvt_mem_map *mmap = ts->chip->mmap;
	int ret;

	ret = nvt_set_bld_crc_bank(ts, mmap->ilm_des_addr, mmap->ilm_length_addr,
				   mmap->g_ilm_checksum_addr, &ts->bin_map[0]);
	if (ret)
		return ret;

	return nvt_set_bld_crc_bank(ts, mmap->dlm_des_addr, mmap->dlm_length_addr,
				    mmap->g_dlm_checksum_addr, &ts->bin_map[1]);
}

static int nvt_wait_spi_dma_tx(struct nvt_ts *ts)
{
	int retries = 200;
	u8 val;
	int ret;

	while (retries--) {
		ret = nvt_read_addr(ts, ts->chip->mmap->spi_dma_tx_info, &val, 1);
		if (ret)
			return ret;
		if (!val)
			return 0;

		usleep_range(1000, 1100);
	}

	return -ETIMEDOUT;
}

static void nvt_dump_bld_crc(struct nvt_ts *ts)
{
	const struct nvt_mem_map *mmap = ts->chip->mmap;
	u8 flag = 0, g[4] = { }, r[4] = { };

	nvt_read_addr(ts, mmap->bld_ilm_dlm_crc_addr, &flag, 1);
	dev_err(&ts->spi->dev, "crc_done %u, ilm_crc_flag %u, dlm_crc_flag %u\n",
		(flag >> 2) & 1, flag & 1, (flag >> 1) & 1);

	nvt_read_addr(ts, mmap->g_ilm_checksum_addr, g, sizeof(g));
	nvt_read_addr(ts, mmap->r_ilm_checksum_addr, r, sizeof(r));
	dev_err(&ts->spi->dev, "ILM: image crc %08x, golden %08x, result %08x\n",
		ts->bin_map[0].crc, get_unaligned_le32(g), get_unaligned_le32(r));

	nvt_read_addr(ts, mmap->g_dlm_checksum_addr, g, sizeof(g));
	nvt_read_addr(ts, mmap->r_dlm_checksum_addr, r, sizeof(r));
	dev_err(&ts->spi->dev, "DLM: image crc %08x, golden %08x, result %08x\n",
		ts->bin_map[1].crc, get_unaligned_le32(g), get_unaligned_le32(r));
}

static int nvt_download_firmware_once(struct nvt_ts *ts)
{
	const struct nvt_mem_map *mmap = ts->chip->mmap;
	u8 val;
	int ret;

	ret = nvt_bootloader_reset(ts);
	if (ret)
		return ret;

	ret = nvt_set_bld_hw_crc(ts);
	if (ret)
		return ret;

	if (ts->fw_cascade) {
		ret = nvt_write_addr_u8(ts, mmap->tx_auto_copy_en, 0x69);
		if (ret)
			return ret;
	}

	ret = nvt_write_firmware(ts);
	if (ret)
		return ret;

	if (ts->fw_cascade) {
		ret = nvt_wait_spi_dma_tx(ts);
		if (ret) {
			dev_err(&ts->spi->dev, "SPI DMA TX did not finish: %d\n", ret);
			return ret;
		}
	}

	/* Enable the boot loader CRC check */
	ret = nvt_read_addr(ts, mmap->bld_crc_en_addr, &val, 1);
	if (ret)
		return ret;
	val |= BIT(7);
	ret = nvt_spi_write(ts, mmap->bld_crc_en_addr & 0x7f, &val, 1);
	if (ret)
		return ret;

	/* Clear the reset status and enable the firmware CRC check */
	ret = nvt_write_addr_u8(ts, mmap->event_buf_addr | NVT_EVENT_RESET_COMPLETE, 0x00);
	if (ret)
		return ret;
	val = NVT_CMD_FW_CRC_ENABLE;
	ret = nvt_spi_write(ts, NVT_EVENT_HOST_CMD, &val, 1);
	if (ret)
		return ret;

	/* Let the firmware run */
	ret = nvt_write_addr_u8(ts, mmap->boot_rdy_addr, 1);
	if (ret)
		return ret;
	usleep_range(5000, 6000);

	return nvt_check_fw_reset_state(ts, NVT_RESET_STATE_INIT);
}

static int nvt_download_firmware(struct nvt_ts *ts)
{
	int retry, ret;

	for (retry = 0; retry < 3; retry++) {
		ret = nvt_download_firmware_once(ts);
		if (!ret)
			return 0;

		dev_warn(&ts->spi->dev, "firmware download attempt %d failed: %d\n",
			 retry + 1, ret);
	}

	nvt_dump_bld_crc(ts);

	return ret;
}

/* ---------------------------------------------------------------------- */
/* Start / stop                                                           */
/* ---------------------------------------------------------------------- */

static void nvt_release_fingers(struct nvt_ts *ts)
{
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static void nvt_fw_retry(struct nvt_ts *ts, const char *what, int err)
{
	if (++ts->fw_tries >= NVT_FW_LOAD_TRIES) {
		dev_err(&ts->spi->dev, "%s failed (%d), giving up\n", what, err);
		return;
	}

	if (ts->fw_tries == 1)
		dev_warn(&ts->spi->dev, "%s failed (%d), will retry\n", what, err);

	schedule_delayed_work(&ts->fw_work, msecs_to_jiffies(NVT_FW_RETRY_MS));
}

static void nvt_fw_work(struct work_struct *work)
{
	struct nvt_ts *ts = container_of(work, struct nvt_ts, fw_work.work);
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
			nvt_fw_retry(ts, "loading the firmware image", ret);
			return;
		}

		ret = nvt_fw_parse(ts);
		if (ret) {
			nvt_fw_release(ts);
			return;
		}
	}

	if (!ts->chip_identified) {
		ret = nvt_identify_chip(ts);
		if (ret) {
			nvt_fw_retry(ts, "identifying the controller", ret);
			return;
		}
		ts->chip_identified = true;
	}

	start = ktime_get();
	ret = nvt_download_firmware(ts);
	if (ret) {
		nvt_fw_retry(ts, "firmware download", ret);
		return;
	}

	dev_info(&ts->spi->dev, "firmware downloaded in %lld ms\n",
		 ktime_ms_delta(ktime_get(), start));

	/* Give the firmware time to calibrate its baseline; not fatal */
	nvt_check_fw_reset_state(ts, NVT_RESET_STATE_REK);

	nvt_get_fw_info(ts);
	if (ts->fw_max_x != ts->prop.max_x + 1 || ts->fw_max_y != ts->prop.max_y + 1)
		dev_warn(&ts->spi->dev,
			 "firmware resolution %ux%u differs from the configured %ux%u\n",
			 ts->fw_max_x, ts->fw_max_y, ts->prop.max_x + 1, ts->prop.max_y + 1);

	/* Touch reports are read from the start of the event buffer */
	ret = nvt_set_page(ts, ts->chip->mmap->event_buf_addr);
	if (ret) {
		nvt_fw_retry(ts, "selecting the event buffer", ret);
		return;
	}

	ts->fw_tries = 0;
	ts->wdt_count = 0;
	ts->running = true;
	enable_irq(ts->spi->irq);

	/*
	 * The interrupt is edge triggered: if the controller raised it while
	 * it was still disabled, read the pending report so the line drops.
	 */
	nvt_spi_read(ts, 0x00, ts->scratch, sizeof(ts->scratch));
}

static void nvt_start(struct nvt_ts *ts, unsigned int delay_ms)
{
	scoped_guard(mutex, &ts->lock) {
		ts->enabled = true;
		ts->fw_tries = 0;
	}

	schedule_delayed_work(&ts->fw_work, msecs_to_jiffies(delay_ms));
}

static void nvt_stop(struct nvt_ts *ts)
{
	u8 cmd = NVT_CMD_DEEP_SLEEP;
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
	nvt_spi_write(ts, NVT_EVENT_HOST_CMD, &cmd, 1);
	msleep(20);
	nvt_release_fingers(ts);
}

/* ---------------------------------------------------------------------- */
/* Touch reports                                                          */
/* ---------------------------------------------------------------------- */

static bool nvt_is_pattern(const u8 *data, u8 a, u8 b)
{
	int i;

	for (i = 0; i < 6; i++) {
		if (data[i] != a && data[i] != b)
			return false;
	}

	return true;
}

static irqreturn_t nvt_irq_thread(int irq, void *dev_id)
{
	struct nvt_ts *ts = dev_id;
	u8 data[NVT_POINT_DATA_LEN];
	u8 checksum = 0;
	int i, ret;

	guard(mutex)(&ts->lock);

	if (!ts->running)
		return IRQ_HANDLED;

	ret = nvt_spi_read(ts, 0x00, data, sizeof(data));
	if (ret) {
		dev_err_ratelimited(&ts->spi->dev, "failed to read touch data: %d\n", ret);
		return IRQ_HANDLED;
	}

	/* The controller lost its firmware (ESD, watchdog): load it again */
	if (nvt_is_pattern(data, 0xfd, 0xfe)) {
		if (++ts->wdt_count > NVT_WDT_RECOVERY_COUNT) {
			dev_warn(&ts->spi->dev, "controller was reset, reloading firmware\n");
			ts->running = false;
			disable_irq_nosync(irq);
			nvt_release_fingers(ts);
			ts->fw_tries = 0;
			schedule_delayed_work(&ts->fw_work, 0);
		}
		return IRQ_HANDLED;
	}
	ts->wdt_count = 0;

	/* Firmware is busy recovering */
	if (nvt_is_pattern(data, 0x77, 0x77))
		return IRQ_HANDLED;

	for (i = 0; i < NVT_POINT_DATA_LEN - 1; i++)
		checksum += data[i];
	checksum = ~checksum + 1;
	if (checksum != data[NVT_POINT_DATA_LEN - 1]) {
		dev_dbg_ratelimited(&ts->spi->dev, "touch data checksum mismatch\n");
		return IRQ_HANDLED;
	}

	for (i = 0; i < NVT_MAX_FINGERS; i++) {
		const u8 *p = &data[i * NVT_FINGER_LEN];
		unsigned int id = p[0] >> 3;
		unsigned int status = p[0] & 0x07;
		unsigned int x, y, major, pressure;

		if (!id || id > NVT_MAX_FINGERS)
			continue;

		/* 1: finger down, 2: finger moving */
		if (status != 0x01 && status != 0x02)
			continue;

		x = (p[1] << 4) | (p[3] >> 4);
		y = (p[2] << 4) | (p[3] & 0x0f);
		if (x > ts->fw_max_x || y > ts->fw_max_y)
			continue;
		x = min(x, ts->fw_max_x - 1U);
		y = min(y, ts->fw_max_y - 1U);

		major = p[4] ?: 1;

		pressure = p[5];
		if (i < 2)
			pressure |= data[62 + i] << 8;
		pressure = clamp(pressure, 1U, NVT_MAX_PRESSURE);

		input_mt_slot(ts->input, id - 1);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, major);
		input_report_abs(ts->input, ABS_MT_PRESSURE, pressure);
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

	return IRQ_HANDLED;
}

/* ---------------------------------------------------------------------- */
/* Panel follower and power management                                    */
/* ---------------------------------------------------------------------- */

static int nvt_panel_prepared(struct drm_panel_follower *follower)
{
	struct nvt_ts *ts = container_of(follower, struct nvt_ts, follower);

	/* The panel reset wiped the touch firmware; load it once the panel is up */
	nvt_start(ts, NVT_FW_PANEL_DELAY_MS);

	return 0;
}

static int nvt_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct nvt_ts *ts = container_of(follower, struct nvt_ts, follower);

	nvt_stop(ts);

	return 0;
}

static const struct drm_panel_follower_funcs nvt_panel_follower_funcs = {
	.panel_prepared = nvt_panel_prepared,
	.panel_unpreparing = nvt_panel_unpreparing,
};

static int nvt_suspend(struct device *dev)
{
	struct nvt_ts *ts = dev_get_drvdata(dev);

	if (!ts->is_follower)
		nvt_stop(ts);

	return 0;
}

static int nvt_resume(struct device *dev)
{
	struct nvt_ts *ts = dev_get_drvdata(dev);

	if (!ts->is_follower)
		nvt_start(ts, NVT_FW_PANEL_DELAY_MS);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(nvt_pm_ops, nvt_suspend, nvt_resume);

/* ---------------------------------------------------------------------- */
/* Probe                                                                  */
/* ---------------------------------------------------------------------- */

static void nvt_cleanup(void *data)
{
	struct nvt_ts *ts = data;

	nvt_stop(ts);
	nvt_fw_release(ts);
}

/*
 * Tablets built with different panels need different touch firmware, and the
 * vendor names the files after the panel. Where the bootloader names the
 * fitted panel on the command line (msm_drm.dsi_display0=...nt36523_<panel>_video)
 * and the configured file follows that naming, load the file of that panel.
 */
static const char *nvt_fw_name_for_panel(struct device *dev, const char *name)
{
	static const char key[] = "mdss_dsi_nt36523_";
	static const char prefix[] = "novatek_ts_fw_";
	const char *args, *s, *e, *fw;

	if (!name || strncmp(name, prefix, sizeof(prefix) - 1))
		return name;

	if (!of_chosen || of_property_read_string(of_chosen, "bootargs", &args))
		return name;

	s = strstr(args, key);
	if (!s)
		return name;

	s += sizeof(key) - 1;
	e = strstr(s, "_video");
	if (!e || e == s || e - s > 32 || memchr(s, ' ', e - s))
		return name;

	fw = devm_kasprintf(dev, GFP_KERNEL, "%s%.*s.bin", prefix, (int)(e - s), s);

	return fw ?: name;
}

/*
 * The touch node of these tablets is shared by the drivers of every touch
 * controller fitted to the different panel variants. When the bootloader
 * names a panel of another controller family, leave the device to its driver.
 */
static bool nvt_other_panel(void)
{
	static const char key[] = "dsi_display0=qcom,mdss_dsi_";
	const char *args, *s;

	if (!of_chosen || of_property_read_string(of_chosen, "bootargs", &args))
		return false;

	s = strstr(args, key);
	if (!s)
		return false;

	return strncmp(s + sizeof(key) - 1, "nt36523_", 8) != 0;
}

static int nvt_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct input_dev *input;
	struct nvt_ts *ts;
	int ret;

	if (nvt_other_panel())
		return -ENODEV;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	ts->chip = spi_get_device_match_data(spi);
	if (!ts->chip)
		return -ENODEV;

	if (!spi->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	ts->tx_buf = devm_kmalloc(dev, NVT_XFER_BUF_LEN, GFP_KERNEL);
	ts->rx_buf = devm_kmalloc(dev, NVT_XFER_BUF_LEN, GFP_KERNEL);
	if (!ts->tx_buf || !ts->rx_buf)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &ts->lock);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&ts->fw_work, nvt_fw_work);
	spi_set_drvdata(spi, ts);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "SPI setup failed\n");

	ts->fw_name = ts->chip->default_fw_name;
	device_property_read_string(dev, "firmware-name", &ts->fw_name);
	ts->fw_name = nvt_fw_name_for_panel(dev, ts->fw_name);

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	ts->input = input;
	input->name = "Novatek NT36523 Touchscreen";
	input->id.bustype = BUS_SPI;

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ts->chip->max_x - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ts->chip->max_y - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, NVT_MAX_PRESSURE, 0, 0);
	touchscreen_parse_properties(input, true, &ts->prop);

	ret = input_mt_init_slots(input, NVT_MAX_FINGERS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, nvt_irq_thread,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					dev_name(dev), ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request interrupt\n");

	ret = input_register_device(input);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, nvt_cleanup, ts);
	if (ret)
		return ret;

	ts->is_follower = drm_is_panel_follower(dev);
	if (ts->is_follower) {
		ts->follower.funcs = &nvt_panel_follower_funcs;
		ret = devm_drm_panel_add_follower(dev, &ts->follower);
		if (ret)
			return dev_err_probe(dev, ret, "failed to follow the panel\n");
	} else {
		dev_warn(dev, "no panel to follow, assuming the display is already on\n");
		nvt_start(ts, 0);
	}

	return 0;
}

static const struct nvt_mem_map nt36523_mem_map = {
	.event_buf_addr		= 0x2fe00,
	.boot_rdy_addr		= 0x3f10d,
	.tx_auto_copy_en	= 0x3f7e8,
	.spi_dma_tx_info	= 0x3f7f1,
	.ilm_length_addr	= 0x3f118,
	.dlm_length_addr	= 0x3f130,
	.ilm_des_addr		= 0x3f128,
	.dlm_des_addr		= 0x3f12c,
	.g_ilm_checksum_addr	= 0x3f100,
	.g_dlm_checksum_addr	= 0x3f104,
	.r_ilm_checksum_addr	= 0x3f120,
	.r_dlm_checksum_addr	= 0x3f124,
	.bld_crc_en_addr	= 0x3f30e,
	.bld_ilm_dlm_crc_addr	= 0x3f133,
};

static const struct nvt_chip_data nt36523_data = {
	.name			= "NT36523",
	.mmap			= &nt36523_mem_map,
	.swrst_n8_addr		= 0x3f0fe,
	.spi_rd_fast_addr	= 0x3f310,
	.id			= { 0xff, 0xff, 0xff, 0x23, 0x65, 0x03 },
	.id_mask		= { 0, 0, 0, 1, 1, 1 },
	.max_x			= 1200,
	.max_y			= 2000,
	.default_fw_name	= "novatek/nt36523.bin",
};

static const struct of_device_id nvt_of_match[] = {
	{ .compatible = "novatek,nt36523-spi", .data = &nt36523_data },
	{ }
};
MODULE_DEVICE_TABLE(of, nvt_of_match);

static const struct spi_device_id nvt_spi_ids[] = {
	{ "nt36523-spi", (kernel_ulong_t)&nt36523_data },
	{ }
};
MODULE_DEVICE_TABLE(spi, nvt_spi_ids);

static struct spi_driver nvt_spi_driver = {
	.driver = {
		.name = "novatek-nt36523-spi",
		.of_match_table = nvt_of_match,
		.pm = pm_sleep_ptr(&nvt_pm_ops),
	},
	.probe = nvt_probe,
	.id_table = nvt_spi_ids,
};
module_spi_driver(nvt_spi_driver);

MODULE_DESCRIPTION("Novatek NT36523 SPI touchscreen driver");
MODULE_LICENSE("GPL");
