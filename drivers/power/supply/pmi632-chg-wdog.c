// SPDX-License-Identifier: GPL-2.0-only
/*
 * PMI632 charger watchdog keeper
 *
 * The boot firmware leaves the charger's bark/bite watchdog armed on every
 * cable insertion and set to switch charging off when it bites. The vendor
 * charger driver pets it; mainline has no PMI632 charger driver yet, so
 * charging stops about two minutes after plugging in. Pet it from here until
 * a real charger driver exists. Charging is also switched back on once
 * after a bite that has already happened.
 *
 * Input current: with no charger driver the chip stays in USB 2.0 mode
 * (500 mA) for every charger. When the port detection says it is a wall
 * charger (DCP), a charger it cannot identify (float) or a QC charger, do
 * what the stock driver does with this board's settings (float-option = 1,
 * usb-icl = 1.55 A): treat float as DCP and let the hardware run at the
 * configured 1.5 A limit. A computer port (SDP/CDP) is left at 500 mA.
 * AICL stays enabled so a weak charger makes the limit back off.
 */
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/spmi.h>
#include <linux/workqueue.h>

#define CHGR_STATUS_1			0x1006
#define  CHGR_STATUS_MASK		GENMASK(2, 0)
#define  CHGR_STATUS_DISABLED		7
#define CHGR_CHARGING_ENABLE_CMD	0x1042
#define  CHARGING_ENABLE_CMD_BIT	BIT(0)
#define USBIN_INPUT_STATUS		0x1306
#define  USBIN_PRESENT_MASK		GENMASK(1, 0)
#define MISC_WDOG_PET			0x1643
#define  WDOG_PET_BIT			BIT(0)
#define DCDC_CMD_OTG			0x1140
#define  OTG_EN_BIT			BIT(0)
#define APSD_STATUS			0x1307
#define  APSD_DTC_DONE_BIT		BIT(0)
#define APSD_RESULT_STATUS		0x1308
#define  APSD_SDP_BIT			BIT(0)
#define  APSD_OCP_BIT			BIT(1)
#define  APSD_CDP_BIT			BIT(2)
#define  APSD_DCP_BIT			BIT(3)
#define  APSD_FLOAT_BIT			BIT(4)
#define  APSD_QC2_BIT			BIT(5)
#define  APSD_QC3_BIT			BIT(6)
#define USBIN_INT_RT_STS		0x1310
#define  USBIN_PLUGIN_RT_STS_BIT	BIT(4)
#define CMD_ICL_OVERRIDE		0x1342
#define  ICL_OVERRIDE_BIT		BIT(0)
#define USBIN_OPTIONS_2_CFG		0x1363
#define  FLOAT_OPTIONS_MASK		GENMASK(2, 0)
#define USBIN_LOAD_CFG			0x1365
#define  ICL_OVERRIDE_AFTER_APSD_BIT	BIT(4)
#define USBIN_ICL_OPTIONS		0x1366
#define  USBIN_MODE_CHG_BIT		BIT(0)
#define USBIN_CURRENT_LIMIT_CFG		0x1370
#define  ICL_STEP_UA			50000
#define  ICL_WALL_UA			1500000
#define USBIN_AICL_OPTIONS_CFG		0x1380
#define  USBIN_AICL_EN_BIT		BIT(2)
#define ICL_STATUS			0x1107

#define POLL_INTERVAL			(5 * HZ)
#define PET_EVERY			4	/* polls, = 20 s */
#define ATTACH_INTERVAL			HZ
#define ATTACH_TRIES			120	/* wait up to 2 min for the PMIC */

static struct regmap *wd_regmap;
static struct device *wd_dev;
static struct delayed_work wd_work;
static struct delayed_work attach_work;
static unsigned int attach_tries;
static unsigned int poll_count;
static bool icl_done;

static void pmi632_set_icl(void)
{
	unsigned int rt = 0, otg = 0, apsd_st = 0, apsd = 0, mode = 0, icl = 0;
	unsigned int aicl = 0;

	regmap_read(wd_regmap, USBIN_INT_RT_STS, &rt);
	regmap_read(wd_regmap, DCDC_CMD_OTG, &otg);
	if (!(rt & USBIN_PLUGIN_RT_STS_BIT) || (otg & OTG_EN_BIT)) {
		icl_done = false;
		return;
	}

	regmap_read(wd_regmap, APSD_STATUS, &apsd_st);
	if (!(apsd_st & APSD_DTC_DONE_BIT))
		return;
	regmap_read(wd_regmap, APSD_RESULT_STATUS, &apsd);
	if (apsd & (APSD_SDP_BIT | APSD_CDP_BIT))
		return;
	if (!(apsd & (APSD_DCP_BIT | APSD_FLOAT_BIT | APSD_OCP_BIT |
		      APSD_QC2_BIT | APSD_QC3_BIT)))
		return;

	regmap_read(wd_regmap, USBIN_ICL_OPTIONS, &mode);
	if (icl_done && (mode & USBIN_MODE_CHG_BIT))
		return;

	/* float charger counts as a wall charger, as in the stock config */
	regmap_update_bits(wd_regmap, USBIN_OPTIONS_2_CFG, FLOAT_OPTIONS_MASK, 0);
	regmap_write(wd_regmap, USBIN_CURRENT_LIMIT_CFG, ICL_WALL_UA / ICL_STEP_UA);
	regmap_update_bits(wd_regmap, CMD_ICL_OVERRIDE, ICL_OVERRIDE_BIT, 0);
	regmap_update_bits(wd_regmap, USBIN_LOAD_CFG,
			   ICL_OVERRIDE_AFTER_APSD_BIT, 0);
	regmap_update_bits(wd_regmap, USBIN_ICL_OPTIONS, USBIN_MODE_CHG_BIT,
			   USBIN_MODE_CHG_BIT);
	msleep(200);

	regmap_read(wd_regmap, ICL_STATUS, &icl);
	regmap_read(wd_regmap, USBIN_AICL_OPTIONS_CFG, &aicl);
	dev_info(wd_dev, "wall charger (apsd 0x%02x): input limit 1500 mA, now %u mA, aicl %s\n",
		 apsd, icl * ICL_STEP_UA / 1000,
		 (aicl & USBIN_AICL_EN_BIT) ? "on" : "off");
	icl_done = true;
}

static void pmi632_wdog_work(struct work_struct *work)
{
	unsigned int status = 0, usbin = 0, en = 0;

	pmi632_set_icl();

	if (poll_count++ % PET_EVERY)
		goto out;

	regmap_write(wd_regmap, MISC_WDOG_PET, WDOG_PET_BIT);

	regmap_read(wd_regmap, CHGR_STATUS_1, &status);
	regmap_read(wd_regmap, USBIN_INPUT_STATUS, &usbin);
	regmap_read(wd_regmap, CHGR_CHARGING_ENABLE_CMD, &en);

	/* Bitten before we got here: a cable is in, yet charging is off */
	if ((status & CHGR_STATUS_MASK) == CHGR_STATUS_DISABLED &&
	    (usbin & USBIN_PRESENT_MASK) && (en & CHARGING_ENABLE_CMD_BIT)) {
		dev_info(wd_dev, "charging was switched off by the watchdog, restarting it\n");
		regmap_update_bits(wd_regmap, CHGR_CHARGING_ENABLE_CMD,
				   CHARGING_ENABLE_CMD_BIT, 0);
		msleep(100);
		regmap_update_bits(wd_regmap, CHGR_CHARGING_ENABLE_CMD,
				   CHARGING_ENABLE_CMD_BIT,
				   CHARGING_ENABLE_CMD_BIT);
	}

out:
	schedule_delayed_work(&wd_work, POLL_INTERVAL);
}

/* Built in, this runs before the PMIC has probed: wait for its regmap */
static void pmi632_wdog_attach(struct work_struct *work)
{
	struct device_node *np;
	struct spmi_device *sdev = NULL;

	for_each_compatible_node(np, NULL, "qcom,pmi632") {
		u32 usid;

		if (of_property_read_u32(np, "reg", &usid) || usid != 2)
			continue;
		sdev = spmi_find_device_by_of_node(np);
		of_node_put(np);
		break;
	}

	if (sdev) {
		wd_regmap = dev_get_regmap(&sdev->dev, NULL);
		if (wd_regmap) {
			wd_dev = &sdev->dev;
			schedule_delayed_work(&wd_work, 0);
			dev_info(wd_dev, "PMI632 charger watchdog is being petted every %u s\n",
				 PET_EVERY * POLL_INTERVAL / HZ);
			return;
		}
		put_device(&sdev->dev);
	}

	if (++attach_tries < ATTACH_TRIES)
		schedule_delayed_work(&attach_work, ATTACH_INTERVAL);
	else
		pr_warn("pmi632-chg-wdog: no PMI632 regmap, charging may stop\n");
}

static int __init pmi632_wdog_init(void)
{
	/* Only the boards whose boot firmware is known to arm the watchdog */
	if (!of_machine_is_compatible("samsung,gta4l"))
		return -ENODEV;

	INIT_DELAYED_WORK(&wd_work, pmi632_wdog_work);
	INIT_DELAYED_WORK(&attach_work, pmi632_wdog_attach);
	schedule_delayed_work(&attach_work, 0);

	return 0;
}
module_init(pmi632_wdog_init);

static void __exit pmi632_wdog_exit(void)
{
	cancel_delayed_work_sync(&attach_work);
	cancel_delayed_work_sync(&wd_work);
	if (wd_dev)
		put_device(wd_dev);
}
module_exit(pmi632_wdog_exit);

MODULE_DESCRIPTION("PMI632 charger watchdog keeper");
MODULE_LICENSE("GPL");
