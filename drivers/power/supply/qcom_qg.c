// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm QGauge (PMI632) battery monitor, read-only
 *
 * The QGauge block samples the battery voltage and current on its own. This
 * driver only reads those samples and never writes to the PMIC: it does not
 * configure the gauge and it does not control charging.
 *
 * The state of charge is estimated from the open circuit voltage, which in
 * turn is estimated from the measured voltage, the measured current and the
 * internal resistance given in the battery description. That is cruder than
 * the coulomb counting done by the vendor driver, but good enough to show a
 * battery level and a charging state.
 *
 * Register layout taken from the downstream qpnp-qg driver,
 * Copyright (c) 2018-2020 The Linux Foundation.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/iio/consumer.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define QG_PERPH_SUBTYPE		0x05
#define QG_ADC_IBAT_5A			0x3

#define QG_STATUS2			0x09
#define QG_BATTERY_MISSING		BIT(3)

#define QG_LAST_ADC_V_DATA0		0xc0
#define QG_LAST_ADC_I_DATA0		0xc2

#define QG_ADC_RESET_VAL		0x8000

/* Nanovolts and nanoamperes per LSB */
#define QG_V_LSB_NV			194637
#define QG_I_LSB_5A_NA			152588
#define QG_I_LSB_10A_NA			305176

#define QG_POLL_MS			10000

#define QG_VBAT_MIN_PLAUSIBLE_UV	2000000
#define QG_VBAT_MAX_PLAUSIBLE_UV	5000000
#define QG_USB_PRESENT_UV		4000000
#define QG_CHARGING_UA			30000
#define QG_FULL_UA			150000
#define QG_FULL_BELOW_MAX_UV		60000
#define QG_DEFAULT_RESISTANCE_UOHM	100000
#define QG_DEFAULT_VMIN_UV		3400000
#define QG_DEFAULT_VMAX_UV		4350000

struct qcom_qg {
	struct device *dev;
	struct regmap *regmap;
	unsigned int base;
	unsigned int i_lsb_na;

	struct power_supply *psy;
	struct power_supply_battery_info *info;
	struct iio_channel *vbat_chan;
	struct iio_channel *usbin_chan;
	struct delayed_work work;

	/* Protects the cached state below */
	struct mutex lock;
	bool valid;
	bool present;
	int vbat_uv;
	/* Positive while charging, as the power supply class wants it */
	int ibat_ua;
	int ocv_uv;
	int soc;
	int status;
};

static int qcom_qg_read_u16(struct qcom_qg *qg, unsigned int reg, u16 *val)
{
	u8 buf[2];
	int ret;

	ret = regmap_bulk_read(qg->regmap, qg->base + reg, buf, sizeof(buf));
	if (ret)
		return ret;

	*val = buf[0] | (buf[1] << 8);

	return 0;
}

static int qcom_qg_read_vbat(struct qcom_qg *qg, int *uv)
{
	int ret, val;
	u16 raw;

	ret = qcom_qg_read_u16(qg, QG_LAST_ADC_V_DATA0, &raw);
	if (!ret && raw != QG_ADC_RESET_VAL) {
		val = div_u64((u64)raw * QG_V_LSB_NV, 1000);
		if (val >= QG_VBAT_MIN_PLAUSIBLE_UV && val <= QG_VBAT_MAX_PLAUSIBLE_UV) {
			*uv = val;
			return 0;
		}
	}

	/* The gauge has no usable sample; ask the general purpose ADC */
	if (qg->vbat_chan) {
		ret = iio_read_channel_processed(qg->vbat_chan, &val);
		if (ret < 0)
			return ret;
		if (val >= QG_VBAT_MIN_PLAUSIBLE_UV && val <= QG_VBAT_MAX_PLAUSIBLE_UV) {
			*uv = val;
			return 0;
		}
	}

	return ret ?: -ENODATA;
}

/* Returns the battery current, positive while charging */
static int qcom_qg_read_ibat(struct qcom_qg *qg, int *ua)
{
	u16 raw;
	int ret;

	ret = qcom_qg_read_u16(qg, QG_LAST_ADC_I_DATA0, &raw);
	if (ret)
		return ret;

	if (raw == QG_ADC_RESET_VAL)
		return -ENODATA;

	/* The gauge counts current flowing out of the battery as positive */
	*ua = -div_s64((s64)(s16)raw * qg->i_lsb_na, 1000);

	return 0;
}

static bool qcom_qg_usb_present(struct qcom_qg *qg)
{
	int ret, uv;

	if (!qg->usbin_chan)
		return false;

	ret = iio_read_channel_processed(qg->usbin_chan, &uv);
	if (ret < 0)
		return false;

	return uv > QG_USB_PRESENT_UV;
}

static int qcom_qg_ocv_to_soc(struct qcom_qg *qg, int ocv_uv)
{
	struct power_supply_battery_info *info = qg->info;
	int vmin = QG_DEFAULT_VMIN_UV, vmax = QG_DEFAULT_VMAX_UV;
	int soc;

	if (info) {
		soc = power_supply_batinfo_ocv2cap(info, ocv_uv, 25);
		if (soc >= 0)
			return clamp(soc, 0, 100);

		if (info->voltage_min_design_uv > 0)
			vmin = info->voltage_min_design_uv;
		if (info->voltage_max_design_uv > vmin)
			vmax = info->voltage_max_design_uv;
	}

	/* No table: a straight line is better than nothing */
	soc = div_s64((s64)(ocv_uv - vmin) * 100, vmax - vmin);

	return clamp(soc, 0, 100);
}

static void qcom_qg_update(struct qcom_qg *qg)
{
	struct power_supply_battery_info *info = qg->info;
	int resistance = QG_DEFAULT_RESISTANCE_UOHM;
	int vmax = QG_DEFAULT_VMAX_UV;
	int vbat, ibat = 0, ocv, soc, status, old_soc, old_status;
	unsigned int sts = 0;
	bool present, usb, changed;

	if (info) {
		if (info->factory_internal_resistance_uohm > 0)
			resistance = info->factory_internal_resistance_uohm;
		if (info->voltage_max_design_uv > 0)
			vmax = info->voltage_max_design_uv;
	}

	regmap_read(qg->regmap, qg->base + QG_STATUS2, &sts);
	present = !(sts & QG_BATTERY_MISSING);

	if (qcom_qg_read_vbat(qg, &vbat)) {
		guard(mutex)(&qg->lock);
		qg->valid = false;
		qg->present = present;
		return;
	}

	/* Without a current sample the voltage is used as it is */
	qcom_qg_read_ibat(qg, &ibat);
	usb = qcom_qg_usb_present(qg);

	/* Remove the voltage drop (or rise, while charging) across the cell */
	ocv = vbat - div_s64((s64)ibat * resistance, 1000000);
	soc = qcom_qg_ocv_to_soc(qg, ocv);

	guard(mutex)(&qg->lock);

	old_soc = qg->soc;
	old_status = qg->status;

	if (qg->valid) {
		/*
		 * The estimate moves with the load. Follow it slowly, and do
		 * not let the level rise while discharging or fall while
		 * charging.
		 */
		if (soc > qg->soc && ibat >= 0)
			soc = qg->soc + 1;
		else if (soc < qg->soc && ibat <= 0)
			soc = qg->soc - 1;
		else
			soc = qg->soc;
	}

	if (!present)
		status = POWER_SUPPLY_STATUS_UNKNOWN;
	else if (ibat > QG_CHARGING_UA && soc < 100)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else if (soc >= 100 ||
		 (usb && abs(ibat) < QG_FULL_UA && vbat > vmax - QG_FULL_BELOW_MAX_UV))
		status = POWER_SUPPLY_STATUS_FULL;
	else if (usb || ibat > QG_CHARGING_UA)
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (status == POWER_SUPPLY_STATUS_FULL)
		soc = 100;

	changed = !qg->valid || soc != old_soc || status != old_status ||
		  present != qg->present;

	qg->valid = true;
	qg->present = present;
	qg->vbat_uv = vbat;
	qg->ibat_ua = ibat;
	qg->ocv_uv = ocv;
	qg->soc = soc;
	qg->status = status;

	if (changed && qg->psy)
		power_supply_changed(qg->psy);
}

static void qcom_qg_work(struct work_struct *work)
{
	struct qcom_qg *qg = container_of(work, struct qcom_qg, work.work);

	qcom_qg_update(qg);
	schedule_delayed_work(&qg->work, msecs_to_jiffies(QG_POLL_MS));
}

static enum power_supply_property qcom_qg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static int qcom_qg_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct qcom_qg *qg = power_supply_get_drvdata(psy);

	guard(mutex)(&qg->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = qg->present;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		return 0;
	default:
		break;
	}

	if (!qg->valid)
		return -ENODATA;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = qg->status;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = qg->soc;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = qg->vbat_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = qg->ocv_uv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = qg->ibat_ua;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct power_supply_desc qcom_qg_psy_desc = {
	.name = "qcom-qg-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = qcom_qg_props,
	.num_properties = ARRAY_SIZE(qcom_qg_props),
	.get_property = qcom_qg_get_property,
};

static struct iio_channel *qcom_qg_get_channel(struct device *dev, const char *name)
{
	struct iio_channel *chan;

	chan = devm_iio_channel_get(dev, name);
	if (IS_ERR(chan)) {
		if (PTR_ERR(chan) == -EPROBE_DEFER)
			return chan;
		/* The channels only refine the result; work without them */
		return NULL;
	}

	return chan;
}

static void qcom_qg_stop(void *data)
{
	struct qcom_qg *qg = data;

	cancel_delayed_work_sync(&qg->work);
}

static int qcom_qg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = { };
	struct qcom_qg *qg;
	unsigned int subtype;
	int ret;

	qg = devm_kzalloc(dev, sizeof(*qg), GFP_KERNEL);
	if (!qg)
		return -ENOMEM;

	qg->dev = dev;
	qg->status = POWER_SUPPLY_STATUS_UNKNOWN;

	qg->regmap = dev_get_regmap(dev->parent, NULL);
	if (!qg->regmap)
		return dev_err_probe(dev, -ENODEV, "no regmap from the PMIC\n");

	ret = device_property_read_u32(dev, "reg", &qg->base);
	if (ret)
		return dev_err_probe(dev, ret, "no register base\n");

	ret = regmap_read(qg->regmap, qg->base + QG_PERPH_SUBTYPE, &subtype);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the gauge type\n");

	qg->i_lsb_na = subtype == QG_ADC_IBAT_5A ? QG_I_LSB_5A_NA : QG_I_LSB_10A_NA;

	qg->vbat_chan = qcom_qg_get_channel(dev, "vbat");
	if (IS_ERR(qg->vbat_chan))
		return PTR_ERR(qg->vbat_chan);

	qg->usbin_chan = qcom_qg_get_channel(dev, "usbin");
	if (IS_ERR(qg->usbin_chan))
		return PTR_ERR(qg->usbin_chan);

	ret = devm_mutex_init(dev, &qg->lock);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&qg->work, qcom_qg_work);

	cfg.drv_data = qg;
	cfg.fwnode = dev_fwnode(dev);

	qg->psy = devm_power_supply_register(dev, &qcom_qg_psy_desc, &cfg);
	if (IS_ERR(qg->psy))
		return dev_err_probe(dev, PTR_ERR(qg->psy),
				     "failed to register the battery\n");

	ret = power_supply_get_battery_info(qg->psy, &qg->info);
	if (ret) {
		dev_warn(dev, "no battery description (%d), the level will be rough\n", ret);
		qg->info = NULL;
	}

	ret = devm_add_action_or_reset(dev, qcom_qg_stop, qg);
	if (ret)
		return ret;

	qcom_qg_update(qg);

	scoped_guard(mutex, &qg->lock) {
		if (qg->valid)
			dev_info(dev, "battery %d%%, %d mV, %d mA, gauge subtype %u\n",
				 qg->soc, qg->vbat_uv / 1000, qg->ibat_ua / 1000, subtype);
		else
			dev_warn(dev, "no valid battery voltage yet, gauge subtype %u\n",
				 subtype);
	}

	schedule_delayed_work(&qg->work, msecs_to_jiffies(QG_POLL_MS));

	return 0;
}

static const struct of_device_id qcom_qg_of_match[] = {
	{ .compatible = "qcom,pmi632-qg" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_qg_of_match);

static struct platform_driver qcom_qg_driver = {
	.driver = {
		.name = "qcom-qg",
		.of_match_table = qcom_qg_of_match,
	},
	.probe = qcom_qg_probe,
};
module_platform_driver(qcom_qg_driver);

MODULE_DESCRIPTION("Qualcomm QGauge read-only battery monitor");
MODULE_LICENSE("GPL");
