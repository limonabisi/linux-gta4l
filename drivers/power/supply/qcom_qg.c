// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm QGauge (PMI632) battery monitor, read-only
 *
 * The QGauge block samples the battery voltage and current on its own. This
 * driver only reads those samples, and a few status bits of the charger in
 * the same PMIC. It never writes to the PMIC: it does not configure the gauge
 * and it does not control charging.
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

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
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

#define QG_STATUS1			0x08
#define QG_BATTERY_PRESENT		BIT(0)

#define QG_LAST_ADC_V_DATA0		0xc0
#define QG_LAST_ADC_I_DATA0		0xc2

#define QG_ADC_RESET_VAL		0x8000

/* Nanovolts and nanoamperes per LSB */
#define QG_V_LSB_NV			194637
#define QG_I_LSB_5A_NA			152588
#define QG_I_LSB_10A_NA			305176

/* Charger and boost blocks of the same PMIC, only ever read */
#define CHGR_STATUS1			0x1006
#define CHGR_STATUS_MASK		GENMASK(2, 0)
#define CHGR_STATUS_TRICKLE		1
#define CHGR_STATUS_TAPER		4
#define DCDC_CMD_OTG			0x1140
#define DCDC_OTG_EN			BIT(0)
#define USBIN_INT_RT_STS		0x1310
#define USBIN_PLUGIN_RT_STS		BIT(4)

#define QG_POLL_MS			10000
/* After the cable is plugged or pulled, look again a few times soon */
#define QG_PLUG_DEBOUNCE_MS		300
#define QG_FAST_POLL_MS			1000
#define QG_FAST_POLLS			6

#define QG_VBAT_MIN_PLAUSIBLE_UV	2000000
#define QG_VBAT_MAX_PLAUSIBLE_UV	5000000
#define QG_USB_PRESENT_UV		4000000
#define QG_CHARGING_UA			30000
#define QG_FULL_UA			150000
#define QG_FULL_BELOW_MAX_UV		60000
#define QG_SOC_HYSTERESIS		3
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
	atomic_t fast_polls;
	/* The charger says whether a cable is in, and agrees with the ADC */
	bool plug_status_ok;

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

static int qcom_qg_usbin_adc(struct qcom_qg *qg)
{
	int ret, uv;

	if (!qg->usbin_chan)
		return -ENODEV;

	ret = iio_read_channel_processed(qg->usbin_chan, &uv);
	if (ret < 0)
		return ret;

	return uv > QG_USB_PRESENT_UV;
}

static int qcom_qg_usbin_plugged(struct qcom_qg *qg)
{
	unsigned int val;
	int ret;

	ret = regmap_read(qg->regmap, USBIN_INT_RT_STS, &val);
	if (ret)
		return ret;

	return !!(val & USBIN_PLUGIN_RT_STS);
}

/* Returns 1 with a charger plugged in, 0 without, negative if unknown */
static int qcom_qg_usb_present(struct qcom_qg *qg)
{
	unsigned int val;

	/* The tablet powers the port itself: a device is plugged in, no charger */
	if (!regmap_read(qg->regmap, DCDC_CMD_OTG, &val) && (val & DCDC_OTG_EN))
		return 0;

	if (qg->plug_status_ok)
		return qcom_qg_usbin_plugged(qg);

	return qcom_qg_usbin_adc(qg);
}

static bool qcom_qg_charger_charging(struct qcom_qg *qg)
{
	unsigned int val;

	if (regmap_read(qg->regmap, CHGR_STATUS1, &val))
		return false;

	val &= CHGR_STATUS_MASK;

	return val >= CHGR_STATUS_TRICKLE && val <= CHGR_STATUS_TAPER;
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
	unsigned int sts;
	bool present, changed;
	int usb;

	if (info) {
		if (info->factory_internal_resistance_uohm > 0)
			resistance = info->factory_internal_resistance_uohm;
		if (info->voltage_max_design_uv > 0)
			vmax = info->voltage_max_design_uv;
	}

	/* If the status cannot be read, assume the battery is there */
	if (regmap_read(qg->regmap, qg->base + QG_STATUS1, &sts))
		present = true;
	else
		present = sts & QG_BATTERY_PRESENT;

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
		int diff = soc - qg->soc;
		int step = 1 + abs(diff) / 10;

		/*
		 * The estimate moves with the load. Follow it slowly. A move
		 * against the direction of the current (rising while
		 * discharging, falling while charging) needs a clear
		 * difference, so that noise is ignored but a wrong value
		 * left behind by a load peak is still corrected.
		 */
		if (diff > 0 && (ibat >= 0 || diff >= QG_SOC_HYSTERESIS))
			soc = qg->soc + step;
		else if (diff < 0 && (ibat <= 0 || diff <= -QG_SOC_HYSTERESIS))
			soc = qg->soc - step;
		else
			soc = qg->soc;
	}

	if (!present)
		status = POWER_SUPPLY_STATUS_UNKNOWN;
	else if (!usb)
		/*
		 * Known to be without a charger. The last current sample may
		 * still be from before the cable was pulled, so do not ask it.
		 */
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (usb > 0 && abs(ibat) < QG_FULL_UA && vbat > vmax - QG_FULL_BELOW_MAX_UV)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (ibat > QG_CHARGING_UA)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else if (soc >= 100 && ibat > -QG_CHARGING_UA)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (usb > 0 && ibat > -QG_CHARGING_UA && qcom_qg_charger_charging(qg))
		/* Just plugged in: the charger runs, the sample is still old */
		status = POWER_SUPPLY_STATUS_CHARGING;
	else if (usb > 0)
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

	if (atomic_dec_if_positive(&qg->fast_polls) >= 0)
		schedule_delayed_work(&qg->work, msecs_to_jiffies(QG_FAST_POLL_MS));
	else
		schedule_delayed_work(&qg->work, msecs_to_jiffies(QG_POLL_MS));
}

static irqreturn_t qcom_qg_plug_irq(int irq, void *data)
{
	struct qcom_qg *qg = data;

	atomic_set(&qg->fast_polls, QG_FAST_POLLS);
	mod_delayed_work(system_wq, &qg->work, msecs_to_jiffies(QG_PLUG_DEBOUNCE_MS));

	return IRQ_HANDLED;
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
	int ret, irq, plugged, adc;

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

	/*
	 * Believe the charger's plug status only if it can be read and tells
	 * the same as the measured input voltage right now.
	 */
	plugged = qcom_qg_usbin_plugged(qg);
	adc = qcom_qg_usbin_adc(qg);
	qg->plug_status_ok = plugged >= 0 && (adc < 0 || adc == plugged);

	cfg.drv_data = qg;
	cfg.fwnode = dev_fwnode(dev);
	/* A battery level changing is no reason to stay awake or wake up */
	cfg.no_wakeup_source = true;

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

	/* Optional: without it the state is only as fresh as the polling */
	irq = platform_get_irq_byname_optional(pdev, "usb-plugin");
	if (irq == -EPROBE_DEFER)
		return irq;
	if (irq > 0) {
		ret = devm_request_threaded_irq(dev, irq, NULL, qcom_qg_plug_irq,
						IRQF_ONESHOT, "qcom-qg-usb-plugin", qg);
		if (ret)
			dev_warn(dev, "no plug interrupt (%d), polling only\n", ret);
	}
	dev_info(dev, "plug status from %s, plug interrupt %s\n",
		 qg->plug_status_ok ? "the charger" : "the input voltage",
		 irq > 0 && !ret ? "on" : "off");

	return 0;
}

static const struct of_device_id qcom_qg_of_match[] = {
	{ .compatible = "qcom,pmi632-qg" },
	{ .compatible = "qcom,pmi632-qgauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_qg_of_match);

static struct platform_driver qcom_qg_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = qcom_qg_of_match,
	},
	.probe = qcom_qg_probe,
};
module_platform_driver(qcom_qg_driver);

MODULE_DESCRIPTION("Qualcomm QGauge read-only battery monitor");
MODULE_LICENSE("GPL");
