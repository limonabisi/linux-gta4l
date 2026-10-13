// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm QGauge (PMI632) battery monitor, read-only
 *
 * The QGauge block samples the battery voltage and current on its own. This
 * driver only reads those samples, and a few status bits of the charger in
 * the same PMIC. It never writes to the PMIC: it does not configure the gauge
 * and it does not control charging.
 *
 * The state of charge is counted: every sample of the measured current is
 * added to the charge in the battery, against the design capacity from the
 * battery description. The count is set to full when the charger reports
 * that it terminated charging. The open circuit voltage, estimated from the
 * measured voltage, current and internal resistance, is only used to start
 * the count at boot and to restart it after a long sleep, when the battery
 * has rested and its voltage is a good measure.
 *
 * Register layout taken from the downstream qpnp-qg driver,
 * Copyright (c) 2018-2020 The Linux Foundation.
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/timekeeping.h>
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
#define CHGR_STATUS_TERMINATE		5
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
/* Used when the battery description gives no capacity */
#define QG_DEFAULT_CAPACITY_UAH		7040000
/* A gap longer than this is a sleep: the samples did not see it */
#define QG_MAX_COUNT_GAP_MS		(3 * QG_POLL_MS)
/* After a sleep this long the battery has rested; its voltage can be trusted */
#define QG_REST_MS			(10 * 60 * 1000)
#define QG_REST_UA			50000
/*
 * Until the count has a reliable starting point (full, empty, a rested
 * battery or a value saved at shutdown), pull it towards the voltage
 * estimate whenever the current is low enough for that estimate to hold.
 * The first sample is taken while the system boots under heavy load.
 */
#define QG_SETTLE_UA			300000
#define QG_SETTLE_DIV			16
/* Do not show 0% while the voltage says there is still charge left */
#define QG_EMPTY_ABOVE_VMIN_UV		100000

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

	/* Coulomb counter */
	bool counting;
	/* The count started from a point that is known to be right */
	bool anchored;
	/* The charger finished; stays until unplugged or discharging */
	bool full_latched;
	/* A full or empty point was seen since the other one: can learn */
	bool full_seen;
	bool empty_seen;
	s64 charge_uah;
	s64 full_uah;
	s64 design_uah;
	ktime_t last_sample;
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

/* Returns the charger's state machine position, negative if unknown */
static int qcom_qg_charger_state(struct qcom_qg *qg)
{
	unsigned int val;

	if (regmap_read(qg->regmap, CHGR_STATUS1, &val))
		return -EIO;

	return val & CHGR_STATUS_MASK;
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
	int vmax = QG_DEFAULT_VMAX_UV, vmin = QG_DEFAULT_VMIN_UV;
	int vbat, ibat = 0, ocv, soc, status, old_soc, old_status;
	unsigned int sts;
	bool present, changed, terminated;
	int usb, chg_state;
	ktime_t now;

	if (info) {
		if (info->factory_internal_resistance_uohm > 0)
			resistance = info->factory_internal_resistance_uohm;
		if (info->voltage_max_design_uv > 0)
			vmax = info->voltage_max_design_uv;
		if (info->voltage_min_design_uv > 0)
			vmin = info->voltage_min_design_uv;
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

	chg_state = usb > 0 ? qcom_qg_charger_state(qg) : -ENODEV;
	now = ktime_get_boottime();

	/* Remove the voltage drop (or rise, while charging) across the cell */
	ocv = vbat - div_s64((s64)ibat * resistance, 1000000);

	guard(mutex)(&qg->lock);

	old_soc = qg->soc;
	old_status = qg->status;

	/* The charger says it is done: the battery is full */
	terminated = usb > 0 && (chg_state == CHGR_STATUS_TERMINATE ||
				 (abs(ibat) < QG_FULL_UA &&
				  vbat > vmax - QG_FULL_BELOW_MAX_UV));

	if (!qg->counting) {
		/* Nothing counted yet: start from the voltage, once */
		qg->charge_uah = div_s64((s64)qcom_qg_ocv_to_soc(qg, ocv) *
					 qg->full_uah, 100);
		qg->counting = true;
	} else {
		s64 gap_ms = ktime_ms_delta(now, qg->last_sample);

		if (gap_ms <= QG_MAX_COUNT_GAP_MS) {
			/* Charge moved in or out since the last sample */
			qg->charge_uah += div_s64((s64)(ibat + qg->ibat_ua) / 2 * gap_ms,
						  3600 * 1000);
		} else if (gap_ms >= QG_REST_MS && abs(ibat) < QG_REST_UA) {
			/* Long sleep: the battery rested, its voltage is right */
			qg->charge_uah = div_s64((s64)qcom_qg_ocv_to_soc(qg, ocv) *
						 qg->full_uah, 100);
			qg->anchored = true;
		} else {
			/* Short sleep: count it at the current seen on waking */
			qg->charge_uah += div_s64((s64)min(ibat, 0) * gap_ms,
						  3600 * 1000);
		}
	}
	qg->last_sample = now;

	if (!qg->anchored && abs(ibat) < QG_SETTLE_UA) {
		s64 target = div_s64((s64)qcom_qg_ocv_to_soc(qg, ocv) * qg->full_uah, 100);

		qg->charge_uah += div_s64(target - qg->charge_uah, QG_SETTLE_DIV);
	}

	if (terminated) {
		/* Learn the real capacity when counted up from empty */
		if (qg->empty_seen) {
			s64 learned = qg->charge_uah;

			if (learned > qg->design_uah / 2 &&
			    learned < qg->design_uah + qg->design_uah / 10)
				qg->full_uah = learned;
			qg->empty_seen = false;
		}
		qg->charge_uah = qg->full_uah;
		qg->full_latched = true;
		qg->full_seen = true;
		qg->anchored = true;
	} else if (usb <= 0 || ibat < -QG_CHARGING_UA) {
		/* Unplugged, or the load is bigger than the charger */
		qg->full_latched = false;
	}

	if (!usb && vbat < vmin + QG_EMPTY_ABOVE_VMIN_UV && ibat > -QG_FULL_UA) {
		/* Light load and the voltage at the bottom: the battery is empty */
		if (qg->full_seen) {
			s64 learned = qg->full_uah - qg->charge_uah;

			if (learned > qg->design_uah / 2 &&
			    learned < qg->design_uah + qg->design_uah / 10)
				qg->full_uah = learned;
			qg->full_seen = false;
		}
		qg->charge_uah = 0;
		qg->empty_seen = true;
		qg->anchored = true;
	}

	/*
	 * While a capacity is being learned the count may run past the end
	 * it is measured from: below empty after a full charge, above full
	 * after an empty battery. Otherwise it stays within the battery.
	 */
	qg->charge_uah = clamp_t(s64, qg->charge_uah,
				 qg->full_seen ? -qg->design_uah / 2 : 0,
				 qg->empty_seen ? qg->design_uah + qg->design_uah / 10 :
						  qg->full_uah);
	soc = DIV_ROUND_CLOSEST_ULL(clamp_t(s64, qg->charge_uah, 0, qg->full_uah) * 100,
				    qg->full_uah);

	/* Counted to zero while the voltage still has charge: hold at 1% */
	if (soc == 0 && vbat >= vmin + QG_EMPTY_ABOVE_VMIN_UV)
		soc = 1;
	/* Only the charger finishing makes 100% */
	if (soc == 100 && !qg->full_latched)
		soc = 99;

	if (!present)
		status = POWER_SUPPLY_STATUS_UNKNOWN;
	else if (!usb)
		/*
		 * Known to be without a charger. The last current sample may
		 * still be from before the cable was pulled, so do not ask it.
		 */
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (qg->full_latched)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (ibat > QG_CHARGING_UA)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else if (usb > 0 && ibat > -QG_CHARGING_UA &&
		 chg_state >= CHGR_STATUS_TRICKLE && chg_state <= CHGR_STATUS_TAPER)
		/* Just plugged in: the charger runs, the sample is still old */
		status = POWER_SUPPLY_STATUS_CHARGING;
	else if (usb > 0)
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (qg->full_latched)
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
		queue_delayed_work(system_dfl_wq, &qg->work, msecs_to_jiffies(QG_FAST_POLL_MS));
	else
		queue_delayed_work(system_dfl_wq, &qg->work, msecs_to_jiffies(QG_POLL_MS));
}

static irqreturn_t qcom_qg_plug_irq(int irq, void *data)
{
	struct qcom_qg *qg = data;

	atomic_set(&qg->fast_polls, QG_FAST_POLLS);
	mod_delayed_work(system_dfl_wq, &qg->work, msecs_to_jiffies(QG_PLUG_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static enum power_supply_property qcom_qg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
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
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = qg->full_uah;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = qg->design_uah;
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
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = clamp_t(s64, qg->charge_uah, 0, qg->full_uah);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * The learned capacity is lost at reboot. Userspace may save it and write it
 * back, so that the level is right without a full charge and discharge.
 */
static int qcom_qg_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct qcom_qg *qg = power_supply_get_drvdata(psy);
	s64 v = val->intval;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (v <= qg->design_uah / 2 || v >= qg->design_uah + qg->design_uah / 10)
			return -ERANGE;
		scoped_guard(mutex, &qg->lock) {
			/* Keep the level, change what it is a level of */
			qg->charge_uah = div_s64(qg->charge_uah * v, qg->full_uah);
			qg->full_uah = v;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		/* The count saved at shutdown: continue from it */
		scoped_guard(mutex, &qg->lock) {
			if (v < 0 || v > qg->full_uah)
				return -ERANGE;
			qg->charge_uah = v;
			qg->counting = true;
			qg->anchored = true;
		}
		break;
	default:
		return -EINVAL;
	}

	mod_delayed_work(system_dfl_wq, &qg->work, 0);

	return 0;
}

static int qcom_qg_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_FULL || psp == POWER_SUPPLY_PROP_CHARGE_NOW;
}

static const struct power_supply_desc qcom_qg_psy_desc = {
	.name = "qcom-qg-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = qcom_qg_props,
	.num_properties = ARRAY_SIZE(qcom_qg_props),
	.get_property = qcom_qg_get_property,
	.set_property = qcom_qg_set_property,
	.property_is_writeable = qcom_qg_property_is_writeable,
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

	qg->design_uah = QG_DEFAULT_CAPACITY_UAH;
	if (qg->info && qg->info->charge_full_design_uah > 0)
		qg->design_uah = qg->info->charge_full_design_uah;
	qg->full_uah = qg->design_uah;

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

	queue_delayed_work(system_dfl_wq, &qg->work, msecs_to_jiffies(QG_POLL_MS));

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
