// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023, Nikita Travkin <nikita@trvn.ru>
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/extcon-provider.h>
#include <linux/math.h>
#include <linux/workqueue.h>

/* Two bytes: type + subtype */
#define PM8916_PERPH_TYPE 0x04
#define PM8916_LBC_CHGR_TYPE 0x1502
#define PM8916_LBC_BAT_IF_TYPE 0x1602
#define PM8916_LBC_USB_TYPE 0x1702
#define PM8916_LBC_MISC_TYPE 0x1802

#define PM8916_LBC_CHGR_CHG_OPTION 0x08
#define PM8916_LBC_CHGR_PMIC_CHARGER BIT(7)

#define PM8916_LBC_CHGR_CHG_STATUS 0x09

#define PM8916_INT_RT_STS 0x10

#define PM8916_LBC_USB_USBIN_VALID BIT(1)

#define PM8916_LBC_CHGR_VDD_MAX 0x40
#define PM8916_LBC_CHGR_VDD_SAFE 0x41
#define PM8916_LBC_CHGR_IBAT_MAX 0x44
#define PM8916_LBC_CHGR_IBAT_SAFE 0x45

#define PM8916_LBC_CHGR_TCHG_MAX_EN 0x60
#define PM8916_LBC_CHGR_TCHG_MAX_ENABLED BIT(7)
#define PM8916_LBC_CHGR_TCHG_MAX 0x61

#define PM8916_LBC_CHGR_CHG_CTRL 0x49
#define PM8916_LBC_CHGR_CHG_EN BIT(7)
#define PM8916_LBC_CHGR_PSTG_EN BIT(5)
#define PM8916_LBC_CHGR_FORCE_BATT_ON BIT(0)

#define PM8916_LBC_THERMAL_POLL_MS 30000
#define PM8916_LBC_THERMAL_FIRST_POLL_MS 5000
#define PM8916_LBC_THERMAL_WARM_DECIDEGC 450
#define PM8916_LBC_THERMAL_HOT_DECIDEGC 500
#define PM8916_LBC_THERMAL_RESUME_DECIDEGC 400
#define PM8916_LBC_THERMAL_HOT_RESUME_DECIDEGC 480

#define PM8916_LBC_CHGR_MIN_CURRENT 90000
#define PM8916_LBC_CHGR_MAX_CURRENT 1440000

#define PM8916_LBC_CHGR_MIN_VOLTAGE 4000000
#define PM8916_LBC_CHGR_MAX_VOLTAGE 4775000
#define PM8916_LBC_CHGR_VOLTAGE_STEP 25000

#define PM8916_LBC_CHGR_MIN_TIME 4
#define PM8916_LBC_CHGR_MAX_TIME 256

enum pm8916_lbc_thermal_state {
	PM8916_LBC_THERMAL_NORMAL,
	PM8916_LBC_THERMAL_WARM,
	PM8916_LBC_THERMAL_HOT,
	PM8916_LBC_THERMAL_FAULT,
};

struct pm8916_lbc_charger {
	struct device *dev;
	struct extcon_dev *edev;
	struct power_supply *charger;
	struct power_supply_battery_info *info;
	struct regmap *regmap;
	struct delayed_work thermal_work;
	struct mutex configure_lock;
	const char *battery_temp_supply;
	unsigned int reg[4];
	bool online;
	bool charge_enabled;
	enum pm8916_lbc_thermal_state thermal_state;
	unsigned int thermal_warm_decidegc;
	unsigned int thermal_hot_decidegc;
	unsigned int thermal_resume_decidegc;
	unsigned int thermal_hot_resume_decidegc;
	unsigned int charge_voltage_max;
	unsigned int charge_voltage_safe;
	unsigned int charge_current_max;
	unsigned int charge_current_safe;
	unsigned int charge_current_applied;
};


static const unsigned int pm8916_lbc_charger_cable[] = {
	EXTCON_USB,
	EXTCON_NONE,
};

enum {
	LBC_CHGR = 0,
	LBC_BAT_IF,
	LBC_USB,
	LBC_MISC,
};

static int __pm8916_lbc_charger_configure(struct pm8916_lbc_charger *chg)
{
	int ret = 0;
	unsigned int ibat, tmp;

	chg->charge_voltage_max = clamp_t(u32, chg->charge_voltage_max,
					  PM8916_LBC_CHGR_MIN_VOLTAGE, chg->charge_voltage_safe);

	tmp = chg->charge_voltage_max - PM8916_LBC_CHGR_MIN_VOLTAGE;
	tmp /= PM8916_LBC_CHGR_VOLTAGE_STEP;
	chg->charge_voltage_max = PM8916_LBC_CHGR_MIN_VOLTAGE + tmp * PM8916_LBC_CHGR_VOLTAGE_STEP;

	ret = regmap_write(chg->regmap, chg->reg[LBC_CHGR] + PM8916_LBC_CHGR_VDD_MAX, tmp);
	if (ret)
		goto error;

	ibat = min(chg->charge_current_max, chg->charge_current_safe);
	if (chg->thermal_state != PM8916_LBC_THERMAL_NORMAL)
		ibat = PM8916_LBC_CHGR_MIN_CURRENT;

	ibat = clamp_t(u32, ibat, PM8916_LBC_CHGR_MIN_CURRENT,
			  PM8916_LBC_CHGR_MAX_CURRENT);
	ibat /= PM8916_LBC_CHGR_MIN_CURRENT;
	chg->charge_current_applied = ibat * PM8916_LBC_CHGR_MIN_CURRENT;

	ret = regmap_write(chg->regmap, chg->reg[LBC_CHGR] + PM8916_LBC_CHGR_IBAT_MAX,
			   ibat - 1);
	if (ret)
		goto error;

	tmp = PM8916_LBC_CHGR_PSTG_EN;
	if (chg->charge_enabled)
		tmp |= PM8916_LBC_CHGR_CHG_EN;
	else
		tmp |= PM8916_LBC_CHGR_FORCE_BATT_ON;

	ret = regmap_write(chg->regmap, chg->reg[LBC_CHGR] + PM8916_LBC_CHGR_CHG_CTRL, tmp);
	if (ret)
		goto error;

	return 0;

error:
	dev_err(chg->dev, "Failed to configure charging: %pe\n", ERR_PTR(ret));
	return ret;
}

static int pm8916_lbc_charger_configure(struct pm8916_lbc_charger *chg)
{
	int ret;

	mutex_lock(&chg->configure_lock);
	ret = __pm8916_lbc_charger_configure(chg);
	mutex_unlock(&chg->configure_lock);

	return ret;
}


static int pm8916_lbc_charger_get_property(struct power_supply *psy,
					   enum power_supply_property psp,
					   union power_supply_propval *val)
{
	struct pm8916_lbc_charger *chg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chg->online;
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = chg->charge_voltage_max;
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = chg->charge_current_applied ?
			      chg->charge_current_applied : chg->charge_current_max;
		return 0;

	default:
		return -EINVAL;
	};
}

static int pm8916_lbc_charger_set_property(struct power_supply *psy,
					   enum power_supply_property prop,
					   const union power_supply_propval *val)
{
	struct pm8916_lbc_charger *chg = power_supply_get_drvdata(psy);
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		mutex_lock(&chg->configure_lock);
		chg->charge_current_max = val->intval;
		ret = __pm8916_lbc_charger_configure(chg);
		mutex_unlock(&chg->configure_lock);
		return ret;
	default:
		return -EINVAL;
	}
}


static int pm8916_lbc_charger_property_is_writeable(struct power_supply *psy,
						    enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return true;
	default:
		return false;
	}
}

static enum power_supply_property pm8916_lbc_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
};

static irqreturn_t pm8916_lbc_charger_state_changed_irq(int irq, void *data)
{
	struct pm8916_lbc_charger *chg = data;
	unsigned int tmp;
	int ret;

	ret = regmap_read(chg->regmap, chg->reg[LBC_USB] + PM8916_INT_RT_STS, &tmp);
	if (ret)
		return IRQ_HANDLED;

	chg->online = !!(tmp & PM8916_LBC_USB_USBIN_VALID);
	extcon_set_state_sync(chg->edev, EXTCON_USB, chg->online);

	power_supply_changed(chg->charger);

	return IRQ_HANDLED;
}

static const char *pm8916_lbc_thermal_state_name(enum pm8916_lbc_thermal_state state)
{
	switch (state) {
	case PM8916_LBC_THERMAL_NORMAL:
		return "normal";
	case PM8916_LBC_THERMAL_WARM:
		return "warm";
	case PM8916_LBC_THERMAL_HOT:
		return "hot";
	case PM8916_LBC_THERMAL_FAULT:
		return "fault";
	}

	return "unknown";
}

static enum pm8916_lbc_thermal_state
pm8916_lbc_thermal_next_state(struct pm8916_lbc_charger *chg, int temp)
{
	if (temp >= chg->thermal_hot_decidegc)
		return PM8916_LBC_THERMAL_HOT;

	if (temp <= chg->thermal_resume_decidegc)
		return PM8916_LBC_THERMAL_NORMAL;

	/*
	 * Re-enter WARM, not NORMAL, after a HOT event.  Under sustained
	 * system load the pack can sit in the high 40s and never reach the
	 * 40 C NORMAL resume point; WARM's 90 mA is already the policy for
	 * 45..50 C, so use a separate HOT->WARM release point.
	 */
	if (chg->thermal_state == PM8916_LBC_THERMAL_HOT) {
		if (temp <= chg->thermal_hot_resume_decidegc)
			return PM8916_LBC_THERMAL_WARM;
		return PM8916_LBC_THERMAL_HOT;
	}

	if (temp >= chg->thermal_warm_decidegc)
		return PM8916_LBC_THERMAL_WARM;

	if (chg->thermal_state == PM8916_LBC_THERMAL_WARM)
		return PM8916_LBC_THERMAL_WARM;

	return PM8916_LBC_THERMAL_NORMAL;
}

static int pm8916_lbc_read_battery_temp(struct pm8916_lbc_charger *chg, int *temp)
{
	struct power_supply *battery;
	union power_supply_propval val;
	int ret;

	battery = power_supply_get_by_name(chg->battery_temp_supply);
	if (!battery)
		return -ENODEV;

	ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_TEMP, &val);
	power_supply_put(battery);
	if (ret)
		return ret;

	*temp = val.intval;
	return 0;
}

static void pm8916_lbc_thermal_work(struct work_struct *work)
{
	struct pm8916_lbc_charger *chg = container_of(work, struct pm8916_lbc_charger,
						      thermal_work.work);
	enum pm8916_lbc_thermal_state old_state, new_state;
	bool old_enabled;
	int temp = 0, ret;
	bool temp_valid = false;

	ret = pm8916_lbc_read_battery_temp(chg, &temp);
	if (ret == -ENODEV) {
		dev_warn_once(chg->dev, "battery temperature supply not ready\n");
		new_state = PM8916_LBC_THERMAL_FAULT;
	} else if (ret) {
		dev_warn_once(chg->dev, "battery temperature read failed: %d\n", ret);
		new_state = PM8916_LBC_THERMAL_FAULT;
	} else {
		temp_valid = true;
		new_state = pm8916_lbc_thermal_next_state(chg, temp);
	}

	mutex_lock(&chg->configure_lock);
	old_state = chg->thermal_state;
	old_enabled = chg->charge_enabled;

	if (new_state != old_state) {
		chg->thermal_state = new_state;
		chg->charge_enabled = new_state != PM8916_LBC_THERMAL_HOT;

		ret = __pm8916_lbc_charger_configure(chg);
		if (ret) {
			chg->thermal_state = old_state;
			chg->charge_enabled = old_enabled;
		}
	}
	mutex_unlock(&chg->configure_lock);

	if (new_state != old_state && !ret) {
		int temp_abs = abs(temp);

		if (temp_valid)
			dev_info(chg->dev, "battery %s%d.%d C: %s, %u uA, charging %s\n",
				 temp < 0 ? "-" : "", temp_abs / 10, temp_abs % 10,
				 pm8916_lbc_thermal_state_name(new_state),
				 chg->charge_current_applied,
				 chg->charge_enabled ? "on" : "off");
		else
			dev_info(chg->dev, "temperature unavailable: %s, %u uA, charging %s\n",
				 pm8916_lbc_thermal_state_name(new_state),
				 chg->charge_current_applied,
				 chg->charge_enabled ? "on" : "off");
	}

	schedule_delayed_work(&chg->thermal_work,
			      msecs_to_jiffies(PM8916_LBC_THERMAL_POLL_MS));
}

static int pm8916_lbc_charger_probe_dt(struct pm8916_lbc_charger *chg)
{
	struct device *dev = chg->dev;
	int ret = 0;
	unsigned int tmp;

	chg->thermal_warm_decidegc = PM8916_LBC_THERMAL_WARM_DECIDEGC;
	chg->thermal_hot_decidegc = PM8916_LBC_THERMAL_HOT_DECIDEGC;
	chg->thermal_resume_decidegc = PM8916_LBC_THERMAL_RESUME_DECIDEGC;
	chg->thermal_hot_resume_decidegc = PM8916_LBC_THERMAL_HOT_RESUME_DECIDEGC;

	ret = device_property_read_string(dev, "qcom,battery-temperature-psy",
					  &chg->battery_temp_supply);
	if (ret && ret != -EINVAL)
		return ret;

	if (chg->battery_temp_supply) {
		ret = device_property_read_u32(dev, "qcom,thermal-warm-decidegc", &tmp);
		if (!ret)
			chg->thermal_warm_decidegc = tmp;

		ret = device_property_read_u32(dev, "qcom,thermal-hot-decidegc", &tmp);
		if (!ret)
			chg->thermal_hot_decidegc = tmp;

		ret = device_property_read_u32(dev, "qcom,thermal-resume-decidegc", &tmp);
		if (!ret)
			chg->thermal_resume_decidegc = tmp;

		ret = device_property_read_u32(dev, "qcom,thermal-hot-resume-decidegc", &tmp);
		if (!ret)
			chg->thermal_hot_resume_decidegc = tmp;

		if (chg->thermal_resume_decidegc >= chg->thermal_warm_decidegc ||
		    chg->thermal_warm_decidegc > chg->thermal_hot_resume_decidegc ||
		    chg->thermal_hot_resume_decidegc >= chg->thermal_hot_decidegc)
			return -EINVAL;
	}

	ret = device_property_read_u32(dev, "qcom,fast-charge-safe-voltage", &chg->charge_voltage_safe);

	if (ret)
		return ret;
	if (chg->charge_voltage_safe < PM8916_LBC_CHGR_MIN_VOLTAGE)
		return -EINVAL;

	chg->charge_voltage_safe = clamp_t(u32, chg->charge_voltage_safe,
					PM8916_LBC_CHGR_MIN_VOLTAGE, PM8916_LBC_CHGR_MAX_VOLTAGE);

	tmp = chg->charge_voltage_safe - PM8916_LBC_CHGR_MIN_VOLTAGE;
	tmp /= PM8916_LBC_CHGR_VOLTAGE_STEP;
	ret = regmap_write(chg->regmap, chg->reg[LBC_CHGR] + PM8916_LBC_CHGR_VDD_SAFE, tmp);
	if (ret)
		return ret;

	ret = device_property_read_u32(dev, "qcom,fast-charge-safe-current", &chg->charge_current_safe);
	if (ret)
		return ret;
	if (chg->charge_current_safe < PM8916_LBC_CHGR_MIN_CURRENT)
		return -EINVAL;

	chg->charge_current_safe = clamp_t(u32, chg->charge_current_safe,
					PM8916_LBC_CHGR_MIN_CURRENT, PM8916_LBC_CHGR_MAX_CURRENT);

	chg->charge_current_max = chg->charge_current_safe;

	tmp = chg->charge_current_safe / PM8916_LBC_CHGR_MIN_CURRENT - 1;
	ret = regmap_write(chg->regmap, chg->reg[LBC_CHGR] + PM8916_LBC_CHGR_IBAT_SAFE, tmp);
	if (ret)
		return ret;

	/* Disable charger timeout. */
	return regmap_write(chg->regmap, chg->reg[LBC_CHGR] + PM8916_LBC_CHGR_TCHG_MAX_EN, 0x00);
}

static const struct power_supply_desc pm8916_lbc_charger_psy_desc = {
	.name = "pm8916-lbc-chgr",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = pm8916_lbc_charger_properties,
	.num_properties = ARRAY_SIZE(pm8916_lbc_charger_properties),
	.get_property = pm8916_lbc_charger_get_property,
	.set_property = pm8916_lbc_charger_set_property,
	.property_is_writeable = pm8916_lbc_charger_property_is_writeable,
};

static int pm8916_lbc_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pm8916_lbc_charger *chg;
	struct power_supply_config psy_cfg = {};
	int ret, len, irq;
	unsigned int tmp;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	chg->thermal_state = PM8916_LBC_THERMAL_NORMAL;
	chg->charge_enabled = true;
	mutex_init(&chg->configure_lock);
	INIT_DELAYED_WORK(&chg->thermal_work, pm8916_lbc_thermal_work);
	platform_set_drvdata(pdev, chg);

	chg->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chg->regmap)
		return -ENODEV;

	len = device_property_count_u32(dev, "reg");
	if (len < 0)
		return len;
	if (len != 4)
		return dev_err_probe(dev, -EINVAL,
				     "Wrong amount of reg values: %d (4 expected)\n", len);

	ret = device_property_read_u32_array(dev, "reg", chg->reg, len);
	if (ret)
		return ret;

	ret = regmap_bulk_read(chg->regmap, chg->reg[LBC_CHGR] + PM8916_PERPH_TYPE, &tmp, 2);
	if (ret)
		goto comm_error;
	tmp &= 0xFFFF;
	if (tmp != PM8916_LBC_CHGR_TYPE)
		goto type_error;

	ret = regmap_bulk_read(chg->regmap, chg->reg[LBC_BAT_IF] + PM8916_PERPH_TYPE, &tmp, 2);
	if (ret)
		goto comm_error;
	tmp &= 0xFFFF;
	if (tmp != PM8916_LBC_BAT_IF_TYPE)
		goto type_error;

	ret = regmap_bulk_read(chg->regmap, chg->reg[LBC_USB] + PM8916_PERPH_TYPE, &tmp, 2);
	if (ret)
		goto comm_error;
	tmp &= 0xFFFF;
	if (tmp != PM8916_LBC_USB_TYPE)
		goto type_error;

	ret = regmap_bulk_read(chg->regmap, chg->reg[LBC_MISC] + PM8916_PERPH_TYPE, &tmp, 2);
	if (ret)
		goto comm_error;
	tmp &= 0xFFFF;
	if (tmp != PM8916_LBC_MISC_TYPE)
		goto type_error;

	ret = regmap_read(chg->regmap, chg->reg[LBC_CHGR] + PM8916_LBC_CHGR_CHG_OPTION, &tmp);
	if (ret)
		goto comm_error;
	if (tmp != PM8916_LBC_CHGR_PMIC_CHARGER)
		dev_err_probe(dev, -ENODEV, "The system is using an external charger\n");

	ret = pm8916_lbc_charger_probe_dt(chg);
	if (ret)
		return dev_err_probe(dev, ret, "Error while parsing device tree\n");

	psy_cfg.drv_data = chg;
	psy_cfg.fwnode = dev_fwnode(dev);

	chg->charger = devm_power_supply_register(dev, &pm8916_lbc_charger_psy_desc, &psy_cfg);
	if (IS_ERR(chg->charger))
		return dev_err_probe(dev, PTR_ERR(chg->charger), "Unable to register charger\n");

	ret = power_supply_get_battery_info(chg->charger, &chg->info);
	if (ret)
		return dev_err_probe(dev, ret, "Unable to get battery info\n");

	irq = platform_get_irq_byname(pdev, "usb_vbus");
	if (irq < 0)
		return irq;

	chg->edev = devm_extcon_dev_allocate(dev, pm8916_lbc_charger_cable);
	if (IS_ERR(chg->edev))
		return PTR_ERR(chg->edev);

	ret = devm_extcon_dev_register(dev, chg->edev);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to register extcon device\n");

	ret = devm_request_threaded_irq(dev, irq, NULL, pm8916_lbc_charger_state_changed_irq,
					IRQF_ONESHOT, "pm8916_lbc", chg);
	if (ret)
		return ret;

	ret = regmap_read(chg->regmap, chg->reg[LBC_USB] + PM8916_INT_RT_STS, &tmp);
	if (ret)
		goto comm_error;

	chg->online = !!(tmp & PM8916_LBC_USB_USBIN_VALID);
	extcon_set_state_sync(chg->edev, EXTCON_USB, chg->online);

	chg->charge_voltage_max = chg->info->voltage_max_design_uv;
	ret = pm8916_lbc_charger_configure(chg);
	if (ret)
		return ret;

	if (chg->battery_temp_supply)
		schedule_delayed_work(&chg->thermal_work,
				      msecs_to_jiffies(PM8916_LBC_THERMAL_FIRST_POLL_MS));

	return 0;

comm_error:
	return dev_err_probe(dev, ret, "Unable to communicate with device\n");

type_error:
	return dev_err_probe(dev, -ENODEV, "Device reported wrong type: 0x%X\n", tmp);
}

static void pm8916_lbc_charger_remove(struct platform_device *pdev)
{
	struct pm8916_lbc_charger *chg = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&chg->thermal_work);
	mutex_destroy(&chg->configure_lock);
}

static const struct of_device_id pm8916_lbc_charger_of_match[] = {
	{ .compatible = "qcom,pm8916-lbc", },
	{}
};
MODULE_DEVICE_TABLE(of, pm8916_lbc_charger_of_match);

static struct platform_driver pm8916_lbc_charger_driver = {
	.driver = {
		.name = "pm8916-lbc",
		.of_match_table = pm8916_lbc_charger_of_match,
	},
	.probe = pm8916_lbc_charger_probe,
	.remove = pm8916_lbc_charger_remove,
};

module_platform_driver(pm8916_lbc_charger_driver);

MODULE_DESCRIPTION("pm8916 LBC driver");
MODULE_AUTHOR("Nikita Travkin <nikita@trvn.ru>");
MODULE_LICENSE("GPL");
