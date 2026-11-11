// SPDX-License-Identifier: GPL-2.0-only
/* CAT S22 read-only voltage/temperature observation, not a charger or SOC estimator.
 * QCOM SPMI VADC processed values are microvolts for VBAT and millidegrees C
 * for the ratiometric battery thermistor.
 */
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/power_supply.h>

struct cat_battery {
	struct iio_channel *vbat;
	struct iio_channel *temp;
};

static enum power_supply_property cat_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static int cat_battery_get(struct power_supply *psy,
			   enum power_supply_property prop,
			   union power_supply_propval *val)
{
	struct cat_battery *bat = power_supply_get_drvdata(psy);
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = iio_read_channel_processed(bat->vbat, &val->intval);
		if (ret < 0)
			return ret;
		if (val->intval < 0)
			return -ERANGE;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		if (!bat->temp)
			return -ENODATA;
		ret = iio_read_channel_processed(bat->temp, &val->intval);
		if (ret < 0)
			return ret;
		/* IIO temperature is millidegrees C; power_supply uses tenths. */
		val->intval /= 100;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc cat_battery_desc = {
	.name = "cat-s22-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = cat_battery_props,
	.num_properties = ARRAY_SIZE(cat_battery_props),
	.get_property = cat_battery_get,
};

static int cat_battery_probe(struct platform_device *pdev)
{
	struct power_supply_config cfg = {};
	struct cat_battery *bat;
	struct power_supply *psy;

	bat = devm_kzalloc(&pdev->dev, sizeof(*bat), GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	bat->vbat = devm_iio_channel_get(&pdev->dev, "vbat");
	if (IS_ERR(bat->vbat))
		return dev_err_probe(&pdev->dev, PTR_ERR(bat->vbat),
				     "VBAT ADC unavailable\n");

	bat->temp = devm_iio_channel_get(&pdev->dev, "temp");
	if (IS_ERR(bat->temp)) {
		if (PTR_ERR(bat->temp) != -ENODEV)
			return dev_err_probe(&pdev->dev, PTR_ERR(bat->temp),
					     "battery temperature ADC unavailable\n");
		bat->temp = NULL;
	}

	cfg.drv_data = bat;
	cfg.fwnode = dev_fwnode(&pdev->dev);
	psy = devm_power_supply_register(&pdev->dev, &cat_battery_desc, &cfg);
	return PTR_ERR_OR_ZERO(psy);
}

static const struct of_device_id cat_battery_match[] = {
	{ .compatible = "cat,s22flip-battery-voltage" }, {}
};
MODULE_DEVICE_TABLE(of, cat_battery_match);

static struct platform_driver cat_battery_driver = {
	.probe = cat_battery_probe,
	.driver = {
		.name = "cat-s22-battery-voltage",
		.of_match_table = cat_battery_match,
	},
};
module_platform_driver(cat_battery_driver);

MODULE_DESCRIPTION("CAT S22 read-only battery voltage and temperature");
MODULE_LICENSE("GPL");
