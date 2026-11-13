// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only SMB1360 fuel-gauge telemetry for the CAT S22.
 *
 * The vendor charger/fuel-gauge driver owns charger control.  This driver
 * deliberately exposes only the FG shadow registers: capacity, voltage,
 * current and temperature.  It never writes the charger configuration.
 */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define SMB1360_CMD_I2C_REG		0x40
#define SMB1360_FG_ACCESS_BIT		BIT(5)
#define SMB1360_IRQ_I_REG		0x58
#define SMB1360_FG_ACCESS_ALLOWED_BIT	BIT(0)

#define SMB1360_SHDW_FG_MSYS_SOC	0x61
#define SMB1360_SHDW_FG_CAPACITY	0x62
#define SMB1360_SHDW_FG_VTG_NOW		0x69
#define SMB1360_SHDW_FG_CURR_NOW	0x6B
#define SMB1360_SHDW_FG_BATT_TEMP	0x6D

struct smb1360_fg {
	struct i2c_client *client;
	struct power_supply *psy;
	struct mutex lock;
};

static int smb1360_fg_read(struct smb1360_fg *fg, u8 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2] = {
		{ .addr = fg->client->addr, .flags = 0, .len = 1, .buf = &reg },
		{ .addr = fg->client->addr, .flags = I2C_M_RD, .len = len, .buf = buf },
	};
	int ret;

	ret = i2c_transfer(fg->client->adapter, msgs, 2);
	if (ret < 0)
		return ret;
	return ret == 2 ? 0 : -EIO;
}

static int smb1360_fg_read8(struct smb1360_fg *fg, u8 reg, u8 *val)
{
	return smb1360_fg_read(fg, reg, val, 1);
}

static int smb1360_fg_read16(struct smb1360_fg *fg, u8 reg, u16 *val)
{
	u8 buf[2];
	int ret;

	ret = smb1360_fg_read(fg, reg, buf, sizeof(buf));
	if (ret)
		return ret;
	*val = buf[0] | (buf[1] << 8);
	return 0;
}

static int smb1360_fg_access(struct smb1360_fg *fg)
{
	u8 irq, cmd;
	int ret, i;

	ret = smb1360_fg_read8(fg, SMB1360_IRQ_I_REG, &irq);
	if (ret)
		return ret;
	if (irq & SMB1360_FG_ACCESS_ALLOWED_BIT)
		return 0;

	ret = smb1360_fg_read8(fg, SMB1360_CMD_I2C_REG, &cmd);
	if (ret)
		return ret;
	ret = i2c_smbus_write_byte_data(fg->client, SMB1360_CMD_I2C_REG,
					cmd | SMB1360_FG_ACCESS_BIT);
	if (ret)
		return ret;

	for (i = 0; i < 20; i++) {
		usleep_range(10000, 11000);
		ret = smb1360_fg_read8(fg, SMB1360_IRQ_I_REG, &irq);
		if (ret)
			return ret;
		if (irq & SMB1360_FG_ACCESS_ALLOWED_BIT)
			return 0;
	}
	return -ETIMEDOUT;
}

static int smb1360_fg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct smb1360_fg *fg = power_supply_get_drvdata(psy);
	u8 reg;
	u16 raw;
	u64 n;
	int ret;

	mutex_lock(&fg->lock);
	ret = smb1360_fg_access(fg);
	if (ret)
		goto out;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = smb1360_fg_read8(fg, SMB1360_SHDW_FG_MSYS_SOC, &reg);
		if (!ret)
			val->intval = min_t(int, 100, (100 * reg + 127) / 255);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = smb1360_fg_read16(fg, SMB1360_SHDW_FG_CAPACITY, &raw);
		if (!ret)
			val->intval = raw * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = smb1360_fg_read16(fg, SMB1360_SHDW_FG_VTG_NOW, &raw);
		if (!ret) {
			n = div_u64((u64)raw * 5000, 0x7fff);
			val->intval = n * 1000;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = smb1360_fg_read16(fg, SMB1360_SHDW_FG_CURR_NOW, &raw);
		if (!ret) {
			s64 curr = (s16)raw * 2500;

			val->intval = div_s64(curr, 0x7fff) * 1000;
		}
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = smb1360_fg_read16(fg, SMB1360_SHDW_FG_BATT_TEMP, &raw);
		if (!ret) {
			n = div_u64((u64)raw * 625, 10000);
			val->intval = ((int)n - 273) * 10;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
out:
	mutex_unlock(&fg->lock);
	return ret;
}

static enum power_supply_property smb1360_fg_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc smb1360_fg_desc = {
	.name = "smb1360-fg",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = smb1360_fg_props,
	.num_properties = ARRAY_SIZE(smb1360_fg_props),
	.get_property = smb1360_fg_get_property,
};

static int smb1360_fg_probe(struct i2c_client *client)
{
	struct power_supply_config cfg = {};
	struct smb1360_fg *fg;
	u8 irq;
	int ret;

	fg = devm_kzalloc(&client->dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;
	fg->client = client;
	mutex_init(&fg->lock);

	/* Fail fast instead of registering a battery that cannot answer I2C. */
	ret = smb1360_fg_read8(fg, SMB1360_IRQ_I_REG, &irq);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "SMB1360 does not answer at the stock address\n");

	cfg.drv_data = fg;
	cfg.fwnode = dev_fwnode(&client->dev);
	fg->psy = devm_power_supply_register(&client->dev, &smb1360_fg_desc, &cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(&client->dev, PTR_ERR(fg->psy),
				     "failed to register fuel gauge\n");

	i2c_set_clientdata(client, fg);
	dev_info(&client->dev, "SMB1360 read-only fuel gauge registered\n");
	return 0;
}

static const struct of_device_id smb1360_fg_of_match[] = {
	{ .compatible = "qcom,smb1360-chg-fg" },
	{}
};
MODULE_DEVICE_TABLE(of, smb1360_fg_of_match);

static const struct i2c_device_id smb1360_fg_id[] = {
	{ "smb1360-fg", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, smb1360_fg_id);

static struct i2c_driver smb1360_fg_driver = {
	.driver = {
		.name = "smb1360-fg",
		.of_match_table = smb1360_fg_of_match,
	},
	.probe = smb1360_fg_probe,
	.id_table = smb1360_fg_id,
};
module_i2c_driver(smb1360_fg_driver);

MODULE_DESCRIPTION("SMB1360 read-only fuel-gauge telemetry");
MODULE_LICENSE("GPL");
