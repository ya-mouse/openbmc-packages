/*
 * Hardware monitoring driver for TI TPS2480 / TPS2481
 *
 * Copyright (c) 2014-2015 AppearTV AS
 *
 * Parts of this file are based from:
 *    ads1015.c (platform data/of config reading)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include "tps2480.h"

#define TPS2480_REG_CONFIG        0x00 /* R/W */
#define TPS2480_REG_SHUNT_VOLTAGE 0x01 /* R   */
#define TPS2480_REG_BUS_VOLTAGE   0x02 /* R   */
#define TPS2480_REG_POWER         0x03 /* R   */
#define TPS2480_REG_CURRENT       0x04 /* R   */
#define TPS2480_REG_CALIBRATION   0x05 /* R/W */
#define TPS2480_MAX_REGISTERS     6

enum tps2480_chips {
	tps2480,
	tps2481,
};

struct tps2480_data {
	struct regmap  *regmap;
	int             shunt;
} tps2480_data;

static bool is_readable_reg(struct device *dev, unsigned int reg)
{
	return reg < TPS2480_MAX_REGISTERS;
}

static bool is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TPS2480_REG_CONFIG:
	case TPS2480_REG_CALIBRATION:
		return true;
	}

	return false;
}

static bool is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TPS2480_REG_SHUNT_VOLTAGE:
	case TPS2480_REG_BUS_VOLTAGE:
	case TPS2480_REG_POWER:
	case TPS2480_REG_CURRENT:
		return true;
	}

	return false;
}

static const struct reg_default tps2480_reg_defaults[] = {
	{TPS2480_REG_CONFIG, 0x399f},
	{TPS2480_REG_POWER, 0x0000},
	{TPS2480_REG_CURRENT, 0x0000},
	{TPS2480_REG_CALIBRATION, 0x0000},
};

static const struct regmap_config tps2480_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.writeable_reg = is_writeable_reg,
	.readable_reg = is_readable_reg,
	.volatile_reg = is_volatile_reg,
	.max_register = TPS2480_MAX_REGISTERS,
	.reg_defaults = tps2480_reg_defaults,
	.num_reg_defaults = 4,
	.cache_type = REGCACHE_FLAT,
};

/*
 * Helper functions
 */

static struct tps2480_data *get_data(struct device *dev)
{
	return i2c_get_clientdata(to_i2c_client(dev));
}

static int init_chip(struct tps2480_data *data)
{
	return regmap_write(data->regmap, TPS2480_REG_CONFIG, 1 << 15);
}

static int extract_bus_voltage(unsigned int value)
{
	return (int)(value >> 3);
}

static int extract_shunt_voltage(unsigned int value)
{
	return (int)((s16)value);
}

/*
 * Sysfs callbacks
 */

static ssize_t show_in0(struct device *dev, struct device_attribute *attr, char
*buf)
{
	/* Read bus voltage */
	unsigned int value;
	int signed_value;
	int err = regmap_read(get_data(dev)->regmap,
		TPS2480_REG_BUS_VOLTAGE,
		&value);
	if (err)
		return err;

	/* Step size is 4mV */
	signed_value = extract_bus_voltage(value) * 4;

	return snprintf(buf, PAGE_SIZE, "%d\n", signed_value);
}

static ssize_t show_in1(struct device *dev, struct device_attribute *attr,
char *buf)
{
	/* Read shunt voltage */
	unsigned int value;
	int signed_value;
	int err = regmap_read(get_data(dev)->regmap,
		TPS2480_REG_SHUNT_VOLTAGE,
		&value);
	if (err)
		return err;

	/* Step size is 10uV */
	signed_value = extract_shunt_voltage(value) / 100;

	return snprintf(buf, PAGE_SIZE, "%d\n", signed_value);
}

static ssize_t show_curr1(struct device *dev, struct device_attribute *attr,
char *buf)
{
	/* Measured current is computed by dividing the shunt voltage by
	 * the shunt resistance.
	 */

	/* Read shunt voltage */
	unsigned int value;
	int signed_value;
	struct tps2480_data *data = get_data(dev);
	int err = regmap_read(data->regmap, TPS2480_REG_SHUNT_VOLTAGE, &value);

	if (err)
		return err;

	/* Shunt voltage has 10uV step
	 * Shunt resistance has 1 mOhm step
	 */
	signed_value = extract_shunt_voltage(value) * 10 / data->shunt;

	return snprintf(buf, PAGE_SIZE, "%d\n", signed_value);
}

static ssize_t show_power1(struct device *dev, struct device_attribute *attr,
char *buf)
{
	/* Measured power is computed by multiplying the bus voltage by
	 * the current passing through the shunt resistor.
	 *
	 * V = bus voltage
	 * U = shunt voltage
	 * R = shunt resistance
	 * I = shunt current
	 *
	 * Power = V * I = V * (U/R)
	 */

	/* Read bus & shunt voltage */
	unsigned int value;
	int signed_value;
	int bus_voltage;
	int shunt_voltage;
	int err;
	struct tps2480_data *data = get_data(dev);

	err = regmap_read(data->regmap, TPS2480_REG_BUS_VOLTAGE, &value);
	if (err)
		return err;

	bus_voltage = extract_bus_voltage(value);

	err = regmap_read(data->regmap, TPS2480_REG_SHUNT_VOLTAGE, &value);
	if (err)
		return err;

	shunt_voltage = extract_shunt_voltage(value);

	/* Bus voltage has 4mV step
	 * Shunt voltage has 10uV step
	 * Shunt resistance has 1 mOhm step
	 */
	signed_value = (bus_voltage * shunt_voltage * 40) / data->shunt;

	return snprintf(buf, PAGE_SIZE, "%d\n", signed_value);
}

static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, show_in0, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, show_in1, NULL, 1);
static SENSOR_DEVICE_ATTR(curr1_input, S_IRUGO, show_curr1, NULL, 1);
static SENSOR_DEVICE_ATTR(power1_input, S_IRUGO, show_power1, NULL, 1);

static struct attribute *tps2480_attrs[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_curr1_input.dev_attr.attr,
	&sensor_dev_attr_power1_input.dev_attr.attr,
	NULL,
};

static struct attribute_group tps2480_attr_grp = {
	.attrs = tps2480_attrs,
};

static const struct attribute_group *tps2480_attr_grps[] = {
	&tps2480_attr_grp,
	NULL,
};

/*
 * Driver interface functions
 */

#ifdef CONFIG_OF
static int tps2480_read_config_of(struct i2c_client *client,
	struct tps2480_data *data)
{
	int err;
	unsigned int value;

	if (!client->dev.of_node) {
		dev_err(&client->dev, "cannot find of node");
		return -EINVAL;
	}

	err = of_property_read_u32(client->dev.of_node, "shunt", &value);
	data->shunt = (int)value;

	return err;
}
#endif

static int tps2480_read_config(struct i2c_client *client,
	struct tps2480_data *data)
{
	struct tps2480_platform_data *pdata = dev_get_platdata(&client->dev);
#ifdef CONFIG_OF
	int err;
#endif

	if (pdata) {
		data->shunt = (int)pdata->shunt;
		return 0;
	}

#ifdef CONFIG_OF
	err = tps2480_read_config_of(client, data);
	if (err)
		return err;
#endif
	return 0;
}

static int tps2480_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct tps2480_data *data;
	struct device *hwmon_dev;
	int err;

	data = devm_kzalloc(&client->dev,
						sizeof(struct tps2480_data),
						GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);

	err = tps2480_read_config(client, data);
	if (err)
		return err;

	/* Check that shunt is != 0. */
	if (data->shunt == 0) {
		dev_err(&client->dev, "shunt cannot be 0");
		return -EINVAL;
	}

	/* Initialize the register map which accesses the chip registers. */
	data->regmap = devm_regmap_init_i2c(client, &tps2480_regmap_config);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	/* Initialize the chip to its default configuration. */
	err = init_chip(data);
	if (err)
		return err;

	/* Registers the device to appear in sysfs. */
	hwmon_dev = devm_hwmon_device_register_with_groups(&client->dev,
		client->name,
		data,
		tps2480_attr_grps);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id tps2480_id[] = {
	{"tps2480", tps2480},
	{"tps2481", tps2481},
	{},
};

MODULE_DEVICE_TABLE(i2c, tps2480_id);

static struct i2c_driver tps2480_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		   .name = "tps2480",
		   },
	.probe = tps2480_probe,
	.id_table = tps2480_id,
};

module_i2c_driver(tps2480_driver);

MODULE_AUTHOR("Hadrien Copponnex <hadrien.copponnex@xxxxxxxxxxxx>");
MODULE_DESCRIPTION("I2C/SMBus Driver for TI TPS2480/TPS2481");
MODULE_LICENSE("GPL");
