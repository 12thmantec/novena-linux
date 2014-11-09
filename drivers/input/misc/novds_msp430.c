/*
 * Driver for power board
 *
 * Copyright (C) 2014 12thmantec Inc.  All rights reserved.
 *
 * Author: Will <niutao0602@163.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License v2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 021110-1307, USA.
 *
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#define MSP430_NAME "msp430-keys"

/* the i2c addresses of power board */
#define NOVDS_MSP430_NONE               0x00
#define NOVDS_MSP430_MODE               0x81
#define NOVDS_MSP430_ADC_H              0x82
#define NOVDS_MSP430_ADC_L              0x83
#define NOVDS_MSP430_AC_ONLINE          0x84
#define NOVDS_MSP430_POWER_KEY          0x85
#define NOVDS_MSP430_POWER_OFF          0x86

struct novds_msp430 {
	struct i2c_client *client;
	struct input_dev *input;

    int irq_gpio;
    int irq;
};

static struct novds_msp430 g_novds_msp430;

/*
 * Write a byte to the MSP430
 */
static int msp430_write_byte(struct novds_msp430 *pdata,
			      int reg, u8 val)
{
	int error;

	error = i2c_smbus_write_byte_data(pdata->client, reg, val);
	if (error < 0) {
		dev_err(&pdata->client->dev,
			"%s failed, reg: %d, val: %d, error: %d\n",
			__func__, reg, val, error);
		return error;
	}

	return 0;
}

/*
 * Read a byte from the MSP430
 */
static int msp430_read_byte(struct novds_msp430 *pdata,
			     int reg, u8 *val)
{
	int error;

	error = i2c_smbus_read_byte_data(pdata->client, reg);
	if (error < 0) {
		dev_err(&pdata->client->dev,
				"%s failed, reg: %d, error: %d\n",
				__func__, reg, error);
		return error;
	}

	*val = (u8)error;

	return 0;
}

/*
 * Threaded IRQ handler and this can (and will) sleep.
 */
static irqreturn_t msp430_irq_handler(int irq, void *dev_id)
{
	struct novds_msp430 *pdata = dev_id;
	u8 reg;
	int error;

	error = msp430_read_byte(pdata, NOVDS_MSP430_POWER_KEY, &reg);
	if (error) {
		dev_err(&pdata->client->dev,
			"unable to read NOVDS_MSP430_POWER_KEY\n");
		return IRQ_NONE;
	}

	if (!reg)
		return IRQ_HANDLED;

    switch (reg) {
    case 1:
        input_report_key(pdata->input, KEY_F9, 1);
	    input_sync(pdata->input);
        input_report_key(pdata->input, KEY_F9, 0);
	    input_sync(pdata->input);
        break;
    case 2:
        input_report_key(pdata->input, KEY_F6, 1);
	    input_sync(pdata->input);
        input_report_key(pdata->input, KEY_F6, 0);
	    input_sync(pdata->input);
        break;
    }

	return IRQ_HANDLED;
}

static ssize_t novds_msp430_show_aconline(struct kobject *kobj, 
        struct kobj_attribute *attr, char *buf)  
{  
	u8 reg = 1;

	msp430_read_byte(&g_novds_msp430, NOVDS_MSP430_AC_ONLINE, &reg);

    return sprintf(buf, "%d", !!reg);
}  
static ssize_t novds_msp430_show_battery(struct kobject *kobj, 
        struct kobj_attribute *attr, char *buf)  
{  
	u8 h, l;
    unsigned short battery = 0;
    static int cache = 0;

	msp430_read_byte(&g_novds_msp430, NOVDS_MSP430_ADC_H, &h);
	msp430_read_byte(&g_novds_msp430, NOVDS_MSP430_ADC_L, &l);

    battery = (h << 8) | (l << 0);

    if (battery == 0)
        battery = cache;

    cache = battery;

    return sprintf(buf, "%d", battery);
}  


#define MSP430_ATTR(name, mode, show, store) \
    struct kobj_attribute msp430_attr_##name = __ATTR(name, mode, show, store);

static MSP430_ATTR(aconline, 0444, novds_msp430_show_aconline, NULL);
static MSP430_ATTR(battery,  0444, novds_msp430_show_battery, NULL);

static struct kobject *novds_msp430_kobj;

static struct attribute *novds_msp430_attrs[] = {
        &msp430_attr_aconline.attr,
        &msp430_attr_battery.attr,
        NULL,   /* need to NULL terminate the list of attributes */
};

static struct attribute_group novds_msp430_attr_group = {
        .attrs = novds_msp430_attrs,
};

static int novds_msp430_probe(struct i2c_client *client,
					  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct novds_msp430 *pdata;
	struct input_dev *input;
	int error;
	struct device_node *np = dev->of_node;

	/* Check i2c driver capabilities */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(dev, "%s adapter not supported\n",
			dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}

	pdata = &g_novds_msp430;

    /* This interrupt is from MSP430, the pin CPU_INT, just for power key */
    pdata->irq_gpio = of_get_named_gpio(np, "irq-gpios", 0);

	pdata->client = client;

	/* Configure input device */
	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	pdata->input = input;

    pdata->input->evbit[0] = BIT_MASK(EV_KEY);
    /* for power key */
    set_bit(KEY_F6, pdata->input->keybit);
    /* for standy key */
    set_bit(KEY_F9, pdata->input->keybit);

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->id.vendor  = 0x0001;
	input->id.product = 0x001;
	input->id.version = 0x0001;

	input_set_drvdata(input, pdata);

    error = devm_gpio_request_one(dev, pdata->irq_gpio,        
                    GPIOF_DIR_IN, "msp430");
    if (error) {
            dev_err(dev, "failed to request IRQ GPIO: %d\n",
                                error);
                return error;
    }

    pdata->irq = gpio_to_irq(pdata->irq_gpio);

	error = devm_request_threaded_irq(dev, pdata->irq, NULL, msp430_irq_handler,
					    IRQF_TRIGGER_RISING |
						IRQF_ONESHOT,
					    client->name, pdata);
	if (error) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			client->irq, error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		return error;
	}

    novds_msp430_kobj = kobject_create_and_add("novds_msp430", kernel_kobj);
    error = sysfs_create_group(novds_msp430_kobj, &novds_msp430_attr_group);

	return 0;
}

static const struct i2c_device_id msp430_id[] = {
	{ MSP430_NAME, 114, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, msp430_id);

#ifdef CONFIG_OF
static const struct of_device_id msp430_dt_ids[] = {
	{ .compatible = "novds,msp430", },
	{ }
};
MODULE_DEVICE_TABLE(of, msp430_dt_ids);
#endif

static struct i2c_driver novds_msp430_driver = {
	.driver = {
		.name	= MSP430_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(msp430_dt_ids),
	},
	.probe		= novds_msp430_probe,
	.id_table	= msp430_id,
};

static int __init novds_msp430_init(void)
{
	return i2c_add_driver(&novds_msp430_driver);
}
subsys_initcall(novds_msp430_init);

static void __exit novds_msp430_exit(void)
{
	i2c_del_driver(&novds_msp430_driver);
}
module_exit(novds_msp430_exit);

MODULE_AUTHOR("Will <niutao0602@163.com>");
MODULE_DESCRIPTION("key driver for MSP430");
MODULE_LICENSE("GPL");
