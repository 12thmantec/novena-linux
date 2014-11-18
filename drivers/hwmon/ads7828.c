/*
 * ads7828.c - driver for TI ADS7828 8-channel A/D converter and compatibles
 * (C) 2007 EADS Astrium
 *
 * This driver is based on the lm75 and other lm_sensors/hwmon drivers
 *
 * Written by Steve Hardy <shardy@redhat.com>
 *
 * ADS7830 support, by Guillaume Roguez <guillaume.roguez@savoirfairelinux.com>
 *
 * For further information, see the Documentation/hwmon/ads7828 file.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_data/ads7828.h>
#include <linux/slab.h>
#include <linux/kthread.h>

#define __ADS7828_USE_THREAD

/* The ADS7828 registers */
#define ADS7828_NCH		5	/* 8 channels supported, but we only use 5 in NOventDS */
#define ADS7828_CMD_SD_SE	0x80	/* Single ended inputs */
#define ADS7828_CMD_PD1		0x04	/* Internal vref OFF && A/D ON */
#define ADS7828_CMD_PD3		0x0C	/* Internal vref ON && A/D ON */
#define ADS7828_INT_VREF_MV	2500	/* Internal vref is 2.5V, 2500mV */
#define ADS7828_EXT_VREF_MV_MIN	50	/* External vref min value 0.05V */
#define ADS7828_EXT_VREF_MV_MAX	5250	/* External vref max value 5.25V */

/* List of supported devices */
enum ads7828_chips { ads7828, ads7830 };

/* Client specific data */
struct ads7828_data {
	struct device *hwmon_dev;
	struct mutex update_lock;	/* Mutex protecting updates */
	unsigned long last_updated;	/* Last updated time (in jiffies) */
	u16 adc_input[ADS7828_NCH];	/* ADS7828_NCH samples */
	bool valid;			/* Validity flag */
	bool diff_input;		/* Differential input */
	bool ext_vref;			/* External voltage reference */
	unsigned int vref_mv;		/* voltage reference value */
	u8 cmd_byte;			/* Command byte without channel bits */
	unsigned int lsb_resol;		/* Resolution of the ADC sample LSB */
	s32 (*read_channel)(const struct i2c_client *client, u8 command);
#ifdef __ADS7828_USE_THREAD
	struct task_struct *thread;
#endif
};

/* Command byte C2,C1,C0 - see datasheet */
static inline u8 ads7828_cmd_byte(u8 cmd, int ch)
{
	return cmd | (((ch >> 1) | (ch & 0x01) << 2) << 4);
}

#ifndef __ADS7828_USE_THREAD
/* Update data for the device (all 8 channels) */
static struct ads7828_data *ads7828_update_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ads7828_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
			|| !data->valid) {
		unsigned int ch;
		dev_dbg(&client->dev, "Starting ads7828 update\n");

		for (ch = 0; ch < ADS7828_NCH; ch++) {
			u8 cmd = ads7828_cmd_byte(data->cmd_byte, ch);
			data->adc_input[ch] = data->read_channel(client, cmd);
		}
		data->last_updated = jiffies;
		data->valid = true;
	}

	mutex_unlock(&data->update_lock);

	return data;
}
#endif

/* sysfs callback function */
static ssize_t ads7828_show_in(struct device *dev, struct device_attribute *da,
			       char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
#ifdef __ADS7828_USE_THREAD
	struct i2c_client *client = to_i2c_client(dev);
	struct ads7828_data *data = i2c_get_clientdata(client);
#else
	struct ads7828_data *data = ads7828_update_device(dev);
#endif

	unsigned int value = DIV_ROUND_CLOSEST(data->adc_input[attr->index] *
					       data->lsb_resol, 1000);

	return sprintf(buf, "%d\n", value);
}
static ssize_t show_in_all(struct device *dev, struct device_attribute *da,
	char *buf)
{
#ifdef __ADS7828_USE_THREAD
	struct i2c_client *client = to_i2c_client(dev);
	struct ads7828_data *data = i2c_get_clientdata(client);
#else
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ads7828_data *data = ads7828_update_device(dev);
#endif

	/* Print value (in mV as specified in sysfs-interface documentation) */
	return sprintf(buf, "%d %d %d %d %d %d %d %d\n", 
			(data->adc_input[0] * data->lsb_resol) / 1000,
			(data->adc_input[1] * data->lsb_resol) / 1000,
			(data->adc_input[2] * data->lsb_resol) / 1000,
			(data->adc_input[3] * data->lsb_resol) / 1000,
			(data->adc_input[4] * data->lsb_resol) / 1000,
            0,
			0,
			0
			);
}

static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, ads7828_show_in, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, ads7828_show_in, NULL, 1);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, ads7828_show_in, NULL, 2);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, ads7828_show_in, NULL, 3);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, ads7828_show_in, NULL, 4);
static SENSOR_DEVICE_ATTR(in5_input, S_IRUGO, ads7828_show_in, NULL, 5);
static SENSOR_DEVICE_ATTR(in6_input, S_IRUGO, ads7828_show_in, NULL, 6);
static SENSOR_DEVICE_ATTR(in7_input, S_IRUGO, ads7828_show_in, NULL, 7);
static SENSOR_DEVICE_ATTR(all_input, S_IRUGO, show_in_all, NULL, 8);

static struct attribute *ads7828_attributes[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in7_input.dev_attr.attr,
	&sensor_dev_attr_all_input.dev_attr.attr,
	NULL
};

static const struct attribute_group ads7828_group = {
	.attrs = ads7828_attributes,
};

static int ads7828_remove(struct i2c_client *client)
{
	struct ads7828_data *data = i2c_get_clientdata(client);

#ifdef __ADS7828_USE_THREAD
	kthread_stop(data->thread);
#endif
	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &ads7828_group);

	return 0;
}
#ifdef __ADS7828_USE_THREAD
struct sma_struct {
	int *averages;
	int size;
	int last_sum;
	int last_average;
	int update_index;
};

struct sma_struct *sma_init(int size, int init_data)
{
	struct sma_struct *sma;
	int i;

	sma = (struct sma_struct *)kzalloc(sizeof(struct sma_struct), GFP_KERNEL);

	sma->averages = (int *)kzalloc(sizeof(int) * size, GFP_KERNEL);

	for (i = 0; i < size; i++)
		sma->averages[i] = init_data;

	sma->size = size;
	sma->last_sum = init_data * size;
	sma->last_average = init_data;
	sma->update_index = 0;

	return sma;
}
void sma_exit(struct sma_struct *sma)
{
	kfree(sma->averages);
	kfree(sma);
}
int sma_calc(struct sma_struct *sma, int data)
{
	int oldest1;
	int oldest2;
	int sum;
#ifdef DEBUG
	int i;
#endif

	oldest1 = sma->averages[sma->update_index];

	if (sma->update_index == sma->size - 1)
		oldest2 = sma->averages[0];
	else
		oldest2 = sma->averages[sma->update_index + 1];

	sma->averages[sma->update_index] = sma->last_average;

	sma->last_sum = sma->last_sum - oldest1 + sma->last_average;
	sum = sma->last_sum + data - oldest2;

	sma->last_average = sum / sma->size;
#ifdef DEBUG
	for (i = 0; i < sma->size; i++)
		printf("%d ", sma->averages[i]);
#endif

	sma->update_index = (sma->update_index + 1 == sma->size) ? 0 : sma->update_index + 1;

	return sma->last_average;
}

static int ads7828_update_data(void *arg)
{
	struct sma_struct *sma;
	unsigned long last_jiffies;
	struct i2c_client *client = (struct i2c_client *)arg;
	struct ads7828_data *data = i2c_get_clientdata(client);
	int period = msecs_to_jiffies(2000);
	s32 adc_data;
	u8 cmd;
	int ch;

#define NO_CHANNEL 3
	cmd = ads7828_cmd_byte(data->cmd_byte, NO_CHANNEL);
	adc_data = data->read_channel(client, cmd);
	sma = sma_init(20, adc_data);

	last_jiffies = jiffies;

	while (!kthread_should_stop()) {
		if (jiffies - last_jiffies > period) {
			last_jiffies = jiffies;
			for (ch = 0; ch < ADS7828_NCH; ch++) {
				if (ch == NO_CHANNEL)
					continue;

	            cmd = ads7828_cmd_byte(data->cmd_byte, ch);
	            adc_data = data->read_channel(client, cmd);
				if (adc_data < 0)
					continue;
				data->adc_input[ch] = adc_data;
			}
		}
	    cmd = ads7828_cmd_byte(data->cmd_byte, NO_CHANNEL);
	    adc_data = data->read_channel(client, cmd);
		if (adc_data >= 0) {
			data->adc_input[NO_CHANNEL] = sma_calc(sma, adc_data);
		}
		//printk("adc_data = %d, average = %d\n", adc_data, data->adc_input[NO_CHANNEL]);

		schedule_timeout_interruptible(msecs_to_jiffies(50));
	}
	sma_exit(sma);
	return 0;
}
#endif

static int ads7828_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ads7828_platform_data *pdata = dev_get_platdata(&client->dev);
	struct ads7828_data *data;
    struct device_node *np = client->dev.of_node;
	int err;

	data = devm_kzalloc(&client->dev, sizeof(struct ads7828_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (!pdata) {
		if (!np)
			return -EINVAL;

		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

        pdata->diff_input = of_property_read_bool(np, "diff_input");
        pdata->ext_vref = of_property_read_bool(np, "ext_vref");
	    of_property_read_u32(np, "vref_mv",	&pdata->vref_mv);
	}


	if (pdata) {
		data->diff_input = pdata->diff_input;
		data->ext_vref = pdata->ext_vref;
		if (data->ext_vref)
			data->vref_mv = pdata->vref_mv;
	}

	/* Bound Vref with min/max values if it was provided */
	if (data->vref_mv)
		data->vref_mv = clamp_val(data->vref_mv,
					  ADS7828_EXT_VREF_MV_MIN,
					  ADS7828_EXT_VREF_MV_MAX);
	else
		data->vref_mv = ADS7828_INT_VREF_MV;

	/* ADS7828 uses 12-bit samples, while ADS7830 is 8-bit */
	if (id->driver_data == ads7828) {
		data->lsb_resol = DIV_ROUND_CLOSEST(data->vref_mv * 1000, 4096);
		data->read_channel = i2c_smbus_read_word_swapped;
	} else {
		data->lsb_resol = DIV_ROUND_CLOSEST(data->vref_mv * 1000, 256);
		data->read_channel = i2c_smbus_read_byte_data;
	}

	data->cmd_byte = data->ext_vref ? ADS7828_CMD_PD1 : ADS7828_CMD_PD3;
	if (!data->diff_input)
		data->cmd_byte |= ADS7828_CMD_SD_SE;

	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);

#ifdef __ADS7828_USE_THREAD
	data->thread = kthread_run(ads7828_update_data, client, "ads7828");
#endif
	err = sysfs_create_group(&client->dev.kobj, &ads7828_group);
	if (err)
		return err;

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto error;
	}

	return 0;

error:
	sysfs_remove_group(&client->dev.kobj, &ads7828_group);
	return err;
}

static const struct i2c_device_id ads7828_device_ids[] = {
	{ "ads7828", ads7828 },
	{ "ads7830", ads7830 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ads7828_device_ids);

static struct i2c_driver ads7828_driver = {
	.driver = {
		.name = "ads7828",
	},

	.id_table = ads7828_device_ids,
	.probe = ads7828_probe,
	.remove = ads7828_remove,
};

module_i2c_driver(ads7828_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Steve Hardy <shardy@redhat.com>");
MODULE_DESCRIPTION("Driver for TI ADS7828 A/D converter and compatibles");
