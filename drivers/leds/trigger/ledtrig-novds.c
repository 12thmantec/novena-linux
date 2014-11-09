/*
 * LED Kernel Timer Trigger
 *
 * Copyright 2005-2006 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include "../leds.h"

enum novds_leds_sys_color {
    NOVDS_LEDS_COLOR_RED = 0x5,
    NOVDS_LEDS_COLOR_GREEN = 0x6,
    NOVDS_LEDS_COLOR_BLUE = 0x3,
    NOVDS_LEDS_COLOR_WHITE = 0x0,
    NOVDS_LEDS_COLOR_ORANGE = 0x4,
    NOVDS_LEDS_COLOR_OFF = 0x7,
};

struct timer_trig_data {
	int brightness_on;		/* LED brightness during "on" period.
					 * (LED_OFF < brightness_on <= LED_FULL)
					 */
	unsigned long delay_on;		/* milliseconds on */
	unsigned long delay_off;	/* milliseconds off */
	unsigned long on_off;
	unsigned int on_data;
	unsigned int off_data;
	unsigned long state;
	unsigned int *gpio2_data;
	unsigned int *gpio5_data;
	struct timer_list timer;
	struct led_classdev *led_cdev;
};

static struct timer_trig_data *g_timer_data;
int novds_kernel_set_led(int on_data, int state);
/*
 * the bit map of state
 *
 * BIT0 LED1（GREEN） EIM_LBA(GPIO2_IO27)
 * BIT1 LED2（RED）   DISP0_DAT11(GPIO5_IO05)
 * BIT2 LED3（BLUE）  EIM_RW(GPIO2_IO26)
 *
 */
static void novds_led_set_state(unsigned int state)
{
	struct timer_trig_data *timer_data = g_timer_data;
    unsigned int data2, reg2;
    unsigned int data5, reg5;

    data2 = ((state & 0x1) << 27) | (((state & 0x4) >> 2) << 26);
    reg2 = *(timer_data->gpio2_data);
    reg2 &= 0xF3FFFFFF;
    reg2 |= data2;
    *(timer_data->gpio2_data) = reg2;

    data5 = (((state & 0x2) >> 1) << 5);
    reg5 = *(timer_data->gpio5_data);
    reg5 &= 0xFFFFFFDF;
    reg5 |= data5;
    *(timer_data->gpio5_data) = reg5;
}

static void led_novds_function(unsigned long data)
{
	struct led_classdev *led_cdev = (struct led_classdev *) data;
	struct timer_trig_data *timer_data = led_cdev->trigger_data;
	unsigned long brightness;
	unsigned long delay;


	if (!timer_data->delay_on || !timer_data->delay_off) {
		if (!timer_data->on_off) {
			led_cdev->brightness = LED_OFF;
            novds_led_set_state(NOVDS_LEDS_COLOR_OFF);
		}
		return;
	}

	brightness = led_get_brightness(led_cdev);
	if (!brightness) {
		/* Time to switch the LED on. */
		brightness = timer_data->brightness_on;
		delay = timer_data->delay_on;
	} else {
		/* Store the current brightness value to be able
		 * to restore it when the delay_off period is over.
		 */
		timer_data->brightness_on = brightness;
		brightness = LED_OFF;
		delay = timer_data->delay_off;
	}

	led_cdev->brightness = brightness;

	if (brightness)
        novds_led_set_state(timer_data->on_data);
	else
        novds_led_set_state(timer_data->off_data);

	mod_timer(&timer_data->timer, jiffies + msecs_to_jiffies(delay));
}

static ssize_t led_delay_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct timer_trig_data *timer_data = led_cdev->trigger_data;

	return sprintf(buf, "%lu\n", timer_data->delay_on);
}

static ssize_t led_delay_on_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct timer_trig_data *timer_data = led_cdev->trigger_data;
	unsigned long state = simple_strtoul(buf, NULL, 10);

	timer_data->delay_on = state;

	return size;
}

static ssize_t led_delay_off_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct timer_trig_data *timer_data = led_cdev->trigger_data;

	return sprintf(buf, "%lu\n", timer_data->delay_off);
}
static ssize_t led_on_off_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct timer_trig_data *timer_data = led_cdev->trigger_data;

	return sprintf(buf, "%lu\n", timer_data->state);
}


static ssize_t led_delay_off_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct timer_trig_data *timer_data = led_cdev->trigger_data;
	unsigned long state = simple_strtoul(buf, NULL, 10);

	timer_data->delay_off = state;

	return size;
}
static ssize_t led_on_off_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int state;
	unsigned int on_data;

	if (sscanf(buf, "%u %d", &on_data, &state) != 2)
		return -EINVAL;

    novds_kernel_set_led(on_data, state);

	return size;
}

int novds_kernel_set_led(int on_data, int state)
{
	struct timer_trig_data *timer_data = g_timer_data;
	struct led_classdev *led_cdev = timer_data->led_cdev;

	on_data &= 0x7;

	del_timer_sync(&timer_data->timer);
	switch (state) {
	case 0: /* off */
		led_cdev->brightness = LED_OFF;
        novds_led_set_state(NOVDS_LEDS_COLOR_OFF);
		break;
	case 1: /* on */
		led_cdev->brightness = LED_HALF;
        novds_led_set_state(on_data);
		break;
	case 2: /* flashing */
		led_cdev->brightness = LED_HALF;
        novds_led_set_state(on_data);
		timer_data->on_data = on_data;
		timer_data->off_data = NOVDS_LEDS_COLOR_OFF;
		mod_timer(&timer_data->timer, jiffies + 1);
		break;
	default:
		return -EINVAL;
	}

	timer_data->state = state;

	return 0;

}
EXPORT_SYMBOL(novds_kernel_set_led);

static DEVICE_ATTR(delay_on, 0644, led_delay_on_show, led_delay_on_store);
static DEVICE_ATTR(delay_off, 0644, led_delay_off_show, led_delay_off_store);
static DEVICE_ATTR(on_off, 0644, led_on_off_show, led_on_off_store);

static void timer_trig_activate(struct led_classdev *led_cdev)
{
	struct timer_trig_data *timer_data;
	int rc;
    struct device_node *np;
    void __iomem *base;

	timer_data = kzalloc(sizeof(struct timer_trig_data), GFP_KERNEL);
	if (!timer_data)
		return;

	g_timer_data = timer_data;

    np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-gpio");
    if (!np) {
        pr_warn("failed to find ocotp node\n");
        goto err_out;
    }
    
    // get the base address of gpio2, it should be the address of data register
    base = of_iomap(np, 1);
	//timer_data->gpio2_data = (unsigned int *)(IO_ADDRESS(0x020A0000));
	timer_data->gpio2_data = (unsigned int *)base;
    if (!base) {
        pr_warn("Failed the get the base address of gpio2\n");
        goto err_out;
    }
    base = of_iomap(np, 4);
	//timer_data->gpio5_data = (unsigned int *)(IO_ADDRESS(0x020AC000));
	timer_data->gpio5_data = (unsigned int *)base;
    if (!base) {
        pr_warn("Failed the get the base address of gpio5\n");
        goto err_out;
    }

	timer_data->brightness_on = led_get_brightness(led_cdev);
	if (timer_data->brightness_on == LED_OFF)
		timer_data->brightness_on = led_cdev->max_brightness;
	led_cdev->trigger_data = timer_data;

	init_timer(&timer_data->timer);
	timer_data->timer.function = led_novds_function;
	timer_data->timer.data = (unsigned long) led_cdev;
	timer_data->led_cdev = led_cdev;

	rc = device_create_file(led_cdev->dev, &dev_attr_delay_on);
	if (rc)
		goto err_out;
	rc = device_create_file(led_cdev->dev, &dev_attr_delay_off);
	if (rc)
		goto err_out_delayon;

	rc = device_create_file(led_cdev->dev, &dev_attr_on_off);
	if (rc)
		goto err_out_delayon1;

	/* If there is hardware support for blinking, start one
	 * user friendly blink rate chosen by the driver.
	 */
	if (led_cdev->blink_set)
		led_cdev->blink_set(led_cdev,
			&timer_data->delay_on, &timer_data->delay_off);

	return;

err_out_delayon1:
	device_remove_file(led_cdev->dev, &dev_attr_delay_off);
err_out_delayon:
	device_remove_file(led_cdev->dev, &dev_attr_delay_on);
err_out:
	led_cdev->trigger_data = NULL;
	kfree(timer_data);
}

static void timer_trig_deactivate(struct led_classdev *led_cdev)
{
	struct timer_trig_data *timer_data = led_cdev->trigger_data;
	unsigned long on = 0, off = 0;

	if (timer_data) {
		device_remove_file(led_cdev->dev, &dev_attr_delay_on);
		device_remove_file(led_cdev->dev, &dev_attr_delay_off);
		device_remove_file(led_cdev->dev, &dev_attr_on_off);
		del_timer_sync(&timer_data->timer);
		kfree(timer_data);
	}

	/* If there is hardware support for blinking, stop it */
	if (led_cdev->blink_set)
		led_cdev->blink_set(led_cdev, &on, &off);
}

static struct led_trigger timer_led_trigger = {
	.name     = "novds",
	.activate = timer_trig_activate,
	.deactivate = timer_trig_deactivate,
};

static int __init timer_trig_init(void)
{
	return led_trigger_register(&timer_led_trigger);
}

static void __exit timer_trig_exit(void)
{
	led_trigger_unregister(&timer_led_trigger);
}

module_init(timer_trig_init);
module_exit(timer_trig_exit);

MODULE_AUTHOR("Will <niutao0602@163.com>");
MODULE_DESCRIPTION("Timer LED trigger");
MODULE_LICENSE("GPL");
