/*
 * NOventDS board support code
 *
 * Copyright (C) 2011-2012 will.niu
 * Written by will.niu <niutao0602@gmail.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

static struct gpio_led novds_leds[] = {
	{
		.name = "colors",
		.gpio = 58,
		.default_trigger = "novds",
		.active_low = 1,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data novds_leds_data = {
	.num_leds = ARRAY_SIZE(novds_leds),
	.leds = novds_leds,
};

static struct platform_device novds_leds_dev = {
	.name = "leds-novds",
	.id = -1,
	.dev = {
		.platform_data = &novds_leds_data,
	},
};

static int __init novds_leds_init(void)
{
	return platform_device_register(&novds_leds_dev);
}
static void __exit novds_leds_exit(void)
{
	platform_device_unregister(&novds_leds_dev);
}

module_init(novds_leds_init);
module_exit(novds_leds_exit);

MODULE_AUTHOR("Will <niutao0602@163.com>");
MODULE_DESCRIPTION("NOventDS/COventDS LEDS module");
MODULE_LICENSE("GPL");
