/*
 * Copyright (C) 2014 12thmantec Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define NOVDS_GPIO_DRIVER_VERSION    "1.1"

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/system.h>
#include <linux/gpio.h>
#include "novds_gpio_driver.h"

#define MK_GPIO(base, nr) (((base) - 1) * 32 + (nr))

struct novds_gpio_cfg {
    int gpio; // gpio port
    int dir; // the direction. 1 for output, 0 for input
    int defval; // the default value of gpio port, just for gpio output
    const char *desc;
};

static struct novds_gpio_cfg g_novds_gpios[] = {
    {0, 0, 0, NULL}, //unused
    {MK_GPIO(4, 26), 1, 1, "ZERO_VALVE"}, //zero valve
    {MK_GPIO(2, 22), 0, 0, "WATER_FULL"}, // water full detector
    {MK_GPIO(6, 2),  0, 0, "BACKUP_SWITCH"}, // backup switch detector
    {MK_GPIO(4, 19), 1, 0, "DELIVERY_POWER"}, // power enable for delivery board
};

#define NOVDS_GPIOS_SIZE (sizeof(g_novds_gpios) / sizeof(g_novds_gpios[0]))

static int g_novds_gpio_open_cnt = 0;

/*
 * The are the file operation function for user access to /dev/novds_gpio
 */

static loff_t novds_gpio_llseek(struct file *file, loff_t offset, int origin)
{
    return -EOPNOTSUPP;
}

static ssize_t novds_gpio_read(struct file *file, char __user *buf,
                        size_t count, loff_t *ppos)
{
    return -EOPNOTSUPP;
}

static ssize_t novds_gpio_write(struct file *file, const char __user *buf,
                        size_t count, loff_t *ppos)
{
    return -EOPNOTSUPP;
}

static int novds_gpio_ops(int nr, void __user *argp)
{
    novds_gpio_driver_t gpio;

    if (copy_from_user(&gpio, argp, sizeof(novds_gpio_driver_t)))
        return -EFAULT;

    switch (gpio.opcode) {
    case NOVDS_GPIO_DRIVER_READ:
        gpio.data = gpio_get_value(g_novds_gpios[nr].gpio);
        if (copy_to_user(argp, &gpio, sizeof(novds_gpio_driver_t)))
            return -EFAULT;
        break;
    case NOVDS_GPIO_DRIVER_WRITE:
        gpio_set_value(g_novds_gpios[nr].gpio, gpio.data);
        break;
    }
    return 0;
}
static long novds_gpio_ioctl(struct file *file, unsigned int cmd,
            unsigned long arg)
{
    int ret;
    int nr = _IOC_NR(cmd);
    void __user *argp = (void __user *)arg;

    if (nr <= 0 || nr >= NOVDS_GPIOS_SIZE)
        return -EOPNOTSUPP;

    if (nr > 0 && nr < NOVDS_GPIOS_SIZE)
        ret = novds_gpio_ops(nr, argp);
    else
        ret = -EOPNOTSUPP;

    return ret;
}

static int novds_gpio_open(struct inode *inode, struct file *file)
{
    g_novds_gpio_open_cnt++;

    return 0;
}

static int novds_gpio_release(struct inode *inode, struct file *file)
{
    g_novds_gpio_open_cnt--;
    return 0;
}

static int novds_gpio_setup(void)
{
    int i;

    for (i = 1; i < NOVDS_GPIOS_SIZE; i++) {
        if (gpio_request(g_novds_gpios[i].gpio, g_novds_gpios[i].desc)) {
            printk(KERN_ERR "failed to request gpio %s\n", g_novds_gpios[i].desc);
            continue;
        }

        if (g_novds_gpios[i].dir)
            gpio_direction_output(g_novds_gpios[i].gpio, g_novds_gpios[i].defval);
        else
            gpio_direction_input(g_novds_gpios[i].gpio);
    }

    return 0;
}

static const struct file_operations novds_gpio_fops = {
    .owner          = THIS_MODULE,
    .llseek         = novds_gpio_llseek,
    .read           = novds_gpio_read,
    .write          = novds_gpio_write,
    .unlocked_ioctl = novds_gpio_ioctl,
    .open           = novds_gpio_open,
    .release        = novds_gpio_release,
};

static struct miscdevice novds_gpio_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "novds_gpio",
    .fops  = &novds_gpio_fops
};

static int __init novds_gpio_init(void)
{
    int ret;

    novds_gpio_setup();

    ret = misc_register(&novds_gpio_dev);
    if (ret) {
        printk(KERN_ERR "novds_gpio: can't misc_register\n");
        goto out;
    }
    ret = 0;
    printk(KERN_INFO "NOVDS GPIO driver v" NOVDS_GPIO_DRIVER_VERSION "\n");
out:
    return ret;
}

static void __exit novds_gpio_exit(void)
{
    misc_deregister(&novds_gpio_dev);
}

module_init(novds_gpio_init);
module_exit(novds_gpio_exit);

MODULE_AUTHOR("Will <niutao0602@163.com>");
MODULE_DESCRIPTION("driver for NOventDS");
MODULE_LICENSE("GPL");
