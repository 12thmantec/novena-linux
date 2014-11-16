/*
 *   Copyright (C) 2012 will.niu@pitayatech.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __NOVDS_GPIO_DRIVER_H__
#define __NOVDS_GPIO_DRIVER_H__

#include <linux/fs.h>
#include <linux/types.h>
#include <linux/magic.h>

/* gpio port operation code */
typedef enum {
    NOVDS_GPIO_DRIVER_READ,
    NOVDS_GPIO_DRIVER_WRITE,
} novds_gpio_driver_opcode_t;

typedef struct {
    novds_gpio_driver_opcode_t opcode;
    int data;
} novds_gpio_driver_t;

#define NOVDS_GPIO_DRIVER_IOC_ZERO_VALVE     _IOWR('m', 1, novds_gpio_driver_t)
#define NOVDS_GPIO_DRIVER_IOC_WATER_FULL     _IOWR('m', 2, novds_gpio_driver_t)
#define NOVDS_GPIO_DRIVER_IOC_BACKUP_SWITCH  _IOWR('m', 3, novds_gpio_driver_t)
#define NOVDS_GPIO_DRIVER_IOC_DEILVERY_POWER _IOWR('m', 4, novds_gpio_driver_t)

#endif
