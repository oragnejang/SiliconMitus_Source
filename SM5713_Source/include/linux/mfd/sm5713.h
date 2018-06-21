/*
 *  sm5713.h - mfd driver for SM5713.
 *
 *  Copyright (C) 2017 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SM5713_H__
#define __SM5713_H__
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define MFD_DEV_NAME "sm5713"

struct sm5713_regulator_data {
	int id;
	struct regulator_init_data *initdata;
	struct device_node *reg_node;
};

struct sm5713_platform_data {
	/* IRQ */
	int irq_base;
	int irq_gpio;
	bool wakeup;

    /* USBLDO */
    struct sm5713_regulator_data *regulators;
    int num_regulators;


	struct mfd_cell *sub_devices;
	int num_subdevs;
};

extern int sm5713_fled_prepare_flash(u8 index);
extern int sm5713_fled_torch_on(u8 index, u8 brightness);
extern int sm5713_fled_pre_flash_on(u8 index, u8 brightness);
extern int sm5713_fled_flash_on(u8 index, u8 brightness);
extern int sm5713_fled_led_off(u8 index);
extern int sm5713_fled_close_flash(u8 index);

#endif /* __SM5713_H__ */

