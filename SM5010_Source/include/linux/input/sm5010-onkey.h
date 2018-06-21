/*
 * sm5010-onkey.h
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_INPUT_SM5010_H
#define __LINUX_INPUT_SM5010_H

/*
 * struct sm5010_onkey_info - sm5010's onkey device
 * @ iodev: master device of the chip
 * @ idev : point of input device
 * @ irq_base : base IRQ number for sm5010, required for IRQs
 * @ dev : point of onkey device 
 */
struct sm5010_onkey_info {
	struct sm5010_pmic_dev	*iodev;
	struct input_dev	*idev;
	int irq_base;
	struct device		*dev;
};

/* CNTL2 */
#define SM5010_LONGKEY_SHIFT		5
#define SM5010_LONGKEY_MASK			(7 << SM5010_LONGKEY_SHIFT)

/* MRSTBCNTL */
#define SM5010_MRSTBTMR_SHIFT		4
#define SM5010_MRSTBTMR_MASK		(7 << SM5010_MRSTBTMR_SHIFT)
#define SM5010_ENPMICOFF2ON_SHIFT	3
#define SM5010_ENPMICOFF2ON_MASK	(1 << SM5010_ENPMICOFF2ON_SHIFT)
#define SM5010_ENnRESETOFF2ON_SHIFT	2
#define SM5010_ENnRESETOFF2ON_MASK	(1 << SM5010_ENnRESETOFF2ON_SHIFT)
#define SM5010_KEYOPTION_SHIFT		1
#define SM5010_KEYOPTION_MASK		(1 << SM5010_KEYOPTION_SHIFT)
#define SM5010_ENMRSTB_SHIFT		0
#define SM5010_ENMRSTB_MASK			(1 << SM5010_ENMRSTB_SHIFT)
#endif /* __LINUX_INPUT_SM5010_H */
