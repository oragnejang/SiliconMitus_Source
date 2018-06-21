/*
 * sm5011-onkey.h
 *
 * Copyright (c) 2018 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_INPUT_SM5011_H
#define __LINUX_INPUT_SM5011_H

/*
 * struct sm5011_onkey_info - sm5011's onkey device
 * @ iodev: master device of the chip
 * @ idev : point of input device
 * @ irq_base : base IRQ number for sm5011, required for IRQs
 * @ dev : point of onkey device 
 */
struct sm5011_onkey_info {
	struct sm5011_pmic_dev	*iodev;
	struct input_dev	*idev;
	int irq_base;
	struct device		*dev;
};

/* CNTL2 */
#define SM5011_LONGKEY_SHIFT		5
#define SM5011_LONGKEY_MASK			(7 << SM5011_LONGKEY_SHIFT)

/* MRSTBCNTL */
#define SM5011_MRSTBTMR_SHIFT		4
#define SM5011_MRSTBTMR_MASK		(7 << SM5011_MRSTBTMR_SHIFT)
#define SM5011_ENPMICOFF2ON_SHIFT	3
#define SM5011_ENPMICOFF2ON_MASK	(1 << SM5011_ENPMICOFF2ON_SHIFT)
#define SM5011_ENnRESETOFF2ON_SHIFT	2
#define SM5011_ENnRESETOFF2ON_MASK	(1 << SM5011_ENnRESETOFF2ON_SHIFT)
#define SM5011_KEYOPTION_SHIFT		1
#define SM5011_KEYOPTION_MASK		(1 << SM5011_KEYOPTION_SHIFT)
#define SM5011_ENMRSTB_SHIFT		0
#define SM5011_ENMRSTB_MASK			(1 << SM5011_ENMRSTB_SHIFT)
#endif /* __LINUX_INPUT_SM5011_H */
