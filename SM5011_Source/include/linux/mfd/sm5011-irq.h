/* sm5011-irq.h
 *
 * Copyright (c) 2018 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */


#ifndef __LINUX_MFD_SM5011_IRQ_H
#define __LINUX_MFD_SM5011_IRQ_H

enum sm5011_irq {
	SM5011_IRQ1_MICDET,
	SM5011_IRQ1_VBAT_VALID,
	SM5011_IRQ1_MANUALRST,
	SM5011_IRQ1_LONGKEY_MRSTB,
	SM5011_IRQ1_LONGKEY_CHGON,
	SM5011_IRQ1_LONGKEY_nONKEY,
	SM5011_IRQ1_SHORTKEY,

	SM5011_IRQ2_ALARM2_ON,
	SM5011_IRQ2_ALARM1_ON,
	SM5011_IRQ2_WDTMEROUT,

	SM5011_IRQ_NR,
};

#define SM5011_IRQ1_MICDET_NAME			"SM5011_MICDET"
#define SM5011_IRQ1_VBAT_VALID_NAME		"SM5011_VBAT_VALID"
#define SM5011_IRQ1_MANUALRST_NAME		"SM5011_MANUALRST"
#define SM5011_IRQ1_LONGKEY_MRSTB_NAME	"SM5011_LONGKEY_MRSTB"
#define SM5011_IRQ1_LONGKEY_CHGON_NAME	"SM5011_LONGKEY_CHGON"
#define SM5011_IRQ1_LONGKEY_nONKEY_NAME	"SM5011_LONGKEY_nONKEY"
#define SM5011_IRQ1_SHORTKEY_NAME		"SM5011_SHORTKEY"

#define SM5011_IRQ2_ALARM2_ON_NAME		"SM5011_ALARM2_ON"
#define SM5011_IRQ2_ALARM1_ON_NAME		"SM5011_ALARM1_ON"
#define SM5011_IRQ2_WDTMEROUT_NAME		"SM5011_WDTMEROUT"

#define SM5011_IRQ1_MICDET_MASK			(1 << 6)
#define SM5011_IRQ1_VBAT_VALID_MASK		(1 << 5)
#define SM5011_IRQ1_MANUALRST_MASK		(1 << 4)
#define SM5011_IRQ1_LONGKEY_MRSTB_MASK	(1 << 3)
#define SM5011_IRQ1_LONGKEY_CHGON_MASK	(1 << 2)
#define SM5011_IRQ1_LONGKEY_nONKEY_MASK	(1 << 1)
#define SM5011_IRQ1_SHORTKEY_MASK		(1 << 0)
 
#define SM5011_IRQ2_ALARM2_ON_MASK		(1 << 2)
#define SM5011_IRQ2_ALARM1_ON_MASK		(1 << 1)
#define SM5011_IRQ2_WDTMEROUT_MASK		(1 << 0)

struct sm5011_irq_handler {
	char *name;
	int irq_index;
	irqreturn_t (*handler)(int irq, void *data);
};

const char *sm5011_get_irq_name_by_index(int index);

/* 
  * [ External Function ]
  */
extern int sm5011_irq_init(struct sm5011_pmic_dev *sm5011_pmic);
extern void sm5011_irq_exit(struct sm5011_pmic_dev *sm5011_pmic);
#endif /*  __LINUX_MFD_SM5011_IRQ_H */
