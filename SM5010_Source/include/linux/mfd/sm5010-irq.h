/* sm5010-irq.h
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */


#ifndef __LINUX_MFD_SM5010_IRQ_H
#define __LINUX_MFD_SM5010_IRQ_H

enum sm5010_irq {
	SM5010_IRQ1_MICDET,
	SM5010_IRQ1_VBAT_VALID,
	SM5010_IRQ1_MANUALRST,
	SM5010_IRQ1_LONGKEY_MRSTB,
	SM5010_IRQ1_LONGKEY_CHGON,
	SM5010_IRQ1_LONGKEY_nONKEY,
	SM5010_IRQ1_SHORTKEY,

	SM5010_IRQ2_ALARM2_ON,
	SM5010_IRQ2_ALARM1_ON,
	SM5010_IRQ2_WDTMEROUT,

	SM5010_IRQ_NR,
};

#define SM5010_IRQ1_MICDET_NAME			"SM5010_MICDET"
#define SM5010_IRQ1_VBAT_VALID_NAME		"SM5010_VBAT_VALID"
#define SM5010_IRQ1_MANUALRST_NAME		"SM5010_MANUALRST"
#define SM5010_IRQ1_LONGKEY_MRSTB_NAME	"SM5010_LONGKEY_MRSTB"
#define SM5010_IRQ1_LONGKEY_CHGON_NAME	"SM5010_LONGKEY_CHGON"
#define SM5010_IRQ1_LONGKEY_nONKEY_NAME	"SM5010_LONGKEY_nONKEY"
#define SM5010_IRQ1_SHORTKEY_NAME		"SM5010_SHORTKEY"

#define SM5010_IRQ2_ALARM2_ON_NAME		"SM5010_ALARM2_ON"
#define SM5010_IRQ2_ALARM1_ON_NAME		"SM5010_ALARM1_ON"
#define SM5010_IRQ2_WDTMEROUT_NAME		"SM5010_WDTMEROUT"

#define SM5010_IRQ1_MICDET_MASK			(1 << 6)
#define SM5010_IRQ1_VBAT_VALID_MASK		(1 << 5)
#define SM5010_IRQ1_MANUALRST_MASK		(1 << 4)
#define SM5010_IRQ1_LONGKEY_MRSTB_MASK	(1 << 3)
#define SM5010_IRQ1_LONGKEY_CHGON_MASK	(1 << 2)
#define SM5010_IRQ1_LONGKEY_nONKEY_MASK	(1 << 1)
#define SM5010_IRQ1_SHORTKEY_MASK		(1 << 0)
 
#define SM5010_IRQ2_ALARM2_ON_MASK		(1 << 2)
#define SM5010_IRQ2_ALARM1_ON_MASK		(1 << 1)
#define SM5010_IRQ2_WDTMEROUT_MASK		(1 << 0)

struct sm5010_irq_handler {
	char *name;
	int irq_index;
	irqreturn_t (*handler)(int irq, void *data);
};

const char *sm5010_get_irq_name_by_index(int index);

/* 
  * [ External Function ]
  */
extern int sm5010_irq_init(struct sm5010_pmic_dev *sm5010_pmic);
extern void sm5010_irq_exit(struct sm5010_pmic_dev *sm5010_pmic);
#endif /*  __LINUX_MFD_SM5010_IRQ_H */
