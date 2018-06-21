/*
 * sm5010-npwrstm.h
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_SM5010_nPWRSTM_H
#define __LINUX_MFD_SM5010_nPWRSTM_H

#include <linux/mfd/sm5010-core.h>
#include <linux/regulator/sm5010-regulator.h>

/*
 * struct sm5010_npwrstm_dev - sm5010's npwrstm function device
 * @iodev : pointer to pointer to master device
 */
struct sm5010_npwrstm_dev {
	struct sm5010_pmic_dev *iodev;
};

#define nPWRSTM_STATUS_OK			 1
#define nPWRSTM_STATUS_FAIL			(-1)
#define nPWRSTM_STATUS_UNSUPPORTED	(-2)

/* CNTL2 */
#define SM5010_ENnPWRSTM_SHIFT		3
#define SM5010_ENnPWRSTM_MASK		(1 << SM5010_ENnPWRSTM_SHIFT)

/* SM5010 Buck1 nPWRSTM regulator ids */
enum sm5010_buck1npwrstm {
	SM5010_BUCK1nPWRSTM, /* nPWRSTM */
 
	SM5010_BUCK1nPWRSTMMAX,
};

/* 
  * [ External Function ]
  */
extern int sm5010_npwrstm_get_pins(void);
extern int sm5010_npwrstm_set_pins(bool val);
extern int sm5010_npwrstm_get_bits(void);
extern int sm5010_npwrstm_gpio_init(struct sm5010_pmic_dev *iodev);
extern int sm5010_set_ennpwrstm(int val);
extern unsigned int sm5010_npwrstm_buck1enable(void);


extern unsigned int sm5010_npwrstm_setvolt(unsigned int lvl, int uV);
extern unsigned int sm5010_npwrstm_getvolt(unsigned int lvl, int* uV);

extern unsigned int sm5010_npwrstm_interface_init(struct sm5010_pmic_dev *sm5010_pmic);
extern unsigned int sm5010_npwrstm_preinit(void);

#endif /*  __LINUX_MFD_SM5010_nPWRSTM_H */
