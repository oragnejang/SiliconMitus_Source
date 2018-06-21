/*
 * sm5011-npwrstm.h
 *
 * Copyright (c) 2018 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_SM5011_nPWRSTM_H
#define __LINUX_MFD_SM5011_nPWRSTM_H

#include <linux/mfd/sm5011-core.h>
#include <linux/regulator/sm5011-regulator.h>

/*
 * struct sm5011_npwrstm_dev - sm5011's npwrstm function device
 * @iodev : pointer to pointer to master device
 */
struct sm5011_npwrstm_dev {
	struct sm5011_pmic_dev *iodev;
};

#define nPWRSTM_STATUS_OK			 1
#define nPWRSTM_STATUS_FAIL			(-1)
#define nPWRSTM_STATUS_UNSUPPORTED	(-2)

/* CNTL2 */
#define SM5011_ENnPWRSTM_SHIFT		3
#define SM5011_ENnPWRSTM_MASK		(1 << SM5011_ENnPWRSTM_SHIFT)

/* 
  * [ External Function ]
  */
extern int sm5011_npwrstm_get_bits(void);
extern int sm5011_npwrstm_gpio_init(struct sm5011_pmic_dev *iodev);
extern int sm5011_set_ennpwrstm(int val);

extern unsigned int sm5011_npwrstm_interface_init(struct sm5011_pmic_dev *sm5011_pmic);
extern unsigned int sm5011_npwrstm_preinit(void);

#endif /*  __LINUX_MFD_SM5011_nPWRSTM_H */
