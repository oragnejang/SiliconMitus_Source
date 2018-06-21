/*
 * sm5010-dvs.h
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_SM5010_DVS_H
#define __LINUX_MFD_SM5010_DVS_H

#include <linux/mfd/sm5010-core.h>
#include <linux/regulator/sm5010-regulator.h>

#define DVS_STATUS_OK			 1
#define DVS_STATUS_FAIL			(-1)
#define DVS_STATUS_UNSUPPORTED	(-2)

/*
 * struct sm5010_dvs_dev - sm5010's dvs function device
 * @iodev : pointer to pointer to master device
 */
struct sm5010_dvs_dev {
	struct sm5010_pmic_dev *iodev;
};

/* SM5010 Buck1 DVS regulator ids */
enum sm5010_buck1dvs {
	SM5010_BUCK1DVS_0, /* LLL */
	SM5010_BUCK1DVS_1, /* LLH */
	SM5010_BUCK1DVS_2, /* LHL */
	SM5010_BUCK1DVS_3, /* LHH */
	SM5010_BUCK1DVS_4, /* HLL */
	SM5010_BUCK1DVS_5, /* HLH */
	SM5010_BUCK1DVS_6, /* HHL */
	SM5010_BUCK1DVS_7, /* HHH */
 
	SM5010_BUCK1DVSMAX,
};

/* 
  * [ External Function ]
  */
extern int sm5010_dvs_gpio_init(struct sm5010_pmic_dev *iodev);
extern int sm5010_dvs_get_dvspins(void);
extern int sm5010_dvs_set_dvspins(bool dvs1_val, bool dvs2_val, bool dvs3_val);

extern unsigned int sm5010_dvs_buck1enable(void);
extern unsigned int sm5010_dvs_buck1disable(void);

extern unsigned int sm5010_dvs_setvolt(unsigned int lvl, int uV);
extern unsigned int sm5010_dvs_getvolt(unsigned int lvl, int* uV);
extern unsigned int sm5010_dvs_enshedding(int enable);
extern unsigned int sm5010_dvs_freq(int hz);
extern unsigned int sm5010_dvs_dvsrampb1(int slope);
extern unsigned int sm5010_dvs_enrampb1down(int enable);
extern unsigned int sm5010_dvs_buck1adisen(int enable);
extern unsigned int sm5010_dvs_buck1mode(int mode);
extern unsigned int sm5010_dvs_interface_init(struct sm5010_pmic_dev *sm5010_pmic);
extern unsigned int sm5010_dvs_preinit(void);

#endif /*  __LINUX_MFD_SM5010_DVS_H */
