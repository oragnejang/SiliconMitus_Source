/*
 * sm5010-cntl.h
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_SM5010_CNTL_H
#define __LINUX_MFD_SM5010_CNTL_H

#include <linux/mfd/sm5010-core.h>
#include <linux/regulator/sm5010-regulator.h>

/*
 * struct sm5010_cntl_dev - sm5010's cntl function device
 * @iodev : pointer to pointer to master device
 */
struct sm5010_cntl_dev {
	struct sm5010_pmic_dev *iodev;
};

#define CNTL_STATUS_OK			 1
#define CNTL_STATUS_FAIL			(-1)
#define CNTL_STATUS_UNSUPPORTED	(-2)

/* CNTL1 */
#define SM5010_SMPLTMR_SHIFT		1
#define SM5010_SMPLTMR_MASK			(3 << SM5010_SMPLTMR_SHIFT)
#define SM5010_SMPLAUTO_SHIFT		0
#define SM5010_SMPLAUTO_MASK		(1 << SM5010_SMPLAUTO_SHIFT)

/* CNTL2 */
#define SM5010_MASK_INT_SHIFT		0
#define SM5010_MASK_INT_MASK		(1 << SM5010_MASK_INT_SHIFT)
#define SM5010_SOFTRESET_SHIFT		1
#define SM5010_SOFTRESET_MASK		(1 << SM5010_SOFTRESET_SHIFT)
#define SM5010_HARDRESET_SHIFT		2
#define SM5010_HARDRESET_MASK		(1 << SM5010_HARDRESET_SHIFT)
#define SM5010_ENCNTL_SHIFT			3
#define SM5010_ENCNTL_MASK			(1 << SM5010_ENCNTL_SHIFT)
#define SM5010_GLOBALSHDN_SHIFT		5
#define SM5010_GLOBALSHDN_MASK		(7 << SM5010_GLOBALSHDN_SHIFT)

/* CNTL3 */
#define SM5010_ENVBATNG_SHIFT		1
#define SM5010_ENVBATNG_MASK		(3 << SM5010_ENVBATNG_SHIFT)
#define SM5010_ENVREF_SHIFT			0
#define SM5010_ENVREF_MASK			(1 << SM5010_ENVREF_SHIFT)

/* 
  * [ External Function ]
  */
extern int sm5010_set_smpltmr(int val);
extern int sm5010_set_smpl_auto(int val);
extern int sm5010_set_globalshdn(int val);
extern int sm5010_set_hardreset(int val);
extern int sm5010_set_softreset(int val);
extern int sm5010_set_mask_int(int val);
extern int sm5010_set_envbatng(int val);
extern int sm5010_set_envref(int val);

extern unsigned int sm5010_cntl_interface_init(struct sm5010_pmic_dev *sm5010_pmic);
extern unsigned int sm5010_cntl_preinit(void);

#endif /*  __LINUX_MFD_SM5010_CNTL_H */

