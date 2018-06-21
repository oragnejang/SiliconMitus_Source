/*
 * sm5011-cntl.h
 *
 * Copyright (c) 2018 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_SM5011_CNTL_H
#define __LINUX_MFD_SM5011_CNTL_H

#include <linux/mfd/sm5011-core.h>
#include <linux/regulator/sm5011-regulator.h>

/*
 * struct sm5011_cntl_dev - sm5011's cntl function device
 * @iodev : pointer to pointer to master device
 */
struct sm5011_cntl_dev {
	struct sm5011_pmic_dev *iodev;
};

#define CNTL_STATUS_OK			 1
#define CNTL_STATUS_FAIL			(-1)
#define CNTL_STATUS_UNSUPPORTED	(-2)

/* CNTL1 */
#define SM5011_SMPLTMR_SHIFT		1
#define SM5011_SMPLTMR_MASK			(3 << SM5011_SMPLTMR_SHIFT)
#define SM5011_SMPLAUTO_SHIFT		0
#define SM5011_SMPLAUTO_MASK		(1 << SM5011_SMPLAUTO_SHIFT)

/* CNTL2 */
#define SM5011_MASK_INT_SHIFT		0
#define SM5011_MASK_INT_MASK		(1 << SM5011_MASK_INT_SHIFT)
#define SM5011_SOFTRESET_SHIFT		1
#define SM5011_SOFTRESET_MASK		(1 << SM5011_SOFTRESET_SHIFT)
#define SM5011_HARDRESET_SHIFT		2
#define SM5011_HARDRESET_MASK		(1 << SM5011_HARDRESET_SHIFT)
#define SM5011_ENCNTL_SHIFT			3
#define SM5011_ENCNTL_MASK			(1 << SM5011_ENCNTL_SHIFT)
#define SM5011_GLOBALSHDN_SHIFT		4
#define SM5011_GLOBALSHDN_MASK		(1 << SM5011_GLOBALSHDN_SHIFT)
#define SM5011_LONGKEY_SHIFT		5
#define SM5011_LONGKEY_MASK	    	(7 << SM5011_LONGKEY_SHIFT)


/* CNTL3 */
#define SM5011_ENVBATNG_SHIFT		1
#define SM5011_ENVBATNG_MASK		(1 << SM5011_ENVBATNG_SHIFT)
#define SM5011_ENVREF_SHIFT			0
#define SM5011_ENVREF_MASK			(1 << SM5011_ENVREF_SHIFT)

/* 
  * [ External Function ]
  */
extern int sm5011_set_smpltmr(int val);
extern int sm5011_set_smpl_auto(int val);
extern int sm5011_set_globalshdn(int val);
extern int sm5011_set_hardreset(int val);
extern int sm5011_set_softreset(int val);
extern int sm5011_set_mask_int(int val);
extern int sm5011_set_envbatng(int val);
extern int sm5011_set_envref(int val);

extern unsigned int sm5011_cntl_interface_init(struct sm5011_pmic_dev *sm5011_pmic);
extern unsigned int sm5011_cntl_preinit(void);

#endif /*  __LINUX_MFD_SM5011_CNTL_H */

