/*
 * sm5010-cntl.c
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under	 the terms of  the GNU General	Public License as published by the
 *  Free Software Foundation;	 either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
 
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/mfd/sm5010.h>
#include <linux/mfd/sm5010-core.h>
#include <linux/mfd/sm5010-cntl.h>  
 
static struct sm5010_cntl_dev cntl_info;

/* CNTL1 */
int sm5010_set_smpltmr(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5010_reg_update(cntl_info.iodev, SM5010_REG_CNTL1, 
		val << SM5010_SMPLTMR_SHIFT , SM5010_SMPLTMR_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5010_set_smpltmr);

int sm5010_set_smpl_auto(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5010_reg_update(cntl_info.iodev, SM5010_REG_CNTL1, 
		val << SM5010_SMPLAUTO_SHIFT , SM5010_SMPLAUTO_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5010_set_smpl_auto);

/* CNTL2 */
int sm5010_set_globalshdn(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5010_reg_update(cntl_info.iodev, SM5010_REG_CNTL2, 
		val << SM5010_GLOBALSHDN_SHIFT , SM5010_GLOBALSHDN_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5010_set_globalshdn);

int sm5010_set_hardreset(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5010_reg_update(cntl_info.iodev, SM5010_REG_CNTL2, 
		val << SM5010_HARDRESET_SHIFT , SM5010_HARDRESET_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5010_set_hardreset);

int sm5010_set_softreset(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5010_reg_update(cntl_info.iodev, SM5010_REG_CNTL2, 
		val << SM5010_SOFTRESET_SHIFT , SM5010_SOFTRESET_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5010_set_softreset);

int sm5010_set_mask_int(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5010_reg_update(cntl_info.iodev, SM5010_REG_CNTL2, 
		val << SM5010_MASK_INT_SHIFT , SM5010_MASK_INT_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5010_set_mask_int);

/* CNTL2 */
int sm5010_set_envbatng(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5010_reg_update(cntl_info.iodev, SM5010_REG_CNTL3, 
		val << SM5010_ENVBATNG_SHIFT , SM5010_ENVBATNG_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5010_set_envbatng);

int sm5010_set_envref(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5010_reg_update(cntl_info.iodev, SM5010_REG_CNTL3, 
		val << SM5010_ENVREF_SHIFT , SM5010_ENVREF_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5010_set_envref);

/* 
  * CNTL Setting Initialization 
  */
unsigned int sm5010_cntl_preinit(void)
{
	unsigned int status = CNTL_STATUS_OK;

	pr_info("%s: Start\n", __func__);

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	    

	/* Set SMPLTMR */
	sm5010_set_smpltmr(cntl_info.iodev->pdata->smpl_timer_val);
	/* Set SMPL_AUTO */
	sm5010_set_smpl_auto(cntl_info.iodev->pdata->smpl_power_on_type);
	/* Set MASK_INT */
	sm5010_set_mask_int(cntl_info.iodev->pdata->mask_int_en);
	/* Set ENVBATNG */
	sm5010_set_envbatng(cntl_info.iodev->pdata->envbatng_en);
	/* Set ENVREF */
	sm5010_set_envref(cntl_info.iodev->pdata->envref_en);

	pr_info("%s: Success\n", __func__);

	return status;
}
EXPORT_SYMBOL(sm5010_cntl_preinit);
 
/* 
  *  CNTL control interface Initialization
  */
unsigned int sm5010_cntl_interface_init(struct sm5010_pmic_dev *sm5010_pmic)
{
	if (sm5010_pmic == NULL) {
		pr_err("%s: invalid sm5010_pmic dev ", __func__);
		return -EINVAL;
	}
	
	cntl_info.iodev = sm5010_pmic;

	pr_info("%s: Success\n", __func__);

	return CNTL_STATUS_OK;
}
EXPORT_SYMBOL(sm5010_cntl_interface_init);

