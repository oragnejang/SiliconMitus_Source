/*
 * sm5011-cntl.c
 *
 * Copyright (c) 2018 SILICONMITUS COMPANY,LTD
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
#include <linux/mfd/sm5011.h>
#include <linux/mfd/sm5011-core.h>
#include <linux/mfd/sm5011-cntl.h>  
 
static struct sm5011_cntl_dev cntl_info;

/* CNTL1 */
int sm5011_set_smpltmr(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5011_reg_update(cntl_info.iodev, SM5011_REG_CNTL1, 
		val << SM5011_SMPLTMR_SHIFT , SM5011_SMPLTMR_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5011_set_smpltmr);

int sm5011_set_smpl_auto(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5011_reg_update(cntl_info.iodev, SM5011_REG_CNTL1, 
		val << SM5011_SMPLAUTO_SHIFT , SM5011_SMPLAUTO_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5011_set_smpl_auto);

/* CNTL2 */
int sm5011_set_longkey(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5011_reg_update(cntl_info.iodev, SM5011_REG_CNTL2, 
		val << SM5011_LONGKEY_SHIFT , SM5011_LONGKEY_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5011_set_longkey);

int sm5011_set_globalshdn(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5011_reg_update(cntl_info.iodev, SM5011_REG_CNTL2, 
		val << SM5011_GLOBALSHDN_SHIFT , SM5011_GLOBALSHDN_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5011_set_globalshdn);

int sm5011_set_hardreset(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5011_reg_update(cntl_info.iodev, SM5011_REG_CNTL2, 
		val << SM5011_HARDRESET_SHIFT , SM5011_HARDRESET_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5011_set_hardreset);

int sm5011_set_softreset(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5011_reg_update(cntl_info.iodev, SM5011_REG_CNTL2, 
		val << SM5011_SOFTRESET_SHIFT , SM5011_SOFTRESET_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5011_set_softreset);

int sm5011_set_mask_int(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5011_reg_update(cntl_info.iodev, SM5011_REG_CNTL2, 
		val << SM5011_MASK_INT_SHIFT , SM5011_MASK_INT_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5011_set_mask_int);

/* CNTL2 */
int sm5011_set_envbatng(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5011_reg_update(cntl_info.iodev, SM5011_REG_CNTL3, 
		val << SM5011_ENVBATNG_SHIFT , SM5011_ENVBATNG_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5011_set_envbatng);

int sm5011_set_envref(int val)
{
	int ret = 0;

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	
	
	ret = sm5011_reg_update(cntl_info.iodev, SM5011_REG_CNTL3, 
		val << SM5011_ENVREF_SHIFT , SM5011_ENVREF_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5011_set_envref);

/* 
  * CNTL Setting Initialization 
  */
unsigned int sm5011_cntl_preinit(void)
{
	unsigned int status = CNTL_STATUS_OK;

	pr_info("%s: Start\n", __func__);

	if (cntl_info.iodev->i2c == NULL)
		return CNTL_STATUS_FAIL;	    

	/* Set SMPLTMR */
	sm5011_set_smpltmr(cntl_info.iodev->pdata->smpl_timer_val);
	/* Set SMPL_AUTO */
	sm5011_set_smpl_auto(cntl_info.iodev->pdata->smpl_power_on_type);
	/* Set MASK_INT */
	sm5011_set_mask_int(cntl_info.iodev->pdata->mask_int_en);
	/* Set ENVBATNG */
	sm5011_set_envbatng(cntl_info.iodev->pdata->envbatng_en);
	/* Set ENVREF */
	sm5011_set_envref(cntl_info.iodev->pdata->envref_en);
   	/* Set LONGKEY */
	sm5011_set_longkey(cntl_info.iodev->pdata->longkey_val);

	pr_info("%s: Success\n", __func__);

	return status;
}
EXPORT_SYMBOL(sm5011_cntl_preinit);
 
/* 
  *  CNTL control interface Initialization
  */
unsigned int sm5011_cntl_interface_init(struct sm5011_pmic_dev *sm5011_pmic)
{
	if (sm5011_pmic == NULL) {
		pr_err("%s: invalid sm5011_pmic dev ", __func__);
		return -EINVAL;
	}
	
	cntl_info.iodev = sm5011_pmic;

	pr_info("%s: Success\n", __func__);

	return CNTL_STATUS_OK;
}
EXPORT_SYMBOL(sm5011_cntl_interface_init);

