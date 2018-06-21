/*
 * sm5010-npwrstm.c
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
#include <linux/mfd/sm5010-npwrstm.h>  
 
static struct sm5010_npwrstm_dev npwrstm_info;

static inline int _sm5010_volt_to_reg(int uV)
{
        return (uV - SM5010_BUCK_MIN_0P6V) / SM5010_BUCK_STEP_12P5MV;
}

static inline int _sm5010_reg_to_volt(int regval)
{
        return regval * SM5010_BUCK_STEP_12P5MV + SM5010_BUCK_MIN_0P6V;
}

int sm5010_set_ennpwrstm(int val)
{
	int ret = 0;

	if (npwrstm_info.iodev->i2c == NULL)
		return nPWRSTM_STATUS_FAIL;	
	
	ret = sm5010_reg_update(npwrstm_info.iodev, SM5010_REG_CNTL2, 
		val << SM5010_ENnPWRSTM_SHIFT , SM5010_ENnPWRSTM_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5010_set_ennpwrstm);

unsigned int sm5010_npwrstm_setvolt(unsigned int lvl, int uV)
{
	unsigned int status = nPWRSTM_STATUS_OK;
	int ret = 0;
	int cur_reg = lvl + SM5010_REG_BUCK1CNTL10;
		
	if (lvl < SM5010_BUCK1nPWRSTM || lvl > SM5010_BUCK1nPWRSTM)
		return nPWRSTM_STATUS_FAIL;
	
	if (uV < SM5010_BUCK_MIN_0P6V || uV > SM5010_BUCK_MAX_1P2V)
		return nPWRSTM_STATUS_FAIL;

	if (npwrstm_info.iodev->i2c == NULL)
		return nPWRSTM_STATUS_FAIL;
	
	ret = sm5010_reg_update(npwrstm_info.iodev, cur_reg, 
			uV, SM5010_BUCK_VSEL_MASK_3F);

	return status;
}
EXPORT_SYMBOL(sm5010_npwrstm_setvolt);

unsigned int sm5010_npwrstm_getvolt(unsigned int lvl, int* uV)
{
	unsigned int status = nPWRSTM_STATUS_OK;
	int regval = 0, ret = 0;
	
	int cur_reg = lvl + SM5010_REG_BUCK1CNTL10;
	
	if (lvl < SM5010_BUCK1nPWRSTM || lvl > SM5010_BUCK1nPWRSTM)
		return nPWRSTM_STATUS_FAIL;

	if (npwrstm_info.iodev->i2c == NULL)
		return nPWRSTM_STATUS_FAIL;

	ret = sm5010_reg_read(npwrstm_info.iodev, cur_reg, &regval);	
	
	if (!ret)
	{	
		regval &= SM5010_BUCK_VSEL_MASK_3F;
		*uV = _sm5010_reg_to_volt(regval);
	}

	return status;
}
EXPORT_SYMBOL(sm5010_npwrstm_getvolt);

int sm5010_npwrstm_get_pins(void)
{
	int val = 0;

	if (npwrstm_info.iodev->i2c == NULL)
		return nPWRSTM_STATUS_FAIL;	
	
	val = gpio_get_value(npwrstm_info.iodev->pdata->npwrstm_pin);

	return (val & 0x01);
}
EXPORT_SYMBOL_GPL(sm5010_npwrstm_get_pins);

int sm5010_npwrstm_set_pins(bool val)
{
	if (npwrstm_info.iodev->i2c == NULL)
		return nPWRSTM_STATUS_FAIL;	

	if (!gpio_is_valid(npwrstm_info.iodev->pdata->npwrstm_pin)) 
	{
		pr_warn("%s: dvs pin ctrl failed\n", __func__);
		return -EINVAL;
	}
	gpio_direction_output(npwrstm_info.iodev->pdata->npwrstm_pin, val);

	return 0;
}
EXPORT_SYMBOL_GPL(sm5010_npwrstm_set_pins);

int sm5010_npwrstm_get_bits(void)
{
	int val = 0, enpwrstm_sta = 0, ret = 0;

	if (npwrstm_info.iodev->i2c == NULL)
		return nPWRSTM_STATUS_FAIL;	
	
	/* Check ENnPWRSTM Bit Condtion */
	ret = sm5010_reg_read(npwrstm_info.iodev, SM5010_REG_CNTL2, &val);	
	if (ret)
		return ret;	
	enpwrstm_sta = ((val & SM5010_ENnPWRSTM_MASK) >> SM5010_ENnPWRSTM_SHIFT);

	return enpwrstm_sta;
}
EXPORT_SYMBOL_GPL(sm5010_npwrstm_get_bits);


int sm5010_npwrstm_gpio_init(struct sm5010_pmic_dev *sm5010_pmic)
{
	int ret = 0;	 
 
	if (gpio_is_valid(sm5010_pmic->pdata->npwrstm_pin)) {
		ret = devm_gpio_request(sm5010_pmic->dev, sm5010_pmic->pdata->npwrstm_pin,
					"SM5010 nPWRSTM_PIN");
		if (ret < 0)
			return ret;
	}

	pr_info("%s: nPWRSTM Pin = %d\n", __func__, sm5010_pmic->pdata->npwrstm_pin);
	 
	return ret;
}
EXPORT_SYMBOL(sm5010_npwrstm_gpio_init);

unsigned int sm5010_npwrstm_buck1enable(void)
{
	unsigned int status = nPWRSTM_STATUS_OK;
	int ret = 0;
	int cur_reg = SM5010_REG_BUCK1CNTL1;

	if (npwrstm_info.iodev->i2c == NULL)
		return nPWRSTM_STATUS_FAIL;
	
	ret = sm5010_reg_update(npwrstm_info.iodev, cur_reg, 
		SM5010_OPMODE_nPWRSTM, SM5010_BUCK_ENABLE_MASK_03);
	if (ret < 0)
		return nPWRSTM_STATUS_FAIL;
	
	return status;
}
EXPORT_SYMBOL(sm5010_npwrstm_buck1enable);

/* 
  * nPWRSTM Setting Initialization 
  */
unsigned int sm5010_npwrstm_preinit(void)
{
	unsigned int status = nPWRSTM_STATUS_OK;

	pr_info("%s: Start\n", __func__);

	if (npwrstm_info.iodev->i2c == NULL)
		return nPWRSTM_STATUS_FAIL;	    

	/* Set ENnPWERTM */
	sm5010_set_ennpwrstm(npwrstm_info.iodev->pdata->npwrstm_en);

	sm5010_npwrstm_setvolt(SM5010_BUCK1nPWRSTM, npwrstm_info.iodev->pdata->buck1npwrstmout);

	pr_info("%s: Success\n", __func__);

	return status;
}
EXPORT_SYMBOL(sm5010_npwrstm_preinit);
 
/* 
  *  nPWRSTM control interface Initialization
  */
unsigned int sm5010_npwrstm_interface_init(struct sm5010_pmic_dev *sm5010_pmic)
{
	if (sm5010_pmic == NULL) {
		pr_err("%s: invalid sm5010_pmic dev ", __func__);
		return -EINVAL;
	}
	
	npwrstm_info.iodev = sm5010_pmic;

	pr_info("%s: Success\n", __func__);

	return nPWRSTM_STATUS_OK;
}
EXPORT_SYMBOL(sm5010_npwrstm_interface_init);
