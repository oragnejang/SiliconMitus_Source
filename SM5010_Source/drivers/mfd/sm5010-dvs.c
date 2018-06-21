 /*
 * sm5010-dvs.c
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
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
#include <linux/mfd/sm5010-dvs.h>

static struct sm5010_dvs_dev dvs_info;

static inline int _sm5010_volt_to_reg(int uV)
{
        return (uV - SM5010_BUCK_MIN_0P6V) / SM5010_BUCK_STEP_12P5MV;
}

static inline int _sm5010_reg_to_volt(int regval)
{
        return regval * SM5010_BUCK_STEP_12P5MV + SM5010_BUCK_MIN_0P6V;
}

unsigned int sm5010_dvs_setvolt(unsigned int lvl, int uV)
{
	unsigned int status = DVS_STATUS_OK;
	int ret = 0;
	int cur_reg = lvl + SM5010_REG_BUCK1CNTL2;
		
	if (lvl < SM5010_BUCK1DVS_0 || lvl > SM5010_BUCK1DVS_7)
		return DVS_STATUS_FAIL;
	
	if (uV < SM5010_BUCK_MIN_0P6V || uV > SM5010_BUCK_MAX_1P2V)
		return DVS_STATUS_FAIL;

	if (dvs_info.iodev->i2c == NULL)
		return DVS_STATUS_FAIL;
	
	ret = sm5010_reg_update(dvs_info.iodev, cur_reg, 
		(uV & SM5010_BUCK_VSEL_MASK_3F), SM5010_BUCK_VSEL_MASK_3F);

	return status;
}
EXPORT_SYMBOL(sm5010_dvs_setvolt);

unsigned int sm5010_dvs_getvolt(unsigned int lvl, int* uV)
{
	unsigned int status = DVS_STATUS_OK;
	int regval = 0, ret = 0;
	
	int cur_reg = lvl + SM5010_REG_BUCK1CNTL2;
	
	if (lvl < SM5010_BUCK1DVS_0 || lvl > SM5010_BUCK1DVS_7)
		return DVS_STATUS_FAIL;

	if (dvs_info.iodev->i2c == NULL)
		return DVS_STATUS_FAIL;

	ret = sm5010_reg_read(dvs_info.iodev, cur_reg, &regval);	
	
	if (!ret)
	{	
		regval &= SM5010_BUCK_VSEL_MASK_3F;
		*uV = _sm5010_reg_to_volt(regval);
	}

	return status;
}
EXPORT_SYMBOL(sm5010_dvs_getvolt);

unsigned int sm5010_dvs_enshedding(int enable)
{
	unsigned int status = DVS_STATUS_OK;
	int ret = 0;
	int cur_reg = SM5010_REG_BUCK1CNTL11;

	if (dvs_info.iodev->i2c == NULL)
		return DVS_STATUS_FAIL;
	
	ret = sm5010_reg_update(dvs_info.iodev, cur_reg, 
		enable << SM5010_ENSHEDDING_SHIFT, SM5010_ENSHEDDING_MASK);
	if (ret < 0)
		return DVS_STATUS_FAIL;
	
	return status;
}
EXPORT_SYMBOL(sm5010_dvs_enshedding);

unsigned int sm5010_dvs_freq(int hz)
{
	unsigned int status = DVS_STATUS_OK;
	int ret = 0;
	int cur_reg = SM5010_REG_BUCK1CNTL11;

	if (dvs_info.iodev->i2c == NULL)
		return DVS_STATUS_FAIL;
	
	ret = sm5010_reg_update(dvs_info.iodev, cur_reg, 
		hz << SM5010_FREQ_SHIFT, SM5010_FREQ_MASK);
	if (ret < 0)
		return DVS_STATUS_FAIL;
	
	return status;
}
EXPORT_SYMBOL(sm5010_dvs_freq);

unsigned int sm5010_dvs_dvsrampb1(int slope)
{
	unsigned int status = DVS_STATUS_OK;
	int ret = 0;
	int cur_reg = SM5010_REG_BUCK1CNTL11;

	if (dvs_info.iodev->i2c == NULL)
		return DVS_STATUS_FAIL;
	
	ret = sm5010_reg_update(dvs_info.iodev, cur_reg, 
		slope << SM5010_DVSRAMPB1_SHIFT, SM5010_DVSRAMPB1_MASK);
	if (ret < 0)
		return DVS_STATUS_FAIL;
	
	return status;
}
EXPORT_SYMBOL(sm5010_dvs_dvsrampb1);

unsigned int sm5010_dvs_enrampb1down(int enable)
{
	unsigned int status = DVS_STATUS_OK;
	int ret = 0;
	int cur_reg = SM5010_REG_BUCK1CNTL11;

	if (dvs_info.iodev->i2c == NULL)
		return DVS_STATUS_FAIL;
	
	ret = sm5010_reg_update(dvs_info.iodev, cur_reg, 
		enable << SM5010_ENRAMPB1DOWN_SHIFT, SM5010_ENRAMPB1DOWN_MASK);
	if (ret < 0)
		return DVS_STATUS_FAIL;
	
	return status;
}
EXPORT_SYMBOL(sm5010_dvs_enrampb1down);

unsigned int sm5010_dvs_buck1adisen(int enable)
{
	unsigned int status = DVS_STATUS_OK;
	int ret = 0;
	int cur_reg = SM5010_REG_BUCK1CNTL11;

	if (dvs_info.iodev->i2c == NULL)
		return DVS_STATUS_FAIL;
	
	ret = sm5010_reg_update(dvs_info.iodev, cur_reg, 
		enable << SM5010_BUCK1ADISEN_SHIFT, SM5010_BUCK1ADISEN_MASK);
	if (ret < 0)
		return DVS_STATUS_FAIL;
	
	return status;
}
EXPORT_SYMBOL(sm5010_dvs_buck1adisen);

unsigned int sm5010_dvs_buck1mode(int mode)
{
	unsigned int status = DVS_STATUS_OK;
	int ret = 0;
	int cur_reg = SM5010_REG_BUCK1CNTL11;

	if (dvs_info.iodev->i2c == NULL)
		return DVS_STATUS_FAIL;
	
	ret = sm5010_reg_update(dvs_info.iodev, cur_reg, 
		mode << SM5010_BUCK1MODE_SHIFT, SM5010_BUCK1MODE_MASK);
	if (ret < 0)
		return DVS_STATUS_FAIL;
	
	return status;
}
EXPORT_SYMBOL(sm5010_dvs_buck1mode);

unsigned int sm5010_dvs_buck1enable(void)
{
	unsigned int status = DVS_STATUS_OK;
	int ret = 0;
	int cur_reg = SM5010_REG_BUCK1CNTL1;

	if (dvs_info.iodev->i2c == NULL)
		return DVS_STATUS_FAIL;
	
	ret = sm5010_reg_update(dvs_info.iodev, cur_reg, 
		SM5010_BUCK_ENABLE_MASK_03, SM5010_BUCK_ENABLE_MASK_03);
	if (ret < 0)
		return DVS_STATUS_FAIL;
	
	return status;
}
EXPORT_SYMBOL(sm5010_dvs_buck1enable);

unsigned int sm5010_dvs_buck1disable(void)
{
	unsigned int status = DVS_STATUS_OK;
	int ret = 0;
	int cur_reg = SM5010_REG_BUCK1CNTL1;

	if (dvs_info.iodev->i2c == NULL)
		return DVS_STATUS_FAIL;
	
	ret = sm5010_reg_update(dvs_info.iodev, cur_reg, 
		~SM5010_BUCK_ENABLE_MASK_03, SM5010_BUCK_ENABLE_MASK_03);
	if (ret < 0)
		return DVS_STATUS_FAIL;
	
	return status;
}
EXPORT_SYMBOL(sm5010_dvs_buck1disable);


int sm5010_dvs_gpio_init(struct sm5010_pmic_dev *sm5010_pmic)
{
	int i = 0, ret = 0;
	 
	for (i = 0; i < SM5010_DVSPINS_MAX; i++)
	{
		if (gpio_is_valid(sm5010_pmic->pdata->dvs_pin[i])) {
			ret = devm_gpio_request(sm5010_pmic->dev, sm5010_pmic->pdata->dvs_pin[i],
						"SM5010 DVS_PIN_##i");
			if (ret < 0)
				return ret;
		}
	}

	pr_info("%s: DVS3 = %d, DVS2 = %d, DVS1 = %d\n", __func__, 
		sm5010_pmic->pdata->dvs_pin[2], sm5010_pmic->pdata->dvs_pin[1], sm5010_pmic->pdata->dvs_pin[0]);
	 
	return ret;
}
EXPORT_SYMBOL(sm5010_dvs_gpio_init);

/*
  * Get dvs_pins information
  * DVS3/2/1 = dvs_pin[2]/[1]/[0] 
  */
int sm5010_dvs_get_dvspins(void)
{
	int val = 0;

	if (dvs_info.iodev->i2c == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}
	
	val |= gpio_get_value(dvs_info.iodev->pdata->dvs_pin[0]);
	val |= (gpio_get_value(dvs_info.iodev->pdata->dvs_pin[1]) << 1);
	val |= (gpio_get_value(dvs_info.iodev->pdata->dvs_pin[2]) << 2);

	return (val & 0x07);
}
EXPORT_SYMBOL_GPL(sm5010_dvs_get_dvspins);

int sm5010_dvs_set_dvspins(bool dvs1_val, bool dvs2_val, bool dvs3_val)
{
	if (dvs_info.iodev->i2c == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}

	if (!gpio_is_valid(dvs_info.iodev->pdata->dvs_pin[0]) 
		|| !gpio_is_valid(dvs_info.iodev->pdata->dvs_pin[1]) 
		|| !gpio_is_valid(dvs_info.iodev->pdata->dvs_pin[2]) ) {
		pr_warn("%s: dvs pin ctrl failed\n", __func__);
		return -EINVAL;
	}
	gpio_direction_output(dvs_info.iodev->pdata->dvs_pin[0], dvs1_val);
	gpio_direction_output(dvs_info.iodev->pdata->dvs_pin[1], dvs2_val);
	gpio_direction_output(dvs_info.iodev->pdata->dvs_pin[2], dvs3_val);
	udelay(100);

	return 0;
}
EXPORT_SYMBOL_GPL(sm5010_dvs_set_dvspins);

/* 
  * DVS & nPWRSTM Setting Initialization 
  */
unsigned int sm5010_dvs_preinit(void)
{
	unsigned int status = DVS_STATUS_OK;
	int i = 0;

	if (dvs_info.iodev->i2c == NULL)
		return DVS_STATUS_FAIL;

	for (i = SM5010_BUCK1DVS_0 ; i < SM5010_BUCK1DVSMAX; i++)
		sm5010_dvs_setvolt(i, dvs_info.iodev->pdata->buck1dvsout[i]);		

	sm5010_dvs_enshedding(dvs_info.iodev->pdata->enshedding);
	sm5010_dvs_freq(dvs_info.iodev->pdata->freq);
	sm5010_dvs_dvsrampb1(dvs_info.iodev->pdata->dvsrampb1);
	sm5010_dvs_enrampb1down(dvs_info.iodev->pdata->enrampb1down);
	sm5010_dvs_buck1adisen(dvs_info.iodev->pdata->buck1adisen);
	sm5010_dvs_buck1mode(dvs_info.iodev->pdata->buck1mode);		

	return status;
}
EXPORT_SYMBOL(sm5010_dvs_preinit);

/* 
  *  DVS control interface Initialization
  */
unsigned int sm5010_dvs_interface_init(struct sm5010_pmic_dev *sm5010_pmic)
{	
	if (sm5010_pmic == NULL) {
		pr_err("%s: invalid sm5010_pmic dev ", __func__);
		return -EINVAL;
	}
	dvs_info.iodev = sm5010_pmic;

	return DVS_STATUS_OK;
}
EXPORT_SYMBOL(sm5010_dvs_interface_init);
