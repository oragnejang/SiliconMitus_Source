/*
 * sm5011-npwrstm.c
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
#include <linux/mfd/sm5011-npwrstm.h>  
 
static struct sm5011_npwrstm_dev npwrstm_info;

static inline int _sm5011_volt_to_reg(int uV)
{
        return (uV - SM5011_BUCK_MIN_0P6V) / SM5011_BUCK_STEP_12P5MV;
}

static inline int _sm5011_reg_to_volt(int regval)
{
        return regval * SM5011_BUCK_STEP_12P5MV + SM5011_BUCK_MIN_0P6V;
}

int sm5011_set_ennpwrstm(int val)
{
	int ret = 0;

	if (npwrstm_info.iodev->i2c == NULL)
		return nPWRSTM_STATUS_FAIL;	
	
	ret = sm5011_reg_update(npwrstm_info.iodev, SM5011_REG_CNTL2, 
		val << SM5011_ENnPWRSTM_SHIFT , SM5011_ENnPWRSTM_MASK);

	return ret;
}
EXPORT_SYMBOL(sm5011_set_ennpwrstm);

int sm5011_npwrstm_get_bits(void)
{
	int val = 0, enpwrstm_sta = 0, ret = 0;

	if (npwrstm_info.iodev->i2c == NULL)
		return nPWRSTM_STATUS_FAIL;	
	
	/* Check ENnPWRSTM Bit Condtion */
	ret = sm5011_reg_read(npwrstm_info.iodev, SM5011_REG_CNTL2, &val);	
	if (ret)
		return ret;	
	enpwrstm_sta = ((val & SM5011_ENnPWRSTM_MASK) >> SM5011_ENnPWRSTM_SHIFT);

	return enpwrstm_sta;
}
EXPORT_SYMBOL_GPL(sm5011_npwrstm_get_bits);


int sm5011_npwrstm_gpio_init(struct sm5011_pmic_dev *sm5011_pmic)
{
	int ret = 0;	 
 
	if (gpio_is_valid(sm5011_pmic->pdata->npwrstm_pin)) {
		ret = devm_gpio_request(sm5011_pmic->dev, sm5011_pmic->pdata->npwrstm_pin,
					"SM5011 nPWRSTM_PIN");
		if (ret < 0)
			return ret;
	}

	pr_info("%s: nPWRSTM Pin = %d\n", __func__, sm5011_pmic->pdata->npwrstm_pin);
	 
	return ret;
}
EXPORT_SYMBOL(sm5011_npwrstm_gpio_init);

/* 
  * nPWRSTM Setting Initialization 
  */
unsigned int sm5011_npwrstm_preinit(void)
{
	unsigned int status = nPWRSTM_STATUS_OK;

	pr_info("%s: Start\n", __func__);

	if (npwrstm_info.iodev->i2c == NULL)
		return nPWRSTM_STATUS_FAIL;	    

	/* Set ENnPWERTM */
	sm5011_set_ennpwrstm(npwrstm_info.iodev->pdata->npwrstm_en);

	pr_info("%s: Success\n", __func__);

	return status;
}
EXPORT_SYMBOL(sm5011_npwrstm_preinit);
 
/* 
  *  nPWRSTM control interface Initialization
  */
unsigned int sm5011_npwrstm_interface_init(struct sm5011_pmic_dev *sm5011_pmic)
{
	if (sm5011_pmic == NULL) {
		pr_err("%s: invalid sm5011_pmic dev ", __func__);
		return -EINVAL;
	}
	
	npwrstm_info.iodev = sm5011_pmic;

	pr_info("%s: Success\n", __func__);

	return nPWRSTM_STATUS_OK;
}
EXPORT_SYMBOL(sm5011_npwrstm_interface_init);
