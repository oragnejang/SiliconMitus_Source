/*
 * sm5010-ldo.c
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/sm5010-core.h>
#include <linux/mfd/sm5010-irq.h>
#include <linux/mfd/sm5010.h>
#include <linux/mfd/sm5010-npwrstm.h>
#include <linux/regulator/sm5010-regulator.h>
#include <linux/io.h>
#include <linux/mutex.h>

#include "internal.h"

/* For LDO1~LDO20 except for LDO6 and LDO16 */
#define LDO1toLDO20_ENABLED(ENnPWRSTM, nPWRSTM, LDOxONOFF)	\
	((ENnPWRSTM == 1) && (nPWRSTM == 1) && (LDOxONOFF == 1 || LDOxONOFF == 2 || LDOxONOFF == 3)) ||	\
	((ENnPWRSTM == 1) && (nPWRSTM == 0) && (LDOxONOFF == 2 || LDOxONOFF == 3)) 					 ||	\
	((ENnPWRSTM == 0) && (nPWRSTM == 1) && (LDOxONOFF == 1 || LDOxONOFF == 2 || LDOxONOFF == 3)) ||	\
	((ENnPWRSTM == 0) && (nPWRSTM == 0) && (LDOxONOFF == 1 || LDOxONOFF == 2 || LDOxONOFF == 3))

/* For LDO6 */
#define LDO6_ENABLED(ENnPWRSTM, nPWRSTM, LDOxONOFF, ENTCXO)	\
	(ENTCXO) || \
	((ENnPWRSTM == 1) && (nPWRSTM == 1) && (LDOxONOFF == 1 || LDOxONOFF == 2 || LDOxONOFF == 3)) ||	\
	((ENnPWRSTM == 1) && (nPWRSTM == 0) && (LDOxONOFF == 2 || LDOxONOFF == 3)) 					 ||	\
	((ENnPWRSTM == 0) && (nPWRSTM == 1) && (LDOxONOFF == 1 || LDOxONOFF == 2 || LDOxONOFF == 3)) ||	\
	((ENnPWRSTM == 0) && (nPWRSTM == 0) && (LDOxONOFF == 1 || LDOxONOFF == 2 || LDOxONOFF == 3))

/* For LDO16 */
#define LDO16_ENABLED(ENnPWRSTM, nPWRSTM, LDOxONOFF, ENL16)	\
	(ENL16)	|| \
	((ENnPWRSTM == 1) && (nPWRSTM == 1) && (LDOxONOFF == 1 || LDOxONOFF == 2 || LDOxONOFF == 3)) ||	\
	((ENnPWRSTM == 1) && (nPWRSTM == 0) && (LDOxONOFF == 2 || LDOxONOFF == 3)) 					 ||	\
	((ENnPWRSTM == 0) && (nPWRSTM == 1) && (LDOxONOFF == 1 || LDOxONOFF == 2 || LDOxONOFF == 3)) ||	\
	((ENnPWRSTM == 0) && (nPWRSTM == 0) && (LDOxONOFF == 1 || LDOxONOFF == 2 || LDOxONOFF == 3))

static struct sm5010_ldo_info *static_info = NULL;

struct sm5010_ldo_info {	
	struct device			*dev;
	struct sm5010_pmic_dev *iodev;
	struct regulator_dev *rdev[SM5010_LDOMAX];
	
	int dvs_pin[SM5010_DVSPINS_MAX];
	int npwrstm_pin;	/* nPWRSTM GPIO */
	int entcxo_pin;		/* ENTCXO GPIO */
	int	enl16_pin;		/* ENL16 GPIO */
	int num_ldos;
	int irq_base;
	struct mutex lock;
};

static int _sm5010_ldo_is_enabled(struct regulator_dev *rdev)
{
	struct sm5010_ldo_info *sm5010 = rdev_get_drvdata(rdev);
	int reg_id = rdev_get_id(rdev);
	int ret, pin_sta, enpwrstm_sta, reg_sta, entcxo_pin_sta, enl16_pin_sta, enabled_val = 0;
	unsigned int val;

	/* Check nPWRSTM Pin Condtion */
	pin_sta = gpio_get_value(sm5010->npwrstm_pin);

	/* Check ENTCXO Pin Condtion */
	entcxo_pin_sta = gpio_get_value(sm5010->entcxo_pin);

	/* Check ENL16 Pin Condtion */
	enl16_pin_sta = gpio_get_value(sm5010->enl16_pin);

	/* Check ENnPWRSTM Bit Condtion */
	ret = sm5010_reg_read(sm5010->iodev, SM5010_REG_CNTL2, &val);	
	if (ret)
		return ret;	
	enpwrstm_sta = ((val & SM5010_ENnPWRSTM_MASK) >> SM5010_ENnPWRSTM_SHIFT);

	/* Check ONOFF Bit Condtion */
	ret = sm5010_reg_read(sm5010->iodev, rdev->desc->enable_reg, &val);	
	if (ret)
		return ret;	
	reg_sta = (val & rdev->desc->enable_mask);

	switch (reg_id) {			
		case SM5010_LDO1 ... SM5010_LDO5:
		case SM5010_LDO7 ... SM5010_LDO15:			
		case SM5010_LDO17 ... SM5010_LDO20:						
			enabled_val = LDO1toLDO20_ENABLED(enpwrstm_sta, pin_sta,reg_sta); 
			break;			
		case SM5010_LDO6:			
			enabled_val = LDO6_ENABLED(enpwrstm_sta, pin_sta,reg_sta, entcxo_pin_sta); 
			break;			
		case SM5010_LDO16:			
			enabled_val = LDO16_ENABLED(enpwrstm_sta, pin_sta,reg_sta, enl16_pin_sta); 
			break;						
		default:
			break;
	}

	return enabled_val;
}


/* LDOs supports [LPM/Normal] mode */
static int sm5010_set_mode(struct regulator_dev *rdev,
				     unsigned int mode)
{
	struct sm5010_ldo_info *sm5010 = rdev_get_drvdata(rdev);	
	struct sm5010_platform_data *pdata = sm5010->iodev->pdata;
	unsigned int val;	
	int reg_id = rdev_get_id(rdev);
	int ret, enpwrstm_sta= 0;	

	if (reg_id < SM5010_LDO1 && reg_id > SM5010_LDO20) {		
		pr_warn("%s: regulator_set_mode : not supported\n",
			rdev->desc->name);		
		return -EINVAL;
	}

	/* Check ENnPWRSTM Bit Condtion */
	ret = sm5010_reg_read(sm5010->iodev, SM5010_REG_CNTL2, &val);	
	if (ret)
		return ret;	
	enpwrstm_sta = ((val & SM5010_ENnPWRSTM_MASK) >> SM5010_ENnPWRSTM_SHIFT);

	/* 
	  * ENnPWRSTM Bit is 1 and ldo's lpm is always controlled by nPWRSTM , 
	  * register is set to SM5010_LPM_OPMODE_nPWRSTM 
	  */
	if (enpwrstm_sta == 1 && pdata->ldos[reg_id].always_lpm_npwrstm == 1)
	{
		pr_info("%s: [%s] Controlled by nPWRSTM\n", __func__, rdev->desc->name);
		val = SM5010_LPM_OPMODE_nPWRSTM << SM5010_LPM_OPMODE_SHIFT;
		return sm5010_reg_update(sm5010->iodev, rdev->desc->vsel_reg,
				  val, SM5010_LPM_OPMODE_MASK);
	}

	/* Non SM5010_LPM_OPMODE_nPWRSTM */
	switch (mode) {
	case REGULATOR_MODE_STANDBY:
		val = SM5010_LPM_OPMODE_ON << SM5010_LPM_OPMODE_SHIFT;
		break;
	case REGULATOR_MODE_NORMAL:
		val = SM5010_LPM_OPMODE_OFF << SM5010_LPM_OPMODE_SHIFT;
		break;		
	default:
		pr_warn("%s: regulator_lpm_mode : 0x%x not supported\n",
			rdev->desc->name, mode);
		return -EINVAL;
	}

	ret = sm5010_reg_update(sm5010->iodev, rdev->desc->vsel_reg,
				  val, SM5010_LPM_OPMODE_MASK);
	if (ret)
		return ret;

	return 0;
}

static unsigned int sm5010_get_mode(struct regulator_dev *rdev)
{
	struct sm5010_ldo_info *sm5010 = rdev_get_drvdata(rdev);	
	struct sm5010_platform_data *pdata = sm5010->iodev->pdata;
	unsigned int val;	
	int reg_id = rdev_get_id(rdev);
	int ret, enlpm_sta, pin_sta, enpwrstm_sta, enabled_val = 0;

	if (reg_id < SM5010_LDO1 && reg_id > SM5010_LDO20) {		
		pr_warn("%s: regulator_get_mode : not supported\n",
			rdev->desc->name);		
		return -EINVAL;
	}

	/* Check ENnPWRSTM Bit Condtion */
	ret = sm5010_reg_read(sm5010->iodev, SM5010_REG_CNTL2, &val);	
	if (ret)
		return ret;	
	enpwrstm_sta = ((val & SM5010_ENnPWRSTM_MASK) >> SM5010_ENnPWRSTM_SHIFT);
	
	enabled_val = _sm5010_ldo_is_enabled(rdev);

	ret = sm5010_reg_read(sm5010->iodev, rdev->desc->vsel_reg, &val);
	if (ret)
		return ret;
	enlpm_sta = ((val & SM5010_LPM_OPMODE_MASK) >> SM5010_LPM_OPMODE_SHIFT);
	
	/* 
	  * ENnPWRSTM Bit is 1 and ldo's lpm is always controlled by nPWRSTM , 
	  * register is set to SM5010_LPM_OPMODE_nPWRSTM 
	  */
	if (enpwrstm_sta == 1 && pdata->ldos[reg_id].always_lpm_npwrstm == 1 
		&& enabled_val == 1 && enlpm_sta == SM5010_LPM_OPMODE_nPWRSTM)
	{	
		pr_info("%s: [%s] Controlled by nPWRSTM\n", __func__, rdev->desc->name);
		return pin_sta == 0 ? REGULATOR_MODE_STANDBY : REGULATOR_MODE_NORMAL;
	}
	
	pr_info("%s : [%s] enlpm_sta = %d\n", __func__, rdev->desc->name, enlpm_sta);

	/* Non SM5010_LPM_OPMODE_nPWRSTM */
	return enlpm_sta == SM5010_LPM_OPMODE_ON ? REGULATOR_MODE_STANDBY : REGULATOR_MODE_NORMAL;
}


static int sm5010_enable(struct regulator_dev *rdev)
{
	struct sm5010_ldo_info *sm5010 = rdev_get_drvdata(rdev);
	struct sm5010_platform_data *pdata = sm5010->iodev->pdata;
	int reg_id = rdev_get_id(rdev);
	int ret, enpwrstm_sta= 0;	
	unsigned int val;

	/* Check ENnPWRSTM Bit Condtion */
	ret = sm5010_reg_read(sm5010->iodev, SM5010_REG_CNTL2, &val);	
	if (ret)
		return ret;	
	enpwrstm_sta = ((val & SM5010_ENnPWRSTM_MASK) >> SM5010_ENnPWRSTM_SHIFT);

	switch (reg_id)
	{
		case SM5010_LDO1 ... SM5010_LDO20:
			/* 
			  * ENnPWRSTM Bit is 1 and ldo is always controlled by nPWRSTM , 
			  * register is set to SM5010_OPMODE_nPWRSTM 
			  */
			if (enpwrstm_sta == 1 && pdata->ldos[reg_id].always_onoff_npwrstm == 1)
			{
				pr_info("%s: [%s] Controlled by nPWRSTM\n", __func__, rdev->desc->name);
				return sm5010_reg_update(sm5010->iodev, rdev->desc->enable_reg,
								  SM5010_OPMODE_nPWRSTM, rdev->desc->enable_mask);
			}
			break;			
		default:
			break;		
	}	

	return sm5010_reg_update(sm5010->iodev, rdev->desc->enable_reg,
				  rdev->desc->enable_mask, rdev->desc->enable_mask);
}

static int sm5010_disable(struct regulator_dev *rdev)
{
	struct sm5010_ldo_info *sm5010 = rdev_get_drvdata(rdev);
	struct sm5010_platform_data *pdata = sm5010->iodev->pdata;
	int reg_id = rdev_get_id(rdev);
	int ret, enpwrstm_sta= 0;	
	unsigned int val;

	/* Check ENnPWRSTM Bit Condtion */
	ret = sm5010_reg_read(sm5010->iodev, SM5010_REG_CNTL2, &val);	
	if (ret)
		return ret;	
	enpwrstm_sta = ((val & SM5010_ENnPWRSTM_MASK) >> SM5010_ENnPWRSTM_SHIFT);

	switch (reg_id)
	{
		case SM5010_LDO1 ... SM5010_LDO20:
			/* 
			  * ENnPWRSTM Bit is 1 and ldo is always controlled by nPWRSTM , 
			  * register is set to SM5010_OPMODE_nPWRSTM
			  */
			if (enpwrstm_sta == 1 && pdata->ldos[reg_id].always_onoff_npwrstm == 1)
			{
				pr_info("%s: [%s] Controlled by nPWRSTM\n", __func__, rdev->desc->name);
				return sm5010_reg_update(sm5010->iodev, rdev->desc->enable_reg,
								  SM5010_OPMODE_nPWRSTM, rdev->desc->enable_mask);
			}
			break;			
		default:
			break;		
	}

	return sm5010_reg_update(sm5010->iodev, rdev->desc->enable_reg,
				  ~(rdev->desc->enable_mask), rdev->desc->enable_mask);
}

static int sm5010_is_enabled(struct regulator_dev *rdev)
{
	return _sm5010_ldo_is_enabled(rdev);
}

static int sm5010_get_voltage_sel(struct regulator_dev *rdev)
{
	struct sm5010_ldo_info *sm5010 = rdev_get_drvdata(rdev);
	int ret;
	unsigned int val;

	ret = sm5010_reg_read(sm5010->iodev, rdev->desc->vsel_reg, &val);
	if (ret)
		return ret;

	val &= rdev->desc->vsel_mask;

	return val;
}

static int sm5010_set_voltage_sel(struct regulator_dev *rdev,
								unsigned sel)
{
	int ret;
	struct sm5010_ldo_info *sm5010 = rdev_get_drvdata(rdev);
	int set_val;

	set_val = rdev->desc->min_uV + (rdev->desc->uV_step * sel);

	ret = sm5010_reg_update(sm5010->iodev, rdev->desc->vsel_reg, 
		(sel & rdev->desc->vsel_mask), rdev->desc->vsel_mask);
	
	return ret;
}

static struct regulator_ops sm5010_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= sm5010_is_enabled,
	.enable			= sm5010_enable,
	.disable		= sm5010_disable,
	.get_voltage_sel	= sm5010_get_voltage_sel,
	.set_voltage_sel	= sm5010_set_voltage_sel,
	.set_mode		= sm5010_set_mode,	
	.get_mode		= sm5010_get_mode,
};

#define _LDO(macro)	SM5010_LDO##macro
#define _ldo_ops(num)	sm5010_ldo_ops##num
#define _REG_LDOxCNTLx(num1, num2)	SM5010_REG_LDO##num1##CNTL##num2

#define _REG(ctrl)	SM5010_REG##ctrl

#define SM5010_LDO_DESC(_name, _id, _ops, _mu, _us, _nv, _vr, _vm, _er, _em)	{	\
	.name		= _name,				\
	.id			= _id,					\
	.ops		= _ops,					\
	.type		= REGULATOR_VOLTAGE,	\
	.owner		= THIS_MODULE,			\
	.min_uV		= _mu,					\
	.uV_step	= _us,					\
	.n_voltages	= _nv,		\
	.vsel_reg	= _vr,					\
	.vsel_mask	= _vm,		\
	.enable_reg	= _er,					\
	.enable_mask	= _em,			\
}

static struct regulator_desc sm5010_lods_desc[SM5010_LDOMAX] = {
	/* name, id, ops, min_uv, uV_step, n_voltages, vsel_reg, vsel_mask, enable_reg, enable_mask */
	SM5010_LDO_DESC("LDO1", _LDO(1), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51),
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(1,2), _REG_LDOxCNTLx(1,1), _LDO(_ENABLE_MASK_03)),
	SM5010_LDO_DESC("LDO2", _LDO(2), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(2,2), _REG_LDOxCNTLx(2,1), _LDO(_ENABLE_MASK_03)),	
	SM5010_LDO_DESC("LDO3", _LDO(3), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(3,2), _REG_LDOxCNTLx(3,1), _LDO(_ENABLE_MASK_03)),	
	SM5010_LDO_DESC("LDO4", _LDO(4), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(4,2), _REG_LDOxCNTLx(4,1), _LDO(_ENABLE_MASK_03)),						
	SM5010_LDO_DESC("LDO5", _LDO(5), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(5,2), _REG_LDOxCNTLx(5,1), _LDO(_ENABLE_MASK_03)),
	SM5010_LDO_DESC("LDO6", _LDO(6), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(6,2), _REG_LDOxCNTLx(6,1), _LDO(_ENABLE_MASK_03)),	
	SM5010_LDO_DESC("LDO7", _LDO(7), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(7,2), _REG_LDOxCNTLx(7,1), _LDO(_ENABLE_MASK_03)),	
	SM5010_LDO_DESC("LDO8", _LDO(8), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(8,2), _REG_LDOxCNTLx(8,1), _LDO(_ENABLE_MASK_03)),
	SM5010_LDO_DESC("LDO9", _LDO(9), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(9,2), _REG_LDOxCNTLx(9,1), _LDO(_ENABLE_MASK_03)),
	SM5010_LDO_DESC("LDO10", _LDO(10), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(10,2), _REG_LDOxCNTLx(10,1), _LDO(_ENABLE_MASK_03)),	
	SM5010_LDO_DESC("LDO11", _LDO(11), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(11,2), _REG_LDOxCNTLx(11,1), _LDO(_ENABLE_MASK_03)),	
	SM5010_LDO_DESC("LDO12", _LDO(12), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(12,2), _REG_LDOxCNTLx(12,1), _LDO(_ENABLE_MASK_03)),	
	SM5010_LDO_DESC("LDO13", _LDO(13), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(13,2), _REG_LDOxCNTLx(13,1), _LDO(_ENABLE_MASK_03)),
	SM5010_LDO_DESC("LDO14", _LDO(14), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(14,2), _REG_LDOxCNTLx(14,1), _LDO(_ENABLE_MASK_03)),	
	SM5010_LDO_DESC("LDO15", _LDO(15), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(15,2), _REG_LDOxCNTLx(15,1), _LDO(_ENABLE_MASK_03)),	
	SM5010_LDO_DESC("LDO16", _LDO(16), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(16,2), _REG_LDOxCNTLx(16,1), _LDO(_ENABLE_MASK_03)),						
	SM5010_LDO_DESC("LDO17", _LDO(17), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(17,2), _REG_LDOxCNTLx(17,1), _LDO(_ENABLE_MASK_03)), 					
	SM5010_LDO_DESC("LDO18", _LDO(18), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(18,2), _REG_LDOxCNTLx(18,1), _LDO(_ENABLE_MASK_03)), 					
	SM5010_LDO_DESC("LDO19", _LDO(19), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(19,2), _REG_LDOxCNTLx(19,1), _LDO(_ENABLE_MASK_03)), 					
	SM5010_LDO_DESC("LDO20", _LDO(20), &_ldo_ops(), _LDO(_MIN_0P8V), _LDO(_STEP_50P0MV), _LDO(_NUM_51), 
					_LDO(_VSEL_MASK_3F), _REG_LDOxCNTLx(20,2), _REG_LDOxCNTLx(20,1), _LDO(_ENABLE_MASK_03)), 										
};

#ifdef CONFIG_OF
static int sm5010_ldo_dt_parse_pdata(struct sm5010_pmic_dev *iodev,
					struct sm5010_platform_data *pdata)
{
	struct device_node *pmic_np, *ldo_np, *ldos_np, *reg_np;
	struct sm5010_regulator_data *rdata;
	int i, ret;
	u32 val;

	pr_info("%s: Start\n", __func__);

	pmic_np = iodev->dev->of_node;
	if (!pmic_np) {
		dev_err(iodev->dev, "could not find ldo sub-node\n");
		return -ENODEV;
	}

	ldo_np = of_find_node_by_name(pmic_np, "sm5010-ldo");

	ldos_np = of_find_node_by_name(ldo_np, "ldos");
	if (!ldos_np) {
		dev_err(iodev->dev, "could not find ldos sub-node\n");
		return -EINVAL;
	}

	/* count the number of regulators to be supported in regulator */
	pdata->num_ldos = 0;
	for_each_child_of_node(ldos_np, reg_np) {
		pdata->num_ldos++;
	}

	rdata = devm_kzalloc(iodev->dev, sizeof(*rdata) *
				pdata->num_ldos, GFP_KERNEL);
	if (!rdata) {
		dev_err(iodev->dev,
			"could not allocate memory for regulator data\n");
		return -ENOMEM;
	}

	pdata->ldos = rdata;
	
	for_each_child_of_node(ldos_np, reg_np) {
		for (i = 0; i < ARRAY_SIZE(sm5010_lods_desc); i++)
			if (!of_node_cmp(reg_np->name, sm5010_lods_desc[i].name))
				break;

		if (i == ARRAY_SIZE(sm5010_lods_desc)) {
			dev_warn(iodev->dev, "don't know how to configure regulator %s\n",
			reg_np->name);
			continue;
		}
		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(
						iodev->dev, reg_np, &sm5010_lods_desc[i]);
		rdata->reg_node = reg_np;		
		rdata->initdata->constraints.valid_modes_mask	= REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY;
		rdata->initdata->constraints.valid_ops_mask |= REGULATOR_CHANGE_MODE;
		
		ret = of_property_read_u32(rdata->reg_node, "always_onoff_npwrstm", &val);
		if (ret)
			return -EINVAL;
		rdata->always_onoff_npwrstm = val;

		ret = of_property_read_u32(rdata->reg_node, "always_lpm_npwrstm", &val);
		if (ret)
			return -EINVAL;
		rdata->always_lpm_npwrstm = val;
		
		rdata++;
	}

	pr_info("%s: Done\n", __func__);

	return 0;
}
#else
static int sm5010_ldo_dt_parse_pdata(struct sm5010_pmic_dev *iodev,
					struct sm5010_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

static int sm5010_ldo_preinit(struct sm5010_ldo_info *info)
{
	int ret = 0;

	/*
	  * If need to add pre-initialisation, please add below
	  */
	  
	return ret;
}

static int sm5010_ldo_probe(struct platform_device *pdev)
{
	struct sm5010_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct sm5010_platform_data *pdata = iodev->pdata;
	struct regulator_config config = { };
	struct sm5010_ldo_info *info;
	int i;
	int ret = 0;

	pr_info("%s : Start\n", __func__);

	info = devm_kzalloc(&pdev->dev, sizeof(struct sm5010_ldo_info),
				GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	info->iodev = iodev;

	if (iodev->dev->of_node) {
		ret = sm5010_ldo_dt_parse_pdata(iodev, pdata);
		if (ret) {
			return ret;
		}
	}
	if (!pdata) {
		dev_err(pdev->dev.parent, "Platform data not supplied\n");
		return -ENODEV;
	}

	info->irq_base = iodev->irq_base;
	if (!info->irq_base) {
		dev_err(&pdev->dev, "Failed to get irq base %d\n", info->irq_base);
		return -ENODEV;
	}

	mutex_init(&info->lock);

	static_info = info;

	for (i = 0; i < SM5010_DVSPINS_MAX; i++)
	{
		if (gpio_is_valid(pdata->dvs_pin[i]))
			info->dvs_pin[i] = pdata->dvs_pin[i];
	}

	if (gpio_is_valid(pdata->npwrstm_pin))	
		info->npwrstm_pin = pdata->npwrstm_pin;

	if (gpio_is_valid(pdata->entcxo_pin))	
		info->entcxo_pin = pdata->entcxo_pin;

	if (gpio_is_valid(pdata->enl16_pin))	
		info->enl16_pin = pdata->enl16_pin;

	platform_set_drvdata(pdev, info);	

	for (i = 0; i < pdata->num_ldos; i++) {
		int id = pdata->ldos[i].id;
		config.dev = &pdev->dev;
		config.regmap = iodev->regmap;
		config.init_data = pdata->ldos[i].initdata;
		config.driver_data = info;
		config.of_node = pdata->ldos[i].reg_node;
		
		info->rdev[i] = regulator_register(&sm5010_lods_desc[id], &config);
		if (IS_ERR(info->rdev[i])) {
			ret = PTR_ERR(info->rdev[i]);
			dev_err(&pdev->dev, "regulator init failed for %d\n", i);
			info->rdev[i] = NULL;
			goto err;
		}
	}

	info->num_ldos = pdata->num_ldos;
	
	/* Initialisation */
    ret = sm5010_ldo_preinit(info);
	if (ret < 0)
		goto err;

	pr_info("%s : Done\n", __func__);

	return 0;
err:
	for (i = 0; i < SM5010_LDOMAX; i++)
		regulator_unregister(info->rdev[i]);

	return ret;
}

static int sm5010_ldo_remove(struct platform_device *pdev)
{
	struct sm5010_ldo_info *info = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < SM5010_LDOMAX; i++)
		regulator_unregister(info->rdev[i]);

	return 0;
}

static const struct of_device_id sm5010_ldo_of_match[] = {
	{ .compatible = "sm,sm5010-ldo", },
	{ },
};
MODULE_DEVICE_TABLE(platform, sm5010_ldo_of_match);

static const struct platform_device_id sm5010_ldo_id[] = {
	{ "sm5010-ldo", 0 },
};

static struct platform_driver sm5010_ldo_driver = {
	.driver = {
		.name = "sm5010-ldo",
		.owner = THIS_MODULE,
		.of_match_table = sm5010_ldo_of_match,		
	},
	.probe = sm5010_ldo_probe,
	.remove = sm5010_ldo_remove,
	.id_table = sm5010_ldo_id,
};

static int __init sm5010_ldo_init(void)
{
	pr_info("%s\n", __func__);

	return platform_driver_register(&sm5010_ldo_driver);
}
module_init(sm5010_ldo_init);

static void __exit sm5010_regulator_exit(void)
{
	platform_driver_unregister(&sm5010_ldo_driver);
}
module_exit(sm5010_regulator_exit);

/* Module information */
MODULE_DESCRIPTION("SM5010 Regulator LDO Driver");
MODULE_LICENSE("GPL");
