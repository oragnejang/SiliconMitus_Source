/*
 * sm5011-buck.c
 *
 * Copyright (c) 2018 SILICONMITUS COMPANY,LTD
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
#include <linux/mfd/sm5011-core.h>
#include <linux/mfd/sm5011-irq.h>
#include <linux/mfd/sm5011.h>
#include <linux/mfd/sm5011-npwrstm.h>
#include <linux/regulator/sm5011-regulator.h>
#include <linux/io.h>
#include <linux/mutex.h>

#include "internal.h"

#define BUCK2toBUCK6_ENABLED(ENnPWRSTM, nPWRSTM, BUCKxONOFF)	\
	((ENnPWRSTM == 1) && (nPWRSTM == 1) && (BUCKxONOFF == 1 || BUCKxONOFF == 2 || BUCKxONOFF == 3)) ||	\
	((ENnPWRSTM == 1) && (nPWRSTM == 0) && (BUCKxONOFF == 2 || BUCKxONOFF == 3)) 					 ||	\
	((ENnPWRSTM == 0) && (nPWRSTM == 1) && (BUCKxONOFF == 1 || BUCKxONOFF == 2 || BUCKxONOFF == 3)) ||	\
	((ENnPWRSTM == 0) && (nPWRSTM == 0) && (BUCKxONOFF == 1 || BUCKxONOFF == 2 || BUCKxONOFF == 3))

static struct sm5011_buck_info *static_info = NULL;

/* In case of Freq = 0 */
enum sm5011_ramp0_rate {
	RAMP_RATE_2P343MV,
	RAMP_RATE_4P69MV,
	RAMP_RATE_9P38MV,
	RAMP_RATE_18P76MV,
};

/* In case of Freq = 1 */
enum sm5011_ramp1_rate {
	RAMP_RATE_3P125MV,
	RAMP_RATE_6P25MV,
	RAMP_RATE_12P5MV,
	RAMP_RATE_25P0MV,
};

struct sm5011_buck_info {	
	struct device			*dev;
	struct sm5011_pmic_dev *iodev;
	struct regulator_dev *rdev[SM5011_BUCKMAX];
	
	int npwrstm_pin;	/* nPWRSTM GPIO */
	int num_bucks;
	int irq_base;
	struct mutex lock;
};

static int _sm5011_buck_is_enabled(struct regulator_dev *rdev)
{
	struct sm5011_buck_info *sm5011 = rdev_get_drvdata(rdev);
	int reg_id = rdev_get_id(rdev);
	int ret, pin_sta, enpwrstm_sta, reg_sta, enabled_val = 0;
	unsigned int val;

	/* Check nPWRSTM Pin Condtion */
	pin_sta = gpio_get_value(sm5011->npwrstm_pin);

	/* Check ENnPWRSTM Bit Condtion */
	ret = sm5011_reg_read(sm5011->iodev, SM5011_REG_CNTL2, &val);	
	if (ret)
		return ret;	
	enpwrstm_sta = ((val & SM5011_ENnPWRSTM_MASK) >> SM5011_ENnPWRSTM_SHIFT);

	/* Check ONOFF Bit Condtion */
	ret = sm5011_reg_read(sm5011->iodev, rdev->desc->enable_reg, &val);	
	if (ret)
		return ret;	
	reg_sta = (val & rdev->desc->enable_mask);

	switch (reg_id) {
		case SM5011_BUCK2 ... SM5011_BUCK6:						
			enabled_val = BUCK2toBUCK6_ENABLED(enpwrstm_sta, pin_sta,reg_sta);
			break;			
		default:
			break;
	}

	return enabled_val;
}


/* 
 * BUCKs supports [Auto-Mode / Forced PWM] 
 * mode 2 : SM5011_PWM_OPMODE_AUTOMODE
		1 : SM5011_PWM_OPMODE_FORCEPWMMODE
 */
static int sm5011_set_mode(struct regulator_dev *rdev,
				     unsigned int mode)
{
	struct sm5011_buck_info *sm5011 = rdev_get_drvdata(rdev);
	int reg_id = rdev_get_id(rdev);	
	unsigned int mode_reg = 0;
	int ret, val;

	pr_warn("%s : %s: test-----regulator_pwm_mode : 0x%x\n",
		__func__, rdev->desc->name, mode);		

	switch (reg_id) {		
	case SM5011_BUCK2:
		mode_reg = SM5011_REG_BUCK2CNTL3;		
		break;		
	case SM5011_BUCK3:
		mode_reg = SM5011_REG_BUCK3CNTL3;		
		break;		
	case SM5011_BUCK4:
		mode_reg = SM5011_REG_BUCK4CNTL3;		
		break;		
	case SM5011_BUCK5:
		mode_reg = SM5011_REG_BUCK5CNTL3;		
		break;		
	case SM5011_BUCK6:
		mode_reg = SM5011_REG_BUCK6CNTL3;		
		break;
	default:
		pr_warn("%s: regulator_pwm_mode : 0x%x not supported\n",
			rdev->desc->name, mode);		
		return -EINVAL;
	}

	if (mode == REGULATOR_MODE_FAST)
		val = SM5011_PWM_OPMODE_FORCEPWMMODE;
	else if (mode == REGULATOR_MODE_NORMAL)
		val = SM5011_PWM_OPMODE_AUTOMODE;
	else
		return -EINVAL;

	ret = sm5011_reg_update(sm5011->iodev, mode_reg,
				  val << SM5011_PWM_OPMODE_SHIFT, SM5011_PWM_OPMODE_MASK);
	if (ret)
		return ret;

	return 0;
}

static unsigned int sm5011_get_mode(struct regulator_dev *rdev)
{
	struct sm5011_buck_info *sm5011 = rdev_get_drvdata(rdev);
	int reg_id = rdev_get_id(rdev);	
	unsigned int mode_reg = 0;
	int ret, val;

	switch (reg_id) {		
	case SM5011_BUCK2:
		mode_reg = SM5011_REG_BUCK2CNTL3;		
		break;		
	case SM5011_BUCK3:
		mode_reg = SM5011_REG_BUCK3CNTL3;		
		break;		
	case SM5011_BUCK4:
		mode_reg = SM5011_REG_BUCK4CNTL3;		
		break;		
	case SM5011_BUCK5:
		mode_reg = SM5011_REG_BUCK5CNTL3;		
		break;		
	case SM5011_BUCK6:
		mode_reg = SM5011_REG_BUCK6CNTL3;		
		break;
	default:
		pr_warn("%s: regulator_pwm_mode : not supported\n",
			rdev->desc->name);		
		return -EINVAL;
	}

	ret = sm5011_reg_read(sm5011->iodev, mode_reg, &val);
	if (ret)
		return ret;
	
	pr_info("%s : [%s] val = %d\n", __func__, rdev->desc->name, val & 0x1);

	return val & 0x1 ? REGULATOR_MODE_FAST : REGULATOR_MODE_NORMAL;
}

static int sm5011_enable(struct regulator_dev *rdev)
{
	struct sm5011_buck_info *sm5011 = rdev_get_drvdata(rdev);
	struct sm5011_platform_data *pdata = sm5011->iodev->pdata;
	int reg_id = rdev_get_id(rdev);
	int ret, enpwrstm_sta= 0;	
	unsigned int val;

	/* Check ENnPWRSTM Bit Condtion */
	ret = sm5011_reg_read(sm5011->iodev, SM5011_REG_CNTL2, &val);	
	if (ret)
		return ret;	
	enpwrstm_sta = ((val & SM5011_ENnPWRSTM_MASK) >> SM5011_ENnPWRSTM_SHIFT);

	switch (reg_id)
	{
		case SM5011_BUCK2 ... SM5011_BUCK6:
			/* ENnPWRSTM Bit is 1 and buck is always controlled by nPWRSTM , register is set to SM5011_OPMODE_nPWRSTM */
			if (enpwrstm_sta == 1 && pdata->bucks[reg_id].always_onoff_npwrstm == 1)
			{
				pr_info("%s: [%s] Controlled by nPWRSTM\n", __func__, rdev->desc->name);
				return sm5011_reg_update(sm5011->iodev, rdev->desc->enable_reg,
								  SM5011_OPMODE_nPWRSTM, rdev->desc->enable_mask);
			}
			break;			
		default:
			break;		
	}		
	
	return sm5011_reg_update(sm5011->iodev, rdev->desc->enable_reg,
				  rdev->desc->enable_mask, rdev->desc->enable_mask);
}

static int sm5011_disable(struct regulator_dev *rdev)
{
	struct sm5011_buck_info *sm5011 = rdev_get_drvdata(rdev);
	struct sm5011_platform_data *pdata = sm5011->iodev->pdata;
	int reg_id = rdev_get_id(rdev);
	int ret, enpwrstm_sta= 0;	
	unsigned int val;

	/* Check ENnPWRSTM Bit Condtion */
	ret = sm5011_reg_read(sm5011->iodev, SM5011_REG_CNTL2, &val);	
	if (ret)
		return ret; 
	enpwrstm_sta = ((val & SM5011_ENnPWRSTM_MASK) >> SM5011_ENnPWRSTM_SHIFT);

	switch (reg_id)
	{
		case SM5011_BUCK2 ... SM5011_BUCK6:
			/* ENnPWRSTM Bit is 1 and buck is always controlled by nPWRSTM , register is set to SM5011_OPMODE_nPWRSTM */
			if (enpwrstm_sta == 1 && pdata->bucks[reg_id].always_onoff_npwrstm == 1)
			{
				pr_info("%s: [%s] Controlled by nPWRSTM\n", __func__, rdev->desc->name);
				return sm5011_reg_update(sm5011->iodev, rdev->desc->enable_reg,
								  SM5011_OPMODE_nPWRSTM, rdev->desc->enable_mask);
			}
			break;			
		default:
			break;		
	}

	return sm5011_reg_update(sm5011->iodev, rdev->desc->enable_reg,
				  ~(rdev->desc->enable_mask), rdev->desc->enable_mask);
}

static int sm5011_is_enabled(struct regulator_dev *rdev)
{
	return _sm5011_buck_is_enabled(rdev);
}

static int get_ramp_delay(int ramp_delay, int freq_val)
{
	unsigned int ramp_value = 0;

	if (freq_val == 0)
	{
		switch (ramp_delay) {
			case 1 ... 2343:
				ramp_value = RAMP_RATE_2P343MV;
				break;
			case 2344 ... 4690:
				ramp_value = RAMP_RATE_4P69MV;
				break;
			case 4691 ... 9380:
				ramp_value = RAMP_RATE_9P38MV;
				break;
			case 9381 ... 18760:
				ramp_value = RAMP_RATE_18P76MV;
				break;
			default:
				pr_warn("ramp_delay: %d not supported, setting 2343\n",
					 	ramp_delay);
				ramp_value = RAMP_RATE_2P343MV;
		}
	} else {
		switch (ramp_delay) {
			case 1 ... 3125:
				ramp_value = RAMP_RATE_3P125MV;
				break;
			case 3126 ... 6250:
				ramp_value = RAMP_RATE_6P25MV;
				break;
			case 6251 ... 12500:
				ramp_value = RAMP_RATE_12P5MV;
				break;
			case 12501 ... 25000:
				ramp_value = RAMP_RATE_25P0MV;
				break;
			default:
				pr_warn("ramp_delay: %d not supported, setting 3125\n",
					 	ramp_delay);
				ramp_value = RAMP_RATE_3P125MV;
		}
	}

	return ramp_value;
}


#define	RAMP_MASK	0x03
#define	RAMP_SHIFT	3
static int sm5011_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	struct sm5011_buck_info *sm5011 = rdev_get_drvdata(rdev);
	int reg_id = rdev_get_id(rdev);
	int ramp_shift = RAMP_SHIFT, ramp_mask = RAMP_MASK;
	unsigned int ramp_reg = 0, ramp_value = 0;

	switch (reg_id) {		
	case SM5011_BUCK2:
		ramp_value = get_ramp_delay(ramp_delay, 1);
		ramp_reg = SM5011_REG_BUCK2CNTL3;		
		break;		
	case SM5011_BUCK3:
		ramp_value = get_ramp_delay(ramp_delay, 1);
		ramp_reg = SM5011_REG_BUCK3CNTL3;		
		break;		
	case SM5011_BUCK4:
		ramp_value = get_ramp_delay(ramp_delay, 1);
		ramp_reg = SM5011_REG_BUCK4CNTL3;		
		break;		
	case SM5011_BUCK5:
		ramp_value = get_ramp_delay(ramp_delay, 1);
		ramp_reg = SM5011_REG_BUCK5CNTL3;		
		break;		
	case SM5011_BUCK6:
		ramp_value = get_ramp_delay(ramp_delay, 1);
		ramp_reg = SM5011_REG_BUCK6CNTL3;		
		break;
	default:
		return -EINVAL;
	}

	return sm5011_reg_update(sm5011->iodev, ramp_reg,
		ramp_value << ramp_shift, ramp_mask << ramp_shift);
}

static int sm5011_get_voltage_sel(struct regulator_dev *rdev)
{
	struct sm5011_buck_info *sm5011 = rdev_get_drvdata(rdev);
	int ret;
	unsigned int val;

	ret = sm5011_reg_read(sm5011->iodev, rdev->desc->vsel_reg, &val);
	if (ret)
		return ret;

	val &= rdev->desc->vsel_mask;

	return val;
}

static int sm5011_set_voltage_sel(struct regulator_dev *rdev,
								unsigned sel)
{
	int ret;
	struct sm5011_buck_info *sm5011 = rdev_get_drvdata(rdev);
	int set_val;

	set_val = rdev->desc->min_uV + (rdev->desc->uV_step * sel);

	ret = sm5011_reg_update(sm5011->iodev, rdev->desc->vsel_reg, 
		(sel & rdev->desc->vsel_mask), rdev->desc->vsel_mask);
	
	return ret;
}

static int sm5011_set_voltage_time_sel(struct regulator_dev *rdev,
				   unsigned int old_selector,
				   unsigned int new_selector)
{
	unsigned int ramp_delay = 0;
	int old_volt, new_volt;

	if (rdev->constraints->ramp_delay)
		ramp_delay = rdev->constraints->ramp_delay;
	else if (rdev->desc->ramp_delay)
		ramp_delay = rdev->desc->ramp_delay;

	if (ramp_delay == 0) {
		pr_warn("%s: ramp_delay not set\n", rdev->desc->name);
		return -EINVAL;
	}

	/* sanity check */
	if (!rdev->desc->ops->list_voltage)
		return -EINVAL;

	old_volt = rdev->desc->ops->list_voltage(rdev, old_selector);
	new_volt = rdev->desc->ops->list_voltage(rdev, new_selector);

	if (old_selector < new_selector)
		return DIV_ROUND_UP(new_volt - old_volt, ramp_delay);

	return 0;
}

static struct regulator_ops sm5011_buck_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= sm5011_is_enabled,
	.enable			= sm5011_enable,
	.disable		= sm5011_disable,
	.get_voltage_sel	= sm5011_get_voltage_sel,
	.set_voltage_sel	= sm5011_set_voltage_sel,
	.set_voltage_time_sel	= sm5011_set_voltage_time_sel,
	.set_ramp_delay		= sm5011_set_ramp_delay,
	.set_mode		= sm5011_set_mode,
	.get_mode		= sm5011_get_mode,
};

#define _BUCK(macro)	SM5011_BUCK##macro
#define _buck_ops(num)	sm5011_buck_ops##num
#define _REG_BUCKxCNTLx(num1, num2)	SM5011_REG_BUCK##num1##CNTL##num2

#define _buck_ops(num)	sm5011_buck_ops##num

#define _REG(ctrl)	SM5011_REG##ctrl


#define SM5011_BUCK_DESC(_name, _id, _ops, _mu, _us, _nv, _vr, _vm, _er, _em)	{	\
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

static struct regulator_desc sm5011_bucks_desc[SM5011_BUCKMAX] = {
	/* name, id, ops, min_uv, uV_step, n_voltages, vsel_reg, vsel_mask, enable_reg, enable_mask */
	SM5011_BUCK_DESC("BUCK2", _BUCK(2), &_buck_ops(), _BUCK(_MIN_0P6V), _BUCK(_STEP_12P5MV), _BUCK(_NUM_112),
					_BUCK(_VSEL_MASK_FF), _REG_BUCKxCNTLx(2,2), _REG_BUCKxCNTLx(2,1), _BUCK(_ENABLE_MASK_03)),
	SM5011_BUCK_DESC("BUCK3", _BUCK(3), &_buck_ops(), _BUCK(_MIN_0P6V), _BUCK(_STEP_12P5MV), _BUCK(_NUM_112), 
					_BUCK(_VSEL_MASK_FF), _REG_BUCKxCNTLx(3,2), _REG_BUCKxCNTLx(3,1), _BUCK(_ENABLE_MASK_03)),
	SM5011_BUCK_DESC("BUCK4", _BUCK(4), &_buck_ops(), _BUCK(_MIN_0P6V), _BUCK(_STEP_12P5MV), _BUCK(_NUM_112),
					_BUCK(_VSEL_MASK_FF), _REG_BUCKxCNTLx(4,2), _REG_BUCKxCNTLx(4,1), _BUCK(_ENABLE_MASK_03)),
	SM5011_BUCK_DESC("BUCK5", _BUCK(5), &_buck_ops(), _BUCK(_MIN_0P6V), _BUCK(_STEP_12P5MV), _BUCK(_NUM_112), 
					_BUCK(_VSEL_MASK_FF), _REG_BUCKxCNTLx(5,2), _REG_BUCKxCNTLx(5,1), _BUCK(_ENABLE_MASK_03)),
	SM5011_BUCK_DESC("BUCK6", _BUCK(6), &_buck_ops(), _BUCK(_MIN_0P6V), _BUCK(_STEP_12P5MV), _BUCK(_NUM_112),
					_BUCK(_VSEL_MASK_FF), _REG_BUCKxCNTLx(6,2), _REG_BUCKxCNTLx(6,1), _BUCK(_ENABLE_MASK_03)),
};

#ifdef CONFIG_OF
static int sm5011_buck_dt_parse_pdata(struct sm5011_pmic_dev *iodev,
					struct sm5011_platform_data *pdata)
{
	struct device_node *pmic_np, *buck_np, *bucks_np, *reg_np;
	struct sm5011_regulator_data *rdata;
	int i, ret;
	u32 val;

	pmic_np = iodev->dev->of_node;
	if (!pmic_np) {
		dev_err(iodev->dev, "could not find regulator sub-node\n");
		return -ENODEV;
	}

	buck_np = of_find_node_by_name(pmic_np, "sm5011-buck");

	bucks_np = of_find_node_by_name(buck_np, "bucks");
	if (!bucks_np) {
		dev_err(iodev->dev, "could not find bucks sub-node\n");
		return -EINVAL;
	}

	/* count the number of regulators to be supported in regulator */
	pdata->num_bucks = 0;
	for_each_child_of_node(bucks_np, reg_np) {
		pdata->num_bucks++;
	}

	rdata = devm_kzalloc(iodev->dev, sizeof(*rdata) *
				pdata->num_bucks, GFP_KERNEL);
	if (!rdata) {
		dev_err(iodev->dev,
			"could not allocate memory for regulator data\n");
		return -ENOMEM;
	}

	pdata->bucks = rdata;
	
	for_each_child_of_node(bucks_np, reg_np) {
		for (i = 0; i < ARRAY_SIZE(sm5011_bucks_desc); i++)
			if (!of_node_cmp(reg_np->name, sm5011_bucks_desc[i].name))
				break;

		if (i == ARRAY_SIZE(sm5011_bucks_desc)) {
			dev_warn(iodev->dev, "don't know how to configure regulator %s\n",
			reg_np->name);
			continue;
		}
		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(
						iodev->dev, reg_np, &sm5011_bucks_desc[i]);
		rdata->reg_node = reg_np;
		rdata->initdata->constraints.valid_modes_mask	= REGULATOR_MODE_FAST | REGULATOR_MODE_STANDBY;
		rdata->initdata->constraints.valid_ops_mask |= REGULATOR_CHANGE_MODE;
		ret = of_property_read_u32(rdata->reg_node, "always_onoff_npwrstm", &val);
		if (ret)
			return -EINVAL;
		rdata->always_onoff_npwrstm = val;
	
		rdata++;
	}

	pr_info("%s: Done\n", __func__);

	return 0;
}
#else
static int sm5011_buck_dt_parse_pdata(struct sm5011_pmic_dev *iodev,
					struct sm5011_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

static int sm5011_buck_preinit(struct sm5011_buck_info *info)
{
	struct sm5011_pmic_dev *iodev = info->iodev;
	struct sm5011_platform_data *pdata = iodev->pdata;
	int ret = 0;

	/*
	  * If need to add pre-initialisation, please add below
	  */
	if (iodev->rev_num == 0)
	{
		ret = sm5011_reg_update(iodev, SM5011_REG_BUCK2CNTL3, (pdata->buck2mode & 0x1), SM5011_PWM_OPMODE_MASK);
		if (ret < 0) {
			printk("%s : BUCK2MODE Pre-Init failed\n",__func__);
			return ret;
		}
		ret = sm5011_reg_update(iodev, SM5011_REG_BUCK3CNTL3, (pdata->buck3mode & 0x1), SM5011_PWM_OPMODE_MASK);
		if (ret < 0) {
			printk("%s : BUCK3MODE Pre-Init failed\n",__func__);
			return ret;
		}		
		ret = sm5011_reg_update(iodev, SM5011_REG_BUCK4CNTL3, (pdata->buck4mode & 0x1), SM5011_PWM_OPMODE_MASK);
		if (ret < 0) {
			printk("%s : BUCK4MODE Pre-Init failed\n",__func__);
			return ret;
		}		
		ret = sm5011_reg_update(iodev, SM5011_REG_BUCK5CNTL3, (pdata->buck5mode & 0x1), SM5011_PWM_OPMODE_MASK);
		if (ret < 0) {
			printk("%s : BUCK5MODE Pre-Init failed\n",__func__);
			return ret;
		}		
		ret = sm5011_reg_update(iodev, SM5011_REG_BUCK6CNTL3, (pdata->buck6mode & 0x1), SM5011_PWM_OPMODE_MASK);
		if (ret < 0) {
			printk("%s : BUCK6MODE Pre-Init failed\n",__func__);
			return ret;
		}		
	}
	
	return ret;
}


static int sm5011_buck_probe(struct platform_device *pdev)
{
	struct sm5011_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct sm5011_platform_data *pdata = iodev->pdata;
	struct regulator_config config = { };
	struct sm5011_buck_info *info;
	int i;
	int ret = 0;

	pr_info("%s : Start\n", __func__);

	info = devm_kzalloc(&pdev->dev, sizeof(struct sm5011_buck_info),
				GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	info->iodev = iodev;

	if (iodev->dev->of_node) {
		ret = sm5011_buck_dt_parse_pdata(iodev, pdata);
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

	if (gpio_is_valid(pdata->npwrstm_pin))	
		info->npwrstm_pin = pdata->npwrstm_pin;

	platform_set_drvdata(pdev, info);	

	for (i = 0; i < pdata->num_bucks; i++) {
		int id = pdata->bucks[i].id;
		config.dev = &pdev->dev;
		config.regmap = iodev->regmap;
		config.init_data = pdata->bucks[i].initdata;
		config.driver_data = info;
		config.of_node = pdata->bucks[i].reg_node;
		
		info->rdev[i] = regulator_register(&sm5011_bucks_desc[id], &config);
		if (IS_ERR(info->rdev[i])) {
			ret = PTR_ERR(info->rdev[i]);
			dev_err(&pdev->dev, "regulator init failed for %d\n", i);
			info->rdev[i] = NULL;
			goto err;
		}
	}

	info->num_bucks = pdata->num_bucks;
	
	/* Initialization */
    ret = sm5011_buck_preinit(info);
	if (ret < 0)
		goto err;
	
	pr_info("%s : Done\n", __func__);

	return 0;

err:
	for (i = 0; i < SM5011_BUCKMAX; i++)
		regulator_unregister(info->rdev[i]);

	return ret;
}

static int sm5011_buck_remove(struct platform_device *pdev)
{
	struct sm5011_buck_info *info = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < SM5011_BUCKMAX; i++)
		regulator_unregister(info->rdev[i]);

	return 0;
}

static const struct of_device_id sm5011_buck_of_match[] = {
	{ .compatible = "sm,sm5011-buck", },
	{ },
};
MODULE_DEVICE_TABLE(platform, sm5011_buck_of_match);

static const struct platform_device_id sm5011_buck_id[] = {
	{ "sm5011-buck", 0 },
};

static struct platform_driver sm5011_buck_driver = {
	.driver = {
		.name = "sm5011-buck",
		.owner = THIS_MODULE,
		.of_match_table = sm5011_buck_of_match,		
	},
	.probe = sm5011_buck_probe,
	.remove = sm5011_buck_remove,
	.id_table = sm5011_buck_id,
};

static int __init sm5011_buck_init(void)
{
	pr_info("%s\n", __func__);

	return platform_driver_register(&sm5011_buck_driver);
}
module_init(sm5011_buck_init);

static void __exit sm5011_buck_exit(void)
{
	platform_driver_unregister(&sm5011_buck_driver);
}
module_exit(sm5011_buck_exit);

MODULE_DESCRIPTION("SM5011 Regulator Buck Driver");
MODULE_LICENSE("GPL");
