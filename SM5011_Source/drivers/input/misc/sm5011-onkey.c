/*
 * sm5011-onkey.c
 *
 * Copyright (c) 2018 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/sm5011-core.h>
#include <linux/mfd/sm5011-irq.h>
#include <linux/mfd/sm5011.h>
#include <linux/input/sm5011-onkey.h>
#include <linux/regmap.h>
#include <linux/slab.h>

/******************************************************
  *
  *	[External Functions]
  *
  *****************************************************/
/*int sm5011_set_envbatng(struct sm5011_onkey_info *info, int val)
{
	int ret = 0;
	
	if (info == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}	
	ret = sm5011_reg_update(info->iodev, SM5011_REG_CNTL3, 
		val<< SM5011_ENVBATNG_SHIFT , SM5011_ENVBATNG_MASK);

	return ret;
}
EXPORT_SYMBOL_GPL(sm5011_set_envbatng);

int sm5011_set_envref(struct sm5011_onkey_info *info, int val)
{
	int ret = 0;
	
	if (info == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}	
	ret = sm5011_reg_update(info->iodev, SM5011_REG_CNTL3, 
		val<< SM5011_ENVREF_SHIFT , SM5011_ENVREF_MASK);

	return ret;
}
EXPORT_SYMBOL_GPL(sm5011_set_envref);
*/
/******************************************************
  *
  *	[Internal Functions]
  *
  *****************************************************/
static int sm5011_set_longkey(struct sm5011_onkey_info *info, int val)
{
	int ret = 0;
	
	if (info == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}	
	ret = sm5011_reg_update(info->iodev, SM5011_REG_CNTL2, 
		val<< SM5011_LONGKEY_SHIFT , SM5011_LONGKEY_MASK);

	return ret;
}

static int sm5011_set_mrstbtmr(struct sm5011_onkey_info *info, int val)
{
	int ret = 0;

	if (info == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}
	
	ret = sm5011_reg_update(info->iodev, SM5011_REG_MRSTBCNTL, 
		val << SM5011_MRSTBTMR_SHIFT , SM5011_MRSTBTMR_MASK);

	return ret;
}

static int sm5011_set_enpmicoff2on(struct sm5011_onkey_info *info, int val)
{
	int ret = 0;

	if (info == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}
	
	ret = sm5011_reg_update(info->iodev, SM5011_REG_MRSTBCNTL, 
		val << SM5011_ENPMICOFF2ON_SHIFT, SM5011_ENPMICOFF2ON_MASK);

	return ret;
}

static int sm5011_set_ennresetoff2on(struct sm5011_onkey_info *info, int val)
{
	int ret = 0;

	if (info == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}
	
	ret = sm5011_reg_update(info->iodev, SM5011_REG_MRSTBCNTL, 
		val << SM5011_ENnRESETOFF2ON_SHIFT , SM5011_ENnRESETOFF2ON_MASK);

	return ret;
}

static int sm5011_set_keyoption(struct sm5011_onkey_info *info, int val)
{
	int ret = 0;

	if (info == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}
	
	ret = sm5011_reg_update(info->iodev, SM5011_REG_MRSTBCNTL, 
		val << SM5011_KEYOPTION_SHIFT , SM5011_KEYOPTION_MASK);

	return ret;
}

static int sm5011_set_enmrstb(struct sm5011_onkey_info *info, int val)
{
	int ret = 0;

	if (info == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}
	
	ret = sm5011_reg_update(info->iodev, SM5011_REG_MRSTBCNTL, 
		val << SM5011_ENMRSTB_SHIFT , SM5011_ENMRSTB_MASK);

	return ret;
}

static int sm5011_onkey_preinit(struct sm5011_onkey_info *info)
{
	struct sm5011_platform_data *pdata = dev_get_platdata(info->iodev->dev);
	int ret = 0;

//	sm5011_set_smpltmr(info, pdata->smpl_timer_val);
//	sm5011_set_smpl_auto(info, pdata->smpl_power_on_type);

	sm5011_set_longkey(info, pdata->longkey_val);

	sm5011_set_mrstbtmr(info, pdata->mrstb_timer_val);	
	sm5011_set_enpmicoff2on(info, pdata->mrstb_hehavior);
	sm5011_set_ennresetoff2on(info, pdata->mrstb_nreset);
	sm5011_set_keyoption(info, pdata->mrstb_key);
	sm5011_set_enmrstb(info, pdata->mrstb_en);

	/*
	  * If need to add pre-initialisation, please add below
	  */
	  
	return ret;
}

/******************************************************
  *
  *	[Interrupt Functions]
  *
  *****************************************************/

static irqreturn_t sm5011_onkey_vbat_valid_irq_handler(int irq, void *data)
{
	struct sm5011_onkey_info *info = data;

	if (!info->dev)
		return IRQ_HANDLED;

    printk("%s : SM5011_VBAT_VALID\n", __func__);

	return IRQ_HANDLED;
}

static irqreturn_t sm5011_onkey_manualrst_irq_handler(int irq, void *data)
{
	struct sm5011_onkey_info *info = data;

	if (!info->dev)
		return IRQ_HANDLED;

    printk("%s : SM5011_MANUALRST\n", __func__);

	return IRQ_HANDLED;
}

static irqreturn_t sm5011_onkey_longkey_mrstb_irq_handler(int irq, void *data)
{
	struct sm5011_onkey_info *info = data;

	if (!info->dev)
		return IRQ_HANDLED;

    printk("%s : SM5011_LONGKEY_MRSTB\n", __func__);

	//If needed to add function, Please add here.

	return IRQ_HANDLED;
}

static irqreturn_t sm5011_onkey_longkey_chgon_irq_handler(int irq, void *data)
{
	struct sm5011_onkey_info *info = data;

	if (!info->dev)
		return IRQ_HANDLED;

    printk("%s : SM5011_LONGKEY_CHGON\n", __func__);

	//If needed to add function, Please add here.

	return IRQ_HANDLED;
}

static irqreturn_t sm5011_onkey_longkey_nonkey_irq_handler(int irq, void *data)
{
	struct sm5011_onkey_info *info = data;

	if (!info->dev)
		return IRQ_HANDLED;

    printk("%s : SM5011_LONGKEY_nONKEY\n", __func__);

	//If needed to add function, Please add here. 
	
	return IRQ_HANDLED;
}

static irqreturn_t sm5011_onkey_shortkey_irq_handler(int irq, void *data)
{
	struct sm5011_onkey_info *info = data;

	if (!info->dev)
		return IRQ_HANDLED;

    printk("%s : SM5011_SHORTKEY\n", __func__);

	//If needed to add function, Please add here.

	return IRQ_HANDLED;
}

const struct sm5011_irq_handler sm5011_onkey_irq_handlers[] = {
	{
		.name = "SM5011_VBAT_VALID",
		.irq_index = SM5011_IRQ1_VBAT_VALID,		
		.handler = sm5011_onkey_vbat_valid_irq_handler,
	},
	{
		.name = "SM5011_MANUALRST",
		.irq_index = SM5011_IRQ1_MANUALRST,		
		.handler = sm5011_onkey_manualrst_irq_handler,
	},
	{
		.name = "SM5011_LONGKEY_MRSTB",
		.irq_index = SM5011_IRQ1_LONGKEY_MRSTB,		
		.handler = sm5011_onkey_longkey_mrstb_irq_handler,
	},
	{
		.name = "SM5011_LONGKEY_CHGON",
		.irq_index = SM5011_IRQ1_LONGKEY_CHGON,		
		.handler = sm5011_onkey_longkey_chgon_irq_handler,
	},
	{
		.name = "SM5011_LONGKEY_nONKEY",
		.irq_index = SM5011_IRQ1_LONGKEY_nONKEY,		
		.handler = sm5011_onkey_longkey_nonkey_irq_handler,
	},	
	{
		.name = "SM5011_SHORTKEY",
		.irq_index = SM5011_IRQ1_SHORTKEY,		
		.handler = sm5011_onkey_shortkey_irq_handler,
	},
};

static int register_irq(struct platform_device *pdev,
		struct sm5011_onkey_info *info)
{
	int irq;
	int i, j;
	int ret;
	const struct sm5011_irq_handler *irq_handler = sm5011_onkey_irq_handlers;
	const char *irq_name;
	for (i = 0; i < ARRAY_SIZE(sm5011_onkey_irq_handlers); i++) {
		irq_name = sm5011_get_irq_name_by_index(irq_handler[i].irq_index);
		irq = platform_get_irq_byname(pdev, irq_name);
		if(irq < 0) {
			pr_err("%s,(onkey) ERROR irq = [%d] \n", __func__, irq);
			goto err_irq;
		}
		irq = info->irq_base + irq;
		ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, 
			irq_handler[i].handler, 0, irq_name, info);
		if (ret < 0) {
			pr_err("%s :(onkey) Failed to request IRQ (%s): #%d: %d\n",
					__func__, irq_name, irq, ret);
			goto err_irq;
		}

		pr_info("%s :(onkey) Register IRQ%d(%s) successfully\n",
				__func__, irq, irq_name);
	}

	return 0;
err_irq:
	for (j = 0; j < i; j++) {
		irq_name = sm5011_get_irq_name_by_index(irq_handler[j].irq_index);
		irq = platform_get_irq_byname(pdev, irq_name);
		free_irq(irq, info);
	}

	return ret;
}

static void unregister_irq(struct platform_device *pdev,
		struct sm5011_onkey_info *info)
{
	int irq;
	int i;
	const char *irq_name;
	const struct sm5011_irq_handler *irq_handler = sm5011_onkey_irq_handlers;

	for (i = 0; i < ARRAY_SIZE(sm5011_onkey_irq_handlers); i++) {
		irq_name = sm5011_get_irq_name_by_index(irq_handler[i].irq_index);
		irq = platform_get_irq_byname(pdev, irq_name);
		free_irq(irq, info);
	}
}

static int sm5011_onkey_probe(struct platform_device *pdev)
{
	struct sm5011_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct sm5011_onkey_info *info;
	struct input_dev *input;
	int error;

	pr_info("%s : Start\n", __func__);

	info = kzalloc(sizeof(struct sm5011_onkey_info), GFP_KERNEL);
	input = input_allocate_device();
	if (!info || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	info->iodev = iodev;
	info->idev = input;
	info->dev = &pdev->dev;

	input->name = "sm5011_on";
	input->phys = "sm5011_on/input0";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &pdev->dev;
	input_set_capability(input, EV_KEY, KEY_POWER);

	info->irq_base = iodev->irq_base;
	if (!info->irq_base) {
		dev_err(&pdev->dev, "Failed to get irq base %d\n", info->irq_base);
		return -ENODEV;
	}

    error = register_irq(pdev, info);    
	if (error < 0) {
		goto err_free_mem;
	}

	/* Initialisation */
	sm5011_onkey_preinit(info);

	error = input_register_device(info->idev);
	if (error) {
		dev_err(&pdev->dev, "Can't register input device: %d\n", error);
		goto err_free_irq;
	}

	platform_set_drvdata(pdev, info);
	device_init_wakeup(&pdev->dev, 1);

	pr_info("%s : Done\n", __func__);

	return 0;

err_free_irq:
err_free_mem:
	input_free_device(input);
	kfree(info);
    
	return error;
}

static int sm5011_onkey_remove(struct platform_device *pdev)
{
	struct sm5011_onkey_info *info = platform_get_drvdata(pdev);

    unregister_irq(pdev, info);
	input_unregister_device(info->idev);
	kfree(info);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sm5011_onkey_suspend(struct device *dev)
{
	return 0;
}

static int sm5011_onkey_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(sm5011_onkey_pm_ops, sm5011_onkey_suspend, sm5011_onkey_resume);
#endif

#ifdef CONFIG_OF
static struct of_device_id sm5011_onkey_match_table[] = {
    { .compatible = "sm,sm5011-onkey",},
    {},
};
#else
#define sm5011_onkey_match_table NULL
#endif

static struct platform_driver sm5011_onkey_driver = {
	.driver		= {
		.name	= "sm5011-onkey",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_SLEEP		
		.pm	= &sm5011_onkey_pm_ops,
#endif		
        .of_match_table = sm5011_onkey_match_table,       		
	},
	.probe		= sm5011_onkey_probe,
	.remove		= sm5011_onkey_remove,
};
module_platform_driver(sm5011_onkey_driver);

MODULE_DESCRIPTION("Siliconmitus SM5011 ONKEY driver");
MODULE_LICENSE("GPL");
