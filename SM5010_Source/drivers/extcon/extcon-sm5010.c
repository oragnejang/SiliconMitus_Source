/*
 * excon-sm5010.c
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/extcon.h>

#include <sound/soc.h>

#include <linux/regulator/of_regulator.h>
#include <linux/mfd/sm5010-core.h>
#include <linux/mfd/sm5010-irq.h>
#include <linux/mfd/sm5010.h>

/* COMPCNTL */
#define SM5010_VREF_COMP_SHIFT		4
#define SM5010_VREF_COMP_MASK		(0xF << SM5010_VREF_COMP_SHIFT)
#define SM5010_COMPDUTY_SHIFT		2
#define SM5010_COMPDUTY_MASK		(3 << SM5010_COMPDUTY_SHIFT)
#define SM5010_COMPDBTIME_SHIFT		1
#define SM5010_COMPDBTIME_MASK		(1 << SM5010_COMPDBTIME_SHIFT)
#define SM5010_ENCOMP_SHIFT			0
#define SM5010_ENCOMP_MASK			(1 << SM5010_ENCOMP_SHIFT)

static const char *sm5010_cable[] = {
	"MICDET",
	NULL,
};

struct sm5010_extcon_info {
	struct device *dev;
	struct sm5010_pmic_dev *iodev;

	struct mutex lock;
	struct input_dev *input;

	struct extcon_dev *edev;

	int irq_base;
};

static int sm5010_set_vref_comp(struct device* dev, int val)
{
	struct sm5010_pmic_dev *iodev = dev_get_drvdata(dev->parent);
	int ret = 0;
	
	if (iodev == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}

	ret = sm5010_reg_update(iodev, SM5010_REG_COMPCNTL, 
		val << SM5010_VREF_COMP_SHIFT , SM5010_VREF_COMP_MASK);

	return ret;	
}

static int sm5010_set_compduty(struct device* dev, int val)
{
	struct sm5010_pmic_dev *iodev = dev_get_drvdata(dev->parent);
	int ret = 0;
	
	if (iodev == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}

	ret = sm5010_reg_update(iodev, SM5010_REG_COMPCNTL, 
		val << SM5010_COMPDUTY_SHIFT , SM5010_COMPDUTY_MASK);

	return ret;	
}

static int sm5010_set_compdbtime(struct device* dev, int val)
{
	struct sm5010_pmic_dev *iodev = dev_get_drvdata(dev->parent);
	int ret = 0;
	
	if (iodev == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}

	ret = sm5010_reg_update(iodev, SM5010_REG_COMPCNTL, 
		val << SM5010_COMPDBTIME_SHIFT , SM5010_COMPDBTIME_MASK);

	return ret;	
}

static int sm5010_set_encomp(struct device* dev, int val)
{
	struct sm5010_pmic_dev *iodev = dev_get_drvdata(dev->parent);
	int ret = 0;
	
	if (iodev == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}

	ret = sm5010_reg_update(iodev, SM5010_REG_COMPCNTL, 
		val << SM5010_ENCOMP_SHIFT , SM5010_ENCOMP_MASK);

	return ret;	
}

static irqreturn_t sm5010_extcon_micdet_irq_handler(int irq, void *data)
{
	struct sm5010_extcon_info *info = data;

	if (!info->dev)
		return IRQ_HANDLED;

	printk("%s : SM5010_MICDET\n", __func__);

	return IRQ_HANDLED;
}

const struct sm5010_irq_handler sm5010_extcon_irq_handlers[] = {
	{
		.name = "SM5010_MICDET",
		.irq_index = SM5010_IRQ1_MICDET,		
		.handler = sm5010_extcon_micdet_irq_handler,
	},
};

static int register_irq(struct platform_device *pdev,
		struct sm5010_extcon_info *info)
{
	int irq;
	int i, j;
	int ret;
	const struct sm5010_irq_handler *irq_handler = sm5010_extcon_irq_handlers;
	const char *irq_name;
    
	for (i = 0; i < ARRAY_SIZE(sm5010_extcon_irq_handlers); i++) {
		irq_name = sm5010_get_irq_name_by_index(irq_handler[i].irq_index);
		irq = platform_get_irq_byname(pdev, irq_name);
		if(irq < 0) {
			pr_err("%s, (extcon)ERROR irq = [%d] \n", __func__, irq);
			goto err_irq;
		}
		irq = info->irq_base + irq;
		ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, 
			irq_handler[i].handler, 0, irq_name, info);
		if (ret < 0) {
			pr_err("%s : (extcon)Failed to request IRQ (%s): #%d: %d\n",
					__func__, irq_name, irq, ret);
			goto err_irq;
		}

		pr_info("%s : (extcon) Register IRQ%d(%s) successfully\n",
				__func__, irq, irq_name);
	}

	return 0;
err_irq:
	for (j = 0; j < i; j++) {
		irq_name = sm5010_get_irq_name_by_index(irq_handler[j].irq_index);
		irq = platform_get_irq_byname(pdev, irq_name);
		free_irq(irq, info);
	}

	return ret;
}

static void unregister_irq(struct platform_device *pdev,
		struct sm5010_extcon_info *info)
{
	int irq;
	int i;
	const char *irq_name;
	const struct sm5010_irq_handler *irq_handler = sm5010_extcon_irq_handlers;

	for (i = 0; i < ARRAY_SIZE(sm5010_extcon_irq_handlers); i++) {
		irq_name = sm5010_get_irq_name_by_index(irq_handler[i].irq_index);
		irq = platform_get_irq_byname(pdev, irq_name);
		free_irq(irq, info);
	}
}

static int sm5010_extcon_probe(struct platform_device *pdev)
{
	struct sm5010_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct sm5010_platform_data *pdata = iodev->pdata;
	struct sm5010_extcon_info *info;
	int ret = 0;

	pr_info("%s : Start\n", __func__);
	
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	mutex_init(&info->lock);
	
	info->iodev = iodev;
	info->dev = &pdev->dev;

	info->irq_base = iodev->irq_base;
	if (!info->irq_base) {
		dev_err(&pdev->dev, "Failed to get irq base %d\n", info->irq_base);
		return -ENODEV;
	}
	
	platform_set_drvdata(pdev, info);

	info->edev = devm_extcon_dev_allocate(&pdev->dev, sm5010_cable);
	if (IS_ERR(info->edev)) {
		dev_err(&pdev->dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}
	info->edev->name = "MICDET";

	ret = devm_extcon_dev_register(&pdev->dev, info->edev);
	if (ret < 0) {
		dev_err(info->dev, "extcon_dev_register() failed: %d\n",
			ret);
		return ret;
	}

	info->input = devm_input_allocate_device(&pdev->dev);
	if (!info->input) {
		dev_err(info->dev, "Can't allocate input dev\n");
		ret = -ENOMEM;
		goto err_register;
	}

	info->input->name = "MICDET";
	info->input->phys = "sm5010/extcon";

	if (pdata->comp_vref_val)
		sm5010_set_vref_comp(&pdev->dev, pdata->comp_vref_val);

	if (pdata->comp_duty_val)
		sm5010_set_compduty(&pdev->dev, pdata->comp_duty_val);

	if (pdata->comp_time_val)
		sm5010_set_compdbtime(&pdev->dev, pdata->comp_time_val);

	if (pdata->comp_en)
		sm5010_set_encomp(&pdev->dev, pdata->comp_en);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	ret = register_irq(pdev, info);
	if (ret < 0)
        goto err_reg_irq;
	
	pm_runtime_put(&pdev->dev);

	ret = input_register_device(info->input);
	if (ret) {
		dev_err(&pdev->dev, "Can't register input device: %d\n", ret);
		goto err_input;
	}

	pr_info("%s : Done\n", __func__);

	return 0;
	
err_reg_irq:
err_input:
err_register:
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static int sm5010_extcon_remove(struct platform_device *pdev)
{
	struct sm5010_extcon_info *info = platform_get_drvdata(pdev);;

	unregister_irq(pdev, info);

	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id sm5010_extcon_of_match[] = {
	{ .compatible = "sm,sm5010-extcon", },
	{ },
};
MODULE_DEVICE_TABLE(platform, sm5010_extcon_of_match);

static const struct platform_device_id sm5010_extcon_id[] = {
	{ "sm5010-extcon", 0 },
};

static struct platform_driver sm5010_extcon_driver = {
	.driver		= {
		.name	= "sm5010-extcon",
		.owner = THIS_MODULE,
		.of_match_table = sm5010_extcon_of_match,			
	},
	.probe		= sm5010_extcon_probe,
	.remove		= sm5010_extcon_remove,
	.id_table = sm5010_extcon_id,	
};

module_platform_driver(sm5010_extcon_driver);

MODULE_DESCRIPTION("SM5010 Extcon driver");
MODULE_LICENSE("GPL");
