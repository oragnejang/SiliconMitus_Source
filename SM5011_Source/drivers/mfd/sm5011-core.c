/*
 * sm5011-core.c
 *
 * Copyright (c) 2018 SILICONMITUS COMPANY,LTD
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
#include <linux/of_irq.h>
#include <linux/irqnr.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/mfd/core.h>
#include <linux/mfd/sm5011.h>
#include <linux/mfd/sm5011-core.h>
#include <linux/mfd/sm5011-debugfs.h>
#include <linux/mfd/sm5011-npwrstm.h>
#include <linux/mfd/sm5011-irq.h>
#include <linux/mfd/sm5011-cntl.h>
#include <linux/rtc/rtc-sm5011.h>
#include <linux/regmap.h>

static DEFINE_MUTEX(sm5011_lock);

#define SM5011_DECLARE_IRQ(irq, name) { \
	irq, irq, \
	SM5011##_##name##_NAME, IORESOURCE_IRQ }

const static struct resource sm5011_buck_res[] = {
};

const static struct resource sm5011_ldo_res[] = {
};

const static struct resource sm5011_rtc_res[] = {
	SM5011_DECLARE_IRQ(SM5011_IRQ2_ALARM2_ON, IRQ2_ALARM2_ON),		
	SM5011_DECLARE_IRQ(SM5011_IRQ2_ALARM1_ON, IRQ2_ALARM1_ON),	
};

const static struct resource sm5011_wdt_res[] = {
	SM5011_DECLARE_IRQ(SM5011_IRQ2_WDTMEROUT, IRQ2_WDTMEROUT),	
};

const static struct resource sm5011_onkey_res[] = {
	SM5011_DECLARE_IRQ(SM5011_IRQ1_VBAT_VALID, IRQ1_VBAT_VALID),
	SM5011_DECLARE_IRQ(SM5011_IRQ1_MANUALRST, IRQ1_MANUALRST),	
	SM5011_DECLARE_IRQ(SM5011_IRQ1_LONGKEY_MRSTB, IRQ1_LONGKEY_MRSTB),
	SM5011_DECLARE_IRQ(SM5011_IRQ1_LONGKEY_CHGON, IRQ1_LONGKEY_CHGON),
	SM5011_DECLARE_IRQ(SM5011_IRQ1_LONGKEY_nONKEY, IRQ1_LONGKEY_nONKEY),
	SM5011_DECLARE_IRQ(SM5011_IRQ1_SHORTKEY, IRQ1_SHORTKEY),	
};

const static struct resource sm5011_extcon_res[] = {
	SM5011_DECLARE_IRQ(SM5011_IRQ1_MICDET, IRQ1_MICDET),
};

static struct mfd_cell sm5011_devs[] = {
	{
		.name = "sm5011-buck",
		.num_resources = ARRAY_SIZE(sm5011_buck_res),
		.resources = sm5011_buck_res,	
	}, {	
		.name = "sm5011-ldo",
		.num_resources = ARRAY_SIZE(sm5011_ldo_res),
		.resources = sm5011_ldo_res,	
	}, {		
		.name = "sm5011-rtc",
		.num_resources = ARRAY_SIZE(sm5011_rtc_res),
		.resources = sm5011_rtc_res,		
	}, {
		.name = "sm5011-wdt",
		.num_resources = ARRAY_SIZE(sm5011_wdt_res),
		.resources = sm5011_wdt_res,
	}, {
		.name = "sm5011-onkey",
		.num_resources = ARRAY_SIZE(sm5011_onkey_res),
		.resources = sm5011_onkey_res,
	}, 
	{
		.name = "sm5011-extcon",
		.num_resources = ARRAY_SIZE(sm5011_extcon_res),
		.resources = sm5011_extcon_res,
	},
};

#ifdef CONFIG_OF
static struct of_device_id sm5011_dt_match[] = {
	{	.compatible = "sm,sm5011-pmic",
	},
	{},
};
#endif

int sm5011_reg_read(struct sm5011_pmic_dev *sm5011_pmic, u32 reg, void *dest)
{
	return regmap_read(sm5011_pmic->regmap, reg, dest);
}
EXPORT_SYMBOL_GPL(sm5011_reg_read);

int sm5011_bulk_read(struct sm5011_pmic_dev *sm5011_pmic, u32 reg, int count, u8 *buf)
{
	return regmap_bulk_read(sm5011_pmic->regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(sm5011_bulk_read);

int sm5011_reg_write(struct sm5011_pmic_dev *sm5011_pmic, u32 reg, u32 value)
{
	if (sm5011_pmic->rev_num == 0 
		&& (reg == SM5011_REG_INTMSK1 || reg == SM5011_REG_INTMSK2 || (reg >= SM5011_REG_CNTL1 && reg <= SM5011_REG_WDTCNTL) || (reg >= SM5011_REG_RTCCNTL1 && reg <= SM5011_REG_AUTHCNTL2)))
		regmap_write(sm5011_pmic->regmap, 0xff, value);
	
	return regmap_write(sm5011_pmic->regmap, reg, value);
}
EXPORT_SYMBOL_GPL(sm5011_reg_write);

int sm5011_bulk_write(struct sm5011_pmic_dev *sm5011_pmic, u32 reg, int count, u8 *buf)
{
	if (sm5011_pmic->rev_num == 0)
	{
		int i = 0, ret = 0;
		
		for (i = 0; i < count; i++)
			ret = sm5011_reg_write(sm5011_pmic, reg+i, *(buf+i));
		return ret;	
	} else
		return regmap_raw_write(sm5011_pmic->regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(sm5011_bulk_write);

int sm5011_reg_update(struct sm5011_pmic_dev *sm5011_pmic, u32 reg, u32 val, u32 mask)
{
	if (sm5011_pmic->rev_num == 0 )
	{
		int ret;
		unsigned int tmp, orig;

		ret = sm5011_reg_read(sm5011_pmic, reg, &orig);
		if (ret != 0)
			return ret;

		tmp = orig & ~mask;
		tmp |= val & mask;

		if (tmp != orig)
			ret = sm5011_reg_write(sm5011_pmic, reg, tmp);

		return ret;
	}else
		return regmap_update_bits(sm5011_pmic->regmap, reg, mask, val);
}
EXPORT_SYMBOL_GPL(sm5011_reg_update);

void sm5011_core_lock(void)
{
	mutex_lock(&sm5011_lock);
}
EXPORT_SYMBOL_GPL(sm5011_core_lock);

void sm5011_core_unlock(void)
{
	mutex_unlock(&sm5011_lock);
}
EXPORT_SYMBOL_GPL(sm5011_core_unlock);

static struct regmap_config sm5011_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};


#ifdef CONFIG_OF
static struct sm5011_platform_data *sm5011_pmic_i2c_parse_dt_pdata(
					struct device *dev)
{
	struct sm5011_platform_data *pdata;
	struct device_node *np, *np_sm_pmic;
	int ret;
	u32 val;

	pr_info("%s: Start\n", __func__);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "failed to allocate platform data\n");
		return ERR_PTR(-ENOMEM);
	}
	dev->platform_data = pdata;
	pdata->irq_base = -1;

	np = dev->of_node;

	np_sm_pmic = of_find_node_by_name(NULL, "sm-pmic");
	if (!np_sm_pmic) {
		dev_err(dev, "could not find sm-pmic node\n");
		return ERR_PTR(-ENODEV);
	}

	/* Buck2 */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,buck2mode",
			&pdata->buck2mode);
	if (ret)
		return ERR_PTR(ret);

	/* Buck3 */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,buck3mode",
			&pdata->buck3mode);
	if (ret)
		return ERR_PTR(ret);

	/* Buck4 */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,buck4mode",
			&pdata->buck4mode);
	if (ret)
		return ERR_PTR(ret);

	/* Buck5 */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,buck5mode",
			&pdata->buck5mode);
	if (ret)
		return ERR_PTR(ret);

	/* Buck6 */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,buck6mode",
			&pdata->buck6mode);
	if (ret)
		return ERR_PTR(ret);
	
    /* LDO6 */
	pdata->entcxo_pin = of_get_named_gpio(np_sm_pmic, "sm-pmic,entcxo-pin", 0);
	
	/* LDO16 */
	pdata->enl16_pin = of_get_named_gpio(np_sm_pmic, "sm-pmic,enl16-pin", 0);

	/* Interrupt */
	pdata->irq_gpio = of_get_named_gpio(np_sm_pmic, "sm-pmic,irq-gpio", 0);

	/* 32k */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,en_32kout", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->en_32kout = !!val;	
	
	/* SMPL */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,smpl_en", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->smpl_en = !!val;

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,smpl_timer_val",
			&pdata->smpl_timer_val);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,smpl_power_on_type", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->smpl_power_on_type = !!val;
	
	/* nPWRSTM */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,npwrstm_en", &pdata->npwrstm_en);
	if (ret)
		return ERR_PTR(ret);

	/* Longkey */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,longkey_val",
			&pdata->longkey_val);
	if (ret)
		return ERR_PTR(ret);

	/* MASK Interrupt */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,mask_int_en", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->mask_int_en = !!val;

	/* ENVBATNG */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,envbatng_en", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->envbatng_en = !!val;

	/* ENVREF */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,envref_en", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->envref_en = !!val;


	/* MRSTB */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,mrstb_en", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->mrstb_en = !!val;

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,mrstb_timer_val",
			&pdata->mrstb_timer_val);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,mrstb_hehavior", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->mrstb_hehavior = !!val;

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,mrstb_nreset", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->mrstb_nreset = !!val;

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,mrstb_key", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->mrstb_key = !!val;

	/* WDT */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,wdt_en", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->wdt_en = !!val;

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,wdt_timer_val",
			&pdata->wdt_timer_val);
	if (ret)
		return ERR_PTR(ret);

	/* COMP */
	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,comp_en", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->comp_en = !!val;

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,comp_time_val",
			&pdata->comp_time_val);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,comp_duty_val",
			&pdata->comp_duty_val);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,comp_vref_val",
			&pdata->comp_vref_val);
	if (ret)
		return ERR_PTR(ret);

	/* Init RTC */
	pdata->init_time = devm_kzalloc(dev, sizeof(*pdata->init_time),
			GFP_KERNEL);
	if (!pdata->init_time)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,rtc_24hr_mode",
			&pdata->rtc_24hr_mode);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,init_time,sec",
			&pdata->init_time->tm_sec);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,init_time,min",
			&pdata->init_time->tm_min);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,init_time,hour",
			&pdata->init_time->tm_hour);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,init_time,wday",
			&pdata->init_time->tm_wday);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,init_time,mday",
			&pdata->init_time->tm_mday);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,init_time,mon",
			&pdata->init_time->tm_mon);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np_sm_pmic, "sm-pmic,init_time,year",
			&pdata->init_time->tm_year);
	if (ret)
		return ERR_PTR(ret);

	pr_info("%s: Done\n", __func__);

	return pdata;
}
#else
static struct sm5011_platform_data *sm5011_pmic_i2c_parse_dt_pdata(
					struct device *dev)
{
	return NULL;
}
#endif

static int sm5011_pmic_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct sm5011_platform_data *pdata = i2c->dev.platform_data;
	struct sm5011_pmic_dev *sm5011_pmic;
	int ret = 0;
	unsigned int temp;

	pr_info("%s: Start\n", __func__);

	sm5011_pmic = devm_kzalloc(&i2c->dev, sizeof(struct sm5011_pmic_dev),
				GFP_KERNEL);
	if (sm5011_pmic == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, sm5011_pmic);
	sm5011_pmic->dev = &i2c->dev;
	sm5011_pmic->i2c = i2c;

	mutex_init(&sm5011_pmic->iolock);

	if (sm5011_pmic->dev->of_node) {
		pdata = sm5011_pmic_i2c_parse_dt_pdata(sm5011_pmic->dev);
		if (IS_ERR(pdata)) {
			ret = PTR_ERR(pdata);
			return ret;
		}
	}

	if (pdata) {
		sm5011_pmic->irq_base = pdata->irq_base;
		sm5011_pmic->wakeup = true;
		sm5011_pmic->pdata = pdata;
		sm5011_pmic->irq_gpio = pdata->irq_gpio;
		
		sm5011_pmic->irq = gpio_to_irq(sm5011_pmic->irq_gpio);
		pr_info("%s: irq=%d, irq_gpio=%d\n", __func__,
			sm5011_pmic->irq, sm5011_pmic->irq_gpio);
	}

	sm5011_pmic->regmap = devm_regmap_init_i2c(i2c, &sm5011_regmap_config);
	if (IS_ERR(sm5011_pmic->regmap)) {
		ret = PTR_ERR(sm5011_pmic->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = sm5011_reg_read(sm5011_pmic, SM5011_REG_DEVICEID, &temp);
	if (ret < 0)
		return ret;

	SM5011_PMIC_REV(sm5011_pmic) = (temp >> SM5011_MAST_REV_SHIFT);

	pr_info("%s : REV = 0x%x\n", __func__, sm5011_pmic->rev_num);

	/* nPWRSTM control interface Initialization */
	sm5011_npwrstm_interface_init(sm5011_pmic);
	/* nPWRSTM GPIO Initialization */
	sm5011_npwrstm_gpio_init(sm5011_pmic);	
	/* nPWRSTM Initialization */
	sm5011_npwrstm_preinit();

	/* CNTL control interface Initialization */
	sm5011_cntl_interface_init(sm5011_pmic);
	/* CNTL Initialization */
	sm5011_cntl_preinit();

	/* Irq Initialization*/
	ret = sm5011_irq_init(sm5011_pmic);
	if (ret < 0)
		goto irq_err;

	pm_runtime_set_active(sm5011_pmic->dev);
	ret = mfd_add_devices(sm5011_pmic->dev, -1, sm5011_devs,
			      ARRAY_SIZE(sm5011_devs), NULL, 0, NULL);
	if (ret < 0)
		goto err;

	/* Debugfs Initialization */
	sm5011_debugfs_create_attrs(sm5011_pmic->dev);

	pr_info("%s: Done\n", __func__);

	return ret;

err:
	mfd_remove_devices(sm5011_pmic->dev);
irq_err:	
	sm5011_irq_exit(sm5011_pmic);
	
	return ret;
}

static int sm5011_pmic_remove(struct i2c_client *i2c)
{
	struct sm5011_pmic_dev *sm5011_pmic = i2c_get_clientdata(i2c);

	mfd_remove_devices(sm5011_pmic->dev);
	sm5011_irq_exit(sm5011_pmic);
	regmap_exit(sm5011_pmic->regmap);

	return 0;
}

static const struct i2c_device_id sm5011_pmic_id[] = {
	{ "sm5011_pmic", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5011_pmic_id);

#ifdef CONFIG_PM
static int sm5011_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct sm5011_pmic_dev *sm5011_pmic = i2c_get_clientdata(i2c);

	if (sm5011_pmic->wakeup)
		enable_irq_wake(sm5011_pmic->irq);

	disable_irq(sm5011_pmic->irq);

	return 0;
}

static int sm5011_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct sm5011_pmic_dev *sm5011_pmic = i2c_get_clientdata(i2c);

	if (sm5011_pmic->wakeup)
		disable_irq_wake(sm5011_pmic->irq);

	enable_irq(sm5011_pmic->irq);

	return 0;
}
#else
#define sm5011_suspend	NULL
#define sm5011_resume	NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops sm5011_pmic_apm = {
	.suspend = sm5011_suspend,
	.resume = sm5011_resume,
};

static struct i2c_driver sm5011_pmic_driver = {
	.driver = {
		   .name = "sm5011-pmic",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(sm5011_dt_match),
		   .pm = &sm5011_pmic_apm,
	},
	.probe = sm5011_pmic_probe,
	.remove = sm5011_pmic_remove,
	.id_table = sm5011_pmic_id,
};

static int __init sm5011_pmic_init(void)
{
	pr_info("%s\n", __func__);

	return i2c_add_driver(&sm5011_pmic_driver);
}

subsys_initcall(sm5011_pmic_init);

static void __exit sm5011_pmic_exit(void)
{
	i2c_del_driver(&sm5011_pmic_driver);
}
module_exit(sm5011_pmic_exit);

MODULE_DESCRIPTION("Core support for the SM5011 MFD");
MODULE_LICENSE("GPL");
