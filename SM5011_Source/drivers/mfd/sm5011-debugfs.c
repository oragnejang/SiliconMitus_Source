/*
 * sm5011-debugfs.c
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
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/mfd/core.h>
#include <linux/mfd/sm5011.h>
#include <linux/mfd/sm5011-core.h>
#include <linux/mfd/sm5011-debugfs.h>
#include <linux/mfd/sm5011-npwrstm.h>
#include <linux/mfd/sm5011-irq.h>
#include <linux/rtc/rtc-sm5011.h>
#include <linux/regmap.h>
#include <linux/proc_fs.h>

enum {
	POWERON_STATUS = 0,
	POWEROFF_STATUS,
	DUMP_REGISTER,
};

static const struct sm5011_debugfs_reg32 sm5011_debug_regs[] = {
	sm5011_dump_register(PWRONREG),	
	sm5011_dump_register(PWROFFREG),	
	sm5011_dump_register(REBOOTREG),	
	//sm5011_dump_register(INT1), 		
	//sm5011_dump_register(INT2), 		
	sm5011_dump_register(INTMSK1),		
	sm5011_dump_register(INTMSK2),	
	sm5011_dump_register(STATUS),	

	sm5011_dump_register(CNTL1),
	sm5011_dump_register(CNTL2),
	sm5011_dump_register(CNTL3),
	sm5011_dump_register(MRSTBCNTL),
	sm5011_dump_register(WDTCNTL),
		
	sm5011_dump_register(BUCK2CNTL1),
	sm5011_dump_register(BUCK2CNTL2),
	sm5011_dump_register(BUCK2CNTL3),
	sm5011_dump_register(BUCK3CNTL1),
	sm5011_dump_register(BUCK3CNTL2),
	sm5011_dump_register(BUCK3CNTL3),
	sm5011_dump_register(BUCK4CNTL1),
	sm5011_dump_register(BUCK4CNTL2),
	sm5011_dump_register(BUCK4CNTL3),
	sm5011_dump_register(BUCK5CNTL1),
	sm5011_dump_register(BUCK5CNTL2),
	sm5011_dump_register(BUCK5CNTL3),
	sm5011_dump_register(BUCK6CNTL1),
	sm5011_dump_register(BUCK6CNTL2),
	sm5011_dump_register(BUCK6CNTL3),
	sm5011_dump_register(LDO1CNTL1),
	sm5011_dump_register(LDO1CNTL2),
	sm5011_dump_register(LDO2CNTL1),
	sm5011_dump_register(LDO2CNTL2),
	sm5011_dump_register(LDO3CNTL1),
	sm5011_dump_register(LDO3CNTL2),
	sm5011_dump_register(LDO4CNTL1),
	sm5011_dump_register(LDO4CNTL2),
	sm5011_dump_register(LDO5CNTL1),
	sm5011_dump_register(LDO5CNTL2),
	sm5011_dump_register(LDO6CNTL1),
	sm5011_dump_register(LDO6CNTL2),
	sm5011_dump_register(LDO7CNTL1),
	sm5011_dump_register(LDO7CNTL2), 
	sm5011_dump_register(LDO8CNTL1),
	sm5011_dump_register(LDO8CNTL2), 
	sm5011_dump_register(LDO9CNTL1),
	sm5011_dump_register(LDO9CNTL2), 
	sm5011_dump_register(LDO10CNTL1),
	sm5011_dump_register(LDO10CNTL2), 
	sm5011_dump_register(LDO11CNTL1),	
	sm5011_dump_register(LDO11CNTL2),								
	sm5011_dump_register(LDO12CNTL1),	
	sm5011_dump_register(LDO12CNTL2),	
 	sm5011_dump_register(LDO13CNTL1),	
	sm5011_dump_register(LDO13CNTL2),	 
	sm5011_dump_register(LDO14CNTL1),	
	sm5011_dump_register(LDO14CNTL2),	
	sm5011_dump_register(LDO15CNTL1),	
	sm5011_dump_register(LDO15CNTL2),	 
	sm5011_dump_register(LDO16CNTL1),	
	sm5011_dump_register(LDO16CNTL2),	 
	sm5011_dump_register(LDO17CNTL1),	
	sm5011_dump_register(LDO17CNTL2),	 
	sm5011_dump_register(LDO18CNTL1),	
	sm5011_dump_register(LDO18CNTL2),	 
	sm5011_dump_register(LDO19CNTL1),	
	sm5011_dump_register(LDO19CNTL2),	 
	sm5011_dump_register(LDO20CNTL1),	
	sm5011_dump_register(LDO20CNTL2),	 
	sm5011_dump_register(COMPCNTL),	
			
	sm5011_dump_register(RTCCNTL1),	
	sm5011_dump_register(RTCCNTL2),	
	sm5011_dump_register(RTCCNTL3),	
	sm5011_dump_register(RTCCNTL4),	
	sm5011_dump_register(RTCCNTL5),	
	sm5011_dump_register(RTCCNTL6),	
	sm5011_dump_register(RTCCNTL7),	 
	sm5011_dump_register(RTCALM1CNTL1),
	sm5011_dump_register(RTCALM1CNTL2),
	sm5011_dump_register(RTCALM1CNTL3),
	sm5011_dump_register(RTCALM1CNTL4),
	sm5011_dump_register(RTCALM1CNTL5),
	sm5011_dump_register(RTCALM1CNTL6),
	sm5011_dump_register(RTCALM1CNTL7), 
	sm5011_dump_register(RTCALM2CNTL1),
	sm5011_dump_register(RTCALM2CNTL2),
	sm5011_dump_register(RTCALM2CNTL3),
	sm5011_dump_register(RTCALM2CNTL4),
	sm5011_dump_register(RTCALM2CNTL5),
	sm5011_dump_register(RTCALM2CNTL6),
	sm5011_dump_register(RTCALM2CNTL7), 
	sm5011_dump_register(SECCNTL1),	
	sm5011_dump_register(SECCNTL2),	
	sm5011_dump_register(SECCNTL3),	
	sm5011_dump_register(SECCNTL4),	
	sm5011_dump_register(AUTHCNTL1),	
	sm5011_dump_register(AUTHCNTL2),	
		
	sm5011_dump_register(DEVICEID),	
};

ssize_t sm5011_debugfs_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sm5011_debugfs_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

#define SM5011_DEBUGFS_ATTR(_name, _mode)						\
{									\
	.attr = {.name = #_name, .mode = _mode},	\
	.show = sm5011_debugfs_show_attrs,					\
	.store = sm5011_debugfs_store_attrs,					\
}

static struct device_attribute sm5011_debugfs_attrs[] = {
	SM5011_DEBUGFS_ATTR(poweron_status, 0664),
	SM5011_DEBUGFS_ATTR(poweroff_status, 0664),
	SM5011_DEBUGFS_ATTR(dump_register, 0444),	
};


ssize_t sm5011_debugfs_show_attrs(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sm5011_pmic_dev *sm5011_pmic = dev_get_drvdata(dev->parent);	
	const ptrdiff_t offset = attr - sm5011_debugfs_attrs;	
	int i = 0, val = 0, j = 0;
	
	switch (offset) {
	case POWERON_STATUS:
		sm5011_reg_read(sm5011_pmic, SM5011_REG_PWRONREG, &val);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			val);
		break;
	case POWEROFF_STATUS:
		sm5011_reg_read(sm5011_pmic, SM5011_REG_PWROFFREG, &val);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			val);
		break;
	case DUMP_REGISTER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "========= Reg Dump =========\n");		
		for (j = 0; j < ARRAY_SIZE(sm5011_debug_regs); j++)
		{
			sm5011_reg_read(sm5011_pmic, sm5011_debug_regs[j].reg, &val);		
			i += scnprintf(buf + i, PAGE_SIZE - i, "%s(0x%x) = 0x%x\n",
				sm5011_debug_regs[j].name, sm5011_debug_regs[j].reg, val);			
		}
		i += scnprintf(buf + i, PAGE_SIZE - i, "============================\n");		
		break;
	default:
		i = -EINVAL;
		break;
	}

	return i;
}

ssize_t sm5011_debugfs_store_attrs(
					struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct sm5011_pmic_dev *sm5011_pmic = dev_get_drvdata(dev->parent);	
	const ptrdiff_t offset = attr - sm5011_debugfs_attrs;	
	int ret = -EINVAL;
	int x = 0;

	switch (offset) {
	case POWERON_STATUS:
		if (sscanf(buf, "%d\n", &x) == 1) {
			printk("%s : POWERON_STATUS = %d\n", __func__, x);
			ret = count;
		}
		break;
	case POWEROFF_STATUS:
		if (sscanf(buf, "%d\n", &x) == 1) {
			pr_info("%s : POWEROFF_STATUS = %d\n", __func__, x);
			ret = count;
		}
		break;
	case DUMP_REGISTER:
		pr_info("%s : Not support DUMP_REGISTER Store\n", __func__);
		break;	
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void sm5011_debugfs_device_release(struct device *dev)
{
	struct sm5011_debugfs_device *sm5011_debugfs_dev = to_sm5011_debugfs_device(dev);
	kfree(sm5011_debugfs_dev);
}

int sm5011_debugfs_create_attrs(struct device *dev)
{
	struct sm5011_debugfs_device *sm5011_debugfs_dev;	
	unsigned long i;
	int rc;

	pr_info("%s : Start\n", __func__);

	sm5011_debugfs_dev = kzalloc(sizeof(*sm5011_debugfs_dev), GFP_KERNEL);
	if (!sm5011_debugfs_dev)
		return -ENOMEM;

	mutex_init(&sm5011_debugfs_dev->ops_lock);
	sm5011_debugfs_dev->dev.parent = dev;
	sm5011_debugfs_dev->dev.release = sm5011_debugfs_device_release;
	dev_set_name(&sm5011_debugfs_dev->dev, "sm5011-debugfs");

	rc = device_register(&sm5011_debugfs_dev->dev);
	if (rc) {
		kfree(sm5011_debugfs_dev);
		return rc;
	}

	for (i = 0; i < ARRAY_SIZE(sm5011_debugfs_attrs); i++) {
		rc = device_create_file(&sm5011_debugfs_dev->dev, &sm5011_debugfs_attrs[i]);
		if (rc)
			goto create_attrs_failed;
	}
	goto create_attrs_succeed;

create_attrs_failed:
	while (i--)
		device_remove_file(&sm5011_debugfs_dev->dev, &sm5011_debugfs_attrs[i]);
create_attrs_succeed:
	pr_info("%s : Done\n", __func__);
	
	return rc;
}
EXPORT_SYMBOL(sm5011_debugfs_create_attrs);

