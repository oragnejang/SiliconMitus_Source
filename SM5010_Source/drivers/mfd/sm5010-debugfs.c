/*
 * sm5010-debugfs.c
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
#include <linux/of_irq.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/mfd/core.h>
#include <linux/mfd/sm5010.h>
#include <linux/mfd/sm5010-core.h>
#include <linux/mfd/sm5010-dvs.h>
#include <linux/mfd/sm5010-debugfs.h>
#include <linux/mfd/sm5010-npwrstm.h>
#include <linux/mfd/sm5010-irq.h>
#include <linux/rtc/rtc-sm5010.h>
#include <linux/regmap.h>
#include <linux/proc_fs.h>

enum {
	POWERON_STATUS = 0,
	POWEROFF_STATUS,
	DVS_LEVEL_STATUS,
	nPWRSTM_LEVEL_STATUS,
	DUMP_REGISTER,
};

static const struct sm5010_debugfs_reg32 sm5010_debug_regs[] = {
	sm5010_dump_register(PWRONREG),	
	sm5010_dump_register(PWROFFREG),	
	sm5010_dump_register(REBOOTREG),	
	//sm5010_dump_register(INT1), 		
	//sm5010_dump_register(INT2), 		
	sm5010_dump_register(INTMSK1),		
	sm5010_dump_register(INTMSK2),	
	sm5010_dump_register(STATUS),	

	sm5010_dump_register(CNTL1),
	sm5010_dump_register(CNTL2),
	sm5010_dump_register(CNTL3),
	sm5010_dump_register(MRSTBCNTL),
	sm5010_dump_register(WDTCNTL),
		
	sm5010_dump_register(BUCK1CNTL1),
	sm5010_dump_register(BUCK1CNTL2),
	sm5010_dump_register(BUCK1CNTL3),
	sm5010_dump_register(BUCK1CNTL4),
	sm5010_dump_register(BUCK1CNTL5),
	sm5010_dump_register(BUCK1CNTL6),
	sm5010_dump_register(BUCK1CNTL7),
	sm5010_dump_register(BUCK1CNTL8),
	sm5010_dump_register(BUCK1CNTL9),
	sm5010_dump_register(BUCK1CNTL10),
	sm5010_dump_register(BUCK1CNTL11),
	sm5010_dump_register(BUCK2CNTL1),
	sm5010_dump_register(BUCK2CNTL2),
	sm5010_dump_register(BUCK2CNTL3),
	sm5010_dump_register(BUCK3CNTL1),
	sm5010_dump_register(BUCK3CNTL2),
	sm5010_dump_register(BUCK3CNTL3),
	sm5010_dump_register(BUCK4CNTL1),
	sm5010_dump_register(BUCK4CNTL2),
	sm5010_dump_register(BUCK4CNTL3),
	sm5010_dump_register(BUCK5CNTL1),
	sm5010_dump_register(BUCK5CNTL2),
	sm5010_dump_register(BUCK5CNTL3),
	sm5010_dump_register(BUCK6CNTL1),
	sm5010_dump_register(BUCK6CNTL2),
	sm5010_dump_register(BUCK6CNTL3),
	sm5010_dump_register(PABUCKCNTL1),
	sm5010_dump_register(PABUCKCNTL2),
	sm5010_dump_register(LDO1CNTL1),
	sm5010_dump_register(LDO1CNTL2),
	sm5010_dump_register(LDO2CNTL1),
	sm5010_dump_register(LDO2CNTL2),
	sm5010_dump_register(LDO3CNTL1),
	sm5010_dump_register(LDO3CNTL2),
	sm5010_dump_register(LDO4CNTL1),
	sm5010_dump_register(LDO4CNTL2),
	sm5010_dump_register(LDO5CNTL1),
	sm5010_dump_register(LDO5CNTL2),
	sm5010_dump_register(LDO6CNTL1),
	sm5010_dump_register(LDO6CNTL2),
	sm5010_dump_register(LDO7CNTL1),
	sm5010_dump_register(LDO7CNTL2), 
	sm5010_dump_register(LDO8CNTL1),
	sm5010_dump_register(LDO8CNTL2), 
	sm5010_dump_register(LDO9CNTL1),
	sm5010_dump_register(LDO9CNTL2), 
	sm5010_dump_register(LDO10CNTL1),
	sm5010_dump_register(LDO10CNTL2), 
	sm5010_dump_register(LDO11CNTL1),	
	sm5010_dump_register(LDO11CNTL2),								
	sm5010_dump_register(LDO12CNTL1),	
	sm5010_dump_register(LDO12CNTL2),	
 	sm5010_dump_register(LDO13CNTL1),	
	sm5010_dump_register(LDO13CNTL2),	 
	sm5010_dump_register(LDO14CNTL1),	
	sm5010_dump_register(LDO14CNTL2),	
	sm5010_dump_register(LDO15CNTL1),	
	sm5010_dump_register(LDO15CNTL2),	 
	sm5010_dump_register(LDO16CNTL1),	
	sm5010_dump_register(LDO16CNTL2),	 
	sm5010_dump_register(LDO17CNTL1),	
	sm5010_dump_register(LDO17CNTL2),	 
	sm5010_dump_register(LDO18CNTL1),	
	sm5010_dump_register(LDO18CNTL2),	 
	sm5010_dump_register(LDO19CNTL1),	
	sm5010_dump_register(LDO19CNTL2),	 
	sm5010_dump_register(LDO20CNTL1),	
	sm5010_dump_register(LDO20CNTL2),	 
	sm5010_dump_register(COMPCNTL),	
			
	sm5010_dump_register(RTCCNTL1),	
	sm5010_dump_register(RTCCNTL2),	
	sm5010_dump_register(RTCCNTL3),	
	sm5010_dump_register(RTCCNTL4),	
	sm5010_dump_register(RTCCNTL5),	
	sm5010_dump_register(RTCCNTL6),	
	sm5010_dump_register(RTCCNTL7),	 
	sm5010_dump_register(RTCALM1CNTL1),
	sm5010_dump_register(RTCALM1CNTL2),
	sm5010_dump_register(RTCALM1CNTL3),
	sm5010_dump_register(RTCALM1CNTL4),
	sm5010_dump_register(RTCALM1CNTL5),
	sm5010_dump_register(RTCALM1CNTL6),
	sm5010_dump_register(RTCALM1CNTL7), 
	sm5010_dump_register(RTCALM2CNTL1),
	sm5010_dump_register(RTCALM2CNTL2),
	sm5010_dump_register(RTCALM2CNTL3),
	sm5010_dump_register(RTCALM2CNTL4),
	sm5010_dump_register(RTCALM2CNTL5),
	sm5010_dump_register(RTCALM2CNTL6),
	sm5010_dump_register(RTCALM2CNTL7), 
	sm5010_dump_register(SECCNTL1),	
	sm5010_dump_register(SECCNTL2),	
	sm5010_dump_register(SECCNTL3),	
	sm5010_dump_register(SECCNTL4),	
	sm5010_dump_register(AUTHCNTL1),	
	sm5010_dump_register(AUTHCNTL2),	
		
	sm5010_dump_register(DEVICEID),	
};

ssize_t sm5010_debugfs_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sm5010_debugfs_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

#define SM5010_DEBUGFS_ATTR(_name, _mode)						\
{									\
	.attr = {.name = #_name, .mode = _mode},	\
	.show = sm5010_debugfs_show_attrs,					\
	.store = sm5010_debugfs_store_attrs,					\
}

static struct device_attribute sm5010_debugfs_attrs[] = {
	SM5010_DEBUGFS_ATTR(poweron_status, 0664),
	SM5010_DEBUGFS_ATTR(poweroff_status, 0664),
	SM5010_DEBUGFS_ATTR(dvs_level_status, 0664),
	SM5010_DEBUGFS_ATTR(npwrstm_level_status, 0664),
	SM5010_DEBUGFS_ATTR(dump_register, 0444),	
};


ssize_t sm5010_debugfs_show_attrs(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sm5010_pmic_dev *sm5010_pmic = dev_get_drvdata(dev->parent);	
	const ptrdiff_t offset = attr - sm5010_debugfs_attrs;	
	int i = 0, val = 0, j = 0;
	u8 array_val[SM5010_BUCK1DVSMAX]= {0,};
	
	switch (offset) {
	case POWERON_STATUS:
		sm5010_reg_read(sm5010_pmic, SM5010_REG_PWRONREG, &val);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			val);
		break;
	case POWEROFF_STATUS:
		sm5010_reg_read(sm5010_pmic, SM5010_REG_PWROFFREG, &val);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			val);
		break;
	case DVS_LEVEL_STATUS:
		sm5010_bulk_read(sm5010_pmic, SM5010_REG_BUCK1CNTL2, SM5010_BUCK1DVSMAX, array_val);
		i += scnprintf(buf + i, PAGE_SIZE - i, "LLL=0x%x\nLLH=0x%x\nLHL=0x%x\nLHH=0x%x\nHLL=0x%x\nHLH=0x%x\nHHL=0x%x\nHHH=0x%x\n",
			array_val[0],array_val[1],array_val[2],array_val[3],array_val[4],array_val[5],
			array_val[6],array_val[7]);
		break;
	case nPWRSTM_LEVEL_STATUS:
		sm5010_reg_read(sm5010_pmic, SM5010_REG_BUCK1CNTL10, &val);
		i += scnprintf(buf + i, PAGE_SIZE - i, "nPWRSTM=0x%x\n",
			val);
		break;
	case DUMP_REGISTER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "========= Reg Dump =========\n");		
		for (j = 0; j < ARRAY_SIZE(sm5010_debug_regs); j++)
		{
			sm5010_reg_read(sm5010_pmic, sm5010_debug_regs[j].reg, &val);		
			i += scnprintf(buf + i, PAGE_SIZE - i, "%s(0x%x) = 0x%x\n",
				sm5010_debug_regs[j].name, sm5010_debug_regs[j].reg, val);			
		}
		i += scnprintf(buf + i, PAGE_SIZE - i, "============================\n");		
		break;
	default:
		i = -EINVAL;
		break;
	}

	return i;
}

ssize_t sm5010_debugfs_store_attrs(
					struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct sm5010_pmic_dev *sm5010_pmic = dev_get_drvdata(dev->parent);	
	const ptrdiff_t offset = attr - sm5010_debugfs_attrs;	
	int ret = -EINVAL;
	int x = 0;
	int array_val[SM5010_BUCK1DVSMAX]= {0,};

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
	case DVS_LEVEL_STATUS:
		if (sscanf(buf,"%d %d %d %d %d %d %d %d\n", 
			&array_val[0],&array_val[1],&array_val[2],&array_val[3],
			&array_val[4],&array_val[5],&array_val[6],&array_val[7]) == SM5010_BUCK1DVSMAX) {
			sm5010_bulk_write(sm5010_pmic,SM5010_REG_BUCK1CNTL2,SM5010_BUCK1DVSMAX, (u8*)array_val);
			pr_info("%s : DVS_LEVEL_STATUS = %d\n", __func__, x);
			ret = count;
		}
		break;
	case nPWRSTM_LEVEL_STATUS:
		if (sscanf(buf,"%d\n", &x) == 1) {
			sm5010_reg_write(sm5010_pmic,SM5010_REG_BUCK1CNTL10, x);
			pr_info("%s : nPWRSTM_LEVEL_STATUS = %d\n", __func__, x);
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

static void sm5010_debugfs_device_release(struct device *dev)
{
	struct sm5010_debugfs_device *sm5010_debugfs_dev = to_sm5010_debugfs_device(dev);
	kfree(sm5010_debugfs_dev);
}

int sm5010_debugfs_create_attrs(struct device *dev)
{
	struct sm5010_debugfs_device *sm5010_debugfs_dev;	
	unsigned long i;
	int rc;

	pr_info("%s : Start\n", __func__);

	sm5010_debugfs_dev = kzalloc(sizeof(*sm5010_debugfs_dev), GFP_KERNEL);
	if (!sm5010_debugfs_dev)
		return -ENOMEM;

	mutex_init(&sm5010_debugfs_dev->ops_lock);
	sm5010_debugfs_dev->dev.parent = dev;
	sm5010_debugfs_dev->dev.release = sm5010_debugfs_device_release;
	dev_set_name(&sm5010_debugfs_dev->dev, "sm5010-debugfs");

	rc = device_register(&sm5010_debugfs_dev->dev);
	if (rc) {
		kfree(sm5010_debugfs_dev);
		return rc;
	}

	for (i = 0; i < ARRAY_SIZE(sm5010_debugfs_attrs); i++) {
		rc = device_create_file(&sm5010_debugfs_dev->dev, &sm5010_debugfs_attrs[i]);
		if (rc)
			goto create_attrs_failed;
	}
	goto create_attrs_succeed;

create_attrs_failed:
	while (i--)
		device_remove_file(&sm5010_debugfs_dev->dev, &sm5010_debugfs_attrs[i]);
create_attrs_succeed:
	pr_info("%s : Done\n", __func__);
	
	return rc;
}
EXPORT_SYMBOL(sm5010_debugfs_create_attrs);

