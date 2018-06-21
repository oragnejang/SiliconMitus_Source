/*
 * rtc-sm5011.c
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
#include <linux/gpio.h>
#include <linux/rtc.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/mfd/sm5011-core.h>
#include <linux/mfd/sm5011-irq.h>
#include <linux/rtc/rtc-sm5011.h>
#include <linux/mfd/sm5011.h>

static struct wakeup_source *rtc_ws;

static int sm5011_set_32kout(struct sm5011_rtc_info *info, int enable)
{
	int ret = 0;

	if (info == NULL){
		pr_err("%s : Info Error\n", __func__);
		return -EINVAL;
	}
	
	ret = sm5011_reg_update(info->iodev, SM5011_REG_CNTL1, 
		enable << SM5011_EN32KOUT_SHIFT , SM5011_EN32KOUT_MASK);

	return ret;
}

/*
 * Does the rtc_time represent a valid date/time
 */
int sm5011_rtc_valid_tm(struct rtc_time *tm)
{
	if (tm->tm_year < 70
		|| ((unsigned)tm->tm_mon) > 12
		|| tm->tm_mday < 1
		|| tm->tm_mday > rtc_month_days(tm->tm_mon-1, tm->tm_year + 1900)
		|| ((unsigned)tm->tm_hour) >= 24
		|| ((unsigned)tm->tm_min) >= 60
		|| ((unsigned)tm->tm_sec) >= 60)
		return -EINVAL;

	return 0;
}

static void sm5011_data_to_tm(u8 *data, struct rtc_time *tm,
					   int rtc_24hr_mode)
{	
	tm->tm_sec = bcd2bin(data[RTC_SEC]);
	tm->tm_min = bcd2bin(data[RTC_MIN]);

	if (data[RTC_HOUR] & MODE12_MASK) {
		tm->tm_hour = bcd2bin(data[RTC_HOUR] & 0x0f);
		if (tm->tm_hour == 12)
			tm->tm_hour = 0;
		if (data[RTC_HOUR] & HOUR_PM_MASK)
			tm->tm_hour += 12;
	} else {
		tm->tm_hour = bcd2bin(data[RTC_HOUR] & 0x3f);
	}	

	tm->tm_wday = bcd2bin(data[RTC_WEEKDAY]) - 1;
	tm->tm_mday = bcd2bin(data[RTC_DATE]); 
	tm->tm_mon = bcd2bin(data[RTC_MONTH]);
	tm->tm_year = bcd2bin(data[RTC_YEAR]) + 100;
	tm->tm_yday = 0;
	tm->tm_isdst = 0;
}

static int sm5011_tm_to_data(struct rtc_time *tm, u8 *data, int rtc_24hr_mode)
{
	data[RTC_SEC] = bin2bcd(tm->tm_sec);
	data[RTC_MIN] = bin2bcd(tm->tm_min);
	data[RTC_HOUR] = bin2bcd(tm->tm_hour);
	data[RTC_WEEKDAY] = bin2bcd(tm->tm_wday) + 1;
	data[RTC_DATE] = bin2bcd(tm->tm_mday);
	data[RTC_MONTH] = bin2bcd(tm->tm_mon + 1);

	pr_info("%s: %d-%02d-%02d %02d:%02d:%02d(0x%02x)\n",
				__func__, tm->tm_year + 1900, tm->tm_mon + 1,
				tm->tm_mday, tm->tm_hour, tm->tm_min,
				tm->tm_sec, tm->tm_wday + 1);

	if (tm->tm_year > 100)		
		data[RTC_YEAR] = bin2bcd(tm->tm_year - 100); 
	else 
		data[RTC_YEAR] = 0; //Base  is from 2000 year.
		
	if (tm->tm_year < 100) {
		pr_warn("%s: SM5011 RTC cannot handle the year %d, changed time to 2000y\n", __func__,
				1900 + tm->tm_year);
		//return -EINVAL;
	}
	return 0;
}

static int sm5011_rtc_alarm_control(struct sm5011_rtc_info *info,
				 enum SM5011_RTCALM_OP op, int select)
{
	u32 mask, reg, shift;
	int ret;

	if (!info || !info->iodev) {
		pr_err("%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	switch (op) {
	case SM5011_RTC_ALARM1_PMIC:
		mask = ALARM1_PMIC_MASK;
		shift = ALARM1_PMIC_SHIFT;
		reg = SM5011_REG_RTCALM1CNTL4;
		break;
	case SM5011_RTC_ALARM1_EN:
		mask = ALARM1_ENABLE_MASK;
		shift = ALARM1_ENABLE_SHIFT;
		reg = SM5011_REG_RTCALM1CNTL4;	
		break;
	case SM5011_RTC_ALARM2_PMIC:
		mask = ALARM2_PMIC_MASK;
		shift = ALARM2_PMIC_SHIFT;
		reg = SM5011_REG_RTCALM2CNTL4;
		break;		
	case SM5011_RTC_ALARM2_EN:
		mask = ALARM2_ENABLE_MASK;
		shift = ALARM2_ENABLE_SHIFT;		
		reg = SM5011_REG_RTCALM2CNTL4;
		break;

	default:
		dev_err(info->dev, "%s: invalid op(%d)\n", __func__, op);
		return -EINVAL;
	}

	ret = sm5011_reg_update(info->iodev, reg, select << shift, mask);
	if (ret < 0)
		dev_err(info->dev, "%s: fail to write update reg(%d,%u)\n",
				__func__, ret, select);

	return ret;
}

static int sm5011_rtc1_read_time(struct device *dev, struct rtc_time *tm)
{
	struct sm5011_rtc_info *info = dev_get_drvdata(dev);
	u8 data[NR_RTC_CNT_REGS];
	int ret;

	mutex_lock(&info->lock);

	ret = sm5011_bulk_read(info->iodev, SM5011_REG_RTCCNTL1, NR_RTC_CNT_REGS,
			data);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to read time reg(%d)\n", __func__,
			ret);
		goto out;
	}

	sm5011_data_to_tm(data, tm, info->rtc_24hr_mode);

	dev_info(info->dev, "%s: %d-%02d-%02d %02d:%02d:%02d(0x%02x)%s\n",
			__func__, tm->tm_year + 1900, tm->tm_mon,
			tm->tm_mday, tm->tm_hour, tm->tm_min,
			tm->tm_sec, tm->tm_wday + 1,
			info->rtc_24hr_mode ? "24hm" : ((data[RTC_HOUR] & HOUR_PM_MASK) ? "PM" : "AM"));

	ret = sm5011_rtc_valid_tm(tm);
	if (ret < 0) {
		dev_err(info->dev, "%s: non valid tm(%d)\n", __func__,
			ret);
		goto out;
	}
out:
	mutex_unlock(&info->lock);
	return ret;
}

static int sm5011_rtc1_set_time(struct device *dev, struct rtc_time *tm)
{
	struct sm5011_rtc_info *info = dev_get_drvdata(dev);
	u8 data[NR_RTC_CNT_REGS];
	int ret;

	ret = sm5011_tm_to_data(tm, data, info->rtc_24hr_mode);
	if (ret < 0)
		return ret;
	
	if (info->rtc_24hr_mode == 1) //24hr mode
		data[RTC_HOUR] &= ~MODE12_MASK;
	else if(info->rtc_24hr_mode == 1) //12hr mode
		data[RTC_HOUR] |= MODE12_MASK;

	dev_info(info->dev, "%s: %d-%02d-%02d %02d:%02d:%02d(0x%02x)%s\n",
			__func__, tm->tm_year + 1900, tm->tm_mon + 1,
			tm->tm_mday, tm->tm_hour, tm->tm_min,
			tm->tm_sec, tm->tm_wday + 1,
			info->rtc_24hr_mode ? "24hm" : ((data[RTC_HOUR] & HOUR_PM_MASK) ? "PM" : "AM"));

	mutex_lock(&info->lock);

	ret = sm5011_bulk_write(info->iodev, SM5011_REG_RTCCNTL1, NR_RTC_CNT_REGS,
			data);

	if (ret < 0) {
		dev_err(info->dev, "%s: fail to write time reg(%d)\n", __func__,
			ret);
		goto out;
	}

out:
	mutex_unlock(&info->lock);
	return ret;
}

static int sm5011_rtc1_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct sm5011_rtc_info *info = dev_get_drvdata(dev);
	u8 data[NR_RTC_CNT_REGS];
	int ret;

	mutex_lock(&info->lock);
	
	ret = sm5011_bulk_read(info->iodev, SM5011_REG_RTCALM1CNTL1, NR_RTC_CNT_REGS,
			data);

	if (ret < 0) {
		dev_err(info->dev, "%s:%d fail to read alarm reg(%d)\n",
			__func__, __LINE__, ret);
		goto out;
	}

	sm5011_data_to_tm(data, &alrm->time, info->rtc_24hr_mode);

	dev_info(info->dev, "%s: %d-%02d-%02d %02d:%02d:%02d(%d)\n", __func__,
			alrm->time.tm_year + 1900, alrm->time.tm_mon + 1,
			alrm->time.tm_mday, alrm->time.tm_hour,
			alrm->time.tm_min, alrm->time.tm_sec,
			alrm->time.tm_wday + 1);

	alrm->enabled = info->alarm1_enabled;

out:
	mutex_unlock(&info->lock);
	return ret;
}

static int sm5011_rtc1_set_alarm_enable(struct sm5011_rtc_info *info, bool enabled)
{
	if (!info->use_irq)
		return -EPERM;

	if (enabled && !info->alarm1_enabled) {
		info->alarm1_enabled = true;
		enable_irq(info->irq_base + SM5011_IRQ2_ALARM1_ON);
	} else if (!enabled && info->alarm1_enabled) {
		info->alarm1_enabled = false;
		disable_irq(info->irq_base + SM5011_IRQ2_ALARM1_ON);
	}
	
	return 0;
}

static int sm5011_rtc1_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct sm5011_rtc_info *info = dev_get_drvdata(dev);
	u8 data[NR_RTC_CNT_REGS], temp;
	int ret;

	mutex_lock(&info->lock);
	ret = sm5011_tm_to_data(&alrm->time, data, info->rtc_24hr_mode);
	if (ret < 0)
		goto out;

	if (info->rtc_24hr_mode == 0) //24hr mode
		data[RTC_HOUR] &= ~MODE12_MASK;
	else if(info->rtc_24hr_mode == 1) //12hr mode
		data[RTC_HOUR] |= MODE12_MASK;

	dev_info(info->dev, "%s: %d-%02d-%02d %02d:%02d:%02d(%d)\n", __func__,
			alrm->time.tm_year + 1900, alrm->time.tm_mon + 1,
			alrm->time.tm_mday, alrm->time.tm_hour,
			alrm->time.tm_min, alrm->time.tm_sec,
			alrm->time.tm_wday + 1);

	if (info->alarm1_check) {		
		/* ALARM Disable */
		ret = sm5011_rtc_alarm_control(info, SM5011_RTC_ALARM1_EN, SM5011_ALARM_DIS);
		if (ret < 0)
			goto out;
		
		ret = sm5011_reg_read(info->iodev, SM5011_REG_RTCALM1CNTL4, &temp);
		if (ret < 0)
			goto out;
		
		/* ALARM_PMIC | ALARM_EN*/
		data[RTC_WEEKDAY] &= ~ALARM1_ENABLE_MASK;
		data[RTC_WEEKDAY] &= ~ALARM1_PMIC_MASK;
		data[RTC_WEEKDAY] |= (temp & ALARM1_PMIC_MASK);

		ret = sm5011_bulk_write(info->iodev, SM5011_REG_RTCALM1CNTL1, NR_RTC_CNT_REGS,
				data);

		if (ret < 0) {
			dev_err(info->dev, "%s: fail to disable alarm reg(%d)\n",
				__func__, ret);
			goto out;
		}
	}
	
	/* ALARM Enable */
	ret = sm5011_rtc_alarm_control(info, SM5011_RTC_ALARM1_EN, SM5011_ALARM_EN);
	if (ret < 0)
		goto out;

	ret = sm5011_rtc1_set_alarm_enable(info, alrm->enabled);
out:
	mutex_unlock(&info->lock);
	return ret;
}

static int sm5011_rtc1_alarm_irq_enable(struct device *dev,
					unsigned int enabled)
{
	struct sm5011_rtc_info *info = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&info->lock);
	ret = sm5011_rtc1_set_alarm_enable(info, enabled);
	mutex_unlock(&info->lock);
	return ret;
}

static int sm5011_rtc2_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct sm5011_rtc_info *info = dev_get_drvdata(dev);
	u8 data[NR_RTC_CNT_REGS];
	int ret;

	mutex_lock(&info->lock);
	
	ret = sm5011_bulk_read(info->iodev, SM5011_REG_RTCALM2CNTL1, NR_RTC_CNT_REGS,
			data);

	if (ret < 0) {
		dev_err(info->dev, "%s:%d fail to read alarm reg(%d)\n",
			__func__, __LINE__, ret);
		goto out;
	}

	sm5011_data_to_tm(data, &alrm->time, info->rtc_24hr_mode);

	dev_info(info->dev, "%s: %d-%02d-%02d %02d:%02d:%02d(%d)\n", __func__,
			alrm->time.tm_year + 1900, alrm->time.tm_mon + 1,
			alrm->time.tm_mday, alrm->time.tm_hour,
			alrm->time.tm_min, alrm->time.tm_sec,
			alrm->time.tm_wday + 1);

	alrm->enabled = info->alarm2_enabled;

out:
	mutex_unlock(&info->lock);
	return ret;
}

static int sm5011_rtc2_set_alarm_enable(struct sm5011_rtc_info *info, bool enabled)
{
	if (!info->use_irq)
		return -EPERM;

	if (enabled && !info->alarm2_enabled) {
		info->alarm2_enabled = true;
		enable_irq(info->irq_base + SM5011_IRQ2_ALARM2_ON);
	} else if (!enabled && info->alarm2_enabled) {
		info->alarm2_enabled = false;
		disable_irq(info->irq_base + SM5011_IRQ2_ALARM2_ON);
	}
	
	return 0;
}

static int sm5011_rtc2_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct sm5011_rtc_info *info = dev_get_drvdata(dev);
	u8 data[NR_RTC_CNT_REGS], temp;
	int ret;

	mutex_lock(&info->lock);
	ret = sm5011_tm_to_data(&alrm->time, data, info->rtc_24hr_mode);
	if (ret < 0)
		goto out;

	if (info->rtc_24hr_mode == 0) //24hr mode
		data[RTC_HOUR] &= ~MODE12_MASK;
	else if(info->rtc_24hr_mode == 1) //12hr mode
		data[RTC_HOUR] |= MODE12_MASK;

	dev_info(info->dev, "%s: %d-%02d-%02d %02d:%02d:%02d(%d)\n", __func__,
			alrm->time.tm_year + 1900, alrm->time.tm_mon + 1,
			alrm->time.tm_mday, alrm->time.tm_hour,
			alrm->time.tm_min, alrm->time.tm_sec,
			alrm->time.tm_wday + 1);

	if (info->alarm2_check) {		
		/* ALARM Disable */
		ret = sm5011_rtc_alarm_control(info, SM5011_RTC_ALARM2_EN, SM5011_ALARM_DIS);
		if (ret < 0)
			goto out;
		
		ret = sm5011_reg_read(info->iodev, SM5011_REG_RTCALM2CNTL4, &temp);
		if (ret < 0)
			goto out;
		
		/* ALARM_PMIC | ALARM_EN*/
		data[RTC_WEEKDAY] &= ~ALARM2_ENABLE_MASK;
		data[RTC_WEEKDAY] &= ~ALARM2_PMIC_MASK;
		data[RTC_WEEKDAY] |= (temp & ALARM1_PMIC_MASK);

		ret = sm5011_bulk_write(info->iodev, SM5011_REG_RTCALM2CNTL1, NR_RTC_CNT_REGS,
				data);

		if (ret < 0) {
			dev_err(info->dev, "%s: fail to disable alarm reg(%d)\n",
				__func__, ret);
			goto out;
		}
	}
	
	/* ALARM Enable */
	ret = sm5011_rtc_alarm_control(info, SM5011_RTC_ALARM2_EN, SM5011_ALARM_EN);
	if (ret < 0)
		goto out;

	ret = sm5011_rtc2_set_alarm_enable(info, alrm->enabled);
out:
	mutex_unlock(&info->lock);
	return ret;
}

static int sm5011_rtc2_alarm_irq_enable(struct device *dev,
					unsigned int enabled)
{
	struct sm5011_rtc_info *info = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&info->lock);
	ret = sm5011_rtc2_set_alarm_enable(info, enabled);
	mutex_unlock(&info->lock);
	return ret;
}


static int sm5011_rtc_init_reg(struct sm5011_rtc_info *info,
				struct sm5011_platform_data *pdata)
{
	int ret;

	//pr_info("%s\n", __func__);

	/* Set RTC control register : Binary mode, 24hour mode */
	info->rtc_24hr_mode = pdata->rtc_24hr_mode;
	
	ret = sm5011_reg_update(info->iodev, SM5011_REG_RTCCNTL3, info->rtc_24hr_mode << MODE12_SHIFT, MODE12_MASK);

	if (ret < 0) {
		dev_err(info->dev, "%s: fail to write CTRL reg(%d)\n",
			__func__, ret);
		return ret;
	}

	if (pdata->init_time) {
		dev_info(info->dev, "%s: initialize RTC time\n", __func__);
		ret = sm5011_rtc1_set_time(info->dev, pdata->init_time);
	}

	/* Set 32kout */
	sm5011_set_32kout(info, pdata->en_32kout);

	return ret;
}

static const struct rtc_class_ops sm5011_rtc1_ops = {
	.read_time = sm5011_rtc1_read_time,
	.set_time = sm5011_rtc1_set_time,
	.read_alarm = sm5011_rtc1_read_alarm,
	.set_alarm = sm5011_rtc1_set_alarm,
	.alarm_irq_enable = sm5011_rtc1_alarm_irq_enable,
};

static const struct rtc_class_ops sm5011_rtc2_ops = {
	.read_alarm = sm5011_rtc2_read_alarm,
	.set_alarm = sm5011_rtc2_set_alarm,
	.alarm_irq_enable = sm5011_rtc2_alarm_irq_enable,
};

static irqreturn_t sm5011_rtc_alarm1_irq_handler(int irq, void *data)
{
	struct sm5011_rtc_info *info = data;

	if (!info->rtc1_dev)
		return IRQ_HANDLED;

	dev_info(info->dev, "%s:irq(%d)\n", __func__, irq);

	rtc_update_irq(info->rtc1_dev, 1, RTC_IRQF | RTC_AF);
	__pm_wakeup_event(rtc_ws, 500);
	return IRQ_HANDLED;
}

static irqreturn_t sm5011_rtc_alarm2_irq_handler(int irq, void *data)
{
	struct sm5011_rtc_info *info = data;

	if (!info->rtc2_dev)
		return IRQ_HANDLED;

	dev_info(info->dev, "%s:irq(%d)\n", __func__, irq);

	rtc_update_irq(info->rtc2_dev, 1, RTC_IRQF | RTC_AF);
	__pm_wakeup_event(rtc_ws, 500);
	return IRQ_HANDLED;
}

const struct sm5011_irq_handler sm5011_rtc_irq_handlers[] = {
	{
		.name = "SM5011_ALARM1_ON",
		.irq_index = SM5011_IRQ2_ALARM1_ON,		
		.handler = sm5011_rtc_alarm1_irq_handler,
	},
	{
		.name = "SM5011_ALARM2_ON",
		.irq_index = SM5011_IRQ2_ALARM2_ON,		
		.handler = sm5011_rtc_alarm2_irq_handler,
	},	
};

static int register_irq(struct platform_device *pdev,
		struct sm5011_rtc_info *info)
{
	int irq;
	int i, j;
	int ret;
	const struct sm5011_irq_handler *irq_handler = sm5011_rtc_irq_handlers;
	const char *irq_name;
    
	for (i = 0; i < ARRAY_SIZE(sm5011_rtc_irq_handlers); i++) {
		irq_name = sm5011_get_irq_name_by_index(irq_handler[i].irq_index);
		irq = platform_get_irq_byname(pdev, irq_name);
		if(irq < 0) {
			pr_err("%s,(rtc) ERROR irq = [%d] \n", __func__, irq);
			goto err_irq;
		}
		irq = info->irq_base + irq;
		ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, 
			irq_handler[i].handler, 0, irq_name, info);
		if (ret < 0) {
			pr_err("%s :(rtc) Failed to request IRQ (%s): #%d: %d\n",
					__func__, irq_name, irq, ret);
			goto err_irq;
		}

		pr_info("%s :(rtc) Register IRQ%d(%s) successfully\n",
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
		struct sm5011_rtc_info *info)
{
	int irq;
	int i;
	const char *irq_name;
	const struct sm5011_irq_handler *irq_handler = sm5011_rtc_irq_handlers;

	for (i = 0; i < ARRAY_SIZE(sm5011_rtc_irq_handlers); i++) {
		irq_name = sm5011_get_irq_name_by_index(irq_handler[i].irq_index);
		irq = platform_get_irq_byname(pdev, irq_name);
		free_irq(irq, info);
	}
}

static int sm5011_rtc_probe(struct platform_device *pdev)
{
	struct sm5011_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct sm5011_platform_data *pdata = dev_get_platdata(iodev->dev);
	struct sm5011_rtc_info *info;
	int ret = 0;

	pr_info("%s: Start\n", __func__);

	info = devm_kzalloc(&pdev->dev, sizeof(struct sm5011_rtc_info),
				GFP_KERNEL);

	if (!info)
		return -ENOMEM;

	info->irq_base = iodev->irq_base;

	if (!info->irq_base) {
		dev_err(&pdev->dev, "Failed to get irq base %d\n", info->irq_base);
		return -ENODEV;
	}

	mutex_init(&info->lock);
	info->dev = &pdev->dev;
	info->iodev = iodev;
	info->alarm1_check = true;
	info->alarm2_check = true;

	platform_set_drvdata(pdev, info);

	ret = sm5011_rtc_init_reg(info, pdata);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to initialize RTC reg:%d\n", ret);
		goto err_rtc_init_reg;
	}

	device_init_wakeup(&pdev->dev, true);
	rtc_ws = wakeup_source_register("rtc-sm5011");

	ret = register_irq(pdev, info);
	
	if (ret < 0)
        goto err_rtc_irq; 

	info->use_irq = true;

	info->rtc1_dev = devm_rtc_device_register(&pdev->dev, "sm5011-rtc1",
			&sm5011_rtc1_ops, THIS_MODULE);

	if (IS_ERR(info->rtc1_dev)) {
		ret = PTR_ERR(info->rtc1_dev);
		dev_err(&pdev->dev, "Failed to register RTC device: %d\n", ret);
		goto err_rtc_dev_register;
	}

	info->rtc2_dev = devm_rtc_device_register(&pdev->dev, "sm5011-rtc2",
			&sm5011_rtc2_ops, THIS_MODULE);

	if (IS_ERR(info->rtc2_dev)) {
		ret = PTR_ERR(info->rtc2_dev);
		dev_err(&pdev->dev, "Failed to register RTC device: %d\n", ret);
		goto err_rtc_dev_register;
	}


	pr_info("%s: Done\n", __func__);

	return 0;

err_rtc_dev_register:
err_rtc_irq:
	wakeup_source_unregister(rtc_ws);
err_rtc_init_reg:
	platform_set_drvdata(pdev, NULL);
	mutex_destroy(&info->lock);

	return ret;
}

static int sm5011_rtc_remove(struct platform_device *pdev)
{
	struct sm5011_rtc_info *info = platform_get_drvdata(pdev);

	unregister_irq(pdev, info);

	wakeup_source_unregister(rtc_ws);

	return 0;
}

static void sm5011_rtc_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id sm5011_rtc_of_match[] = {
	{ .compatible = "sm,sm5011-rtc", },
	{ },
};
MODULE_DEVICE_TABLE(of, sm5011_rtc_of_match);

static const struct platform_device_id sm5011_rtc_id[] = {
	{ "sm5011-rtc", 0 },
};

static struct platform_driver sm5011_rtc_driver = {
	.driver		= {
		.name	= "sm5011-rtc",
		.of_match_table = sm5011_rtc_of_match,			
		.owner	= THIS_MODULE,
	},
	.probe		= sm5011_rtc_probe,
	.remove		= sm5011_rtc_remove,
	.shutdown	= sm5011_rtc_shutdown,
	.id_table	= sm5011_rtc_id,
};

module_platform_driver(sm5011_rtc_driver);

/* Module information */
MODULE_DESCRIPTION("SM5011 RTC driver");
MODULE_LICENSE("GPL");

