/*  sm5010-rtc.h
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_SM5010_RTC_H
#define __LINUX_MFD_SM5010_RTC_H

/*
 * struct sm5010_rtc_info - sm5010's rtc device
 * @ dev: rtc device of the chip
 * @ iodev: pointer to master device
 * @ rtc1_dev : pointer to rtc driver for RTC and ALM1
 * @ rtc2_dev : pointer to rtc driver for and ALM2 
 * @ i2c : i2c client private data for sm5010
 * @ lock : mutex for sm5010's rtc
 * @ irq_base : base IRQ number for sm5010, required for IRQs
 * @ alarm1_enabled : check whether alarm1 enabled or disabled
 * @ alarm1_check : check whether alarm1 used or not
 * @ alarm2_enabled : check whether alarm1 enabled or disabled
 * @ alarm2_check : check whether alarm1 used or not 
 * @ use_irq : check whether irq used or not in driver
 * @rtc_24hr_mode : enable 24 hour mode for RTC and ALM1/2
 */
struct sm5010_rtc_info {
	struct device			*dev;
	struct sm5010_pmic_dev	*iodev;
	struct rtc_device		*rtc1_dev; /* RTC & ALM1 */
	struct rtc_device		*rtc2_dev; /* ALM2 */	
	struct mutex			lock;
	int						irq_base;
	bool					alarm1_enabled;
	bool					alarm1_check;
	bool					alarm2_enabled;	
	bool					alarm2_check;	
	int 					use_irq;
	int 					rtc_24hr_mode;
};

/* RTC Counter Register offsets */
enum SM5010_RTCALM_OP {
	SM5010_RTC_ALARM1_PMIC,
	SM5010_RTC_ALARM1_EN,
	SM5010_RTC_ALARM2_PMIC,
	SM5010_RTC_ALARM2_EN,	
};  

/* RTC Counter Register offsets */
enum {
	RTC_SEC = 0,
	RTC_MIN,
	RTC_HOUR,
	RTC_WEEKDAY,
	RTC_DATE,
	RTC_MONTH,
	RTC_YEAR,
	NR_RTC_CNT_REGS,
};

/* CNTL1 */
#define SM5010_EN32KOUT_SHIFT		3
#define SM5010_EN32KOUT_MASK		(1 << SM5010_EN32KOUT_SHIFT)

/* RTC HOUR Register */
#define HOUR_PM_SHIFT			5
#define HOUR_PM_MASK			(1 << HOUR_PM_SHIFT)

/* RTC 12/24-mode Register */
#define MODE12_SHIFT			6
#define MODE12_MASK				(1 << MODE12_SHIFT)

/* RTC Alarm1 */
#define ALARM1_PMIC_SHIFT		7
#define ALARM1_PMIC_MASK		(1 << ALARM1_PMIC_SHIFT)
#define ALARM1_ENABLE_SHIFT		6
#define ALARM1_ENABLE_MASK		(1 << ALARM1_ENABLE_SHIFT)
/* RTC Alarm2 */
#define ALARM2_PMIC_SHIFT		7
#define ALARM2_PMIC_MASK		(1 << ALARM2_PMIC_SHIFT)
#define ALARM2_ENABLE_SHIFT		6
#define ALARM2_ENABLE_MASK		(1 << ALARM2_ENABLE_SHIFT)

#define SM5010_ALARM_DIS	0
#define SM5010_ALARM_EN		1

#define SM5010_ALARM_DIS	0
#define SM5010_ALARM_EN		1

#endif /*  __LINUX_MFD_SM5010_RTC_H */
