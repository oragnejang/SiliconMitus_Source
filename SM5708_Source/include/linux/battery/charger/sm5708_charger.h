/*
 * sm5708_charger.h
 * Samsung SM5708 Charger Header
 *
 * Copyright (C) 2012 Samsung Electronics, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SM5708_CHARGER_H
#define __SM5708_CHARGER_H __FILE__

#include <linux/mfd/core.h>
#include <linux/mfd/sm5708/sm5708.h>
#include <linux/regulator/machine.h>
#include <linux/battery/sec_charging_common.h>

/* CONFIG: Kernel Feature & Target System configuration */


#define SM5708_SUPPORT_OTG_CONTROL        /*- New A series dosen't support, It's MUST be disabled */

enum {
	CHIP_ID = 0,
};
ssize_t sm5708_chg_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf);
ssize_t sm5708_chg_store_attrs(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count);

#define SM5708_CHARGER_ATTR(_name)				\
{							                    \
	.attr = {.name = #_name, .mode = 0664},	    \
	.show = sm5708_chg_show_attrs,			    \
	.store = sm5708_chg_store_attrs,			\
}

enum {
	SM5708_MANUAL_RESET_TIME_7s = 0x1,
	SM5708_MANUAL_RESET_TIME_8s = 0x2,
	SM5708_MANUAL_RESET_TIME_9s = 0x3,
};

enum {
	SM5708_WATCHDOG_RESET_TIME_30s  = 0x0,
	SM5708_WATCHDOG_RESET_TIME_60s  = 0x1,
	SM5708_WATCHDOG_RESET_TIME_90s  = 0x2,
	SM5708_WATCHDOG_RESET_TIME_120s = 0x3,
};

enum {
	SM5708_TOPOFF_TIMER_10m         = 0x0,
	SM5708_TOPOFF_TIMER_20m         = 0x1,
	SM5708_TOPOFF_TIMER_30m         = 0x2,
	SM5708_TOPOFF_TIMER_45m         = 0x3,
};

enum {
	SM5708_BUCK_BOOST_FREQ_3MHz     = 0x0,
	SM5708_BUCK_BOOST_FREQ_2_4MHz   = 0x1,
	SM5708_BUCK_BOOST_FREQ_1_5MHz   = 0x2,
	SM5708_BUCK_BOOST_FREQ_1_8MHz   = 0x3,
};

/* for VZW slow_late_charging support */
#define SLOW_CHARGING_CURRENT_STANDARD		400
#define REDUCE_CURRENT_STEP					100
#define MINIMUM_INPUT_CURRENT				300
#define AICL_VALID_CHECK_DELAY_TIME          10

#define INPUT_CURRENT_TA		                1000


struct sm5708_charger_data {
	struct device *dev;
	struct i2c_client *i2c;
	struct sec_charger_platform_data *pdata;

	struct power_supply	psy_chg;
	struct power_supply	psy_otg;
	int status;

	/* for IRQ-service handling */
	int irq_aicl;
	int irq_vbus_pok;
	int irq_otgfail;
	int irq_done;

	/* for Workqueue & wake-lock, mutex process */
	struct mutex charger_mutex;

	struct workqueue_struct *wqueue;
	struct delayed_work aicl_work;
	struct wake_lock check_slow_wake_lock;
	struct wake_lock aicl_wake_lock;

	/* for charging operation handling */
	int charge_mode;
	unsigned int is_charging;
	unsigned int cable_type;
	unsigned int charging_current_max;
	unsigned int charging_current;
	unsigned int topoff_current;
	bool topoff_pending;

	bool slow_late_chg_mode;


	bool is_mdock;
};


#endif /* __SM5708_CHARGER_H */

