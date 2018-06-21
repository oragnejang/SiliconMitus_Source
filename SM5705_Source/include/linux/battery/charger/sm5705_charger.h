/*
 * sm5705_charger.h
 * Samsung SM5705 Charger Header
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

#ifndef __SM5705_CHARGER_H
#define __SM5705_CHARGER_H __FILE__

#include <linux/mfd/core.h>
#include <linux/mfd/sm5705/sm5705.h>
#include <linux/regulator/machine.h>
#ifdef CONFIG_FLED_SM5705
#include <linux/leds/sm5705_fled.h>
#include <linux/leds/smfled.h>
#endif

enum {
	CHIP_ID = 0,
};

ssize_t sm5705_chg_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sm5705_chg_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

#define SM5705_CHARGER_ATTR(_name)				\
{							\
	.attr = {.name = #_name, .mode = 0664},	\
	.show = sm5705_chg_show_attrs,			\
	.store = sm5705_chg_store_attrs,			\
}

extern sec_battery_platform_data_t sec_battery_pdata;

#define SM5705_STATUS2_DONE         0x40
#define SM5705_STATUS2_DONE_SHIFT   6
#define SM5705_STATUS2_TOPOFF       0x20
#define SM5705_STATUS2_TOPOFF_SHIFT 5
#define SM5705_STATUS2_CHGON        0x80
#define SM5705_STATUS2_CHGON_SHIFT  3

/* SM5705_WCIN/VBUSIN_STATUS */
#define SM5705_WCINOVP_STATUS           0x40
#define SM5705_WCINOVP_STATUS_SHIFT     6
#define SM5705_WCINUVLO_STATUS          0x20
#define SM5705_WCINUVLO_STATUS_SHIFT    5
#define SM5705_WCIN_STATUS		        0x10
#define SM5705_WCIN_STATUS_SHIFT	    4
#define SM5705_VBUSOVP_STATUS           0x04
#define SM5705_VBUSOVP_STATUS_SHIFT     2
#define SM5705_VBUSUVLO_STATUS          0x02
#define SM5705_VBUSUVLO_STATUS_SHIFT    1
#define SM5705_VBUSIN_STATUS            0x01
#define SM5705_VBUSIN_STATUS_SHIFT      0

#define SM5705_CHGON_STATUS            0x08
#define SM5705_CHGON_STATUS_SHIFT      3
#define SM5705_NOBAT_STATUS            0x04
#define SM5705_NOBAT_STATUS_SHIFT      2

/* SM5705_INTMSK */
#define SM5705_VBUSPOKM_INTMSK      (1 << 0)
#define SM5705_VBUSUVLOM_INTMSK     (1 << 1)
#define SM5705_VBUSOVPM_INTMSK      (1 << 2)
#define SM5705_VBUSLIMITM_INTMSK    (1 << 3)
#define SM5705_WPCINPOKM_INTMSK     (1 << 4)
#define SM5705_WPCINUVLOM_INTMSK    (1 << 5)
#define SM5705_WPCINOVPM_INTMSK     (1 << 6)
#define SM5705_WPCINLIMITM_INTMSK   (1 << 7)

#define SM5705_AICLM_INTMSK         (1 << 0)
#define SM5705_BATOVPM_INTMSK       (1 << 1)
#define SM5705_NOBATM_INTMSK        (1 << 2)
#define SM5705_CHGONM_INTMSK        (1 << 3)
#define SM5705_Q4FULLONM_INTMSK     (1 << 4)
#define SM5705_TOPOFFM_INTMSK       (1 << 5)
#define SM5705_DONEM_INTMSK         (1 << 6)
#define SM5705_WDTMROFFM_INTMSK     (1 << 7)

#define SM5705_OPERATION_MODE				        0x07
#define SM5705_OPERATION_MODE_MASK                  0x07
#define SM5705_OPERATION_MODE_SHIFT			        0

#define SM5705_OPERATION_MODE_SUSPEND               0x00//000
#define SM5705_OPERATION_MODE_FACTORY               0x01//001
#define SM5705_OPERATION_MODE_WIR_OTG_CHGOFF        0x02//010
#define SM5705_OPERATION_MODE_WIR_OTG_CHGON         0x03//011
#define SM5705_OPERATION_MODE_CHARGING_OFF          0x04//100
#define SM5705_OPERATION_MODE_CHARGING_ON           0x05//101
#define SM5705_OPERATION_MODE_FLASH_BOOST_MODE      0x06//110
#define SM5705_OPERATION_MODE_USB_OTG_MODE          0x07//111

#define SM5705_CHGCNTL4_AICLTH		        0xC0
#define SM5705_CHGCNTL4_AICLTH_SHIFT	    6

#define SM5705_CHGCNTL4_AICLEN		        0x20
#define SM5705_CHGCNTL4_AICLEN_SHIFT	    5

#define REDUCE_CURRENT_STEP						50
#define MINIMUM_INPUT_CURRENT					300
#define SIOP_INPUT_LIMIT_CURRENT                1200
#define SIOP_CHARGING_LIMIT_CURRENT             1000
#define SIOP_HV_INPUT_LIMIT_CURRENT             1200
#define SIOP_HV_CHARGING_LIMIT_CURRENT          1000
#define SIOP_WIRELESS_INPUT_LIMIT_CURRENT       700
#define SIOP_WIRELESS_CHARGING_LIMIT_CURRENT    600
#define SIOP_HV_WIRELESS_INPUT_LIMIT_CURRENT	500
#define SIOP_HV_WIRELESS_CHARGING_LIMIT_CURRENT	1000
#define SLOW_CHARGING_CURRENT_STANDARD          400

#define SM5705_OTGCURRENT_MASK                  0xC

#define INPUT_CURRENT_TA		                1500
#define INPUT_CURRENT_WPC		                500

struct sm5705_charger_data {
	struct device *dev;
	struct i2c_client *i2c;
	sec_charger_platform_data_t	*pdata;
      
	struct power_supply	psy_chg;
	struct power_supply	psy_otg;

    /* for IRQ-service handling */
	int	irq_aicl;
	int	irq_vbus_pok;
	int	irq_wpcin_pok;
    int irq_topoff;

    /* for Charging enable GPIO-pin control */
	int     nchgen;

    /* for Workqueue & wake-lock, mutex process */
    struct mutex charger_mutex;

	struct workqueue_struct *wqueue;
	struct delayed_work	chgin_work;
	struct delayed_work	wpc_work;
	struct delayed_work	wc_afc_work;
	struct delayed_work	aicl_work;
	struct delayed_work	topoff_work;

    struct wake_lock wpc_wake_lock;
	struct wake_lock afc_wake_lock;
	struct wake_lock chgin_wake_lock;
	struct wake_lock check_slow_wake_lock;
	struct wake_lock aicl_wake_lock;

    /* for charging operation handling */
	unsigned int is_charging;
	unsigned int cable_type;
	unsigned int charging_current_max;
	unsigned int charging_current;
	unsigned int input_current_limit;
	int	irq_wpcin_state;
    bool slow_chg_on;
	int	status;
	int	siop_level;
	int	siop_event;
	bool wc_afc_detect;
	bool is_mdock;
	bool iin_current_detecting;
    bool topoff_pending;
};

#endif /* __SM5705_CHARGER_H */

