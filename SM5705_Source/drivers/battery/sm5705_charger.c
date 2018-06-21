/*
 *  /drivers/battery/sm5705_charger_1.c
 * 
 *  SM5705 Charger driver for SEC_BATTERY Flatform support
 *
 *  Copyright (C) 2015 Silicon Mitus,
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 
 */

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/power_supply.h>
#include <linux/mfd/sm5705/sm5705.h>
#ifdef CONFIG_USB_HOST_NOTIFY
#include <linux/usb_notify.h>
#endif
#if defined(CONFIG_VBUS_NOTIFIER)
#include <linux/vbus_notifier.h>
#endif
#include <linux/of_gpio.h>

#include <linux/battery/charger/sm5705_charger.h>
#include <linux/battery/charger/sm5705_charger_oper.h>

enum {
    SM5705_CHG_SRC_VBUS = 0x0,
    SM5705_CHG_SRC_WPC,
    SM5705_CHG_SRC_MAX,
};

/* Interrupt status Index & Offset */
enum {
    SM5705_INT_STATUS1 = 0x0,
    SM5705_INT_STATUS2,
    SM5705_INT_STATUS3,
    SM5705_INT_STATUS4,
    SM5705_INT_MAX,
};

enum {
    SM5705_INT_STATUS1_VBUSPOK          = 0x0,
    SM5705_INT_STATUS1_VBUSUVLO,
    SM5705_INT_STATUS1_VBUSOVP,
    SM5705_INT_STATUS1_VBUSLIMIT,
    SM5705_INT_STATUS1_WPCINPOK,
    SM5705_INT_STATUS1_WPCINUVLO,
    SM5705_INT_STATUS1_WPCINOVP,
    SM5705_INT_STATUS1_WPCINLIMIT,
};

enum {
    SM5705_INT_STATUS2_AICL             = 0x0,
    SM5705_INT_STATUS2_BATOVP,
    SM5705_INT_STATUS2_NOBAT,
    SM5705_INT_STATUS2_CHGON,
    SM5705_INT_STATUS2_Q4FULLON,
    SM5705_INT_STATUS2_TOPOFF,
    SM5705_INT_STATUS2_DONE,
    SM5705_INT_STATUS2_WDTMROFF,
};

enum {
    SM5705_INT_STATUS3_THEMREG          = 0x0,
    SM5705_INT_STATUS3_THEMSHDN,
    SM5705_INT_STATUS3_OTGFAIL,
    SM5705_INT_STATUS3_DISLIMIT,
    SM5705_INT_STATUS3_PRETMROFF,
    SM5705_INT_STATUS3_FASTTMROFF,
    SM5705_INT_STATUS3_LOWBATT,
    SM5705_INT_STATUS3_nEQ4,
};

enum {
    SM5705_INT_STATUS4_FLED1SHORT       = 0x0,
    SM5705_INT_STATUS4_FLED1OPEN,
    SM5705_INT_STATUS4_FLED2SHORT,
    SM5705_INT_STATUS4_FLED2OPEN,
    SM5705_INT_STATUS4_BOOSTPOK_NG,
    SM5705_INT_STATUS4_BOOSTPOL,
    SM5705_INT_STATUS4_ABSTMR1OFF,
    SM5705_INT_STATUS4_SBPS,

};

#define __n_is_cable_type_for_wireless(cable_type)    ((cable_type != POWER_SUPPLY_TYPE_WIRELESS) && \
                                                     (cable_type != POWER_SUPPLY_TYPE_HV_WIRELESS) && \
                                                     (cable_type != POWER_SUPPLY_TYPE_PMA_WIRELESS) && \
                                                     (cable_type != POWER_SUPPLY_TYPE_HV_WIRELESS_ETX))

/**
 *  SM5705 Charger device register control functions 
 */
static bool sm5705_CHG_get_INT_STATUS(struct sm5705_charger_data *charger, unsigned char index, unsigned char offset)
{
    unsigned char reg_val;
    int ret;
    
    ret = sm5705_read_reg(charger->i2c, SM5705_REG_STATUS1 + index, &reg_val);
    if (ret < 0) {
        dev_err(charger->dev, "%s: fail to I2C read REG:SM5705_REG_INT%d\n", __func__, 1 + index);
        return 0;
    }

    reg_val = (reg_val & (1 << offset)) >> offset;

    return reg_val;
}

static int sm5705_CHG_enable_AUTOSTOP(struct sm5705_charger_data *charger, bool enable)
{
    int ret;

    ret = sm5705_update_reg(charger->i2c, SM5705_REG_CHGCNTL4, (enable << 6), (1 << 6));
    if (ret < 0) {
        dev_err(charger->dev, "%s: fail to update REG:SM5705_REG_CHGCNTL4 in AUTOSTOP[6]\n", __func__);
        return ret;
    }

    return 0;
}

static inline unsigned char _calc_BATREG_offset_to_float_mV(unsigned short mV)
{
    unsigned char offset;

    if (mV < 4000) {
        offset = 0x0;     /* BATREG = 3.8V */
    } else if (mV < 4010) {
        offset = 0x1;     /* BATREG = 4.0V */
    } else if (mV < 4310) {
        offset = (((mV - 4010) / 10) + 2);    /* BATREG = 4.01 ~ 4.30 */
    } else {
        offset = 0x15;    /* default Offset : 4.2V */
    }

    return offset;
}

static int sm5705_CHG_set_BATREG(struct sm5705_charger_data *charger, unsigned short float_mV)
{
    unsigned char offset = _calc_BATREG_offset_to_float_mV(float_mV);
    int ret;

    dev_info(charger->dev, "%s: set BATREG voltage(%dmV - offset=0x%x)\n", __func__, float_mV, offset);

    ret = sm5705_update_reg(charger->i2c, SM5705_REG_CHGCNTL4, offset, 0x3F);
    if (ret < 0) {
        dev_err(charger->dev, "%s: fail to update REG:SM5705_REG_CHGCNTL4 in BATREG[5:0]\n", __func__);
        return ret;
    }

    return 0;
}

static inline unsigned short _calc_float_mV_to_BATREG_offset(unsigned char offset)
{
    return ((offset - 2) * 10) + 4010;
}

static unsigned short sm5705_CHG_get_BATREG(struct sm5705_charger_data *charger)
{
    unsigned char offset;
    int ret;

    ret = sm5705_read_reg(charger->i2c, SM5705_REG_CHGCNTL4, &offset);
    if (ret < 0) {
        dev_err(charger->dev, "%s: fail to read REG:SM5705_REG_CHGCNTL4\n", __func__);
        return 0;
    }

    return _calc_float_mV_to_BATREG_offset(offset & 0x3F);
}

static inline unsigned char _calc_TOPOFF_offset_to_topoff_mA(unsigned short mA)
{
    unsigned char offset;

    if (mA < 100) {
        offset = 0x0;
    } else if (mA < 480) {
        offset = (mA - 100) / 25;
    } else {
        offset = 0xF;
    }

    return offset;
}

static int sm5705_CHG_set_TOPOFF(struct sm5705_charger_data *charger, unsigned short topoff_mA)
{
    unsigned char offset = _calc_TOPOFF_offset_to_topoff_mA(topoff_mA);
    int ret;

    dev_info(charger->dev, "%s: set TOP-OFF current(%dmA - offset=0x%x)\n", __func__, topoff_mA, offset);

    ret = sm5705_update_reg(charger->i2c, SM5705_REG_CHGCNTL5, offset, 0xF);
    if (ret < 0) {
        dev_err(charger->dev, "%s: fail to update REG:SM5705_REG_CHGCNTL5 in TOPOFF[3:0]\n", __func__);
        return ret;
    }

    return 0;
}

static inline unsigned char _calc_AICL_threshold_offset_to_mV(unsigned short aiclth_mV)
{
    unsigned char offset;

    if (aiclth_mV < 4500) {
        offset = 0x0;
    } else if (aiclth_mV < 4900) {
        offset = (aiclth_mV - 4500) / 100;
    } else {
        offset = 0x3;
    }

    return offset;
}

static int sm5705_CHG_set_AICLTH(struct sm5705_charger_data *charger, unsigned short aiclth_mV)
{
    unsigned char offset = _calc_AICL_threshold_offset_to_mV(aiclth_mV);
    int ret;

    dev_info(charger->dev, "%s: set AICL threshold (%dmV - offset=0x%x)\n", __func__, aiclth_mV, offset);

    ret = sm5705_update_reg(charger->i2c, SM5705_REG_CHGCNTL7, (offset << 6), 0xC0);
    if (ret < 0) {
        dev_err(charger->dev, "%s: fail to update REG:SM5705_REG_CHGCNTL7 in AICLTH[7:6]\n", __func__);
        return ret;
    }

    return 0;
}

static int sm5705_CHG_set_OVPSEL(struct sm5705_charger_data *charger, bool enable)
{
    int ret;

    dev_info(charger->dev, "%s: set OVPSEL=%d\n", __func__, enable);

    ret = sm5705_update_reg(charger->i2c, SM5705_REG_CHGCNTL7, (enable << 2), (1 << 2));
    if (ret < 0) {
        dev_err(charger->dev, "%s: fail to update REG:SM5705_REG_CHGCNTL7 in OVPSEL[2]\n", __func__);
        return ret;
    }

    return 0;
}

static int sm5705_CHG_enable_AICL(struct sm5705_charger_data *charger, bool enable)
{
    int ret;

    ret = sm5705_update_reg(charger->i2c, SM5705_REG_CHGCNTL7, (enable << 5), (1 << 5));
    if (ret < 0) {
        dev_err(charger->dev, "%s: fail to update REG:SM5705_REG_CHGCNTL7 in AICLEN[5]\n", __func__);
        return ret;
    }

    return 0;
}

static int sm5705_CHG_enable_AUTOSET(struct sm5705_charger_data *charger, bool enable)
{
    int ret;

    ret = sm5705_update_reg(charger->i2c, SM5705_REG_CHGCNTL7, (enable << 1), (1 << 1));
    if (ret < 0) {
        dev_err(charger->dev, "%s: fail to update REG:SM5705_REG_CHGCNTL7 in AUTOSET[1]\n", __func__);
        return ret;
    }

    return 0;
}

static inline unsigned char _calc_FASTCHG_current_offset_to_mA(unsigned short mA)
{
    unsigned char offset;

    if (mA < 100) {
        offset = 0x12;
    } else {
        offset = ((mA - 100) / 50) & 0x3F;
    }

    return offset;
}

static int sm5705_CHG_set_FASTCHG(struct sm5705_charger_data *charger, unsigned char index, unsigned short FASTCHG_mA)
{
    unsigned char offset = _calc_FASTCHG_current_offset_to_mA(FASTCHG_mA);
    struct device *dev = charger->dev;

    dev_info(dev, "%s: FASTCHG src=%d, current=%dmA offset=0x%x\n", __func__, index, FASTCHG_mA, offset);

    if (index > SM5705_CHG_SRC_WPC) {
        return -EINVAL;
    }

    sm5705_write_reg(charger->i2c, SM5705_REG_CHGCNTL2 + index, offset);

    return 0;
}

static inline unsigned char _calc_INPUT_LIMIT_current_offset_to_mA(unsigned char index, unsigned short mA)
{
    unsigned char offset;

    if (mA < 100) {
        offset = 0x10;
    } else {
        if (index == SM5705_CHG_SRC_VBUS) {
            offset = ((mA - 100) / 25) & 0x7F;       /* max = 3275mA */
        } else {
            offset = ((mA - 100) / 25) & 0x3F;       /* max = 1650mA */
        }
    }

    return offset;
}

static int sm5705_CHG_set_INPUT_LIMIT(struct sm5705_charger_data *charger, unsigned char index, unsigned short LIMIT_mA)
{
    unsigned char offset = _calc_INPUT_LIMIT_current_offset_to_mA(index, LIMIT_mA);
    struct device *dev = charger->dev;

    dev_info(dev, "%s: set Input LIMIT src=%d, current=%dmA offset=0x%x\n", __func__, index, LIMIT_mA, offset);

    if (index > SM5705_CHG_SRC_WPC) {
        return -EINVAL;
    }

    sm5705_write_reg(charger->i2c, SM5705_REG_VBUSCNTL + index, offset);

    return 0;
}

static inline unsigned short _calc_INPUT_LIMIT_current_mA_to_offset(unsigned char index, unsigned char offset)
{
    return (offset * 25) + 100;
}

static unsigned short sm5705_CHG_get_INPUT_LIMIT(struct sm5705_charger_data *charger, unsigned char index)
{
    unsigned short LIMIT_mA;
    unsigned char offset;

    if (index > SM5705_CHG_SRC_WPC) {
        dev_err(charger->dev, "%s: invalid charger source index = %d\n", __func__, index);
        return 0;
    }

    sm5705_read_reg(charger->i2c, SM5705_REG_VBUSCNTL + index, &offset);

    LIMIT_mA = _calc_INPUT_LIMIT_current_mA_to_offset(index, offset);

    dev_info(charger->dev, "%s: get INPUT LIMIT src=%d, offset=0x%x, current=%dmA\n", __func__, index, offset, LIMIT_mA);

    return LIMIT_mA;
}

/* monitering REG_MAP */
static unsigned char sm5705_CHG_read_reg(struct sm5705_charger_data *charger, unsigned char reg)
{
    unsigned char reg_val = 0x0;

    sm5705_read_reg(charger->i2c, reg, &reg_val);

    return reg_val;
}

static void sm5705_CHG_print_REGMAP(struct sm5705_charger_data *charger)
{
    struct device *dev = charger->dev;
    int i;

    dev_info(dev, "sm5705-charger REGMAP info.\n");
    for (i=SM5705_REG_INTMSK1; i < SM5705_REG_FLED1CNTL1; ++i) {
        dev_info(dev, "REG[%02x] = 0x%02x\n", i, sm5705_CHG_read_reg(charger, i));
    }
    dev_info(dev, "REG[%02x] = 0x%02x\n", SM5705_REG_FLEDCNTL6, sm5705_CHG_read_reg(charger, SM5705_REG_FLEDCNTL6));
    dev_info(dev, "REG[%02x] = 0x%02x\n", SM5705_REG_SBPSCNTL, sm5705_CHG_read_reg(charger, SM5705_REG_SBPSCNTL));
}

/**
 *  SM5705 Charger Driver support functions 
 */

static int sm5705_get_input_current(struct sm5705_charger_data *charger)
{
	int get_current;

#if defined(CONFIG_WIRELESS_CHARGER_HIGH_VOLTAGE)
    get_current = sm5705_CHG_get_INPUT_LIMIT(charger, SM5705_CHG_SRC_VBUS);

    dev_info(charger->dev, "%s: HIGI_VOLTAGE_WIRELESS current=%d\n", __func__, get_current);
#else
    if (!(__n_is_cable_type_for_wireless(charger->cable_type))) {
        get_current = sm5705_CHG_get_INPUT_LIMIT(charger, SM5705_CHG_SRC_WPC);
    } else {
        get_current = sm5705_CHG_get_INPUT_LIMIT(charger, SM5705_CHG_SRC_VBUS);
    }
    dev_info(charger->dev, "%s: src_type=%d, current=%d\n", __func__, __n_is_cable_type_for_wireless(charger->cable_type), get_current);
#endif

	return get_current;
}

static void sm5705_enable_charging_on_switch(struct sm5705_charger_data *charger, bool enable)
{
    struct device *dev = charger->dev;

    gpio_direction_output(charger->pdata->chg_gpio_en, !(enable));
    charger->charging_current = (enable) ? charger->charging_current : 0;

    dev_info(dev, "%s: turn-%s Charging enable pin\n", __func__, enable ? "ON" : "OFF");
}

static int sm5705_set_charge_current(struct sm5705_charger_data *charger, unsigned short charge_current)
{
    struct device *dev = charger->dev;

	if (!charge_current) {
        dev_info(dev, "%s: skip process, charge_current = 0\n", __func__);
        return 0;
	}

    if (!(__n_is_cable_type_for_wireless(charger->cable_type))) {
        sm5705_CHG_set_FASTCHG(charger, SM5705_CHG_SRC_WPC, charge_current);
    } else {
        sm5705_CHG_set_FASTCHG(charger, SM5705_CHG_SRC_VBUS, charge_current);
    }

    return 0;
}

static int sm5705_set_input_current(struct sm5705_charger_data *charger, unsigned short input_current)
{
    struct device *dev = charger->dev;

	if (!input_current) {
        dev_info(dev, "%s: skip process, input_current = 0\n", __func__);
        return 0;
	}

    if (!(__n_is_cable_type_for_wireless(charger->cable_type))) {
        sm5705_CHG_set_INPUT_LIMIT(charger, SM5705_CHG_SRC_WPC, input_current);
    } else {
        sm5705_CHG_set_INPUT_LIMIT(charger, SM5705_CHG_SRC_VBUS, input_current);
    }

    return 0;
}

static void sm5705_charger_function_control(struct sm5705_charger_data *charger)
{
    struct device *dev = charger->dev;
	union power_supply_propval value;
	union power_supply_propval chg_mode;
	union power_supply_propval swelling_state;
	union power_supply_propval battery_status;

	psy_do_property("battery", get, POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION, value);
	psy_do_property("battery", get, POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW, battery_status);
	psy_do_property("battery", get, POWER_SUPPLY_PROP_HEALTH, value);

	if (charger->cable_type == POWER_SUPPLY_TYPE_BATTERY || charger->cable_type == POWER_SUPPLY_TYPE_OTG) {
		charger->is_charging = false;
		charger->wc_afc_detect = false;
		charger->slow_chg_on = false;
		charger->is_mdock = false;
		charger->charging_current = 0;

		if ((charger->status == POWER_SUPPLY_STATUS_DISCHARGING) || (value.intval == POWER_SUPPLY_HEALTH_UNSPEC_FAILURE) || (value.intval == POWER_SUPPLY_HEALTH_OVERHEATLIMIT)) {
			charger->charging_current_max = ((value.intval == POWER_SUPPLY_HEALTH_UNSPEC_FAILURE) || (value.intval == POWER_SUPPLY_HEALTH_OVERHEATLIMIT)) ? 0 : charger->pdata->charging_current[POWER_SUPPLY_TYPE_USB].input_current_limit;
		}
		charger->iin_current_detecting = false;

		if (charger->status == POWER_SUPPLY_STATUS_DISCHARGING) { /* cable type battery + discharging */
			dev_info(dev, "%s: Vsysmin set to 3.0V. cable(%d)\n", __func__, charger->cable_type);
		}
	} else {
		if (charger->cable_type == POWER_SUPPLY_TYPE_HMT_CONNECTED)
			charger->is_charging = false;
		else
			charger->is_charging = true;

		if (charger->cable_type == POWER_SUPPLY_TYPE_WIRELESS ||
			charger->cable_type == POWER_SUPPLY_TYPE_HV_WIRELESS ||
			charger->cable_type == POWER_SUPPLY_TYPE_PMA_WIRELESS) {
				dev_info(dev, "%s: Vsysmin set to 3.0V. cable(%d)\n", __func__, charger->cable_type);
		} else {
				dev_info(dev, "%s: Vsysmin set to 3.6V. cable(%d)\n", __func__, charger->cable_type);
		}
		charger->wc_afc_detect = false;
		charger->slow_chg_on = false;
		charger->charging_current_max = charger->pdata->charging_current[charger->cable_type].input_current_limit;
		charger->charging_current = charger->pdata->charging_current[charger->cable_type].fast_charging_current;
		if (charger->is_mdock) { /* if mdock was alread inserted, then check OTG, or NOTG state */
			if (charger->cable_type == POWER_SUPPLY_TYPE_SMART_NOTG) {
				charger->charging_current = charger->pdata->charging_current[POWER_SUPPLY_TYPE_MDOCK_TA].fast_charging_current;
				charger->charging_current_max = charger->pdata->charging_current[POWER_SUPPLY_TYPE_MDOCK_TA].input_current_limit;
			} else if (charger->cable_type == POWER_SUPPLY_TYPE_SMART_OTG) {
				charger->charging_current = charger->pdata->charging_current[POWER_SUPPLY_TYPE_MDOCK_TA].fast_charging_current - 500;
				charger->charging_current_max = charger->pdata->charging_current[POWER_SUPPLY_TYPE_MDOCK_TA].input_current_limit - 500;
			}
		} else { /*if mdock wasn't inserted, then check mdock state*/
			if (charger->cable_type == POWER_SUPPLY_TYPE_MDOCK_TA)
				charger->is_mdock = true;
		}

		if (charger->cable_type == POWER_SUPPLY_TYPE_WIRELESS || charger->cable_type == POWER_SUPPLY_TYPE_PMA_WIRELESS) {
			charger->wc_afc_detect = true;
			charger->charging_current_max = INPUT_CURRENT_WPC;
			queue_delayed_work(charger->wqueue, &charger->wc_afc_work, msecs_to_jiffies(4000));
			wake_lock_timeout(&charger->afc_wake_lock, HZ * 7);
		}
	}

	if (charger->pdata->full_check_type_2nd == SEC_BATTERY_FULLCHARGED_CHGPSY) {
		psy_do_property("battery", get, POWER_SUPPLY_PROP_CHARGE_NOW, chg_mode);
#if defined(CONFIG_BATTERY_SWELLING)
		psy_do_property("battery", get, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, swelling_state);
#else
		swelling_state.intval = 0;
#endif
		if (chg_mode.intval == SEC_BATTERY_CHARGING_2ND || swelling_state.intval) {
			//sm5705_set_charger_state(charger, 0);
			sm5705_CHG_set_TOPOFF(charger, charger->pdata->charging_current[charger->cable_type].full_check_current_2nd);
		} else {
			sm5705_CHG_set_TOPOFF(charger, charger->pdata->charging_current[charger->cable_type].full_check_current_1st);
		}
	} else {
		sm5705_CHG_set_TOPOFF(charger, charger->pdata->charging_current[charger->cable_type].full_check_current_1st);
	}

	dev_info(dev, "charging = %d, fc = %d, il = %d, t1 = %d, t2 = %d, cable = %d\n",
		charger->is_charging,
		charger->charging_current,
		charger->charging_current_max,
		charger->pdata->charging_current[charger->cable_type].full_check_current_1st,
		charger->pdata->charging_current[charger->cable_type].full_check_current_2nd,
		charger->cable_type);

	sm5705_CHG_print_REGMAP(charger);
}

static void sm5705_set_current(struct sm5705_charger_data *charger)
{
	int current_now = charger->charging_current,
		current_max = charger->charging_current_max;
	int usb_charging_current = charger->pdata->charging_current[
		POWER_SUPPLY_TYPE_USB].fast_charging_current;
	int siop_level = charger->iin_current_detecting ? 100 : charger->siop_level;
	union power_supply_propval value;

	dev_info(charger->dev, "%s: siop_level=%d(%d), wc_afc_detec=%d, current_max=%d, current_now=%d\n",
		__func__, siop_level, charger->siop_level, charger->wc_afc_detect, current_max, current_now);

	psy_do_property("battery", get, POWER_SUPPLY_PROP_CAPACITY, value);

	if (charger->siop_event == SIOP_EVENT_WPC_CALL_START) {
		if (value.intval >= charger->pdata->wireless_cc_cv)
			current_max = charger->pdata->siop_call_cv_current;
		else
			current_max = charger->pdata->siop_call_cc_current;
		current_now = charger->pdata->charging_current[POWER_SUPPLY_TYPE_WIRELESS].fast_charging_current;
	} else if (charger->is_charging) {
		/* decrease the charging current according to siop level */
		current_now = current_now * siop_level / 100;

		/* do forced set charging current */
		if (current_now > 0 && current_now < usb_charging_current)
			current_now = usb_charging_current;

		if (siop_level == 3) {
			/* side sync scenario : siop_level 3 */
			if (charger->cable_type == POWER_SUPPLY_TYPE_WIRELESS ||
				charger->cable_type == POWER_SUPPLY_TYPE_PMA_WIRELESS) {
				if (current_max > charger->pdata->siop_wireless_input_limit_current)
					current_max = charger->pdata->siop_wireless_input_limit_current;
				current_now = charger->pdata->siop_wireless_charging_limit_current;
			} else if (charger->cable_type == POWER_SUPPLY_TYPE_HV_WIRELESS) {
				if (current_max > charger->pdata->siop_hv_wireless_input_limit_current)
					current_max = charger->pdata->siop_hv_wireless_input_limit_current;
				current_now = charger->pdata->siop_hv_wireless_charging_limit_current;
			} else if (charger->cable_type == POWER_SUPPLY_TYPE_HV_MAINS ||
					   charger->cable_type == POWER_SUPPLY_TYPE_HV_ERR) {
				if (current_max > 450)
					current_max = 450;
				current_now = charger->pdata->siop_hv_charging_limit_current;
			} else {
				if (current_max > 800)
					current_max = 800;
				current_now = charger->pdata->charging_current[
						charger->cable_type].fast_charging_current;
				if (current_now > charger->pdata->siop_charging_limit_current)
					current_now = charger->pdata->siop_charging_limit_current;
			}
		} else if (siop_level < 100) {
			if (charger->cable_type == POWER_SUPPLY_TYPE_WIRELESS ||
				charger->cable_type == POWER_SUPPLY_TYPE_PMA_WIRELESS) {
				if(current_max > charger->pdata->siop_wireless_input_limit_current)
					current_max = charger->pdata->siop_wireless_input_limit_current;
				if (current_now > charger->pdata->siop_wireless_charging_limit_current)
					current_now = charger->pdata->siop_wireless_charging_limit_current;
			} else if (charger->cable_type == POWER_SUPPLY_TYPE_HV_WIRELESS) {
				if(current_max > charger->pdata->siop_hv_wireless_input_limit_current)
					current_max = charger->pdata->siop_hv_wireless_input_limit_current;
				if (current_now > charger->pdata->siop_hv_wireless_charging_limit_current)
					current_now = charger->pdata->siop_hv_wireless_charging_limit_current;
			} else if (charger->cable_type == POWER_SUPPLY_TYPE_HV_MAINS ||
					   charger->cable_type == POWER_SUPPLY_TYPE_HV_ERR){
				if (current_max > charger->pdata->siop_hv_input_limit_current)
					current_max = charger->pdata->siop_hv_input_limit_current;
				if (current_now > charger->pdata->siop_hv_charging_limit_current)
					current_now = charger->pdata->siop_hv_charging_limit_current;
			} else {
				if (current_max > charger->pdata->siop_input_limit_current)
					current_max = charger->pdata->siop_input_limit_current;
				if (current_now > charger->pdata->siop_charging_limit_current)
					current_now = charger->pdata->siop_charging_limit_current;
			}
		}
	}

	dev_info(charger->dev, "%s: siop_level=%d(%d), wc_afc_detec=%d, current_max=%d, current_now=%d\n",
		__func__, siop_level, charger->siop_level, charger->wc_afc_detect, current_max, current_now);

	sm5705_set_charge_current(charger, current_now);
	sm5705_set_input_current(charger, current_max);

	sm5705_CHG_print_REGMAP(charger);
}

/**
 *  SM5705 Power-supply class management functions 
 */
static inline void psy_chg_set_cable_online(struct sm5705_charger_data *charger, int value, int previous_cable_type)
{
    charger->cable_type = value;

    if (charger->cable_type == POWER_SUPPLY_TYPE_BATTERY) {
        dev_info(charger->dev, "%s: Type Battery\n", __func__);

        if (previous_cable_type == POWER_SUPPLY_TYPE_HV_PREPARE_MAINS || previous_cable_type == POWER_SUPPLY_TYPE_HV_MAINS) {
            dev_info(charger->dev, "%s: Program OVP threshold VBUS=6.2V activate\n", __func__);
            sm5705_CHG_set_OVPSEL(charger, 0);
        }

        if (previous_cable_type == POWER_SUPPLY_TYPE_POWER_SHARING) {
            sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_PWR_SHAR, false);
        } else if (previous_cable_type == POWER_SUPPLY_TYPE_WIRELESS) {
            sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_WPC, false);
        } else {
            sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_VBUS, false);
        }

        sm5705_enable_charging_on_switch(charger, 0);
    } else if (charger->cable_type == POWER_SUPPLY_TYPE_POWER_SHARING) {
        dev_info(charger->dev, "%s: Power SHARING mode\n", __func__);
        sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_PWR_SHAR,true);
    }else {
        dev_info(charger->dev, "%s: Set charging, Cable type = %d\n", __func__, charger->cable_type);

        if (charger->cable_type == POWER_SUPPLY_TYPE_HV_PREPARE_MAINS) {
            dev_info(charger->dev, "%s: Program OVP threshold VBUS=14V activate\n", __func__);
            sm5705_CHG_set_OVPSEL(charger, 1);
        }

        /* Enable charger */
        sm5705_charger_function_control(charger);
        sm5705_set_current(charger);        
        sm5705_enable_charging_on_switch(charger, 1);
    }        
}

static inline void psy_chg_set_current_avg(struct sm5705_charger_data *charger, int value)
{
#if defined(CONFIG_BATTERY_SWELLING)
    if (value > charger->pdata->charging_current[charger->cable_type].fast_charging_current) {
        return;
    }
#endif
    charger->charging_current = value;
    sm5705_set_charge_current(charger, value);
}

static inline void psy_chg_set_charge_full_design(struct sm5705_charger_data *charger, int value)
{
    if (value == SIOP_EVENT_WPC_CALL_START) {
        charger->siop_event = value;
    } else {
        charger->siop_event = 0;
        charger->siop_level = value;
    }

    sm5705_set_current(charger);
}

static inline void psy_chg_set_usb_hc(struct sm5705_charger_data *charger, int value)
{
    if (value) {
        /* set input/charging current for usb up to TA's current */
        charger->pdata->charging_current[POWER_SUPPLY_TYPE_USB].fast_charging_current = charger->pdata->charging_current[POWER_SUPPLY_TYPE_MAINS].fast_charging_current;
        charger->pdata->charging_current[POWER_SUPPLY_TYPE_USB].input_current_limit = charger->pdata->charging_current[POWER_SUPPLY_TYPE_MAINS].input_current_limit;
    } else {
        /* restore input/charging current for usb */
        charger->pdata->charging_current[POWER_SUPPLY_TYPE_USB].fast_charging_current = charger->pdata->charging_current[POWER_SUPPLY_TYPE_BATTERY].input_current_limit;
        charger->pdata->charging_current[POWER_SUPPLY_TYPE_USB].input_current_limit = charger->pdata->charging_current[POWER_SUPPLY_TYPE_BATTERY].input_current_limit;
    }
}

static inline void psy_chg_set_charge_otg_control(struct sm5705_charger_data *charger, int otg_en)
{
    union power_supply_propval value;

    psy_do_property("wireless", get, POWER_SUPPLY_PROP_ONLINE, value);

    if (otg_en && !value.intval) {
        sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_OTG, true);
        dev_info(charger->dev, "%s: OTG enable, cable(%d)\n", __func__, charger->cable_type);
#if defined(CONFIG_WIRELESS_CHARGER_HIGH_VOLTAGE)
        value.intval = 1;
        psy_do_property(charger->pdata->wireless_charger_name, set, POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL, value);
#endif
    } else {
        psy_do_property("battery", get, POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW, value);

        sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_OTG, false);
        dev_info(charger->dev, "%s: OTG disable, cable(%d)\n", __func__, charger->cable_type);

#if defined(CONFIG_WIRELESS_CHARGER_HIGH_VOLTAGE)
        value.intval = 0;
        psy_do_property(charger->pdata->wireless_charger_name, set, POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL, value);
#endif
    }
}

static inline void psy_chg_set_aicl_control(struct sm5705_charger_data *charger, int aicl_en)
{
    if (aicl_en) {
        sm5705_CHG_enable_AICL(charger, 1); 
        dev_info(charger->dev, "%s: CHGIN AICL ENABLE\n", __func__);
    } else {
        sm5705_CHG_enable_AICL(charger, 0);
        dev_info(charger->dev, "%s: CHGIN AICL DISABLE\n", __func__);
    }
}

static inline void psy_chg_set_afc_charger_mode(struct sm5705_charger_data *charger, int afc_mode)
{
    dev_info(charger->dev, "%s: [Monitoring] afc_charger_mode value = %d\n", __func__, afc_mode);
}

static int sm5705_chg_set_property(struct power_supply *psy, enum power_supply_property psp, const union power_supply_propval *val)
{
	struct sm5705_charger_data *charger = container_of(psy, struct sm5705_charger_data, psy_chg);
    struct device *dev = charger->dev;
    int previous_cable_type = charger->cable_type;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_STATUS - status=%d\n", __func__, val->intval);
        charger->status = val->intval;
		break;
    case POWER_SUPPLY_PROP_ONLINE:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_ONLINE - cable type [%d] to [%d]\n", __func__, previous_cable_type, val->intval);
        psy_chg_set_cable_online(charger, val->intval, previous_cable_type);
        break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CURRENT_MAX - current=%d\n", __func__, val->intval);
        sm5705_set_input_current(charger, val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CURRENT_AVG - current=%d\n", __func__, val->intval);
        psy_chg_set_current_avg(charger, val->intval);
		break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CURRENT_NOW - current=%d\n", __func__, val->intval);
		sm5705_set_charge_current(charger, val->intval);
		sm5705_set_input_current(charger, val->intval);
		break;
#if defined(CONFIG_AFC_CHARGER_MODE)
    case POWER_SUPPLY_PROP_AFC_CHARGER_MODE:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_AFC_CHARGER_MODE - value=%d\n", __func__, val->intval);
        psy_chg_set_afc_charger_mode(charger, val->intval);
		break;
#endif
#if defined(CONFIG_BATTERY_SWELLING) || defined(CONFIG_BATTERY_SWELLING_SELF_DISCHARGING)
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_VOLTAGE_MAX - MAX_mV=%d\n", __func__, val->intval);
        sm5705_CHG_set_BATREG(charger, val->intval);
		break;
#endif
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN - value=%d\n", __func__, val->intval);
        psy_chg_set_charge_full_design(charger, val->intval);
		break;
    case POWER_SUPPLY_PROP_USB_HC:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_USB_HC - value=%d\n", __func__, val->intval);
        psy_chg_set_usb_hc(charger, val->intval);
		break;
    case POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL - otg_en=%d\n", __func__, val->intval);
        psy_chg_set_charge_otg_control(charger, val->intval);
		break;
    case POWER_SUPPLY_PROP_CHARGE_NOW:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CHARGE_NOW - value=%d RETURN_ERR\n", __func__, val->intval);
		return -EINVAL;
    case POWER_SUPPLY_PROP_CHARGE_AICL_CONTROL:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CHARGE_AICL_CONTROL - aicl_en=%d\n", __func__, val->intval);
        psy_chg_set_aicl_control(charger, val->intval);
		break;
    case POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW - value=%d RETURN_ERR\n", __func__, val->intval);
		return -EINVAL;
    default:
        dev_err(dev, "%s: un-known Power-supply property type (psp=%d)\n", __func__, psp);
		return -EINVAL;
	}

	return 0;
}

static inline int psy_chg_get_charge_source_type(struct sm5705_charger_data *charger)
{
    int src_type;

    if (sm5705_CHG_get_INT_STATUS(charger, SM5705_INT_STATUS1, SM5705_INT_STATUS1_VBUSPOK)) {
        src_type = POWER_SUPPLY_TYPE_MAINS;
    } else if (sm5705_CHG_get_INT_STATUS(charger, SM5705_INT_STATUS1, SM5705_INT_STATUS1_WPCINPOK)) {
        src_type = POWER_SUPPLY_TYPE_WIRELESS;
    } else {
        src_type = POWER_SUPPLY_TYPE_BATTERY;
    }

    return src_type;
}

static inline bool _decide_charge_full_status(struct sm5705_charger_data *charger)
{
    if ((sm5705_CHG_get_INT_STATUS(charger, SM5705_INT_STATUS2, SM5705_INT_STATUS2_TOPOFF)) || (sm5705_CHG_get_INT_STATUS(charger, SM5705_INT_STATUS2, SM5705_INT_STATUS2_DONE))) {
        return charger->topoff_pending;
    }

    return false;
}

static inline int psy_chg_get_charger_state(struct sm5705_charger_data *charger)
{
    int status = POWER_SUPPLY_STATUS_UNKNOWN;
    int nCHG = 0;

    nCHG = gpio_get_value(charger->pdata->chg_gpio_en);

    if (_decide_charge_full_status(charger)) {
        status = POWER_SUPPLY_STATUS_FULL;
    }else if(sm5705_CHG_get_INT_STATUS(charger, SM5705_INT_STATUS2, SM5705_INT_STATUS2_CHGON)){
        status = POWER_SUPPLY_STATUS_CHARGING;
    }else {
        if (nCHG)
            status = POWER_SUPPLY_STATUS_DISCHARGING;
        else
            status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    }

    return status;
}

static inline int psy_chg_get_charge_type(struct sm5705_charger_data *charger)
{
    int charge_type;

    if (!charger->is_charging) {
        charge_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
    } else if (charger->slow_chg_on) {
        charge_type = POWER_SUPPLY_CHARGE_TYPE_SLOW;
    } else {
        charge_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
    }

    return charge_type;
}

static inline int psy_chg_get_charging_health(struct sm5705_charger_data *charger)
{
	int state;
    unsigned char reg_data;

	sm5705_read_reg(charger->i2c, SM5705_REG_STATUS1, &reg_data);

	dev_info(charger->dev, "%s: REG:SM5705_REG_STATUS1=0x%x\n", __func__, reg_data);
    
    if (charger->cable_type != POWER_SUPPLY_TYPE_WIRELESS) {
        if (reg_data & (1 << SM5705_INT_STATUS1_VBUSPOK)) {
        	state = POWER_SUPPLY_HEALTH_GOOD;
        } else if (reg_data & (1 << SM5705_INT_STATUS1_VBUSOVP)) {
        	state = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        } else if (reg_data & (1 << SM5705_INT_STATUS1_VBUSUVLO)) {
        	state = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
        } else {
        	state = POWER_SUPPLY_HEALTH_UNKNOWN;        
        }
    }else {
        if (reg_data & (1 << SM5705_INT_STATUS1_WPCINPOK)) {
        	state = POWER_SUPPLY_HEALTH_GOOD;
        } else if (reg_data & (1 << SM5705_INT_STATUS1_WPCINOVP)) {
        	state = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        } else if (reg_data & (1 << SM5705_INT_STATUS1_WPCINUVLO)) {
        	state = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
        } else {
        	state = POWER_SUPPLY_HEALTH_UNKNOWN;      
        }
    }

	return (int)state;
}

static int sm5705_chg_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	struct sm5705_charger_data *charger = container_of(psy, struct sm5705_charger_data, psy_chg);
    struct device *dev = charger->dev;

	switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = psy_chg_get_charge_source_type(charger);
        dev_info(dev, "%s: POWER_SUPPLY_PROP_ONLINE - src_type=%d\n", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = sm5705_CHG_get_INT_STATUS(charger, SM5705_INT_STATUS2, SM5705_INT_STATUS2_NOBAT);
        dev_info(dev, "%s: POWER_SUPPLY_PROP_PRESENT - NOBAT status=%d\n", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = psy_chg_get_charger_state(charger);
        dev_info(dev, "%s: POWER_SUPPLY_PROP_STATUS - charge state=%d\n", __func__, val->intval);
		break;
    case POWER_SUPPLY_PROP_CHARGE_TYPE:
        val->intval = psy_chg_get_charge_type(charger);
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CHARGE_TYPE - charge type=%d\n", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = psy_chg_get_charging_health(charger);
        dev_info(dev, "%s: POWER_SUPPLY_PROP_HEALTH - charge health=%d\n", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = charger->charging_current_max;
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CURRENT_MAX - current max=%d\n", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = sm5705_get_input_current(charger);
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CURRENT_AVG - current avg=%d\n", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = sm5705_get_input_current(charger);
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CURRENT_NOW - current now=%d\n", __func__, val->intval);
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = charger->siop_level;
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN - value=%d\n", __func__, val->intval);
		break;
#if defined(CONFIG_BATTERY_SWELLING) || defined(CONFIG_BATTERY_SWELLING_SELF_DISCHARGING)
    case POWER_SUPPLY_PROP_VOLTAGE_MAX:
        val->intval = sm5705_CHG_get_BATREG(charger);
        dev_info(dev, "%s: POWER_SUPPLY_PROP_VOLTAGE_MAX - voltage MAX=%d\n", __func__, val->intval);
		break;
#endif
#if defined(CONFIG_AFC_CHARGER_MODE)
	case POWER_SUPPLY_PROP_AFC_CHARGER_MODE:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_AFC_CHARGER_MODE - need to work\n", __func__);
        return -ENODATA;
#endif
    case POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL - need to work\n", __func__);
		return -ENODATA;
    case POWER_SUPPLY_PROP_USB_HC:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_USB_HC - need to work\n", __func__);
		return -ENODATA;
    case POWER_SUPPLY_PROP_CHARGE_NOW:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CHARGE_NOW - need to work\n", __func__);
        return -ENODATA;
        return -ENODATA;
    case POWER_SUPPLY_PROP_CHARGE_AICL_CONTROL:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CHARGE_AICL_CONTROL - need to work\n", __func__);
		return -ENODATA;
    case POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW:
        dev_info(dev, "%s: POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW - need to work\n", __func__);
		return -ENODATA;
    default:
        dev_err(dev, "%s: un-known Power-supply property type (psp=%d)\n", __func__, psp);
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property sm5705_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL,
	POWER_SUPPLY_PROP_USB_HC,
#if defined(CONFIG_BATTERY_SWELLING) || defined(CONFIG_BATTERY_SWELLING_SELF_DISCHARGING)
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
#endif
#if defined(CONFIG_AFC_CHARGER_MODE)
	POWER_SUPPLY_PROP_AFC_CHARGER_MODE,
#endif
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_AICL_CONTROL,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
	POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW,
};

static int sm5705_otg_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	struct sm5705_charger_data *charger = container_of(psy, struct sm5705_charger_data, psy_chg);

    dev_info(charger->dev, "%s: not-thing process\n", __func__);

	return 0;
}

static int sm5705_otg_set_property(struct power_supply *psy, enum power_supply_property psp, const union power_supply_propval *val)
{
	struct sm5705_charger_data *charger = container_of(psy, struct sm5705_charger_data, psy_chg);
	union power_supply_propval value;

	switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        dev_info(charger->dev, "%s: POWER_SUPPLY_PROP_ONLINE - %s\n", __func__, (val->intval) ? "ON" : "OFF");
		value.intval = val->intval;
		psy_do_property("sm5705-charger", set, POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL, value);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property sm5705_otg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

/**
 *  SM5705 Charger IRQ & Work-queue service management functions
 */
static void sm5705_topoff_work(struct work_struct *work)
{
	struct sm5705_charger_data *charger = container_of(work, struct sm5705_charger_data, aicl_work.work);
    bool topoff = 1;
    int i;

    dev_info(charger->dev, "%s: scadule work start.\n", __func__);

    for (i=0; i < 3; ++i) {
        topoff &= sm5705_CHG_get_INT_STATUS(charger, SM5705_INT_STATUS2, SM5705_INT_STATUS2_TOPOFF);
        msleep(150);

        dev_info(charger->dev, "%s: %dth Check TOP-OFF state=%d\n", __func__, i, topoff);
    }

    charger->topoff_pending = topoff;

    dev_info(charger->dev, "%s: scadule work done.\n", __func__);
}

static inline void _reduce_input_current(struct sm5705_charger_data *charger, int cur)
{
    unsigned short min_input_current = MINIMUM_INPUT_CURRENT;
    unsigned short vbus_limit_current = sm5705_CHG_get_INPUT_LIMIT(charger, SM5705_CHG_SRC_VBUS);
    
    if ((vbus_limit_current <= min_input_current) || (vbus_limit_current <= cur)) {
        return;
    }

    vbus_limit_current = ((vbus_limit_current - cur) < min_input_current) ? min_input_current : vbus_limit_current - cur;
    sm5705_CHG_set_INPUT_LIMIT(charger, SM5705_CHG_SRC_VBUS, vbus_limit_current);

    charger->charging_current_max = sm5705_get_input_current(charger);

    dev_info(charger->dev, "%s: vbus_limit_current=%d, charger->charging_current_max=%d\n", __func__, vbus_limit_current, charger->charging_current_max);
}

static inline void _check_slow_charging(struct sm5705_charger_data *charger, int input_current)
{
    /* under 400mA considered as slow charging concept for VZW */
    if (input_current <= SLOW_CHARGING_CURRENT_STANDARD && charger->cable_type != POWER_SUPPLY_TYPE_BATTERY) {
        union power_supply_propval value;

        charger->slow_chg_on = true;
        dev_info(charger->dev, "%s: slow charging on : input current(%dmA), cable type(%d)\n", __func__, input_current, charger->cable_type);

        value.intval = POWER_SUPPLY_CHARGE_TYPE_SLOW;
        psy_do_property("battery", set, POWER_SUPPLY_PROP_CHARGE_TYPE, value);
    }
}

static inline bool _check_aicl_state(struct sm5705_charger_data *charger)
{
    return sm5705_CHG_get_INT_STATUS(charger, SM5705_INT_STATUS2, SM5705_INT_STATUS2_AICL);
}

#define AICL_VALID_CHECK_DELAY_TIME     100
#define AICL_VALID_CHECK_NUM            3

static void sm5705_aicl_work(struct work_struct *work)
{
	struct sm5705_charger_data *charger = container_of(work, struct sm5705_charger_data, aicl_work.work);
    int prev_current_max, max_count, now_count;
    bool aicl_state = 1;
    int i;

	if ((!charger->is_charging) || (!__n_is_cable_type_for_wireless(charger->cable_type))) {
        dev_info(charger->dev, "%s: don't need AICL work\n", __func__);
        goto aicl_work_out;
    }

    mutex_lock(&charger->charger_mutex);

    dev_info(charger->dev, "%s: scadule work start.\n", __func__);

    /* AICL valid state check */
    for (i = 0; i < AICL_VALID_CHECK_NUM; ++i) {
        aicl_state &= _check_aicl_state(charger);
    }
    if (!aicl_state) {
        dev_info(charger->dev, "%s: AICL state un-stabled, skipped reduce process\n", __func__);
        goto aicl_work_mutex_out;
    }

    /* Reduce input limit current */
    max_count = charger->charging_current_max / REDUCE_CURRENT_STEP;
    prev_current_max = charger->charging_current_max;
    while (_check_aicl_state(charger) && (now_count++ < max_count)) {
        _reduce_input_current(charger, REDUCE_CURRENT_STEP);
        msleep(50);
    }
    if (prev_current_max > charger->charging_current_max) {
        dev_info(charger->dev, "%s: charging_current_max(%d --> %d)\n", __func__, prev_current_max, charger->charging_current_max);
        _check_slow_charging(charger, charger->charging_current_max);
    }

    dev_info(charger->dev, "%s: scadule work done.\n", __func__);

aicl_work_mutex_out:
    mutex_unlock(&charger->charger_mutex);
aicl_work_out:
	wake_unlock(&charger->aicl_wake_lock);    
}

static void wc_afc_detect_work(struct work_struct *work)
{
	struct sm5705_charger_data *charger = container_of(work, struct sm5705_charger_data, wc_afc_work.work);

    dev_info(charger->dev, "%s: scadule work start.\n", __func__);

	if ((charger->cable_type == POWER_SUPPLY_TYPE_WIRELESS || charger->cable_type == POWER_SUPPLY_TYPE_PMA_WIRELESS) && \
		charger->is_charging && charger->wc_afc_detect) {
		charger->wc_afc_detect = false;

		if (charger->charging_current_max >= INPUT_CURRENT_WPC) {
			charger->charging_current_max = charger->pdata->charging_current[POWER_SUPPLY_TYPE_WIRELESS].input_current_limit;
		}
		dev_info(charger->dev, "%s: current_max(%d)\n", __func__, charger->charging_current_max);

        sm5705_set_current(charger);
	}

    dev_info(charger->dev, "%s: scadule work doen.\n", __func__);
}

static void wpc_detect_work(struct work_struct *work)
{
	struct sm5705_charger_data *charger = container_of(work, struct sm5705_charger_data, wpc_work.work);
	union power_supply_propval value;
    int wpcin_state;

    dev_info(charger->dev, "%s: scadule work start.\n", __func__);

#if defined(CONFIG_WIRELESS_CHARGER_P9220)
	wpcin_state = !gpio_get_value(charger->pdata->irq_gpio);
#else
	wpcin_state = gpio_get_value(charger->pdata->wpc_det);
#endif
	dev_info(charger->dev, "%s wc_w_state = %d \n", __func__, wpcin_state);

	if ((charger->irq_wpcin_state == 0) && (wpcin_state == 1)) {
		value.intval = 1;
		psy_do_property("wireless", set, POWER_SUPPLY_PROP_ONLINE, value);
		value.intval = POWER_SUPPLY_TYPE_WIRELESS;
		psy_do_property(charger->pdata->wireless_charger_name, set, POWER_SUPPLY_PROP_ONLINE, value);

		dev_info(charger->dev, "%s: wpc activated, set V_INT as PN\n", __func__);
	} else if ((charger->irq_wpcin_state == 1) && (wpcin_state == 0)) {
        value.intval = 0;
        psy_do_property("wireless", set, POWER_SUPPLY_PROP_ONLINE, value);

        dev_info(charger->dev, "%s: wpc deactivated, set V_INT as PD\n", __func__);
	}

	dev_info(charger->dev, "%s: w(%d to %d)\n", __func__, charger->irq_wpcin_state, wpcin_state);

	charger->irq_wpcin_state = wpcin_state;

	wake_unlock(&charger->wpc_wake_lock);

    dev_info(charger->dev, "%s: scadule work done.\n", __func__);
}

static inline unsigned char _get_valid_vbus_status(struct sm5705_charger_data *charger)
{
	unsigned char vbusin, prev_vbusin = 0xff;
	int stable_count = 0;

    while (1) {
        sm5705_read_reg(charger->i2c, SM5705_REG_STATUS1, &vbusin);
        vbusin &= 0xF;

		if (prev_vbusin == vbusin) {
			stable_count++;
        } else {
            dev_info(charger->dev, "%s: VBUS status mismatch (0x%x / 0x%x), Reset stable count\n", __func__, vbusin, prev_vbusin);
			stable_count = 0;
        }

        if (stable_count == 10) {
            break;
        }

		prev_vbusin = vbusin;
		msleep(10);
    }

    return vbusin;
}

static inline int _check_vbus_power_supply_status(struct sm5705_charger_data *charger, unsigned char vbus_status, int prev_battery_health)
{
    int battery_health = prev_battery_health;

    if (charger->is_charging) {
        if ((vbus_status & SM5705_VBUSOVP_STATUS) && (prev_battery_health != POWER_SUPPLY_HEALTH_OVERVOLTAGE)) {
            dev_info(charger->dev, "%s: charger is over voltage\n", __func__);
            battery_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        } else if ((vbus_status & SM5705_VBUSUVLO_STATUS) && (prev_battery_health != POWER_SUPPLY_HEALTH_UNDERVOLTAGE) && \
                   __n_is_cable_type_for_wireless(charger->cable_type)) {
            dev_info(charger->dev, "%s: vBus is undervoltage\n", __func__);
            battery_health = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
        }
    } else {
        if ((prev_battery_health == POWER_SUPPLY_HEALTH_OVERVOLTAGE) && (vbus_status & SM5705_VBUSIN_STATUS)) {
            dev_info(charger->dev, "%s: overvoltage->normal\n", __func__);
            battery_health = POWER_SUPPLY_HEALTH_GOOD;
        } else if ((prev_battery_health == POWER_SUPPLY_HEALTH_UNDERVOLTAGE) && (vbus_status & SM5705_VBUSIN_STATUS)){
            dev_info(charger->dev, "%s: undervoltage->normal\n", __func__);
            battery_health = POWER_SUPPLY_HEALTH_GOOD;
        }
    }

    return battery_health;
}

static void sm5705_chgin_isr_work(struct work_struct *work)
{
	struct sm5705_charger_data *charger = container_of(work, struct sm5705_charger_data, chgin_work.work);
	union power_supply_propval value;
    unsigned char vbus_status;
	int prev_battery_health;

    dev_info(charger->dev, "%s: scadule work start.\n", __func__);

    wake_lock(&charger->chgin_wake_lock);

    vbus_status = _get_valid_vbus_status(charger);

    psy_do_property("battery", get,POWER_SUPPLY_PROP_HEALTH, value);
    prev_battery_health = value.intval;

    value.intval = _check_vbus_power_supply_status(charger, vbus_status, prev_battery_health);
    if (prev_battery_health != value.intval) {
        psy_do_property("battery", set, POWER_SUPPLY_PROP_HEALTH, value);
    }
    dev_info(charger->dev, "%s: battery change status [%d] -> [%d] (VBUS_REG:0x%x)\n", __func__, prev_battery_health, value.intval, vbus_status);

/*
    if (prev_battery_health == POWER_SUPPLY_HEALTH_UNDERVOLTAGE && value.intval == POWER_SUPPLY_HEALTH_GOOD) {
        sm5705_set_input_current(charger, charger->charging_current_max);
    }
*/
	wake_unlock(&charger->chgin_wake_lock);

    dev_info(charger->dev, "%s: scadule work done.\n", __func__);
}

static irqreturn_t sm5705_chg_vbus_in_isr(int irq, void *data)
{
	struct sm5705_charger_data *charger = data;

    dev_info(charger->dev, "%s: IRQ=%d\n", __func__, irq);

	queue_delayed_work(charger->wqueue, &charger->chgin_work, msecs_to_jiffies(0));

	return IRQ_HANDLED;
}

static irqreturn_t sm5705_chg_aicl_isr(int irq, void *data)
{
	struct sm5705_charger_data *charger = data;

    dev_info(charger->dev, "%s: IRQ=%d\n", __func__, irq);

	wake_lock(&charger->aicl_wake_lock);
	queue_delayed_work(charger->wqueue, &charger->aicl_work, msecs_to_jiffies(500));

	return IRQ_HANDLED;
}

static irqreturn_t sm5705_chg_topoff_isr(int irq, void *data)
{
	struct sm5705_charger_data *charger = data;

    dev_info(charger->dev, "%s: IRQ=%d\n", __func__, irq);

    charger->topoff_pending = 0;
	queue_delayed_work(charger->wqueue, &charger->topoff_work, msecs_to_jiffies(500));

	return IRQ_HANDLED;
}

static irqreturn_t sm5705_chg_wpcin_pok_isr(int irq, void *data)
{
	struct sm5705_charger_data *charger = data;
	unsigned long delay;

#ifdef CONFIG_SAMSUNG_BATTERY_FACTORY
	delay = msecs_to_jiffies(0);
#else
	if (charger->irq_wpcin_state)
		delay = msecs_to_jiffies(500);
	else
		delay = msecs_to_jiffies(0);
#endif
    dev_info(charger->dev, "%s: IRQ=%d delay = %ld\n", __func__, irq, delay);

	wake_lock(&charger->wpc_wake_lock);
	queue_delayed_work(charger->wqueue, &charger->wpc_work, delay);

    return IRQ_HANDLED;
}

/** 
 *  SM5705 Charger driver management functions   
 **/ 

#ifdef CONFIG_OF
static int _parse_sm5705_charger_node_propertys(struct device *dev, struct device_node *np, sec_charger_platform_data_t *pdata)
{
    int ret;

    ret = of_property_read_u32(np, "battery,chg_float_voltage", &pdata->chg_float_voltage);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,chg_float_voltage\n", __func__);
	}

    ret = of_property_read_u32(np, "battery,siop_call_cc_current", &pdata->siop_call_cc_current);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,siop_call_cc_current\n", __func__);
	}

    ret = of_property_read_u32(np, "battery,siop_call_cv_current", &pdata->siop_call_cv_current);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,siop_call_cv_current\n", __func__);
	}

    ret = of_property_read_u32(np, "battery,siop_input_limit_current", &pdata->siop_input_limit_current);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,siop_input_limit_current\n", __func__);
        pdata->siop_input_limit_current = SIOP_INPUT_LIMIT_CURRENT;
	}

    ret = of_property_read_u32(np, "battery,siop_charging_limit_current", &pdata->siop_charging_limit_current);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,siop_charging_limit_current\n", __func__);
        pdata->siop_charging_limit_current = SIOP_CHARGING_LIMIT_CURRENT;
	}

    ret = of_property_read_u32(np, "battery,siop_hv_input_limit_current", &pdata->siop_hv_input_limit_current);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,siop_hv_input_limit_current\n", __func__);
        pdata->siop_hv_input_limit_current = SIOP_HV_INPUT_LIMIT_CURRENT;
	}

    ret = of_property_read_u32(np, "battery,siop_hv_charging_limit_current", &pdata->siop_hv_charging_limit_current);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,siop_hv_charging_limit_current\n", __func__);
        pdata->siop_hv_charging_limit_current = SIOP_HV_CHARGING_LIMIT_CURRENT;
	}

    ret = of_property_read_u32(np, "battery,siop_wireless_input_limit_current", &pdata->siop_wireless_input_limit_current);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,siop_wireless_input_limit_current\n", __func__);
        pdata->siop_wireless_input_limit_current = SIOP_WIRELESS_INPUT_LIMIT_CURRENT;
	}

    ret = of_property_read_u32(np, "battery,siop_wireless_charging_limit_current", &pdata->siop_wireless_charging_limit_current);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,siop_wireless_charging_limit_current\n", __func__);
        pdata->siop_wireless_charging_limit_current = SIOP_WIRELESS_CHARGING_LIMIT_CURRENT;
	}

    ret = of_property_read_u32(np, "battery,siop_hv_wireless_input_limit_current", &pdata->siop_hv_wireless_input_limit_current);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,siop_hv_wireless_input_limit_current\n", __func__);
        pdata->siop_hv_wireless_input_limit_current = SIOP_HV_WIRELESS_INPUT_LIMIT_CURRENT;
	}

    ret = of_property_read_u32(np, "battery,siop_hv_wireless_charging_limit_current", &pdata->siop_hv_wireless_charging_limit_current);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,siop_hv_wireless_charging_limit_current\n", __func__);
        pdata->siop_hv_wireless_charging_limit_current = SIOP_HV_WIRELESS_CHARGING_LIMIT_CURRENT;
	}

    return 0;
}

static int _get_of_charging_current_table_max_size(struct device *dev, struct device_node *np)
{
	const unsigned int *propertys;
    int len;

    propertys = of_get_property(np, "battery,input_current_limit", &len);
	if (unlikely(!propertys)) {
        dev_err(dev, "%s: can't parsing dt:battery,input_current_limit\n", __func__);
    } else {
        dev_info(dev, "%s: dt:battery,input_current_limit length=%d\n", __func__, len);
    }

    return len / sizeof(unsigned int);
}

static int _parse_battery_node_propertys(struct device *dev, struct device_node *np, sec_charger_platform_data_t *pdata)
{
    int i, array_max_size, ret;

    ret = of_property_read_string(np,"battery,wirelss_charger_name", (char const **)&pdata->wireless_charger_name);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,wirelss_charger_name\n", __func__);
    }
    dev_info(dev, "%s: wireless charger name=%s\n", __func__, pdata->wireless_charger_name);

    pdata->chg_gpio_en = of_get_named_gpio(np, "battery,chgen_gpio", 0); //nCHGEN
	if (IS_ERR_VALUE(pdata->chg_gpio_en)) {
        dev_err(dev, "%s: can't parsing dt:battery,chgen_gpio\n", __func__);
        return -ENOENT;
    }
    dev_info(dev, "%s: battery charge enable pin = %d\n", __func__, pdata->chg_gpio_en);

    pdata->wpc_det = of_get_named_gpio(np, "battery,wpc_det", 0);
	if (IS_ERR_VALUE(pdata->wpc_det)) {
        dev_err(dev, "%s: can't parsing dt:battery,wpc_det\n", __func__);
        return -ENOENT;
    }
    dev_info(dev, "%s: WPC detect pin = %d\n", __func__, pdata->wpc_det);

#if defined(CONFIG_WIRELESS_CHARGER_HIGH_VOLTAGE)
    ret = of_property_read_u32(np, "battery,wpc_charging_limit_current", &pdata->wpc_charging_limit_current);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,wpc_charging_limit_current\n", __func__);
    }

    ret = of_property_read_u32(np, "battery,sleep_mode_limit_current", &pdata->sleep_mode_limit_current);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,sleep_mode_limit_current\n", __func__);
    }
#endif

    ret = of_property_read_u32(np, "battery,wireless_cc_cv", &pdata->wireless_cc_cv);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,wireless_cc_cv\n", __func__);
    }

    ret = of_property_read_u32(np, "battery,full_check_type_2nd", &pdata->full_check_type_2nd);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't parsing dt:battery,full_check_type_2nd\n", __func__);
    }

    array_max_size = _get_of_charging_current_table_max_size(dev, np);
    if (array_max_size == 0) {
        return -ENOENT;
    }
    dev_info(dev, "%s: charging current table max size = %d\n", __func__, array_max_size);

    pdata->charging_current = kzalloc(sizeof(sec_charging_current_t) * array_max_size, GFP_KERNEL);
	if (unlikely(!pdata->charging_current)) {
        dev_err(dev, "%s: fail to allocate memory for charging current table\n", __func__);
        return -ENOMEM;
    }

    for(i = 0; i < array_max_size; ++i) {
        of_property_read_u32_index(np, "battery,input_current_limit", i, &pdata->charging_current[i].input_current_limit);
        of_property_read_u32_index(np, "battery,fast_charging_current", i, &pdata->charging_current[i].fast_charging_current);
        of_property_read_u32_index(np, "battery,full_check_current_1st", i, &pdata->charging_current[i].full_check_current_1st);
        of_property_read_u32_index(np, "battery,full_check_current_2nd", i, &pdata->charging_current[i].full_check_current_2nd);
    }

    dev_info(dev, "%s: dt:battery node parse done.\n", __func__);

    return 0;
}


static int sm5705_charger_parse_dt(struct sm5705_charger_data *charger, struct sec_charger_platform_data *pdata)
{
	struct device_node *np;
    struct device *dev = charger->dev;
	int ret;

    np = of_find_node_by_name(NULL, "sm5705-charger");
	if (np == NULL) {
        dev_err(dev, "%s: fail to find dt_node:sm5705-charger\n", __func__);
        return -ENOENT;
	} else {
        ret = _parse_sm5705_charger_node_propertys(dev, np, pdata);
	}

	np = of_find_node_by_name(NULL, "battery");
	if (np == NULL) {
        dev_err(dev, "%s: fail to find dt_node:battery\n", __func__);
        return -ENOENT;
	} else {
        ret = _parse_battery_node_propertys(dev, np, pdata);
        if (IS_ERR_VALUE(ret)) {
            return ret;
        }
    }

    return ret;
}
#endif

static sec_charger_platform_data_t *_get_sm5705_charger_platform_data(struct platform_device *pdev, struct sm5705_charger_data *charger)
{
#ifdef CONFIG_OF
    sec_charger_platform_data_t *pdata;
    struct device *dev = &pdev->dev;
    int ret;

	pdata = kzalloc(sizeof(sec_charger_platform_data_t), GFP_KERNEL);
	if (!pdata) {
        dev_err(dev, "%s: fail to memory allocate for sec_charger_platform_data\n", __func__);
        return NULL;
	}

	ret = sm5705_charger_parse_dt(charger, pdata);
    if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: fail to parse sm5705 charger device tree (ret=%d)\n", __func__, ret);
        kfree(pdata);
        return NULL;
    }
#else
	struct sm5705_platform_data *sm5705_pdata = dev_get_platdata(sm5705->dev);
	struct sm5705_dev *sm5705 = dev_get_drvdata(pdev->dev.parent);
    struct device *dev = &pdev->dev;
    sec_charger_platform_data_t *pdata;

    pdata = sm5705_pdata->charger_data;
    if (!pdata) {
        dev_err(dev, "%s: fail to get sm5705 charger platform data\n", __func__);
        return NULL;
    }
#endif

    dev_info(dev, "%s: Get valid platform data done. (pdata=%p)\n", __func__, pdata);

    return pdata;
}

static int _init_sm5705_charger_info(struct platform_device *pdev, struct sm5705_dev *sm5705, struct sm5705_charger_data *charger)
{
	struct sm5705_platform_data *pdata = dev_get_platdata(sm5705->dev);
    struct device *dev = charger->dev;
    int ret;
	mutex_init(&charger->charger_mutex);

    if (pdata == NULL) {
        dev_err(dev, "%s: can't get sm5705_platform_data\n", __func__);
        return -EINVAL;
    }

    dev_info(dev, "%s: init process start..\n", __func__);

    /* setup default charger configuration parameter & flagment */
	charger->slow_chg_on = false;
	charger->wc_afc_detect = false;
	charger->is_mdock = false;
	charger->siop_level = 100;
	charger->siop_event = 0;
	charger->charging_current_max = 500;
	charger->iin_current_detecting = false;
    charger->topoff_pending = false;

    /* Request GPIO pin - CHG_IN */
	if (charger->pdata->chg_gpio_en) {
		ret = gpio_request(charger->pdata->chg_gpio_en, "sm5705_nCHGEN");
		if (ret) {
			dev_err(dev, "%s: fail to request GPIO %u\n", __func__, charger->pdata->chg_gpio_en);
            return ret;
		}
	}

    /* initialize delayed workqueue */
	charger->wqueue = create_singlethread_workqueue(dev_name(dev));
	if (!charger->wqueue) {
		dev_err(dev, "%s: fail to Create Workqueue\n", __func__);
        return -ENOMEM;
	}

	INIT_DELAYED_WORK(&charger->chgin_work, sm5705_chgin_isr_work);
	INIT_DELAYED_WORK(&charger->wpc_work, wpc_detect_work);
	INIT_DELAYED_WORK(&charger->wc_afc_work, wc_afc_detect_work);
	INIT_DELAYED_WORK(&charger->aicl_work, sm5705_aicl_work);
	INIT_DELAYED_WORK(&charger->topoff_work, sm5705_topoff_work);

    wake_lock_init(&charger->chgin_wake_lock, WAKE_LOCK_SUSPEND, "charger->chgin");
	wake_lock_init(&charger->wpc_wake_lock, WAKE_LOCK_SUSPEND, "charger-wpc");
	wake_lock_init(&charger->afc_wake_lock, WAKE_LOCK_SUSPEND, "charger-afc");
	wake_lock_init(&charger->check_slow_wake_lock, WAKE_LOCK_SUSPEND, "charger-check-slow");
	wake_lock_init(&charger->aicl_wake_lock, WAKE_LOCK_SUSPEND, "charger-aicl");

    /* Get IRQ service routine number */
	charger->irq_wpcin_pok = pdata->irq_base + SM5705_WPCINPOK_IRQ;
	charger->irq_vbus_pok = pdata->irq_base + SM5705_VBUSPOK_IRQ;
	charger->irq_aicl = pdata->irq_base + SM5705_AICL_IRQ;
    charger->irq_topoff = pdata->irq_base + SM5705_TOPOFF_IRQ;

    dev_info(dev, "%s: init process done..\n", __func__);

    return 0;
}

static void sm5705_charger_initialize(struct sm5705_charger_data *charger)
{
    struct device *dev = charger->dev;

    dev_info(dev, "%s: charger initial hardware condition process start..\n", __func__);

    sm5705_CHG_enable_AUTOSTOP(charger, 0);
    sm5705_CHG_set_BATREG(charger, charger->pdata->chg_float_voltage);

    sm5705_CHG_set_TOPOFF(charger, charger->pdata->charging_current[charger->cable_type].full_check_current_1st);

    sm5705_CHG_set_AICLTH(charger, 4500);
    sm5705_CHG_enable_AICL(charger, 1);

    sm5705_CHG_enable_AUTOSET(charger, 1);

    sm5705_CHG_print_REGMAP(charger);

    dev_info(dev, "%s: charger initial hardware condition process done.\n", __func__);
}

static int __devinit sm5705_charger_probe(struct platform_device *pdev)
{
	struct sm5705_dev *sm5705 = dev_get_drvdata(pdev->dev.parent);
	struct sm5705_platform_data *pdata = dev_get_platdata(sm5705->dev);
	struct sm5705_charger_data *charger;
    struct device *dev = &pdev->dev;
	int ret = 0;

	dev_info(dev, "%s: Sm5705 Charger Driver Probing start\n", __func__);

	charger = kzalloc(sizeof(struct sm5705_charger_data), GFP_KERNEL);
	if (!charger) {
        dev_err(dev, "%s: fail to memory allocate for sm5705 charger handler\n", __func__);
		return -ENOMEM;
    }

	charger->dev = &pdev->dev;
	charger->i2c = sm5705->i2c;
	charger->pdata = _get_sm5705_charger_platform_data(pdev, charger);
    if (charger->pdata == NULL) {
        dev_err(dev, "%s: fail to get charger platform data\n", __func__);
        return -ENOENT;
    }

    ret = _init_sm5705_charger_info(pdev, sm5705, charger);
	if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "%s: can't initailize sm5705 charger", __func__);
        goto err_free;
    }
	platform_set_drvdata(pdev, charger);

	sm5705_charger_initialize(charger);

	charger->psy_chg.name = "sm5705-charger";
	charger->psy_chg.type = POWER_SUPPLY_TYPE_UNKNOWN;
	charger->psy_chg.get_property = sm5705_chg_get_property;
	charger->psy_chg.set_property = sm5705_chg_set_property;
	charger->psy_chg.properties	= sm5705_charger_props;
	charger->psy_chg.num_properties	= ARRAY_SIZE(sm5705_charger_props);
	charger_chip_name = charger->psy_chg.name;
	ret = power_supply_register(&pdev->dev, &charger->psy_chg);
	if (ret) {
		dev_err(dev, "%s: fail to register psy_chg\n", __func__);
		goto err_power_supply_register;
	}
        
	charger->psy_otg.name = "otg";
	charger->psy_otg.type = POWER_SUPPLY_TYPE_OTG;
	charger->psy_otg.get_property = sm5705_otg_get_property;
	charger->psy_otg.set_property = sm5705_otg_set_property;
	charger->psy_otg.properties	 = sm5705_otg_props;
	charger->psy_otg.num_properties	= ARRAY_SIZE(sm5705_otg_props);
	ret = power_supply_register(&pdev->dev, &charger->psy_otg);
	if (ret) {
		dev_err(dev, "%s: fail to register otg_chg\n", __func__);
		goto err_power_supply_register_otg;
	}

    /* Operation Mode Initialize */
    sm5705_charger_oper_table_init(charger->i2c);

    /* Request IRQ */
	ret = request_threaded_irq(charger->irq_wpcin_pok, NULL, sm5705_chg_wpcin_pok_isr, IRQF_TRIGGER_FALLING, "wpc-int", charger);
	if (ret) {
		dev_err(dev, "%s: Failed to Request IRQ\n", __func__);
		goto err_wc_irq;
	}

	ret = request_threaded_irq(charger->irq_aicl, NULL, sm5705_chg_aicl_isr, 0, "aicl-irq", charger);
	if (ret < 0) {
		dev_err(dev, "%s: fail to request aicl IRQ: %d: %d\n", __func__, charger->irq_aicl, ret);
	}

	ret = request_threaded_irq(charger->irq_vbus_pok, NULL, sm5705_chg_vbus_in_isr, 0, "chgin-irq", charger);
	if (ret < 0) {
		dev_err(dev, "%s: fail to request chgin IRQ: %d: %d\n", __func__, charger->irq_vbus_pok, ret);
	}

	ret = request_threaded_irq(charger->irq_topoff, NULL, sm5705_chg_topoff_isr, 0, "topoff-irq", charger);
	if (ret < 0) {
		dev_err(dev, "%s: fail to request topoff IRQ: %d: %d\n", __func__, charger->irq_topoff, ret);
	}

	dev_info(dev, "%s: SM5705 Charger Driver Loaded Done\n", __func__);

	return 0;

err_wc_irq:
	free_irq(charger->pdata->chg_irq, NULL);
	power_supply_unregister(&charger->psy_otg);
err_power_supply_register_otg:
	power_supply_unregister(&charger->psy_chg);
err_power_supply_register:
	destroy_workqueue(charger->wqueue);
#ifdef CONFIG_OF
	kfree(pdata->charger_data);
#endif
	mutex_destroy(&charger->charger_mutex);
err_free:
	kfree(charger);

	return ret;
}

static int __devexit sm5705_charger_remove(struct platform_device *pdev)
{
	struct sm5705_charger_data *charger = platform_get_drvdata(pdev);

	destroy_workqueue(charger->wqueue);
	free_irq(charger->irq_wpcin_pok, NULL);
	free_irq(charger->pdata->chg_irq, NULL);
	power_supply_unregister(&charger->psy_chg);
	mutex_destroy(&charger->charger_mutex);
	kfree(charger);

	return 0;
}

static void sm5705_charger_shutdown(struct device *dev)
{
    dev_info(dev, "%s: call shutdown\n", __func__);
}

#if defined CONFIG_PM
static int sm5705_charger_suspend(struct device *dev)
{
    dev_info(dev, "%s: call suspend\n", __func__);
	return 0;
}

static int sm5705_charger_resume(struct device *dev)
{
    dev_info(dev, "%s: call resume\n", __func__);
	return 0;
}
#else
#define sm5705_charger_suspend NULL
#define sm5705_charger_resume NULL
#endif

static SIMPLE_DEV_PM_OPS(sm5705_charger_pm_ops, sm5705_charger_suspend, sm5705_charger_resume);
static struct platform_driver sm5705_charger_driver = {
	.driver = {
		.name = "sm5705-charger",
		.owner = THIS_MODULE,
		.pm = &sm5705_charger_pm_ops,
		.shutdown = sm5705_charger_shutdown,
	},
	.probe = sm5705_charger_probe,
	.remove = __devexit_p(sm5705_charger_remove),
};

static int __init sm5705_charger_init(void)
{
	return platform_driver_register(&sm5705_charger_driver);
}

static void __exit sm5705_charger_exit(void)
{
	platform_driver_unregister(&sm5705_charger_driver);
}


module_init(sm5705_charger_init);
module_exit(sm5705_charger_exit);

MODULE_DESCRIPTION("SM5705 Charger Driver");
MODULE_LICENSE("GPL v2");

