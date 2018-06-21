/*
 * sm5713-charger.c - SM5713 Charger device driver
 *
 * Copyright (C) 2017 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/power_supply.h>
#include "include/sec_charging_common.h"
#include "include/charger/sm5713_charger.h"

#define HEALTH_DEBOUNCE_CNT     1

static struct device_attribute sm5713_charger_attrs[] = {
	SM5713_CHARGER_ATTR(chip_id),
};

static enum power_supply_property sm5713_charger_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_CHARGING_ENABLED,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_CHARGE_TYPE,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_CURRENT_MAX,
    POWER_SUPPLY_PROP_CURRENT_AVG,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CURRENT_FULL,
#if defined(CONFIG_BATTERY_SWELLING)
    POWER_SUPPLY_PROP_VOLTAGE_MAX,
#endif
    POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL,
    POWER_SUPPLY_PROP_USB_HC,
    POWER_SUPPLY_PROP_ENERGY_NOW,
#if defined(CONFIG_AFC_CHARGER_MODE)
    POWER_SUPPLY_PROP_AFC_CHARGER_MODE,
#endif
};

static enum power_supply_property sm5713_otg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static void chg_set_aicl(struct sm5713_charger_data *charger, bool enable, u8 aicl)
{
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_CHGCNTL6, (aicl << 6), (0x3 << 6));
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_CNTL1, (enable << 6), (0x1 << 6));
}

static void chg_set_dischg_limit(struct sm5713_charger_data *charger, u8 dischg)
{
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_CHGCNTL6, (dischg << 2), (0x3 << 2));
}

static void chg_set_batreg(struct sm5713_charger_data *charger, u16 float_voltage)
{
    u8 offset;

	if (float_voltage < 4000) {
		offset = 0x0;     /* BATREG = 3.8V */
	} else if (float_voltage < 4010) {
		offset = 0x1;     /* BATREG = 4.0V */
	} else if (float_voltage < 4630) {
		offset = (((float_voltage - 4010) / 10) + 2);    /* BATREG = 4.01 ~ 4.62 in 0.01V steps */
	} else {
		dev_err(charger->dev, "%s: can't support BATREG at over voltage 4.62V (mV=%d)\n", __func__, float_voltage);
		offset = 0x15;    /* default Offset : 4.2V */
	}

    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_CHGCNTL4, ((offset & 0x3F) << 0), (0x3F << 0));
}

static void chg_set_iq3limit(struct sm5713_charger_data *charger, u8 q3limit)
{
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_BSTCNTL1, (q3limit << 4), (0x3 << 4));
}

static void chg_set_wdt_timer(struct sm5713_charger_data *charger, u8 wdt_timer)
{
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_WDTCNTL, (wdt_timer << 1), (0x3 << 1));
}

static void chg_set_wdt_enable(struct sm5713_charger_data *charger, bool enable)
{
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_WDTCNTL, (enable << 0), (0x1 << 0));
}

static void chg_set_wdt_tmr_reset(struct sm5713_charger_data *charger)
{
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_WDTCNTL, (0x1 << 3), (0x1 << 3));
}

static void chg_set_enq4fet(struct sm5713_charger_data *charger, bool enable)
{
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_CNTL1, (enable << 3), (0x1 << 3));
}

static void chg_set_input_current_limit(struct sm5713_charger_data *charger, int mA)
{
    u8 offset;

	mutex_lock(&charger->charger_mutex);

	if (mA < 100) {
		offset = 0x00;
	} else {
		offset = ((mA - 100) / 25) & 0x7F;
	}
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_VBUSCNTL, (offset << 0), (0x7F << 0));
    
	mutex_unlock(&charger->charger_mutex);
}

static void chg_set_charging_current(struct sm5713_charger_data *charger, int mA)
{
    u8 offset;

	if (mA < 100) {
		offset = 0x00;
	} else if (mA > 3500) {
        offset = 0x44;
    } else {
		offset = ((mA - 100) / 50) & 0x7F;
	}
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_CHGCNTL2, (offset << 0), (0x7F << 0));
}

static void chg_set_topoff_current(struct sm5713_charger_data *charger, int mA)
{
    u8 offset;

	if (mA < 100) {
		offset = 0x0;               /* Topoff = 100mA */
	} else if (mA < 600) {
		offset = (mA - 100) / 25;   /* Topoff = 125mA ~ 575mA in 25mA steps */
	} else {
		offset = 0x14;              /* Topoff = 600mA */
	}
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_CHGCNTL5, (offset << 0), (0x1F << 0));
}

static void chg_set_topoff_timer(struct sm5713_charger_data *charger, u8 tmr_offset)
{
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_CHGCNTL7, (tmr_offset << 3), (0x3 << 3));

}

static void chg_set_autostop(struct sm5713_charger_data *charger, bool enable)
{
    sm5713_update_reg(charger->i2c, SM5713_CHG_REG_CHGCNTL4, (enable << 6), (0x1 << 6));
}

static int chg_get_input_current_limit(struct sm5713_charger_data *charger)
{
    u8 reg;

    sm5713_read_reg(charger->i2c, SM5713_CHG_REG_VBUSCNTL, &reg);

    return ((reg & 0x7F) * 25) + 100;
}

static int chg_get_charging_current(struct sm5713_charger_data *charger)
{
    u8 reg;
    int fast_curr;

    sm5713_read_reg(charger->i2c, SM5713_CHG_REG_CHGCNTL2, &reg);

    if ((reg & 0x7F) >= 0x44) {
        fast_curr = 3500;
    } else {
        fast_curr = ((reg & 0x7F) * 50) + 100;
    }

    return fast_curr;
}

static int chg_get_topoff_current(struct sm5713_charger_data *charger)
{
    u8 reg;
    int topoff;

    sm5713_read_reg(charger->i2c, SM5713_CHG_REG_CHGCNTL5, &reg);

    if ((reg & 0x1F) >= 0x14) {
        topoff = 600;
    } else {
        topoff = ((reg & 0x1F) * 25) + 100;
    }

    return topoff;
}

static int chg_get_regulation_voltage(struct sm5713_charger_data *charger)
{
    u8 reg;

    sm5713_read_reg(charger->i2c, SM5713_CHG_REG_CHGCNTL4, &reg);

	return (((reg & 0x3F) - 2) * 10) + 4010;
}

#define PRINT_CHG_REG_NUM   26   
static void chg_print_regmap(struct sm5713_charger_data *charger)
{
	u8 regs[PRINT_CHG_REG_NUM] = {0x0, };
	int i;

	sm5713_bulk_read(charger->i2c, SM5713_CHG_REG_STATUS1, PRINT_CHG_REG_NUM, regs);

	pr_info("sm5713-charger: print regmap\n");
	for (i = 0; i < PRINT_CHG_REG_NUM; ++i) {
		pr_info("0x%02x:0x%02x ", SM5713_CHG_REG_STATUS1 + i, regs[i]);
        if (i % 8 == 0)
            pr_info("\n");
    }
}

static int sm5713_chg_create_attrs(struct device *dev)
{
	unsigned long i;
	int rc;

	for (i = 0; i < ARRAY_SIZE(sm5713_charger_attrs); i++) {
		rc = device_create_file(dev, &sm5713_charger_attrs[i]);
		if (rc)
			goto create_attrs_failed;
	}
	return rc;

create_attrs_failed:
	dev_err(dev, "%s: failed (%d)\n", __func__, rc);
	while (i--)
		device_remove_file(dev, &sm5713_charger_attrs[i]);
	return rc;
}

ssize_t sm5713_chg_show_attrs(struct device *dev, struct device_attribute *attr, char *buf)
{
	const ptrdiff_t offset = attr - sm5713_charger_attrs;
	int i = 0;

	switch (offset) {
	case CHIP_ID:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n", "SM5713");
		break;
	default:
		return -EINVAL;
	}
	return i;
}

ssize_t sm5713_chg_store_attrs(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	const ptrdiff_t offset = attr - sm5713_charger_attrs;
	int ret = 0;

	switch(offset) {
	case CHIP_ID:
		ret = count;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int psy_chg_get_online(struct sm5713_charger_data *charger)
{
    u8 reg;

    sm5713_read_reg(charger->i2c, SM5713_CHG_REG_STATUS1, &reg);

    return (reg & 0x1) ? 1 : 0;
}

static int psy_chg_get_status(struct sm5713_charger_data *charger)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	u8 reg_st1, reg_st2;

	sm5713_read_reg(charger->i2c, SM5713_CHG_REG_STATUS1, &reg_st1);
    sm5713_read_reg(charger->i2c, SM5713_CHG_REG_STATUS2, &reg_st2);
	dev_info(charger->dev, "%s: STATUS1(0x%02x), STATUS2(0x%02x)\n", __func__, reg_st1, reg_st2);

	if (reg_st2 & (0x1 << 5)) { /* check: Top-off */
		status = POWER_SUPPLY_STATUS_FULL;
	} else if (reg_st2 & (0x1 << 3)) {  /* check: Charging ON */
		status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		if (reg_st1 & (0x1 << 0)) { /* check: VBUS_POK */
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else {
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
	}

	return status;
}

static int psy_chg_get_health(struct sm5713_charger_data *charger)
{
    u8 reg;
    int health = POWER_SUPPLY_HEALTH_UNKNOWN;

    if (charger->is_charging) {
        chg_set_wdt_tmr_reset(charger);
    }
    chg_print_regmap(charger);  /* please keep this log message */

    sm5713_read_reg(charger->i2c, SM5713_CHG_REG_STATUS1, &reg);

    if (reg & (0x1 << 0)) {
        charger->unhealth_cnt = 0;
        health = POWER_SUPPLY_HEALTH_GOOD;
    } else {
        if (charger->unhealth_cnt < HEALTH_DEBOUNCE_CNT) {
            health = POWER_SUPPLY_HEALTH_GOOD;
            charger->unhealth_cnt++;
        } else {
            if (reg & (0x1 << 2)) {
                health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
            } else if (reg & (0x1 << 1)) {
                health = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
            }
        }
    }

	return health;
}

static int psy_chg_get_charge_type(struct sm5713_charger_data *charger)
{
    int charge_type;
    u8 reg;

    sm5713_read_reg(charger->i2c, SM5713_CHG_REG_STATUS2, &reg);

	if (charger->is_charging) {
        if (reg & (0x1 << 4)) { /* CHECK Q4FULLON Status */
			charge_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
        } else {
            charge_type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;     /* Linear-Charge mode */
        }
	} else {
		charge_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	return charge_type;
}

static int psy_chg_get_present(struct sm5713_charger_data *charger)
{
    u8 reg;

    sm5713_read_reg(charger->i2c, SM5713_CHG_REG_STATUS2, &reg);

    return (reg & (0x1 << 2)) ? 0 : 1;
}

static int sm5713_chg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct sm5713_charger_data *charger =
		container_of(psy, struct sm5713_charger_data, psy_chg);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
        val->intval = psy_chg_get_online(charger);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = psy_chg_get_status(charger);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = psy_chg_get_health(charger);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX: /* get input current which was set */
        val->intval = charger->input_current;
		break;
    case POWER_SUPPLY_PROP_CURRENT_AVG: /* get input current which was read */
		val->intval = chg_get_input_current_limit(charger);
        break;
	case POWER_SUPPLY_PROP_CURRENT_NOW: /* get charge current which was set */
        val->intval = charger->charging_current;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT: /* get charge current which was read */	
        val->intval = chg_get_charging_current(charger);
		break;
	case POWER_SUPPLY_PROP_CURRENT_FULL:
		val->intval = chg_get_topoff_current(charger);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = psy_chg_get_charge_type(charger);
		break;
#if defined(CONFIG_BATTERY_SWELLING) || defined(CONFIG_BATTERY_SWELLING_SELF_DISCHARGING)
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = chg_get_regulation_voltage(charger);
		break;
#endif
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = psy_chg_get_present(charger);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = charger->is_charging;
		break;
	case POWER_SUPPLY_PROP_MAX ... POWER_SUPPLY_EXT_PROP_MAX:
		return -ENODATA;
	default:
		return -EINVAL;
	}

	return 0;
}

static void psy_chg_set_charging_enable(struct sm5713_charger_data *charger, int charge_mode)
{
	int buck_off = false;
    bool buck_off_status = (sm5713_charger_oper_get_current_status() & (0x1 << SM5713_CHARGER_OP_EVENT_SUSPEND)) ? 1 : 0;

	dev_info(charger->dev, "charger_mode changed [%d] -> [%d]\n", charger->charge_mode, charge_mode);
	charger->charge_mode = charge_mode;

	switch (charger->charge_mode) {
		case SEC_BAT_CHG_MODE_BUCK_OFF:
			buck_off = true;
			break;
		case SEC_BAT_CHG_MODE_CHARGING_OFF:
			charger->is_charging = false;
			break;
		case SEC_BAT_CHG_MODE_CHARGING:
			charger->is_charging = true;
			break;
	}

    chg_set_wdt_enable(charger, charger->is_charging);
    chg_set_enq4fet(charger, charger->is_charging);
    if (buck_off != buck_off_status) {
        sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_SUSPEND, buck_off);
    }
}

static void psy_chg_set_online(struct sm5713_charger_data *charger, int cable_type)
{
	dev_info(charger->dev, "[start] cable_type(%d->%d), op_mode(%d), op_status(0x%x)",
                charger->cable_type, cable_type, sm5713_charger_oper_get_current_op_mode(), 
                sm5713_charger_oper_get_current_status());
    charger->cable_type = cable_type;

	if (charger->cable_type == POWER_SUPPLY_TYPE_BATTERY || 
                charger->cable_type == POWER_SUPPLY_TYPE_UNKNOWN) {
		sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_VBUSIN, 0);
		sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_PWR_SHAR, 0);
		/* set default input current */
		chg_set_input_current_limit(charger, 500);
	} else {
		sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_VBUSIN, 1);
	}

	dev_info(charger->dev, "[end] op_mode(%d), op_status(0x%x)\n",
                sm5713_charger_oper_get_current_op_mode(),
                sm5713_charger_oper_get_current_status());
}

static void psy_chg_set_otg_control(struct sm5713_charger_data *charger, bool enable)
{
    if (enable == charger->otg_on) {
        return;
    }
    sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_USB_OTG, enable);
    charger->otg_on = enable;
    power_supply_changed(&charger->psy_otg);
}

#if defined(CONFIG_AFC_CHARGER_MODE)
extern void sm5713_muic_charger_init(void);
#endif

static int sm5713_chg_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct sm5713_charger_data *charger =
		container_of(psy, struct sm5713_charger_data, psy_chg);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		charger->status = val->intval;
		break;
    case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        psy_chg_set_charging_enable(charger, val->intval);
        chg_print_regmap(charger);
		break;
    case POWER_SUPPLY_PROP_ONLINE:
        psy_chg_set_online(charger, val->intval);
        break;
    case POWER_SUPPLY_PROP_CURRENT_MAX:
        dev_info(charger->dev, "input limit changed [%dmA] -> [%dmA]\n",
                    charger->input_current, val->intval);
        charger->input_current = val->intval;
        chg_set_input_current_limit(charger, charger->input_current);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        dev_info(charger->dev, "charging current changed [%dmA] -> [%dmA]\n",
                    charger->charging_current, val->intval);
        charger->charging_current = val->intval;
        chg_set_charging_current(charger, charger->charging_current);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		break;
	case POWER_SUPPLY_PROP_CURRENT_FULL:
        chg_set_topoff_current(charger, val->intval);
		break;
#if defined(CONFIG_BATTERY_SWELLING) || defined(CONFIG_BATTERY_SWELLING_SELF_DISCHARGING)
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
        dev_info(charger->dev, "float voltage changed [%dmV] -> [%dmV]\n",
                    charger->pdata->chg_float_voltage, val->intval);
        charger->pdata->chg_float_voltage = val->intval;
        chg_set_batreg(charger, charger->pdata->chg_float_voltage);
		break;
#endif
    case POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL:
        dev_info(charger->dev, "OTG_CONTROL=%s\n", val->intval ? "ON" : "OFF");
        psy_chg_set_otg_control(charger,val->intval);
		break;
		case POWER_SUPPLY_PROP_ENERGY_NOW:
			/* if jig attached, */
			break;
#if defined(CONFIG_AFC_CHARGER_MODE) && defined(CONFIG_MUIC_HV)
		case POWER_SUPPLY_PROP_AFC_CHARGER_MODE:
			sm5713_muic_charger_init();
			break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

static int sm5713_otg_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sm5713_charger_data *charger =
		container_of(psy, struct sm5713_charger_data, psy_otg);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger->otg_on;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sm5713_otg_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct sm5713_charger_data *charger =
		container_of(psy, struct sm5713_charger_data, psy_otg);

	switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        dev_info(charger->dev, "%s: OTG %s\n", __func__, 
                    val->intval ? "ON" : "OFF");
        psy_chg_set_otg_control(charger, val->intval);
		break;
	default:
		return -EINVAL;
	}

    return 0;
}


static irqreturn_t chg_vbuspok_isr(int irq, void *data)
{
	struct sm5713_charger_data *charger = data;

	dev_info(charger->dev, "%s: irq=%d\n", __func__, irq);

	return IRQ_HANDLED;
}

static irqreturn_t chg_aicl_isr(int irq, void *data)
{
	struct sm5713_charger_data *charger = data;

	dev_info(charger->dev, "%s: irq=%d\n", __func__, irq);

	return IRQ_HANDLED;
}

static irqreturn_t chg_done_isr(int irq, void *data)
{
	struct sm5713_charger_data *charger = data;

	dev_info(charger->dev, "%s: irq=%d\n", __func__, irq);

    // Toggle ENQ4FET for Re-cycling charger loop 
	chg_set_enq4fet(charger, 0);
	msleep(10);
	chg_set_enq4fet(charger, 1);

	return IRQ_HANDLED;
}

static irqreturn_t chg_vsysovp_isr(int irq, void *data)
{
	struct sm5713_charger_data *charger = data;

	dev_info(charger->dev, "%s: irq=%d\n", __func__, irq);

	return IRQ_HANDLED;
}

static irqreturn_t chg_vbusshort_isr(int irq, void *data)
{
	struct sm5713_charger_data *charger = data;

	dev_info(charger->dev, "%s: irq=%d\n", __func__, irq);

	return IRQ_HANDLED;
}

static inline void sm5713_chg_init(struct sm5713_charger_data *charger)
{
    chg_set_aicl(charger, 1, AICL_TH_V_4_5);
    chg_set_dischg_limit(charger, DISCHG_LIMIT_C_4_5);
    chg_set_batreg(charger, charger->pdata->chg_float_voltage);
    chg_set_iq3limit(charger, BST_IQ3LIMIT_C_4_0);
    chg_set_wdt_timer(charger, WDT_TIME_S_90);
    chg_set_topoff_timer(charger, TOPOFF_TIME_M_45);
    chg_set_autostop(charger, 1);

    chg_print_regmap(charger);

	dev_info(charger->dev, "%s: init done.\n", __func__);
}

static int sm5713_charger_parse_dt(struct device *dev,	
	struct sm5713_charger_platform_data *pdata)
{
	struct device_node *np;
	int ret = 0;

	np = of_find_node_by_name(NULL, "battery");
	if (!np) {
		dev_err(dev, "%s: can't find battery node\n", __func__);
	} else {
		ret = of_property_read_u32(np, "battery,chg_float_voltage",
					   &pdata->chg_float_voltage);
		if (ret) {
			dev_info(dev, "%s: battery,chg_float_voltage is Empty\n", __func__);
			pdata->chg_float_voltage = 4200;
		}
		pr_info("%s: battery,chg_float_voltage is %d\n",
			__func__, pdata->chg_float_voltage);
	}

	dev_info(dev, "%s: parse dt done.\n", __func__);
	return 0;
}

/* if need to set sm5713 pdata */
static struct of_device_id sm5713_charger_match_table[] = {
	{ .compatible = "samsung,sm5713-charger",},
	{},
};

static int sm5713_charger_probe(struct platform_device *pdev)
{
	struct sm5713_dev *sm5713 = dev_get_drvdata(pdev->dev.parent);
	struct sm5713_platform_data *pdata = dev_get_platdata(sm5713->dev);
	struct sm5713_charger_data *charger;
	int ret = 0;

	dev_info(&pdev->dev, "%s: probe start\n", __func__);
	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	charger->dev = &pdev->dev;
	charger->i2c = sm5713->charger;
	charger->otg_on = false;
	mutex_init(&charger->charger_mutex);

	charger->pdata = devm_kzalloc(&pdev->dev, sizeof(*(charger->pdata)),
			GFP_KERNEL);
	if (!charger->pdata) {
		dev_err(&pdev->dev, "%s: failed to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_parse_dt_nomem;
	}
    ret = sm5713_charger_parse_dt(&pdev->dev, charger->pdata);
	if (ret < 0) {
		goto err_parse_dt;
    }
	platform_set_drvdata(pdev, charger);

    sm5713_chg_init(charger);
    sm5713_charger_oper_table_init(charger->i2c);

	charger->psy_chg.name           = "sm5713-charger";
	charger->psy_chg.type           = POWER_SUPPLY_TYPE_UNKNOWN;
	charger->psy_chg.get_property   = sm5713_chg_get_property;
	charger->psy_chg.set_property   = sm5713_chg_set_property;
	charger->psy_chg.properties     = sm5713_charger_props;
	charger->psy_chg.num_properties = ARRAY_SIZE(sm5713_charger_props);
	charger->psy_otg.name			= "otg";
	charger->psy_otg.type			= POWER_SUPPLY_TYPE_OTG;
	charger->psy_otg.get_property	= sm5713_otg_get_property;
	charger->psy_otg.set_property	= sm5713_otg_set_property;
	charger->psy_otg.properties		= sm5713_otg_props;
	charger->psy_otg.num_properties	= ARRAY_SIZE(sm5713_otg_props);

    ret = power_supply_register(&pdev->dev, &charger->psy_chg);
	if (ret) {
		goto err_power_supply_register;
	}
	ret = power_supply_register(&pdev->dev, &charger->psy_otg);
	if (ret) {
		goto err_power_supply_register_otg;
	}

    ret = sm5713_chg_create_attrs(charger->psy_chg.dev);
	if (ret) {
		dev_err(charger->dev,"%s : Failed to create_attrs\n", __func__);
		goto err_reg_irq;
	}

    /* Request IRQs */
    charger->irq_vbuspok = pdata->irq_base + SM5713_CHG_IRQ_INT1_VBUSPOK;
	ret = request_threaded_irq(charger->irq_vbuspok, NULL,
			chg_vbuspok_isr, 0 , "vbuspok-irq", charger);
	if (ret < 0) {
		dev_err(sm5713->dev, "%s: fail to request vbuspok-irq:%d (ret=%d)\n",
					__func__, charger->irq_vbuspok, ret);
		goto err_reg_irq;
	}

    charger->irq_aicl = pdata->irq_base + SM5713_CHG_IRQ_INT2_AICL;
	ret = request_threaded_irq(charger->irq_aicl, NULL,
			chg_aicl_isr, 0 , "aicl-irq", charger);
	if (ret < 0) {
		dev_err(sm5713->dev, "%s: fail to request aicl-irq:%d (ret=%d)\n",
					__func__, charger->irq_aicl, ret);
		goto err_reg_irq;
	}

    charger->irq_done = pdata->irq_base + SM5713_CHG_IRQ_INT2_DONE;
    ret = request_threaded_irq(charger->irq_done, NULL,
            chg_done_isr, 0 , "done-irq", charger);
    if (ret < 0) {
        dev_err(sm5713->dev, "%s: fail to request done-irq:%d (ret=%d)\n",
                    __func__, charger->irq_done, ret);
        goto err_reg_irq;
    }

    charger->irq_vsysovp = pdata->irq_base + SM5713_CHG_IRQ_INT3_VSYSOVP;
    ret = request_threaded_irq(charger->irq_vsysovp, NULL,
            chg_vsysovp_isr, 0 , "vsysovp-irq", charger);
    if (ret < 0) {
        dev_err(sm5713->dev, "%s: fail to request vsysovp-irq:%d (ret=%d)\n",
                    __func__, charger->irq_vsysovp, ret);
        goto err_reg_irq;
    }

    charger->irq_vbusshort = pdata->irq_base + SM5713_CHG_IRQ_INT6_VBUSSHORT;
    ret = request_threaded_irq(charger->irq_vbusshort, NULL,
            chg_vbusshort_isr, 0 , "vbusshort-irq", charger);
    if (ret < 0) {
        dev_err(sm5713->dev, "%s: fail to request vbusshort-irq:%d (ret=%d)\n",
                    __func__, charger->irq_vbusshort, ret);
        goto err_reg_irq;
    }

	dev_info(&pdev->dev, "%s: probe done.\n", __func__);

	return 0;

err_reg_irq:
	power_supply_unregister(&charger->psy_chg);
err_power_supply_register_otg:
	power_supply_unregister(&charger->psy_otg);
err_power_supply_register:
err_parse_dt:
err_parse_dt_nomem:
	mutex_destroy(&charger->charger_mutex);
	kfree(charger);
	return ret;
}

static int sm5713_charger_remove(struct platform_device *pdev)
{
	struct sm5713_charger_data *charger =
		platform_get_drvdata(pdev);

	power_supply_unregister(&charger->psy_chg);
	power_supply_unregister(&charger->psy_chg);

	mutex_destroy(&charger->charger_mutex);

    kfree(charger);

    return 0;
}

#if defined CONFIG_PM
static int sm5713_charger_suspend(struct device *dev)
{
	return 0;
}

static int sm5713_charger_resume(struct device *dev)
{
	return 0;
}
#else
#define sm5713_charger_suspend NULL
#define sm5713_charger_resume NULL
#endif

static void sm5713_charger_shutdown(struct device *dev)
{
	pr_info("%s: SM5713 Charger driver shutdown\n", __func__);
}

static SIMPLE_DEV_PM_OPS(sm5713_charger_pm_ops, sm5713_charger_suspend,
		sm5713_charger_resume);

static struct platform_driver sm5713_charger_driver = {
    .driver = {
        .name	        = "sm5713-charger",
		.owner	        = THIS_MODULE,
		.of_match_table = sm5713_charger_match_table,
		.pm		        = &sm5713_charger_pm_ops,
		.shutdown	    = sm5713_charger_shutdown,
	},
	.probe      = sm5713_charger_probe,
	.remove		= sm5713_charger_remove,
};

static int __init sm5713_charger_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&sm5713_charger_driver);

	return ret;
}
module_init(sm5713_charger_init);

static void __exit sm5713_charger_exit(void)
{
	platform_driver_unregister(&sm5713_charger_driver);
}
module_exit(sm5713_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Charger driver for SM5713");
