/*
 * sm5713-charger.c - SM5713 Charger operation mode control module.
 *
 * Copyright (C) 2017 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/mfd/sm5713.h>
#include <linux/mfd/sm5713-private.h>

enum {
    OP_MODE_SUSPEND     = 0x0,
    OP_MODE_CHG_ON_VBUS = 0x5,
    OP_MODE_USB_OTG     = 0x7,
    OP_MODE_FLASH_BOOST = 0x8,
};

enum {
    BSTOUT_4400mV   = 0x0,
    BSTOUT_4500mV   = 0x1,
    BSTOUT_4600mV   = 0x2,
    BSTOUT_4700mV   = 0x3,
    BSTOUT_4800mV   = 0x4,
    BSTOUT_5000mV   = 0x5,
    BSTOUT_5100mV   = 0x6,
    BSTOUT_5200mV   = 0x7,
};

enum {
	OTG_CURRENT_500mA   = 0x0,
	OTG_CURRENT_900mA   = 0x1,
	OTG_CURRENT_1200mA  = 0x2,
	OTG_CURRENT_1500mA  = 0x3,
};

#define make_OP_STATUS(vbus, otg, pwr_shar, flash, torch, suspend)  (((vbus & 0x1)      << SM5713_CHARGER_OP_EVENT_VBUSIN)      | \
                                                                     ((otg & 0x1)       << SM5713_CHARGER_OP_EVENT_USB_OTG)     | \
                                                                     ((pwr_shar & 0x1)  << SM5713_CHARGER_OP_EVENT_FLASH)       | \
                                                                     ((flash & 0x1)     << SM5713_CHARGER_OP_EVENT_FLASH)       | \
                                                                     ((torch & 0x1)     << SM5713_CHARGER_OP_EVENT_TORCH)       | \
                                                                     ((suspend & 0x1)   << SM5713_CHARGER_OP_EVENT_SUSPEND))

struct sm5713_charger_oper_table_info {
	unsigned short status;
	unsigned char oper_mode;
	unsigned char BST_OUT;
	unsigned char OTG_CURRENT;
};

struct sm5713_charger_oper_info {
	struct i2c_client *i2c;
    struct mutex op_mutex;
    int max_table_num;
	struct sm5713_charger_oper_table_info current_table;
};
static struct sm5713_charger_oper_info *oper_info = NULL;

/**
 *  (VBUS in/out) (USB-OTG in/out) (PWR-SHAR in/out)
 *  (Flash on/off) (Torch on/off) (Suspend mode on/off)
 **/
static struct sm5713_charger_oper_table_info sm5713_charger_op_mode_table[] = {
	/* Charger=ON Mode in a valid Input */
	{ make_OP_STATUS(0,0,0,0,0,0), OP_MODE_CHG_ON_VBUS, BSTOUT_5100mV, OTG_CURRENT_500mA},
    { make_OP_STATUS(1,0,0,0,0,0), OP_MODE_CHG_ON_VBUS, BSTOUT_5100mV, OTG_CURRENT_500mA},
    { make_OP_STATUS(1,1,0,0,0,0), OP_MODE_CHG_ON_VBUS, BSTOUT_5100mV, OTG_CURRENT_500mA},      /* Prevent : VBUS + OTG timing sync */
    { make_OP_STATUS(1,0,0,0,1,0), OP_MODE_CHG_ON_VBUS, BSTOUT_5100mV, OTG_CURRENT_500mA},
    /* Flash Boost Mode */
    { make_OP_STATUS(0,0,0,1,0,0), OP_MODE_FLASH_BOOST, BSTOUT_4500mV, OTG_CURRENT_500mA},
    { make_OP_STATUS(1,0,0,1,0,0), OP_MODE_FLASH_BOOST, BSTOUT_4500mV, OTG_CURRENT_500mA},
    { make_OP_STATUS(0,1,0,1,0,0), OP_MODE_FLASH_BOOST, BSTOUT_4500mV, OTG_CURRENT_900mA},
    { make_OP_STATUS(0,0,1,1,0,0), OP_MODE_FLASH_BOOST, BSTOUT_4500mV, OTG_CURRENT_900mA},
    { make_OP_STATUS(0,0,0,1,1,0), OP_MODE_FLASH_BOOST, BSTOUT_4500mV, OTG_CURRENT_900mA},
    { make_OP_STATUS(0,0,0,0,1,0), OP_MODE_FLASH_BOOST, BSTOUT_4500mV, OTG_CURRENT_900mA},
    /* USB OTG Mode */
    { make_OP_STATUS(0,1,0,0,0,0), OP_MODE_USB_OTG,     BSTOUT_5100mV, OTG_CURRENT_900mA},
    { make_OP_STATUS(0,0,1,0,0,0), OP_MODE_USB_OTG,     BSTOUT_5100mV, OTG_CURRENT_900mA},
    { make_OP_STATUS(0,1,0,0,1,0), OP_MODE_USB_OTG,     BSTOUT_5100mV, OTG_CURRENT_900mA},
    { make_OP_STATUS(0,0,1,0,1,0), OP_MODE_USB_OTG,     BSTOUT_5100mV, OTG_CURRENT_900mA},
    /* Suspend Mode */
    { make_OP_STATUS(0,0,0,0,0,1), OP_MODE_SUSPEND,     BSTOUT_5100mV, OTG_CURRENT_900mA},      /* Reserved position of SUSPEND mode table */
};

static int set_OP_MODE(struct i2c_client *i2c, u8 mode)
{
	return sm5713_update_reg(i2c, SM5713_CHG_REG_CNTL2, (mode << 0), (0xF << 0));
}

static int set_BSTOUT(struct i2c_client *i2c, u8 bstout)
{
	return sm5713_update_reg(i2c, SM5713_CHG_REG_BSTCNTL1, (bstout << 0), (0xF << 0));
}

static int set_OTG_CURRENT(struct i2c_client *i2c, u8 otg_curr)
{
	return sm5713_update_reg(i2c, SM5713_CHG_REG_BSTCNTL1, (otg_curr << 6), (0x3 << 6));
}

static inline int change_op_table(unsigned char new_status)
{
	int i = 0;

    /* Check actvated Suspend Mode */
    if (new_status & (0x1 << SM5713_CHARGER_OP_EVENT_SUSPEND)) {
        i = oper_info->max_table_num - 1;    /* Reserved SUSPEND Mode Table index */
    } else {
        /* Search matched Table */
    	for (i=0; i < oper_info->max_table_num; ++i) {
    		if (new_status == sm5713_charger_op_mode_table[i].status) {
    			break;
    		}
    	}
    }
    if (i == oper_info->max_table_num) {
		pr_err("sm5713-charger: %s: can't find matched charger op_mode table (status = 0x%x)\n", __func__, new_status);
		return -EINVAL;
	}

    /* Update current table info */
	if (sm5713_charger_op_mode_table[i].BST_OUT != oper_info->current_table.BST_OUT) {
		set_BSTOUT(oper_info->i2c, sm5713_charger_op_mode_table[i].BST_OUT);
		oper_info->current_table.BST_OUT = sm5713_charger_op_mode_table[i].BST_OUT;
	}
	if (sm5713_charger_op_mode_table[i].OTG_CURRENT != oper_info->current_table.OTG_CURRENT) {
		set_OTG_CURRENT(oper_info->i2c, sm5713_charger_op_mode_table[i].OTG_CURRENT);
		oper_info->current_table.OTG_CURRENT = sm5713_charger_op_mode_table[i].OTG_CURRENT;
	}
    if (sm5713_charger_op_mode_table[i].oper_mode != oper_info->current_table.oper_mode) {
        set_OP_MODE(oper_info->i2c, sm5713_charger_op_mode_table[i].oper_mode);
		oper_info->current_table.oper_mode = sm5713_charger_op_mode_table[i].oper_mode;
	}
	oper_info->current_table.status = new_status;

	pr_info("sm5713-charger: %s: New table[%d] info (STATUS: 0x%x, MODE: %d, BST_OUT: 0x%x, OTG_CURRENT: 0x%x\n",
			__func__, i, oper_info->current_table.status, oper_info->current_table.oper_mode, 
            oper_info->current_table.BST_OUT, oper_info->current_table.OTG_CURRENT);

    return 0;
}

static inline unsigned char update_status(int event_type, bool enable)
{
	if (event_type > SM5713_CHARGER_OP_EVENT_VBUSIN) {
        pr_debug("sm5713-charger: %s: invalid event type (type=0x%x)\n", __func__, event_type);
		return oper_info->current_table.status;
	}

	if (enable) {
		return (oper_info->current_table.status | (1 << event_type));
	} else {
		return (oper_info->current_table.status & ~(1 << event_type));
	}
}

int sm5713_charger_oper_push_event(int event_type, bool enable)
{
	unsigned char new_status;
    int ret = 0;

	if (oper_info == NULL) {
		pr_err("sm5713-charger: %s: required init op_mode table\n", __func__);
		return -ENOENT;
	}
	pr_info("sm5713-charger: %s: event_type=%d, enable=%d\n", __func__, event_type, enable);

	mutex_lock(&oper_info->op_mutex);

	new_status = update_status(event_type, enable);
    if (new_status == oper_info->current_table.status) {
		goto out;
	}
    ret = change_op_table(new_status);

out:
	mutex_unlock(&oper_info->op_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(sm5713_charger_oper_push_event);

static inline int detect_initial_table_index(struct i2c_client *i2c)
{
    return 0;
}

int sm5713_charger_oper_table_init(struct i2c_client *i2c)
{
    int index;

    if (oper_info) {
        pr_info("sm5713-charger: %s: already initialized\n", __func__);
        return 0;
    }

	if (i2c == NULL) {
		pr_err("sm5713-charger: %s: invalid i2c client handler=n", __func__);
		return -EINVAL;
	}

    oper_info = kmalloc(sizeof(struct sm5713_charger_oper_info), GFP_KERNEL);
    if (oper_info == NULL) {
        pr_err("sm5713-charger: %s: failed to alloctae memory\n", __func__);
        return -ENOMEM;
    }
	oper_info->i2c = i2c;

    mutex_init(&oper_info->op_mutex);

	/* set default operation mode condition */
	oper_info->max_table_num = ARRAY_SIZE(sm5713_charger_op_mode_table);
    index = detect_initial_table_index(oper_info->i2c);
	oper_info->current_table.status = sm5713_charger_op_mode_table[index].status;
	oper_info->current_table.oper_mode = sm5713_charger_op_mode_table[index].oper_mode;
	oper_info->current_table.BST_OUT = sm5713_charger_op_mode_table[index].BST_OUT;
	oper_info->current_table.OTG_CURRENT = sm5713_charger_op_mode_table[index].OTG_CURRENT;

	set_OP_MODE(oper_info->i2c, oper_info->current_table.oper_mode);
	set_BSTOUT(oper_info->i2c, oper_info->current_table.BST_OUT);
	set_OTG_CURRENT(oper_info->i2c, oper_info->current_table.OTG_CURRENT);

	pr_info("sm5713-charger: %s: current table info (STATUS: 0x%x, MODE: %d, BST_OUT: 0x%x, OTG_CURRENT: 0x%x)\n", \
			__func__, oper_info->current_table.status, oper_info->current_table.oper_mode, oper_info->current_table.BST_OUT, \
            oper_info->current_table.OTG_CURRENT);

	return 0;
}
EXPORT_SYMBOL_GPL(sm5713_charger_oper_table_init);

int sm5713_charger_oper_get_current_status(void)
{
    if (oper_info == NULL) {
        return -EINVAL;
    }
	return oper_info->current_table.status;
}
EXPORT_SYMBOL_GPL(sm5713_charger_oper_get_current_status);

int sm5713_charger_oper_get_current_op_mode(void)
{
    if (oper_info == NULL) {
        return -EINVAL;
    }
    return oper_info->current_table.oper_mode;
}
EXPORT_SYMBOL_GPL(sm5713_charger_oper_get_current_op_mode);
