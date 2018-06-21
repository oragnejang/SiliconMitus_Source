/* 
 * drivers/battery/sm5705_charger_oper.h
 * 
 * SM5705 Charger Operation Mode controller
 *
 * Copyright (C) 2015 Siliconmitus Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 */

#include <linux/init.h>
#include <linux/mfd/sm5705/sm5705.h>

enum SM5705_CHARGER_OP_EVENT_TYPE {
    SM5705_CHARGER_OP_EVENT_VBUS                = 0x5,
    SM5705_CHARGER_OP_EVENT_WPC                 = 0x4,
    SM5705_CHARGER_OP_EVENT_FLASH               = 0x3,
    SM5705_CHARGER_OP_EVENT_TORCH               = 0x2,
    SM5705_CHARGER_OP_EVENT_OTG                 = 0x1,
    SM5705_CHARGER_OP_EVENT_PWR_SHAR            = 0x0,
};

int sm5705_charger_oper_push_event(int event_type, bool enable);
int sm5705_charger_oper_table_init(struct i2c_client *i2c);



