/*
 * sm5011-core.h
 *
 * Copyright (c) 2018 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_SM5011_CORE_H
#define __LINUX_MFD_SM5011_CORE_H

#include <linux/regulator/consumer.h>

#define NUM_IRQ_REGS	2

#define SM5011_PMIC_REV(iodev)	(iodev)->rev_num

/*
 * struct sm5011_pmic_dev - sm5011's master device for sub-drivers
 * @ dev: master device of the chip (can be used to access platform data)
 * @ pdata: pointer to private data used to pass platform data to child
 * @ regmap : register map used to access registers  
 * @ i2c : i2c client private data for sm5011
 * @ sm5011_lock : mutex for sm5011
 * @ iolock : mutex for serializing io access
 * @ irqlock : mutex for buslock
 * @ rev_num : revision number for sm5011
 * @ irq_base : base IRQ number for sm5011, required for IRQs
 * @ irq : generic IRQ number for sm5011
 * @ irq_gpio : GPIO for generic IRQ for sm5011
 * @ irq_masks_cur: currently active value
 * @ irq_masks_cache: cached hardware value
 */
struct sm5011_pmic_dev {
	struct device *dev;
	struct sm5011_platform_data *pdata;
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct mutex sm5011_lock;
	struct mutex iolock;
	struct mutex irqlock;

	int rev_num;
	int irq_base;
	int irq;
	int irq_gpio;
	//struct regmap_irq_chip_data *irq_data;

	u8 irq_masks_cur[NUM_IRQ_REGS];
	u8 irq_masks_cache[NUM_IRQ_REGS];
	bool wakeup;
};

/*
 * struct sm5011_pmic_dev - sm5011's master device for sub-drivers
 * @bucks : pointer to private data used to buck regulator data(SM5011_BUCK2 ~ SM5011_BUCK6)
 * @ldos : pointer to private data used to ldo regulator data(SM5011_LDO1 ~ SM5011_LDO20)
 * @opmode : pointer to regulator operation mode data
 * @num_bucks : number of bucks (SM5011_BUCK2 ~ SM5011_BUCK6)
 * @num_ldos : number of ldos (SM5011_LDO1 ~ SM5011_LDO120)
 * @npwrstm_pin : gpio number of nPWRSTM pin
 * @ irq_base : base IRQ number for sm5011, required for IRQs
 * @ irq_gpio : GPIO for generic IRQ for sm5011 
 * @ entcxo_pin : GPIO for ENTCXO (Only LDO6)
 * @ enl16_pin : GPIO for ENL16 (Only LDO16) 
 */
struct sm5011_platform_data {
	struct sm5011_regulator_data	*bucks;	//SM5011_BUCK2 ~ SM5011_BUCK6
	struct sm5011_regulator_data	*ldos;	//SM5011_LDO1 ~ SM5011_LDO20

	struct sm5011_opmode_data		*opmode;

	int				num_bucks;
	int				num_ldos;

	int				npwrstm_pin;
	int				irq_base;
	int 			irq_gpio;
	int 			entcxo_pin;
	int				enl16_pin;

	bool			wakeup;

	/* Bcuk2 */
	int buck2mode;

	/* Bcuk3 */
	int buck3mode;

	/* Bcuk4 */
	int buck4mode;	

	/* Bcuk5 */
	int buck5mode;
	
	/* Bcuk6 */
	int buck6mode;

	int en_32kout;
	int smpl_en;
	int smpl_power_on_type;
	int smpl_timer_val;

	int longkey_val;

	int npwrstm_en;
	int mask_int_en;

	int envbatng_en;
	int envref_en;
	
	int mrstb_en;
	int mrstb_hehavior;
	int mrstb_nreset;
	int mrstb_key;
	int mrstb_timer_val;
	
	int wdt_en;
	int wdt_timer_val;	
	
	int comp_en;		
	int comp_time_val;
	int comp_duty_val;
	int comp_vref_val;

	/* ---- RTC ---- */
	int rtc_24hr_mode;	
	struct rtc_time *init_time;
};

/* 
  * [ External Function ]
  */
extern int sm5011_reg_read(struct sm5011_pmic_dev *sm5011_pmic, u32 reg, void *dest);
extern int sm5011_bulk_read(struct sm5011_pmic_dev *sm5011_pmic, u32 reg, int count, u8 *buf);
extern int sm5011_reg_write(struct sm5011_pmic_dev *sm5011_pmic, u32 reg, u32 value);
extern int sm5011_bulk_write(struct sm5011_pmic_dev *sm5011_pmic, u32 reg, int count, u8 *buf);
extern int sm5011_reg_update(struct sm5011_pmic_dev *sm5011_pmic, u32 reg, u32 val, u32 mask);

extern void sm5011_core_lock(void);
extern void sm5011_core_unlock(void);

#endif /*  __LINUX_MFD_SM5011_CORE_H */
