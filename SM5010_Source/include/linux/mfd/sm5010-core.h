/*
 * sm5010-core.h
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_SM5010_CORE_H
#define __LINUX_MFD_SM5010_CORE_H

#include <linux/regulator/consumer.h>

#define NUM_IRQ_REGS	2

#define SM5010_PMIC_REV(iodev)	(iodev)->rev_num

/*
 * struct sm5010_pmic_dev - sm5010's master device for sub-drivers
 * @ dev: master device of the chip (can be used to access platform data)
 * @ pdata: pointer to private data used to pass platform data to child
 * @ regmap : register map used to access registers  
 * @ i2c : i2c client private data for sm5010
 * @ sm5010_lock : mutex for sm5010
 * @ iolock : mutex for serializing io access
 * @ irqlock : mutex for buslock
 * @ rev_num : revision number for sm5010
 * @ irq_base : base IRQ number for sm5010, required for IRQs
 * @ irq : generic IRQ number for sm5010
 * @ irq_gpio : GPIO for generic IRQ for sm5010
 * @ irq_masks_cur: currently active value
 * @ irq_masks_cache: cached hardware value
 */
struct sm5010_pmic_dev {
	struct device *dev;
	struct sm5010_platform_data *pdata;
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct mutex sm5010_lock;
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
 * struct sm5010_pmic_dev - sm5010's master device for sub-drivers
 * @bucks : pointer to private data used to buck regulator data(SM5010_BUCK2 ~ SM5010_PABUCK)
 * @ldos : pointer to private data used to ldo regulator data(SM5010_LDO1 ~ SM5010_LDO20)
 * @opmode : pointer to regulator operation mode data
 * @num_bucks : number of bucks (SM5010_BUCK2 ~ SM5010_PABUCK)
 * @num_ldos : number of ldos (SM5010_LDO1 ~ SM5010_LDO120)
 * @dvs_pin : gpios number of three dvs pins
 * @npwrstm_pin : gpio number of nPWRSTM pin
 * @ irq_base : base IRQ number for sm5010, required for IRQs
 * @ irq_gpio : GPIO for generic IRQ for sm5010 
 * @ entcxo_pin : GPIO for ENTCXO (Only LDO6)
 * @ enl16_pin : GPIO for ENL16 (Only LDO16) 
 * @ buck1dvsout : Initial voltage values for SM5010_BUCK1DVS
 * @ buck1npwrstmout : Initial voltage value for SM5010_BUCK1nPWRSTM
 */
struct sm5010_platform_data {
	struct sm5010_regulator_data	*bucks;	//SM5010_BUCK2 ~ SM5010_PABUCK
	struct sm5010_regulator_data	*ldos;	//SM5010_LDO1 ~ SM5010_LDO20

	struct sm5010_opmode_data		*opmode;

	int				num_bucks;
	int				num_ldos;

	int				dvs_pin[3];
	int				npwrstm_pin;
	int				irq_base;
	int 			irq_gpio;
	int 			entcxo_pin;
	int				enl16_pin;

	bool			wakeup;

	unsigned int buck1dvsout[8];
	unsigned int buck1npwrstmout;

	/* Bcuk1 DVS */
	int	enshedding;
	int freq;
	int dvsrampb1;
	int enrampb1down;
	int buck1adisen;
	int buck1mode;

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
extern int sm5010_reg_read(struct sm5010_pmic_dev *sm5010_pmic, u32 reg, void *dest);
extern int sm5010_bulk_read(struct sm5010_pmic_dev *sm5010_pmic, u32 reg, int count, u8 *buf);
extern int sm5010_reg_write(struct sm5010_pmic_dev *sm5010_pmic, u32 reg, u32 value);
extern int sm5010_bulk_write(struct sm5010_pmic_dev *sm5010_pmic, u32 reg, int count, u8 *buf);
extern int sm5010_reg_update(struct sm5010_pmic_dev *sm5010_pmic, u32 reg, u32 val, u32 mask);

extern int sm5010_get_dvs_is_enabled(void);
extern int sm5010_set_dvs_pin(bool gpio_val);

extern void sm5010_core_lock(void);
extern void sm5010_core_unlock(void);

#endif /*  __LINUX_MFD_SM5010_CORE_H */
