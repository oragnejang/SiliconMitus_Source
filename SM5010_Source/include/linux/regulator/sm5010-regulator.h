/*
 * sm5010-regulator.h
 *
 * Copyright (c) 2016 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_REGULATOR_SM5010_H
#define __LINUX_REGULATOR_SM5010_H

/**
 * sm5010_regulator_data - regulator data
 * @id: regulator id
 * @initdata: regulator init data (contraints, supplies, ...)
 * @always_onoff_npwrstm : regulator's on/off is always controlled by npwrstm when system is on  
 * @always_lpm_npwrstm : regulator's lpm is always controlled by npwrstm when system is on 
 */
struct sm5010_regulator_data {
	int				id;
	struct regulator_init_data	*initdata;
	struct device_node *reg_node;
	int				always_onoff_npwrstm;	
	int				always_lpm_npwrstm;
};

/*
 * sm5010_opmode_data - regulator operation mode data
 * @id: regulator id
 * @mode: regulator operation mode
 */
struct sm5010_opmode_data {
	int id;
	unsigned int mode;
};

/* SM5010 Buck2~PABUCK regulator ids */
enum sm5010_bucks {	
	SM5010_BUCK2,
	SM5010_BUCK3,
	SM5010_BUCK4,
	SM5010_BUCK5,
	SM5010_BUCK6,
	SM5010_PABUCK,
 
	SM5010_BUCKMAX,
};

/* SM5010 LDO regulator ids */
enum sm5010_ldos {
	SM5010_LDO1,
	SM5010_LDO2,
	SM5010_LDO3,
	SM5010_LDO4,
	SM5010_LDO5,
	SM5010_LDO6,
	SM5010_LDO7,
	SM5010_LDO8,
	SM5010_LDO9,
	SM5010_LDO10,
	SM5010_LDO11,
	SM5010_LDO12,
	SM5010_LDO13, 
	SM5010_LDO14,
	SM5010_LDO15,
	SM5010_LDO16,
	SM5010_LDO17,
	SM5010_LDO18,
	SM5010_LDO19,
	SM5010_LDO20,
 
	SM5010_LDOMAX,
};

#define SM5010_DVSPINS_MAX	3

/*
 * regulator operation mode
 * SM5010_OPMODE_OFF	Regulator always OFF
 * SM5010_OPMODE_nPWRSTM  Regulator is changed by nPWRSTM pin
 *							   If nPWRSTM is low to high, regulator is on
 *							   If nPWRSTM is high to low, regulator is off 
 * SM5010_OPMODE_ON		Regulator always ON
 * SM5010_OPMODE_ON_T	Regulator always ON
 */
enum sm5010_opmode {
	SM5010_OPMODE_OFF,
	SM5010_OPMODE_nPWRSTM,
	SM5010_OPMODE_ON,
	SM5010_OPMODE_ON_T,
};

/*
 * regulator lpm operation mode
 * SM5010_LPM_OPMODE_ON	Regulator always ON in lpm
 * SM5010_LPM_OPMODE_nPWRSTM  LPM is changed by nPWRSTM pin
 *							   	    If nPWRSTM is low to high, lpm is disabled
 *							   	    If nPWRSTM is high to low, lpm is enabled
 * SM5010_LPM_OPMODE_OFF		Regulator always ON in normal
 * SM5010_LPM_OPMODE_OFF_T	Regulator always ON in normal
 */
enum sm5010_lpm_opmode {
	SM5010_LPM_OPMODE_ON,
	SM5010_LPM_OPMODE_nPWRSTM,
	SM5010_LPM_OPMODE_OFF,
	SM5010_LPM_OPMODE_OFF_T,	
};

/*
 * regulator lpm operation mode
 * SM5010_BUCK_OPMODE_AUTOMODE			Regulator is the Auto-Mode
 * SM5010_BUCK_OPMODE_FORCEPWMMODE	Regulator is the forced PWM-Mode
 */
enum sm5010_pwm_opmode {
	SM5010_PWM_OPMODE_AUTOMODE,
	SM5010_PWM_OPMODE_FORCEPWMMODE,
};

#define SM5010_LPM_OPMODE_SHIFT		6
#define SM5010_LPM_OPMODE_MASK		(3 << SM5010_LPM_OPMODE_SHIFT)

#define SM5010_PWM_OPMODE_SHIFT		0
#define SM5010_PWM_OPMODE_MASK		(1 << SM5010_PWM_OPMODE_SHIFT)

/* BUCK2CNTL11 */
#define SM5010_ENSHEDDING_SHIFT		6
#define SM5010_ENSHEDDING_MASK		(1 << SM5010_ENSHEDDING_SHIFT)
#define SM5010_FREQ_SHIFT			5
#define SM5010_FREQ_MASK			(1 << SM5010_FREQ_SHIFT)
#define SM5010_DVSRAMPB1_SHIFT		3
#define SM5010_DVSRAMPB1_MASK		(3 << SM5010_DVSRAMPB1_SHIFT)
#define SM5010_ENRAMPB1DOWN_SHIFT	2
#define SM5010_ENRAMPB1DOWN_MASK	(1 << SM5010_ENRAMPB1DOWN_SHIFT)
#define SM5010_BUCK1ADISEN_SHIFT	1
#define SM5010_BUCK1ADISEN_MASK		(1 << SM5010_BUCK1ADISEN_SHIFT)
#define SM5010_BUCK1MODE_SHIFT		0
#define SM5010_BUCK1MODE_MASK		(1 << SM5010_BUCK1MODE_SHIFT)

#define SM5010_BUCK_MIN_0P6V			600000
#define SM5010_BUCK_MIN_0P5XV			562500
#define SM5010_BUCK_MIN_0P4V			400000
#define SM5010_LDO_MIN_0P8V				800000

#define SM5010_BUCK_MAX_1P2V			1200000

#define SM5010_BUCK_STEP_12P5MV			12500
#define SM5010_LDO_STEP_50P0MV			50000

#define SM5010_BUCK_NUM_48				48
#define SM5010_BUCK_NUM_51				51
#define SM5010_BUCK_NUM_112				112
#define SM5010_BUCK_NUM_255				255
#define SM5010_LDO_NUM_51				51

#define SM5010_BUCK_VSEL_MASK_3F		0x3F
#define SM5010_BUCK_VSEL_MASK_FF		0xFF
#define SM5010_LDO_VSEL_MASK_3F			0x3F

#define SM5010_BUCK_ENABLE_MASK_03		0x03
#define SM5010_BUCK_ENABLE_MASK_10		0x10
#define SM5010_LDO_ENABLE_MASK_03		0x03

#define SM5010_LDO_VSEL_MASK_03			0x03

#endif /*  __LINUX_REGULATOR_SM5010_H */
