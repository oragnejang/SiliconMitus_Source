/*
 * sm5011.h
 *
 * Copyright (c) 2018 SILICONMITUS COMPANY,LTD
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_SM5011_H
#define __LINUX_MFD_SM5011_H

/* SM5011 registers */
enum SM5011_reg {
	SM5011_REG_PWRONREG,	
	SM5011_REG_PWROFFREG,	
	SM5011_REG_REBOOTREG,	
	SM5011_REG_INT1, 		
	SM5011_REG_INT2, 		
	SM5011_REG_INTMSK1,		
	SM5011_REG_INTMSK2,	
	SM5011_REG_STATUS,	
	
	SM5011_REG_CNTL1 = 0x10,
	SM5011_REG_CNTL2,
	SM5011_REG_CNTL3,
	SM5011_REG_MRSTBCNTL,
	SM5011_REG_WDTCNTL,
							
	SM5011_REG_BUCK2CNTL1 = 0x2B, 
	SM5011_REG_BUCK2CNTL2,
	SM5011_REG_BUCK2CNTL3,
	SM5011_REG_BUCK3CNTL1,
	SM5011_REG_BUCK3CNTL2,
	SM5011_REG_BUCK3CNTL3,
	SM5011_REG_BUCK4CNTL1,
	SM5011_REG_BUCK4CNTL2,
	SM5011_REG_BUCK4CNTL3,
	SM5011_REG_BUCK5CNTL1,
	SM5011_REG_BUCK5CNTL2,
	SM5011_REG_BUCK5CNTL3,
	SM5011_REG_BUCK6CNTL1,
	SM5011_REG_BUCK6CNTL2,
	SM5011_REG_BUCK6CNTL3,
	SM5011_REG_LDO1CNTL1 = 0x3c, 
	SM5011_REG_LDO1CNTL2,
	SM5011_REG_LDO2CNTL1,
	SM5011_REG_LDO2CNTL2,
	SM5011_REG_LDO3CNTL1,
	SM5011_REG_LDO3CNTL2,
	SM5011_REG_LDO4CNTL1,
	SM5011_REG_LDO4CNTL2,
	SM5011_REG_LDO5CNTL1,
	SM5011_REG_LDO5CNTL2,
	SM5011_REG_LDO6CNTL1,
	SM5011_REG_LDO6CNTL2,
	SM5011_REG_LDO7CNTL1,
	SM5011_REG_LDO7CNTL2, 
	SM5011_REG_LDO8CNTL1,
	SM5011_REG_LDO8CNTL2, 
	SM5011_REG_LDO9CNTL1,
	SM5011_REG_LDO9CNTL2, 
	SM5011_REG_LDO10CNTL1,
	SM5011_REG_LDO10CNTL2, 
	SM5011_REG_LDO11CNTL1,	
	SM5011_REG_LDO11CNTL2,								
	SM5011_REG_LDO12CNTL1,	
	SM5011_REG_LDO12CNTL2,	
 	SM5011_REG_LDO13CNTL1,	
	SM5011_REG_LDO13CNTL2,	 
	SM5011_REG_LDO14CNTL1,	
	SM5011_REG_LDO14CNTL2,	
	SM5011_REG_LDO15CNTL1,	
	SM5011_REG_LDO15CNTL2,	 
	SM5011_REG_LDO16CNTL1,	
	SM5011_REG_LDO16CNTL2,	 
	SM5011_REG_LDO17CNTL1,	
	SM5011_REG_LDO17CNTL2,	 
	SM5011_REG_LDO18CNTL1,	
	SM5011_REG_LDO18CNTL2,	 
	SM5011_REG_LDO19CNTL1,	
	SM5011_REG_LDO19CNTL2,	 
	SM5011_REG_LDO20CNTL1,	
	SM5011_REG_LDO20CNTL2,	 
	SM5011_REG_COMPCNTL,	
							
	SM5011_REG_RTCCNTL1 = 0x70,	
	SM5011_REG_RTCCNTL2,	
	SM5011_REG_RTCCNTL3,	
	SM5011_REG_RTCCNTL4,	
	SM5011_REG_RTCCNTL5,	
	SM5011_REG_RTCCNTL6,	
	SM5011_REG_RTCCNTL7,	 
	SM5011_REG_RTCALM1CNTL1,
	SM5011_REG_RTCALM1CNTL2,
	SM5011_REG_RTCALM1CNTL3,
	SM5011_REG_RTCALM1CNTL4,
	SM5011_REG_RTCALM1CNTL5,
	SM5011_REG_RTCALM1CNTL6,
	SM5011_REG_RTCALM1CNTL7, 
	SM5011_REG_RTCALM2CNTL1,
	SM5011_REG_RTCALM2CNTL2,
	SM5011_REG_RTCALM2CNTL3,
	SM5011_REG_RTCALM2CNTL4,
	SM5011_REG_RTCALM2CNTL5,
	SM5011_REG_RTCALM2CNTL6,
	SM5011_REG_RTCALM2CNTL7, 
	SM5011_REG_SECCNTL1,	
	SM5011_REG_SECCNTL2,	
	SM5011_REG_SECCNTL3,	
	SM5011_REG_SECCNTL4,	
	SM5011_REG_AUTHCNTL1,	
	SM5011_REG_AUTHCNTL2,	
							
	SM5011_REG_DEVICEID = 0x90,

	SM5011_REG_MAX,
};

#define SM5011_MAST_REV_SHIFT		4
#define SM5011_MAST_REV_MASK		(0x0F << SM5011_MAST_REV_SHIFT)

#endif /*  __LINUX_MFD_SM5011_H */
