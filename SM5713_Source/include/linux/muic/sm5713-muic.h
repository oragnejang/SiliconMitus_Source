/*
 * Copyright (C) 2015 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *
 */

#ifndef __SM5713_MUIC_H__
#define __SM5713_MUIC_H__


#include <linux/muic/muic.h>

#define MUIC_DEV_NAME	"muic-sm5713"


/* sm5713 muic register read/write related information defines. */

/* sm5713 Control register */
#define CTRL_nSW_OPEN_SHIFT				5
#define CTRL_ENDCDTIMER_SHIFT			2
#define CTRL_BC12OFF_SHIFT				1
#define CTRL_AUTOVBUS_CHECK_SHIFT		0

#define CTRL_nSW_OPEN_MASK				(0x1 << CTRL_nSW_OPEN_SHIFT)
#define CTRL_ENDCDTIMER_MASK			(0x1 << CTRL_ENDCDTIMER_SHIFT)
#define CTRL_BC12OFF_MASK				(0x1 << CTRL_BC12OFF_SHIFT)
#define CTRL_AUTOVBUS_CHECK_MASK		(0x1 << CTRL_AUTOVBUS_CHECK_SHIFT)

/* SM5713 MUIC Interrupt 1 register */
#define INT1_DPDM_OVP_SHIFT				5
#define INT1_VBUS_RID_DETACH_SHIFT		4
#define INT1_AUTOVBUS_CHECK_SHIFT		3
#define INT1_RID_DETECT_SHIFT			2
#define INT1_CHGTYPE_SHIFT				1
#define INT1_DCDTIMEOUT_SHIFT			0

#define INT1_DPDM_OVP_MASK				(0x1 << INT1_DPDM_OVP_SHIFT)
#define INT1_VBUS_RID_DETACH_MASK		(0x1 << INT1_VBUS_RID_DETACH_SHIFT)
#define INT1_AUTOVBUS_CHECK_MASK		(0x1 << INT1_AUTOVBUS_CHECK_SHIFT)
#define INT1_RID_DETECT_MASK			(0x1 << INT1_RID_DETECT_SHIFT)
#define INT1_CHGTYPE_MASK				(0x1 << INT1_CHGTYPE_SHIFT)
#define INT1_DCDTIMEOUT_MASK			(0x1 << INT1_DCDTIMEOUT_SHIFT)

/* SM5713 MUIC Interrupt 2 register */
#define INT2_AFC_ERROR_SHIFT			5
#define INT2_AFC_STA_CHG_SHIFT			4
#define INT2_MULTI_BYTE_SHIFT			3
#define INT2_VBUS_UPDATE_SHIFT			2
#define INT2_AFC_ACCEPTED_SHIFT			1
#define INT2_AFC_TA_ATTACHED_SHIFT		0

#define INT2_AFC_ERROR_MASK				(0x1 << INT2_AFC_ERROR_SHIFT)
#define INT2_AFC_STA_CHG_MASK			(0x1 << INT2_AFC_STA_CHG_SHIFT)
#define INT2_MULTI_BYTE_MASK			(0x1 << INT2_MULTI_BYTE_SHIFT)
#define INT2_VBUS_UPDATE_MASK			(0x1 << INT2_VBUS_UPDATE_SHIFT)
#define INT2_AFC_ACCEPTED_MASK			(0x1 << INT2_AFC_ACCEPTED_SHIFT)
#define INT2_AFC_TA_ATTACHED_MASK		(0x1 << INT2_AFC_TA_ATTACHED_SHIFT)


/* SM5713 ADC register */
//#define ADC_MASK				(0x1f)
//#define ADC_CONVERSION_MASK	(0x1 << 7)

/* SM5713 Timing Set 1 & 2 register Timing table */
//#define KEY_PRESS_TIME_100MS		(0x00)
//#define KEY_PRESS_TIME_200MS		(0x10)
//#define KEY_PRESS_TIME_300MS		(0x20)
//#define KEY_PRESS_TIME_700MS		(0x60)

//#define LONGKEY_PRESS_TIME_300MS	(0x00)
//#define LONGKEY_PRESS_TIME_500MS	(0x02)
//#define LONGKEY_PRESS_TIME_1000MS	(0x07)
//#define LONGKEY_PRESS_TIME_1500MS	(0x0C)

//#define SWITCHING_WAIT_TIME_10MS	(0x00)
//#define SWITCHING_WAIT_TIME_210MS	(0xa0)

/* SM5713 MUIC Device Type 1 register */
#define DEV_TYPE1_LO_TA			(0x1 << 7)
#define DEV_TYPE1_QC20_TA		(0x1 << 6)
#define DEV_TYPE1_AFC_TA		(0x1 << 5)
#define DEV_TYPE1_U200			(0x1 << 4)
#define DEV_TYPE1_CDP			(0x1 << 3)
#define DEV_TYPE1_DCP			(0x1 << 2)
#define DEV_TYPE1_SDP			(0x1 << 1)
#define DEV_TYPE1_DCD_OUT_SDP	(0x1 << 0)

#define DEV_TYPE1_USB_TYPES		(DEV_TYPE1_CDP | DEV_TYPE1_SDP)
#define DEV_TYPE1_CHG_TYPES		(DEV_TYPE1_DCP | DEV_TYPE1_CDP | DEV_TYPE1_U200)

/* SM5713 MUIC Device Type 2 register */
#define DEV_TYPE2_USB_OTG			(0x1 << 4)
#define DEV_TYPE2_JIG_UART_OFF		(0x1 << 3)
#define DEV_TYPE2_JIG_UART_ON		(0x1 << 2)
#define DEV_TYPE2_JIG_USB_OFF		(0x1 << 1)
#define DEV_TYPE2_JIG_USB_ON		(0x1 << 0)

#define DEV_TYPE2_JIG_USB_TYPES		(DEV_TYPE2_JIG_USB_OFF | DEV_TYPE2_JIG_USB_ON)
#define DEV_TYPE2_JIG_UART_TYPES	(DEV_TYPE2_JIG_UART_OFF)
#define DEV_TYPE2_JIG_TYPES			(DEV_TYPE2_JIG_UART_TYPES | DEV_TYPE2_JIG_USB_TYPES)

/* SM5713 MUIC Device Type 3 register */
//#define DEV_TYPE3_U200_CHG		(0x1 << 7)
//#define DEV_TYPE3_AV_WITH_VBUS	(0x1 << 4)
//#define DEV_TYPE3_VBUS_R255		(0x1 << 1)
//#define DEV_TYPE3_MHL			(0x1 << 0)
//#define DEV_TYPE3_CHG_TYPE		(DEV_TYPE3_U200_CHG | DEV_TYPE3_VBUS_R255)

/* SM5713 MUIC APPLE Device Type register */
//#define DEV_TYPE_APPLE_APPLE0P5A_CHG	(0x1 << 7)
//#define DEV_TYPE_APPLE_APPLE1A_CHG		(0x1 << 6)
//#define DEV_TYPE_APPLE_APPLE2A_CHG		(0x1 << 5)
//#define DEV_TYPE_APPLE_APPLE2P4A_CHG	(0x1 << 4)
//#define DEV_TYPE_APPLE_SDP_DCD_OUT		(0x1 << 3)
//#define DEV_TYPE_APPLE_RID_WAKEUP		(0x1 << 2)
//#define DEV_TYPE_APPLE_VBUS_WAKEUP		(0x1 << 1)
//#define DEV_TYPE_APPLE_BCV1P2_OR_OPEN	(0x1 << 0)

/* S2MU004 MUIC CHG Type register */
//#define CHG_TYPE_VBUS_R255	(0x1 << 7)
//#define DEV_TYPE_U200		(0x1 << 4)
//#define DEV_TYPE_SDP_1P8S	(0x1 << 3)
//#define DEV_TYPE_USB		(0x1 << 2)
//#define DEV_TYPE_CDPCHG		(0x1 << 1)
//#define DEV_TYPE_DCPCHG		(0x1 << 0)
//#define DEV_TYPE_CHG_TYPE		(CHG_TYPE_VBUS_R255 | DEV_TYPE_U200 | DEV_TYPE_SDP_1P8S)

//#define MANUAL_SW_JIG_EN		(0x1 << 0)

/* SM5713 AFC CTRL register */
#define AFCCTRL_QC20_9V			6
#define AFCCTRL_DIS_AFC			5
#define AFCCTRL_HVDCPTIMER		4
#define AFCCTRL_VBUS_READ		3
#define AFCCTRL_DM_RESET		2
#define AFCCTRL_DP_RESET		1
#define AFCCTRL_ENAFC			0


/*
 * Manual Switch
 * D- [5:3] / D+ [2:0]
 * 000: Open all / 001: USB / 010: AUDIO / 011: UART
 */
#define MANUAL_SW_DM_SHIFT		3
#define MANUAL_SW_DP_SHIFT		0

#define MANUAL_SW_OPEN			(0x0)
#define MANUAL_SW_USB			(0x1 << MANUAL_SW_DM_SHIFT | 0x1 << MANUAL_SW_DP_SHIFT)
#define MANUAL_SW_AUDIO			(0x2 << MANUAL_SW_DM_SHIFT | 0x2 << MANUAL_SW_DP_SHIFT)
#define MANUAL_SW_UART			(0x3 << MANUAL_SW_DM_SHIFT | 0x3 << MANUAL_SW_DP_SHIFT)


//#define MANUAL_SW_OTGEN		(0x1)
//#define MANUAL_SW_CHARGER	(0x1 << MANUAL_SW_CHG_SHIFT)

enum sm5713_reg_manual_sw_value {
	MANSW_OPEN				=	(MANUAL_SW_OPEN),
	MANSW_USB				=	(MANUAL_SW_USB),
	MANSW_AUDIO				=	(MANUAL_SW_AUDIO),
	MANSW_OTG				=	(MANUAL_SW_USB),
	MANSW_UART				=	(MANUAL_SW_UART),
	MANSW_OPEN_RUSTPROOF	=	(MANUAL_SW_OPEN),
};

enum sm5713_muic_mode {
	SM5713_NONE_CABLE,
	SM5713_FIRST_ATTACH,
	SM5713_SECOND_ATTACH,
	SM5713_MUIC_DETACH,
	SM5713_MUIC_OTG,
	SM5713_MUIC_JIG,
};

#define AFC_TXBYTE_5V		0x0
#define AFC_TXBYTE_6V		0x1
#define AFC_TXBYTE_7V		0x2
#define AFC_TXBYTE_8V		0x3
#define AFC_TXBYTE_9V		0x4
#define AFC_TXBYTE_10V		0x5
#define AFC_TXBYTE_11V		0x6
#define AFC_TXBYTE_12V		0x7
#define AFC_TXBYTE_13V		0x8
#define AFC_TXBYTE_14V		0x9
#define AFC_TXBYTE_15V		0xA
#define AFC_TXBYTE_16V		0xB
#define AFC_TXBYTE_17V		0xC
#define AFC_TXBYTE_18V		0xD
#define AFC_TXBYTE_19V		0xE
#define AFC_TXBYTE_20V		0xF

#define AFC_TXBYTE_0_75A	0x0
#define AFC_TXBYTE_0_90A	0x1
#define AFC_TXBYTE_1_05A	0x2
#define AFC_TXBYTE_1_20A	0x3
#define AFC_TXBYTE_1_35A	0x4
#define AFC_TXBYTE_1_50A	0x5
#define AFC_TXBYTE_1_65A	0x6
#define AFC_TXBYTE_1_80A	0x7
#define AFC_TXBYTE_1_95A	0x8
#define AFC_TXBYTE_2_10A	0x9
#define AFC_TXBYTE_2_25A	0xA
#define AFC_TXBYTE_2_40A	0xB
#define AFC_TXBYTE_2_55A	0xC
#define AFC_TXBYTE_2_70A	0xD
#define AFC_TXBYTE_2_85A	0xE
#define AFC_TXBYTE_3_00A	0xF

#define AFC_TXBYTE_5V_1_95A		((AFC_TXBYTE_5V << 4) | AFC_TXBYTE_1_95A);
#define AFC_TXBYTE_9V_1_65A		((AFC_TXBYTE_9V << 4) | AFC_TXBYTE_1_65A);
#define AFC_TXBYTE_12V_2_10A	((AFC_TXBYTE_12V << 4) | AFC_TXBYTE_2_10A);



#define AFC_RXBYTE_MAX	16

#define SM5713_MUIC_HV_5V	0x08
#define SM5713_MUIC_HV_9V	0x46
#define SM5713_MUIC_HV_12V	0x79

#define SM5713_ENQC20_5V    0x0
#define SM5713_ENQC20_9V    0x1
#define SM5713_ENQC20_12V   0x2


extern struct device *switch_device;
extern unsigned int system_rev;
extern struct muic_platform_data muic_pdata;

int sm5713_i2c_read_byte(struct i2c_client *client, u8 command);
int sm5713_i2c_write_byte(struct i2c_client *client, u8 command, u8 value);

int muic_disable_afc_state(void);
int muic_check_fled_state(bool enable, bool mode);
int muic_disable_afc(int disable);
int sm5713_muic_voltage_control(int afctxd, int qc20);

int sm5713_afc_ta_attach(muic_data_t *muic_data);
int sm5713_afc_ta_accept(muic_data_t *muic_data);
int sm5713_afc_vbus_update(muic_data_t *muic_data);
int sm5713_afc_multi_byte(muic_data_t *muic_data);
int sm5713_afc_error(muic_data_t *muic_data);
int sm5713_afc_sta_chg(muic_data_t *muic_data);

int sm5713_set_afc_ctrl_reg(muic_data_t *muic_data, int shift, bool on);
void sm5713_hv_muic_initialize(muic_data_t *muic_data);

#endif /* __SM5713_MUIC_H__ */
