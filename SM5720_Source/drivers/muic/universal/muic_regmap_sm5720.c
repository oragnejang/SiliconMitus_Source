/*
 * muic_regmap_sm5720.c
 *
 * Copyright (C) 2014 Samsung Electronics
 * Thomas Ryu <smilesr.ryu@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/host_notify.h>
#include <linux/string.h>

#include <linux/muic/muic.h>

#if defined(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */

#if defined(CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

#include "muic-internal.h"
#include "muic_i2c.h"
#include "muic_regmap.h"
#include "muic_debug.h"


#include "muic_sm5720_afc.h"


#define ADC_DETECT_TIME_200MS (0x03)
#define KEY_PRESS_TIME_100MS  (0x00)

enum sm5720_muic_reg_init_value {
    REG_INTMASK1_VALUE     = (0x1C),
    REG_INTMASK2_VALUE     = (0x18),
    REG_INTMASK3AFC_VALUE  = (0x00),
    REG_TIMING1_VALUE      = (ADC_DETECT_TIME_200MS | KEY_PRESS_TIME_100MS),
    REG_RSVDID2_VALUE      = (0x24),
    REG_AFC_TXD_VALUE      = (0x79), /* 0x79 : 12V,2.1A */
};

/* sm5720 I2C registers */
enum sm5720_muic_reg {
    REG_DEVID    = 0x01,
    REG_CTRL     = 0x02,
    REG_INT1     = 0x03,
    REG_INT2     = 0x04,
    REG_INT3     = 0x05,
    REG_INTMASK1 = 0x06,
    REG_INTMASK2 = 0x07,
    REG_INTMASK3 = 0x08,
    REG_ADC      = 0x09,
    REG_DEVT1    = 0x0a,
    REG_DEVT2    = 0x0b,
    REG_DEVT3    = 0x0c,
    REG_TIMING1  = 0x0d,
    REG_TIMING2  = 0x0e,
    /* 0x0f is reserved. */
    REG_BTN1     = 0x10,
    REG_BTN2     = 0x11,
    REG_CarKit   = 0x12,
    REG_MANSW1   = 0x13,
    REG_MANSW2   = 0x14,
    REG_RSVDID1  = 0x15,
    REG_RSVDID2  = 0x16,
    REG_CHGTYPE  = 0x17,
    /* 0x18 ~ 0x23 for AFC  */
    REG_AFCCNTL     = 0x18,
    REG_AFCTXD      = 0x19,
    REG_AFCSTAT     = 0x1a,
    REG_VBUSVOL1    = 0x1b,
    REG_VBUSVOL2    = 0x1c,
    REG_AFCRXD1     = 0x1d,
    REG_AFCRXD2     = 0x1e,
    REG_AFCRXD3     = 0x1f,
    REG_AFCRXD4     = 0x20,
    REG_AFCRXD5     = 0x21,
    REG_AFCRXD6     = 0x22,
    REG_AFCTASTATUS = 0x23,
    REG_RESET       = 0x24,

    REG_END,
};

#define REG_ITEM(addr, bitp, mask) ((bitp<<16) | (mask<<8) | addr)

/* Field */
enum sm5720_muic_reg_item {
    DEVID_VendorID = REG_ITEM(REG_DEVID, _BIT0, _MASK3),

    CTRL_SW_OPEN    = REG_ITEM(REG_CTRL, _BIT4, _MASK1),
    CTRL_RAWDATA    = REG_ITEM(REG_CTRL, _BIT3, _MASK1),
    CTRL_ManualSW   = REG_ITEM(REG_CTRL, _BIT2, _MASK1),
    CTRL_WAIT       = REG_ITEM(REG_CTRL, _BIT1, _MASK1),
    CTRL_MASK_INT   = REG_ITEM(REG_CTRL, _BIT0, _MASK1),

    INT1_DETACH = REG_ITEM(REG_INT1, _BIT1, _MASK1),
    INT1_ATTACH = REG_ITEM(REG_INT1, _BIT0, _MASK1),

    INT2_VBUSDET_ON  = REG_ITEM(REG_INT2, _BIT7, _MASK1),
    INT2_RID_CHARGER = REG_ITEM(REG_INT2, _BIT6, _MASK1),
    INT2_MHL         = REG_ITEM(REG_INT2, _BIT5, _MASK1),
    INT2_ADC_CHG     = REG_ITEM(REG_INT2, _BIT2, _MASK1),
    INT2_VBUS_OFF    = REG_ITEM(REG_INT2, _BIT0, _MASK1),

    INT3_QC20_ACCEPTED    = REG_ITEM(REG_INT3, _BIT6, _MASK1),
    INT3_AFC_ERROR        = REG_ITEM(REG_INT3, _BIT5, _MASK1),
    INT3_AFC_STA_CHG      = REG_ITEM(REG_INT3, _BIT4, _MASK1),
    INT3_MULTI_BYTE       = REG_ITEM(REG_INT3, _BIT3, _MASK1),
    INT3_VBUS_UPDATE      = REG_ITEM(REG_INT3, _BIT2, _MASK1),
    INT3_AFC_ACCEPTED     = REG_ITEM(REG_INT3, _BIT1, _MASK1),
    INT3_AFC_TA_ATTACHED  = REG_ITEM(REG_INT3, _BIT0, _MASK1),

    INTMASK1_DETACH_M     = REG_ITEM(REG_INTMASK1, _BIT1, _MASK1),
    INTMASK1_ATTACH_M     = REG_ITEM(REG_INTMASK1, _BIT0, _MASK1),

    INTMASK2_VBUSDET_ON_M = REG_ITEM(REG_INTMASK2, _BIT7, _MASK1),
    INTMASK2_RID_CHARGERM = REG_ITEM(REG_INTMASK2, _BIT6, _MASK1),
    INTMASK2_MHL_M        = REG_ITEM(REG_INTMASK2, _BIT5, _MASK1),
    INTMASK2_ADC_CHG_M    = REG_ITEM(REG_INTMASK2, _BIT2, _MASK1),
    INTMASK2_REV_ACCE_M   = REG_ITEM(REG_INTMASK2, _BIT1, _MASK1),
    INTMASK2_VBUS_OFF_M   = REG_ITEM(REG_INTMASK2, _BIT0, _MASK1),

    INT3_QC20_ACCEPTED_M    = REG_ITEM(REG_INTMASK3, _BIT6, _MASK1),
    INT3_AFC_ERROR_M        = REG_ITEM(REG_INTMASK3, _BIT5, _MASK1),
    INT3_AFC_STA_CHG_M      = REG_ITEM(REG_INTMASK3, _BIT4, _MASK1),
    INT3_MULTI_BYTE_M       = REG_ITEM(REG_INTMASK3, _BIT3, _MASK1),
    INT3_VBUS_UPDATE_M      = REG_ITEM(REG_INTMASK3, _BIT2, _MASK1),
    INT3_AFC_ACCEPTED_M     = REG_ITEM(REG_INTMASK3, _BIT1, _MASK1),
    INT3_AFC_TA_ATTACHED_M  = REG_ITEM(REG_INTMASK3, _BIT0, _MASK1),

    ADC_ADC_VALUE  =  REG_ITEM(REG_ADC, _BIT0, _MASK6),

    DEVT1_USB_OTG         = REG_ITEM(REG_DEVT1, _BIT7, _MASK1),
    DEVT1_DEDICATED_CHG   = REG_ITEM(REG_DEVT1, _BIT6, _MASK1),
    DEVT1_USB_CHG         = REG_ITEM(REG_DEVT1, _BIT5, _MASK1),
    DEVT1_CAR_KIT_CHARGER = REG_ITEM(REG_DEVT1, _BIT4, _MASK1),
    DEVT1_UART            = REG_ITEM(REG_DEVT1, _BIT3, _MASK1),
    DEVT1_USB             = REG_ITEM(REG_DEVT1, _BIT2, _MASK1),
    DEVT1_AUDIO_TYPE2     = REG_ITEM(REG_DEVT1, _BIT1, _MASK1),
    DEVT1_AUDIO_TYPE1     = REG_ITEM(REG_DEVT1, _BIT0, _MASK1),

    DEVT2_AV              = REG_ITEM(REG_DEVT2, _BIT6, _MASK1),
    DEVT2_TTY             = REG_ITEM(REG_DEVT2, _BIT5, _MASK1),
    DEVT2_PPD             = REG_ITEM(REG_DEVT2, _BIT4, _MASK1),
    DEVT2_JIG_UART_OFF    = REG_ITEM(REG_DEVT2, _BIT3, _MASK1),
    DEVT2_JIG_UART_ON     = REG_ITEM(REG_DEVT2, _BIT2, _MASK1),
    DEVT2_JIG_USB_OFF     = REG_ITEM(REG_DEVT2, _BIT1, _MASK1),
    DEVT2_JIG_USB_ON      = REG_ITEM(REG_DEVT2, _BIT0, _MASK1),

    DEVT3_AFC_TA          = REG_ITEM(REG_DEVT3, _BIT7, _MASK1),
    DEVT3_U200_CHG        = REG_ITEM(REG_DEVT3, _BIT6, _MASK1),
    DEVT3_LO_TA           = REG_ITEM(REG_DEVT3, _BIT5, _MASK1),
    DEVT3_AV_CABLE_VBUS   = REG_ITEM(REG_DEVT3, _BIT4, _MASK1),
    DEVT3_DCD_OUT_SDP     = REG_ITEM(REG_DEVT3, _BIT2, _MASK1),
    DEVT3_QC20_TA         = REG_ITEM(REG_DEVT3, _BIT1, _MASK1),
    DEVT3_MHL             = REG_ITEM(REG_DEVT3, _BIT0, _MASK1),

    TIMING1_KEY_PRESS_T   = REG_ITEM(REG_TIMING1, _BIT4, _MASK4),
    TIMING1_ADC_DET_T     = REG_ITEM(REG_TIMING1, _BIT0, _MASK4),

    TIMING2_SW_WAIT_T     = REG_ITEM(REG_TIMING2, _BIT4, _MASK4),
    TIMING2_LONG_KEY_T    = REG_ITEM(REG_TIMING2, _BIT0, _MASK4),

    MANSW1_DM_CON_SW      = REG_ITEM(REG_MANSW1, _BIT5, _MASK3),
    MANSW1_DP_CON_SW      = REG_ITEM(REG_MANSW1, _BIT2, _MASK3),

    MANSW2_JIG_ON         = REG_ITEM(REG_MANSW2, _BIT2, _MASK1),
    MANSW2_SINGLE_MODE    = REG_ITEM(REG_MANSW2, _BIT1, _MASK1),
    MANSW2_ID_SW          = REG_ITEM(REG_MANSW2, _BIT0, _MASK1),

    RSVDID1_VBUS_VALID    = REG_ITEM(REG_RSVDID1, _BIT0, _MASK1),
    
    RSVDID2_CHGPUMP_nEN   = REG_ITEM(REG_RSVDID2, _BIT5, _MASK1),
    RSVDID2_BCD_RESCAN    = REG_ITEM(REG_RSVDID2, _BIT4, _MASK1),    
    RSVDID2_VDP_SRC_EN    = REG_ITEM(REG_RSVDID2, _BIT3, _MASK1),        
    RSVDID2_DCD_TIME_EN   = REG_ITEM(REG_RSVDID2, _BIT2, _MASK1),    
    RSVDID2_DCD_TIMER     = REG_ITEM(REG_RSVDID2, _BIT1, _MASK1),
    
    CHGTYPE_CHG_TYPE      = REG_ITEM(REG_CHGTYPE, _BIT0, _MASK5),

    AFCCNTL_ENQC20        = REG_ITEM(REG_AFCCNTL, _BIT6, _MASK2),
    AFCCNTL_DISAFC        = REG_ITEM(REG_AFCCNTL, _BIT5, _MASK1),
    AFCCNTL_VBUS_READ     = REG_ITEM(REG_AFCCNTL, _BIT3, _MASK1),
    AFCCNTL_DM_RESET      = REG_ITEM(REG_AFCCNTL, _BIT2, _MASK1),
    AFCCNTL_DP_RESET      = REG_ITEM(REG_AFCCNTL, _BIT1, _MASK1),
    AFCCNTL_ENAFC         = REG_ITEM(REG_AFCCNTL, _BIT0, _MASK1),

    AFCTXD_VOLTAGE  = REG_ITEM(REG_AFCTXD, _BIT4, _MASK4),
    AFCTXD_CURRENT  = REG_ITEM(REG_AFCTXD, _BIT0, _MASK4),
    
    AFCSTAT_STATUS  = REG_ITEM(REG_AFCSTAT, _BIT0, _MASK8),
    
    VBUS_VOLTAGE1   = REG_ITEM(REG_VBUSVOL1, _BIT0, _MASK5),
    VBUS_VOLTAGE2   = REG_ITEM(REG_VBUSVOL2, _BIT0, _MASK8),

    AFCRXD1_DATA    = REG_ITEM(REG_AFCRXD1, _BIT0, _MASK8),
    AFCRXD2_DATA    = REG_ITEM(REG_AFCRXD2, _BIT0, _MASK8),
    AFCRXD3_DATA    = REG_ITEM(REG_AFCRXD3, _BIT0, _MASK8),
    AFCRXD4_DATA    = REG_ITEM(REG_AFCRXD4, _BIT0, _MASK8),
    AFCRXD5_DATA    = REG_ITEM(REG_AFCRXD5, _BIT0, _MASK8),
    AFCRXD6_DATA    = REG_ITEM(REG_AFCRXD6, _BIT0, _MASK8),
    
    AFCTASTATUS_DATA  = REG_ITEM(REG_AFCTASTATUS, _BIT0, _MASK1),

    RESET_RESET = REG_ITEM(REG_RESET, _BIT0, _MASK1),
};

/* sm5720 Control register */
#define CTRL_SWITCH_OPEN_SHIFT  4
#define CTRL_RAW_DATA_SHIFT     3
#define CTRL_MANUAL_SW_SHIFT    2
#define CTRL_WAIT_SHIFT         1
#define CTRL_INT_MASK_SHIFT     0

#define CTRL_SWITCH_OPEN_MASK   (0x1 << CTRL_SWITCH_OPEN_SHIFT)
#define CTRL_RAW_DATA_MASK      (0x1 << CTRL_RAW_DATA_SHIFT)
#define CTRL_MANUAL_SW_MASK     (0x1 << CTRL_MANUAL_SW_SHIFT)
#define CTRL_WAIT_MASK          (0x1 << CTRL_WAIT_SHIFT)
#define CTRL_INT_MASK_MASK      (0x1 << CTRL_INT_MASK_SHIFT)
#define CTRL_MASK               (CTRL_SWITCH_OPEN_MASK | CTRL_RAW_DATA_MASK | \
                                 CTRL_MANUAL_SW_MASK | CTRL_WAIT_MASK )

/* SM5720 Device Type 3 register */
#define DEV_TYPE3_AFC_TA      (0x1 << 7)
#define DEV_TYPE3_QC20_TA     (0x1 << 1)

#define ENQC20_NONE  0x0
#define ENQC20_9V    0x1
#define ENQC20_12V   0x2
#define ENQC20_20V   0x3

struct reg_value_set {
    int value;
    char *alias;
};

/* ADC Scan Mode Values : b'1 */
static struct reg_value_set sm5720_adc_scanmode_tbl[] = {
    [ADC_SCANMODE_CONTINUOUS] = {0x01, "Periodic"},
    [ADC_SCANMODE_ONESHOT]    = {0x00, "Oneshot."},
    [ADC_SCANMODE_PULSE]      = {0x00, "Oneshot.."},
};

/*
 * Manual Switch
 * D- [7:5] / D+ [4:2] / Reserved [1:0]
 * 000: Open all / 001: USB / 010: AUDIO / 011: UART / 100:USB_CP / 101: UART_CP
 * No Vbus switching in SM5720
 */
#define _D_OPEN   (0x0)
#define _D_USB    (0x1)
#define _D_AUDIO  (0x2)
#define _D_UART   (0x3)
//#define _V_OPEN     (0x0)
//#define _V_CHARGER  (0x1)
//#define _V_MIC      (0x2)

/* COM patch Values */
#define COM_VALUE(dm) ((dm<<5) | (dm<<2))

#define _COM_OPEN       COM_VALUE(_D_OPEN)
//#define _COM_OPEN_WITH_V_BUS  COM_VALUE(_D_OPEN, _V_CHARGER)
#define _COM_UART_AP    COM_VALUE(_D_UART)
#define _COM_UART_CP    _COM_UART_AP
#define _COM_USB_AP     COM_VALUE(_D_USB)
#define _COM_USB_CP     _COM_USB_AP
#define _COM_AUDIO      COM_VALUE(_D_AUDIO)

static int sm5720_com_value_tbl[] = {
    [COM_OPEN]      = _COM_OPEN,
//  [COM_OPEN_WITH_V_BUS]   = _COM_OPEN_WITH_V_BUS,
    [COM_UART_AP]   = _COM_UART_AP,
    [COM_UART_CP]   = _COM_UART_CP,
    [COM_USB_AP]    = _COM_USB_AP,
    [COM_USB_CP]    = _COM_USB_CP,
    [COM_AUDIO]     = _COM_AUDIO,
};

#define REG_CTRL_INITIAL (CTRL_MASK)

static regmap_t sm5720_muic_regmap_table[] = {
    [REG_DEVID]       = {"DeviceID",      0x01, 0x00, INIT_NONE},
    [REG_CTRL]        = {"CONTROL",       0x1E, 0x00, REG_CTRL_INITIAL,},
    [REG_INT1]        = {"INT1",          0x00, 0x00, INIT_NONE,},
    [REG_INT2]        = {"INT2",          0x00, 0x00, INIT_NONE,},
    [REG_INT3]        = {"INT3_AFC",      0x00, 0x00, INIT_NONE,},
    [REG_INTMASK1]    = {"INTMASK1",      0x00, 0x00, REG_INTMASK1_VALUE,},
    [REG_INTMASK2]    = {"INTMASK2",      0x00, 0x00, REG_INTMASK2_VALUE,},
    [REG_INTMASK3]    = {"INTMASK3_AFC",  0x00, 0x00, REG_INTMASK3AFC_VALUE,},
    [REG_ADC]         = {"ADC",           0x1F, 0x00, INIT_NONE,},
    [REG_DEVT1]       = {"DEVICETYPE1",   0x00, 0x00, INIT_NONE,},
    [REG_DEVT2]       = {"DEVICETYPE2",   0x00, 0x00, INIT_NONE,},
    [REG_DEVT3]       = {"DEVICETYPE3",   0x00, 0x00, INIT_NONE,},
    [REG_TIMING1]     = {"TimingSet1",    0x00, 0x00, REG_TIMING1_VALUE,},
    [REG_TIMING2]     = {"TimingSet2",    0x00, 0x00, INIT_NONE,},
    /* 0x0F: Reserved */
    [REG_BTN1]        = {"BUTTON1",       0x00, 0x00, INIT_NONE,},
    [REG_BTN2]        = {"BUTTON2",       0x00, 0x00, INIT_NONE,},
    [REG_CarKit]      = {"CarKitStatus",  0x00, 0x00, INIT_NONE,},
    [REG_MANSW1]      = {"ManualSW1",     0x00, 0x00, INIT_NONE,},
    [REG_MANSW2]      = {"ManualSW2",     0x00, 0x00, INIT_NONE,},
    [REG_RSVDID1]     = {"Reserved_ID1",  0x00, 0x00, INIT_NONE,},
    [REG_RSVDID2]     = {"Reserved_ID2",  0x24, 0x00, REG_RSVDID2_VALUE,},
    [REG_CHGTYPE]     = {"REG_CHG_TYPE",  0x00, 0x00, INIT_NONE,},
    /* 0x18 ~ 0x23: AFC */
    [REG_AFCCNTL]     = {"AFC_CNTL",      0x00, 0x00, INIT_NONE,},
    [REG_AFCTXD]      = {"AFC_TXD",       0x46, 0x00, REG_AFC_TXD_VALUE,}, /* 0x46 : 9V / 1.65A , 0x79 : 12V,2.1A */
    [REG_AFCSTAT]     = {"AFC_STATUS",    0x00, 0x00, INIT_NONE,},
    [REG_VBUSVOL1]    = {"VBUS_STATUS",   0x00, 0x00, INIT_NONE,},
    [REG_VBUSVOL2]    = {"VBUS_STATUS",   0x00, 0x00, INIT_NONE,},
    [REG_AFCRXD1]     = {"AFC_RXD1",      0x00, 0x00, INIT_NONE,},
    [REG_AFCRXD2]     = {"AFC_RXD2",      0x00, 0x00, INIT_NONE,},
    [REG_AFCRXD3]     = {"AFC_RXD3",      0x00, 0x00, INIT_NONE,},
    [REG_AFCRXD4]     = {"AFC_RXD4",      0x00, 0x00, INIT_NONE,},
    [REG_AFCRXD5]     = {"AFC_RXD5",      0x00, 0x00, INIT_NONE,},
    [REG_AFCRXD6]     = {"AFC_RXD6",      0x00, 0x00, INIT_NONE,},
    [REG_AFCTASTATUS] = {"AFC_TA_STATUS", 0x00, 0x00, INIT_NONE,},
    [REG_RESET]       = {"RESET",         0x00, 0x00, INIT_NONE,},
    [REG_END]         = {NULL, 0, 0, INIT_NONE},
};

static int sm5720_muic_ioctl(struct regmap_desc *pdesc,
        int arg1, int *arg2, int *arg3)
{
    int ret = 0;

    switch (arg1) {
    case GET_COM_VAL:
        *arg2 = sm5720_com_value_tbl[*arg2];
        *arg3 = REG_MANSW1;
        break;
    case GET_CTLREG:
        *arg3 = REG_CTRL;
        break;

    case GET_ADC:
        *arg3 = ADC_ADC_VALUE;
        break;

    case GET_SWITCHING_MODE:
        *arg3 = CTRL_ManualSW;
        break;

    case GET_INT_MASK:
        *arg3 = CTRL_MASK_INT;
        break;

    case GET_REVISION:
        *arg3 = DEVID_VendorID;
        break;

    case GET_OTG_STATUS:
        *arg3 = INTMASK2_VBUS_OFF_M;
        break;

    case GET_CHGTYPE:
        *arg3 = CHGTYPE_CHG_TYPE;
        break;

    case GET_RESID3:
        *arg3 = RSVDID2_BCD_RESCAN;
        break;

    default:
        ret = -1;
        break;
    }

    if (pdesc->trace) {
        pr_info("%s: ret:%d arg1:%x,", __func__, ret, arg1);

        if (arg2)
            pr_info(" arg2:%x,", *arg2);

        if (arg3)
            pr_info(" arg3:%x - %s", *arg3,
                regmap_to_name(pdesc, _ATTR_ADDR(*arg3)));
        pr_info("\n");
    }

    return ret;
}

static int sm5720_attach_ta(struct regmap_desc *pdesc)
{
    pr_info("%s\n", __func__);
    return 0;
}

static int sm5720_detach_ta(struct regmap_desc *pdesc)
{
    pr_info("%s\n", __func__);
    return 0;
}

static int sm5720_set_rustproof(struct regmap_desc *pdesc, int op)
{
    int attr = 0, value = 0, ret = 0;

    pr_info("%s\n", __func__);

    do {
        attr = MANSW2_JIG_ON;
        value = op ? 1 : 0;
        ret = regmap_write_value(pdesc, attr, value);
        if (ret < 0) {
            pr_err("%s MANSW2_JIG_ON write fail.\n", __func__);
            break;
        }

        _REGMAP_TRACE(pdesc, 'w', ret, attr, value);

        attr = CTRL_ManualSW;
        value = op ? 0 : 1;
        ret = regmap_write_value(pdesc, attr, value);
        if (ret < 0) {
            pr_err("%s CTRL_ManualSW write fail.\n", __func__);
            break;
        }

        _REGMAP_TRACE(pdesc, 'w', ret, attr, value);

    } while (0);

    return ret;
}

static int sm5720_get_vps_data(struct regmap_desc *pdesc, void *pbuf)
{
    muic_data_t *pmuic = pdesc->muic;
    vps_data_t *pvps = (vps_data_t *)pbuf;
    int attr;

    if (pdesc->trace)
        pr_info("%s\n", __func__);

    *(u8 *)&pvps->s.val1 = muic_i2c_read_byte(pmuic->i2c, REG_DEVT1);
    *(u8 *)&pvps->s.val2 = muic_i2c_read_byte(pmuic->i2c, REG_DEVT2);
    *(u8 *)&pvps->s.val3 = muic_i2c_read_byte(pmuic->i2c, REG_DEVT3);

    attr = RSVDID1_VBUS_VALID;
    *(u8 *)&pvps->s.vbvolt = regmap_read_value(pdesc, attr);

    attr = ADC_ADC_VALUE;
    *(u8 *)&pvps->s.adc = regmap_read_value(pdesc, attr);

    return 0;
}

static int sm5720_get_adc_scan_mode(struct regmap_desc *pdesc)
{
    struct reg_value_set *pvset;
    int attr, value, mode = 0;

    attr = MANSW2_SINGLE_MODE;
    value = regmap_read_value(pdesc, attr);

    for ( ; mode <ARRAY_SIZE(sm5720_adc_scanmode_tbl); mode++) {
        pvset = &sm5720_adc_scanmode_tbl[mode];
        if (pvset->value == value)
            break;
    }

    pr_info("%s: [%2d]=%02x,%02x\n", __func__, mode, value, pvset->value);
    pr_info("%s:       %s\n", __func__, pvset->alias);

    return mode;
}

static void sm5720_set_adc_scan_mode(struct regmap_desc *pdesc,
        const int mode)
{
    struct reg_value_set *pvset;
    int attr, ret, value;

    if (mode > ADC_SCANMODE_PULSE) {
        pr_err("%s Out of range(%d).\n", __func__, mode);
        return;
    }

    pvset = &sm5720_adc_scanmode_tbl[mode];
    pr_info("%s: [%2d] %s\n", __func__, mode, pvset->alias);

    do {
        value = pvset->value;
        attr = MANSW2_SINGLE_MODE;
        ret = regmap_write_value(pdesc, attr, value);
        if (ret < 0) {
            pr_err("%s MANSW2_SINGLE_MODE write fail.\n", __func__);
            break;
        }

        _REGMAP_TRACE(pdesc, 'w', ret, attr, value);

#define _ENABLE_PERIODIC_SCAN (0)
#define _DISABLE_PERIODIC_SCAN (1)

        attr = CTRL_RAWDATA;
        if (mode == ADC_SCANMODE_CONTINUOUS)
            value = _ENABLE_PERIODIC_SCAN;
        else
            value = _DISABLE_PERIODIC_SCAN;

        ret = regmap_write_value(pdesc, attr, value);
        if (ret < 0) {
            pr_err("%s CTRL_RAWDATA write fail.\n", __func__);
            break;
        }

        _REGMAP_TRACE(pdesc, 'w', ret, attr, value);

        } while (0);
}

enum switching_mode_val{
    _SWMODE_AUTO =1,
    _SWMODE_MANUAL =0,
};

static int sm5720_get_switching_mode(struct regmap_desc *pdesc)
{
    int attr, value, mode;

    pr_info("%s\n",__func__);

    attr = CTRL_ManualSW;
    value = regmap_read_value(pdesc, attr);

    mode = (value == _SWMODE_AUTO) ? SWMODE_AUTO : SWMODE_MANUAL;

    return mode;
}

static void sm5720_set_switching_mode(struct regmap_desc *pdesc, int mode)
{
        int attr, value;
    int ret = 0;

    pr_info("%s\n",__func__);

    value = (mode == SWMODE_AUTO) ? _SWMODE_AUTO : _SWMODE_MANUAL;
    attr = CTRL_ManualSW;
    ret = regmap_write_value(pdesc, attr, value);
    if (ret < 0)
        pr_err("%s REG_CTRL write fail.\n", __func__);
    else
        _REGMAP_TRACE(pdesc, 'w', ret, attr, value);
}

static int sm5720_run_BCD_rescan(struct regmap_desc *pdesc, int value)
{
    int attr, ret;

    attr = RSVDID2_BCD_RESCAN;

    ret = regmap_write_value(pdesc, attr, value);
    if (ret < 0)
        pr_err("%s BCD_RESCAN write fail.\n", __func__);
    else
        _REGMAP_TRACE(pdesc, 'w', ret, attr, value);

    return ret;
}

static void sm5720_get_fromatted_dump(struct regmap_desc *pdesc, char *mesg)
{
    muic_data_t *muic = pdesc->muic;
    int val;

    if (pdesc->trace)
        pr_info("%s\n", __func__);

    val = i2c_smbus_read_byte_data(muic->i2c, REG_CTRL);
    sprintf(mesg, "CT:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_INTMASK1);
    sprintf(mesg+strlen(mesg), "IM1:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_INTMASK2);
    sprintf(mesg+strlen(mesg), "IM2:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_INTMASK3);
    sprintf(mesg+strlen(mesg), "IM3:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_MANSW1);
    sprintf(mesg+strlen(mesg), "MS1:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_MANSW2);
    sprintf(mesg+strlen(mesg), "MS2:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_ADC);
    sprintf(mesg+strlen(mesg), "ADC:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_DEVT1);
    sprintf(mesg+strlen(mesg), "DT1:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_DEVT2);
    sprintf(mesg+strlen(mesg), "DT2:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_DEVT3);
    sprintf(mesg+strlen(mesg), "DT3:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_RSVDID1);
    sprintf(mesg+strlen(mesg), "RS1:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_RSVDID2);
    sprintf(mesg+strlen(mesg), "RS2:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_CHGTYPE);
    sprintf(mesg+strlen(mesg), "CTY:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_AFCCNTL);
    sprintf(mesg+strlen(mesg), "AFC:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_AFCTXD);
    sprintf(mesg+strlen(mesg), "TXD:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_AFCSTAT);
    sprintf(mesg+strlen(mesg), "AST:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_VBUSVOL1);
    sprintf(mesg+strlen(mesg), "VO1:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_VBUSVOL2);
    sprintf(mesg+strlen(mesg), "VO2:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, REG_AFCTASTATUS);
    sprintf(mesg+strlen(mesg), "ATT:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, 0x3C);
    sprintf(mesg+strlen(mesg), "H3C:%x ", val);
    val = i2c_smbus_read_byte_data(muic->i2c, 0x3D);
    sprintf(mesg+strlen(mesg), "H3D:%x ", val);
}

static int sm5720_get_sizeof_regmap(void)
{
    pr_info("%s:%s\n", MUIC_DEV_NAME, __func__);
    return (int)ARRAY_SIZE(sm5720_muic_regmap_table);
}

static int sm5720_set_afc_ctrl_reg(struct regmap_desc *pdesc, int shift, bool on)
{
    muic_data_t *pmuic = pdesc->muic;
    struct i2c_client *i2c = pmuic->i2c;
    u8 reg_val;
    int ret = 0;

    pr_info("%s: Register[%d], set [%d]\n", __func__, shift, on);
    ret = muic_i2c_read_byte(i2c, REG_AFCCNTL);
    if (ret < 0)
        printk(KERN_ERR "[muic] %s(%d)\n", __func__, ret);
    if (on)
        reg_val = ret | (0x1 << shift);
    else
        reg_val = ret & ~(0x1 << shift);

    if (reg_val ^ ret) {
        printk(KERN_DEBUG "[muic] %s reg_val(0x%x)!=AFC_CTRL reg(0x%x), update reg\n",
            __func__, reg_val, ret);

        ret = muic_i2c_write_byte(i2c, REG_AFCCNTL,
                reg_val);
        if (ret < 0)
            printk(KERN_ERR "[muic] %s err write AFC_CTRL(%d)\n",
                    __func__, ret);
    } else {
        printk(KERN_DEBUG "[muic] %s (0x%x), just return\n",
                __func__, ret);
        return 0;
    }

    ret = muic_i2c_read_byte(i2c, REG_AFCCNTL);
    if (ret < 0)
        printk(KERN_ERR "[muic] %s err read AFC_CTRL(%d)\n",
            __func__, ret);
    else
        printk(KERN_DEBUG "[muic] %s AFC_CTRL reg after change(0x%x)\n",
            __func__, ret);

    return ret;
}

static int sm5720_afc_ta_attach(struct regmap_desc *pdesc)
{
    muic_data_t *pmuic = pdesc->muic;
    struct i2c_client *i2c = pmuic->i2c;
    int ret, value;

    pr_info("%s:%s AFC_TA_ATTACHED \n",MUIC_DEV_NAME, __func__);

    // read clear : AFC_STATUS
    value = muic_i2c_read_byte(i2c, REG_AFCSTAT);
    if (value < 0)
        printk(KERN_ERR "%s: err read AFC_STATUS %d\n", __func__, value);
    pr_info("%s:%s AFC_STATUS [0x%02x]\n",MUIC_DEV_NAME, __func__, value);

    pmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC;
    muic_notifier_attach_attached_dev(pmuic->attached_dev);

	cancel_delayed_work(&pmuic->afc_retry_work);
	schedule_delayed_work(&pmuic->afc_retry_work, msecs_to_jiffies(5000)); // 5sec
    pr_info("%s: afc_retry_work(ATTACH) start \n", __func__);

    msleep(120); // 120ms delay

    // voltage(9.0V)  + current(1.65A) setting : 0x46
    // voltage(12.0V) + current(2.1A) setting : 0x79
    value = 0x79;
    ret = muic_i2c_write_byte(i2c, REG_AFCTXD, value);
    if (ret < 0)
        printk(KERN_ERR "[muic] %s: err write AFC_TXD(%d)\n", __func__, ret);
    pr_info("%s:%s AFC_TXD [0x%02x]\n",MUIC_DEV_NAME, __func__, value);

    // ENAFC set '1'
    sm5720_set_afc_ctrl_reg(pdesc, AFCCTRL_ENAFC, 1);
    pr_info("%s:%s AFCCTRL_ENAFC 1 \n",MUIC_DEV_NAME, __func__);
    pmuic->afc_retry_count = 0;
    pmuic->afc_vbus_retry_count = 0;
    pmuic->qc20_vbus = ENQC20_NONE;
    return 0;
}

static int sm5720_afc_ta_accept(struct regmap_desc *pdesc)
{
    muic_data_t *pmuic = pdesc->muic;
    struct i2c_client *i2c = pmuic->i2c;
    int device_type3;

    // ENAFC set '0'
    sm5720_set_afc_ctrl_reg(pdesc, AFCCTRL_ENAFC, 0);
    pr_info("%s:%s AFCCTRL_ENAFC 0 \n",MUIC_DEV_NAME, __func__);

    pr_info("%s:%s AFC_ACCEPTED \n",MUIC_DEV_NAME, __func__);

    cancel_delayed_work(&pmuic->afc_retry_work);
    pr_info("%s: afc_retry_work(ACCEPTED) cancel \n", __func__);

    device_type3 = muic_i2c_read_byte(i2c, REG_DEVT3);
    pr_info("%s: dev3 [0x%02x]\n",MUIC_DEV_NAME, device_type3);
    if (device_type3 & DEV_TYPE3_AFC_TA) {
        // VBUS_READ
        sm5720_set_afc_ctrl_reg(pdesc, AFCCTRL_VBUS_READ, 1);
        pr_info("%s:%s VBUS READ start(AFC)\n",MUIC_DEV_NAME, __func__);
        if (pmuic->attached_dev != ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC) {
            pmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC;
            muic_notifier_attach_attached_dev(pmuic->attached_dev);
        }
        pmuic->afc_vbus_retry_count++;
    } else {
        pmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_5V_MUIC;
        muic_notifier_attach_attached_dev(pmuic->attached_dev);
        pr_info("%s:%s  attached_dev(%d) \n",MUIC_DEV_NAME, __func__, pmuic->attached_dev);
    }
    
    return 0;

}

static int sm5720_afc_vbus_update(struct regmap_desc *pdesc)
{
    muic_data_t *pmuic = pdesc->muic;
    struct i2c_client *i2c = pmuic->i2c;
    int vbus_txd_voltage;
    int vbus_voltage, voltage1, voltage2;
    int voltage_min,voltage_max;
    int device_type3;
    int ret,reg_val;

    pr_info("%s:%s AFC_VBUS_UPDATE \n",MUIC_DEV_NAME, __func__);

    if (pmuic->attached_dev == ATTACHED_DEV_NONE_MUIC) {
        pr_info("%s:%s Device type is None\n",MUIC_DEV_NAME, __func__);
        return 0;
    }

    voltage1 = muic_i2c_read_byte(i2c, REG_VBUSVOL1);
    voltage2 = muic_i2c_read_byte(i2c, REG_VBUSVOL2);
    vbus_voltage = voltage1*1000 + (voltage2*3900)/1000;
    
    pr_info("%s: voltage1=[0x%02x], voltage2=[0x%02x], vbus_voltage=%d mV \n",
        MUIC_DEV_NAME, voltage1,voltage2,vbus_voltage);

    device_type3 = muic_i2c_read_byte(i2c, REG_DEVT3);
    pr_info("%s:%s DEVICE_TYPE3 [0x%02x]\n",MUIC_DEV_NAME, __func__, device_type3);
    if (device_type3 & DEV_TYPE3_QC20_TA){ // QC20_TA vbus update
        if (pmuic->qc20_vbus == ENQC20_12V){
            voltage_min  = 10000; // - 10000mV
            voltage_max  = 13000; // + 13000mV
        } else if (pmuic->qc20_vbus == ENQC20_9V){
            voltage_min  = 7000;  // - 7000mV
            voltage_max  = 10000; // + 10000mV
        } 
        
        pr_info("%s:%s QC20 vbus_voltage:%d mV (%d)\n"
                ,MUIC_DEV_NAME, __func__,vbus_voltage, pmuic->qc20_vbus );

        if ( (voltage_min <= vbus_voltage) && (vbus_voltage <= voltage_max) ) { /* AFC DONE */
            if (pmuic->qc20_vbus == ENQC20_12V){
                pmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_9V_MUIC;
                muic_notifier_attach_attached_dev(pmuic->attached_dev);
                pr_info("%s:%s QC20 12V (%d)\n",MUIC_DEV_NAME, __func__,pmuic->attached_dev);
            } else if(pmuic->qc20_vbus == ENQC20_9V){
                pmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_9V_MUIC;
                muic_notifier_attach_attached_dev(pmuic->attached_dev);
                pr_info("%s:%s QC20 9V (%d)\n",MUIC_DEV_NAME, __func__,pmuic->attached_dev);
            }
        } else { // vbus fail
            if (pmuic->qc20_vbus == ENQC20_12V){ // 9V retry
                if ( pmuic->afc_vbus_retry_count < 3 ) {
                    msleep(100);
                    pmuic->afc_vbus_retry_count++;
                    
                    sm5720_set_afc_ctrl_reg(pdesc, AFCCTRL_VBUS_READ, 1);
                    pr_info("%s:%s [QC20-12V] VBUS READ retry = %d \n",MUIC_DEV_NAME, __func__,
                            pmuic->afc_vbus_retry_count);
                } else {
                    msleep(100);
                    pmuic->qc20_vbus = ENQC20_9V;
                    ret = muic_i2c_read_byte(i2c, REG_AFCCNTL);
                    reg_val = (ret & 0x3F) | (ENQC20_9V<<6);    // QC20 9V
                    muic_i2c_write_byte(i2c, REG_AFCCNTL, reg_val);
                    pr_info("%s:%s read REG_AFCCNTL=0x%x ,  write REG_AFCCNTL=0x%x , qc20_vbus=%d \n",
                        MUIC_DEV_NAME, __func__, ret, reg_val, pmuic->qc20_vbus );
                    
                    // VBUS_READ
                    sm5720_set_afc_ctrl_reg(pdesc, AFCCTRL_VBUS_READ, 1);
                    pr_info("%s:%s VBUS READ start(QC20-9V)\n",MUIC_DEV_NAME, __func__);
                    pmuic->afc_vbus_retry_count = 0;
                    return 0;
                }
            } else {
                if ( pmuic->afc_vbus_retry_count < 3 ) {
                    msleep(100);
                    pmuic->afc_vbus_retry_count++;
                    
                    sm5720_set_afc_ctrl_reg(pdesc, AFCCTRL_VBUS_READ, 1);
                    pr_info("%s:%s [QC20-9V] VBUS READ retry = %d \n",MUIC_DEV_NAME, __func__,
                            pmuic->afc_vbus_retry_count);
                } else {
                    pmuic->qc20_vbus = ENQC20_NONE;
                    ret = muic_i2c_read_byte(i2c, REG_AFCCNTL);
                    reg_val = (ret & 0x3F) | (ENQC20_NONE<<6);    // QC20 noce
                    muic_i2c_write_byte(i2c, REG_AFCCNTL, reg_val);
                    pr_info("%s:%s read REG_AFCCNTL=0x%x ,  write REG_AFCCNTL=0x%x , qc20_vbus=%d \n",
                        MUIC_DEV_NAME, __func__, ret, reg_val, pmuic->qc20_vbus );
                
                    pmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_5V_MUIC;
                    muic_notifier_attach_attached_dev(pmuic->attached_dev);
                }
            }
        }
        
    } else { // AFC vbus update
        vbus_txd_voltage = muic_i2c_read_byte(i2c, REG_AFCTXD);
        pr_info("%s: AFC_TXD [0x%02x]\n",MUIC_DEV_NAME, vbus_txd_voltage);
        vbus_txd_voltage = 5000 + ((vbus_txd_voltage&0xF0)>>4)*1000;
        
        pr_info("%s:%s vbus_voltage:%d mV , AFC_TXD_VOLTAGE:%d mV \n"
                ,MUIC_DEV_NAME, __func__,vbus_voltage, vbus_txd_voltage);
        
        voltage_min = vbus_txd_voltage - 2000; // - 2000mV
        voltage_max = vbus_txd_voltage + 1000; // + 1000mV
        
        if ( (voltage_min <= vbus_voltage) && (vbus_voltage <= voltage_max) ) { /* AFC DONE */
            pmuic->afc_vbus_retry_count = 0;
        
            pr_info("%s:%s AFC done \n",MUIC_DEV_NAME, __func__);
            if (vbus_txd_voltage == 12000) { // 12V
                pmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_9V_MUIC;
                muic_notifier_attach_attached_dev(pmuic->attached_dev);
                pr_info("%s:%s AFC 12V (%d)\n",MUIC_DEV_NAME, __func__,pmuic->attached_dev);
            } else if (vbus_txd_voltage == 9000){ // 9V
                pmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_9V_MUIC;
                muic_notifier_attach_attached_dev(pmuic->attached_dev);
                pr_info("%s:%s AFC 9V (%d)\n",MUIC_DEV_NAME, __func__,pmuic->attached_dev);
            } else {  // 5V
                pmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_5V_MUIC;
                muic_notifier_attach_attached_dev(pmuic->attached_dev);
                pr_info("%s:%s AFC 5V (%d)\n",MUIC_DEV_NAME, __func__,pmuic->attached_dev);
            }
        } else {
            // VBUS_READ
            if ( pmuic->afc_vbus_retry_count < 3 ) {
                msleep(100);
                pmuic->afc_vbus_retry_count++;
                
                sm5720_set_afc_ctrl_reg(pdesc, AFCCTRL_VBUS_READ, 1);
                pr_info("%s:%s VBUS READ retry = %d \n",MUIC_DEV_NAME, __func__,
                        pmuic->afc_vbus_retry_count);
            } else {
                pmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_5V_MUIC;
                muic_notifier_attach_attached_dev(pmuic->attached_dev);
                pmuic->afc_vbus_retry_count = 0;
            }
        }
    } // if (device_type3 & DEV_TYPE3_QC20_TA) end

    return 0;
}

static int sm5720_afc_multi_byte(struct regmap_desc *pdesc)
{
    muic_data_t *pmuic = pdesc->muic;
    struct i2c_client *i2c = pmuic->i2c;
    int multi_byte[6] = {0,0,0,0,0,0};
    int i;
    int ret;
    int voltage_find;

    // ENAFC set '0'
    sm5720_set_afc_ctrl_reg(pdesc, AFCCTRL_ENAFC, 0);
    pr_info("%s:%s AFCCTRL_ENAFC 0 \n",MUIC_DEV_NAME, __func__);

    pr_info("%s:%s AFC_MULTI_BYTE \n",MUIC_DEV_NAME, __func__);

    // read AFC_RXD1 ~ RXD6
    voltage_find = 0;
    for(i = 0 ; i < 6 ; i++ ){
        multi_byte[i] = muic_i2c_read_byte(i2c, REG_AFCRXD1 + i);
        if (multi_byte[i] < 0)
            printk(KERN_ERR "%s: err read AFC_RXD%d %d\n", __func__,i+1, multi_byte[i]);
        pr_info("%s:%s AFC_RXD%d [0x%02x]\n",MUIC_DEV_NAME, __func__,i+1, multi_byte[i]);

        if( i >= 1 ){   // voltate find
            if ( ((multi_byte[i]&0xF0)>>4) >= ((multi_byte[voltage_find]&0xF0)>>4) ) {
                voltage_find = i;
            }
        }
    }

    pr_info("%s:%s AFC_TXD multi_byte[%d]=0x%02x \n", MUIC_DEV_NAME, __func__,
                                    voltage_find+1, multi_byte[voltage_find] );

    ret = muic_i2c_write_byte(i2c, REG_AFCTXD, multi_byte[voltage_find]);
    if (ret < 0)
        printk(KERN_ERR "[muic] %s: err write AFC_TXD(%d)\n", __func__, ret);

    // ENAFC set '1'
    sm5720_set_afc_ctrl_reg(pdesc, AFCCTRL_ENAFC, 1);
    pr_info("%s:%s AFCCTRL_ENAFC 1 \n",MUIC_DEV_NAME, __func__);

    return 0;

}

static int sm5720_afc_error(struct regmap_desc *pdesc)
{
    muic_data_t *pmuic = pdesc->muic;
    struct i2c_client *i2c = pmuic->i2c;
    int value;
    int device_type3;
    int ret,reg_val;

    // ENAFC set '0'
    sm5720_set_afc_ctrl_reg(pdesc, AFCCTRL_ENAFC, 0);
    pr_info("%s:%s AFCCTRL_ENAFC 0 \n",MUIC_DEV_NAME, __func__);

    pr_info("%s:%s AFC_ERROR (%d) \n",MUIC_DEV_NAME, __func__, pmuic->afc_retry_count);

    // read AFC_STATUS
    value = muic_i2c_read_byte(i2c, REG_AFCSTAT);
    if (value < 0)
        printk(KERN_ERR "%s: err read AFC_STATUS %d\n", __func__, value);
    pr_info("%s:%s AFC_STATUS [0x%02x]\n",MUIC_DEV_NAME, __func__, value);

    if (pmuic->afc_retry_count < 5) {
        device_type3 = muic_i2c_read_byte(i2c, REG_DEVT3);
        pr_info("%s:%s DEVICE_TYPE3 [0x%02x]\n",MUIC_DEV_NAME, __func__, device_type3);
        if (device_type3 & DEV_TYPE3_QC20_TA){ // QC20_TA

            pmuic->qc20_vbus = ENQC20_12V;
            ret = muic_i2c_read_byte(i2c, REG_AFCCNTL);
            reg_val = (ret & 0x3F) | (ENQC20_12V<<6);    // QC20 12V
            muic_i2c_write_byte(i2c, REG_AFCCNTL, reg_val);
            pr_info("%s:%s read REG_AFCCNTL=0x%x ,  write REG_AFCCNTL=0x%x , qc20_vbus=%d \n",
                MUIC_DEV_NAME, __func__, ret, reg_val, pmuic->qc20_vbus );

            msleep(40); // 40ms delay
            
            // VBUS_READ
            sm5720_set_afc_ctrl_reg(pdesc, AFCCTRL_VBUS_READ, 1);
            pr_info("%s:%s VBUS READ start(QC20-12V)\n",MUIC_DEV_NAME, __func__);
        } else {
            msleep(10); // 10ms delay
            // ENAFC set '1'
            sm5720_set_afc_ctrl_reg(pdesc, AFCCTRL_ENAFC, 1);
            pmuic->afc_retry_count++;
            pr_info("%s:%s re-start AFC (afc_retry_count=%d)\n",MUIC_DEV_NAME, __func__, pmuic->afc_retry_count);
        }
    } else {
        pr_info("%s:%s  ENAFC end = %d \n",MUIC_DEV_NAME, __func__, pmuic->afc_retry_count);
        pmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_5V_MUIC;
        muic_notifier_attach_attached_dev(pmuic->attached_dev);
    }
    return 0;
}

static int sm5720_afc_init_check(struct regmap_desc *pdesc)
{
    muic_data_t *pmuic = pdesc->muic;
    struct afc_ops *afcops = pmuic->regmapdesc->afcops;
    int afc_ta_attached;

    pr_info("%s:%s AFC_INIT_CHECK\n",MUIC_DEV_NAME, __func__);

    pr_info("%s:%s pmuic->vps.s.val1[0x%02x] , pmuic->vps.s.val3[0x%02x]\n",
         MUIC_DEV_NAME, __func__, pmuic->vps.s.val1, pmuic->vps.s.val3);

    /* check afc interrupt state */
    afc_ta_attached = muic_i2c_read_byte(pmuic->i2c, REG_AFCTASTATUS);
    pr_info("%s:%s REG_AFCTASTATUS:[0x%02x]\n", MUIC_DEV_NAME, __func__, afc_ta_attached);

    if (afc_ta_attached & 0x01){ // AFC_TA_ATTACHED 
        afcops->afc_ta_attach(pmuic->regmapdesc);
    }

    return 0;
}

static struct regmap_ops sm5720_muic_regmap_ops = {
    .get_size = sm5720_get_sizeof_regmap,
    .ioctl = sm5720_muic_ioctl,
    .get_formatted_dump = sm5720_get_fromatted_dump,
};

static struct vendor_ops sm5720_muic_vendor_ops = {
    .attach_ta = sm5720_attach_ta,
    .detach_ta = sm5720_detach_ta,
    .get_switch = sm5720_get_switching_mode,
    .set_switch = sm5720_set_switching_mode,
    .set_adc_scan_mode = sm5720_set_adc_scan_mode,
    .get_adc_scan_mode = sm5720_get_adc_scan_mode,
    .set_rustproof = sm5720_set_rustproof,
    .get_vps_data = sm5720_get_vps_data,
    .rescan = sm5720_run_BCD_rescan,
};

static struct afc_ops sm5720_muic_afc_ops = {
    .afc_ta_attach = sm5720_afc_ta_attach,
    .afc_ta_accept = sm5720_afc_ta_accept,
    .afc_vbus_update = sm5720_afc_vbus_update,
    .afc_multi_byte = sm5720_afc_multi_byte,
    .afc_error = sm5720_afc_error,
    .afc_ctrl_reg = sm5720_set_afc_ctrl_reg,
    .afc_init_check = sm5720_afc_init_check,
};

static struct regmap_desc sm5720_muic_regmap_desc = {
    .name = "sm5720-MUIC",
    .regmap = sm5720_muic_regmap_table,
    .size = REG_END,
    .regmapops = &sm5720_muic_regmap_ops,
    .vendorops = &sm5720_muic_vendor_ops,
    .afcops = &sm5720_muic_afc_ops,
};

void muic_register_sm5720_regmap_desc(struct regmap_desc **pdesc)
{
    *pdesc = &sm5720_muic_regmap_desc;
}

void sm5720_muic_check_reset(struct i2c_client *i2c, void *muic_data )
{
    muic_data_t *pmuic = muic_data;
    int value;

    pr_info("%s:%s \n",MUIC_DEV_NAME, __func__);

    value = muic_i2c_read_byte(pmuic->i2c, REG_INTMASK1);
    if ( value == 0x00 ) { // muic reset ...
        muic_print_reg_log();
        muic_print_reg_dump(pmuic);
        pr_err("%s: err muic could have been reseted. Initilize!!\n",
            __func__);
        muic_reg_init(pmuic);
        muic_print_reg_dump(pmuic);
        
        /* MUIC Interrupt On */
        set_int_mask(pmuic, false);
    }

    return;
}

