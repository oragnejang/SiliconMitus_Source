/*
 * muic_sm5720_afc.c
 *
 * Copyright (C) 2014 Samsung Electronics
 * Jeongrae Kim <jryu.kim@samsung.com>
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
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <linux/muic/muic.h>
#include "muic-internal.h"
#include "muic_regmap.h"
#include "muic_i2c.h"
#if defined(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */
#include "muic_sm5720_afc.h"


/* Bit 0 : VBUS_VAILD, Bit 1~7 : Reserved */
#define REG_RSVDID1     0x15


#define REG_AFCTXD       0x19
#define REG_AFCSTAT      0x1a
#define REG_VBUSVOL1     0x1b
#define REG_VBUSVOL2     0x1c
#define REG_AFCTASTATUS  0x23


muic_data_t *gpmuic;

int muic_is_afc_voltage(void);
int muic_dpreset_afc(void);
int muic_restart_afc(void);

/* To make AFC work properly on boot */
static int is_charger_ready;
static struct work_struct muic_afc_init_work;


int muic_is_afc_voltage(void)
{
    struct i2c_client *i2c = gpmuic->i2c;
    int vbus_voltage, voltage1, voltage2;
    
    if (gpmuic->attached_dev == ATTACHED_DEV_NONE_MUIC) {
        pr_info("%s attached_dev None \n", __func__);
        return 0;
    }
    voltage1 = muic_i2c_read_byte(i2c, REG_VBUSVOL1);
    voltage2 = muic_i2c_read_byte(i2c, REG_VBUSVOL2);    

    vbus_voltage = voltage1*1000 + (voltage2*3900)/1000;
    
    return vbus_voltage;
}

int muic_dpreset_afc(void)
{
    struct afc_ops *afcops = gpmuic->regmapdesc->afcops;

    pr_info("%s: gpmuic->attached_dev = %d\n", __func__, gpmuic->attached_dev);
    if ( (gpmuic->attached_dev == ATTACHED_DEV_AFC_CHARGER_9V_MUIC) ||
         //(gpmuic->attached_dev == ATTACHED_DEV_AFC_CHARGER_12V_MUIC) ||
         (gpmuic->attached_dev == ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC) ) {
        // ENAFC set '0'
        afcops->afc_ctrl_reg(gpmuic->regmapdesc, AFCCTRL_ENAFC, 0);
        msleep(50); // 50ms delay

        // DP_RESET
        pr_info("%s:AFC Disable \n", __func__);
        afcops->afc_ctrl_reg(gpmuic->regmapdesc, AFCCTRL_DP_RESET, 1);

        gpmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_5V_MUIC;
        muic_notifier_attach_attached_dev(gpmuic->attached_dev);
    }

    return 0;
}

int muic_restart_afc(void)
{
    struct i2c_client *i2c = gpmuic->i2c;
    int ret, value;
    struct afc_ops *afcops = gpmuic->regmapdesc->afcops;
    int afc_ta_attached;

    pr_info("%s:AFC Restart attached_dev = 0x%x\n", __func__, gpmuic->attached_dev);

    if (gpmuic->attached_dev == ATTACHED_DEV_NONE_MUIC) {
        pr_info("%s:%s Device type is None\n",MUIC_DEV_NAME, __func__);
        return 0;
    }
    
    msleep(120); // 120ms delay
    afc_ta_attached = muic_i2c_read_byte(i2c, REG_AFCTASTATUS);    
    pr_info("%s: afc_ta_attached = 0x%x\n", __func__, afc_ta_attached );
    if (afc_ta_attached == 0x00){ 
        return 0;
    }
    
    gpmuic->attached_dev = ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC;
    muic_notifier_attach_attached_dev(gpmuic->attached_dev);
	cancel_delayed_work(&gpmuic->afc_retry_work);
	schedule_delayed_work(&gpmuic->afc_retry_work, msecs_to_jiffies(5000)); // 5sec
	pr_info("%s: afc_retry_work(RESTART) start \n", __func__);

    // voltage(9.0V)  + current(1.65A) setting : 0x46
    // voltage(12.0V) + current(2.1A) setting : 0x79
    value = 0x79;
    ret = muic_i2c_write_byte(i2c, REG_AFCTXD, value);
    if (ret < 0)
        printk(KERN_ERR "[muic] %s: err write AFC_TXD(%d)\n", __func__, ret);
    pr_info("%s:AFC_TXD [0x%02x]\n", __func__, value);

    // ENAFC set '1'
    afcops->afc_ctrl_reg(gpmuic->regmapdesc, AFCCTRL_ENAFC, 1);

    return 0;
}

static void muic_afc_retry_work(struct work_struct *work)
{
	struct afc_ops *afcops = gpmuic->regmapdesc->afcops;
	struct i2c_client *i2c = gpmuic->i2c;
	int ret, vbus;

	pr_info("%s:AFC retry work\n", __func__);

    ret = muic_i2c_read_byte(i2c, REG_AFCSTAT);
    pr_info("%s : Read REG_AFCSTAT = [0x%02x]\n", __func__, ret);

	ret = muic_i2c_read_byte(i2c, 0x3C);
	pr_info("%s : Read 0x3C = [0x%02x]\n", __func__, ret);
    
	ret = muic_i2c_read_byte(i2c, 0x3D);
	pr_info("%s : Read 0x3D = [0x%02x]\n", __func__, ret);
	
    pr_info("%s:attached_dev = %d \n", __func__, gpmuic->attached_dev);
	if (gpmuic->attached_dev == ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC) {
		vbus = muic_i2c_read_byte(i2c, REG_RSVDID1);
		if (!(vbus & 0x01)) {
			pr_info("%s:%s VBUS is nothing\n",MUIC_DEV_NAME, __func__);
			gpmuic->attached_dev = ATTACHED_DEV_NONE_MUIC;
			muic_notifier_attach_attached_dev(gpmuic->attached_dev);
			return;
		}

		pr_info("%s: [MUIC] device type is afc prepare - Disable AFC\n", __func__);
		afcops->afc_ctrl_reg(gpmuic->regmapdesc, AFCCTRL_DP_RESET, 1);
	}
}


static void muic_focrced_detection_by_charger(struct work_struct *work)
{
    struct afc_ops *afcops = gpmuic->regmapdesc->afcops;

    pr_info("%s\n", __func__);

    mutex_lock(&gpmuic->muic_mutex);

    afcops->afc_init_check(gpmuic->regmapdesc);

    mutex_unlock(&gpmuic->muic_mutex);
}

void sm5720_muic_charger_init(void)
{
    pr_info("%s\n", __func__);

    if (!gpmuic) {
        pr_info("%s: MUIC AFC is not ready.\n", __func__);
        return;
    }

    if (is_charger_ready) {
        pr_info("%s: charger is already ready.\n", __func__);
        return;
    }

    is_charger_ready = true;

    if (gpmuic->attached_dev == ATTACHED_DEV_TA_MUIC)
        schedule_work(&muic_afc_init_work);
}

//static ssize_t afc_off_show(struct device *dev,
//      struct device_attribute *attr, char *buf)
//{
//  muic_data_t *pmuic = dev_get_drvdata(dev);
//  return snprintf(buf, 4, "%d\n", pmuic->is_flash_on);
//}

//static ssize_t afc_off_store(struct device *dev,
//      struct device_attribute *attr,
//      const char *buf, size_t size)
//{
//  if (!strncmp(buf, "1", 1)) {
//      pr_info("%s, Disable AFC\n", __func__);
//      muic_dpreset_afc();
//  } else {
//      pr_info("%s, Enable AFC\n", __func__);
//      muic_restart_afc();
// }
//  return size;
//}
//
//static DEVICE_ATTR(afc_off, S_IRUGO | S_IWUSR,
//      afc_off_show, afc_off_store);


void sm5720_muic_init_afc_state(muic_data_t *pmuic)
{
    //int ret;
    struct i2c_client *i2c;
    
    gpmuic = pmuic;
    i2c = gpmuic->i2c;

    /* To make AFC work properly on boot */
    INIT_WORK(&muic_afc_init_work, muic_focrced_detection_by_charger);
    INIT_DELAYED_WORK(&gpmuic->afc_retry_work, muic_afc_retry_work);

    //ret = device_create_file(switch_device, &dev_attr_afc_off);
    //if (ret < 0) {
    //  pr_err("[MUIC] Failed to create file (disable AFC)!\n");
    //}

    pr_info("%s:attached_dev = %d\n", __func__, gpmuic->attached_dev);

}

MODULE_DESCRIPTION("MUIC driver");
MODULE_AUTHOR("<jryu.kim@samsung.com>");
MODULE_LICENSE("GPL");
