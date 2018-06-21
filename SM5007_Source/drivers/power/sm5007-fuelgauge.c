/* drivers/power/sm5007-fuelgauge.c
 * SM5007 Voltage Tracking Fuelgauge Driver
 *
 * Copyright (C) 2012-2014 SILICONMITUS COMPANY,LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include <linux/of.h>
#include <linux/fs.h>
#include <linux/math64.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/power_supply.h>
#include <linux/mfd/sm5007.h>
#include <linux/power/sm5007_charger.h>
#include <linux/power/sm5007-fuelgauge.h>
#include <linux/power/sm5007-fuelgauge-impl.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>

#include <linux/alarmtimer.h>
#include <linux/rtc.h>
#include "../mmc/host/jzmmc_utils.h"

#define ALIAS_NAME "sm5007-fuelgauge"
#define LOW_VOL_ALERT 3200//mean 3.2v
#define LOW_SOC_ALERT_10 1050//mean 10.5
#define LOW_SOC_ALERT_5 550//mean 5.5
#define LOW_SOC_ALERT 50 //mean 0.50
static struct i2c_client *m_client;
struct sm5007_fuelgauge_info *sm5007_fuelgauge = NULL;
extern struct sm5007_charger_info *sm5007_charger;
extern int get_power_supply_status(struct sm5007_charger_info *info);

bool sm5007_hal_fg_reset(struct i2c_client *client);
void sm5007_fg_low_soc_alert_update(struct i2c_client *client);

enum battery_table_type {
	DISCHARGE_TABLE = 0,
	CHARGE_TABLE,
	Q_TABLE,
	TABLE_MAX,
};

static inline int __sm5007_fg_bulk_reads(struct i2c_client *client, u8 reg,
				int len, uint8_t *val)
{
	int ret;
	int i;

	ret = i2c_smbus_read_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading from 0x%02x\n", reg);
		return ret;
	}
	for (i = 0; i < len; ++i) {
		dev_dbg(&client->dev, "sm5007: reg read  reg=%x, val=%x\n",
				reg + i, *(val + i));
	}
	return 0;
}

static inline int __sm5007_fg_bulk_writes(struct i2c_client *client, u8 reg,
				  int len, uint8_t *val)
{
	int ret;
	int i;

	for (i = 0; i < len; ++i) {
		dev_dbg(&client->dev, "sm5007: reg write  reg=%x, val=%x\n",
				reg + i, *(val + i));
	}

	ret = i2c_smbus_write_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writings to 0x%02x\n", reg);
		return ret;
	}

	return 0;
}

int sm5007_fg_bulk_reads_bank1(struct i2c_client *client, u8 reg, u8 len, uint8_t *val)
{
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&fuelgauge->fg_lock);
	//ret = set_bank_sm5007(dev, 1);
	//if (!ret)
		ret = __sm5007_fg_bulk_reads(client, reg, len, val);
	mutex_unlock(&fuelgauge->fg_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(sm5007_fg_bulk_reads_bank1);

int sm5007_fg_bulk_writes_bank1(struct i2c_client *client, u8 reg, u8 len, uint8_t *val)
{
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&fuelgauge->fg_lock);
	ret = __sm5007_fg_bulk_writes(client, reg, len, val);
	mutex_unlock(&fuelgauge->fg_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(sm5007_fg_bulk_writes_bank1);

static int32_t sm5007_fg_i2c_read_word(struct i2c_client *client,
						uint8_t reg_addr)
{
	uint16_t data = 0;
	int ret;
	/*ret = sm5007_fg_read_device(fuelgauge->dev->parent, reg_addr, 2, &data);*/
	ret = sm5007_fg_bulk_reads_bank1(client, reg_addr, 2, (void *)&data);
	/*pr_debug("%s: ret = %d, addr = 0x%x, data = 0x%x\n",
	__func__, ret, reg_addr, data);*/

	if (ret < 0)
		return ret;
	else
		return data;
}


static int32_t sm5007_fg_i2c_write_word(struct i2c_client *client,
uint8_t reg_addr, uint16_t data)
{
	int ret;

	/* not use big endian*/
	/*data = cpu_to_be16(data);*/
	/*ret = i2c_smbus_write_i2c_block_data(fuelgauge->dev->parent, reg_addr, 2,
	(uint8_t *)&data);*/
	ret = sm5007_fg_bulk_writes_bank1(client, reg_addr, 2, (uint8_t *)&data);

	/*pr_debug("%s: ret = %d, addr = 0x%x, data = 0x%x\n",
	__func__, ret, reg_addr, data);*/
	return ret;
}

static int Battery_Type(void)
{
#if defined(CONFIG_320MAH_BATTERY)
	return 1;
#elif defined(CONFIG_295MAH_BATTERY)
    return 2;
#elif defined(CONFIG_300MAH_BATTERY)
    return 3;
#elif defined(CONFIG_400MAH_BATTERY)
    return 4;
#else
	return 0;
#endif
}

static unsigned int fg_get_vbat(struct i2c_client *client);
static unsigned int fg_get_ocv(struct i2c_client *client);
static int fg_get_curr(struct i2c_client *client);
static int fg_get_temp(struct i2c_client *client);
static bool sm5007_fg_init(struct i2c_client *client);

static void sm5007_pr_ver_info(struct i2c_client *client)
{
	pr_debug("sm5007 Fuel-Gauge Ver %s\n", FG_DRIVER_VER);
}

void fg_abnormal_reset_check(struct i2c_client *client)
{
	int ret;

	/* abnormal case.... SW reset*/
	ret = sm5007_fg_i2c_read_word(client, SM5007_REG_FG_OP_STATUS);
	pr_debug("%s : SM5007_REG_FG_OP_STATUS VALUE = 0x%x\n", __func__, ret);
	if ((ret&0x00FF) != 0x0007) {
		/* SW reset code*/
		printk("%s : sm5007 FG abnormal case... SW reset, SM5007_REG_FG_OP_STATUS: 0x%x\n", __func__, ret);
		sm5007_hal_fg_reset(client);
	}
}

void fg_vbatocv_check(struct i2c_client *client)
{
	int ret;
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	// iocv error case cover start
	if((abs(fuelgauge->info.batt_current)<20)
		&& (abs(fuelgauge->info.p_batt_current-fuelgauge->info.batt_current)<10))
	{
		if(abs(fuelgauge->info.batt_ocv-fuelgauge->info.batt_voltage)>30) // 30mV over
		{
			fuelgauge->info.iocv_error_count ++;
		}
		if(fuelgauge->info.iocv_error_count > 5) // prevent to overflow
			fuelgauge->info.iocv_error_count = 6;
	}
	else
	{
		fuelgauge->info.iocv_error_count = 0;
	}

	pr_debug("%s: iocv_error_count (%d)\n", __func__, fuelgauge->info.iocv_error_count);

	if(fuelgauge->info.iocv_error_count > 5)
	{
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
		printk("%s: p_v - v = (%d)\n", __func__, fuelgauge->info.p_batt_voltage - fuelgauge->info.batt_voltage);
#endif
		if(abs(fuelgauge->info.p_batt_voltage - fuelgauge->info.batt_voltage)>15) // 15mV over
		{
			fuelgauge->info.iocv_error_count = 0;
		}
		else
		{
			// mode change to mix RS manual mode
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
			printk("%s: mode change to mix RS manual mode\n", __func__);
#endif

			// RS manual value write
			sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_RS_MAN, fuelgauge->info.rs_value[0]);
			// run update
			sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_PARAM_RUN_UPDATE, 0);
			sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_PARAM_RUN_UPDATE, 1);
			// mode change
			ret = sm5007_fg_i2c_read_word(client, SM5007_REG_CNTL);
			ret = ret | ENABLE_RS_MAN_MODE;//+RS_MAN_MODE
			sm5007_fg_i2c_write_word(client, SM5007_REG_CNTL, ret);
		}
	}
	else
	{
		if((fuelgauge->info.p_batt_voltage < fuelgauge->info.alarm_vol_mv)
			&& (fuelgauge->info.batt_voltage < fuelgauge->info.alarm_vol_mv) && (!fuelgauge->is_charging))
		{
			// mode change to mix RS manual mode
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
			printk("%s: mode change to mix RS manual mode\n", __func__);
#endif
			// RS manual value write
			sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_RS_MAN, fuelgauge->info.rs_value[0]);
			// run update
			sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_PARAM_RUN_UPDATE, 0);
			sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_PARAM_RUN_UPDATE, 1);
			// mode change
			ret = sm5007_fg_i2c_read_word(client, SM5007_REG_CNTL);
			ret = ret | ENABLE_RS_MAN_MODE;//+RS_MAN_MODE
			sm5007_fg_i2c_write_word(client, SM5007_REG_CNTL, ret);
		}
		else
		{
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
			printk("%s: mode change to mix RS auto mode\n", __func__);
#endif
			// mode change to mix RS auto mode
			ret = sm5007_fg_i2c_read_word(client, SM5007_REG_CNTL);
			ret = ret & ~ENABLE_RS_MAN_MODE;//+RS_AUTO_MODE
			sm5007_fg_i2c_write_word(client, SM5007_REG_CNTL, ret);
		}
	}
	fuelgauge->info.p_batt_voltage = fuelgauge->info.batt_voltage;
	fuelgauge->info.p_batt_current = fuelgauge->info.batt_current;
	// iocv error case cover end
}

#ifdef CONFIG_SM5007_FG_DEBUG_LOG
static void sm5007_fg_test_read(struct i2c_client *client)
{
	int ret1, ret2, ret3, ret4;
   	ret1 = sm5007_fg_i2c_read_word(client, 0xBC);
   	ret2 = sm5007_fg_i2c_read_word(client, 0xBD);
   	ret3 = sm5007_fg_i2c_read_word(client, 0xBE);
   	ret4 = sm5007_fg_i2c_read_word(client, 0xBF);
   	printk("0xBC=0x%04x, 0xBD=0x%04x, 0xBE=0x%04x, 0xBF=0x%04x \n", ret1, ret2, ret3, ret4);

   	ret1 = sm5007_fg_i2c_read_word(client, 0xCC);
   	ret2 = sm5007_fg_i2c_read_word(client, 0xCD);
	ret3 = sm5007_fg_i2c_read_word(client, 0xCE);
	ret4 = sm5007_fg_i2c_read_word(client, 0xCF);
	printk("0xCC=0x%04x, 0xCD=0x%04x, 0xCE=0x%04x, 0xCF=0x%04x \n", ret1, ret2, ret3, ret4);
		 
	ret1 = sm5007_fg_i2c_read_word(client, 0x85);
	ret2 = sm5007_fg_i2c_read_word(client, 0x86);
	ret3 = sm5007_fg_i2c_read_word(client, 0x87);
	ret4 = sm5007_fg_i2c_read_word(client, 0x28);
	printk("0x85=0x%04x, 0x86=0x%04x, 0x87=0x%04x, 0x28=0x%04x \n", ret1, ret2, ret3, ret4);
																	  	  	 
	ret1 = sm5007_fg_i2c_read_word(client, SM5007_REG_FG_OP_STATUS);
	printk("SM5007_REG_FG_OP_STATUS VALUE = 0x%x\n", ret1);
	ret2 = sm5007_fg_i2c_read_word(client, 0x01);//CNTL
	printk("SM5007_REG_FG_CNTR = 0x%x\n", ret2);
}
#endif

static unsigned int u32_fg_get_soc(struct i2c_client * client)
{
	int ret;
	unsigned int soc;/* = 850; 850 means 85% */

	ret = sm5007_fg_i2c_read_word(client, SM5007_REG_SOC);
	if (ret < 0) {
		pr_err("%s: read soc reg fail\n", __func__);
		soc = 500;
	} else {
		/*integer bit;*/
		soc = ((ret&0xff00)>>8) * 10;
		/* integer + fractional bit*/
		soc = soc + (((ret&0x00ff)*10)/256);
	}
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("%s: read = 0x%x, soc = %d\n", __func__, ret, soc);
	sm5007_fg_test_read(client);
#endif
    return soc;
}

unsigned int fg_get_soc(struct i2c_client *client)
{
	unsigned int soc;
	int curr_cal;
	int temp_cal_fact;
	int temp_off_fact;
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	fg_get_vbat(fuelgauge->client);
	fg_get_ocv(fuelgauge->client);
	fg_get_curr(fuelgauge->client);
	fg_get_temp(fuelgauge->client);

	fuelgauge->is_charging = (fuelgauge->info.batt_current > 10) ? true : false;
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("%s: is_charging = %d\n", __func__, fuelgauge->is_charging);
#endif
	fg_abnormal_reset_check(client);
	fg_vbatocv_check(client);

	if (fuelgauge->is_charging)
		curr_cal = fuelgauge->info.curr_cal
		+(fuelgauge->info.charge_offset_cal << 8);
	else
		curr_cal = fuelgauge->info.curr_cal;
	pr_debug("%s: curr_cal = 0x%x\n", __func__, curr_cal);

	temp_cal_fact = fuelgauge->info.temp_std
		- (fuelgauge->info.temperature / 10);
	if(0 < temp_cal_fact){
		if(30 < temp_cal_fact) //protect overflow
			temp_cal_fact = 30;
		temp_cal_fact = temp_cal_fact / fuelgauge->info.temp_offset;
		temp_off_fact = temp_cal_fact;
		temp_cal_fact = temp_cal_fact * fuelgauge->info.temp_offset_cal;
		curr_cal = curr_cal + (temp_cal_fact << 8);

		temp_off_fact = temp_off_fact * fuelgauge->info.temp_offset_cal_ofs;
		curr_cal = curr_cal + temp_off_fact + 0x80;
	}
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("%s: temp_std = %d, temperature = %d, temp_offset = %d, temp_offset_cal = 0x%x, temp_offset_cal_ofs = 0x%x,  curr_cal = 0x%x\n",
		__func__, fuelgauge->info.temp_std, fuelgauge->info.temperature,
		fuelgauge->info.temp_offset,
		fuelgauge->info.temp_offset_cal,
		fuelgauge->info.temp_offset_cal_ofs,
		curr_cal);
#endif
	sm5007_fg_i2c_write_word(client, SM5007_REG_CURR_CAL, curr_cal);

	soc = u32_fg_get_soc(client);
/*if ocv <= 3400, force soc = 0*/
	if (fuelgauge->info.batt_ocv <= 3400 /*|| soc < 10*/) {
		soc = 0;
	} else {
/* round off the soc value */
		soc = (((soc % 10) > 4) ? 10 : 0) + (soc / 10) * 10;
	}

    fuelgauge->info.batt_soc = soc;
    sm5007_fg_low_soc_alert_update(client);

	return soc;
}

static unsigned int fg_get_ocv(struct i2c_client *client)
{
	int ret;
	unsigned int ocv;/* = 3500; 3500 means 3500mV*/
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	ret = sm5007_fg_i2c_read_word(client, SM5007_REG_OCV);
	if (ret < 0) {
		pr_err("%s: read ocv reg fail\n", __func__);
		ocv = 4000;
	} else {
		/*integer bit;*/
		ocv = ((ret&0xf000)>>0xc) * 1000;
		/* integer + fractional bit*/
		ocv = ocv + (((ret&0x0fff)*1000)/4096);
	}

	fuelgauge->info.batt_ocv = ocv;
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("%s: read = 0x%x, ocv = %d\n", __func__, ret, ocv);
#endif


	return ocv;
}

static unsigned int fg_get_vbat(struct i2c_client *client)
{
	int ret;
	unsigned int vbat;/* = 3500; 3500 means 3500mV*/
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	ret = sm5007_fg_i2c_read_word(client, SM5007_REG_VOLTAGE);

	if (ret < 0) {
		pr_err("%s: read vbat reg fail", __func__);
		vbat = 4000;
	} else {
		/*integer bit*/
		vbat = ((ret&0xf000)>>0xc) * 1000;
		/* integer + fractional bit*/
		vbat = vbat + (((ret&0x0fff)*1000)/4096);
	}
	fuelgauge->info.batt_voltage = vbat;

	/*cal avgvoltage*/
	fuelgauge->info.batt_avgvoltage =
	(((fuelgauge->info.batt_avgvoltage)*4) + vbat)/5;

#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("%s: read = 0x%x, vbat = %d\n", __func__, ret, vbat);
	printk("%s: batt_avgvoltage = %d\n", __func__, fuelgauge->info.batt_avgvoltage);
#endif

	return vbat;
}

static int fg_get_curr(struct i2c_client *client)
{
	int ret;

	int curr;/* = 1000; 1000 means 1000mA*/
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	ret = sm5007_fg_i2c_read_word(client, SM5007_REG_CURRENT);
	if (ret < 0) {
		pr_err("%s: read curr reg fail", __func__);
		curr = 0;
	} else {
		/*integer bit*/
		curr = ((ret&0x7800)>>0xb) * 1000;
		/* integer + fractional bit*/
		curr = curr + (((ret&0x07ff)*1000)/2048);
		if (ret&0x8000)
			curr *= -1;
	}
	fuelgauge->info.batt_current = curr;

	/*cal avgcurr*/
	fuelgauge->info.batt_avgcurrent =
	(((fuelgauge->info.batt_avgcurrent)*4) + curr)/5;

#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("%s: read = 0x%x, curr = %d\n", __func__, ret, curr);
	printk("%s: batt_avgcurrent = %d\n", __func__, fuelgauge->info.batt_avgcurrent);
#endif

	return curr;
}

static int fg_get_temp(struct i2c_client *client)
{
	int ret;

	int temp;/* = 250; 250 means 25.0oC*/
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	ret = sm5007_fg_i2c_read_word(client, SM5007_REG_TEMPERATURE);
	if (ret < 0) {
		pr_err("%s: read temp reg fail", __func__);
		temp = 0;
	} else {
		 /*integer bit*/
		temp = ((ret&0x7F00)>>8) * 10;
		 /* integer + fractional bit*/
		temp = temp + ((((ret&0x00f0)>>4)*10)/16);
		if (ret&0x8000)
			temp *= -1;
	}
	fuelgauge->info.temperature = temp;

#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("%s: read = 0x%x, temperature = %d\n", __func__, ret, temp);
#endif

	return temp;
}

static unsigned int fg_get_device_id(struct i2c_client *client)
{
	int ret;

	ret = sm5007_fg_i2c_read_word(client, SM5007_REG_DEVICE_ID);
	/*ret &= 0x00ff;*/
	pr_debug("%s: device_id = 0x%x\n", __func__, ret);

	return ret;
}

static bool sm5007_fg_check_reg_init_need(struct i2c_client *client)
{

	int ret;

	ret = sm5007_fg_i2c_read_word(client, SM5007_REG_FG_OP_STATUS);

	if ((ret & 0x00FF) == DISABLE_RE_INIT) {
		pr_debug("%s: return 0\n", __func__);
		return 0;
	} else {
		pr_debug("%s: return 1\n", __func__);
		return 1;
	}

	return 1;
}

static bool sm5007_fg_reg_init(struct i2c_client *client)
{
	int i, j, value, ret;
	uint8_t table_reg;
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	pr_debug("%s: sm5007_fg_reg_init START!!\n", __func__);

	/*start first param_ctrl unlock*/
	sm5007_fg_i2c_write_word(client,
		SM5007_REG_PARAM_CTRL, SM5007_FG_PARAM_UNLOCK_CODE);

	/* RCE write*/
	for (i = 0; i < 3; i++)	{
		sm5007_fg_i2c_write_word(client,
			SM5007_REG_RCE0+i, fuelgauge->info.rce_value[i]);
		pr_debug("%s: RCE write RCE%d = 0x%x : 0x%x\n",
			__func__,  i, SM5007_REG_RCE0+i,
			fuelgauge->info.rce_value[i]);
	}

	/* DTCD write*/
	sm5007_fg_i2c_write_word(client,
		SM5007_REG_DTCD, fuelgauge->info.dtcd_value);
	pr_debug("%s: DTCD write DTCD = 0x%x : 0x%x\n",
		__func__, SM5007_REG_DTCD, fuelgauge->info.dtcd_value);

	/* RS write*/
	sm5007_fg_i2c_write_word(client,
		SM5007_REG_RS, fuelgauge->info.rs_value[0]);
	pr_debug("%s: RS write RS = 0x%x : 0x%x\n",
		__func__, SM5007_REG_RS, fuelgauge->info.rs_value[0]);

    /*RS_MAN write*/
    sm5007_fg_i2c_write_word(client,
        SM5007_REG_RS_MAN, fuelgauge->info.rs_value[0]);
	/*/ VIT_PERIOD write*/
	sm5007_fg_i2c_write_word(client,
		SM5007_REG_VIT_PERIOD, fuelgauge->info.vit_period);
	pr_debug("%s: VIT_PERIOD write VIT_PERIOD = 0x%x : 0x%x\n",
		__func__, SM5007_REG_VIT_PERIOD, fuelgauge->info.vit_period);

	/*/ TABLE_LEN write & pram unlock*/
	sm5007_fg_i2c_write_word(client,
		SM5007_REG_PARAM_CTRL, SM5007_FG_PARAM_UNLOCK_CODE | SM5007_FG_TABLE_LEN);

	for (i = 0; i < 3; i++) {
		table_reg = SM5007_REG_TABLE_START + (i<<4);
		for (j = 0; j <= SM5007_FG_TABLE_LEN; j++) {
			sm5007_fg_i2c_write_word(client,
				(table_reg + j),
				fuelgauge->info.battery_table[i][j]);
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
			printk("%s: TABLE write table[%d][%d] = 0x%x : 0x%x\n",
				__func__, i, j, (table_reg + j), fuelgauge->info.battery_table[i][j]);
#endif
		}
	}

	/* MIX_MODE write*/
	sm5007_fg_i2c_write_word(client,
		SM5007_REG_RS_MIX_FACTOR, fuelgauge->info.rs_value[1]);
	sm5007_fg_i2c_write_word(client,
		SM5007_REG_RS_MAX, fuelgauge->info.rs_value[2]);
	sm5007_fg_i2c_write_word(client,
		SM5007_REG_RS_MIN, fuelgauge->info.rs_value[3]);
	sm5007_fg_i2c_write_word(client,
		SM5007_REG_MIX_RATE, fuelgauge->info.mix_value[0]);
	sm5007_fg_i2c_write_word(client,
		SM5007_REG_MIX_INIT_BLANK, fuelgauge->info.mix_value[1]);
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	pr_debug("%s: RS_MIX_FACTOR = 0x%x, RS_MAX = 0x%x, RS_MIN = 0x%x, MIX_RATE = 0x%x, MIX_INIT_BLANK = 0x%x\n",
		__func__,
		fuelgauge->info.rs_value[1], fuelgauge->info.rs_value[2],
		fuelgauge->info.rs_value[3], fuelgauge->info.mix_value[0],
		fuelgauge->info.mix_value[1]);
#endif

	/* CAL write*/
	sm5007_fg_i2c_write_word(client, SM5007_REG_VOLT_CAL, fuelgauge->info.volt_cal);
	sm5007_fg_i2c_write_word(client, SM5007_REG_CURR_CAL, fuelgauge->info.curr_cal);
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("%s: VOLT_CAL = 0x%x, CURR_CAL = 0x%x\n",
		__func__, fuelgauge->info.volt_cal, fuelgauge->info.curr_cal);
#endif

	/*INIT_last -  control register set*/
	value = sm5007_fg_i2c_read_word(client, SM5007_REG_CNTL);
	value |= ENABLE_MIX_MODE | ENABLE_TEMP_MEASURE| ENABLE_V_ALARM;

	/*TOPOFFSOC*/
	if(fuelgauge->info.enable_topoff_soc)
		value |= ENABLE_TOPOFF_SOC;
	else
		value &= (~ENABLE_TOPOFF_SOC);

	ret = sm5007_fg_i2c_write_word(client, SM5007_REG_CNTL, value);
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("%s: sm5007_REG_CNTL reg : 0x%x\n",
		__func__, value);
#endif

	/* LOCK PARAM_CTRL*/
	value = SM5007_FG_PARAM_LOCK_CODE | SM5007_FG_TABLE_LEN;
	sm5007_fg_i2c_write_word(client, SM5007_REG_PARAM_CTRL, value);
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("%s: LAST PARAM CTRL VALUE = 0x%x : 0x%x\n",
		__func__, SM5007_REG_PARAM_CTRL, value);
#endif

	return 1;
}

void sm5007_fg_low_vol_alert_update(struct i2c_client *client, unsigned int low_vol_alert_mv)
{
	unsigned int value = 0;
	unsigned int temp = 0;
	value = (low_vol_alert_mv / 1000);
	temp = ((low_vol_alert_mv - (value * 1000))<<8)/1000;
	value = (value << 8)|temp;
	printk("%s: low_vol_alert_mv:%d, 0x%x\n", __func__, low_vol_alert_mv, value);
	sm5007_fg_i2c_write_word(client, SM5007_REG_V_ALARM, value);
	sm5007_fg_i2c_write_word(client, SM5007_REG_PARAM_RUN_UPDATE, 0);
	sm5007_fg_i2c_write_word(client, SM5007_REG_PARAM_RUN_UPDATE, 1);
}

void sm5007_fg_temp_alert_update(struct i2c_client *client, int low_temp_alert, int high_temp_alert)
{
	int ret = 0;
	if (low_temp_alert < 0)
		ret |=  0x80;
	ret |= (low_temp_alert) & 0x7F;
	if (high_temp_alert < 0)
		ret |=  0x8000;
	ret |= (high_temp_alert << 8) & 0x7F00;

	printk("%s: temp_alert:%d, %d, 0x%x\n", __func__, low_temp_alert, high_temp_alert, ret);
	sm5007_fg_i2c_write_word(client, SM5007_REG_T_ALARM, ret);
	sm5007_fg_i2c_write_word(client, SM5007_REG_PARAM_RUN_UPDATE, 0);
	sm5007_fg_i2c_write_word(client, SM5007_REG_PARAM_RUN_UPDATE, 1);
}

void sm5007_fg_set_soc_alert(struct i2c_client *client, unsigned int low_soc_alert) //15%==> 1500 0.5 ==> 50
{
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	int value = 0;
	unsigned int temp = 0;
	value = (low_soc_alert/100);
	temp = ((low_soc_alert - (value * 100))<<8)/100;
	value = (value << 8) | temp;
	sm5007_fg_i2c_write_word(client, SM5007_REG_SOC_ALARM, value);
	sm5007_fg_i2c_write_word(client, SM5007_REG_PARAM_RUN_UPDATE, 0);
	sm5007_fg_i2c_write_word(client, SM5007_REG_PARAM_RUN_UPDATE, 1);
	fuelgauge->info.batt_alert_soc = low_soc_alert;
	printk("%s:%d%%\n", __func__, low_soc_alert/100);
}

void sm5007_fg_low_soc_alert_update(struct i2c_client *client)
{
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	if (fuelgauge->info.batt_soc > 100) {
		if (fuelgauge->info.batt_alert_soc != LOW_SOC_ALERT_10)
			sm5007_fg_set_soc_alert(client, LOW_SOC_ALERT_10);
	} else if (fuelgauge->info.batt_soc > 50) {
		if (fuelgauge->info.batt_alert_soc != LOW_SOC_ALERT_5) 
			sm5007_fg_set_soc_alert(client, LOW_SOC_ALERT_5);
	} else {
		if (fuelgauge->info.batt_alert_soc != LOW_SOC_ALERT)
			sm5007_fg_set_soc_alert(client, LOW_SOC_ALERT);
	}
}

bool sm5007_fg_fuelalert_init(struct i2c_client *client, struct sm5007_fuelgauge_platform_data *pdata)
{
	int ret = 0;
	int value = 0;
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	printk("%s: sm5007_fg_fuelalert_init\n", __func__);
	//reset vol alert
	fuelgauge->info.volt_alert_flag = false;

	sm5007_fg_low_vol_alert_update(client, LOW_VOL_ALERT);
	sm5007_fg_set_soc_alert(client, LOW_SOC_ALERT_10); //default 10%

	value = sm5007_fg_i2c_read_word(client, SM5007_REG_CNTL);
	value |=  ENABLE_V_ALARM | ENABLE_SOC_ALARM;

	sm5007_fg_i2c_write_word(client,SM5007_REG_CNTL, value);
	/* remove vol/soc alert mask & interrupt */
	sm5007_fg_i2c_write_word(client,SM5007_REG_INTFG_MASK, 0x0006);
	/*clear interrupt*/
	sm5007_fg_i2c_read_word(client, SM5007_REG_INTFG);

	ret = sm5007_fg_i2c_read_word(client, SM5007_REG_STATUS);
	printk("%s, FG_STATUS:0x%x\r\n", __func__, ret);
	if (ret & ENABLE_L_VOL_STATUS || ret & ENABLE_SOC_ALARM) {
		fg_get_soc(fuelgauge->client);
		power_supply_changed(&fuelgauge->psy_fg);
	}

    return true;
}

static void sm5007_fg_irq_check(struct sm5007_fuelgauge_info *fuelgauge)
{
	int ret;

	pr_debug("%s: sm5007_fg_fuelalert_process\n", __func__);

	/*alert process */
	ret = sm5007_fg_i2c_read_word(fuelgauge->client, SM5007_REG_INTFG);
	printk("%s: intfg = %x\n", __func__, ret);
	/*read fg status*/
	ret = sm5007_fg_i2c_read_word(fuelgauge->client, SM5007_REG_STATUS);
	printk("%s, FG_STATUS:0x%x\r\n", __func__, ret);
	if (ret & ENABLE_L_VOL_STATUS || ret & ENABLE_SOC_ALARM) {
		//update soc first, and then report
		fg_get_soc(fuelgauge->client);
		power_supply_changed(&fuelgauge->psy_fg);
    }
}
static bool sm5007_fg_init(struct i2c_client *client)
{
	int ret;
	union power_supply_propval value;
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	/*sm5007 i2c read check*/
	ret = fg_get_device_id(client);
	if (ret < 0) {
		pr_err("%s: fail to do i2c read(%d)\n", __func__, ret);
		/*return false;*/
	}

	/*ENABLE_FUELGAUGE check*/
	ret = sm5007_fg_i2c_read_word(client, SM5007_REG_CNTL);
	ret |= (ENABLE_FUELGAUGE);
	sm5007_fg_i2c_write_word(client, SM5007_REG_CNTL, ret);

	if (sm5007_fg_check_reg_init_need(client))
		sm5007_fg_reg_init(client);

	value.intval = POWER_SUPPLY_HEALTH_UNKNOWN;
	/*UNFIXED*/
	/*psy_do_property("sm5007-charger", get,
			POWER_SUPPLY_PROP_HEALTH, value);*/
	pr_debug("%s: get POWER_SUPPLY_PROP_HEALTH = 0x%x\n",
		__func__, value.intval);

	fuelgauge->is_charging = (fuelgauge->info.batt_current > 10) ? true : false;

#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("%s: is_charging = %d\n", __func__, fuelgauge->is_charging);
#endif
	/* get first voltage measure to avgvoltage*/
	fuelgauge->info.batt_avgvoltage = fg_get_vbat(client);

	/* get first temperature*/
	fuelgauge->info.temperature = fg_get_temp(client);

	return true;
}

bool sm5007_fg_full_charged(struct i2c_client *client)
{
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	fuelgauge->info.flag_full_charge = 1;

	pr_debug("%s: full_charged\n", __func__);

	return true;
}
#ifdef CONFIG_SM5007_FG_N_VS_LPM
bool sm5007_en_lp_mode(struct device *dev, bool is_enable)
{
	int ret;
	struct sm5007_fuelgauge_info *fuelgauge = dev_get_drvdata(dev);

	ret = sm5007_fg_i2c_read_word(fuelgauge->client, SM5007_REG_CNTL);
	if(is_enable) {
		ret |= ENABLE_LP_MODE;
		sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_CNTL, ret);
		ret |= ENABLE_RS_MAN_MODE;//+RS_MAN_MODE
		sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_CNTL, ret);

		// RS manual value write, change lp mode slope.
		sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_RS_MAN, fuelgauge->info.rs_value[0]>>4);
		// run update
		sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_PARAM_RUN_UPDATE, 0);
		sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_PARAM_RUN_UPDATE, 1);
	} else {
		// RS manual value write
		sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_RS_MAN, fuelgauge->info.rs_value[0]);
		// run update
		sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_PARAM_RUN_UPDATE, 0);
		sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_PARAM_RUN_UPDATE, 1);

		ret &= (~ENABLE_RS_MAN_MODE);//+RS_AUTO_MODE
		sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_CNTL, ret);
		ret &= (~ENABLE_LP_MODE);
		sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_CNTL, ret);
	}

	return true;
}

void sm5007_en_fuelgauge(struct device *dev, bool is_enable) 
{
	int ret = 0;
	struct sm5007_fuelgauge_info *fuelgauge = dev_get_drvdata(dev);

	ret = sm5007_fg_i2c_read_word(fuelgauge->client, SM5007_REG_CNTL);
	if(is_enable)
		ret |= ENABLE_FUELGAUGE;
	else
		ret &= (~ENABLE_FUELGAUGE);

	sm5007_fg_i2c_write_word(fuelgauge->client, SM5007_REG_CNTL, ret);

	pr_debug("%s:\n", __func__);

}
#endif
bool sm5007_hal_fg_reset(struct i2c_client *client)
{
	pr_err("%s: sm5007_hal_fg_reset\n", __func__);

	/* SW reset code*/
	sm5007_fg_i2c_write_word(client, 0x91, 0x1341);
	/* delay 200ms*/
	msleep(200);
	/* init code*/
	sm5007_fg_init(client);

	return true;
}

bool sm5007_hal_fg_init(struct i2c_client *client)
{
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	pr_debug("sm5007 sm5007_hal_fg_init...\n");
	mutex_init(&fuelgauge->info.param_lock);
	mutex_lock(&fuelgauge->info.param_lock);
	sm5007_fg_init(client);
	sm5007_pr_ver_info(client);
	fuelgauge->info.temperature = 250;

	mutex_unlock(&fuelgauge->info.param_lock);
	pr_debug("sm5007 hal fg init OK\n");
	return true;
}

static void sm5007_work(struct work_struct *work)
{
	struct sm5007_fuelgauge_info *fuelgauge;
	int pre_soc = 0;
	fuelgauge = container_of(work,
		struct sm5007_fuelgauge_info, monitor_work.work);

	pre_soc = (fuelgauge->info.batt_soc/10);

	if (pre_soc != (fg_get_soc(fuelgauge->client)/10)){
		power_supply_changed(&fuelgauge->psy_fg);
	}
	pr_debug("sm5007_work for FG");

	schedule_delayed_work(&fuelgauge->monitor_work, SM5007_DELAY);
}

static void sm5007_external_power_changed(struct power_supply *psy) {
    //printk("external_power_changed\n");
    power_supply_changed(psy);
}

static enum power_supply_property sm5007_fuelgauge_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
};

char * supplied_from[] = {
	"sm5007-charger",
};

static int sm5007_fg_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sm5007_fuelgauge_info *fuelgauge = container_of(psy,
			struct sm5007_fuelgauge_info, psy_fg);

	/*pr_debug("%s: power_supply_property = %d\n",__func__, psp);*/
	switch (psp) {
	/*case POWER_SUPPLY_PROP_ONLINE:
		val->intval = fuelgauge->info.online;
		break;*/
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = fuelgauge->info.batt_voltage * 1000;//mV->uV
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = fuelgauge->info.batt_avgvoltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = fuelgauge->info.batt_ocv * 1000; //mV->uV
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = fuelgauge->info.batt_current * 1000;//mA->uA
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = fuelgauge->info.batt_avgcurrent * 1000;//mA->uA
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = (fuelgauge->info.batt_soc >= 1000) ? true : false;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
        //larger than 100%, force to 100%
		if(fuelgauge->info.batt_soc > 1000) {
			fuelgauge->info.batt_soc = 1000;
		}
        //smaller than 1% but not 0%, force to 1%
		if(fuelgauge->info.batt_soc < 10 && fuelgauge->info.batt_soc > 0)
			fuelgauge->info.batt_soc = 10;
		val->intval = (((fuelgauge->info.batt_soc % 10) > 4)? 1:0) + fuelgauge->info.batt_soc/10;
		break;
		/* Battery Temperature */
	case POWER_SUPPLY_PROP_TEMP:
		/* Target Temperature */
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
		val->intval = fuelgauge->info.temperature;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		/*for test: 1 is Low vbat*/
		val->intval = fuelgauge->info.volt_alert_flag;
		break;
    case POWER_SUPPLY_PROP_STATUS:
        psy_do_property("sm5007-charger", get, POWER_SUPPLY_PROP_STATUS, (*val));
		break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sm5007_fg_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct sm5007_fuelgauge_info *fuelgauge = container_of(psy,
				struct sm5007_fuelgauge_info, psy_fg);
	/*struct sm5007_fuelgauge_info *fuelgauge =
		dev_get_drvdata(psy->dev);*/

	switch (psp) {
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
	pr_debug("POWER_SUPPLY_PROP_ENERGY_EMPTY val->intval = %d\n",val->intval);
	if (val->intval == 1)
		sm5007_fg_irq_check(fuelgauge);
	break;
	default:
		return -EINVAL;
	}
	return 0;
}

static ssize_t sm5007_fg_reset_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret=0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	sm5007_hal_fg_reset(client);
	return ret;
}

static DEVICE_ATTR(fg_reset, S_IRUGO, sm5007_fg_reset_show, NULL);

static ssize_t sm5007_fg_reg_show(struct device *dev,struct device_attribute *attr, char *buf) 
{
	int i = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	printk("+++++++sm5007_fuelgauge_reg dump +++++++++\r\n");
	for (i = 0; i < 0x0A; i++)
		printk("0x%x: 0x%x\r\n", i, sm5007_fg_i2c_read_word(client, i));
	for (i = 0xC; i < 0x0F; i++)
		printk("0x%x: 0x%x\r\n", i, sm5007_fg_i2c_read_word(client, i));
	printk("0x%x: 0x%x\r\n", 0x10, sm5007_fg_i2c_read_word(client, 0x10));
	for (i = 0x12; i < 0x15; i++)
		printk("0x%x: 0x%x\r\n", i, sm5007_fg_i2c_read_word(client, i));
	for (i = 0x1A; i < 0x1D; i++)
		printk("0x%x: 0x%x\r\n", i, sm5007_fg_i2c_read_word(client, i));
	for (i = 0xA0; i < 0xD0; i++)
		printk("0x%x: 0x%x\r\n", i, sm5007_fg_i2c_read_word(client, i));
	printk("+++++++sm5007_fuelgauge_reg dump +++++++++\r\n");

	return 0;
}

static ssize_t sm5007_fg_reg_store(struct device *dev,struct device_attribute *attr, char *buf, size_t count)
{
	int retval;
	ssize_t num_read_chars = 0;
	u8 valbuf[7]= {0};
	long unsigned int wmreg = 0;
	int reg_addr = 0;
	int reg_value = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	memset(valbuf, 0, sizeof(valbuf));
	num_read_chars = count - 1;

	if (num_read_chars != 2) {
		if (num_read_chars != 6) { 
			printk("Please input right addr and value\r\n");
			return count;
		}
	}
	memcpy(valbuf, buf, num_read_chars);

	retval = strict_strtoul(valbuf, 16, &wmreg);
	if (0 != retval) {
		printk("Please input right addr and value\r\n");
		return count;
	}

	printk("regaddr: 0x%x\r\n", (unsigned int)wmreg);

	if (num_read_chars == 2) {
		reg_addr = wmreg;
		reg_value = sm5007_fg_i2c_read_word(client, reg_addr);
		printk("regaddr: 0x%x, regvalue: 0x%x\r\n", reg_addr, reg_value);
	} else {
		reg_addr = wmreg >> 16;
		reg_value = wmreg&(0x00FFFF);
		printk("regaddr: 0x%x, regvalue: 0x%x\r\n", reg_addr, reg_value);
		sm5007_fg_i2c_write_word(client, reg_addr, reg_value);
	}

	return count;
}

static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR, sm5007_fg_reg_show, sm5007_fg_reg_store);

static ssize_t sm5007_fg_ocv_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	int ret = fg_get_ocv(client);
	return sprintf(buf, "%d\n", ret);
}

static DEVICE_ATTR(fg_ocv, S_IRUGO, sm5007_fg_ocv_show, NULL);

struct sm5007_storage sm5007_uboot_last_charging_mmc = {0};
int mmc_load_sm5007(struct sm5007_storage *sm5007_mmc);
static ssize_t sm5007_charging_time_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(sm5007_uboot_last_charging_mmc.last_time == 0)
		mmc_load_sm5007(&sm5007_uboot_last_charging_mmc);
	return sprintf(buf, "%d", sm5007_uboot_last_charging_mmc.last_time);
}
static DEVICE_ATTR(charging_time, S_IRUGO, sm5007_charging_time_show, NULL);

static ssize_t sm5007_charging_level_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	unsigned int last_soc = 0;
	unsigned int batt_level = 0;

	if(sm5007_uboot_last_charging_mmc.last_soc == 0)
		mmc_load_sm5007(&sm5007_uboot_last_charging_mmc);
	last_soc = sm5007_uboot_last_charging_mmc.last_soc;
	batt_level = (last_soc / 10) + ((last_soc % 10) > 4 ? 1 : 0);
	if (batt_level > 100)
		batt_level = 100;
	return sprintf(buf, "%d", batt_level);
}
static DEVICE_ATTR(charging_level, S_IRUGO, sm5007_charging_level_show, NULL);
static struct attribute *sm5007_fg_attr[] = {
	&dev_attr_fg_reset.attr,
	&dev_attr_reg.attr,
	&dev_attr_fg_ocv.attr,
	&dev_attr_charging_time.attr,
	&dev_attr_charging_level.attr,
	NULL
};


static const struct attribute_group sm5007_fg_attr_grp = {
	.attrs = sm5007_fg_attr,
};

#define CONFIG_SM5007_MMC_ADDR (10*1024*1024 + 2*512)
#define CONFIG_STORE_SM5007_MMC_ADDR (10*1024*1024 + 3*512)
int mmc_save_sm5007(void)
{
	struct rtc_device * rtc = alarmtimer_get_rtcdev();
	struct rtc_time tm;
	struct mmc_card *mmc = NULL;
	struct sm5007_storage sm5007_mmc = {0};
	unsigned long time;

	rtc_read_time(rtc, &tm);
	rtc_tm_to_time(&tm, &time);

	sm5007_mmc.last_ocv = fg_get_ocv(sm5007_fuelgauge->client);
	sm5007_mmc.last_soc = u32_fg_get_soc(sm5007_fuelgauge->client);
	sm5007_mmc.last_time = time;
	printk("sm5007: kernel save: last ocv: %d, last_soc:%d, last_time:%ld\n", sm5007_mmc.last_ocv, sm5007_mmc.last_soc, sm5007_mmc.last_time);
	mmc = find_emmc_card();
	if (mmc != NULL) {
		mmc_write(mmc, (unsigned char *)&sm5007_mmc, CONFIG_SM5007_MMC_ADDR, sizeof(struct sm5007_storage));
		if (get_power_supply_status(sm5007_charger) & SM5007_STATE_VBUSPOK)
			mmc_write(mmc, (unsigned char *)&sm5007_mmc, CONFIG_STORE_SM5007_MMC_ADDR, sizeof(struct sm5007_storage));
	} else {
		printk("sm5007: kernel save failed\n");
	}
	return 0;
}
EXPORT_SYMBOL(mmc_save_sm5007);

int mmc_clear_sm5007(void)
{
	struct mmc_card *mmc = NULL;
	struct sm5007_storage sm5007_mmc = {0};

	mmc = find_emmc_card();
	if (mmc != NULL)
		mmc_write(mmc, (unsigned char *)&sm5007_mmc, CONFIG_SM5007_MMC_ADDR, sizeof(struct sm5007_storage));
}

int mmc_load_sm5007(struct sm5007_storage *sm5007_mmc)
{
	struct mmc_card *mmc = NULL;

	mmc = find_emmc_card();
	if (mmc != NULL) {
		mmc_read(mmc, (unsigned char *)sm5007_mmc, CONFIG_STORE_SM5007_MMC_ADDR, sizeof(struct sm5007_storage));
		printk("sm5007: kernel load: last ocv: %d, last_soc:%d, last_time:%ld\n", sm5007_mmc->last_ocv, sm5007_mmc->last_soc, sm5007_mmc->last_time);
	} else {
		printk("sm5007: can't read last information\n");
	}

	return 0;
}

static int sm5007_fuelgauge_probe (struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct sm5007_fuelgauge_info *fuelgauge;
	struct sm5007_fuelgauge_platform_data *pdata;
	int ret, type_n, temp, i, j;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	fuelgauge = kzalloc(sizeof(*fuelgauge), GFP_KERNEL);
	if (!fuelgauge)
		return -ENOMEM;

	fuelgauge->client = client;
	i2c_set_clientdata(client, fuelgauge);
	fuelgauge->dev = &client->dev;
	pdata = client->dev.platform_data;

	mutex_init(&fuelgauge->fg_lock);
	/* check rage of battery type */
	type_n = Battery_Type();
	temp = sizeof(pdata->type)/(sizeof(struct sm5007_fuelgauge_type_data));
	if(type_n  >= temp)
	{
		printk("%s : Battery type num is out of range\n", __func__);
		type_n = 0;
	}

	printk("%s type_n=%d,temp is %d\n", __func__, type_n,temp);

	/***********these valuse are set in platform ************/
	/* get battery_table*/
	for (i = DISCHARGE_TABLE; i < TABLE_MAX; i++) {
		for (j = 0; j <= SM5007_FG_TABLE_LEN; j++) {
			fuelgauge->info.battery_table[i][j] = battery_table_para[i][j];
			pr_debug("<table[%d][%d] 0x%x>\n", i, j, fuelgauge->info.battery_table[i][j]);
		}
	}

	/*alarm_vol_mv*/
	fuelgauge->info.alarm_vol_mv = pdata->alarm_vol_mv;

    printk("alarm_vol:%d\n", fuelgauge->info.alarm_vol_mv);    
	/* get rce*/
	fuelgauge->info.rce_value[0] = pdata->type[type_n].rce_value[0];
	fuelgauge->info.rce_value[1] = pdata->type[type_n].rce_value[1];
	fuelgauge->info.rce_value[2] = pdata->type[type_n].rce_value[2];
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("rce_value = <0x%x 0x%x 0x%x>\n",
		fuelgauge->info.rce_value[0], fuelgauge->info.rce_value[1],
		fuelgauge->info.rce_value[2]);
#endif
	/* get dtcd_value*/
	fuelgauge->info.dtcd_value = pdata->type[type_n].dtcd_value;
	pr_debug("dtcd_value = <0x%x>\n", fuelgauge->info.dtcd_value);

	/* get rs_value*/
	fuelgauge->info.rs_value[0] = pdata->type[type_n].rs_value[0];
	fuelgauge->info.rs_value[1] = pdata->type[type_n].rs_value[1];
	fuelgauge->info.rs_value[2] = pdata->type[type_n].rs_value[2];
	fuelgauge->info.rs_value[3] = pdata->type[type_n].rs_value[3];
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("rs_value = <0x%x 0x%x 0x%x 0x%x>\n",
		fuelgauge->info.rs_value[0], fuelgauge->info.rs_value[1],
		fuelgauge->info.rs_value[2], fuelgauge->info.rs_value[3]);
#endif
	/* get vit_period*/
	fuelgauge->info.vit_period = pdata->type[type_n].vit_period;
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("vit_period = <0x%x>\n", fuelgauge->info.vit_period);
#endif
	/* get mix_value*/
	fuelgauge->info.mix_value[0] = pdata->type[type_n].mix_value[0];
	fuelgauge->info.mix_value[1] = pdata->type[type_n].mix_value[1];
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("mix_value = <0x%x 0x%x>\n",
		fuelgauge->info.mix_value[0], fuelgauge->info.mix_value[1]);
#endif
	/* battery_type*/
	fuelgauge->info.battery_type = pdata->type[type_n].battery_type;
	pr_debug("battery_type = <0x%d>\n", fuelgauge->info.battery_type);

	/* VOL & CURR CAL*/
	fuelgauge->info.volt_cal = pdata->type[type_n].volt_cal;
	fuelgauge->info.curr_cal = pdata->type[type_n].curr_cal;
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("volt_cal = <0x%x>\n", fuelgauge->info.volt_cal);
	printk("curr_cal = <0x%x>\n", fuelgauge->info.curr_cal);
#endif
	/* temp_std*/
	fuelgauge->info.temp_std = pdata->type[type_n].temp_std;
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("temp_std = <0x%x>\n", fuelgauge->info.temp_std);
#endif

	/* temp_offset*/
	fuelgauge->info.temp_offset = pdata->type[type_n].temp_offset;
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	pr_debug("temp_offset = <0x%x>\n", fuelgauge->info.temp_offset);
#endif
	/* temp_offset_cal*/
	fuelgauge->info.temp_offset_cal = pdata->type[type_n].temp_offset_cal;
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("temp_offset_cal = <0x%x>\n", fuelgauge->info.temp_offset_cal);
#endif

	fuelgauge->info.temp_offset_cal_ofs = pdata->type[type_n].temp_offset_cal_ofs;
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("temp_offset_cal_ofs = <0x%x>\n", fuelgauge->info.temp_offset_cal_ofs);
#endif

	/* charge_offset_cal*/
	fuelgauge->info.charge_offset_cal = pdata->type[type_n].charge_offset_cal;
#ifdef CONFIG_SM5007_FG_DEBUG_LOG
	printk("charge_offset_cal = <0x%x>\n", fuelgauge->info.charge_offset_cal);
#endif

	/*TOPOFF_ENABLE*/
	fuelgauge->info.enable_topoff_soc= pdata->type[type_n].topoff_soc[0];

	/*TOPOFF_SOC*/
	fuelgauge->info.topoff_soc = pdata->type[type_n].topoff_soc[1];
	/**************************************************/

	/*INIT*/
	if (!sm5007_hal_fg_init(client))
		dev_err(fuelgauge->dev, "Failed to Initialize Fuelgauge\n");


	fuelgauge->psy_fg.name		= "sm5007-fuelgauge";
	fuelgauge->psy_fg.type		= POWER_SUPPLY_TYPE_BATTERY;
	fuelgauge->psy_fg.get_property	= sm5007_fg_get_property;
	fuelgauge->psy_fg.set_property	= sm5007_fg_set_property;
	fuelgauge->psy_fg.properties	= sm5007_fuelgauge_props;
	fuelgauge->psy_fg.num_properties = ARRAY_SIZE(sm5007_fuelgauge_props);
    fuelgauge->psy_fg.external_power_changed = sm5007_external_power_changed;
    fuelgauge->psy_fg.supplied_from = supplied_from;
    fuelgauge->psy_fg.num_supplies = ARRAY_SIZE(supplied_from);

	/* Register the battery with the power supply class. */
	ret = power_supply_register(&client->dev, &fuelgauge->psy_fg);
	if (ret) {
		dev_err(fuelgauge->dev, "failed: power supply register\n");
		power_supply_unregister(&fuelgauge->psy_fg);
		kfree(fuelgauge);
		return ret;
	}

	sm5007_fuelgauge = fuelgauge;
	/*IRQ INIT*/
	fuelgauge->is_fuel_alerted = false;
	sm5007_fg_fuelalert_init(fuelgauge->client, pdata);
	m_client = fuelgauge->client;
	fuelgauge->initial_update_of_soc = true;


	INIT_DELAYED_WORK(&fuelgauge->monitor_work, sm5007_work);
	schedule_delayed_work(&fuelgauge->monitor_work, SM5007_DELAY);

#ifdef CONFIG_SM5007_FG_N_VS_LPM
	/*V_ALARM_TOGGLE*/
	ret = sm5007_fg_i2c_read_word(client, SM5007_REG_CNTL);
	ret &= (~ENABLE_V_ALARM);
	sm5007_fg_i2c_write_word(client, SM5007_REG_CNTL, ret);
	ret |= ENABLE_V_ALARM;
	sm5007_fg_i2c_write_word(client, SM5007_REG_CNTL, ret);
#endif

	ret = sysfs_create_group(&fuelgauge->dev->kobj, &sm5007_fg_attr_grp);

	if (ret)
		dev_err(fuelgauge->dev, "fail to create sysfs\n");
	pr_debug("%s: SM5007 Fuelgauge Driver Loaded\n", __func__);

	return 0;
}

static int sm5007_fuelgauge_suspend(struct device *dev)
{
	struct sm5007_fuelgauge_info *fuelgauge = dev_get_drvdata(dev);
	pr_debug("sm5007_fuelgauge_suspend");
	cancel_delayed_work(&fuelgauge->monitor_work);
#ifdef CONFIG_SM5007_FG_N_VS_LPM
	sm5007_en_lp_mode(dev, true);
#endif
	return 0;
}

static int sm5007_fuelgauge_resume(struct device *dev)
{
	struct sm5007_fuelgauge_info *fuelgauge = dev_get_drvdata(dev);
	pr_debug("sm5007_fuelgauge_resume\n");
#ifdef CONFIG_SM5007_FG_N_VS_LPM
	sm5007_en_lp_mode(dev, false);
#endif
	schedule_work(&fuelgauge->monitor_work);
	return 0;
}

static int sm5007_fuelgauge_remove(struct i2c_client *client)
{
	struct sm5007_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	pr_debug("sm5007_fuelgauge_remove\n");
	sysfs_remove_group(&fuelgauge->dev->kobj, &sm5007_fg_attr_grp);
	power_supply_unregister(&fuelgauge->psy_fg);
	cancel_delayed_work(&fuelgauge->monitor_work);
	kfree(fuelgauge);
	return 0;
}

static const struct i2c_device_id sm5007_fuelgague_i2c_id[] = {
	{"sm5007-fuelgauge", 0},
	{}
};

static const struct dev_pm_ops sm5007_fuelgauge_pm_ops = {
	.suspend = sm5007_fuelgauge_suspend,
	.resume	 = sm5007_fuelgauge_resume,
};

MODULE_DEVICE_TABLE(i2c, sm5007_fuelgague_i2c_id);

static struct i2c_driver sm5007_fuelgauge_driver = {
	.probe			= sm5007_fuelgauge_probe,
	.remove			= sm5007_fuelgauge_remove,
	.id_table = sm5007_fuelgague_i2c_id,
	.driver = {
		.name	= ALIAS_NAME,
		.owner	= THIS_MODULE,
		.pm = &sm5007_fuelgauge_pm_ops,
	},
};
static int __init sm5007_fuelgauge_init(void)
{
	int ret = -ENODEV;

	pr_debug("%s\n", __func__);

	ret = i2c_add_driver(&sm5007_fuelgauge_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);

	return ret;
}

static void __exit sm5007_fuelgauge_exit(void)
{
	i2c_del_driver(&sm5007_fuelgauge_driver);
}

module_init(sm5007_fuelgauge_init);
module_exit(sm5007_fuelgauge_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sm5007 Fuel Gauge Driver");
MODULE_ALIAS(ALIAS_NAME);

