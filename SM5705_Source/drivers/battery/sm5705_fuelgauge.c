/* drivers/battery/sm5705_fuelgauge.c
 * SM5705 Voltage Tracking Fuelgauge Driver
 *
 * Copyright (C) 2013
 * Author: Dongik Sin <dongik.sin@samsung.com>
 * Modified by SW Jung
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/battery/fuelgauge/sm5705_fuelgauge.h>
#include <linux/battery/fuelgauge/sm5705_fuelgauge_impl.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/math64.h>
#include <linux/compiler.h>
#include <linux/of_gpio.h>

#define SM5705_FG_DEVICE_NAME "sm5705-fg"
#define ALIAS_NAME "sm5705-fuelgauge"

#define FG_DET_BAT_PRESENT 1
#define MINVAL(a, b) ((a <= b) ? a : b)

enum battery_table_type {
	DISCHARGE_TABLE = 0,
	Q_TABLE,
	TABLE_MAX,
};

static struct device_attribute sec_fg_attrs[] = {
	SEC_FG_ATTR(reg),
	SEC_FG_ATTR(data),
	SEC_FG_ATTR(regs),
};

static enum power_supply_property sm5705_fuelgauge_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
};



static inline int sm5705_fg_read_device(struct i2c_client *client,
				      int reg, int bytes, void *dest)
{
	int ret;

	if (bytes > 1)
		ret = i2c_smbus_read_i2c_block_data(client, reg, bytes, dest);
	else {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			return ret;
		*(unsigned char *)dest = (unsigned char)ret;
	}
	return ret;
}

static int32_t sm5705_fg_i2c_read_word(struct i2c_client *client,
                        uint8_t reg_addr)
{
	uint16_t data = 0;
	int ret;
	ret = sm5705_fg_read_device(client, reg_addr, 2, &data);
	//pr_info("%s: ret = %d, addr = 0x%x, data = 0x%x\n", __func__, ret, reg_addr, data);

	if (ret < 0)
		return ret;
	else
		return data;

	// not use big endian
	//return (int32_t)be16_to_cpu(data);
}


static int32_t sm5705_fg_i2c_write_word(struct i2c_client *client,
                            uint8_t reg_addr,uint16_t data)
{
	int ret;

	// not use big endian
	//data = cpu_to_be16(data);
	ret = i2c_smbus_write_i2c_block_data(client, reg_addr, 2, (uint8_t *)&data);
//	pr_info("%s: ret = %d, addr = 0x%x, data = 0x%x\n", __func__, ret, reg_addr, data);

	return ret;
}

static int32_t sm5705_fg_i2c_verified_write_word(struct i2c_client *client,
		uint8_t reg_addr,uint16_t data)
{
	int ret;

	// not use big endian
	//data = cpu_to_be16(data);
	ret = i2c_smbus_write_i2c_block_data(client, reg_addr, 2, (uint8_t *)&data);
	if(ret<0)
	{
		msleep(50);
		pr_info("1st fail i2c write %s: ret = %d, addr = 0x%x, data = 0x%x\n", __func__, ret, reg_addr, data);
		ret = i2c_smbus_write_i2c_block_data(client, reg_addr, 2, (uint8_t *)&data);
		if(ret<0)
		{
			msleep(50);
			pr_info("2nd fail i2c write %s: ret = %d, addr = 0x%x, data = 0x%x\n", __func__, ret, reg_addr, data);
			ret = i2c_smbus_write_i2c_block_data(client, reg_addr, 2, (uint8_t *)&data);
			if(ret<0)
			{
				pr_info("3rd fail i2c write %s: ret = %d, addr = 0x%x, data = 0x%x\n", __func__, ret, reg_addr, data);
			}
		}
	}
	//	pr_info("%s: ret = %d, addr = 0x%x, data = 0x%x\n", __func__, ret, reg_addr, data);

	return ret;
}

static unsigned int sm5705_get_all_value(struct i2c_client *client);
static unsigned int sm5705_get_vbat(struct i2c_client *client);
static unsigned int sm5705_get_ocv(struct i2c_client *client);
static int sm5705_get_curr(struct i2c_client *client);
static int sm5705_get_temperature(struct i2c_client *client);
static unsigned int sm5705_get_soc(struct i2c_client *client);

static void sm5705_pr_ver_info(struct i2c_client *client)
{
	pr_info("SM5705 Fuel-Gauge Ver %s\n", FG_DRIVER_VER);
}

static unsigned int sm5705_get_ocv(struct i2c_client *client)
{
	int ret;
	unsigned int ocv;// = 3500; /*3500 means 3500mV*/
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	ret = sm5705_fg_i2c_read_word(client, SM5705_REG_OCV);
	if (ret<0) {
		pr_err("%s: read ocv reg fail\n", __func__);
		ocv = 4000;
	} else {
		ocv = ((ret&0x7800)>>11) * 1000; //integer;
		ocv = ocv + (((ret&0x07ff)*1000)/2048); // integer + fractional
	}

	fuelgauge->info.batt_ocv = ocv;
	pr_info("%s: read = 0x%x, ocv = %d\n", __func__, ret, ocv);

	return ocv;
}

void sm5705_cal_avg_vbat(struct sec_fuelgauge_info *fuelgauge)
{
	if (fuelgauge->info.batt_avgvoltage == 0)
		fuelgauge->info.batt_avgvoltage = fuelgauge->info.batt_voltage;

	else if (fuelgauge->info.batt_voltage == 0 && fuelgauge->info.p_batt_voltage == 0)
		fuelgauge->info.batt_avgvoltage = 3400;

	else if(fuelgauge->info.batt_voltage == 0)
		fuelgauge->info.batt_avgvoltage = ((fuelgauge->info.batt_avgvoltage) + (fuelgauge->info.p_batt_voltage))/2;

	else if(fuelgauge->info.p_batt_voltage == 0)
		fuelgauge->info.batt_avgvoltage = ((fuelgauge->info.batt_avgvoltage) + (fuelgauge->info.batt_voltage))/2;

	else
		fuelgauge->info.batt_avgvoltage = ((fuelgauge->info.batt_avgvoltage*2) + (fuelgauge->info.p_batt_voltage+fuelgauge->info.batt_voltage))/4;

	pr_info("%s: batt_avgvoltage = %d\n", __func__, fuelgauge->info.batt_avgvoltage);

	return;
}

static unsigned int sm5705_get_vbat(struct i2c_client *client)
{
	int ret;
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	unsigned int vbat;/* = 3500; 3500 means 3500mV*/
	
	ret = sm5705_fg_i2c_read_word(client, SM5705_REG_VOLTAGE);
	if (ret<0) {
		pr_err("%s: read vbat reg fail", __func__);
		vbat = 4000;
	} else {
		vbat = ((ret&0x3800)>>11) * 1000; //integer;
		vbat = vbat + (((ret&0x07ff)*1000)/2048); // integer + fractional
	}
	fuelgauge->info.batt_voltage = vbat;

	pr_info("%s: read = 0x%x, vbat = %d\n", __func__, ret, vbat);
	sm5705_cal_avg_vbat(fuelgauge);

	if ((fuelgauge->volt_alert_flag == true) && vbat > 3400) {
		fuelgauge->volt_alert_flag = false;
		sec_hal_fg_fuelalert_init(client,
				fuelgauge->pdata->fuel_alert_soc);
		pr_info("%s : volt_alert_flag = %d \n", __func__, fuelgauge->volt_alert_flag);
	}

	return vbat;
}

void sm5705_cal_avg_current(struct sec_fuelgauge_info *fuelgauge)
{
	if (fuelgauge->info.batt_avgcurrent == 0)
		fuelgauge->info.batt_avgcurrent = fuelgauge->info.batt_current;

	else if (fuelgauge->info.batt_avgcurrent == 0 && fuelgauge->info.p_batt_current == 0)
		fuelgauge->info.batt_avgcurrent = fuelgauge->info.batt_current;

	else if(fuelgauge->info.batt_current == 0)
		fuelgauge->info.batt_avgcurrent = ((fuelgauge->info.batt_avgcurrent) + (fuelgauge->info.p_batt_current))/2;

	else if(fuelgauge->info.p_batt_current == 0)
		fuelgauge->info.batt_avgcurrent = ((fuelgauge->info.batt_avgcurrent) + (fuelgauge->info.batt_current))/2;

	else
		fuelgauge->info.batt_avgcurrent = ((fuelgauge->info.batt_avgcurrent*2) + (fuelgauge->info.p_batt_current+fuelgauge->info.batt_current))/4;

	pr_info("%s: batt_avgcurrent = %d\n", __func__, fuelgauge->info.batt_avgcurrent);

	return;
}


static int sm5705_get_curr(struct i2c_client *client)
{
	int ret;
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	int curr;/* = 1000; 1000 means 1000mA*/
	
	ret = sm5705_fg_i2c_read_word(client, SM5705_REG_CURRENT);
	if (ret<0) {
		pr_err("%s: read curr reg fail", __func__);
		curr = 0;
	} else {
		curr = ((ret&0x1800)>>11) * 1000; //integer;
		curr = curr + (((ret&0x07ff)*1000)/2048); // integer + fractional

		if(ret&0x8000) {
			curr *= -1;
		}
	}
	fuelgauge->info.batt_current = curr;

	pr_info("%s: read = 0x%x, curr = %d\n", __func__, ret, curr);

	sm5705_cal_avg_current(fuelgauge);

	return curr;
}

static int sm5705_get_temperature(struct i2c_client *client)
{
	int ret;
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	int temp;/* = 250; 250 means 25.0oC*/
	//double temp_data;
	
	ret = sm5705_fg_i2c_read_word(client, SM5705_REG_TEMPERATURE);
	if (ret<0) {
		pr_err("%s: read temp reg fail", __func__);
		temp = 0;
	} else {
		temp = ((ret&0x7F00)>>8) * 10; //integer bit
		temp = temp + (((ret&0x00f0)*10)/256); // integer + fractional bit
		if(ret&0x8000) {
			temp *= -1;
		}
	}
	fuelgauge->info.temp_fg = temp;

	pr_info("%s: read = 0x%x, temp_fg = %d\n", __func__, ret, temp);

	return temp;
}

static void sm5705_fg_test_read(struct i2c_client *client)
{
	int ret0, ret1, ret2, ret3, ret4;

	ret0 = sm5705_fg_i2c_read_word(client, 0xA0);
	ret1 = sm5705_fg_i2c_read_word(client, 0xAC);
	ret2 = sm5705_fg_i2c_read_word(client, 0xAD);
	ret3 = sm5705_fg_i2c_read_word(client, 0xAE);
	ret4 = sm5705_fg_i2c_read_word(client, 0xAF);
	pr_info("0xA0=0x%04x, 0xAC=0x%04x, 0xAD=0x%04x, 0xAE=0x%04x, 0xAF=0x%04x \n", ret0, ret1, ret2, ret3, ret4);

	ret0 = sm5705_fg_i2c_read_word(client, 0xB0);
	ret1 = sm5705_fg_i2c_read_word(client, 0xBC);
	ret2 = sm5705_fg_i2c_read_word(client, 0xBD);
	ret3 = sm5705_fg_i2c_read_word(client, 0xBE);
	ret4 = sm5705_fg_i2c_read_word(client, 0xBF);
	pr_info("0xB0=0x%04x, 0xBC=0x%04x, 0xBD=0x%04x, 0xBE=0x%04x, 0xBF=0x%04x \n", ret0, ret1, ret2, ret3, ret4);

	ret1 = sm5705_fg_i2c_read_word(client, 0x85);
	ret2 = sm5705_fg_i2c_read_word(client, 0x86);
	ret3 = sm5705_fg_i2c_read_word(client, 0x87);
	ret4 = sm5705_fg_i2c_read_word(client, 0x28);
	pr_info("0x85=0x%04x, 0x86=0x%04x, 0x87=0x%04x, 0x28=0x%04x \n", ret1, ret2, ret3, ret4);

	return;
}

unsigned int sm5705_get_soc(struct i2c_client *client)
{
	int ret;
	unsigned int soc;

	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	ret = sm5705_fg_i2c_read_word(client, SM5705_REG_SOC);
	if (ret<0) {
		pr_err("%s: Warning!!!! read soc reg fail\n", __func__);
		soc = 500;
	} else {
		soc = ((ret&0xff00)>>8) * 10; //integer bit;
		soc = soc + (((ret&0x00ff)*10)/256); // integer + fractional bit
	}

	pr_info("%s: read = 0x%x, soc = %d\n", __func__, ret, soc);
	fuelgauge->info.batt_soc = soc;

	//temp for SM5705 FG debug
	sm5705_fg_test_read(client);

	// for low temp power off test
	if(fuelgauge->info.volt_alert_flag && (fuelgauge->info.temperature < -100))
	{
		pr_info("%s: volt_alert_flag is TRUE!!!! SOC make force ZERO!!!!\n", __func__);
		fuelgauge->info.batt_soc = 0;
		return 0;
	}

	return soc;
}

static bool sm5705_fg_check_reg_init_need(struct i2c_client *client)
{
	int ret;

	ret = sm5705_fg_i2c_read_word(client, SM5705_REG_FG_OP_STATUS);
	pr_info("%s: SM5705_REG_FG_OP_STATUS : 0x%x\n", __func__, ret);

	if((ret & INIT_CHECK_MASK) == DISABLE_RE_INIT)
	{
		pr_info("%s: return 0\n", __func__);
		return 0;
	}
	else
	{
		pr_info("%s: return 1\n", __func__);
		return 1;
	}
}

static void sm5705_fg_buffer_read(struct i2c_client *client)
{
	int ret0, ret1, ret2, ret3, ret4, ret5;

	ret0 = sm5705_fg_i2c_read_word(client, 0x30);
	ret1 = sm5705_fg_i2c_read_word(client, 0x31);
	ret2 = sm5705_fg_i2c_read_word(client, 0x32);
	ret3 = sm5705_fg_i2c_read_word(client, 0x33);
	ret4 = sm5705_fg_i2c_read_word(client, 0x34);
	ret5 = sm5705_fg_i2c_read_word(client, 0x35);
	pr_info("%s: sm5705 FG buffer 0x30_0x35 = 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x \n", __func__, ret0, ret1, ret2, ret3, ret4, ret5);

	ret0 = sm5705_fg_i2c_read_word(client, 0x36);
	ret1 = sm5705_fg_i2c_read_word(client, 0x37);
	ret2 = sm5705_fg_i2c_read_word(client, 0x38);
	ret3 = sm5705_fg_i2c_read_word(client, 0x39);
	ret4 = sm5705_fg_i2c_read_word(client, 0x3A);
	ret5 = sm5705_fg_i2c_read_word(client, 0x3B);
	pr_info("%s: sm5705 FG buffer 0x36_0x3B = 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x \n", __func__, ret0, ret1, ret2, ret3, ret4, ret5);


	ret0 = sm5705_fg_i2c_read_word(client, 0x40);
	ret1 = sm5705_fg_i2c_read_word(client, 0x41);
	ret2 = sm5705_fg_i2c_read_word(client, 0x42);
	ret3 = sm5705_fg_i2c_read_word(client, 0x43);
	ret4 = sm5705_fg_i2c_read_word(client, 0x44);
	ret5 = sm5705_fg_i2c_read_word(client, 0x45);
	pr_info("%s: sm5705 FG buffer 0x40_0x45 = 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x \n", __func__, ret0, ret1, ret2, ret3, ret4, ret5);


	ret0 = sm5705_fg_i2c_read_word(client, 0x46);
	ret1 = sm5705_fg_i2c_read_word(client, 0x47);
	ret2 = sm5705_fg_i2c_read_word(client, 0x48);
	ret3 = sm5705_fg_i2c_read_word(client, 0x49);
	ret4 = sm5705_fg_i2c_read_word(client, 0x4A);
	ret5 = sm5705_fg_i2c_read_word(client, 0x4B);
	pr_info("%s: sm5705 FG buffer 0x46_0x4B = 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x \n", __func__, ret0, ret1, ret2, ret3, ret4, ret5);

	return;
}

static unsigned int sm5705_get_device_id(struct i2c_client *client)
{
	int ret;
	ret = sm5705_fg_i2c_read_word(client, SM5705_REG_DEVICE_ID);
	//ret &= 0x00ff;
	pr_info("%s: device_id = 0x%x\n", __func__, ret);

	return ret;
}



static bool sm5705_fg_get_batt_present(struct i2c_client *client)
{
	// SM5705 is not suport batt present
	dev_dbg(&client->dev, "%s: sm5705_fg_get_batt_present\n", __func__);

	return true;
}


int sm5705_calculate_iocv(struct i2c_client *client)
{
	bool only_lb=false;
	int roop_start=0, roop_max=0, i=0;
	int v_max=0, v_min=0, v_sum=0, lb_v_avg=0, cb_v_avg=0, lb_v_minmax_offset=0;
	int i_max=0, i_min=0, i_sum=0, lb_i_avg=0, cb_i_avg=0, lb_i_minmax_offset=0;

	int v_ret=0, i_ret=0, ret=0;

	ret = sm5705_fg_i2c_read_word(client, SM5705_REG_END_V_IDX);
	pr_info("%s: iocv_status_read = addr : 0x%x , data : 0x%x\n", __func__, SM5705_REG_END_V_IDX, ret);

	if((ret & 0x0010) == 0x0000)
	{
		only_lb = true;
	}

	roop_max = (ret & 0x000F)-1;
	if(roop_max > 6)
		roop_max = 6;
 
	roop_start = SM5705_REG_IOCV_B_L_MIN;
	for (i = roop_start; i < roop_start + roop_max; i++)
	{
		v_ret = sm5705_fg_i2c_read_word(client, i);
		i_ret = sm5705_fg_i2c_read_word(client, i+0x10);

		if (i == roop_start)
		{
			v_max = v_ret;
			v_min = v_ret;
			v_sum = v_ret;
			i_max = i_ret;
			i_min = i_ret;
			i_sum = i_ret;
		}
		else
		{
			if(v_ret > v_max)
				v_max = v_ret;
			else if(v_ret < v_min)
				v_min = v_ret;
			v_sum = v_sum + v_ret;

			if(i_ret > i_max)
				i_max = i_ret;
			else if(i_ret < i_min)
				i_min = i_ret;
			i_sum = i_sum + i_ret;
		}
	}
	v_sum = v_sum - v_max - v_min;
	i_sum = i_sum - i_max - i_min;

	lb_v_minmax_offset = v_max - v_min;
	lb_i_minmax_offset = i_max - i_min;

	lb_v_avg = v_sum / (roop_max-2);
	lb_i_avg = i_sum / (roop_max-2);

	pr_info("%s: iocv_l_max=0x%x, iocv_l_min=0x%x, iocv_l_sum=0x%x, iocv_l_avg=0x%x \n", __func__, v_max, v_min, v_sum, lb_v_avg);
	pr_info("%s: ioci_l_max=0x%x, ioci_l_min=0x%x, ioci_l_sum=0x%x, ioci_l_avg=0x%x \n", __func__, i_max, i_min, i_sum, lb_i_avg);

	if(!only_lb)
	{
		roop_start = SM5705_REG_IOCV_B_C_MIN;
		roop_max = 6;
		for (i = roop_start; i < roop_start + roop_max; i++)
		{
			v_ret = sm5705_fg_i2c_read_word(client, i);
			i_ret = sm5705_fg_i2c_read_word(client, i+0x10);

			if (i == roop_start)
			{
				v_max = v_ret;
				v_min = v_ret;
				v_sum = v_ret;
				i_max = i_ret;
				i_min = i_ret;
				i_sum = i_ret;
			}
			else
			{
				if(v_ret > v_max)
					v_max = v_ret;
				else if(v_ret < v_min)
					v_min = v_ret;
				v_sum = v_sum + v_ret;
			
				if(i_ret > i_max)
					i_max = i_ret;
				else if(i_ret < i_min)
					i_min = i_ret;
				i_sum = i_sum + i_ret;
			}

		}
		v_sum = v_sum - v_max - v_min;
		i_sum = i_sum - i_max - i_min;
		
		cb_v_avg = v_sum / (roop_max-2);
		cb_i_avg = i_sum / (roop_max-2);

		pr_info("%s: iocv_c_max=0x%x, iocv_c_min=0x%x, iocv_c_sum=0x%x, iocv_c_avg=0x%x \n", __func__, v_max, v_min, v_sum, cb_v_avg);
		pr_info("%s: ioci_c_max=0x%x, ioci_c_min=0x%x, ioci_c_sum=0x%x, ioci_c_avg=0x%x \n", __func__, i_max, i_min, i_sum, cb_i_avg);

	}

	if((abs(lb_v_avg - cb_v_avg) > 0x20) || only_lb)
	{
		ret = lb_v_avg;
	}
	else
	{
		ret = cb_v_avg;
	}

	return ret;
}

static bool sm5705_fg_reg_init(struct i2c_client *client, int is_surge)
{
	int i, j, value, ret;
	uint8_t table_reg;
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	pr_info("%s: sm5705_fg_reg_init START!!\n", __func__);

	// start first param_ctrl unlock
	sm5705_fg_i2c_write_word(client, SM5705_REG_PARAM_CTRL, SM5705_FG_PARAM_UNLOCK_CODE);

	// RCE write
	for (i = 0; i < 3; i++)
	{
		sm5705_fg_i2c_write_word(client, SM5705_REG_RCE0+i, fuelgauge->info.rce_value[i]);
		pr_info("%s: RCE write RCE%d = 0x%x : 0x%x\n", __func__,  i, SM5705_REG_RCE0+i, fuelgauge->info.rce_value[i]);
	}

	// DTCD write
	sm5705_fg_i2c_write_word(client, SM5705_REG_DTCD, fuelgauge->info.dtcd_value);
	pr_info("%s: DTCD write DTCD = 0x%x : 0x%x\n", __func__, SM5705_REG_DTCD, fuelgauge->info.dtcd_value);

	// RS write
	sm5705_fg_i2c_write_word(client, SM5705_REG_AUTO_RS_MAN, fuelgauge->info.rs_value[0]);
	pr_info("%s: RS write RS = 0x%x : 0x%x\n", __func__, SM5705_REG_AUTO_RS_MAN, fuelgauge->info.rs_value[0]);

	// VIT_PERIOD write
	sm5705_fg_i2c_write_word(client, SM5705_REG_VIT_PERIOD, fuelgauge->info.vit_period);
	pr_info("%s: VIT_PERIOD write VIT_PERIOD = 0x%x : 0x%x\n", __func__, SM5705_REG_VIT_PERIOD, fuelgauge->info.vit_period);

	// TABLE_LEN write & pram unlock
	sm5705_fg_i2c_write_word(client, SM5705_REG_PARAM_CTRL, SM5705_FG_PARAM_UNLOCK_CODE | SM5705_FG_TABLE_LEN);

	for (i=0; i < TABLE_MAX; i++)
	{
		table_reg = SM5705_REG_TABLE_START + (i<<4);
		for(j=0; j <= SM5705_FG_TABLE_LEN; j++)
		{
			sm5705_fg_i2c_write_word(client, (table_reg + j), fuelgauge->info.battery_table[i][j]);
			msleep(10);
			if(fuelgauge->info.battery_table[i][j] != sm5705_fg_i2c_read_word(client, (table_reg + j)))
			{
				pr_info("%s: TABLE write FAIL retry[%d][%d] = 0x%x : 0x%x\n", __func__, i, j, (table_reg + j), fuelgauge->info.battery_table[i][j]);
				sm5705_fg_i2c_write_word(client, (table_reg + j), fuelgauge->info.battery_table[i][j]);
			}
			pr_info("%s: TABLE write OK [%d][%d] = 0x%x : 0x%x\n", __func__, i, j, (table_reg + j), fuelgauge->info.battery_table[i][j]);
		}
	}

	// MIX_MODE write
	sm5705_fg_i2c_write_word(client, SM5705_REG_RS_MIX_FACTOR, fuelgauge->info.rs_value[1]);
	sm5705_fg_i2c_write_word(client, SM5705_REG_RS_MAX, fuelgauge->info.rs_value[2]);
	sm5705_fg_i2c_write_word(client, SM5705_REG_RS_MIN, fuelgauge->info.rs_value[3]);
	sm5705_fg_i2c_write_word(client, SM5705_REG_MIX_RATE, fuelgauge->info.mix_value[0]);
	sm5705_fg_i2c_write_word(client, SM5705_REG_MIX_INIT_BLANK, fuelgauge->info.mix_value[1]);

	pr_info("%s: RS_MIX_FACTOR = 0x%x, RS_MAX = 0x%x, RS_MIN = 0x%x, MIX_RATE = 0x%x, MIX_INIT_BLANK = 0x%x\n",
		__func__, fuelgauge->info.rs_value[1], fuelgauge->info.rs_value[2], fuelgauge->info.rs_value[3], fuelgauge->info.mix_value[0], fuelgauge->info.mix_value[1]);

	// CAL write
	sm5705_fg_i2c_write_word(client, SM5705_REG_VOLT_CAL, fuelgauge->info.volt_cal);
	sm5705_fg_i2c_write_word(client, SM5705_REG_CURR_P_SLOPE, fuelgauge->info.p_curr_cal);
	sm5705_fg_i2c_write_word(client, SM5705_REG_CURR_N_SLOPE, fuelgauge->info.n_curr_cal);
	pr_info("%s: VOLT_CAL = 0x%x, p_curr_cal = 0x%x, n_curr_cal = 0x%x\n", __func__, fuelgauge->info.volt_cal, fuelgauge->info.p_curr_cal, fuelgauge->info.n_curr_cal);

	// TOPOFF SOC
	sm5705_fg_i2c_write_word(client, SM5705_REG_TOPOFFSOC, fuelgauge->info.topoff_soc);
	pr_info("%s: SM5705_REG_TOPOFFSOC 0x%x : 0x%x\n", __func__, SM5705_REG_TOPOFFSOC, fuelgauge->info.topoff_soc);

	// INIT_last -  control register set
	value = sm5705_fg_i2c_read_word(client, SM5705_REG_CNTL);
	value |= ENABLE_MIX_MODE | ENABLE_TEMP_MEASURE | (fuelgauge->info.enable_topoff_soc << 13);
	value |= ENABLE_MANUAL_OCV;
	pr_info("%s: SM5705_REG_CNTL reg : 0x%x\n", __func__, value);

	ret = sm5705_fg_i2c_write_word(client, SM5705_REG_CNTL, value);
	if (ret < 0)
		pr_info("%s: fail control register set(%d)\n", __func__, ret);

	pr_info("%s: LAST SM5705_REG_CNTL = 0x%x : 0x%x\n", __func__, SM5705_REG_CNTL, value);

	// LOCK
	value = SM5705_FG_PARAM_LOCK_CODE | SM5705_FG_TABLE_LEN;
	sm5705_fg_i2c_write_word(client, SM5705_REG_PARAM_CTRL, value);
	pr_info("%s: LAST PARAM CTRL VALUE = 0x%x : 0x%x\n", __func__, SM5705_REG_PARAM_CTRL, value);

	// surge reset defence
	if(is_surge)
	{
		value = ((fuelgauge->info.batt_ocv<<8)/125);
	}
	else
	{
		value = sm5705_calculate_iocv(client);
	}

	sm5705_fg_i2c_write_word(client, SM5705_REG_IOCV_MAN, value);
	pr_info("%s: IOCV_MAN_WRITE = %d : 0x%x\n", __func__, fuelgauge->info.batt_ocv, value);

	return 1;
}

static bool sm5705_fg_init(struct i2c_client *client, bool is_surge)
{
	int ret;
	union power_supply_propval value;
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	fuelgauge->info.is_FG_initialised = 0;
	fuelgauge->info.iocv_error_count = 0;

	//board_fuelgauge_init(fuelgauge);

	//SM5705 i2c read check
	ret = sm5705_get_device_id(client);
	if (ret < 0)
	{
		pr_info("%s: fail to do i2c read(%d)\n", __func__, ret);
		return false;
	}

	if(sm5705_fg_check_reg_init_need(client))
	{
		sm5705_fg_reg_init(client, is_surge);
	}

	value.intval = POWER_SUPPLY_HEALTH_UNKNOWN;
	psy_do_property("sm5705-charger", get,
			POWER_SUPPLY_PROP_HEALTH, value);
	pr_info("%s: get POWER_SUPPLY_PROP_HEALTH = 0x%x\n", __func__, value.intval);

	fuelgauge->is_charging = ((value.intval == POWER_SUPPLY_HEALTH_GOOD) |
		fuelgauge->ta_exist) && (fuelgauge->info.batt_current>=0);

	pr_info("%s: is_charging = %d, ta_exist = %d\n", __func__, fuelgauge->is_charging, fuelgauge->ta_exist);

	// get first measure value
	sm5705_get_all_value(client);

	fuelgauge->info.is_FG_initialised = 1;

	// for debug
	sm5705_fg_buffer_read(client);

	return true;
}

void sm5705_abnormal_reset_check(struct i2c_client *client)
{
	int ret;
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	// abnormal case.... SW reset
	if(sm5705_fg_check_reg_init_need(client) && (fuelgauge->info.is_FG_initialised == 1))
	{
		ret = sm5705_fg_i2c_read_word(client, SM5705_REG_CNTL);
		pr_info("%s: SM5705 FG abnormal case!!!! SM5705_REG_CNTL : 0x%x\n", __func__, ret);
		if(ret == CNTL_REG_DEFAULT_VALUE)
		{
			// SW reset code
			if(sm5705_fg_i2c_verified_write_word(client, SM5705_REG_RESET, SW_RESET_CODE) < 0)
			{
				pr_info("%s: Warning!!!! SM5705 FG abnormal case.... SW reset FAIL \n", __func__);
			}
			else
			{
				pr_info("%s: SM5705 FG abnormal case.... SW reset OK\n", __func__);
			}
			// delay 200ms
			msleep(200);
			// init code
			sm5705_fg_init(client, true);
		}
	}
}

void sm5705_vbatocv_check(struct i2c_client *client)
{
	int ret;
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	// iocv error case cover start
	if((abs(fuelgauge->info.batt_current)<40) || ((fuelgauge->is_charging) && (abs(fuelgauge->info.batt_current)<100)))
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

	pr_info("%s: iocv_error_count (%d)\n", __func__, fuelgauge->info.iocv_error_count);

	if(fuelgauge->info.iocv_error_count > 5)
	{
		pr_info("%s: p_v - v = (%d)\n", __func__, fuelgauge->info.p_batt_voltage - fuelgauge->info.batt_voltage);
		if(abs(fuelgauge->info.p_batt_voltage - fuelgauge->info.batt_voltage)>15) // 15mV over
		{
			fuelgauge->info.iocv_error_count = 0;
		}
		else
		{
			// mode change to mix RS manual mode
			pr_info("%s: mode change to mix RS manual mode\n", __func__);
			// RS manual value write
			sm5705_fg_i2c_write_word(client, SM5705_REG_RS_MAN, fuelgauge->info.rs_value[0]);
			// run update
			sm5705_fg_i2c_write_word(client, SM5705_REG_PARAM_RUN_UPDATE, 0);
			sm5705_fg_i2c_write_word(client, SM5705_REG_PARAM_RUN_UPDATE, 1);
			// mode change
			ret = sm5705_fg_i2c_read_word(client, SM5705_REG_CNTL);
			ret = (ret | ENABLE_MIX_MODE) | ENABLE_RS_MAN_MODE; // +RS_MAN_MODE
			sm5705_fg_i2c_write_word(client, SM5705_REG_CNTL, ret);
		}
	}
	else
	{
		if((fuelgauge->info.temperature/10) > 15)
		{
			if((fuelgauge->info.p_batt_voltage < fuelgauge->info.n_tem_poff) && (fuelgauge->info.batt_voltage < fuelgauge->info.n_tem_poff) && (!fuelgauge->is_charging))
			{
				pr_info("%s: mode change to normal tem mix RS manual mode\n", __func__);
				// mode change to mix RS manual mode
				// RS manual value write
				if((fuelgauge->info.p_batt_voltage < (fuelgauge->info.n_tem_poff - fuelgauge->info.n_tem_poff_offset)) && (fuelgauge->info.batt_voltage < (fuelgauge->info.n_tem_poff - fuelgauge->info.n_tem_poff_offset)))
				{
					sm5705_fg_i2c_write_word(client, SM5705_REG_RS_MAN, fuelgauge->info.rs_value[0]>>1);
				}
				else
				{
					sm5705_fg_i2c_write_word(client, SM5705_REG_RS_MAN, fuelgauge->info.rs_value[0]);
				}
				// run update
				sm5705_fg_i2c_write_word(client, SM5705_REG_PARAM_RUN_UPDATE, 0);
				sm5705_fg_i2c_write_word(client, SM5705_REG_PARAM_RUN_UPDATE, 1);

				// mode change
				ret = sm5705_fg_i2c_read_word(client, SM5705_REG_CNTL);
				ret = (ret | ENABLE_MIX_MODE) | ENABLE_RS_MAN_MODE; // +RS_MAN_MODE
				sm5705_fg_i2c_write_word(client, SM5705_REG_CNTL, ret);
			}
			else
			{
				pr_info("%s: mode change to mix RS auto mode\n", __func__);

				// mode change to mix RS auto mode
				ret = sm5705_fg_i2c_read_word(client, SM5705_REG_CNTL);
				ret = (ret | ENABLE_MIX_MODE) & ~ENABLE_RS_MAN_MODE; // -RS_MAN_MODE
				sm5705_fg_i2c_write_word(client, SM5705_REG_CNTL, ret);
			}
		}
		else
		{
			if((fuelgauge->info.p_batt_voltage < fuelgauge->info.l_tem_poff) && (fuelgauge->info.batt_voltage < fuelgauge->info.l_tem_poff) && (!fuelgauge->is_charging))
			{
				pr_info("%s: mode change to normal tem mix RS manual mode\n", __func__);
				// mode change to mix RS manual mode
				// RS manual value write
				if((fuelgauge->info.p_batt_voltage < (fuelgauge->info.l_tem_poff - fuelgauge->info.l_tem_poff_offset)) && (fuelgauge->info.batt_voltage < (fuelgauge->info.l_tem_poff - fuelgauge->info.l_tem_poff_offset)))
				{
					sm5705_fg_i2c_write_word(client, SM5705_REG_RS_MAN, fuelgauge->info.rs_value[0]>>1);
				}
				else
				{
					sm5705_fg_i2c_write_word(client, SM5705_REG_RS_MAN, fuelgauge->info.rs_value[0]);
				}
				// run update
				sm5705_fg_i2c_write_word(client, SM5705_REG_PARAM_RUN_UPDATE, 0);
				sm5705_fg_i2c_write_word(client, SM5705_REG_PARAM_RUN_UPDATE, 1);

				// mode change
				ret = sm5705_fg_i2c_read_word(client, SM5705_REG_CNTL);
				ret = (ret | ENABLE_MIX_MODE) | ENABLE_RS_MAN_MODE; // +RS_MAN_MODE
				sm5705_fg_i2c_write_word(client, SM5705_REG_CNTL, ret);
			}
			else
			{
				pr_info("%s: mode change to mix RS auto mode\n", __func__);

				// mode change to mix RS auto mode
				ret = sm5705_fg_i2c_read_word(client, SM5705_REG_CNTL);
				ret = (ret | ENABLE_MIX_MODE) & ~ENABLE_RS_MAN_MODE; // -RS_MAN_MODE
				sm5705_fg_i2c_write_word(client, SM5705_REG_CNTL, ret);
			}
		}
	}
	fuelgauge->info.p_batt_voltage = fuelgauge->info.batt_voltage;
	fuelgauge->info.p_batt_current = fuelgauge->info.batt_current;
	// iocv error case cover end
}

static void sm5705_cal_carc (struct i2c_client *client)
{
	int p_curr_cal=0, n_curr_cal=0, p_delta_cal=0, n_delta_cal=0;
	int temp_cal_fact;
	int temp_gap;

	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	sm5705_abnormal_reset_check(client);
	sm5705_vbatocv_check(client);

	n_curr_cal = fuelgauge->info.n_curr_cal;
	p_curr_cal = fuelgauge->info.p_curr_cal;

	temp_cal_fact = fuelgauge->info.temp_std - (fuelgauge->info.temp_fg/10);
	temp_cal_fact = temp_cal_fact / fuelgauge->info.temp_offset;
	temp_cal_fact = temp_cal_fact * fuelgauge->info.temp_offset_cal;
	p_curr_cal = p_curr_cal + (temp_cal_fact);
	n_curr_cal = n_curr_cal + (temp_cal_fact);

	pr_info("%s: temp_std = %d , temp_fg = %d , temp_offset = %d , temp_offset_cal = 0x%x, p_curr_cal = 0x%x, n_curr_cal = 0x%x, batt_temp = %d\n",
		__func__, fuelgauge->info.temp_std, fuelgauge->info.temp_fg, fuelgauge->info.temp_offset, fuelgauge->info.temp_offset_cal, p_curr_cal, n_curr_cal, fuelgauge->info.temperature);

	temp_gap = fuelgauge->info.temp_std - (fuelgauge->info.temperature/10);
	if (fuelgauge->info.en_high_temp_cal && (temp_gap < 0))	{
		p_delta_cal = (temp_gap / fuelgauge->info.high_temp_cal_denom)*fuelgauge->info.high_temp_p_cal_fact;
		n_delta_cal = (temp_gap / fuelgauge->info.high_temp_cal_denom)*fuelgauge->info.high_temp_n_cal_fact;
	}
	else if (fuelgauge->info.en_low_temp_cal && (temp_gap > 0))
	{
		p_delta_cal = (temp_gap / fuelgauge->info.low_temp_cal_denom)*fuelgauge->info.low_temp_p_cal_fact;
		n_delta_cal = (temp_gap / fuelgauge->info.low_temp_cal_denom)*fuelgauge->info.low_temp_n_cal_fact;
	}
	p_curr_cal = p_curr_cal + (p_delta_cal);
	n_curr_cal = n_curr_cal + (n_delta_cal);

	sm5705_fg_i2c_write_word(client, SM5705_REG_CURR_P_SLOPE, p_curr_cal);
	sm5705_fg_i2c_write_word(client, SM5705_REG_CURR_N_SLOPE, n_curr_cal);

	pr_info("%s: %d %d %d %d %d %d %d %d, p_curr_cal = 0x%x, n_curr_cal = 0x%x\n",
		__func__, fuelgauge->info.en_high_temp_cal, fuelgauge->info.high_temp_cal_denom, fuelgauge->info.high_temp_p_cal_fact, fuelgauge->info.high_temp_n_cal_fact, fuelgauge->info.en_low_temp_cal, fuelgauge->info.low_temp_cal_denom, fuelgauge->info.low_temp_p_cal_fact, fuelgauge->info.low_temp_n_cal_fact, p_curr_cal, n_curr_cal);

	return;
}

static unsigned int sm5705_get_all_value(struct i2c_client *client)
{
	union power_supply_propval value;
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	// check charging.
	value.intval = POWER_SUPPLY_HEALTH_UNKNOWN;
	psy_do_property("sm5705-charger", get,
			POWER_SUPPLY_PROP_HEALTH, value);
	pr_info("%s: get POWER_SUPPLY_PROP_HEALTH = 0x%x\n", __func__, value.intval);
	fuelgauge->is_charging = ((value.intval == POWER_SUPPLY_HEALTH_GOOD) |
		fuelgauge->ta_exist) && (fuelgauge->info.batt_current>=0);
	pr_info("%s: is_charging = %d, ta_exist = %d\n", __func__, fuelgauge->is_charging, fuelgauge->ta_exist);

	//carc
	sm5705_cal_carc(client);

	//vbat
	sm5705_get_vbat(client);
	//current
	sm5705_get_curr(client);
	//ocv
    sm5705_get_ocv(client);
	//temperature
	sm5705_get_temperature(client);
	//soc
	sm5705_get_soc(client);

    return 0;
}


#ifdef CONFIG_OF
static int get_battery_id(struct sec_fuelgauge_info *fuelgauge)
{
	// sm5705fg does not support this function
	return 0;
}
#define PROPERTY_NAME_SIZE 128

#define PINFO(format, args...) \
	printk(KERN_INFO "%s() line-%d: " format, \
		__func__, __LINE__, ## args)

#define DECL_PARAM_PROP(_id, _name) {.id = _id, .name = _name,}

static int sm5705_fg_parse_dt(struct sec_fuelgauge_info *fuelgauge)
{
	struct device *dev = &fuelgauge->client->dev;
	struct device_node *np = dev->of_node;
	char prop_name[PROPERTY_NAME_SIZE];
	int battery_id = -1;
	int table[16];
	int rce_value[3];
	int rs_value[4];
	int mix_value[2];
	int topoff_soc[2];
	int temp_cal[2];
	int ext_temp_cal[8];
	int set_temp_poff[4];

	int ret;
	int i, j;

	BUG_ON(dev == 0);
	BUG_ON(np == 0);

	// get battery_params node
	np = of_find_node_by_name(of_node_get(np), "battery_params");
	if (np == NULL) {
		PINFO("Cannot find child node \"battery_params\"\n");
		return -EINVAL;
	}

	// get battery_id
	if (of_property_read_u32(np, "battery,id", &battery_id) < 0)
		PINFO("not battery,id property\n");
	if (battery_id == -1)
		battery_id = get_battery_id(fuelgauge);
	PINFO("battery id = %d\n", battery_id);

	// get battery_table
	for (i = DISCHARGE_TABLE; i < TABLE_MAX; i++) {
		snprintf(prop_name, PROPERTY_NAME_SIZE,
			 "battery%d,%s%d", battery_id, "battery_table", i);

		ret = of_property_read_u32_array(np, prop_name, table, 16);
		if (ret < 0) {
			PINFO("Can get prop %s (%d)\n", prop_name, ret);
		}
		for (j = 0; j <= SM5705_FG_TABLE_LEN; j++)
		{
			fuelgauge->info.battery_table[i][j] = table[j];
			PINFO("%s = <table[%d][%d] 0x%x>\n", prop_name, i, j, table[j]);
		}
	}

	// get rce
	for (i = 0; i < 3; i++) {
		snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "rce_value");
		ret = of_property_read_u32_array(np, prop_name, rce_value, 3);
		if (ret < 0) {
			PINFO("Can get prop %s (%d)\n", prop_name, ret);
		}
        fuelgauge->info.rce_value[i] = rce_value[i];
	}
	PINFO("%s = <0x%x 0x%x 0x%x>\n", prop_name, rce_value[0], rce_value[1], rce_value[2]);

	// get dtcd_value
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "dtcd_value");
	ret = of_property_read_u32_array(np, prop_name, &fuelgauge->info.dtcd_value, 1);
	if (ret < 0)
		PINFO("Can get prop %s (%d)\n", prop_name, ret);
	PINFO("%s = <0x%x>\n",prop_name, fuelgauge->info.dtcd_value);

	// get rs_value
	for (i = 0; i < 4; i++) {
		snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "rs_value");
		ret = of_property_read_u32_array(np, prop_name, rs_value, 4);
		if (ret < 0) {
			PINFO("Can get prop %s (%d)\n", prop_name, ret);
		}
        fuelgauge->info.rs_value[i] = rs_value[i];
	}
	PINFO("%s = <0x%x 0x%x 0x%x 0x%x>\n", prop_name, rs_value[0], rs_value[1], rs_value[2], rs_value[3]);

	// get vit_period
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "vit_period");
	ret = of_property_read_u32_array(np, prop_name, &fuelgauge->info.vit_period, 1);
	if (ret < 0)
		PINFO("Can get prop %s (%d)\n", prop_name, ret);
	PINFO("%s = <0x%x>\n",prop_name, fuelgauge->info.vit_period);

	// get mix_value
	for (i = 0; i < 2; i++) {
		snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "mix_value");
		ret = of_property_read_u32_array(np, prop_name, mix_value, 2);
		if (ret < 0) {
			PINFO("Can get prop %s (%d)\n", prop_name, ret);
		}
        fuelgauge->info.mix_value[i] = mix_value[i];
	}
	PINFO("%s = <0x%x 0x%x>\n", prop_name, mix_value[0], mix_value[1]);

	// battery_type
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "battery_type");
	ret = of_property_read_u32_array(np, prop_name, &fuelgauge->info.battery_type, 1);
	if (ret < 0)
		PINFO("Can get prop %s (%d)\n", prop_name, ret);
	PINFO("%s = <%d>\n", prop_name, fuelgauge->info.battery_type);

	// TOP OFF SOC
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "topoff_soc");
	ret = of_property_read_u32_array(np, prop_name, topoff_soc, 2);
	if (ret < 0)
		PINFO("Can get prop %s (%d)\n", prop_name, ret);

	fuelgauge->info.enable_topoff_soc = topoff_soc[0];
	fuelgauge->info.topoff_soc = topoff_soc[1];
	PINFO("%s = <0x%x 0x%x>\n", prop_name, fuelgauge->info.enable_topoff_soc, fuelgauge->info.topoff_soc);

	// VOL & CURR CAL
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "volt_cal");
	ret = of_property_read_u32_array(np, prop_name, &fuelgauge->info.volt_cal, 1);
	if (ret < 0)
		PINFO("Can get prop %s (%d)\n", prop_name, ret);
	PINFO("%s = <0x%x>\n", prop_name, fuelgauge->info.volt_cal);

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "p_curr_cal");
	ret = of_property_read_u32_array(np, prop_name, &fuelgauge->info.p_curr_cal, 1);
	if (ret < 0)
		PINFO("Can get prop %s (%d)\n", prop_name, ret);
	PINFO("%s = <0x%x>\n", prop_name, fuelgauge->info.p_curr_cal);

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "n_curr_cal");
	ret = of_property_read_u32_array(np, prop_name, &fuelgauge->info.n_curr_cal, 1);
	if (ret < 0)
		PINFO("Can get prop %s (%d)\n", prop_name, ret);
	PINFO("%s = <0x%x>\n", prop_name, fuelgauge->info.n_curr_cal);

	// temp_std
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "temp_std");
	ret = of_property_read_u32_array(np, prop_name, &fuelgauge->info.temp_std, 1);
	if (ret < 0)
		PINFO("Can get prop %s (%d)\n", prop_name, ret);
	PINFO("%s = <%d>\n", prop_name, fuelgauge->info.temp_std);

	// temp_calc
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "temp_cal");
	ret = of_property_read_u32_array(np, prop_name, temp_cal, 2);
	if (ret < 0)
		PINFO("Can get prop %s (%d)\n", prop_name, ret);
	fuelgauge->info.temp_offset = temp_cal[0];
	fuelgauge->info.temp_offset_cal = temp_cal[1];
	PINFO("%s = <%d, %d>\n", prop_name, fuelgauge->info.temp_offset, fuelgauge->info.temp_offset_cal);

	// ext_temp_calc
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "ext_temp_cal");
	ret = of_property_read_u32_array(np, prop_name, ext_temp_cal, 8);
	if (ret < 0)
		PINFO("Can get prop %s (%d)\n", prop_name, ret);
	fuelgauge->info.en_high_temp_cal = ext_temp_cal[0];
	fuelgauge->info.high_temp_cal_denom = ext_temp_cal[1];
	fuelgauge->info.high_temp_p_cal_fact = ext_temp_cal[2];
	fuelgauge->info.high_temp_n_cal_fact = ext_temp_cal[3];
	fuelgauge->info.en_low_temp_cal = ext_temp_cal[4];
	fuelgauge->info.low_temp_cal_denom = ext_temp_cal[5];
	fuelgauge->info.low_temp_p_cal_fact = ext_temp_cal[6];
	fuelgauge->info.low_temp_n_cal_fact = ext_temp_cal[7];
	PINFO("%s = <%d, %d, %d, %d, %d, %d, %d, %d>\n", prop_name, fuelgauge->info.en_high_temp_cal, fuelgauge->info.high_temp_cal_denom, fuelgauge->info.high_temp_p_cal_fact, fuelgauge->info.high_temp_n_cal_fact, fuelgauge->info.en_low_temp_cal, fuelgauge->info.low_temp_cal_denom, fuelgauge->info.low_temp_p_cal_fact, fuelgauge->info.low_temp_n_cal_fact);

	// charge_offset_cal
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "charge_offset_cal");
	ret = of_property_read_u32_array(np, prop_name, &fuelgauge->info.charge_offset_cal, 1);
	if (ret < 0)
		PINFO("Can get prop %s (%d)\n", prop_name, ret);
	PINFO("%s = <0x%x>\n", prop_name, fuelgauge->info.charge_offset_cal);

	// tem poff level
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "tem_poff");
	ret = of_property_read_u32_array(np, prop_name, set_temp_poff, 4);
	if (ret < 0)
		PINFO("Can get prop %s (%d)\n", prop_name, ret);
	fuelgauge->info.n_tem_poff = set_temp_poff[0];
	fuelgauge->info.n_tem_poff_offset = set_temp_poff[1];
	fuelgauge->info.l_tem_poff = set_temp_poff[2];
	fuelgauge->info.l_tem_poff_offset = set_temp_poff[3];

	PINFO("%s = <%d, %d, %d, %d>\n", prop_name, fuelgauge->info.n_tem_poff, fuelgauge->info.n_tem_poff_offset, fuelgauge->info.l_tem_poff, fuelgauge->info.l_tem_poff_offset);

	return 0;
}
#else
static int sm5705_fg_parse_dt(struct sec_fuelgauge_info *fuelgauge)
{
	return 0;
}
#endif

bool sec_hal_fg_init(struct i2c_client *client)
{
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	struct battery_data_t *battery_data;
	pr_info("sm5705 sec_hal_fg_init...\n");
	mutex_init(&fuelgauge->info.param_lock);
	mutex_lock(&fuelgauge->info.param_lock);
	if (client->dev.of_node) {
	    // Load battery data from DTS
	    sm5705_fg_parse_dt(fuelgauge);

	} else {
		// Copy battery data from platform data
		battery_data = &get_battery_data(fuelgauge);
		fuelgauge->info.battery_type =
			battery_data->battery_type;
	}
	//struct battery_data_t *battery_data = &get_battery_data(fuelgauge);

	sm5705_fg_init(client, false);
	sm5705_pr_ver_info(client);
	fuelgauge->info.temperature = 250;

	mutex_unlock(&fuelgauge->info.param_lock);
	pr_info("sm5705 hal fg init OK\n");
	return true;
}

bool sec_hal_fg_suspend(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s: sec_hal_fg_suspend\n", __func__);

	return true;
}

bool sec_hal_fg_resume(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s: sec_hal_fg_resume\n", __func__);

	return true;
}

bool sec_hal_fg_fuelalert_init(struct i2c_client *client, int soc)
{
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	int ret;
	int value_v_alarm, value_soc_alarm;

	if (soc >= 0)
	{
		pr_info("%s: sec_hal_fg_fuelalert_init\n", __func__);

		// remove interrupt
		ret = sm5705_fg_i2c_read_word(client, SM5705_REG_INTFG);

		// check status
		ret = sm5705_fg_i2c_read_word(client, SM5705_REG_STATUS);

		// remove all mask
		sm5705_fg_i2c_write_word(client,SM5705_REG_INTFG_MASK, 0x0000);

		/* enable volt alert only, other alert mask*/
		ret = MASK_L_SOC_INT|MASK_H_TEM_INT|MASK_L_TEM_INT;
		sm5705_fg_i2c_write_word(client,SM5705_REG_INTFG_MASK,ret);
		fuelgauge->info.irq_ctrl = ~(ret);

		/* set volt and soc alert threshold */
		value_v_alarm = 0x02B4; // 2700mV
		sm5705_fg_i2c_write_word(client, SM5705_REG_V_ALARM, value_v_alarm);
		value_soc_alarm = 0x0100; // 1.00%
		sm5705_fg_i2c_write_word(client, SM5705_REG_SOC_ALARM, value_soc_alarm);

		/* update parameters */
		sm5705_fg_i2c_write_word(client, SM5705_REG_PARAM_RUN_UPDATE, 0);
		sm5705_fg_i2c_write_word(client, SM5705_REG_PARAM_RUN_UPDATE, 1);

		// enabel volt alert control, other alert disable
		ret = sm5705_fg_i2c_read_word(client, SM5705_REG_CNTL);
		ret = ret | ENABLE_V_ALARM;
		ret = ret & (~ENABLE_SOC_ALARM & ~ENABLE_T_H_ALARM & ~ENABLE_T_L_ALARM);
		sm5705_fg_i2c_write_word(client, SM5705_REG_CNTL, ret);

		pr_info("%s: irq_ctrl=0x%x, REG_CNTL=0x%x, V_ALARM=0x%x, SOC_ALARM=0x%x \n", __func__, fuelgauge->info.irq_ctrl, ret, value_v_alarm, value_soc_alarm);
	}

	/* alert flag init*/
	fuelgauge->info.soc_alert_flag = false;
	fuelgauge->is_fuel_alerted = false;

	return true;
}

bool sec_hal_fg_is_fuelalerted(struct i2c_client *client)
{
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	int ret;

	pr_info("%s: sec_hal_fg_is_fuelalerted\n", __func__);

	/* alert process */
	ret = sm5705_fg_i2c_read_word(client, SM5705_REG_INTFG);
	pr_info("%s: SM5705_REG_INTFG(0x%x)\n",
		__func__, ret);

	if(ret & fuelgauge->info.irq_ctrl)
	{
		// check status
		ret = sm5705_fg_i2c_read_word(client, SM5705_REG_STATUS);
		pr_info("%s: SM5705_REG_STATUS(0x%x)\n",
			__func__, ret);

		if(ret & fuelgauge->info.irq_ctrl)
		{
			return true;
		}
	}

	return false;
}

bool sec_hal_fg_fuelalert_process(void *irq_data, bool is_fuel_alerted)
{
	struct sec_fuelgauge_info *fuelgauge = irq_data;
	struct i2c_client *client = fuelgauge->client;
	int ret;

	pr_info("%s: sec_hal_fg_fuelalert_process :: is_fuel_alerted=%d \n", __func__, is_fuel_alerted);

	if(is_fuel_alerted)
	{
		ret = sm5705_fg_i2c_read_word(client, SM5705_REG_STATUS);
		pr_info("%s: SM5705_REG_STATUS(0x%x)\n",
			__func__, ret);

		/* not use SOC alarm
		if(ret & fuelgauge->info.irq_ctrl & ENABLE_SOC_ALARM) {
			fuelgauge->info.soc_alert_flag = true;
			// todo more action
		}
		*/

		if(ret & fuelgauge->info.irq_ctrl & ENABLE_V_ALARM) {
			fuelgauge->info.volt_alert_flag = true;
			// todo more action
		}
	}

	return true;
}

/* capacity is  0.1% unit */
static void sec_fg_get_scaled_capacity(
				struct sec_fuelgauge_info *fuelgauge,
				union power_supply_propval *val)
{
	val->intval = (val->intval < fuelgauge->pdata->capacity_min) ?
		0 : ((val->intval - fuelgauge->pdata->capacity_min) * 1000 /
		(fuelgauge->capacity_max - fuelgauge->pdata->capacity_min));

	pr_info("%s: scaled capacity (%d.%d)\n",
		__func__, val->intval/10, val->intval%10);
}

/* capacity is integer */
static void sec_fg_get_atomic_capacity(
				struct sec_fuelgauge_info *fuelgauge,
				union power_supply_propval *val)
{
	if (fuelgauge->pdata->capacity_calculation_type &
		SEC_FUELGAUGE_CAPACITY_TYPE_ATOMIC) {
		if (fuelgauge->capacity_old < val->intval)
			val->intval = fuelgauge->capacity_old + 1;
		else if (fuelgauge->capacity_old > val->intval)
			val->intval = fuelgauge->capacity_old - 1;
	}
	pr_info("%s: val.intval = %d\n", __func__, val->intval);

	/* keep SOC stable in abnormal status */
	if (fuelgauge->pdata->capacity_calculation_type &
		SEC_FUELGAUGE_CAPACITY_TYPE_SKIP_ABNORMAL) {
		if (!fuelgauge->ta_exist &&
			fuelgauge->capacity_old < val->intval) {
			pr_info("%s: capacity (old %d : new %d)\n",
				__func__, fuelgauge->capacity_old, val->intval);
			val->intval = fuelgauge->capacity_old;
		}
	}

	/* updated old capacity */
	fuelgauge->capacity_old = val->intval;
}

static int sec_fg_check_capacity_max(
				struct sec_fuelgauge_info *fuelgauge, int capacity_max)
{
	int new_capacity_max = capacity_max;

	if (new_capacity_max < (fuelgauge->pdata->capacity_max -
			fuelgauge->pdata->capacity_max_margin - 10)) {
		new_capacity_max =
			(fuelgauge->pdata->capacity_max -
			fuelgauge->pdata->capacity_max_margin);

		pr_info("%s: set capacity max(%d --> %d)\n",
			__func__, capacity_max, new_capacity_max);
	} else if (new_capacity_max > (fuelgauge->pdata->capacity_max +
			fuelgauge->pdata->capacity_max_margin)) {
		new_capacity_max =
			(fuelgauge->pdata->capacity_max +
			fuelgauge->pdata->capacity_max_margin);

		pr_info("%s: set capacity max(%d --> %d)\n",
			__func__, capacity_max, new_capacity_max);
	}

	return new_capacity_max;
}

static int sec_fg_calculate_dynamic_scale(
				struct sec_fuelgauge_info *fuelgauge, int capacity)
{
	union power_supply_propval raw_soc_val;

	raw_soc_val.intval = SEC_FUELGAUGE_CAPACITY_TYPE_RAW;
	if (!sec_hal_fg_get_property(fuelgauge->client,
		POWER_SUPPLY_PROP_CAPACITY,
		&raw_soc_val))
		return -EINVAL;
	raw_soc_val.intval /= 10;

	if (raw_soc_val.intval <
		fuelgauge->pdata->capacity_max -
		fuelgauge->pdata->capacity_max_margin) {
		fuelgauge->capacity_max =
			fuelgauge->pdata->capacity_max -
			fuelgauge->pdata->capacity_max_margin;
		dev_dbg(&fuelgauge->client->dev, "%s: capacity_max (%d)",
			__func__, fuelgauge->capacity_max);
	} else {
		fuelgauge->capacity_max =
			(raw_soc_val.intval >
			fuelgauge->pdata->capacity_max +
			fuelgauge->pdata->capacity_max_margin) ?
			(fuelgauge->pdata->capacity_max +
			fuelgauge->pdata->capacity_max_margin) :
			raw_soc_val.intval;
		dev_dbg(&fuelgauge->client->dev, "%s: raw soc (%d)",
			__func__, fuelgauge->capacity_max);
	}

	if (capacity != 100) {
		fuelgauge->capacity_max = sec_fg_check_capacity_max(
			fuelgauge, (fuelgauge->capacity_max * 100 / capacity));
	} else {
		fuelgauge->capacity_max =
			(fuelgauge->capacity_max * 99 / 100);
	}
	/* update capacity_old for sec_fg_get_atomic_capacity algorithm */
	fuelgauge->capacity_old = capacity;

	pr_info("%s: %d is used for capacity_max\n",
		__func__, fuelgauge->capacity_max);

	return fuelgauge->capacity_max;
}

bool sec_hal_fg_full_charged(struct i2c_client *client)
{
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	fuelgauge->info.flag_full_charge = 1;

	dev_dbg(&client->dev, "%s: full_charged\n", __func__);

	return true;
}

bool sm5705_fg_reset(struct i2c_client *client)
{
	pr_info("%s: sec_hal_fg_reset\n", __func__);

	// SW reset code
	sm5705_fg_i2c_verified_write_word(client, SM5705_REG_RESET, SW_RESET_CODE);
	// delay 200ms
	msleep(200);
	// init code
	sm5705_fg_init(client, false);

	return true;
}

static void sm5705_fg_reset_capacity_by_jig_connection(struct sec_fuelgauge_info *fuelgauge)
{
	union power_supply_propval value;
	int ret;

	ret = sm5705_fg_i2c_read_word(fuelgauge->client, SM5705_REG_CNTL);
	ret |= 0x0010;
	sm5705_fg_i2c_write_word(fuelgauge->client, SM5705_REG_CNTL, ret);
	/* If JIG is attached, the voltage is set as 1079 */
	value.intval = 1079;
	psy_do_property("battery", set,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, value);
}

bool sec_hal_fg_get_property(struct i2c_client *client,
			     enum power_supply_property psp,
			     union power_supply_propval *val)
{
	union power_supply_propval value;
	struct sec_fuelgauge_info *fuelgauge =
		i2c_get_clientdata(client);

	psy_do_property("sm5705-charger", get,
			POWER_SUPPLY_PROP_STATUS, value);
	fuelgauge->info.flag_full_charge =
		(value.intval == POWER_SUPPLY_STATUS_FULL) ? 1 : 0;
	fuelgauge->info.flag_chg_status =
		(value.intval == POWER_SUPPLY_STATUS_CHARGING) ? 1 : 0;

	pr_info("%s: psp=%d, val->intval=%d\n", __func__, psp, val->intval);
	sm5705_get_all_value(client);

	switch (psp) {
	/* Cell voltage (VCELL, mV) */
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = fuelgauge->info.batt_voltage;
		break;
	/* Additional Voltage Information (mV) */
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		switch (val->intval) {
		case SEC_BATTERY_VOLTAGE_AVERAGE:
			val->intval = fuelgauge->info.batt_avgvoltage;
			break;
		case SEC_BATTERY_VOLTAGE_OCV:
			val->intval = fuelgauge->info.batt_ocv;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_PRESENT:
            // SM5705 is not suport this prop
            sm5705_fg_get_batt_present(client);
		break;
		/* Current (mA) */
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = fuelgauge->info.batt_current;
		/* Average Current (mA) */
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		sm5705_get_curr(client);
		val->intval = fuelgauge->info.batt_avgcurrent;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval =
			(fuelgauge->info.batt_soc >= 1000) ? true : false;
		break;
		/* SOC (%) */
	case POWER_SUPPLY_PROP_CAPACITY:
		/* SM5705 F/G unit is 0.1%, raw ==> convert the unit to 0.01% */
		if (val->intval == SEC_FUELGAUGE_CAPACITY_TYPE_RAW)
			val->intval = fuelgauge->info.batt_soc * 10;
		else
			val->intval = fuelgauge->info.batt_soc;
		pr_info("%s: %d, val->intval=%d, batt_soc=%d\n", __func__, POWER_SUPPLY_PROP_CAPACITY, val->intval, fuelgauge->info.batt_soc);
		break;
		/* Battery Temperature */
	case POWER_SUPPLY_PROP_TEMP:
		/* Target Temperature */
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
		val->intval = fuelgauge->info.temp_fg;
		break;
	default:
		return false;
	}
	return true;
}

static int sm5705_fg_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct sec_fuelgauge_info *fuelgauge =
		container_of(psy, struct sec_fuelgauge_info, psy_fg);
	int soc_type = val->intval;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_ENERGY_NOW:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
		pr_info("%s: soc_type = %d\n", __func__, soc_type);

		if (!sec_hal_fg_get_property(fuelgauge->client, psp, val))
			return -EINVAL;
		if (psp == POWER_SUPPLY_PROP_CAPACITY) {
			if (soc_type == SEC_FUELGAUGE_CAPACITY_TYPE_RAW)
				break;

			pr_info("%s: capacity_calculation_type = %d\n", __func__, fuelgauge->pdata->capacity_calculation_type);

			if (fuelgauge->pdata->capacity_calculation_type &
				(SEC_FUELGAUGE_CAPACITY_TYPE_SCALE |
				 SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE))
				sec_fg_get_scaled_capacity(fuelgauge, val);

			pr_info("%s: val.intval = %d\n", __func__, val->intval);
			/* capacity should be between 0% and 100%
			 * (0.1% degree)
			 */
			if (val->intval > 1000)
				val->intval = 1000;
			if (val->intval < 0)
				val->intval = 0;

			/* get only integer part */
			val->intval /= 10;
			pr_info("%s: val.intval = %d\n", __func__, val->intval);

			/* check whether doing the wake_unlock */
			if ((val->intval > fuelgauge->pdata->fuel_alert_soc) &&
				fuelgauge->is_fuel_alerted) {
				wake_unlock(&fuelgauge->fuel_alert_wake_lock);
				sec_hal_fg_fuelalert_init(fuelgauge->client,
					fuelgauge->pdata->fuel_alert_soc);
			}
			pr_info("%s: val.intval = %d\n", __func__, val->intval);

			/* (Only for atomic capacity)
			 * In initial time, capacity_old is 0.
			 * and in resume from sleep,
			 * capacity_old is too different from actual soc.
			 * should update capacity_old
			 * by val->intval in booting or resume.
			 */
			if (fuelgauge->initial_update_of_soc) {
				/* updated old capacity */
				fuelgauge->capacity_old = val->intval;
				fuelgauge->initial_update_of_soc = false;
				break;
			}

			if (fuelgauge->pdata->capacity_calculation_type &
				(SEC_FUELGAUGE_CAPACITY_TYPE_ATOMIC |
				 SEC_FUELGAUGE_CAPACITY_TYPE_SKIP_ABNORMAL))
				sec_fg_get_atomic_capacity(fuelgauge, val);

		}
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		val->intval = fuelgauge->capacity_max;
		break;
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		return -ENODATA;
	default:
		return -EINVAL;
	}
	return 0;
}

bool sec_hal_fg_set_property(struct i2c_client *client,
			     enum power_supply_property psp,
			     const union power_supply_propval *val)
{
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	pr_info("%s: psp=%d\n", __func__, psp);

	switch (psp) {
		/* Battery Temperature */
	case POWER_SUPPLY_PROP_TEMP:
		fuelgauge->info.temperature = val->intval;
                break;
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
                break;
	default:
		return false;
	}
	return true;
}

ssize_t sec_hal_fg_show_attrs(struct device *dev,
				const ptrdiff_t offset, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct sec_fuelgauge_info *fg =
		container_of(psy, struct sec_fuelgauge_info, psy_fg);
	int i = 0;

    dev_dbg(dev, "%s: offset=%d\n", __func__, (int)offset);

	switch (offset) {
	case FG_REG:
		break;
	case FG_DATA:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       fg->info.batt_soc);

		break;
	default:
		i = -EINVAL;
		break;
	}
	return i;
}

ssize_t sec_hal_fg_store_attrs(struct device *dev,
				const ptrdiff_t offset,
				const char *buf, size_t count)
{
	int ret = 0;

        dev_dbg(dev, "%s: offset=%d\n", __func__, (int)offset);

	switch (offset) {
	case FG_REG:
		break;
	case FG_DATA:
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int sm5705_fg_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
	struct sec_fuelgauge_info *fuelgauge =
		container_of(psy, struct sec_fuelgauge_info, psy_fg);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (fuelgauge->pdata->capacity_calculation_type &
				SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE) {
#if defined(CONFIG_PREVENT_SOC_JUMP)
				sec_fg_calculate_dynamic_scale(fuelgauge, val->intval);
#else
				sec_fg_calculate_dynamic_scale(fuelgauge, 100);
#endif
			}
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		fuelgauge->cable_type = val->intval;
		if (val->intval == POWER_SUPPLY_TYPE_BATTERY)
			fuelgauge->ta_exist = false;
		else
			fuelgauge->ta_exist = true;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (val->intval == SEC_FUELGAUGE_CAPACITY_TYPE_RESET) {
			fuelgauge->initial_update_of_soc = true;
			if (!sm5705_fg_reset(fuelgauge->client))
				return -EINVAL;
			else
				break;
		}
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
		if (!sec_hal_fg_set_property(fuelgauge->client, psp, val))
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		pr_info(  
				"%s: capacity_max changed, %d -> %d\n",
				__func__, fuelgauge->capacity_max, val->intval);
		fuelgauge->capacity_max = sec_fg_check_capacity_max(fuelgauge, val->intval);
		fuelgauge->initial_update_of_soc = true;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		sm5705_fg_reset_capacity_by_jig_connection(fuelgauge);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void sm5705_fg_isr_work(struct work_struct *work)
{
	struct sec_fuelgauge_info *fuelgauge =
		container_of(work, struct sec_fuelgauge_info, isr_work.work);

	/* process for fuel gauge chip */
	sec_hal_fg_fuelalert_process(fuelgauge, fuelgauge->is_fuel_alerted);

	/* process for others */
	if (fuelgauge->pdata->fuelalert_process != NULL)
		fuelgauge->pdata->fuelalert_process(fuelgauge->is_fuel_alerted);
}

static irqreturn_t sm5705_fg_irq_thread(int irq, void *irq_data)
{
	struct sec_fuelgauge_info *fuelgauge = irq_data;
	bool fuel_alerted;

	if (fuelgauge->pdata->fuel_alert_soc >= 0) {
		fuel_alerted =
			sec_hal_fg_is_fuelalerted(fuelgauge->client);

		pr_info(  
			"%s: Fuel-alert %salerted!\n",
			__func__, fuel_alerted ? "" : "NOT ");

		if (fuel_alerted == fuelgauge->is_fuel_alerted) {
			if (!fuelgauge->pdata->repeated_fuelalert) {
				dev_dbg(&fuelgauge->client->dev,
					"%s: Fuel-alert Repeated (%d)\n",
					__func__, fuelgauge->is_fuel_alerted);
				return IRQ_HANDLED;
			}
		}

		if (fuel_alerted)
			wake_lock(&fuelgauge->fuel_alert_wake_lock);
		else
			wake_unlock(&fuelgauge->fuel_alert_wake_lock);

		fuelgauge->is_fuel_alerted = fuel_alerted;

		schedule_delayed_work(&fuelgauge->isr_work, 0);
	}

	return IRQ_HANDLED;
}

static int sm5705_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(sec_fg_attrs); i++) {
		rc = device_create_file(dev, &sec_fg_attrs[i]);
		if (rc)
			goto create_attrs_failed;
	}
	goto create_attrs_succeed;

create_attrs_failed:
	dev_err(dev, "%s: failed (%d)\n", __func__, rc);
	while (i--)
		device_remove_file(dev, &sec_fg_attrs[i]);
create_attrs_succeed:
	return rc;
}

ssize_t sec_fg_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	const ptrdiff_t offset = attr - sec_fg_attrs;
	int i = 0;

	switch (offset) {
	case FG_REG:
	case FG_DATA:
	case FG_REGS:
		i = sec_hal_fg_show_attrs(dev, offset, buf);
		break;
	default:
		i = -EINVAL;
		break;
	}

	return i;
}

ssize_t sec_fg_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	const ptrdiff_t offset = attr - sec_fg_attrs;
	int ret = 0;

	switch (offset) {
	case FG_REG:
	case FG_DATA:
		ret = sec_hal_fg_store_attrs(dev, offset, buf, count);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#ifdef CONFIG_OF
static int fuelgauge_parse_dt(struct device *dev,
			struct sec_fuelgauge_info *fuelgauge)
{
	struct device_node *np = dev->of_node;
	sec_battery_platform_data_t *pdata = fuelgauge->pdata;
	int ret;

	/* reset, irq gpio info */
	if (np == NULL) {
		pr_err("%s np NULL\n", __func__);
	} else {
		ret = of_get_named_gpio(np, "fuelgauge,fuel_int", 0);
		if (ret > 0) {
			pdata->fg_irq = ret;
			pr_info("%s reading fg_irq = %d\n", __func__, ret);
		}

		ret = of_get_named_gpio(np, "fuelgauge,bat_int", 0);
		if (ret > 0) {
			pdata->bat_irq_gpio = ret;
			pdata->bat_irq = gpio_to_irq(ret);
			pr_info("%s reading bat_int_gpio = %d\n", __func__, ret);
		}

		ret = of_property_read_u32(np, "fuelgauge,capacity_max",
				&fuelgauge->pdata->capacity_max);
		if (ret < 0)
			pr_err("%s error reading capacity_max %d\n", __func__, ret);

		ret = of_property_read_u32(np, "fuelgauge,capacity_max_margin",
				&fuelgauge->pdata->capacity_max_margin);
		if (ret < 0)
			pr_err("%s error reading capacity_max_margin %d\n", __func__, ret);

		ret = of_property_read_u32(np, "fuelgauge,capacity_min",
				&fuelgauge->pdata->capacity_min);
		if (ret < 0)
			pr_err("%s error reading capacity_min %d\n", __func__, ret);

		pr_info("%s: capacity_max: %d, "
				"capacity_max_margin: 0x%x, "
				"capacity_min: %d\n", __func__, pdata->capacity_max,
				pdata->capacity_max_margin,
				pdata->capacity_min
				);

		ret = of_property_read_u32(np, "fuelgauge,capacity_calculation_type",
				&pdata->capacity_calculation_type);
		if (ret < 0)
			pr_err("%s error reading capacity_calculation_type %d\n",
					__func__, ret);
		ret = of_property_read_u32(np, "fuelgauge,fuel_alert_soc",
				&pdata->fuel_alert_soc);
		if (ret < 0)
			pr_err("%s error reading pdata->fuel_alert_soc %d\n",
					__func__, ret);
		pdata->repeated_fuelalert = of_property_read_bool(np,
				"fuelgaguge,repeated_fuelalert");

		pr_info("%s: fg_irq: %d, "
				"calculation_type: 0x%x, fuel_alert_soc: %d,\n"
				"repeated_fuelalert: %d\n", __func__, pdata->fg_irq,
				pdata->capacity_calculation_type,
				pdata->fuel_alert_soc, pdata->repeated_fuelalert
				);
	}
	return 0;
}
#else
static int fuelgauge_parse_dt(struct device *dev,
			struct synaptics_rmi4_power_data *pdata)
{
	return -ENODEV;
}
#endif

static int sm5705_fuelgauge_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct sec_fuelgauge_info *fuelgauge;
	sec_battery_platform_data_t *pdata = NULL;
//	struct battery_data_t *battery_data = NULL;
	int ret = 0;
	union power_supply_propval raw_soc_val;

	pr_info(
		"%s: SM5705 Fuelgauge Driver Loading\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	fuelgauge = kzalloc(sizeof(*fuelgauge), GFP_KERNEL);
	if (!fuelgauge)
		return -ENOMEM;

	mutex_init(&fuelgauge->fg_lock);

	fuelgauge->client = client;

	if (client->dev.of_node) {
		int error;
		pdata = devm_kzalloc(&client->dev,
			sizeof(sec_battery_platform_data_t),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			ret = -ENOMEM;
			goto err_free;
		}

		fuelgauge->pdata = pdata;
		error = fuelgauge_parse_dt(&client->dev, fuelgauge);
		if (error) {
			dev_err(&client->dev,
				"%s: Failed to get fuel_int\n", __func__);
		}
	} else	{
		dev_err(&client->dev,
			"%s: Failed to get of_node\n", __func__);
		fuelgauge->pdata = client->dev.platform_data;
	}
	i2c_set_clientdata(client, fuelgauge);

	if (fuelgauge->pdata->fg_gpio_init != NULL) {
		dev_err(&client->dev,
				"%s: @@@\n", __func__);
		if (!fuelgauge->pdata->fg_gpio_init()) {
			dev_err(&client->dev,
					"%s: Failed to Initialize GPIO\n", __func__);
			goto err_devm_free;
		}
	}

	if (!sec_hal_fg_init(fuelgauge->client)) {
		dev_err(&client->dev,
			"%s: Failed to Initialize Fuelgauge\n", __func__);
		goto err_devm_free;
	}

	fuelgauge->psy_fg.name		= "sm5705-fuelgauge";
	fuelgauge->psy_fg.type		= POWER_SUPPLY_TYPE_UNKNOWN;
	fuelgauge->psy_fg.get_property	= sm5705_fg_get_property;
	fuelgauge->psy_fg.set_property	= sm5705_fg_set_property;
	fuelgauge->psy_fg.properties	= sm5705_fuelgauge_props;
	fuelgauge->psy_fg.num_properties =
		ARRAY_SIZE(sm5705_fuelgauge_props);
        fuelgauge->capacity_max = fuelgauge->pdata->capacity_max;
        raw_soc_val.intval = SEC_FUELGAUGE_CAPACITY_TYPE_RAW;
        sec_hal_fg_get_property(fuelgauge->client,
                POWER_SUPPLY_PROP_CAPACITY, &raw_soc_val);
        raw_soc_val.intval /= 10;
        if(raw_soc_val.intval > fuelgauge->pdata->capacity_max)
                sec_fg_calculate_dynamic_scale(fuelgauge, 100);

	ret = power_supply_register(&client->dev, &fuelgauge->psy_fg);
	if (ret) {
		dev_err(&client->dev,
			"%s: Failed to Register psy_fg\n", __func__);
		goto err_free;
	}

	fuelgauge->is_fuel_alerted = false;
	if (fuelgauge->pdata->fuel_alert_soc >= 0) {
		if (sec_hal_fg_fuelalert_init(fuelgauge->client,
			fuelgauge->pdata->fuel_alert_soc))
			wake_lock_init(&fuelgauge->fuel_alert_wake_lock,
				WAKE_LOCK_SUSPEND, "fuel_alerted");
		else {
			dev_err(&client->dev,
				"%s: Failed to Initialize Fuel-alert\n",
				__func__);
			goto err_irq;
		}
	}

	if (fuelgauge->pdata->fg_irq > 0) {
		INIT_DELAYED_WORK(
			&fuelgauge->isr_work, sm5705_fg_isr_work);

		fuelgauge->fg_irq = gpio_to_irq(fuelgauge->pdata->fg_irq);
		pr_info(
			"%s: fg_irq = %d\n", __func__, fuelgauge->fg_irq);
		if (fuelgauge->fg_irq > 0) {
			ret = request_threaded_irq(fuelgauge->fg_irq,
					NULL, sm5705_fg_irq_thread,
					IRQF_TRIGGER_FALLING
					 | IRQF_ONESHOT,
					"fuelgauge-irq", fuelgauge);
			if (ret) {
				dev_err(&client->dev,
					"%s: Failed to Reqeust IRQ\n", __func__);
				goto err_supply_unreg;
			}

			ret = enable_irq_wake(fuelgauge->fg_irq);
			if (ret < 0)
				dev_err(&client->dev,
					"%s: Failed to Enable Wakeup Source(%d)\n",
					__func__, ret);
		} else {
			dev_err(&client->dev, "%s: Failed gpio_to_irq(%d)\n",
				__func__, fuelgauge->fg_irq);
			goto err_supply_unreg;
		}
	}

	fuelgauge->initial_update_of_soc = true;
	//if (sec_bat_check_jig_status())
	//	sm5705_fg_reset_capacity_by_jig_connection(fuelgauge);

	ret = sm5705_create_attrs(fuelgauge->psy_fg.dev);
	if (ret) {
		dev_err(&client->dev,
			"%s : Failed to create_attrs\n", __func__);
		goto err_irq;
	}

	pr_info(
		"%s: SEC Fuelgauge Driver Loaded\n", __func__);
	return 0;

err_irq:
	if (fuelgauge->fg_irq > 0)
		free_irq(fuelgauge->fg_irq, fuelgauge);
	wake_lock_destroy(&fuelgauge->fuel_alert_wake_lock);
err_supply_unreg:
	power_supply_unregister(&fuelgauge->psy_fg);
err_devm_free:
	if(pdata)
		devm_kfree(&client->dev, pdata);
//	if(battery_data)
//		devm_kfree(&client->dev, battery_data);
err_free:
	mutex_destroy(&fuelgauge->fg_lock);
	kfree(fuelgauge);

	pr_info("%s: Fuel gauge probe failed\n", __func__);
	return ret;
}

static int sm5705_fuelgauge_remove(
						struct i2c_client *client)
{
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	if (fuelgauge->pdata->fuel_alert_soc >= 0)
		wake_lock_destroy(&fuelgauge->fuel_alert_wake_lock);

	return 0;
}

static int sm5705_fuelgauge_suspend(struct device *dev)
{
	struct sec_fuelgauge_info *fuelgauge = dev_get_drvdata(dev);

	if (!sec_hal_fg_suspend(fuelgauge->client))
		dev_err(&fuelgauge->client->dev,
			"%s: Failed to Suspend Fuelgauge\n", __func__);

	return 0;
}

static int sm5705_fuelgauge_resume(struct device *dev)
{
	struct sec_fuelgauge_info *fuelgauge = dev_get_drvdata(dev);

	if (!sec_hal_fg_resume(fuelgauge->client))
		dev_err(&fuelgauge->client->dev,
			"%s: Failed to Resume Fuelgauge\n", __func__);

	return 0;
}

static void sm5705_fuelgauge_shutdown(struct i2c_client *client)
{
}

static const struct i2c_device_id sm5705_fuelgauge_id[] = {
	{"sm5705-fuelgauge", 0},
	{}
};

static const struct dev_pm_ops sm5705_fuelgauge_pm_ops = {
	.suspend = sm5705_fuelgauge_suspend,
	.resume  = sm5705_fuelgauge_resume,
};

MODULE_DEVICE_TABLE(i2c, sm5705_fuelgauge_id);
static struct of_device_id fuelgague_i2c_match_table[] = {
	{ .compatible = "sm5705-fuelgauge,i2c", },
	{ },
};
MODULE_DEVICE_TABLE(i2c, fuelgague_i2c_match_table);

static struct i2c_driver sm5705_fuelgauge_driver = {
	.driver = {
		   .name = "sm5705-fuelgauge",
		   .owner = THIS_MODULE,
		   .of_match_table = fuelgague_i2c_match_table,
#ifdef CONFIG_PM
		   .pm = &sm5705_fuelgauge_pm_ops,
#endif
	},
	.probe	= sm5705_fuelgauge_probe,
	.remove	= sm5705_fuelgauge_remove,
	.shutdown   = sm5705_fuelgauge_shutdown,
	.id_table   = sm5705_fuelgauge_id,
};

static int __init sm5705_fuelgauge_init(void)
{
	pr_info("%s \n", __func__);

	return i2c_add_driver(&sm5705_fuelgauge_driver);
}

static void __exit sm5705_fuelgauge_exit(void)
{
	i2c_del_driver(&sm5705_fuelgauge_driver);
}

module_init(sm5705_fuelgauge_init);
module_exit(sm5705_fuelgauge_exit);

MODULE_DESCRIPTION("Samsung SM5705 Fuel Gauge Driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
