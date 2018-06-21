/*
 * Flash-LED device driver for SM5708
 *
 * Copyright (C) 2015 Silicon Mitus
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/leds-sm5708.h>
#include <linux/mfd/sm5708/sm5708.h>


#include <linux/battery/charger/sm5708_charger_oper.h>

enum {
	SM5708_FLED_OFF_MODE					= 0x0,
	SM5708_FLED_ON_MOVIE_MODE				= 0x1,
	SM5708_FLED_ON_FLASH_MODE				= 0x2,
	SM5708_FLED_ON_EXTERNAL_CONTROL_MODE	= 0x3,
};

struct sm5708_fled_info {
	struct device *dev;
	struct i2c_client *i2c;

	struct sm5708_fled_platform_data *pdata;
	struct device *rear_fled_dev;

	/* for Flash VBUS check */
	struct workqueue_struct *wqueue;
	struct delayed_work	fled0_vbus_check_work;
	struct delayed_work	fled1_vbus_check_work;
};

extern struct class *camera_class; /*sys/class/camera*/
bool assistive_light = false;
static DEFINE_MUTEX(lock);
static struct sm5708_fled_info *g_sm5708_fled;

static inline int __get_revision_number(void)
{
	return 2;
}

/**
 * SM5708 Flash-LEDs device register control functions
 */

static int sm5708_FLEDx_mode_enable(struct sm5708_fled_info *sm5708_fled, int index, unsigned char FLEDxEN)
{
	int ret;

	ret = sm5708_update_reg(sm5708_fled->i2c, SM5708_REG_FLED1CNTL1 + (index * 4),	(FLEDxEN & 0x3), 0x3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(sm5708_fled->dev, "%s: fail to update REG:FLED%dEN (value=%d)\n",	__func__, index, FLEDxEN);
		return ret;
	}

	dev_info(sm5708_fled->dev, "%s: FLED[%d] set mode = %d\n",	__func__, index, FLEDxEN);

	return 0;
}

#if 0
static inline unsigned char _calc_oneshot_time_offset_to_ms(unsigned short ms)
{
	if (ms < 100) {
		return 0;
	} else {
		return (((ms - 100) / 100) & 0xF);
	}
}

static int sm5708_FLEDx_oneshot_config(struct sm5708_fled_info *sm5708_fled, int index, bool enable, unsigned short timer_ms)
{
	int ret;
	unsigned char reg_val;

	reg_val = (((!enable) & 0x1) << 4) | (_calc_oneshot_time_offset_to_ms(timer_ms) & 0xF);
	ret = sm5708_write_reg(sm5708_fled->i2c,
		SM5708_REG_FLED1CNTL2 + (index * 4), reg_val);
	if (IS_ERR_VALUE(ret)) {
		dev_err(sm5708_fled->dev, "%s: fail to write REG:FLED%dCNTL2 (value=%d)\n",	__func__, index, reg_val);
		return ret;
	}

	return 0;
}
#endif

static inline unsigned char _calc_flash_current_offset_to_mA(int index, unsigned short current_mA)
{
	if (index) {	/* FLED1 */
		return current_mA < 400 ? (((current_mA - 25) / 25) & 0x1F) : ((((current_mA - 400) / 25) + 0xF) & 0x1F);
	} else {		/* FLED0 */
		return current_mA < 700 ? (((current_mA - 300) / 25) & 0x1F) : ((((current_mA - 700) / 50) + 0xF) & 0x1F);
	}
}

static int sm5708_FLEDx_set_flash_current(struct sm5708_fled_info *sm5708_fled, int index, unsigned short current_mA)
{
	int ret;
	unsigned char reg_val;

	reg_val = _calc_flash_current_offset_to_mA(index, current_mA);
	ret = sm5708_write_reg(sm5708_fled->i2c, SM5708_REG_FLED1CNTL3 + (index * 4), reg_val);
	if (IS_ERR_VALUE(ret)) {
		dev_err(sm5708_fled->dev, "%s: fail to write REG:FLED%dCNTL3 (value=%d)\n",	__func__, index, reg_val);
		return ret;
	}

	return 0;
}

static inline unsigned char _calc_torch_current_offset_to_mA(unsigned short current_mA)
{
	return (((current_mA - 10) / 10) & 0x1F);
}

static inline unsigned short _calc_torch_current_mA_to_offset(unsigned char offset)
{
	return (((offset & 0x1F) + 1) * 10);
}

static int sm5708_FLEDx_set_torch_current(struct sm5708_fled_info *sm5708_fled, int index, unsigned short current_mA)
{
	int ret;
	unsigned char reg_val;

	reg_val = _calc_torch_current_offset_to_mA(current_mA);
	ret = sm5708_write_reg(sm5708_fled->i2c, SM5708_REG_FLED1CNTL4 + (index * 4), reg_val);
	if (IS_ERR_VALUE(ret)) {
		dev_err(sm5708_fled->dev, "%s: fail to write REG:FLED%dCNTL4 (value=%d)\n",	__func__, index, reg_val);
		return ret;
	}

	return 0;
}

/**
 * SM5708 Flash-LED operation control functions
 */
static int sm5708_fled_initialize(struct sm5708_fled_info *sm5708_fled)
{
	struct device *dev = sm5708_fled->dev;
	struct sm5708_fled_platform_data *pdata = sm5708_fled->pdata;
	int i, ret;

	for (i = 0; i < SM5708_FLED_MAX; ++i) {
		if (pdata->led[i].used_gpio) {
			ret = gpio_request(pdata->led[i].flash_en_pin, "sm5708_fled");
			if (IS_ERR_VALUE(ret)) {
				dev_err(dev, "%s: fail to request flash gpio pin = %d (ret=%d)\n",	__func__, pdata->led[i].flash_en_pin, ret);
				return ret;
			}
			gpio_direction_output(pdata->led[i].flash_en_pin, 0);

			ret = gpio_request(pdata->led[i].torch_en_pin, "sm5708_fled");
			if (IS_ERR_VALUE(ret)) {
				dev_err(dev, "%s: fail to request torch gpio pin = %d (ret=%d)\n",	__func__, pdata->led[i].torch_en_pin, ret);
				return ret;
			}
			gpio_direction_output(pdata->led[i].torch_en_pin, 0);

			dev_info(dev, "SM5708 FLED[%d] used External GPIO control Mode (Flash pin=%d, Torch pin=%d)\n",
				i, pdata->led[i].flash_en_pin, pdata->led[i].torch_en_pin);
		} else {
			dev_info(dev, "SM5708 FLED[%d] used I2C control Mode\n", i);
		}
		ret = sm5708_FLEDx_mode_enable(sm5708_fled, i, SM5708_FLED_OFF_MODE);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to set FLED[%d] external control mode\n", __func__, i);
			return ret;
		}
	}

	return 0;
}

static void sm5708_fled_deinitialize(struct sm5708_fled_info *sm5708_fled)
{
	struct device *dev = sm5708_fled->dev;
	struct sm5708_fled_platform_data *pdata = sm5708_fled->pdata;
	int i;

	for (i = 0; i < SM5708_FLED_MAX; ++i) {
		if (pdata->led[i].used_gpio) {
			gpio_free(pdata->led[i].flash_en_pin);
			gpio_free(pdata->led[i].torch_en_pin);
		}
		sm5708_FLEDx_mode_enable(sm5708_fled, i, SM5708_FLED_OFF_MODE);
	}

	dev_info(dev, "%s: FLEDs de-initialize done.\n", __func__);
}

static inline int _fled_turn_on_torch(struct sm5708_fled_info *sm5708_fled, int index)
{
	struct sm5708_fled_platform_data *pdata = sm5708_fled->pdata;
	struct device *dev = sm5708_fled->dev;
	int ret;

	pr_debug("%s: FLED[%d] Torch turn-on done.\n", __func__, index);

	if (pdata->led[index].used_gpio) {
		ret = sm5708_FLEDx_mode_enable(sm5708_fled, index,	SM5708_FLED_ON_EXTERNAL_CONTROL_MODE);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to set FLED[%d] External control mode\n", __func__, index);
			return ret;
		}
		gpio_set_value(pdata->led[index].flash_en_pin, 0);
		gpio_set_value(pdata->led[index].torch_en_pin, 1);
	} else {
		ret = sm5708_FLEDx_mode_enable(sm5708_fled, index,	SM5708_FLED_ON_MOVIE_MODE);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to set FLED[%d] Movie mode\n", __func__, index);
			return ret;
		}
	}
	sm5708_charger_oper_push_event(SM5708_CHARGER_OP_EVENT_TORCH, 1);

	dev_info(dev, "%s: FLED[%d] Torch turn-on done.\n", __func__, index);

	return 0;
}

static int sm5708_fled_turn_on_torch(struct sm5708_fled_info *sm5708_fled, int index, unsigned short current_mA)
{
	struct device *dev = sm5708_fled->dev;
	int ret;

	dev_err(dev, "%s: set FLED[%d] torch current (current_mA=%d)\n", __func__, index, current_mA);

	ret = sm5708_FLEDx_set_torch_current(sm5708_fled, index, current_mA);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "%s: fail to set FLED[%d] torch current (current_mA=%d)\n", __func__, index, current_mA);
		return ret;
	}

	_fled_turn_on_torch(sm5708_fled, index);

	return 0;
}


static int sm5708_fled_turn_on_flash(struct sm5708_fled_info *sm5708_fled, int index, unsigned short current_mA)
{
	struct device *dev = sm5708_fled->dev;
	struct sm5708_fled_platform_data *pdata = sm5708_fled->pdata;
	int ret;

	ret = sm5708_FLEDx_set_flash_current(sm5708_fled, index, current_mA);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "%s: fail to set FLED[%d] flash current (current_mA=%d)\n", __func__, index, current_mA);
		return ret;
	}
	/*
	Charger event need to push first before gpio enable.
	In flash capture mode, when connect some types charger or USB, from pre-flash -> main-flash ,
	torch on first and charger has been change as torch mode, if enable flash first, it will work
	on torch mode which use flash current , Vbus will pull down and charger disconnect.
	*/
	sm5708_charger_oper_push_event(SM5708_CHARGER_OP_EVENT_FLASH, 1);

	if (pdata->led[index].used_gpio) {
		ret = sm5708_FLEDx_mode_enable(sm5708_fled, index,	SM5708_FLED_ON_EXTERNAL_CONTROL_MODE);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to set FLED[%d] External control mode\n", __func__, index);
			return ret;
		}
		gpio_set_value(pdata->led[index].torch_en_pin, 0);
		gpio_set_value(pdata->led[index].flash_en_pin, 1);
	} else {
		ret = sm5708_FLEDx_mode_enable(sm5708_fled, index,	SM5708_FLED_ON_FLASH_MODE);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to set FLED[%d] Flash mode\n", __func__, index);
			return ret;
		}
	}


	dev_info(dev, "%s: FLED[%d] Flash turn-on done.\n", __func__, index);

	return 0;
}

static int sm5708_fled_turn_off(struct sm5708_fled_info *sm5708_fled, int index)
{
	struct device *dev = sm5708_fled->dev;
	struct sm5708_fled_platform_data *pdata = sm5708_fled->pdata;
	int ret;

	if (pdata->led[index].used_gpio) {
		gpio_set_value(pdata->led[index].flash_en_pin, 0);
		gpio_set_value(pdata->led[index].torch_en_pin, 0);
	}

	ret = sm5708_FLEDx_mode_enable(sm5708_fled, index, SM5708_FLED_OFF_MODE);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "%s: fail to set FLED[%d] OFF mode\n", __func__, index);
		return ret;
	}


	ret = sm5708_FLEDx_set_flash_current(sm5708_fled, index, 0);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "%s: fail to set FLED[%d] flash current\n", __func__, index);
		return ret;
	}

	ret = sm5708_FLEDx_set_torch_current(sm5708_fled, index, 0);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "%s: fail to set FLED[%d] torch current\n", __func__, index);
		return ret;
	}

	sm5708_charger_oper_push_event(SM5708_CHARGER_OP_EVENT_FLASH, 0);
	sm5708_charger_oper_push_event(SM5708_CHARGER_OP_EVENT_TORCH, 0);

	dev_info(dev, "%s: FLED[%d] turn-off done.\n", __func__, index);

	return 0;
}

/**
 *  For Export Flash control functions (external GPIO control)
 */

int sm5708_fled_prepare_flash(unsigned char index)
{

	if (g_sm5708_fled == NULL) {
		pr_err("sm5708-fled: %s: invalid g_sm5708_fled, maybe not registed fled \
			device driver\n", __func__);
		return -ENXIO;
	}

	dev_info(g_sm5708_fled->dev, "%s: check - GPIO used, set - Torch/Flash current\n",  __func__);

	if (g_sm5708_fled->pdata->led[index].used_gpio == 0) {
		pr_err("sm5708-fled: %s: can't used external GPIO control, check device tree\n", __func__);
		return -ENOENT;
	}


	pr_err("sm5708-fled: torch current : %d, flash_current : %d", g_sm5708_fled->pdata->led[index].torch_current_mA, g_sm5708_fled->pdata->led[index].flash_current_mA);

	sm5708_FLEDx_set_torch_current(g_sm5708_fled, index, g_sm5708_fled->pdata->led[index].torch_current_mA);
	sm5708_FLEDx_set_flash_current(g_sm5708_fled, index, g_sm5708_fled->pdata->led[index].flash_current_mA);

	return 0;
}
EXPORT_SYMBOL(sm5708_fled_prepare_flash);

int sm5708_fled_torch_on(unsigned char index)
{
	if (assistive_light == false) {
		if (g_sm5708_fled == NULL) {
			pr_err("sm5708-fled: %s: invalid g_sm5708_fled, maybe not registed fled device driver\n", __func__);
			return -ENXIO;
		}
		dev_info(g_sm5708_fled->dev, "%s: Torch - ON\n", __func__);
		sm5708_fled_turn_on_torch(g_sm5708_fled, index, g_sm5708_fled->pdata->led[index].torch_current_mA);
	}

	return 0;
}
EXPORT_SYMBOL(sm5708_fled_torch_on);


int sm5708_fled_flash_on(unsigned char index)
{
	if (assistive_light == false) {
		if (g_sm5708_fled == NULL) {
			pr_err("sm5708-fled: %s: invalid g_sm5708_fled, maybe not registed fled device driver\n", __func__);
			return -ENXIO;
		}

		dev_info(g_sm5708_fled->dev, "%s: Flash - ON\n", __func__);
		sm5708_fled_turn_on_flash(g_sm5708_fled, index, g_sm5708_fled->pdata->led[index].flash_current_mA);
	}

	return 0;
}
EXPORT_SYMBOL(sm5708_fled_flash_on);

int sm5708_fled_preflash(unsigned char index)
{
	if (assistive_light == false) {
		dev_info(g_sm5708_fled->dev, "%s: Pre Flash or Movie - ON\n", __func__);
		sm5708_fled_turn_on_torch(g_sm5708_fled, index, g_sm5708_fled->pdata->led[index].preflash_current_mA);
	}
	return 0;
}
EXPORT_SYMBOL(sm5708_fled_preflash);

int sm5708_fled_led_off(unsigned char index)
{
	if (g_sm5708_fled == NULL) {
		printk("%s:invalid g_sm5708_fled, maybe not registed fled device driver\n", __func__);
		return 0;
	}
	if (assistive_light == false) {
		dev_info(g_sm5708_fled->dev, "%s: LED - OFF\n", __func__);
		sm5708_fled_turn_off(g_sm5708_fled, index);
    }

	return 0;
}
EXPORT_SYMBOL(sm5708_fled_led_off);


/**
 * For Camera-class Rear Flash device file support functions
 */
#define REAR_FLASH_INDEX	SM5708_FLED_0

static ssize_t sm5708_rear_flash_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sm5708_fled_info *sm5708_fled = dev_get_drvdata(dev->parent);

	int ret, value_u32 = 0;
#if defined(CONFIG_DUAL_LEDS_FLASH)
	int index = SM5708_FLED_MAX;	/* set defualt value for prevent */

	if (strcmp(attr->attr.name, "rear_flash") == 0) {
		pr_err("flash index is 0\n");
		index = SM5708_FLED_0;
	} else if (strcmp(attr->attr.name, "rear_flash_2") == 0) {
		pr_err("flash index is 1\n");
		index = SM5708_FLED_1;
	} else {
		pr_err("flash index is not match\n");
	}
#endif
	if ((buf == NULL) || kstrtouint(buf, 10, &value_u32)) {
		return -EINVAL;
	}

	/* temp error canceling code */
	if (sm5708_fled != g_sm5708_fled) {
		dev_info(dev, "%s: sm5708_fled handler mismatched (g_handle:%p , l_handle:%p)\n", __func__, g_sm5708_fled, sm5708_fled);
		sm5708_fled = g_sm5708_fled;
	}

	dev_info(dev, "%s: value=%d\n", __func__, value_u32);
	mutex_lock(&lock);

	switch (value_u32) {
	case 0:
		/* Turn off Torch */
#if defined(CONFIG_DUAL_LEDS_FLASH)
		ret = sm5708_fled_turn_off(sm5708_fled, index);
#else
		ret = sm5708_fled_turn_off(sm5708_fled, REAR_FLASH_INDEX);
#endif
		assistive_light = false;
		break;
	case 1:
		 /* Turn on Torch */
#if defined(CONFIG_DUAL_LEDS_FLASH)
		ret = sm5708_fled_turn_on_torch(sm5708_fled, index, 60);
#else
		ret = sm5708_fled_turn_on_torch(sm5708_fled, REAR_FLASH_INDEX, 60);
#endif
		assistive_light = true;
		break;
	case 100:
		/* Factory mode Turn on Torch */
		/*Turn on Torch */
#if defined(CONFIG_DUAL_LEDS_FLASH)
		ret = sm5708_fled_turn_on_torch(sm5708_fled, index, 240);
#else
		ret = sm5708_fled_turn_on_torch(sm5708_fled, REAR_FLASH_INDEX, 240);
#endif
		assistive_light = true;
		break;
	default:
		if (value_u32 > 1000 && value_u32 < (1000 + 32)) {

			/* Turn on Torch : 20mA ~ 320mA */
#if defined(CONFIG_DUAL_LEDS_FLASH)
			if (1 == index)
				value_u32 -= 1;
			ret = sm5708_fled_turn_on_torch(sm5708_fled, index, _calc_torch_current_mA_to_offset(value_u32 - 1000));
#else
			ret = sm5708_fled_turn_on_torch(sm5708_fled, REAR_FLASH_INDEX,	_calc_torch_current_mA_to_offset(value_u32 - 1000));
#endif
			assistive_light = true;
		} else {
			dev_err(dev, "%s: can't process, invalid value=%d\n", __func__, value_u32);
			ret = -EINVAL;
		}
		break;
	}
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "%s: fail to rear flash file operation:store (value=%d, ret=%d)\n", __func__, value_u32, ret);
	}


	mutex_unlock(&lock);

	return count;
}

static ssize_t sm5708_rear_flash_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char offset = _calc_torch_current_offset_to_mA(320);

	dev_info(dev, "%s: SM5708 Movie mode max current = 320mA(offset:%d)\n",	__func__, offset);

	return snprintf(buf, strlen(buf), "%d\n", offset);
}

static DEVICE_ATTR(rear_flash, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH, sm5708_rear_flash_show, sm5708_rear_flash_store);
#if defined(CONFIG_DUAL_LEDS_FLASH)
static DEVICE_ATTR(rear_flash_2, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH, sm5708_rear_flash_show, sm5708_rear_flash_store);
#endif

/**
 * SM5708 Flash-LED device driver management functions
 */

#ifdef CONFIG_OF
static int sm5708_fled_parse_dt(struct device *dev, struct sm5708_fled_platform_data *pdata)
{
	struct device_node *nproot = dev->of_node;
	struct device_node *np;
	unsigned int temp;
	int index;
#if defined(CONFIG_DUAL_LEDS_FLASH)
	struct device_node *c_np;
#endif
	int ret = 0;

	np = of_find_node_by_name(nproot, "sm5708_fled");
	if (unlikely(np == NULL)) {
		dev_err(dev, "%s: fail to find flash node\n", __func__);
		return ret;
	}
#if !defined(CONFIG_DUAL_LEDS_FLASH)
	ret = of_property_read_u32(np, "id", &temp);
	if (ret) {
		pr_err("%s : could not find led id\n", __func__);
		return ret;
	}
	index = temp;

	ret = of_property_read_u32(np, "flash-mode-current-mA", &temp);
	if (ret) {
		pr_err("%s: fail to get dt:flash-mode-current-mA\n", __func__);
		return ret;
	}
	pdata->led[index].flash_current_mA = temp;

	ret = of_property_read_u32(c_np, "preflash-mode-current-mA", &temp);
	if (ret) {
		pr_err("%s: fail to get dt:preflash-mode-current-mA\n", __func__);
		return ret;
	}
	pdata->led[index].preflash_current_mA = temp;

	ret = of_property_read_u32(np, "torch-mode-current-mA", &temp);
	if (ret) {
		pr_err("%s: fail to get dt:torch-mode-current-mA\n", __func__);
		return ret;
	}
	pdata->led[index].torch_current_mA = temp;

	ret = of_property_read_u32(np, "used-gpio-control", &temp);
	if (ret) {
		pr_err("%s: fail to get dt:used-gpio-control\n", __func__);
		return ret;
	}
	pdata->led[index].used_gpio = (bool)(temp & 0x1);

	if (pdata->led[index].used_gpio) {
		ret = of_get_named_gpio(np, "flash-en-gpio", 0);
		if (ret < 0) {
			pr_err("%s: fail to get dt:flash-en-gpio (ret=%d)\n", __func__, ret);
			return ret;
		}
		pdata->led[index].flash_en_pin = ret;

		ret = of_get_named_gpio(np, "torch-en-gpio", 0);
		if (ret < 0) {
			pr_err("%s: fail to get dt:torch-en-gpio (ret=%d)\n", __func__, ret);
			return ret;
		}
		pdata->led[index].torch_en_pin = ret;
	}
#else
	for_each_child_of_node(np, c_np) {
		ret = of_property_read_u32(c_np, "id", &temp);
		if (ret) {
			pr_err("%s: fail to get a id\n", __func__);
			return ret;
		}
		index = temp;

		ret = of_property_read_u32(c_np, "flash-mode-current-mA", &temp);
		if (ret) {
			pr_err("%s: fail to get dt:flash-mode-current-mA\n", __func__);
			return ret;
		}
		pdata->led[index].flash_current_mA = temp;

		ret = of_property_read_u32(c_np, "preflash-mode-current-mA", &temp);
		if (ret) {
			pr_err("%s: fail to get dt:preflash-mode-current-mA\n", __func__);
			return ret;
		}
		pdata->led[index].preflash_current_mA = temp;

		ret = of_property_read_u32(c_np, "torch-mode-current-mA", &temp);
		if (ret) {
			pr_err("%s: fail to get dt:torch-mode-current-mA\n", __func__);
			return ret;
		}
		pdata->led[index].torch_current_mA = temp;

		ret = of_property_read_u32(c_np, "used-gpio-control", &temp);
		if (ret) {
			pr_err("%s: fail to get dt:used-gpio-control\n", __func__);
			return ret;
		}
		pdata->led[index].used_gpio = (bool)(temp & 0x1);

		if (pdata->led[index].used_gpio) {
			ret = of_get_named_gpio(c_np, "flash-en-gpio", 0);
			if (ret < 0) {
				pr_err("%s: fail to get dt:flash-en-gpio (ret=%d)\n", __func__, ret);
				return ret;
			}
			pdata->led[index].flash_en_pin = ret;

			ret = of_get_named_gpio(c_np, "torch-en-gpio", 0);
			if (ret < 0) {
				pr_err("%s: fail to get dt:torch-en-gpio (ret=%d)\n", __func__, ret);
				return ret;
			}
			pdata->led[index].torch_en_pin = ret;
		}
		if (index == SM5708_FLED_1) {
			pr_err("%s: Done\n", __func__);
			break;
		}
	}

#endif
	return ret;
}
#endif

static inline struct sm5708_fled_platform_data *_get_sm5708_fled_platform_data(struct device *dev, struct sm5708_dev *sm5708)
{
	struct sm5708_fled_platform_data *pdata;
	int i, ret = 0;

#ifdef CONFIG_OF
	pdata = devm_kzalloc(dev, sizeof(struct sm5708_fled_platform_data), GFP_KERNEL);
	if (unlikely(!pdata)) {
		dev_err(dev, "%s: fail to allocate memory for sm5708_fled_platform_data\n",	__func__);
		goto out_p;
	}

	ret = sm5708_fled_parse_dt(dev, pdata);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "%s: fail to parse dt for sm5708 flash-led (ret=%d)\n", __func__, ret);
		goto out_kfree_p;
	}
#else
	pdata = sm5708->pdata->fled_platform_data;
	if (unlikely(!pdata)) {
		dev_err(dev, "%s: fail to get sm5708_fled_platform_data\n", __func__);
		goto out_p;
	}
#endif

	dev_info(dev, "sm5708 flash-LED device platform data info,\n");
	for (i = 0; i < SM5708_FLED_MAX; ++i) {
		dev_info(dev, "[FLED-%d] Flash: %dmA, Torch: %dmA, used_gpio=%d, GPIO_PIN(%d, %d)\n",
			i, pdata->led[i].flash_current_mA,
			pdata->led[i].torch_current_mA, pdata->led[i].used_gpio,
			pdata->led[i].flash_en_pin, pdata->led[i].torch_en_pin);
	}

	return pdata;

out_kfree_p:
	devm_kfree(dev, pdata);
out_p:
	return NULL;
}

static int sm5708_fled_probe(struct platform_device *pdev)
{
	struct sm5708_dev *sm5708 = dev_get_drvdata(pdev->dev.parent);
	struct sm5708_fled_info *sm5708_fled;
	struct sm5708_fled_platform_data *sm5708_fled_pdata;
	struct device *dev = &pdev->dev;
	int i, ret = 0;

	if (IS_ERR_OR_NULL(camera_class)) {
		dev_err(dev, "%s: can't find camera_class sysfs object, didn't used rear_flash attribute\n",
			__func__);
		return -ENOENT;
	}

	sm5708_fled = devm_kzalloc(dev, sizeof(struct sm5708_fled_info), GFP_KERNEL);
	if (unlikely(!sm5708_fled)) {
		dev_err(dev, "%s: fail to allocate memory for sm5708_fled_info\n", __func__);
		return -ENOMEM;
	}

	dev_info(dev, "SM5708(rev.%d) Flash-LED devic driver Probing..\n",
		__get_revision_number());

	sm5708_fled_pdata = _get_sm5708_fled_platform_data(dev, sm5708);
	if (unlikely(!sm5708_fled_pdata)) {
		dev_info(dev, "%s: fail to get platform data\n", __func__);
		goto fled_platfrom_data_err;
	}

	sm5708_fled->dev = dev;
	sm5708_fled->i2c = sm5708->i2c;
	sm5708_fled->pdata = sm5708_fled_pdata;
	platform_set_drvdata(pdev, sm5708_fled);
	g_sm5708_fled = sm5708_fled;

	ret = sm5708_fled_initialize(sm5708_fled);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "%s: fail to initialize SM5708 Flash-LED[%d] (ret=%d)\n", __func__, i, ret);
		goto fled_init_err;
	}

	sm5708_fled->wqueue = create_singlethread_workqueue(dev_name(dev));
	if (!sm5708_fled->wqueue) {
		dev_err(dev, "%s: fail to Create Workqueue\n", __func__);
		goto fled_deinit_err;
	}

	/* create camera_class rear_flash device */
	sm5708_fled->rear_fled_dev = device_create(camera_class, NULL, 3, NULL, "flash");
	if (IS_ERR(sm5708_fled->rear_fled_dev)) {
		dev_err(dev, "%s fail to create device for rear_flash\n", __func__);
		goto fled_deinit_err;
	}
	sm5708_fled->rear_fled_dev->parent = dev;

	ret = device_create_file(sm5708_fled->rear_fled_dev, &dev_attr_rear_flash);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "%s fail to create device file for rear_flash\n", __func__);
		goto fled_rear_device_err;
	}
#if defined(CONFIG_DUAL_LEDS_FLASH)
	ret = device_create_file(sm5708_fled->rear_fled_dev, &dev_attr_rear_flash_2);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "%s fail to create device file for rear_flash\n", __func__);
		goto fled_rear_device_err2;
	}
#endif
	pr_err("%s: Probe done.\n", __func__);

	return 0;
#if defined(CONFIG_DUAL_LEDS_FLASH)
fled_rear_device_err2:
	device_remove_file(sm5708_fled->rear_fled_dev, &dev_attr_rear_flash);
#endif

fled_rear_device_err:
	device_destroy(camera_class, sm5708_fled->rear_fled_dev->devt);

fled_deinit_err:
	sm5708_fled_deinitialize(sm5708_fled);

fled_init_err:
	platform_set_drvdata(pdev, NULL);
#ifdef CONFIG_OF
	devm_kfree(dev, sm5708_fled_pdata);
#endif

fled_platfrom_data_err:
	devm_kfree(dev, sm5708_fled);
	g_sm5708_fled = NULL;

	return ret;
}

static int sm5708_fled_remove(struct platform_device *pdev)
{
	struct sm5708_fled_info *sm5708_fled = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int i;

	device_remove_file(sm5708_fled->rear_fled_dev, &dev_attr_rear_flash);
#if defined(CONFIG_DUAL_LEDS_FLASH)
	device_remove_file(sm5708_fled->rear_fled_dev, &dev_attr_rear_flash_2);
#endif
	device_destroy(camera_class, sm5708_fled->rear_fled_dev->devt);

	for (i = 0; i != SM5708_FLED_MAX; ++i) {
		sm5708_fled_turn_off(sm5708_fled, i);
	}
	sm5708_fled_deinitialize(sm5708_fled);

	platform_set_drvdata(pdev, NULL);
#ifdef CONFIG_OF
	devm_kfree(dev, sm5708_fled->pdata);
#endif
	devm_kfree(dev, sm5708_fled);

	return 0;
}

static void sm5708_fled_shutdown(struct device *dev)
{
	struct sm5708_fled_info *sm5708_fled = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < SM5708_FLED_MAX; ++i) {
		sm5708_fled_turn_off(sm5708_fled, i);
	}
}

#ifdef CONFIG_OF
static struct of_device_id sm5708_fled_match_table[] = {
	{ .compatible = "siliconmitus,sm5708-fled",},
	{},
};
#else
#define sm5708_fled_match_table NULL
#endif

static struct platform_driver sm5708_fled_driver = {
	.probe		= sm5708_fled_probe,
	.remove		= sm5708_fled_remove,
	.driver		= {
		.name	= "sm5708-fled",
		.owner	= THIS_MODULE,
		.shutdown = sm5708_fled_shutdown,
		.of_match_table = sm5708_fled_match_table,
	},
};

static int __init sm5708_fled_init(void)
{
	printk("%s\n", __func__);
	return platform_driver_register(&sm5708_fled_driver);
}
module_init(sm5708_fled_init);

static void __exit sm5708_fled_exit(void)
{
	platform_driver_unregister(&sm5708_fled_driver);
}
module_exit(sm5708_fled_exit);

MODULE_DESCRIPTION("SM5708 FLASH-LED driver");
MODULE_ALIAS("platform:sm5708-flashLED");
MODULE_LICENSE("GPL");

