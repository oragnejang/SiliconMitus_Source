/*
 * leds-sm5713-fled.c - SM5713 Flash-LEDs device driver
 *
 * Copyright (C) 2017 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mfd/sm5713.h>
#include <linux/mfd/sm5713-private.h>

#define SM5713_FLED_MAX_NUM     3

enum {
    FLED_MODE_OFF       = 0x0,
    FLED_MODE_TORCH     = 0x1,
    FLED_MODE_FLASH     = 0x2,
    FLED_MODE_EXTERNAL  = 0x3,
};

struct sm5713_fled_platform_data {
    struct {
    	const char *name;
        u8 flash_brightness;
        u8 preflash_brightness;
        u8 torch_brightness;
    	u8 timeout;

        int fen_pin;            /* GPIO-pin for Flash */
        int men_pin;            /* GPIO-pin for Torch */

        bool pre_fled;
        bool en_fled;
        bool en_mled;
    }led[SM5713_FLED_MAX_NUM];
};

struct sm5713_fled_data {
    struct device *dev;
    struct i2c_client *i2c;
	struct mutex fled_mutex;

    struct sm5713_fled_platform_data *pdata;
    struct device *rear_fled_dev;

    int irq_vbus_update;
    bool vbus_update;
    u8 vbus_voltage;
    u8 torch_on_cnt;
    u8 flash_on_cnt;
    u8 flash_prepare_cnt;
};

static struct sm5713_fled_data *g_sm5713_fled = NULL;
extern struct class *camera_class; /*sys/class/camera*/

/* for MUIC HV-VBUS control */
extern int muic_disable_afc_state(void);
extern int muic_check_fled_state(bool enable, bool mode);   /* mode:0 = torch, mode:1 = flash */

static void fled_set_mode(struct sm5713_fled_data *fled, u8 index, u8 mode)
{
    switch (index) {
    case 0:
    case 1:
        sm5713_update_reg(fled->i2c, SM5713_CHG_REG_FLED1CNTL1 + (index * 2), (mode << 0), (0x3 << 0));
        break;
    case 2:
        sm5713_update_reg(fled->i2c, SM5713_CHG_REG_FLED3CNTL, (mode << 4), (0x1 << 4));
        break;
    }
}

static void fled_set_fled_current(struct sm5713_fled_data *fled, u8 index, u8 offset)
{
    switch (index) {
    case 0:
    case 1:
        sm5713_update_reg(fled->i2c, SM5713_CHG_REG_FLED1CNTL2 + (index * 2), (offset << 0), (0xf << 0));
        break;
    case 2:
        dev_err(fled->dev, "%s: can't used fled2\n", __func__);
        break;
    }
}

static void fled_set_mled_current(struct sm5713_fled_data *fled, u8 index, u8 offset)
{
    switch (index) {
    case 0:
    case 1:
        sm5713_update_reg(fled->i2c, SM5713_CHG_REG_FLED1CNTL2 + (index * 2), (offset << 4), (0x7 << 4));
        break;
    case 2:
        sm5713_update_reg(fled->i2c, SM5713_CHG_REG_FLED3CNTL, (offset << 0), (0xf << 0));
        break;
    }
}

static int fled_get_vbus_voltage(struct sm5713_fled_data *fled)
{
    /* This function need to polling time (max=150ms) */
    int i;
    u8 reg;

    sm5713_read_reg(fled->i2c, SM5713_CHG_REG_STATUS1, &reg);
    if ((reg & 0x1) == 0) {
        dev_info(fled->dev, "%s: vbus=uvlo\n", __func__);
        return 0;
    }

    if ((fled->torch_on_cnt + fled->flash_prepare_cnt) > 0) {
        dev_info(fled->dev, "%s: already used fled(%d:%d)\n", __func__, fled->torch_on_cnt, fled->flash_prepare_cnt);
        return 1;
    }

    sm5713_update_reg(fled->i2c, SM5713_CHG_REG_CHGCNTL9, (0x1 << 0), (0x1 << 0));

    for (i=0; i < 15; ++i) {
        if (fled->vbus_update) {
            break;
        }
        msleep(10);
    }

    if (i == 15) {
        dev_err(fled->dev, "%s: time out\n", __func__);
        return -1;
    }

    sm5713_update_reg(fled->i2c, SM5713_CHG_REG_CHGCNTL9, (0x0 << 0), (0x1 << 0));
    sm5713_read_reg(fled->i2c, SM5713_CHG_REG_CHGCNTL10, &reg);

    dev_info(fled->dev, "%s: vbus voltage=%d.xV\n", __func__, (reg >> 2));

    return (reg >> 2);
}

#if 0
#define PRINT_FLED_REG_NUM   5   
static void fled_print_reg(struct sm5713_fled_data *fled)
{
	u8 regs[PRINT_FLED_REG_NUM] = {0x0, };
	int i;

	sm5713_bulk_read(fled->i2c, SM5713_CHG_REG_FLED1CNTL1, PRINT_FLED_REG_NUM, regs);

	pr_info("sm5713-fled: print regmap\n");
	for (i = 0; i < PRINT_FLED_REG_NUM; ++i) {
		pr_info("0x%02x:0x%02x ", SM5713_CHG_REG_FLED1CNTL1 + i, regs[i]);
        if (i % 8 == 0)
            pr_info("\n");
    }
}
#endif

/**
 * For export Camera flash control support 
 *  
 * Caution - MUST be called "sm5713_fled_prepare_flash" before 
 * using to FLED. also if finished using FLED, MUST be called 
 * "sm5713_fled_close_flash". 
 */
int sm5713_fled_prepare_flash(u8 index)
{
    struct sm5713_fled_data *fled = g_sm5713_fled;

    pr_info("sm5713-fled: %s: start.\n", __func__);

    if (g_sm5713_fled == NULL) {
        pr_err("sm5713-fled: %s: not probe fled yet\n", __func__);
        return -ENXIO;
    }

    if (index > 1 || fled->pdata->led[index].flash_brightness == 0) {
        pr_err("sm5713-fled: %s: don't support flash mode (index=%d)\n", __func__, index);
        return -EPERM;
    }

    if (fled->pdata->led[index].pre_fled == true) {
        pr_info("sm5713-fled: %s: already prepared\n", __func__);
        return 0;
    }

    mutex_lock(&fled->fled_mutex);

    if (fled->flash_prepare_cnt == 0) {
        fled->vbus_voltage = fled_get_vbus_voltage(fled);
        if (fled->vbus_voltage > 5) {
            muic_disable_afc_state();
        }
        muic_check_fled_state(1, 1);
    }
    fled_set_fled_current(fled, index, fled->pdata->led[index].flash_brightness);
    fled_set_mled_current(fled, index, fled->pdata->led[index].torch_brightness);

    fled->flash_prepare_cnt++;
    fled->pdata->led[index].pre_fled = true;

    mutex_unlock(&fled->fled_mutex);

    pr_info("sm5713-fled: %s: done.\n", __func__);

    return 0;
}
EXPORT_SYMBOL_GPL(sm5713_fled_prepare_flash);

int sm5713_fled_torch_on(u8 index, u8 brightness)
{
    struct sm5713_fled_data *fled = g_sm5713_fled;

    pr_info("sm5713-fled: %s: start.\n", __func__);

    if (g_sm5713_fled == NULL) {
        pr_err("sm5713-fled: %s: not probe fled yet\n", __func__);
        return -ENXIO;
    }

    mutex_lock(&fled->fled_mutex);

    if (brightness) {
        fled_set_mled_current(fled, index, brightness);
    } else {
        fled_set_mled_current(fled, index, fled->pdata->led[index].torch_brightness);
    }

    if (fled->pdata->led[index].en_mled == false) {
        if (fled->torch_on_cnt == 0) {
            sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_TORCH, 1);
        }
        fled->pdata->led[index].en_mled = true;
        fled->torch_on_cnt++;
    }

    if (fled->pdata->led[index].men_pin >= 0 && fled->pdata->led[index].fen_pin >= 0) {
        fled_set_mode(fled, index, FLED_MODE_EXTERNAL);
        gpio_set_value(fled->pdata->led[index].fen_pin, 0);
        gpio_set_value(fled->pdata->led[index].men_pin, 1);
    } else {
        fled_set_mode(fled, index, FLED_MODE_TORCH);
    }

    mutex_unlock(&fled->fled_mutex);

    pr_info("sm5713-fled: %s: done.\n", __func__);

    return 0;
}
EXPORT_SYMBOL_GPL(sm5713_fled_torch_on);

int sm5713_fled_pre_flash_on(u8 index, u8 brightness)
{
    struct sm5713_fled_data *fled = g_sm5713_fled;

    pr_info("sm5713-fled: %s: start.\n", __func__);

    if (g_sm5713_fled == NULL) {
        pr_err("sm5713-fled: %s: not probe fled yet\n", __func__);
        return -ENXIO;
    }

    mutex_lock(&fled->fled_mutex);

    if (brightness) {
        fled_set_mled_current(fled, index, brightness);
    } else {
        fled_set_mled_current(fled, index, fled->pdata->led[index].preflash_brightness);
    }

    if (fled->pdata->led[index].en_mled == false) {
        if (fled->torch_on_cnt == 0) {
            sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_TORCH, 1);
        }
        fled->pdata->led[index].en_mled = true;
        fled->torch_on_cnt++;
    }

    if (fled->pdata->led[index].men_pin >= 0 && fled->pdata->led[index].fen_pin >= 0) {
        fled_set_mode(fled, index, FLED_MODE_EXTERNAL);
        gpio_set_value(fled->pdata->led[index].fen_pin, 0);
        gpio_set_value(fled->pdata->led[index].men_pin, 1);
    } else {
        fled_set_mode(fled, index, FLED_MODE_TORCH);
    }

    mutex_unlock(&fled->fled_mutex);

    pr_info("sm5713-fled: %s: done.\n", __func__);

    return 0;
}
EXPORT_SYMBOL_GPL(sm5713_fled_pre_flash_on);

int sm5713_fled_flash_on(u8 index, u8 brightness)
{
    struct sm5713_fled_data *fled = g_sm5713_fled;

    pr_info("sm5713-fled: %s: start.\n", __func__);

    if (g_sm5713_fled == NULL) {
        pr_err("sm5713-fled: %s: not probe fled yet\n", __func__);
        return -ENXIO;
    }

    mutex_lock(&fled->fled_mutex);

    if (brightness) {
        fled_set_fled_current(fled, index, brightness);
    } else {
        fled_set_fled_current(fled, index, fled->pdata->led[index].flash_brightness);
    }

    if (fled->pdata->led[index].en_fled == false) {
        if (fled->flash_on_cnt == 0) {
            sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_FLASH, 1);
        }
        fled->pdata->led[index].en_fled = true;
        fled->flash_on_cnt++;
    }

    if (fled->pdata->led[index].men_pin >= 0 && fled->pdata->led[index].fen_pin >= 0) {
        fled_set_mode(fled, index, FLED_MODE_EXTERNAL);
        gpio_set_value(fled->pdata->led[index].men_pin, 0);
        gpio_set_value(fled->pdata->led[index].fen_pin, 1);
    } else {
        fled_set_mode(fled, index, FLED_MODE_FLASH);
    }

    mutex_unlock(&fled->fled_mutex);

    pr_info("sm5713-fled: %s: done.\n", __func__);

    return 0;
}
EXPORT_SYMBOL_GPL(sm5713_fled_flash_on);

int sm5713_fled_led_off(u8 index)
{
    struct sm5713_fled_data *fled = g_sm5713_fled;

    pr_info("sm5713-fled: %s: start.\n", __func__);

    if (g_sm5713_fled == NULL) {
        pr_err("sm5713-fled: %s: not probe fled yet\n", __func__);
        return -ENXIO;
    }

    mutex_lock(&fled->fled_mutex);

    fled_set_mode(fled, index, FLED_MODE_OFF);

    if (fled->pdata->led[index].en_mled == true) {
        fled->torch_on_cnt--;
        if (fled->torch_on_cnt == 0) {
            sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_TORCH, 0);
        }
        fled->pdata->led[index].en_mled = false;
    }

    if (fled->pdata->led[index].en_fled == true) {
        fled->flash_on_cnt--;
        if (fled->flash_on_cnt == 0) {
            sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_FLASH, 0);
        }
        fled->pdata->led[index].en_fled = false;
    }

    mutex_unlock(&fled->fled_mutex);

    pr_info("sm5713-fled: %s: done.\n", __func__);

    return 0;
}
EXPORT_SYMBOL_GPL(sm5713_fled_led_off);

int sm5713_fled_close_flash(u8 index)
{
    struct sm5713_fled_data *fled = g_sm5713_fled;

    pr_info("sm5713-fled: %s: start.\n", __func__);

    if (g_sm5713_fled == NULL) {
        pr_err("sm5713-fled: %s: not probe fled yet\n", __func__);
        return -ENXIO;
    }

    if (index > 1 || fled->pdata->led[index].flash_brightness == 0) {
        pr_err("sm5713-fled: %s: don't support flash mode (index=%d)\n", __func__, index);
        return -EPERM;
    }

    if (fled->pdata->led[index].pre_fled == false) {
        pr_info("sm5713-fled: %s: already closed\n", __func__);
        return 0;
    }

    mutex_lock(&fled->fled_mutex);

    fled_set_mode(fled, index, FLED_MODE_OFF);
    fled->flash_prepare_cnt--;

    if (fled->flash_prepare_cnt == 0) {
        sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_TORCH, 0);
        sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_FLASH, 0);
        muic_check_fled_state(0, 1);
    }
    fled->pdata->led[index].pre_fled = false;

    mutex_unlock(&fled->fled_mutex);


    pr_info("sm5713-fled: %s: done.\n", __func__);

    return 0;
}
EXPORT_SYMBOL_GPL(sm5713_fled_close_flash);

/**
 *  For camera_class device file control (Torch-LED) 
 */

static int get_fled_index(struct device_attribute *attr)
{
    int index;

    if(strcmp(attr->attr.name,"rear_flash_1") == 0) {
		index = 0;
    } else if(strcmp(attr->attr.name,"rear_flash_2") == 0){
		index = 1;
    } else if(strcmp(attr->attr.name,"rear_flash_3") == 0){
		index = 2;
    } else {
		pr_err("flash index not matched (attr=%s) \n", attr->attr.name);
        return -EINVAL;
    }

    return index;
}

static ssize_t sm5713_rear_flash_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct sm5713_fled_data *fled = dev_get_drvdata(dev->parent);
    int index = get_fled_index(attr);
    u32 store_value;
    int ret;

    if (IS_ERR_VALUE(index) || (buf == NULL) || kstrtouint(buf, 10, &store_value)) {
        return -1;
    }
    dev_info(fled->dev, "%s: value=%d\n", __func__, store_value);

    mutex_lock(&fled->fled_mutex);

    if (store_value == 0) {
        /* Torch off */
        if (fled->pdata->led[index].en_mled == false) {
            goto out_skip;
        }
        fled_set_mode(fled, index, FLED_MODE_OFF);

        fled->pdata->led[index].en_mled = false;
        fled->torch_on_cnt--;

        if (fled->torch_on_cnt == 0) {
            sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_TORCH, 0);
            muic_check_fled_state(0, 0);
        }
    } else {
        if (store_value == 100) {
            /* Torch on (Factory mode) */
            fled_set_mled_current(fled, index, 0x7);    /* Set mled=225mA */
        } else if (store_value > 1000 && store_value < (1000 + 32)) {
            /* Torch on (Normal) */
            fled_set_mled_current(fled, index, (store_value-1000));
        } else {
            dev_err(fled->dev, "%s: failed store cmd\n", __func__);
            ret = -EINVAL;
            goto out_p;
        }
        if (fled->pdata->led[index].en_mled == true) {
            goto out_skip;
        }
        if (fled->torch_on_cnt == 0) {
            fled->vbus_voltage = fled_get_vbus_voltage(fled);
            if (fled->vbus_voltage > 5) {
                muic_disable_afc_state();
            }
            muic_check_fled_state(1, 0);
            sm5713_charger_oper_push_event(SM5713_CHARGER_OP_EVENT_TORCH, 1);

        }
        fled_set_mode(fled, index, FLED_MODE_TORCH);
        fled->pdata->led[index].en_mled = true;
        fled->torch_on_cnt++;
    }

out_skip:
    ret = count;

out_p:
    mutex_unlock(&fled->fled_mutex);

    return ret;
}

static ssize_t sm5713_rear_flash_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int index = get_fled_index(attr);
    int len;

    if (IS_ERR_VALUE(index)) {
        return -1;
    }

    switch (index) {
    case 0:
    case 1:
        len = sprintf(buf, "%d\n", 0x7);    /* torch current max=225mA(0x7) */
        break;
    case 2:
        len = sprintf(buf, "%d\n", 0x9);    /* torch current max=500mA(0x9) */
        break;
    }

    return len;
}

static DEVICE_ATTR(rear_flash_1, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH, sm5713_rear_flash_show, sm5713_rear_flash_store);
static DEVICE_ATTR(rear_flash_2, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH, sm5713_rear_flash_show, sm5713_rear_flash_store);
static DEVICE_ATTR(rear_flash_3, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH, sm5713_rear_flash_show, sm5713_rear_flash_store);

static irqreturn_t fled_vbus_update_isr(int irq, void *data)
{
	struct sm5713_fled_data *fled = data;

	dev_info(fled->dev, "%s: irq=%d\n", __func__, irq);

    fled->vbus_update = 1;

	return IRQ_HANDLED;
}

static int sm5713_fled_parse_dt(struct device *dev, struct sm5713_fled_platform_data *pdata)
{
	struct device_node *np, *c_np;
	int index, ret = 0;
    u32 temp;

	np = of_find_node_by_name(NULL, "sm5713-fled");
	if (!np) {
		dev_err(dev, "%s: can't find sm5713-fled np\n", __func__);
        return -EINVAL;
    }

    for_each_child_of_node(np, c_np) {
        ret = of_property_read_u32(c_np, "id", &index);
        if (ret) {
            pr_err("%s: fail to get a id\n", __func__);
            return ret;
        }

        of_property_read_u32(c_np, "flash-brightness", &temp);
        pdata->led[index].flash_brightness = (temp & 0xff);
        of_property_read_u32(c_np, "preflash-brightness", &temp);
        pdata->led[index].preflash_brightness = (temp & 0xff);
        of_property_read_u32(c_np, "torch-brightness", &temp);
        pdata->led[index].torch_brightness = (temp & 0xff);
        of_property_read_u32(c_np, "timeout", &temp);
        pdata->led[index].timeout = (temp & 0xff);

        pdata->led[index].fen_pin = of_get_named_gpio(c_np, "flash-en-gpio", 0);
        pdata->led[index].men_pin = of_get_named_gpio(c_np, "torch-en-gpio", 0);

        dev_info(dev, "%s: f_cur=0x%x, pre_cur=0x%x, t_cur=0x%x, tout=%d, gpio=%d:%d\n",
                    __func__, pdata->led[index].flash_brightness, pdata->led[index].preflash_brightness,
                    pdata->led[index].torch_brightness, pdata->led[index].timeout,
                    pdata->led[index].fen_pin, pdata->led[index].men_pin);
    }

	dev_info(dev, "%s: parse dt done.\n", __func__);
	return 0;
}

static void sm5713_fled_init(struct sm5713_fled_data *fled)
{
    int i, ret;

    for (i=0; i < SM5713_FLED_MAX_NUM; ++i) {
        if (fled->pdata->led[i].fen_pin > 0) {
            ret = gpio_request(fled->pdata->led[i].fen_pin, "sm5713_fled");
            if (ret < 0) {
                dev_err(fled->dev, "%s: failed request fen-gpio(%d)", __func__, fled->pdata->led[i].fen_pin);
                fled->pdata->led[i].fen_pin = ret;
            } else {
                gpio_direction_output(fled->pdata->led[i].fen_pin, 0);
            }
        }
        if (fled->pdata->led[i].men_pin > 0) {
            ret = gpio_request(fled->pdata->led[i].men_pin, "sm5713_fled");
            if (ret < 0) {
                dev_err(fled->dev, "%s: failed request men-gpio(%d)", __func__, fled->pdata->led[i].men_pin);
                fled->pdata->led[i].men_pin = ret;
            } else {
                gpio_direction_output(fled->pdata->led[i].men_pin, 0);
            }
        }

        fled_set_mode(fled, i, FLED_MODE_OFF);
        fled->pdata->led[i].en_mled = 0;
        fled->pdata->led[i].en_fled = 0;
        fled->pdata->led[i].pre_fled = 0;
    }

    fled->torch_on_cnt = 0;
    fled->flash_on_cnt = 0;
    fled->flash_prepare_cnt = 0;
}

static int sm5713_fled_probe(struct platform_device *pdev)
{
    struct sm5713_dev *sm5713 = dev_get_drvdata(pdev->dev.parent);
    struct sm5713_fled_data *fled;
    int ret = 0;

    dev_info(&pdev->dev, "sm5713 fled probe start (rev=%d)\n", sm5713->pmic_rev);

    fled = devm_kzalloc(&pdev->dev, sizeof(struct sm5713_fled_data), GFP_KERNEL);
    if (unlikely(!fled)) {
        dev_err(&pdev->dev, "%s: fail to alloc_devm\n", __func__);
        return -ENOMEM;
    }
    fled->dev = &pdev->dev;
    fled->i2c = sm5713->charger;

    fled->pdata = devm_kzalloc(&pdev->dev, sizeof(struct sm5713_fled_platform_data), GFP_KERNEL);
    if (unlikely(!fled->pdata)) {
        dev_err(fled->dev, "%s: fail to alloc_pdata\n", __func__);
        ret = -ENOMEM;
        goto free_dev;
    }
    ret = sm5713_fled_parse_dt(fled->dev, fled->pdata);
    if (ret < 0) {
        goto free_pdata;
    }

    sm5713_fled_init(fled);
    fled->irq_vbus_update = sm5713->irq_base + SM5713_CHG_IRQ_INT4_VBUS_UPDATE;
	ret = request_threaded_irq(fled->irq_vbus_update, NULL, fled_vbus_update_isr, 0 , "vbusupdate-irq", fled);
	if (ret < 0) {
		dev_err(fled->dev, "%s: failed request irq:%d (ret=%d)\n", __func__, fled->irq_vbus_update, ret);
        goto free_pdata;
	}
    g_sm5713_fled = fled;

    if (IS_ERR_OR_NULL(camera_class)) {
        dev_err(fled->dev, "%s: can't find camera_class sysfs object, didn't used rear_flash attribute\n", __func__);
        goto free_pdata;
    }

    fled->rear_fled_dev = device_create(camera_class, NULL, 3, NULL, "flash");
    if (IS_ERR(fled->rear_fled_dev)) {
        dev_err(fled->dev, "%s failed create device for rear_flash\n", __func__);
        goto free_pdata;
    }
    fled->rear_fled_dev->parent = fled->dev;

    ret = device_create_file(fled->rear_fled_dev, &dev_attr_rear_flash_1);
    if (IS_ERR_VALUE(ret)) {
        dev_err(fled->dev, "%s failed create device file for rear_flash_1\n", __func__);
        goto free_device;
    }
    ret = device_create_file(fled->rear_fled_dev, &dev_attr_rear_flash_2);
    if (IS_ERR_VALUE(ret)) {
        dev_err(fled->dev, "%s failed create device file for rear_flash_2\n", __func__);
        goto free_device;
    }
    ret = device_create_file(fled->rear_fled_dev, &dev_attr_rear_flash_3);
    if (IS_ERR_VALUE(ret)) {
        dev_err(fled->dev, "%s failed create device file for rear_flash_3\n", __func__);
        goto free_device;
    }

    dev_info(&pdev->dev, "sm5713 fled probe done.\n");

	return 0;

free_device:
    device_destroy(camera_class, fled->rear_fled_dev->devt);
free_pdata:
    devm_kfree(&pdev->dev, fled->pdata);    
free_dev:
    devm_kfree(&pdev->dev, fled);
    
    return ret;
}

static int sm5713_fled_remove(struct platform_device *pdev)
{
	struct sm5713_fled_data *fled = platform_get_drvdata(pdev);

    device_remove_file(fled->rear_fled_dev, &dev_attr_rear_flash_1);
    device_remove_file(fled->rear_fled_dev, &dev_attr_rear_flash_2);
    device_remove_file(fled->rear_fled_dev, &dev_attr_rear_flash_3);

    device_destroy(camera_class, fled->rear_fled_dev->devt);

    fled_set_mode(fled, 0, FLED_MODE_OFF);
    fled_set_mode(fled, 1, FLED_MODE_OFF);
    fled_set_mode(fled, 2, FLED_MODE_OFF);

    platform_set_drvdata(pdev, NULL);

    devm_kfree(&pdev->dev, fled->pdata);
    devm_kfree(&pdev->dev, fled);

	return 0;
}

static struct of_device_id sm5713_fled_match_table[] = {
	{ .compatible = "siliconmitus,sm5713-fled",},
	{},
};

static const struct platform_device_id sm5713_fled_id[] = {
	{"sm5713-fled", 0},
	{},
};

static struct platform_driver sm5713_led_driver = {
	.driver = {
		.name  = "sm5713-fled",
		.owner = THIS_MODULE,
		.of_match_table = sm5713_fled_match_table,
		},
	.probe  = sm5713_fled_probe,
	.remove = sm5713_fled_remove,
	.id_table = sm5713_fled_id,
};

static int __init sm5713_led_driver_init(void)
{
	return platform_driver_register(&sm5713_led_driver);
}
module_init(sm5713_led_driver_init);

static void __exit sm5713_led_driver_exit(void)
{
	platform_driver_unregister(&sm5713_led_driver);
}
module_exit(sm5713_led_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Flash-LED device driver for SM5713");

