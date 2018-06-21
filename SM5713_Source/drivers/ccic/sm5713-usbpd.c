/*
 driver/ccic/sm5713-usbpd.c - SM5713 USB PD(Power Delivery) device driver
 *
 * Copyright (C) 2016 Samsung Electronics
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
 */

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/completion.h>

#include <linux/ccic/usbpd.h>
#include <linux/ccic/usbpd-sm5713.h>

#include <linux/muic/muic.h>
#if defined(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */
//#include <linux/sec_batt.h>
#include <linux/battery/sec_charging_common.h>
#ifdef CONFIG_USB_HOST_NOTIFY
#include <linux/usb_notify.h>
#endif
#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
#include <linux/battery/battery_notifier.h>
#endif

#if defined(CONFIG_DUAL_ROLE_USB_INTF)
static enum dual_role_property fusb_drp_properties[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
};
#endif

static usbpd_phy_ops_type sm5713_ops;
static int sm5713_receive_message(void *_data);
struct i2c_client *test_i2c;
int sm5713_set_normal_mode(struct sm5713_usbpd_data *pdic_data);
int sm5713_check_port_detect(struct sm5713_usbpd_data *pdic_data);
static int sm5713_usbpd_reg_init(struct sm5713_usbpd_data *_data);
static void sm5713_set_dfp(struct i2c_client *i2c);
static void sm5713_set_ufp(struct i2c_client *i2c);
int sm5713_set_vconn_source(void *_data, int val);
void process_cc_water_det(void * data, int state);
int sm5713_check_water_status(void *_data);
void sm5713_int_clear(void *_data);
void sm5713_check_abnormal_attach_status(void *_data);

#if 1
static void sm5713_set_src(struct i2c_client *i2c);
static void sm5713_set_snk(struct i2c_client *i2c);
static void sm5713_assert_rd(void *_data);
static void sm5713_assert_rp(void *_data);
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
static int sm5713_set_attach(struct sm5713_usbpd_data *pdic_data, u8 mode);
static int sm5713_set_detach(struct sm5713_usbpd_data *pdic_data, u8 mode);
#endif
#endif
/* #if defined(CONFIG_CCIC_FACTORY) */
static void sm5713_usbpd_check_rid(struct sm5713_usbpd_data *pdic_data);
/* #endif */

int sm5713_usbpd_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest);
int sm5713_usbpd_write_reg(struct i2c_client *i2c, u8 reg, u8 value);

#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
struct pdic_notifier_struct pd_noti;
#endif

void vbus_turn_on_ctrl(struct sm5713_usbpd_data *usbpd_data, bool enable)
{
	struct power_supply *psy_otg;
	union power_supply_propval val;
	int on = !!enable;
	int ret = 0;

//	struct otg_notify *o_notify = get_otg_notify();
//	if (enable && o_notify)
//		o_notify->hw_param[USB_CCIC_OTG_USE_COUNT]++;

	pr_info("%s %d, enable=%d\n", __func__, __LINE__, enable);
	psy_otg = get_power_supply_by_name("otg");

	if (psy_otg) {
		val.intval = enable;
		usbpd_data->is_otg_vboost = enable;
		ret = psy_otg->set_property(psy_otg, POWER_SUPPLY_PROP_ONLINE, &val);
	} else {
		pr_err("%s: Fail to get psy battery\n", __func__);
	}
	if (ret) {
		pr_err("%s: fail to set power_suppy ONLINE property(%d)\n",
			__func__, ret);
	} else {
		pr_info("otg accessory power = %d\n", on);
	}

}

#if defined(CONFIG_DUAL_ROLE_USB_INTF)
void sm5713_rprd_mode_change(struct sm5713_usbpd_data *usbpd_data, u8 mode)
{
	struct i2c_client *i2c = usbpd_data->i2c;
	pr_info("%s, mode=0x%x\n", __func__, mode);

//	mutex_lock(&usbpd_data->lpm_mutex);
//	if (usbpd_data->lpm_mode)
//		goto skip;

	switch (mode) {
	case TYPE_C_ATTACH_DFP: /* SRC */
		sm5713_set_detach(usbpd_data, mode);
		msleep(1500);
		sm5713_set_attach(usbpd_data, mode);
		break;
	case TYPE_C_ATTACH_UFP: /* SNK */
		sm5713_set_detach(usbpd_data, mode);
		msleep(1500);
		sm5713_set_attach(usbpd_data, mode);
		break;
	case TYPE_C_ATTACH_DRP: /* DRP */
        sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL1, 0x80);
		break;
	};
//skip:
//	mutex_unlock(&usbpd_data->lpm_mutex);
}

void role_swap_check(struct work_struct *wk)
{
	struct delayed_work *delay_work =
		container_of(wk, struct delayed_work, work);
	struct sm5713_usbpd_data *usbpd_data =
		container_of(delay_work, struct sm5713_usbpd_data, role_swap_work);
	int mode = 0;

	pr_info("%s: ccic_set_dual_role check again.\n", __func__);
	usbpd_data->try_state_change = 0;

	if (usbpd_data->is_attached == 0) { /* modify here using pd_state */
		pr_err("%s: ccic_set_dual_role reverse failed, set mode to DRP\n", __func__);
		disable_irq(usbpd_data->irq);
		/* exit from Disabled state and set mode to DRP */
		mode =  TYPE_C_ATTACH_DRP;
		sm5713_rprd_mode_change(usbpd_data, mode);
		enable_irq(usbpd_data->irq);
	}

}

static int ccic_set_dual_role(struct dual_role_phy_instance *dual_role,
				   enum dual_role_property prop,
				   const unsigned int *val)
{
	struct sm5713_usbpd_data *usbpd_data = dual_role_get_drvdata(dual_role);
	struct i2c_client *i2c;

	USB_STATUS attached_state;
	int mode;
	int timeout = 0;
	int ret = 0;

	if (!usbpd_data) {
		pr_err("%s : usbpd_data is null \n", __func__);
		return -EINVAL;
	}

	i2c = usbpd_data->i2c;

	/* Get Current Role */
	attached_state = usbpd_data->data_role_dual;
	pr_info("%s : request prop = %d , attached_state = %d\n", __func__, prop, attached_state);

	if (attached_state != USB_STATUS_NOTIFY_ATTACH_DFP
	    && attached_state != USB_STATUS_NOTIFY_ATTACH_UFP) {
		pr_err("%s : current mode : %d - just return \n", __func__, attached_state);
		return 0;
	}

	if (attached_state == USB_STATUS_NOTIFY_ATTACH_DFP
	    && *val == DUAL_ROLE_PROP_MODE_DFP) {
		pr_err("%s : current mode : %d - request mode : %d just return \n",
			__func__, attached_state, *val);
		return 0;
	}

	if (attached_state == USB_STATUS_NOTIFY_ATTACH_UFP
	    && *val == DUAL_ROLE_PROP_MODE_UFP) {
		pr_err("%s : current mode : %d - request mode : %d just return \n",
			__func__, attached_state, *val);
		return 0;
	}

	if (attached_state == USB_STATUS_NOTIFY_ATTACH_DFP) {
		/* Current mode DFP and Source  */
		pr_info("%s: try reversing, from Source to Sink\n", __func__);
		/* turns off VBUS first */
		vbus_turn_on_ctrl(usbpd_data, 0);
		muic_disable_otg_detect();
#if defined(CONFIG_CCIC_NOTIFIER)
		/* muic */
		ccic_event_work(usbpd_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, CCIC_NOTIFY_DETACH/*attach*/, USB_STATUS_NOTIFY_DETACH/*rprd*/);
#endif
		/* exit from Disabled state and set mode to UFP */
		mode =  TYPE_C_ATTACH_UFP;
		usbpd_data->try_state_change = TYPE_C_ATTACH_UFP;
		sm5713_rprd_mode_change(usbpd_data, mode);
	} else {
		/* Current mode UFP and Sink  */
		pr_info("%s: try reversing, from Sink to Source\n", __func__);
		/* exit from Disabled state and set mode to UFP */
		mode =  TYPE_C_ATTACH_DFP;
		usbpd_data->try_state_change = TYPE_C_ATTACH_DFP;
		sm5713_rprd_mode_change(usbpd_data, mode);
	}

	reinit_completion(&usbpd_data->reverse_completion);
	timeout =
	    wait_for_completion_timeout(&usbpd_data->reverse_completion,
					msecs_to_jiffies
					(DUAL_ROLE_SET_MODE_WAIT_MS));

	if (!timeout) {
		usbpd_data->try_state_change = 0;
		pr_err("%s: reverse failed, set mode to DRP\n", __func__);
		disable_irq(usbpd_data->irq);
		/* exit from Disabled state and set mode to DRP */
		mode =  TYPE_C_ATTACH_DRP;
		sm5713_rprd_mode_change(usbpd_data, mode);
		enable_irq(usbpd_data->irq);
		ret = -EIO;
	} else {
		pr_err("%s: reverse success, one more check\n", __func__);
		schedule_delayed_work(&usbpd_data->role_swap_work, msecs_to_jiffies(DUAL_ROLE_SET_MODE_WAIT_MS));
	}

	dev_info(&i2c->dev, "%s -> data role : %d\n", __func__, *val);
	return ret;
}

/* Decides whether userspace can change a specific property */
int dual_role_is_writeable(struct dual_role_phy_instance *drp,
				  enum dual_role_property prop)
{
	if (prop == DUAL_ROLE_PROP_MODE)
		return 1;
	else
		return 0;
}

/* Callback for "cat /sys/class/dual_role_usb/otg_default/<property>" */
int dual_role_get_local_prop(struct dual_role_phy_instance *dual_role,
				    enum dual_role_property prop,
				    unsigned int *val)
{
	struct sm5713_usbpd_data *usbpd_data = dual_role_get_drvdata(dual_role);

	USB_STATUS attached_state;
	int power_role_dual;

	if (!usbpd_data) {
		pr_err("%s : usbpd_data is null : request prop = %d \n", __func__, prop);
		return -EINVAL;
	}
	attached_state = usbpd_data->data_role_dual;
	power_role_dual = usbpd_data->power_role_dual;

	pr_info("%s : request prop = %d , attached_state = %d, power_role_dual = %d\n",
		__func__, prop, attached_state, power_role_dual);

	if (attached_state == USB_STATUS_NOTIFY_ATTACH_DFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_DFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = power_role_dual;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_HOST;
		else
			return -EINVAL;
	} else if (attached_state == USB_STATUS_NOTIFY_ATTACH_UFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_UFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = power_role_dual;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_DEVICE;
		else
			return -EINVAL;
	} else {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_NONE;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_NONE;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_NONE;
		else
			return -EINVAL;
	}

	return 0;
}

/* Callback for "echo <value> >
 *                      /sys/class/dual_role_usb/<name>/<property>"
 * Block until the entire final state is reached.
 * Blocking is one of the better ways to signal when the operation
 * is done.
 * This function tries to switch to Attached.SRC or Attached.SNK
 * by forcing the mode into SRC or SNK.
 * On failure, we fall back to Try.SNK state machine.
 */
int dual_role_set_prop(struct dual_role_phy_instance *dual_role,
			      enum dual_role_property prop,
			      const unsigned int *val)
{
	pr_info("%s : request prop = %d , *val = %d \n", __func__, prop, *val);
	if (prop == DUAL_ROLE_PROP_MODE)
		return ccic_set_dual_role(dual_role, prop, val);
	else
		return -EINVAL;
}
#endif

#if defined(CONFIG_CCIC_NOTIFIER)
extern struct device *ccic_device;

static void ccic_event_notifier(struct work_struct *data)
{
	struct ccic_state_work *event_work =
		container_of(data, struct ccic_state_work, ccic_work);
	CC_NOTI_TYPEDEF ccic_noti;

	switch (event_work->dest) {
	case CCIC_NOTIFY_DEV_USB:
		pr_info("usb:%s, dest=%s, id=%s, attach=%s, drp=%s, event_work=%p\n", __func__,
				CCIC_NOTI_DEST_Print[event_work->dest],
				CCIC_NOTI_ID_Print[event_work->id],
				event_work->attach ? "Attached" : "Detached",
				CCIC_NOTI_USB_STATUS_Print[event_work->event],
				event_work);
		break;
	default:
		pr_info("usb:%s, dest=%s, id=%s, attach=%d, event=%d, event_work=%p\n", __func__,
			CCIC_NOTI_DEST_Print[event_work->dest],
			CCIC_NOTI_ID_Print[event_work->id],
			event_work->attach,
			event_work->event,
			event_work);
		break;
	}

	ccic_noti.src = CCIC_NOTIFY_DEV_CCIC;
	ccic_noti.dest = event_work->dest;
	ccic_noti.id = event_work->id;
	ccic_noti.sub1 = event_work->attach;
	ccic_noti.sub2 = event_work->event;
	ccic_noti.sub3 = 0;
#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
	ccic_noti.pd = &pd_noti;
#endif
	ccic_notifier_notify((CC_NOTI_TYPEDEF *)&ccic_noti, NULL, 0);

	kfree(event_work);
}

void ccic_event_work(void *data, int dest, int id, int attach, int event)
{
	struct sm5713_usbpd_data *usbpd_data = data;
	struct ccic_state_work *event_work;


	event_work = kmalloc(sizeof(struct ccic_state_work), GFP_ATOMIC);
	pr_info("usb: %s,event_work(%p)\n", __func__, event_work);
	INIT_WORK(&event_work->ccic_work, ccic_event_notifier);

	event_work->dest = dest;
	event_work->id = id;
	event_work->attach = attach;
	event_work->event = event;

#if defined(CONFIG_DUAL_ROLE_USB_INTF)
	if (id == CCIC_NOTIFY_ID_USB) {
		pr_info("usb: %s, dest=%d, event=%d, usbpd_data->data_role_dual=%d, usbpd_data->try_state_change=%d\n",
			__func__, dest, event, usbpd_data->data_role_dual, usbpd_data->try_state_change);

		usbpd_data->data_role_dual = event;

		if (usbpd_data->dual_role != NULL)
			dual_role_instance_changed(usbpd_data->dual_role);

		if (usbpd_data->try_state_change &&
			(usbpd_data->data_role_dual != USB_STATUS_NOTIFY_DETACH)) {
			/* Role change try and new mode detected */
			pr_info("usb: %s, reverse_completion\n", __func__);
			complete(&usbpd_data->reverse_completion);
		}
	}
#endif

	if (queue_work(usbpd_data->ccic_wq, &event_work->ccic_work) == 0) {
		pr_info("usb: %s, event_work(%p) is dropped\n", __func__, event_work);
		kfree(event_work);
	}
}

static void process_dr_swap(struct sm5713_usbpd_data *usbpd_data)
{
	struct i2c_client *i2c = usbpd_data->i2c;
	dev_info(&i2c->dev, "%s : before - is_host : %d, is_client : %d\n",
		__func__, usbpd_data->is_host, usbpd_data->is_client);
	if (usbpd_data->is_host == HOST_ON) {
		ccic_event_work(usbpd_data,
			CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB,
				CCIC_NOTIFY_DETACH/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
		msleep(300);
		ccic_event_work(usbpd_data,
			CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB,
				CCIC_NOTIFY_ATTACH/*attach*/, USB_STATUS_NOTIFY_ATTACH_UFP/*drp*/);
		usbpd_data->is_host = HOST_OFF;
		usbpd_data->is_client = CLIENT_ON;
	} else if (usbpd_data->is_client == CLIENT_ON) {
		ccic_event_work(usbpd_data,
			CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB,
				CCIC_NOTIFY_DETACH/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
		msleep(300);
		ccic_event_work(usbpd_data,
			CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB,
				CCIC_NOTIFY_ATTACH/*attach*/, USB_STATUS_NOTIFY_ATTACH_DFP/*drp*/);
		usbpd_data->is_host = HOST_ON;
		usbpd_data->is_client = CLIENT_OFF;
	}
	dev_info(&i2c->dev, "%s : after - is_host : %d, is_client : %d\n",
		__func__, usbpd_data->is_host, usbpd_data->is_client);
}
#endif

int sm5713_usbpd_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	int ret;
	struct device *dev = &i2c->dev;
//	struct otg_notify *o_notify = get_otg_notify();

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0) {
		dev_err(dev, "%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
//		if (o_notify)
//			o_notify->hw_param[USB_CCIC_I2C_ERROR_COUNT]++;
		return ret;
	}
	ret &= 0xff;
	*dest = ret;
	return 0;
}

int sm5713_usbpd_multi_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	int ret;
	struct device *dev = &i2c->dev;
//	struct otg_notify *o_notify = get_otg_notify();

	ret = i2c_smbus_read_i2c_block_data(i2c, reg, count, buf);
	if (ret < 0) {
		dev_err(dev, "%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
//		if (o_notify)
//			o_notify->hw_param[USB_CCIC_I2C_ERROR_COUNT]++;
		return ret;
	}
	return 0;
}

void sm5713_usbpd_test_read(void)
{
	u8 data;
	sm5713_usbpd_read_reg(test_i2c, 0x3f, &data); // default - 0x97
	pr_info("%s 0x3F = %x\n\n\n", __func__, data);
}

EXPORT_SYMBOL_GPL(sm5713_usbpd_test_read);

int sm5713_usbpd_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	int ret;
	struct device *dev = &i2c->dev;
//	struct otg_notify *o_notify = get_otg_notify();

	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	if (ret < 0) {
		dev_err(dev, "%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
//		if (o_notify)
//			o_notify->hw_param[USB_CCIC_I2C_ERROR_COUNT]++;
	}
	return ret;
}

int sm5713_usbpd_multi_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	int ret;
	struct device *dev = &i2c->dev;
//	struct otg_notify *o_notify = get_otg_notify();

	ret = i2c_smbus_write_i2c_block_data(i2c, reg, count, buf);
	if (ret < 0) {
		dev_err(dev, "%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
//		if (o_notify)
//			o_notify->hw_param[USB_CCIC_I2C_ERROR_COUNT]++;
		return ret;
	}
	return 0;
}

int sm5713_write_msg_header(struct i2c_client *i2c, u8 *buf)
{
	int ret;

	ret = sm5713_usbpd_multi_write(i2c, SM5713_REG_TX_HEADER_00, 2, buf);

	return ret;
}

int sm5713_write_msg_obj(struct i2c_client *i2c, int count, data_obj_type *obj)
{
	int ret = 0;
	int i = 0;
	struct device *dev = &i2c->dev;

	if (count > SM5713_MAX_NUM_MSG_OBJ)
		dev_err(dev, "%s, not invalid obj count number\n", __func__);
	else
		for (i = 0; i < count; i++) {
			ret = sm5713_usbpd_multi_write(i2c,
				SM5713_REG_TX_PAYLOAD_00 + (4 * i),
							4, obj[i].byte);
		}

	return ret;
}

int sm5713_send_msg(void *_data, struct i2c_client *i2c)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
    struct policy_data *policy = &data->policy;
	int ret;
    u8 val;

    if(policy->origin_message == 0x00) {
        val = SM5713_REG_MSG_SEND_TX_SOP_REQ;
    } else if(policy->origin_message == 0x01) {
        val = SM5713_REG_MSG_SEND_TX_SOPP_REQ;    
    } else {
        val = SM5713_REG_MSG_SEND_TX_SOPPP_REQ;    
    }
    pr_info("%s, TX_REQ : %x\n", __func__, val);
	ret = sm5713_usbpd_write_reg(i2c, SM5713_REG_TX_REQ, val);

	return ret;
}

int sm5713_read_msg_header(struct i2c_client *i2c, msg_header_type *header)
{
	int ret;

	ret = sm5713_usbpd_multi_read(i2c, SM5713_REG_RX_HEADER_00, 2, header->byte);

	return ret;
}

int sm5713_read_msg_obj(struct i2c_client *i2c, int count, data_obj_type *obj)
{
	int ret = 0;
	int i = 0;
	struct device *dev = &i2c->dev;

	if (count > SM5713_MAX_NUM_MSG_OBJ) {
		dev_err(dev, "%s, not invalid obj count number\n", __func__);
		ret = -EINVAL; /*TODO: check fail case */
	} else {
		for (i = 0; i < count; i++) {
			ret = sm5713_usbpd_multi_read(i2c,
				SM5713_REG_RX_PAYLOAD_00 + (4 * i),
							4, obj[i].byte);
		}
	}

	return ret;
}

static void sm5713_set_irq_enable(struct sm5713_usbpd_data *_data,
		u8 int0, u8 int1, u8 int2, u8 int3, u8 int4)
{
	u8 int_mask[5]
		= {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	int ret = 0;
	struct i2c_client *i2c = _data->i2c;
	struct device *dev = &i2c->dev;

	int_mask[0] &= ~int0;
	int_mask[1] &= ~int1;
	int_mask[2] &= ~int2;
	int_mask[3] &= ~int3;
	int_mask[4] &= ~int4;

	ret = i2c_smbus_write_i2c_block_data(i2c, SM5713_REG_INT_MASK1,
			5, int_mask);

	if (ret < 0)
		dev_err(dev, "err write interrupt mask \n");
}

void sm5713_driver_reset(/*struct usbpd_data */ void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	int i;

	pdic_data->status_reg = 0;
	data->wait_for_msg_arrived = 0;
	pdic_data->header.word = 0;
	for (i = 0; i < SM5713_MAX_NUM_MSG_OBJ; i++)
		pdic_data->obj[i].object = 0;

	sm5713_set_irq_enable(pdic_data, ENABLED_INT_1, ENABLED_INT_2,
			ENABLED_INT_3, ENABLED_INT_4, ENABLED_INT_5_PASS);
}

void sm5713_protocol_layer_reset(void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;

    sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL4, SM5713_REG_CNTL_PROTOCOL_RESET_MESSAGE); // Reset Protocol Layer

    pr_info("%s\n", __func__);
}

void sm5713_cc_state_hold_on_off(void *_data, int onoff)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
    u8 val;
    
    sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_CNTL3, &val);
    if (onoff == 1)
        val |= 0x10;
    else
        val &= 0xEF;
    sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL3, val);
    pr_info("%s: CC State Hold [%d], val = %x\n", __func__, onoff, val);
}

void sm5713_error_recovery(void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
    u8 val;

    sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_CNTL3, &val);
    val |= 0x04;
    sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL3, val);
    pr_info("%s: value [%x]\n", __func__, val);
}

void sm5713_src_transition_to_default(void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
    u8 val;

    sm5713_usbpd_read_reg(i2c, SM5713_REG_PD_CNTL2, &val);
    val &= 0xEF;
    sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL2, val); // BIST Off

    sm5713_set_vconn_source(data, USBPD_VCONN_OFF);
    vbus_turn_on_ctrl(pdic_data, 0);
    sm5713_set_dfp(i2c);

    dev_info(pdic_data->dev, "%s\n", __func__);
}

void sm5713_src_transition_to_pwr_on(void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;

    sm5713_set_vconn_source(data, USBPD_VCONN_ON);
    vbus_turn_on_ctrl(pdic_data, 1);

    sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL4, SM5713_ATTACH_SOURCE); // Hard Reset Done Notify to PRL
    dev_info(pdic_data->dev, "%s\n", __func__);    
}


void sm5713_snk_transition_to_default(void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;

    sm5713_cc_state_hold_on_off(data, 1); // Enable CC State Hold    
    sm5713_set_vconn_source(data, USBPD_VCONN_OFF);
    sm5713_set_ufp(i2c);
    
    sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL4, SM5713_ATTACH_SOURCE); // Hard Reset Done Notify to PRL
    dev_info(pdic_data->dev, "%s\n", __func__);

}

bool sm5713_check_vbus_state(void *_data)
{
    struct usbpd_data *pd_data = (struct usbpd_data *) _data;
    struct sm5713_usbpd_data *pdic_data = pd_data->phy_driver_data;
    struct i2c_client *i2c = pdic_data->i2c;
    u8 val;

    sm5713_usbpd_read_reg(i2c, SM5713_REG_PROBE0, &val);
    if (val & 0x40) {
        return true;
    } else {
        return false;
    }
}
static void sm5713_assert_rd(/*struct usbpd_data */ void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	u8 val;

	sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_CNTL7, &val);

	val ^= 0x01;
    // Apply CC State PR_Swap (Att.Src -> Att.Snk)
    sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL7, val);
}

static void sm5713_assert_rp(/*struct usbpd_data */ void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	u8 val;

	sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_CNTL7, &val);

	val ^= 0x01;
    // Apply CC State PR_Swap (Att.Snk -> Att.Src)
    sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL7, val);
}

unsigned sm5713_get_status(/*struct usbpd_data*/ void *_data, unsigned flag)
{
	unsigned ret;
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;

	if (pdic_data->status_reg & flag) {
		ret = pdic_data->status_reg & flag;
        dev_info(pdic_data->dev, "%s: status_reg = (%x)\n", __func__, ret);
		pdic_data->status_reg &= ~flag; /* clear the flag */
		return ret;
	} else {
		return 0;
	}
}

int sm5713_set_vconn_source(/*struct usbpd_data */ void *_data, int val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	u8 reg_data = 0, reg_val = 0;

	if (!pdic_data->vconn_en) {
		pr_err("%s, not support vconn source\n", __func__);
		return -1;
	}

    sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_STATUS, &reg_val);

	if (val == USBPD_VCONN_ON) {
        reg_data = (reg_val & 0x20) ? 0x1A : 0x19;
        sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL5, reg_data);
	} else if (val == USBPD_VCONN_OFF) {
		reg_data = 0x08;
		sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL5, reg_data);
	} else
		return(-1);

	pdic_data->vconn_source = val;
	return 0;
}

bool sm5713_poll_status(/*struct usbpd_data */ void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
//	struct policy_data *policy = &data->policy;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;
	u8 intr[5] = {0};
    u8 status[5] = {0};
	int ret = 0;

	ret = sm5713_usbpd_multi_read(i2c, SM5713_REG_INT1, 5, intr);
	ret = sm5713_usbpd_multi_read(i2c, SM5713_REG_STATUS1, 5, status);

	dev_info(dev, "%s: INT[0x%x 0x%x 0x%x 0x%x 0x%x], STATUS[0x%x 0x%x 0x%x 0x%x 0x%x]\n",
			__func__, intr[0], intr[1], intr[2], intr[3], intr[4], status[0], status[1], status[2], status[3], status[4]);

	if ((intr[0] | intr[1] | intr[2] | intr[3] | intr[4]) == 0) {
		pdic_data->status_reg |= MSG_NONE;
		goto out;
	}

    if ((intr[2] & SM5713_REG_INT_STATUS3_WATER)) {
		if (sm5713_check_water_status(pdic_data) == 0) {
			pdic_data->is_water_detect = false;
		} else {
	        pdic_data->is_water_detect = true;
		}
    }

    if (intr[0] & SM5713_REG_INT_STATUS1_VBUSUVLO || intr[1] & SM5713_REG_INT_STATUS2_VBUS_0V) {
        pdic_data->vbus_state = 2;
    }

    if (intr[0] & SM5713_REG_INT_STATUS1_VBUSPOK) {
        pdic_data->vbus_state = 1;

		if(pdic_data->is_abnormal_state) {
			pdic_data->status_reg |= PLUG_ATTACH; // CC-VBUS Short & TA Attached - Notify to MUIC
		}
//		if(status[0] & SM5713_REG_INT_STATUS1_DET_DETECT && (status[2] & SM5713_REG_INT_STATUS3_WATER_RLS || status[2] & SM5713_REG_INT_STATUS3_WATER)) {
//			sm5713_check_abnormal_attach_status(pdic_data);
//		}	
    }

    if (intr[2] & SM5713_REG_INT_STATUS3_WATER_RLS) {
        if ((intr[2] & SM5713_REG_INT_STATUS3_WATER) == 0 && pdic_data->is_water_detect) {
	        pdic_data->is_water_detect = false;
            process_cc_water_det(pdic_data, 0);
        }
    }

	if (intr[0] & SM5713_REG_INT_STATUS1_DETACH) {
		pdic_data->status_reg |= PLUG_DETACH;
	}

	mutex_lock(&pdic_data->lpm_mutex);
	if ((intr[0] & SM5713_REG_INT_STATUS1_ATTACH) &&
			!pdic_data->lpm_mode && !pdic_data->is_water_detect)
		pdic_data->status_reg |= PLUG_ATTACH;
	mutex_unlock(&pdic_data->lpm_mutex);

    if (intr[3] & SM5713_REG_INT_STATUS4_HRST_RCVED) {
        pdic_data->status_reg |= MSG_HARDRESET;
        goto out;
    }

    if (intr[4] & SM5713_REG_CNTL_PROTOCOL_RESET_MESSAGE) { // JIG Case On
//        sm5713_usbpd_write_reg(i2c, 0x09, 0xFF);
        goto out;
    }

	if ((intr[1] & SM5713_REG_INT_STATUS2_RID_INT) ||
		(status[1] & SM5713_REG_INT_STATUS2_RID_INT)) {
		pdic_data->status_reg |= MSG_RID;
        goto out;
	}

    if (intr[2] & SM5713_REG_INT_STATUS3_VCONN_OCP) {
        sm5713_set_vconn_source(data, USBPD_VCONN_OFF);
    }

    if (intr[3] & SM5713_REG_INT_STATUS4_RX_DONE) {
        usbpd_protocol_rx(data);        
    }
    
    if (intr[3] & SM5713_REG_INT_STATUS4_TX_DONE) {
        data->protocol_tx.status = MESSAGE_SENT;
        pdic_data->status_reg |= MSG_GOODCRC;
        dev_info(pdic_data->dev, "%s: tx status = (%d)\n", __func__, data->protocol_tx.status);
    }

    if (intr[3] & SM5713_REG_INT_STATUS4_TX_DISCARD) {
        usbpd_tx_request_discard(data);
    }

    if (intr[3] & SM5713_REG_INT_STATUS4_TX_SOP_ERR || intr[3] & SM5713_REG_INT_STATUS4_TX_NSOP_ERR) {
        data->protocol_tx.status = TRANSMISSION_ERROR;
    }

    if (intr[3] & SM5713_REG_INT_STATUS4_PRL_RST_DONE) {
        pdic_data->reset_done = 1;
    }

    if (intr[3] & SM5713_REG_INT_STATUS4_HCRST_DONE) {
        pdic_data->reset_done = 1;        
    }

out:

	if (pdic_data->status_reg & data->wait_for_msg_arrived) {
        dev_info(pdic_data->dev, "%s: wait_for_msg_arrived = (%d) \n", __func__, data->wait_for_msg_arrived);
		data->wait_for_msg_arrived = 0;
		complete(&data->msg_arrived);
	}

	return 0;
}

int sm5713_hard_reset(/*struct usbpd_data */ void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	int ret;
    u8 val;

#if defined(CONFIG_SEC_FACTORY)
	if (pdic_data->rid != REG_RID_UNDF && pdic_data->rid != REG_RID_OPEN && pdic_data->rid != REG_RID_MAX)
		return 0;
#endif
    sm5713_usbpd_read_reg(i2c, SM5713_REG_PD_CNTL4, &val);
    val |= SM5713_REG_CNTL_HARD_RESET_MESSAGE;
	ret = sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL4, val);
	if (ret < 0)
		goto fail;

	pdic_data->status_reg = 0;

	return 0;

fail:
	return -EIO;
}

static int sm5713_receive_message(void *_data)
{
    struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
    struct policy_data *policy = &data->policy;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;
	int obj_num = 0;
	int ret = 0;
    u8 val;

	ret = sm5713_read_msg_header(i2c, &pdic_data->header);
	if (ret < 0)
		dev_err(dev, "%s read msg header error\n", __func__);

	obj_num = pdic_data->header.num_data_objs;

	if (obj_num > 0) {
		ret = sm5713_read_msg_obj(i2c,
			obj_num, &pdic_data->obj[0]);
	}

    sm5713_usbpd_read_reg(i2c, SM5713_REG_RX_SRC, &val);
    policy->origin_message = val & 0x0F; // 0: SOP, 1: SOP', 2: SOP", 3: SOP' Debug, 4: SOP" Debug
    dev_info(pdic_data->dev, "%s: Origin of Message = (%x) \n", __func__, val);
    
	return ret;
}

int sm5713_tx_msg(/*struct usbpd_data*/ void *_data,
		msg_header_type *header, data_obj_type *obj)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	int ret = 0;
	int count = 0;

	mutex_lock(&pdic_data->_mutex);

	/* if there is no attach, skip tx msg */
	if (pdic_data->detach_valid)
		goto done;

	ret = sm5713_write_msg_header(i2c, header->byte);
	if (ret < 0)
		goto done;

	count = header->num_data_objs;

	if (count > 0) {
		ret = sm5713_write_msg_obj(i2c, count, obj);
		if (ret < 0)
			goto done;
	}

	ret = sm5713_send_msg(data, i2c);
	if (ret < 0)
		goto done;

	pdic_data->status_reg = 0;
	data->wait_for_msg_arrived = 0;

done:
	mutex_unlock(&pdic_data->_mutex);
	return ret;
}

int sm5713_rx_msg(/*struct usbpd_data */ void *_data,
		msg_header_type *header, data_obj_type *obj)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	int i;
	int count = 0;

	if (!sm5713_receive_message(data)) {
		header->word = pdic_data->header.word;
		count = pdic_data->header.num_data_objs;
		if (count > 0) {
			for (i = 0; i < count; i++)
				obj[i].object = pdic_data->obj[i].object;
		}
		pdic_data->header.word = 0; /* To clear for duplicated call */
		return 0;
	} else {
		return -EINVAL;
	}
}

int sm5713_get_vconn_source(/*struct usbpd_data */ void *_data, int *val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;

	if (pdic_data->vconn_source != *val) {
		dev_info(pdic_data->dev, "%s, vconn_source(%d) != gpio val(%d)\n",
				__func__, pdic_data->vconn_source, *val);
		pdic_data->vconn_source = *val;
	}

	return 0;
}

/* val : sink(0) or source(1) */
int sm5713_set_power_role(/*struct usbpd_data */ void *_data, int val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;

	pr_info("%s: val : (%s)\n", __func__, val == 1 ? "SOURCE" : "SINK");

	if (val == USBPD_SINK) {
		sm5713_assert_rd(data);
		sm5713_set_snk(pdic_data->i2c);
	} else/* if (val == USBPD_SOURCE)*/ {
		sm5713_assert_rp(data);
		sm5713_set_src(pdic_data->i2c);
	}

	pdic_data->power_role = val;
	return 0;
}

int sm5713_get_power_role(/*struct usbpd_data */ void *_data, int *val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	*val = pdic_data->power_role;
	return 0;
}

int sm5713_set_data_role(/*struct usbpd_data */ void *_data, int val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;

	/* DATA_ROLE
	 * 0 : UFP
	 * 1 : DFP
	 */
	if (val == USBPD_UFP) {
		sm5713_set_ufp(i2c);
	} else {/* (val == USBPD_DFP) */
		sm5713_set_dfp(i2c);
	}

	pdic_data->data_role = val;

#if defined(CONFIG_CCIC_NOTIFIER)
	process_dr_swap(pdic_data);
#endif
	return 0;
}

int sm5713_get_data_role(/*struct usbpd_data */ void *_data, int *val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;
	*val = pdic_data->data_role;
	return 0;
}

int sm5713_set_check_msg_pass(/*struct usbpd_data */ void *_data, int val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct sm5713_usbpd_data *pdic_data = data->phy_driver_data;

	dev_info(pdic_data->dev, "%s: check_msg_pass val(%d)\n", __func__, val);

	pdic_data->check_msg_pass = val;

	return 0;
}

#ifndef CONFIG_SEC_FACTORY
static void sm5713_usbpd_set_rp_scr_sel(struct sm5713_usbpd_data *pdic_data, CCIC_RP_SCR_SEL scr_sel)
{
	struct i2c_client *i2c = pdic_data->i2c;
	u8 data = 0;
	pr_info("%s: scr_sel : (%d)\n", __func__, scr_sel);
	switch (scr_sel) {
	case PLUG_CTRL_RP80:
		sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_CNTL1, &data);
		data &= 0xCF;
		sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL1, data);
		break;
	case PLUG_CTRL_RP180:
		sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_CNTL1, &data);
        data &= 0xDF;
		data |= 0x10;
		sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL1, data);
		break;
	default:
		break;
	}
	return;
}
#endif


static void sm5713_set_dfp(struct i2c_client *i2c)
{
	u8 data;

	sm5713_usbpd_read_reg(i2c, SM5713_REG_PD_CNTL2, &data);
	data |= 0x01;
	sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL2, data);

    sm5713_usbpd_read_reg(i2c, SM5713_REG_PD_CNTL1, &data);
    data |= 0xF0;
    sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL1, data);    
}

static void sm5713_set_ufp(struct i2c_client *i2c)
{
	u8 data;
	sm5713_usbpd_read_reg(i2c, SM5713_REG_PD_CNTL2, &data);
	data &= ~0x01;
	sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL2, data);

    sm5713_usbpd_read_reg(i2c, SM5713_REG_PD_CNTL1, &data);
    data &= 0x0F;
    sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL1, data);
}

static void sm5713_set_src(struct i2c_client *i2c)
{
	u8 data;

	sm5713_usbpd_read_reg(i2c, SM5713_REG_PD_CNTL2, &data);
	data |= 0x02;
	sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL2, data);
}

static void sm5713_set_snk(struct i2c_client *i2c)
{
	u8 data;

	sm5713_usbpd_read_reg(i2c, SM5713_REG_PD_CNTL2, &data);
	data &= ~0x02;
	sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL2, data);
}

#if defined(CONFIG_CCIC_NOTIFIER)
void sm5713_control_option_command (struct sm5713_usbpd_data *pdic_data, int cmd) {
	struct usbpd_data *_data = dev_get_drvdata(pdic_data->dev);
	int pd_cmd = cmd & 0x0f;

/* 0x1 : Vconn control option command ON
 * 0x2 : Vconn control option command OFF
 * 0x3 : Water Detect option command ON
 * 0x4 : Water Detect option command OFF
 */
	switch (pd_cmd) {
	case 1:
		sm5713_set_vconn_source(_data, USBPD_VCONN_ON);
		break;
	case 2:
		sm5713_set_vconn_source(_data, USBPD_VCONN_OFF);
		break;
	case 3:
	case 4:
		pr_err("%s : not implement water control\n", __func__);
		break;
	default:
		break;
	}
}
#endif

static void sm5713_notify_pdic_rid(struct sm5713_usbpd_data *pdic_data, int rid)
{
#if defined(CONFIG_CCIC_NOTIFIER)
	pdic_data->is_factory_mode = false;
	if (rid == RID_523K)
		pdic_data->is_factory_mode = true;
	/* rid */
	ccic_event_work(pdic_data,
		CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_RID, rid/*rid*/, USB_STATUS_NOTIFY_DETACH);

	if (rid == REG_RID_523K || rid == REG_RID_619K || rid == REG_RID_OPEN)
		ccic_event_work(pdic_data,
			CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, CCIC_NOTIFY_DETACH/*attach*/, USB_STATUS_NOTIFY_DETACH);
#else
	muic_attached_dev_t new_dev;
	pdic_data->is_factory_mode = false;
	switch (rid) {
	case REG_RID_255K:
		new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
		break;
	case REG_RID_301K:
		new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
		break;
	case REG_RID_523K:
		new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		pdic_data->is_factory_mode = true;
		break;
	case REG_RID_619K:
		new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
		break;
	default:
		new_dev = ATTACHED_DEV_NONE_MUIC;
		return;
	}
	sm5713_pdic_notifier_attach_attached_jig_dev(new_dev);
#endif
	dev_info(pdic_data->dev, "%s : attached rid state(%d)", __func__, rid);
}

/* #if defined(CONFIG_CCIC_FACTORY) */
static void sm5713_usbpd_check_rid(struct sm5713_usbpd_data *pdic_data)
{
	struct i2c_client *i2c = pdic_data->i2c;
	u8 rid;
	int prev_rid = pdic_data->rid;

	sm5713_usbpd_read_reg(i2c, SM5713_REG_FACTORY, &rid);

	dev_info(pdic_data->dev, "%s : attached rid state(%d)", __func__, rid);

	if (rid) {
		if (prev_rid != rid) {
			pdic_data->rid = rid;
			if (prev_rid >= REG_RID_OPEN && rid >= REG_RID_OPEN)
				dev_err(pdic_data->dev, "%s : rid is not changed, skip notify(%d)", __func__, rid);
			else
				sm5713_notify_pdic_rid(pdic_data, rid);
		}

		if (rid >= REG_RID_MAX) {
			dev_err(pdic_data->dev, "%s : overflow rid value", __func__);
			return;
		}
	}
}
#if defined(CONFIG_DUAL_ROLE_USB_INTF)

static int sm5713_set_attach(struct sm5713_usbpd_data *pdic_data, u8 mode)
{
	int ret = 0;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;

	if (mode == TYPE_C_ATTACH_DFP) {
        sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL1, 0x88);        
        sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL3, 0x81);
	} else if (mode == TYPE_C_ATTACH_UFP) {
        sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL1, 0x84);        	
        sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL3, 0x82);
	}	

	dev_info(dev, "%s sm5713 force to attach\n", __func__);

	return ret;
}

static int sm5713_set_detach(struct sm5713_usbpd_data *pdic_data, u8 mode)
{
	u8 data;
	int ret = 0;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;

    sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_CNTL3, &data);
    data |= 0x08;
    sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL3, data);

	dev_info(dev, "%s sm5713 force to detach\n", __func__);

	return ret;
}
#endif
int sm5713_set_normal_mode(struct sm5713_usbpd_data *pdic_data)
{
	int ret = 0;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;


    sm5713_usbpd_write_reg(i2c, SM5713_REG_CORR_CNTL4, 0xC0);

	pdic_data->lpm_mode = false;

    sm5713_set_irq_enable(pdic_data, ENABLED_INT_1, ENABLED_INT_2, ENABLED_INT_3, ENABLED_INT_4,
                ENABLED_INT_5_PASS);


	dev_info(dev, "%s sm5713 exit lpm mode\n", __func__);

	return ret;
}

int sm5713_set_lpm_mode(struct sm5713_usbpd_data *pdic_data)
{
	int ret = 0;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;

	pdic_data->lpm_mode = true;

    sm5713_usbpd_write_reg(i2c, SM5713_REG_CORR_CNTL4, 0xC7);

//	sm5713_set_irq_enable(pdic_data, 0, 0, 0, 0, 0);

	dev_info(dev, "%s sm5713 enter lpm mode\n", __func__);

	return ret;
}

static void sm5713_set_src_sink_init(struct work_struct *work)
{
	struct sm5713_usbpd_data *pdic_data =
		container_of(work, struct sm5713_usbpd_data, plug_work.work);
	struct usbpd_data *pd_data = dev_get_drvdata(pdic_data->dev);
	bool plug_valid = pd_data->policy.plug_valid;
	struct i2c_client *i2c = pdic_data->i2c;
	u8 data = 0;

	mutex_lock(&pdic_data->_mutex);

	sm5713_usbpd_read_reg(i2c, SM5713_REG_FACTORY, &data);

	pr_info("%s, plug_valid = %x, rid = %x\n", __func__, plug_valid, data);

	if (!plug_valid) {
		sm5713_set_ufp(i2c);
		sm5713_set_snk(i2c);

        sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_CNTL3, &data);
        data |= 0x08;
        sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL3, data); // CC state goes to DISABLE State
    
        sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL1, 0x00); // Disable PD Function
        sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL4, 0x00); // Protocol Layer reset
        sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL3, 0x82); // CC state goes to Unattached SNK State

//		if (!pdic_data->lpm_mode)
//			sm5713_set_lpm_mode(pdic_data);
	}

	mutex_unlock(&pdic_data->_mutex);
}

#if !defined(CONFIG_MUIC_NOTIFIER)
static int type3_handle_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
#if defined(CONFIG_CCIC_NOTIFIER)
	CC_NOTI_ATTACH_TYPEDEF *p_noti = (CC_NOTI_ATTACH_TYPEDEF *)data;
	muic_attached_dev_t attached_dev = p_noti->cable_type;
#else
	muic_attached_dev_t attached_dev = *(muic_attached_dev_t *)data;
#endif
	struct sm5713_usbpd_data *pdic_data =
		container_of(nb, struct sm5713_usbpd_data,
			     type3_nb);
	struct i2c_client *i2c = pdic_data->i2c;
//	u8 reg_data = 0;

	struct otg_notify *o_notify = get_otg_notify();

	mutex_lock(&pdic_data->lpm_mutex);
	pr_info("%s action:%d, attached_dev:%d, lpm:%d, pdic_data->is_otg_vboost:%d, pdic_data->is_otg_reboost:%d\n",
		__func__, (int)action, (int)attached_dev, pdic_data->lpm_mode,
		(int)pdic_data->is_otg_vboost, (int)pdic_data->is_otg_reboost);

	if ((action == MUIC_PDIC_NOTIFY_CMD_ATTACH) &&
		(attached_dev == ATTACHED_DEV_TYPE3_MUIC)) {
		if (pdic_data->lpm_mode) {
			pr_info("%s try to exit lpm mode-->\n", __func__);
			sm5713_set_normal_mode(pdic_data);
			pr_info("%s after exit lpm mode<--\n", __func__);
		}
	} else if ((action == MUIC_PDIC_NOTIFY_CMD_ATTACH) &&
		attached_dev == ATTACHED_DEV_WATER_MUIC) {
		pr_info("%s, ATTACH : ATTACHED_DEV_WATER_MUIC(WATER)\n", __func__);
		pdic_data->is_water_detect = true;

		sm5713_set_lpm_mode(pdic_data);

        sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL1, 0x00); // Disable PD Function

//		if (o_notify)
//			o_notify->hw_param[USB_CCIC_WATER_INT_COUNT]++;
	} else if ((action == MUIC_PDIC_NOTIFY_CMD_DETACH) &&
		attached_dev == ATTACHED_DEV_WATER_MUIC) {
		pr_info("%s, DETACH : ATTACHED_DEV_WATER_MUIC(DRY)\n", __func__);
		pdic_data->is_water_detect = false;

        sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL1, 0x03); // PD Enable

//		if (o_notify)
//			o_notify->hw_param[USB_CCIC_DRY_INT_COUNT]++;
	} else if (action == MUIC_PDIC_NOTIFY_CMD_DETACH) {
		if (!pdic_data->lpm_mode) {
			pr_info("%s try to enter lpm mode-->\n", __func__);
			sm5713_set_lpm_mode(pdic_data);
			pr_info("%s after enter lpm mode<--\n", __func__);
		}
	}
#ifndef CONFIG_SEC_FACTORY
	else if ((action == MUIC_PDIC_NOTIFY_CMD_ATTACH)
			&& (attached_dev == ATTACHED_DEV_CHECK_OCP)
			&& pdic_data->is_otg_vboost
			&& pdic_data->data_role_dual == USB_STATUS_NOTIFY_ATTACH_DFP) {
		if (o_notify) {
			if (is_blocked(o_notify, NOTIFY_BLOCK_TYPE_HOST)) {
				pr_info("%s, upsm mode, skip OCP handling\n", __func__);
				goto EOH;
			}
		}
		if (pdic_data->is_otg_reboost) {
			/* todo : over current event to platform */
			pr_info("%s, CHECK_OCP, Can't afford it(OVERCURRENT)\n", __func__);
			goto EOH;
		}
		ccic_event_work(pdic_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 1/*attach*/, 1/*rprd*/);

		pr_info("%s, CHECK_OCP, start OCP W/A\n", __func__);
	}
EOH:
#endif
	mutex_unlock(&pdic_data->lpm_mutex);

	return 0;
}
#endif

int sm5713_check_port_detect(struct sm5713_usbpd_data *pdic_data)
{
	u8 data, val;
	int ret = 0;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;
	struct usbpd_data *pd_data = dev_get_drvdata(dev);
	struct otg_notify *o_notify = get_otg_notify();

	ret = sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_STATUS, &data);
	if (ret < 0)
		dev_err(dev, "%s, i2c read CC_STATUS error\n", __func__);

	if ((data & SM5713_ATTACH_TYPE) == SM5713_ATTACH_SOURCE) {
		pdic_data->power_role = PDIC_SINK;
		pdic_data->data_role = USBPD_UFP;
		sm5713_set_snk(i2c);
		sm5713_set_ufp(i2c);
   	    sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL1, 0x03); // PD Enable
    	sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_CNTL7, &val);
		val |= 0x04;
   	    sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL7, val);		
		dev_info(dev, "PDIC_SINK\n");
		if (pdic_data->is_factory_mode == true)
#if defined(CONFIG_CCIC_NOTIFIER)
		{
			/* muic */
			ccic_event_work(pdic_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, CCIC_NOTIFY_ATTACH/*attach*/, USB_STATUS_NOTIFY_DETACH/*rprd*/);
			return true;
		}
#else
		return true;
#endif
		usbpd_policy_reset(pd_data, PLUG_EVENT);
		cancel_delayed_work_sync(&pdic_data->plug_work);
#if defined(CONFIG_CCIC_NOTIFIER)
		dev_info(&i2c->dev, "%s %d: is_host = %d, is_client = %d\n",
					__func__, __LINE__, pdic_data->is_host, pdic_data->is_client);
		pdic_data->is_attached = 1;

		if (pdic_data->is_host == HOST_ON) {
			dev_info(&i2c->dev, "%s %d: turn off host\n", __func__, __LINE__);
			ccic_event_work(pdic_data,
				CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, CCIC_NOTIFY_DETACH/*attach*/, USB_STATUS_NOTIFY_ATTACH_DFP/*rprd*/);
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			pdic_data->power_role_dual = DUAL_ROLE_PROP_PR_NONE;
#endif
			/* add to turn off external 5V */
			vbus_turn_on_ctrl(pdic_data, 0);
//			muic_disable_otg_detect();
			ccic_event_work(pdic_data,
				CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, CCIC_NOTIFY_DETACH/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
			pdic_data->is_host = HOST_OFF;
			msleep(300);
		}

		/* muic */
		ccic_event_work(pdic_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, CCIC_NOTIFY_ATTACH/*attach*/, USB_STATUS_NOTIFY_DETACH/*rprd*/);
		if (!(pdic_data->rid == REG_RID_523K || pdic_data->rid == REG_RID_619K)) {
			if (pdic_data->is_client == CLIENT_OFF && pdic_data->is_host == HOST_OFF) {
				/* usb */
				pdic_data->is_client = CLIENT_ON;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
				pdic_data->power_role_dual = DUAL_ROLE_PROP_PR_SNK;
#endif
				ccic_event_work(pdic_data,
					CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, CCIC_NOTIFY_ATTACH/*attach*/, USB_STATUS_NOTIFY_ATTACH_UFP/*drp*/);
			}
		}
#endif
        sm5713_set_vconn_source(pd_data, USBPD_VCONN_OFF);
//        sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL4, 0x08); // Reset Protocol Layer
	} else if ((data & SM5713_ATTACH_TYPE) == SM5713_ATTACH_SINK) {
		pdic_data->power_role = PDIC_SOURCE;
		pdic_data->data_role = USBPD_DFP;

        sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL1, 0xF3); // PD Enable
		dev_info(dev, "PDIC_SOURCE\n");
		usbpd_policy_reset(pd_data, PLUG_EVENT);
#if defined(CONFIG_CCIC_NOTIFIER)
		dev_info(&i2c->dev, "%s %d: is_host = %d, is_client = %d\n",
						__func__, __LINE__, pdic_data->is_host, pdic_data->is_client);
		pdic_data->is_attached = 1;

		if (pdic_data->is_client == CLIENT_ON) {
			dev_info(&i2c->dev, "%s %d: turn off client\n", __func__, __LINE__);
			ccic_event_work(pdic_data,
				CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, CCIC_NOTIFY_DETACH/*attach*/, USB_STATUS_NOTIFY_DETACH/*rprd*/);
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			pdic_data->power_role_dual = DUAL_ROLE_PROP_PR_NONE;
#endif
			ccic_event_work(pdic_data,
				CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, CCIC_NOTIFY_DETACH/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
			pdic_data->is_client = CLIENT_OFF;
			/* msleep(300); */
		}

		if (pdic_data->is_host == HOST_OFF) {
			/* muic */
			ccic_event_work(pdic_data,
				CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, CCIC_NOTIFY_ATTACH/*attach*/, USB_STATUS_NOTIFY_ATTACH_DFP/*rprd*/);
			/* otg */
			pdic_data->is_host = HOST_ON;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			pdic_data->power_role_dual = DUAL_ROLE_PROP_PR_SRC;
#endif
			/* USB */
			ccic_event_work(pdic_data,
					CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, CCIC_NOTIFY_ATTACH/*attach*/, USB_STATUS_NOTIFY_ATTACH_DFP/*drp*/);
			/* add to turn on external 5V */
			if (!is_blocked(o_notify, NOTIFY_BLOCK_TYPE_HOST))
				vbus_turn_on_ctrl(pdic_data, 1);
		}
#else
		usbpd_manager_plug_attach(dev, ATTACHED_DEV_TYPE3_ADAPTER_MUIC);
#endif
		cancel_delayed_work_sync(&pdic_data->plug_work);

		sm5713_set_vconn_source(pd_data, USBPD_VCONN_ON);

		sm5713_set_dfp(i2c);
		sm5713_set_src(i2c);

		msleep(400); /* dont over 310~620ms(tTypeCSinkWaitCap) */
	} else {
		dev_err(dev, "%s, PLUG Error\n", __func__);
		return -1;
	}

	pdic_data->detach_valid = false;

	return ret;
}

int sm5713_check_init_port(struct sm5713_usbpd_data *pdic_data)
{
	u8 data;
	int ret = 0;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;

	ret = sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_STATUS, &data);
	if (ret < 0)
		dev_err(dev, "%s, i2c read CC_STATUS error\n", __func__);

	if ((data & SM5713_ATTACH_TYPE) == SM5713_ATTACH_SINK)
		return PDIC_SOURCE;

	return 0;
}

void sm5713_int_clear(void *_data)
{
	struct sm5713_usbpd_data *pdic_data = _data;
	struct i2c_client *i2c = pdic_data->i2c;
	u8 intr[5] = {0};
	int result = 0;

	result = sm5713_usbpd_multi_read(i2c, SM5713_REG_INT1, 5, intr);
	pr_info("%s, Interrupt Clear\n", __func__);
}

int sm5713_check_water_status(void *_data)
{
	struct sm5713_usbpd_data *pdic_data = _data;
	struct i2c_client *i2c = pdic_data->i2c;
	u8 st1, st3, adc_value;
	int ret = 0;

	sm5713_set_irq_enable(pdic_data, 0, 0, 0, 0, 0);

	sm5713_usbpd_write_reg(i2c, SM5713_REG_CORR_CNTL4, 0xC0);
	sm5713_usbpd_write_reg(i2c, SM5713_REG_CORR_CNTL4, 0xC7);
	msleep(500);

	sm5713_usbpd_read_reg(i2c, SM5713_REG_STATUS1, &st1);
	sm5713_usbpd_read_reg(i2c, SM5713_REG_STATUS3, &st3);
	pr_info("%s, STATUS1 = %x, STATUS3 = %x\n", __func__, st1, st3);

	if(st3 & SM5713_REG_INT_STATUS3_WATER) {
		if(st1 & SM5713_REG_INT_STATUS1_DET_DETECT) {
			sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL3, 0x88);

	        sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL5, 0x1A); // VCONN_CC1
			sm5713_usbpd_write_reg(i2c, SM5713_REG_ADC_CTRL1, 0x0F); // ADC - VBUS
			sm5713_usbpd_read_reg(i2c, SM5713_REG_ADC_CTRL2, &adc_value);
			if(adc_value >= 0x13) { // 3.1234V
				pdic_data->is_abnormal_state = true; // CC1-VBUS SHORT
				ret = 0;
			} else {
	    	    sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL5, 0x19); // VCONN_CC2
				sm5713_usbpd_write_reg(i2c, SM5713_REG_ADC_CTRL1, 0x0F); // ADC - VBUS
				sm5713_usbpd_read_reg(i2c, SM5713_REG_ADC_CTRL2, &adc_value);
				if(adc_value >= 0x13) { // 3.1234V
					pdic_data->is_abnormal_state = true; // CC2-VBUS SHORT
					ret = 0;
				}
			}
			sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL3, 0x80);
			sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL5, 0x80);
		}

		if(!pdic_data->is_abnormal_state) {
			pr_info("%s, real water\n", __func__);
			ret = -1;
		}

		sm5713_int_clear(pdic_data);
	}

	sm5713_set_irq_enable(pdic_data, ENABLED_INT_1, ENABLED_INT_2, ENABLED_INT_3, ENABLED_INT_4,
					ENABLED_INT_5_PASS);

	return ret;
}

void sm5713_check_abnormal_attach_status(void *_data)
{
	struct sm5713_usbpd_data *pdic_data = _data;
	struct i2c_client *i2c = pdic_data->i2c;
	u8 adc_value, reg_data;

	pr_info("%s\n",__func__);

	sm5713_usbpd_write_reg(i2c, SM5713_REG_INT_MASK1, SM5713_REG_INT_STATUS1_ADC_DONE);

	// 0x05 : CC1(Non-Flip), 0x07 : CC2(Flip)
	sm5713_usbpd_write_reg(i2c, SM5713_REG_ADC_CTRL1, SM5713_ADC_PATH_SEL_CC1);
	// Need to Delay?
	sm5713_usbpd_read_reg(i2c, SM5713_REG_ADC_CTRL2, &adc_value);

	if(adc_value >= 0xAD) {
		pdic_data->status_reg |= PLUG_ATTACH; // CC-VBUS Short & TA Attached - Notify to MUIC
		goto out;
	}

	sm5713_usbpd_write_reg(i2c, SM5713_REG_ADC_CTRL1, SM5713_ADC_PATH_SEL_CC2);
	sm5713_usbpd_read_reg(i2c, SM5713_REG_ADC_CTRL2, &adc_value);

	if(adc_value >= 0xAD) {
		pdic_data->status_reg |= PLUG_ATTACH; // CC-VBUS Short & TA Attached - Notify to MUIC
		goto out;
	}

	// 0x09 : SBU1(Non-Flip), 0x0B : SBU2(Flip)
	sm5713_usbpd_write_reg(i2c, SM5713_REG_ADC_CTRL1, SM5713_ADC_PATH_SEL_SBU1);
	sm5713_usbpd_read_reg(i2c, SM5713_REG_ADC_CTRL2, &adc_value);

	if(adc_value >= 0xAE) {
		pdic_data->status_reg |= MSG_NONE; // Notify to AP (No Charging) - SBU-VBUS short & TA Attached
		goto out;
    }

	// 0x09 : SBU1(Non-Flip), 0x0B : SBU2(Flip)
	sm5713_usbpd_write_reg(i2c, SM5713_REG_ADC_CTRL1, SM5713_ADC_PATH_SEL_SBU2);
	sm5713_usbpd_read_reg(i2c, SM5713_REG_ADC_CTRL2, &adc_value);

	if(adc_value >= 0xAE) {
		pdic_data->status_reg |= MSG_NONE; // Notify to AP (No Charging) - SBU-VBUS short & TA Attached
		goto out;
	}

out:
	sm5713_usbpd_read_reg(i2c, SM5713_REG_INT1, &reg_data);
	sm5713_usbpd_write_reg(i2c, SM5713_REG_INT_MASK1, 0x00);
}


void process_cc_water_det(void * data, int state)
{    
    struct sm5713_usbpd_data *pdic_data = data;
   
    ccic_event_work(pdic_data, CCIC_NOTIFY_DEV_BATTERY, CCIC_NOTIFY_ID_WATER, state/*attach*/, USB_STATUS_NOTIFY_DETACH);    
    pr_info("%s, water state : %d\n",__func__, state);
}


static irqreturn_t sm5713_ccic_irq_thread(int irq, void *data)
{
	struct sm5713_usbpd_data *pdic_data = data;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;
	struct usbpd_data *pd_data = dev_get_drvdata(dev);
	int ret = 0;
	u8 reg_data;
	unsigned rid_status = 0;
#if defined(CONFIG_SEC_FACTORY)
	u8 rid;
#endif /* CONFIG_SEC_FACTORY */

	dev_info(dev, "%s\n", __func__);

	mutex_lock(&pdic_data->_mutex);

	sm5713_poll_status(pd_data);

	if (sm5713_get_status(pd_data, MSG_NONE))
		goto out;

    if (pdic_data->is_water_detect) {
        process_cc_water_det(pdic_data, 1);
        goto out;
    }
	if (sm5713_get_status(pd_data, MSG_HARDRESET)) {
		usbpd_rx_hard_reset(dev);
		usbpd_kick_policy_work(dev);
		goto out;
	}

    if (sm5713_get_status(pd_data, MSG_SOFTRESET)) {
        usbpd_rx_soft_reset(pd_data);
  	    usbpd_kick_policy_work(dev);
        goto out;
    }

    if (sm5713_get_status(pd_data, PLUG_ATTACH)) {
		rid_status = sm5713_get_status(pd_data, MSG_RID);

    	ret = sm5713_check_port_detect(pdic_data);
		if (ret >= 0) {
			if (rid_status) {
				sm5713_usbpd_check_rid(pdic_data);
			}
			goto hard_reset;
		}        
    }

	if (sm5713_get_status(pd_data, PLUG_DETACH)) {
#if defined(CONFIG_SEC_FACTORY)
		if (pdic_data->rid == REG_RID_619K) {
			msleep(250);
			sm5713_usbpd_read_reg(i2c, SM5713_REG_FACTORY, &rid);

			dev_info(&i2c->dev, "%s %d: Detached, check if still 619K? => 0x%X\n",
					__func__, __LINE__, rid);
			if (rid == REG_RID_619K)
				goto skip_detach;
		}
#endif /* CONFIG_SEC_FACTORY */
#ifndef CONFIG_SEC_FACTORY
		if (pdic_data->is_otg_reboost) {
			dev_info(&i2c->dev, "%s %d: Detached, go back to 180uA\n",
					__func__, __LINE__);
			sm5713_usbpd_set_rp_scr_sel(pdic_data, PLUG_CTRL_RP180);
			pdic_data->is_otg_reboost = false;
		}
#endif
//		attach_status = sm5713_get_status(pd_data, PLUG_ATTACH);
//		rid_status = sm5713_get_status(pd_data, MSG_RID);
		pdic_data->status_reg = 0;
		usbpd_reinit(dev);

		pdic_data->rid = REG_RID_MAX;
#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
		pd_noti.sink_status.current_pdo_num = 0;
		pd_noti.sink_status.selected_pdo_num = 0;
#endif

		sm5713_set_vconn_source(pd_data, USBPD_VCONN_OFF);
		if (!pdic_data->try_state_change) {
			sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL3, 0x88);
		}

		pdic_data->detach_valid = true;
		pdic_data->is_factory_mode = false;
        pdic_data->vbus_state = 0;
        pdic_data->vbus_tran_st = 0;
		pdic_data->is_abnormal_state = false;

#if defined	(CONFIG_CCIC_NOTIFIER)
		pdic_data->is_attached = 0;

//		usbpd_manager_plug_detach(dev, 0);
        usbpd_policy_reset(pd_data, PLUG_DETACHED);

		/* MUIC */
		ccic_event_work(pdic_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, CCIC_NOTIFY_DETACH/*attach*/, USB_STATUS_NOTIFY_DETACH/*rprd*/);

		ccic_event_work(pdic_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_RID, REG_RID_OPEN/*rid*/, USB_STATUS_NOTIFY_DETACH);
		if (pdic_data->is_host > HOST_OFF || pdic_data->is_client > CLIENT_OFF) {
			if (pdic_data->is_host > HOST_OFF) {
				vbus_turn_on_ctrl(pdic_data, 0);
//				muic_disable_otg_detect();
			}
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			pr_info("%s, data_role (%d)\n", __func__, pdic_data->data_role_dual);
/*
			if (pdic_data->data_role_dual == USB_STATUS_NOTIFY_ATTACH_DFP &&
				!pdic_data->try_state_change) {
				sm5713_usbpd_read_reg(i2c, 0x27, &reg_data);
				reg_data |= 0x1 << 7;
				sm5713_usbpd_write_reg(i2c, 0x27, reg_data);

				msleep(200);

				sm5713_usbpd_read_reg(i2c, 0x27, &reg_data);
				reg_data &= ~(0x1 << 7);
				sm5713_usbpd_write_reg(i2c, 0x27, reg_data);
			}
*/			
#endif
			/* usb or otg */
			dev_info(&i2c->dev, "%s %d: is_host = %d, is_client = %d\n",
					__func__, __LINE__, pdic_data->is_host, pdic_data->is_client);
			pdic_data->is_host = HOST_OFF;
			pdic_data->is_client = CLIENT_OFF;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			pdic_data->power_role_dual = DUAL_ROLE_PROP_PR_NONE;
#endif
			/* USB */
			ccic_event_work(pdic_data,
				CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, CCIC_NOTIFY_DETACH/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			if (!pdic_data->try_state_change)
				sm5713_rprd_mode_change(pdic_data, TYPE_C_ATTACH_DRP);
#endif
		}
#else
		usbpd_manager_plug_detach(dev, 1);
#endif

        sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL2, 0x00); // Set Sink / UFP
        sm5713_usbpd_read_reg(i2c, 0xD8, &reg_data);
		if(reg_data & 0x06) {
	        sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL4, 0x01); // Reset Done			
		} else {
		    sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL4, 0x00); // Protocol Layer reset
		}
        sm5713_usbpd_write_reg(i2c, SM5713_REG_PD_CNTL1, 0x00); // Disable PD Function
		sm5713_usbpd_read_reg(i2c, SM5713_REG_CC_CNTL7, &reg_data);
		reg_data &= 0xFB;
		sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL7, reg_data);
   	    sm5713_usbpd_write_reg(i2c, SM5713_REG_INT_MASK4, 0x00);
		if (!pdic_data->try_state_change) {		
			sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL3, 0x82);
		}

		goto out;
	}
#if defined(CONFIG_SEC_FACTORY)
skip_detach:
#endif /* CONFIG_SEC_FACTORY */
	if (sm5713_get_status(pd_data, PLUG_ATTACH)) {
		if (sm5713_check_port_detect(data) < 0)
			goto out;
	}

	if (sm5713_get_status(pd_data, MSG_RID)) {
		sm5713_usbpd_check_rid(pdic_data);
	}

hard_reset:
	mutex_lock(&pdic_data->lpm_mutex);
	if (!pdic_data->lpm_mode)
		usbpd_kick_policy_work(dev);
	mutex_unlock(&pdic_data->lpm_mutex);
out:
	mutex_unlock(&pdic_data->_mutex);

	return IRQ_HANDLED;
}

static int sm5713_usbpd_reg_init(struct sm5713_usbpd_data *_data)
{
	struct i2c_client *i2c = _data->i2c;
	pr_info("%s",__func__);
	sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL1, 0x80); // Release SNK Only
    sm5713_usbpd_write_reg(i2c, SM5713_REG_CC_CNTL3, 0x80); // Set CC_OP_EN

    sm5713_usbpd_write_reg(i2c, 0xA2, 0x12);
    sm5713_usbpd_write_reg(i2c, SM5713_REG_CORR_CNTL4, 0xC0);
    sm5713_usbpd_write_reg(i2c, 0x22, 0x03);
    sm5713_usbpd_write_reg(i2c, 0x9F, 0x8F);
    sm5713_usbpd_write_reg(i2c, 0xA4, 0x03);
    sm5713_usbpd_write_reg(i2c, 0xA5, 0x0A);
    sm5713_usbpd_write_reg(i2c, SM5713_REG_CORR_CNTL4, 0xC7);

	return 0;
}

#if 0
static irqreturn_t sm5713_irq_isr(int irq, void *data)
{
	return IRQ_WAKE_THREAD;
}
#endif

static int sm5713_usbpd_irq_init(struct sm5713_usbpd_data *_data)
{
	struct i2c_client *i2c = _data->i2c;
	struct device *dev = &i2c->dev;
	int ret = 0;

	pr_info("%s",__func__);
	if (!_data->irq_gpio) {
		dev_err(dev, "%s No interrupt specified\n", __func__);
		return -ENXIO;
	}

	i2c->irq = gpio_to_irq(_data->irq_gpio);

	if (i2c->irq) {
		ret = request_threaded_irq(i2c->irq, NULL,
				sm5713_ccic_irq_thread,
				(IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_ONESHOT),
				"sm5713-usbpd", _data);
		if (ret < 0) {
			dev_err(dev, "%s failed to request irq(%d)\n",
					__func__, i2c->irq);
			return ret;
		}

		ret = enable_irq_wake(i2c->irq);
		if (ret < 0)
			dev_err(dev, "%s failed to enable wakeup src\n",
					__func__);
	}

	if (_data->lpm_mode)
		sm5713_set_irq_enable(_data, 0, 0, 0, 0, 0);
	else
		sm5713_set_irq_enable(_data, ENABLED_INT_1, ENABLED_INT_2, ENABLED_INT_3, ENABLED_INT_4,
					ENABLED_INT_5_PASS);

	return ret;
}

static int of_sm5713_ccic_dt(struct device *dev,
			struct sm5713_usbpd_data *_data)
{
	struct device_node *np_usbpd = dev->of_node;
	int ret = 0;
	pr_info("%s\n",__func__);
	if (np_usbpd == NULL) {
		dev_err(dev, "%s np NULL\n", __func__);
		return -EINVAL;
	} else {
		_data->irq_gpio = of_get_named_gpio(np_usbpd,
							"usbpd,usbpd_int", 0);
		if (_data->irq_gpio < 0) {
			dev_err(dev, "error reading usbpd irq = %d\n",
						_data->irq_gpio);
			_data->irq_gpio = 0;
		}
		pr_info("%s irq_gpio = %d",__func__, _data->irq_gpio);
		if (of_find_property(np_usbpd, "vconn-en", NULL))
			_data->vconn_en = true;
		else
			_data->vconn_en = false;
	}

	return ret;
}

static int sm5713_usbpd_probe(struct i2c_client *i2c,
				const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(i2c->dev.parent);
	struct sm5713_usbpd_data *pdic_data;
	struct device *dev = &i2c->dev;
	int ret = 0;
	u8 rid = 0;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
	struct dual_role_phy_desc *desc;
	struct dual_role_phy_instance *dual_role;
#endif

	pr_info("%s start\n",__func__);
	test_i2c = i2c;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c functionality check error\n", __func__);
		ret = -EIO;
		goto err_return;
	}

	pdic_data = kzalloc(sizeof(struct sm5713_usbpd_data), GFP_KERNEL);
	if (!pdic_data) {
		dev_err(dev, "%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto err_return;
	}

	/* save platfom data for gpio control functions */
	pdic_data->dev = &i2c->dev;
	pdic_data->i2c = i2c;
	i2c_set_clientdata(i2c, pdic_data);

	ret = of_sm5713_ccic_dt(&i2c->dev, pdic_data);
	if (ret < 0)
		dev_err(dev, "%s: not found dt!\n", __func__);

	sm5713_usbpd_read_reg(i2c, SM5713_REG_FACTORY, &rid);

	pdic_data->rid = rid;
    pdic_data->lpm_mode = false;
	pdic_data->is_factory_mode = false;
#ifndef CONFIG_SEC_FACTORY    
//	if (factory_mode) {
		if (rid != REG_RID_523K) {
			dev_err(dev, "%s : In factory mode, but RID is not 523K\n", __func__);
		} else {
			dev_err(dev, "%s : In factory mode, but RID is 523K OK\n", __func__);
			pdic_data->is_factory_mode = true;
		}
//	}
#endif
	pdic_data->check_msg_pass = false;
	pdic_data->vconn_source = USBPD_VCONN_OFF;
	pdic_data->rid = REG_RID_MAX;
	pdic_data->is_host = 0;
	pdic_data->is_client = 0;
	pdic_data->data_role_dual = 0;
	pdic_data->power_role_dual = 0;
	pdic_data->is_attached = 0;
	pdic_data->is_water_detect = false;
	pdic_data->detach_valid = true;
	pdic_data->is_otg_vboost = false;
	pdic_data->is_otg_reboost = false;
    pdic_data->reset_done = 0;
	pdic_data->is_abnormal_state = false;

	ret = usbpd_init(dev, pdic_data);
	if (ret < 0) {
		dev_err(dev, "failed on usbpd_init\n");
		goto err_return;
	}

	usbpd_set_ops(dev, &sm5713_ops);

	mutex_init(&pdic_data->_mutex);
	mutex_init(&pdic_data->lpm_mutex);

	sm5713_usbpd_reg_init(pdic_data);

	pdic_data->pdic_queue =
	    alloc_workqueue(dev_name(dev), WQ_MEM_RECLAIM, 1);
	if (!pdic_data->pdic_queue) {
		dev_err(dev,
			"%s: Fail to Create Workqueue\n", __func__);
		goto err_return;
	}
	INIT_DELAYED_WORK(&pdic_data->plug_work, sm5713_set_src_sink_init);

#if defined(CONFIG_CCIC_NOTIFIER)
//	ccic_notifier_init();
	/* Create a work queue for the ccic irq thread */
	pdic_data->ccic_wq
		= create_singlethread_workqueue("ccic_irq_event");
	if (!pdic_data->ccic_wq) {
		pr_err("%s failed to create work queue for ccic notifier\n", __func__);
		goto err_return;
	}
	if (pdic_data->rid == REG_RID_UNDF)
		pdic_data->rid = REG_RID_MAX;
	dev_set_drvdata(ccic_device, pdic_data);
#endif

	ret = sm5713_usbpd_irq_init(pdic_data);
	if (ret) {
		dev_err(dev, "%s: failed to init irq(%d)\n", __func__, ret);
		goto fail_init_irq;
	}

//	sm5713_ccic_irq_thread(-1, pdic_data);

#if !defined(CONFIG_MUIC_NOTIFIER)
	muic_ccic_notifier_register(&pdic_data->type3_nb,
			       type3_handle_notification,
			       MUIC_NOTIFY_DEV_PDIC);
#endif

#if defined(CONFIG_DUAL_ROLE_USB_INTF)
		desc =
			devm_kzalloc(&i2c->dev,
					 sizeof(struct dual_role_phy_desc), GFP_KERNEL);
		if (!desc) {
			pr_err("unable to allocate dual role descriptor\n");
			goto fail_init_irq;
		}

		desc->name = "otg_default";
		desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
		desc->get_property = dual_role_get_local_prop;
		desc->set_property = dual_role_set_prop;
		desc->properties = fusb_drp_properties;
		desc->num_properties = ARRAY_SIZE(fusb_drp_properties);
		desc->property_is_writeable = dual_role_is_writeable;
		dual_role =
			devm_dual_role_instance_register(&i2c->dev, desc);
		dual_role->drv_data = pdic_data;
		pdic_data->dual_role = dual_role;
		pdic_data->desc = desc;
		init_completion(&pdic_data->reverse_completion);
		INIT_DELAYED_WORK(&pdic_data->role_swap_work, role_swap_check);
#endif

	pr_info("%s sm5713 usbpd driver uploaded!\n", __func__);

	return 0;

fail_init_irq:
	if (i2c->irq)
		free_irq(i2c->irq, pdic_data);
err_return:
	return ret;
}

#if defined CONFIG_PM
static int sm5713_usbpd_suspend(struct device *dev)
{
	struct usbpd_data *_data = dev_get_drvdata(dev);
	struct sm5713_usbpd_data *pdic_data = _data->phy_driver_data;

	if (device_may_wakeup(dev))
		enable_irq_wake(pdic_data->i2c->irq);

	disable_irq(pdic_data->i2c->irq);

	return 0;
}

static int sm5713_usbpd_resume(struct device *dev)
{
	struct usbpd_data *_data = dev_get_drvdata(dev);
	struct sm5713_usbpd_data *pdic_data = _data->phy_driver_data;

	if (device_may_wakeup(dev))
		disable_irq_wake(pdic_data->i2c->irq);

	enable_irq(pdic_data->i2c->irq);

	return 0;
}
#endif

static int sm5713_usbpd_remove(struct i2c_client *i2c)
{
	struct sm5713_usbpd_data *_data = i2c_get_clientdata(i2c);

	if (_data) {
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
		devm_dual_role_instance_unregister(_data->dev, _data->dual_role);
		devm_kfree(_data->dev, _data->desc);
#endif
		disable_irq_wake(_data->i2c->irq);
		free_irq(_data->i2c->irq, _data);
		mutex_destroy(&_data->_mutex);
		i2c_set_clientdata(_data->i2c, NULL);
		kfree(_data);
	}
	return 0;
}

static const struct i2c_device_id sm5713_usbpd_i2c_id[] = {
	{ USBPD_DEV_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, sm5713_usbpd_i2c_id);

static struct of_device_id sec_usbpd_i2c_dt_ids[] = {
	{ .compatible = "sm5713-usbpd,i2c" },
	{ }
};

static void sm5713_usbpd_shutdown(struct i2c_client *i2c)
{
	struct sm5713_usbpd_data *_data = i2c_get_clientdata(i2c);

	if (!_data->i2c)
		return;
}

static usbpd_phy_ops_type sm5713_ops = {
	.tx_msg			= sm5713_tx_msg,
	.rx_msg			= sm5713_rx_msg,
	.hard_reset		= sm5713_hard_reset,
	.set_power_role		= sm5713_set_power_role,
	.get_power_role		= sm5713_get_power_role,
	.set_data_role		= sm5713_set_data_role,
	.get_data_role		= sm5713_get_data_role,
	.set_vconn_source	= sm5713_set_vconn_source,
	.get_vconn_source	= sm5713_get_vconn_source,
	.set_check_msg_pass	= sm5713_set_check_msg_pass,
	.get_status		= sm5713_get_status,
	.poll_status		= sm5713_poll_status,
	.driver_reset  		= sm5713_driver_reset,
};

#if defined CONFIG_PM
const struct dev_pm_ops sm5713_usbpd_pm = {
	.suspend = sm5713_usbpd_suspend,
	.resume = sm5713_usbpd_resume,
};
#endif

static struct i2c_driver sm5713_usbpd_driver = {
	.driver		= {
		.name	= USBPD_DEV_NAME,
		.of_match_table	= sec_usbpd_i2c_dt_ids,
#if defined CONFIG_PM
		.pm	= &sm5713_usbpd_pm,
#endif /* CONFIG_PM */
	},
	.probe		= sm5713_usbpd_probe,
	.remove		= sm5713_usbpd_remove,
	.shutdown	= sm5713_usbpd_shutdown,
	.id_table	= sm5713_usbpd_i2c_id,
};

static int __init sm5713_usbpd_init(void)
{
	return i2c_add_driver(&sm5713_usbpd_driver);
}
late_initcall(sm5713_usbpd_init);

static void __exit sm5713_usbpd_exit(void)
{
	i2c_del_driver(&sm5713_usbpd_driver);
}
module_exit(sm5713_usbpd_exit);

MODULE_DESCRIPTION("SM5713 USB PD driver");
MODULE_LICENSE("GPL");
