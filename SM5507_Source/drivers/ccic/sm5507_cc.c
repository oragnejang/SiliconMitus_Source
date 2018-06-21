

#include <linux/ccic/sm5507.h>
#if defined(CONFIG_CCIC_NOTIFIER)
#include <linux/workqueue.h>
#endif
#if defined(CONFIG_CCIC_ALTERNATE_MODE)
#include <linux/ccic/ccic_alternate_sm.h>
#endif

extern struct pdic_notifier_struct pd_noti;

/* CC_STATUS : ATTACH_TYPE */
#define CCIC_ATTACH_TYPE_SOURCE    0x01
#define CCIC_ATTACH_TYPE_SINK      0x02
#define CCIC_ATTACH_TYPE_AUDIO     0x03

#define DUAL_ROLE_SET_MODE_WAIT_MS 1500


#if defined(CONFIG_CCIC_NOTIFIER)
static void ccic_event_notifier(struct work_struct *data)
{
	struct ccic_state_work *event_work =
		container_of(data, struct ccic_state_work, ccic_work);
	CC_NOTI_TYPEDEF ccic_noti;

	switch(event_work->dest){
		case CCIC_NOTIFY_DEV_USB :
			pr_info("usb:%s, dest=%s, id=%s, attach=%s, drp=%s\n", __func__,
				CCIC_NOTI_DEST_Print[event_work->dest],
				CCIC_NOTI_ID_Print[event_work->id],
				event_work->attach? "Attached": "Detached",
				CCIC_NOTI_USB_STATUS_Print[event_work->event]);
			break;
		default :
			pr_info("usb:%s, dest=%s, id=%s, attach=%d, event=%d\n", __func__,
				CCIC_NOTI_DEST_Print[event_work->dest],
				CCIC_NOTI_ID_Print[event_work->id],
				event_work->attach,
				event_work->event);
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
	ccic_notifier_notify((CC_NOTI_TYPEDEF*)&ccic_noti, NULL, 0);

	kfree(event_work);
}

void ccic_event_work(void *data, int dest, int id, int attach, int event)
{
	struct sm5507_data *sm5507_data = data;
	struct ccic_state_work * event_work;

	pr_info("usb: %s\n", __func__);
	event_work = kmalloc(sizeof(struct ccic_state_work), GFP_ATOMIC);
	INIT_WORK(&event_work->ccic_work, ccic_event_notifier);

	event_work->dest = dest;
	event_work->id = id;
	event_work->attach = attach;
	event_work->event = event;

#if defined(CONFIG_DUAL_ROLE_USB_INTF)
	if (id == CCIC_NOTIFY_ID_USB) {
		pr_info("usb: %s, dest=%d, event=%d, usbpd_data->data_role=%d, usbpd_data->try_state_change=%d\n",
			__func__, dest, event, sm5507_data->data_role, sm5507_data->try_state_change);

		sm5507_data->data_role = event;

		if (sm5507_data->dual_role != NULL)
			dual_role_instance_changed(sm5507_data->dual_role);

		if (sm5507_data->try_state_change &&
			(sm5507_data->data_role != USB_STATUS_NOTIFY_DETACH)) {
			// Role change try and new mode detected
			pr_info("usb: %s, reverse_completion\n", __func__);
			complete(&sm5507_data->reverse_completion);
		}
	}
#endif

	queue_work(sm5507_data->ccic_wq, &event_work->ccic_work);
}
#endif

#if defined(CONFIG_DUAL_ROLE_USB_INTF)

void sm5507_role_swap_check(struct work_struct *wk)
{
	struct delayed_work *delay_work =
		container_of(wk, struct delayed_work, work);
	struct sm5507_data *sm5507_data =
		container_of(delay_work, struct sm5507_data, role_swap_work);
	int mode;

	pr_info("%s: ccic_set_dual_role check again usbpd_data->pd_state=%d\n",
		__func__, sm5507_data->pd_state);

	sm5507_data->try_state_change = 0;

	if (sm5507_data->attach == 0) {
		pr_err("%s: ccic_set_dual_role reverse failed, set mode to DRP\n", __func__);
		disable_irq(sm5507_data->irq);
		/* exit from Disabled state and set mode to DRP */
		mode =  TYPE_C_ATTACH_DRP;
		sm5507_rprd_mode_change(sm5507_data, mode);
		enable_irq(sm5507_data->irq);
	}    

}

static int sm5507_set_dual_role(struct dual_role_phy_instance *dual_role,
				   enum dual_role_property prop,
				   const unsigned int *val)
{
	struct sm5507_data *sm5507_data = dual_role_get_drvdata(dual_role);
	struct i2c_client *i2c = sm5507_data->i2c;

	USB_STATUS attached_state;
	int mode;
	int timeout = 0;
	int ret = 0;    

    if (!sm5507_data) {
        pr_err("%s : sm5507_data is null \n", __func__);
        return -EINVAL;
    }

    // Get Current Role //
    attached_state = sm5507_data->data_role;
    pr_info("%s : request prop = %d , attached_state = %d\n", __func__, prop, attached_state);

    if (attached_state != USB_STATUS_NOTIFY_ATTACH_DFP
        && attached_state != USB_STATUS_NOTIFY_ATTACH_UFP) {
        pr_err("%s : current mode : %d - just return \n",__func__, attached_state);
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

    if ( attached_state == USB_STATUS_NOTIFY_ATTACH_DFP) {
        /* Current mode DFP and Source  */
        pr_info("%s: try reversing, from Source to Sink\n", __func__);
        /* turns off VBUS first */
        vbus_turn_on_ctrl(0);
#if defined(CONFIG_CCIC_NOTIFIER)
        /* muic */
        ccic_event_work(sm5507_data,
            CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 0/*attach*/, 0/*rprd*/);
#endif
        /* exit from Disabled state and set mode to UFP */
        mode =  TYPE_C_ATTACH_UFP;
        sm5507_data->try_state_change = TYPE_C_ATTACH_UFP;
        sm5507_rprd_mode_change(sm5507_data, mode);
    } else {
        // Current mode UFP and Sink  //
        pr_info("%s: try reversing, from Sink to Source\n", __func__);
        /* exit from Disabled state and set mode to UFP */
        mode =  TYPE_C_ATTACH_DFP;
        sm5507_data->try_state_change = TYPE_C_ATTACH_DFP;
        sm5507_rprd_mode_change(sm5507_data, mode);
    }
        
    reinit_completion(&sm5507_data->reverse_completion);
    timeout =
        wait_for_completion_timeout(&sm5507_data->reverse_completion,
                    msecs_to_jiffies
                    (DUAL_ROLE_SET_MODE_WAIT_MS));
        
    if (!timeout) {
        sm5507_data->try_state_change = 0;
        pr_err("%s: reverse failed, set mode to DRP\n", __func__);
        disable_irq(sm5507_data->irq);
        /* exit from Disabled state and set mode to DRP */
        mode =  TYPE_C_ATTACH_DRP;
        sm5507_rprd_mode_change(sm5507_data, mode);
        enable_irq(sm5507_data->irq);
        ret = -EIO;
    } else {
        pr_err("%s: reverse success, one more check\n", __func__);
        schedule_delayed_work(&sm5507_data->role_swap_work, msecs_to_jiffies(DUAL_ROLE_SET_MODE_WAIT_MS));
    }

    dev_info(&i2c->dev, "%s -> data role : %d\n", __func__, *val);
    return ret;

}


/* Decides whether userspace can change a specific property */
int sm5507_dual_role_is_writeable(struct dual_role_phy_instance *drp,
				  enum dual_role_property prop)
{
	if (prop == DUAL_ROLE_PROP_MODE)
		return 1;
	else
		return 0;
}

/* Callback for "cat /sys/class/dual_role_usb/otg_default/<property>" */
int sm5507_dual_role_get_local_prop(struct dual_role_phy_instance *dual_role,
				    enum dual_role_property prop, unsigned int *val)
{
	struct sm5507_data *sm5507_data = dual_role_get_drvdata(dual_role);

	USB_STATUS attached_state;
	int power_role;

	if (!sm5507_data) {
		pr_err("%s : usbpd_data is null : request prop = %d \n",__func__, prop);
		return -EINVAL;
	}
	attached_state = sm5507_data->data_role;
	power_role = sm5507_data->power_role;

	pr_info("%s : request prop = %d , attached_state = %d, power_role = %d\n",
		__func__, prop, attached_state, power_role);

	if (attached_state == USB_STATUS_NOTIFY_ATTACH_DFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_DFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = power_role;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_HOST;
		else
			return -EINVAL;
	} else if (attached_state == USB_STATUS_NOTIFY_ATTACH_UFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_UFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = power_role;
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
int sm5507_dual_role_set_prop(struct dual_role_phy_instance *dual_role,
			      enum dual_role_property prop, const unsigned int *val)
{
	pr_info("%s : request prop = %d , *val = %d \n",__func__, prop, *val);
	if (prop == DUAL_ROLE_PROP_MODE) {
		return sm5507_set_dual_role(dual_role, prop, val);
    } else {
		return -EINVAL;
    }


    return 0;
}

#endif


void process_cc_attach(void * data, u8 *plug_attach_done)
{
	struct sm5507_data *sm5507_data = data;
    struct i2c_client *i2c = sm5507_data->i2c;
	struct otg_notify *o_notify = get_otg_notify();    
//    uint8_t	R_DATA[4];
    u8 ATTACH_TYPE;

    if (sm5507_data->attach == CCIC_NOTIFY_ID_ATTACH)
    {
		*plug_attach_done = 1;    
		sm5507_data->plug_rprd_sel = 1;
		if (sm5507_data->is_dr_swap || sm5507_data->is_pr_swap) {
			dev_info(&i2c->dev, "%s - ignore all pd_state by %s\n",	__func__,(sm5507_data->is_dr_swap ? "dr_swap" : "pr_swap"));
			return;
		}

        ATTACH_TYPE = sm5507_i2c_read_byte(i2c, SM5507_REG_CC_STATUS);
		dev_info(&i2c->dev, "%s %d: pd_state:%02d, is_host = %d, is_client = %d\n",
						__func__, __LINE__, sm5507_data->pd_state, sm5507_data->is_host, sm5507_data->is_client);        
#if defined(CONFIG_CCIC_NOTIFIER)
        if ((ATTACH_TYPE & 0x07) == CCIC_ATTACH_TYPE_SINK)
        {
            if (sm5507_data->is_client == CLIENT_ON) {
				dev_info(&i2c->dev, "%s %d: pd_state:%02d, turn off client\n",
							__func__, __LINE__, sm5507_data->pd_state);
				ccic_event_work(sm5507_data,
					CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 0/*attach*/, 0/*rprd*/);
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
				sm5507_data->power_role = DUAL_ROLE_PROP_PR_NONE;
#endif
                ccic_event_work(sm5507_data,
                    CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 0/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
               sm5507_data->is_client = CLIENT_OFF;
               msleep(300);
            }
			if (sm5507_data->is_host == HOST_OFF) {
				/* muic */
				ccic_event_work(sm5507_data,
					CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 1/*attach*/, 1/*rprd*/);
				/* otg */
				sm5507_data->is_host = HOST_ON_BY_RD;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
				sm5507_data->power_role = DUAL_ROLE_PROP_PR_SRC;
#endif
				ccic_event_work(sm5507_data,
					CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 1/*attach*/, USB_STATUS_NOTIFY_ATTACH_DFP/*drp*/);
				msleep(100);
				/* add to turn on external 5V */
				if (!is_blocked(o_notify, NOTIFY_BLOCK_TYPE_HOST))
					vbus_turn_on_ctrl(1);
#if defined(CONFIG_CCIC_ALTERNATE_MODE)
				// only start alternate mode at DFP state
//				send_alternate_message(usbpd_data, VDM_DISCOVER_ID);
				if (sm5507_data->acc_type != CCIC_DOCK_DETACHED) {
					pr_info("%s: cancel_delayed_work_sync - pd_state : %d\n", __func__, sm5507_data->pd_state);
					cancel_delayed_work_sync(&sm5507_data->acc_detach_work);
				}
#endif
             }            
        } else if ((ATTACH_TYPE & 0x07) == CCIC_ATTACH_TYPE_SOURCE) {

			if (sm5507_data->is_host == HOST_ON_BY_RD) {
				dev_info(&i2c->dev, "%s %d: pd_state:%02d,  turn off host\n",
						__func__, __LINE__, sm5507_data->pd_state);
				ccic_event_work(sm5507_data,
					CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 0/*attach*/, 1/*rprd*/);
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
				sm5507_data->power_role = DUAL_ROLE_PROP_PR_NONE;
#endif
				/* add to turn off external 5V */
				vbus_turn_on_ctrl(0);
				ccic_event_work(sm5507_data,
					CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 0/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
				sm5507_data->is_host = HOST_OFF;
				msleep(300);
			}
			/* muic */
			ccic_event_work(sm5507_data,
				CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 1/*attach*/, 0/*rprd*/);
			if (sm5507_data->is_client == CLIENT_OFF && sm5507_data->is_host == HOST_OFF) {
				/* usb */
				sm5507_data->is_client = CLIENT_ON;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
				sm5507_data->power_role = DUAL_ROLE_PROP_PR_SNK;
#endif
				ccic_event_work(sm5507_data,
					CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 1/*attach*/, USB_STATUS_NOTIFY_ATTACH_UFP/*drp*/);
			}            
        }
#endif
    } else {
		sm5507_data->plug_rprd_sel = 0;
		sm5507_data->is_dr_swap = 0;
		sm5507_data->is_pr_swap = 0;

#if defined(CONFIG_CCIC_NOTIFIER)
		/* muic */
		ccic_event_work(sm5507_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 0/*attach*/, 0/*rprd*/);
		if(sm5507_data->is_host > HOST_OFF || sm5507_data->is_client > CLIENT_OFF) {
			if(sm5507_data->is_host > HOST_OFF)
				vbus_turn_on_ctrl(0);
			/* usb or otg */
			dev_info(&i2c->dev, "%s %d: pd_state:%02d, is_host = %d, is_client = %d\n",
					__func__, __LINE__, sm5507_data->pd_state, sm5507_data->is_host, sm5507_data->is_client);
			sm5507_data->is_host = HOST_OFF;
			sm5507_data->is_client = CLIENT_OFF;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			sm5507_data->power_role = DUAL_ROLE_PROP_PR_NONE;
#endif
			ccic_event_work(sm5507_data,
				CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 0/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
			msleep(300);
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			if (!sm5507_data->try_state_change)
				sm5507_rprd_mode_change(sm5507_data, TYPE_C_ATTACH_DRP);
#endif
#if defined(CONFIG_CCIC_ALTERNATE_MODE)
			if (sm5507_data->acc_type != CCIC_DOCK_DETACHED) {
				pr_info("%s: schedule_delayed_work - pd_state : %d\n", __func__, sm5507_data->pd_state);
				schedule_delayed_work(&sm5507_data->acc_detach_work, msecs_to_jiffies(GEAR_VR_DETACH_WAIT_MS));
			}
#endif
		}

#endif
    }
}

void process_cc_water_det(void * data)
{
    struct sm5507_data *sm5507_data = data;
    sm5507_int_clear(sm5507_data);	// interrupt clear

    ccic_event_work(sm5507_data,
        CCIC_NOTIFY_DEV_BATTERY, CCIC_NOTIFY_ID_WATER, 1/*attach*/, 0);
    sm5507_data->pd_state = 0;

    pr_info("%s\n",__func__);

}

void process_cc_get_int_status(void *data, u8 plug_attach_done, u8 pdic_attach)
{
	struct sm5507_data *sm5507_data = data;
	struct i2c_client *i2c = sm5507_data->i2c;
    u8 PD_REQ;

    PD_REQ = sm5507_i2c_read_byte(i2c, SM5507_REG_PD_REQ);

    pr_info("%s : PD Request = 0x%x\n",__func__, PD_REQ);   

    if (PD_REQ & PD_REQ_DR_SWAP)
    {
		sm5507_data->is_dr_swap++;
		dev_info(&i2c->dev, "is_dr_swap count : 0x%x\n", sm5507_data->is_dr_swap);
		if (sm5507_data->is_host == HOST_ON_BY_RD) {
			ccic_event_work(sm5507_data,
				CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 0/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
			msleep(300);
			ccic_event_work(sm5507_data,
				CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 1/*attach*/, USB_STATUS_NOTIFY_ATTACH_UFP/*drp*/);
			sm5507_data->is_host = HOST_OFF;
			sm5507_data->is_client = CLIENT_ON;
		} else if (sm5507_data->is_client == CLIENT_ON) {
			ccic_event_work(sm5507_data,
				CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 0/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
			msleep(300);
			ccic_event_work(sm5507_data,
				CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 1/*attach*/, USB_STATUS_NOTIFY_ATTACH_DFP/*drp*/);
			sm5507_data->is_host = HOST_ON_BY_RD;
			sm5507_data->is_client = CLIENT_OFF;
		}        
    }

#if defined(CONFIG_CCIC_ALTERNATE_MODE)
    // alternate_message
    if (plug_attach_done && pdic_attach)
    {
        receive_alternate_message(sm5507_data);
    }    
#endif    
}

void process_cc_rid(void *data)
{
	struct sm5507_data *sm5507_data = data;
	struct i2c_client *i2c = sm5507_data->i2c;
	static int prev_rid = RID_OPEN;
	u8 rid;

    pr_info("%s\n",__func__);
    rid = sm5507_i2c_read_byte(i2c, SM5507_REG_FACTORY);
    dev_info(&i2c->dev, "prev_rid:%x , RID:%x\n",prev_rid, rid);
    sm5507_data->cur_rid = rid;

    if(rid)
    {
        if(prev_rid != rid)
        {
#if defined(CONFIG_CCIC_NOTIFIER)
            /* rid */
			ccic_event_work(sm5507_data,
				CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_RID, rid/*rid*/, 0);

            if (rid == RID_000K)
            {
                /* otg */
                dev_info(&i2c->dev, "%s %d: RID_000K\n", __func__, __LINE__);
				if (sm5507_data->is_client) {
					/* usb or otg */
					ccic_event_work(sm5507_data,
						CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 0/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
				}
				sm5507_data->is_host = HOST_ON_BY_RID000K;
				sm5507_data->is_client = CLIENT_OFF;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
				sm5507_data->power_role = DUAL_ROLE_PROP_PR_SRC;
#endif
				/* add to turn on external 5V */
				vbus_turn_on_ctrl(1);
				ccic_event_work(sm5507_data,
					CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 1/*attach*/, USB_STATUS_NOTIFY_ATTACH_DFP/*drp*/);                
            } else if(rid == RID_OPEN || rid == RID_UNDEFINED || rid == RID_523K || rid == RID_619K) {
				if (prev_rid == RID_000K) {
					/* add to turn off external 5V */
					vbus_turn_on_ctrl(0);
				}
				sm5507_data->is_host = HOST_OFF;
				sm5507_data->is_client = CLIENT_OFF;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
				sm5507_data->power_role = DUAL_ROLE_PROP_PR_NONE;
#endif
				/* usb or otg */
				ccic_event_work(sm5507_data,
					CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 0/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);            
            }
#endif
        }
        prev_rid = rid;
    }
    return;
}
