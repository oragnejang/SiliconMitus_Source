
#include <linux/ccic/sm5507.h>
#include <linux/power_supply.h>
#if defined(CONFIG_BATTERY_NOTIFIER)
#include <linux/battery/battery_notifier.h>
#endif

extern struct sm5507_data *g_sm5507_data;
struct pdic_notifier_struct pd_noti;

#define UNIT_FOR_VOLTAGE 50
#define UNIT_FOR_CURRENT 10

static inline struct power_supply *get_power_supply_by_name(char *name)
{
	if (!name)
		return (struct power_supply *)NULL;
	else
		return power_supply_get_by_name(name);
}

static void sm5507_select_pdo(int num)
{
    u8 W_DATA;
        
	if (pd_noti.sink_status.selected_pdo_num == num)
		return;
	else if (num > pd_noti.sink_status.available_pdo_num)
		pd_noti.sink_status.selected_pdo_num = pd_noti.sink_status.available_pdo_num;
	else if (num < 1)
		pd_noti.sink_status.selected_pdo_num = 1;
	else
		pd_noti.sink_status.selected_pdo_num = num;
	pr_info(" %s : PDO(%d) is selected to change\n", __func__, pd_noti.sink_status.selected_pdo_num);

    W_DATA = pd_noti.sink_status.selected_pdo_num;

    sm5507_i2c_write_byte(pd_noti.pusbpd->i2c, SM5507_REG_PDO_CAP4, W_DATA);

}

void select_pdo(int num)
{
	if (g_sm5507_data)
		sm5507_select_pdo(num);
}

void vbus_turn_on_ctrl(bool enable)
{
	struct power_supply *psy_otg;
	union power_supply_propval val;
	int on = !!enable;
	int ret = 0;

	pr_info("%s %d, enable=%d\n", __func__, __LINE__, enable);
	psy_otg = get_power_supply_by_name("otg");
	if (psy_otg) {
		val.intval = enable;
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

static int sm5507_src_capacity_information(const struct i2c_client *i2c, PDIC_SINK_STATUS *pd_sink_status)
{
	uint32_t PDO_cnt;
	uint32_t PDO_sel;
    uint32_t RX_SRC_CAP[7];
    uint16_t rcv_header = 0;
	int available_pdo_num = 0, i;
    u8 uVal;
	SRC_FIXED_SUPPLY_Typedef *MSG_FIXED_SUPPLY;
	SRC_VAR_SUPPLY_Typedef  *MSG_VAR_SUPPLY;
	SRC_BAT_SUPPLY_Typedef  *MSG_BAT_SUPPLY;
    
	MSG_HEADER_Type *MSG_HDR;

    sm5507_i2c_write_byte(i2c, SM5507_REG_PDO_RD_REQ, 0x01); // Read Source Cap

    do {
        uVal = sm5507_i2c_read_byte(i2c, SM5507_REG_PL_SEL);
    } while( !(uVal & 0x80));    

    for (i=0; i<2; i++) {
        uVal = sm5507_i2c_read_byte(i2c, SM5507_REG_TX_HEADER+i);
        
        rcv_header = rcv_header | (uVal << ((i%4)*8));
    } 

    MSG_HDR = (MSG_HEADER_Type *)&rcv_header;

	dev_info(&i2c->dev, "=======================================\n\r");
	dev_info(&i2c->dev, "    MSG Header\n\r");

	dev_info(&i2c->dev, "    Rsvd_msg_header         : %d\n\r",MSG_HDR->Rsvd_msg_header );
	dev_info(&i2c->dev, "    Number_of_obj           : %d\n\r",MSG_HDR->Number_of_obj );
	dev_info(&i2c->dev, "    Message_ID              : %d\n\r",MSG_HDR->Message_ID );
	dev_info(&i2c->dev, "    Port_Power_Role         : %d\n\r",MSG_HDR->Port_Power_Role );
	dev_info(&i2c->dev, "    Specification_Revision  : %d\n\r",MSG_HDR->Specification_Revision );
	dev_info(&i2c->dev, "    Port_Data_Role          : %d\n\r",MSG_HDR->Port_Data_Role );
	dev_info(&i2c->dev, "    Rsvd2_msg_header        : %d\n\r",MSG_HDR->Rsvd2_msg_header );
	dev_info(&i2c->dev, "    Message_Type            : %d\n\r",MSG_HDR->Message_Type );

    for(i = 0; i < MSG_HDR->Number_of_obj*4; i++)
    {
        uVal = sm5507_i2c_read_byte(i2c, SM5507_REG_TX_PAYLOAD+i);
        
        RX_SRC_CAP[i/4] = RX_SRC_CAP[i/4] | (uVal << ((i%4)*8));
    }


	for(PDO_cnt = 0;PDO_cnt < MSG_HDR->Number_of_obj;PDO_cnt++)
	{
	    PDO_sel = (RX_SRC_CAP[PDO_cnt + 1] >> 30) & 0x3;

		dev_info(&i2c->dev, "    =================\n\r");
		dev_info(&i2c->dev, "    PDO_Num : %d\n\r", (PDO_cnt));        

        if(PDO_sel == 0) // *MSG_FIXED_SUPPLY
        {
            MSG_FIXED_SUPPLY = (SRC_FIXED_SUPPLY_Typedef *)&RX_SRC_CAP[PDO_cnt];

			pd_sink_status->power_list[PDO_cnt+1].max_voltage = MSG_FIXED_SUPPLY->Voltage_Unit * UNIT_FOR_VOLTAGE;
			pd_sink_status->power_list[PDO_cnt+1].max_current = MSG_FIXED_SUPPLY->Maximum_Current * UNIT_FOR_CURRENT;

			dev_info(&i2c->dev, "    PDO_Parameter(FIXED_SUPPLY) : %d\n\r",MSG_FIXED_SUPPLY->PDO_Parameter );
			dev_info(&i2c->dev, "    Dual_Role_Power         : %d\n\r",MSG_FIXED_SUPPLY->Dual_Role_Power );
			dev_info(&i2c->dev, "    USB_Suspend_Support     : %d\n\r",MSG_FIXED_SUPPLY->USB_Suspend_Support );
			dev_info(&i2c->dev, "    Externally_POW          : %d\n\r",MSG_FIXED_SUPPLY->Externally_POW );
			dev_info(&i2c->dev, "    USB_Comm_Capable        : %d\n\r",MSG_FIXED_SUPPLY->USB_Comm_Capable );
			dev_info(&i2c->dev, "    Data_Role_Swap          : %d\n\r",MSG_FIXED_SUPPLY->Data_Role_Swap );
			dev_info(&i2c->dev, "    Reserved                : %d\n\r",MSG_FIXED_SUPPLY->Reserved );
			dev_info(&i2c->dev, "    Peak_Current            : %d\n\r",MSG_FIXED_SUPPLY->Peak_Current );
			dev_info(&i2c->dev, "    Voltage_Unit            : %d\n\r",MSG_FIXED_SUPPLY->Voltage_Unit );
			dev_info(&i2c->dev, "    Maximum_Current         : %d\n\r",MSG_FIXED_SUPPLY->Maximum_Current );            
        } else if (PDO_sel == 2) { // *MSG_VAR_SUPPLY
			MSG_VAR_SUPPLY = (SRC_VAR_SUPPLY_Typedef *)&RX_SRC_CAP[PDO_cnt];

			dev_info(&i2c->dev, "    PDO_Parameter(VAR_SUPPLY) : %d\n\r",MSG_VAR_SUPPLY->PDO_Parameter );
			dev_info(&i2c->dev, "    Maximum_Voltage          : %d\n\r",MSG_VAR_SUPPLY->Maximum_Voltage );
			dev_info(&i2c->dev, "    Minimum_Voltage          : %d\n\r",MSG_VAR_SUPPLY->Minimum_Voltage );
			dev_info(&i2c->dev, "    Maximum_Current          : %d\n\r",MSG_VAR_SUPPLY->Maximum_Current );        
        } else if (PDO_sel == 1) { // *MSG_BAT_SUPPLY
			MSG_BAT_SUPPLY = (SRC_BAT_SUPPLY_Typedef *)&RX_SRC_CAP[PDO_cnt];

			dev_info(&i2c->dev, "    PDO_Parameter(BAT_SUPPLY)  : %d\n\r",MSG_BAT_SUPPLY->PDO_Parameter );
			dev_info(&i2c->dev, "    Maximum_Voltage            : %d\n\r",MSG_BAT_SUPPLY->Maximum_Voltage );
			dev_info(&i2c->dev, "    Minimum_Voltage            : %d\n\r",MSG_BAT_SUPPLY->Minimum_Voltage );
			dev_info(&i2c->dev, "    Maximum_Allow_Power        : %d\n\r",MSG_BAT_SUPPLY->Maximum_Allow_Power );        
        }
    }

	/* the number of available pdo list */
	pd_sink_status->available_pdo_num = available_pdo_num;    
	return available_pdo_num;

}

void process_pd(void *data, u8 pdic_attach)
{
	struct sm5507_data *sm5507_data = data;
    struct i2c_client *i2c = sm5507_data->i2c;
	REQUEST_FIXED_SUPPLY_STRUCT_Typedef *request_power_number;    
	CC_NOTI_ATTACH_TYPEDEF pd_notifier;    
    u8 PD_REQ, uVal, PWR_ROLE;
    uint8_t is_src;
    int i;

    PD_REQ = sm5507_i2c_read_byte(i2c, SM5507_REG_PD_REQ);

    pr_info("%s : PD Request = 0x%x\n",__func__, PD_REQ);   

    is_src = sm5507_data->is_host;
    if (PD_REQ & PD_REQ_PR_SWAP)
    {
        sm5507_data->is_pr_swap++;
		dev_info(&i2c->dev, "PR_Swap requested to %s\n", is_src ? "SOURCE" : "SINK");
		vbus_turn_on_ctrl(is_src);
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
		sm5507_data->power_role = is_src ? DUAL_ROLE_PROP_PR_SRC : DUAL_ROLE_PROP_PR_SNK; 
#endif        
    }

    uVal = sm5507_i2c_read_byte(i2c, SM5507_REG_RX_HEADER);
    if(uVal & 0x01)
        PWR_ROLE = 0x00; // Source
    else
        PWR_ROLE = 0x01; // Sink
    
    if (pdic_attach && PWR_ROLE) // PD CONTRACT & Power ROLE is Source
    {
        uint32_t ReadMSG;
        int available_pdo_num;
        available_pdo_num = sm5507_src_capacity_information(i2c, &pd_noti.sink_status);

        sm5507_i2c_write_byte(i2c, SM5507_REG_PDO_RD_REQ, 0x0B); // TX Request RDO

        do {
            uVal = sm5507_i2c_read_byte(i2c, SM5507_REG_PL_SEL);
        } while( !(uVal & 0x80));

        for(i = 0; i < 4; i++)
        {
            uVal = (sm5507_i2c_read_byte(i2c, SM5507_REG_TX_PAYLOAD+i) & 0xff);
            
            ReadMSG = ReadMSG | (uVal << ((i%4)*8));
        }
        request_power_number = (REQUEST_FIXED_SUPPLY_STRUCT_Typedef *)&ReadMSG;

		pr_info(" %s : Object_posision(%d), available_pdo_num(%d), selected_pdo_num(%d) \n", __func__,
			request_power_number->Object_Position, available_pdo_num, pd_noti.sink_status.selected_pdo_num);
		pd_noti.sink_status.current_pdo_num = request_power_number->Object_Position;

		if(available_pdo_num > 0)
		{
			if(request_power_number->Object_Position != pd_noti.sink_status.selected_pdo_num)
			{
				if (pd_noti.sink_status.selected_pdo_num == 0)
				{
					pr_info(" %s : PDO is not selected yet by default\n", __func__);
					pd_noti.sink_status.selected_pdo_num = pd_noti.sink_status.current_pdo_num;
				}
			} else {
				pr_info(" %s : PDO(%d) is selected, but same with previous list, so skip\n",
						__func__, pd_noti.sink_status.selected_pdo_num);
			}
			pdic_attach = 1;
		} else {
			pr_info(" %s : PDO is not selected\n", __func__);
		}        
    }

	/* notify to battery */
#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
	if (sm5507_data->attach) {
		if (pdic_attach) {
			pd_noti.event = PDIC_NOTIFY_EVENT_PD_SINK;
			pd_notifier.src = CCIC_NOTIFY_DEV_CCIC;
			pd_notifier.dest = CCIC_NOTIFY_DEV_BATTERY;
			pd_notifier.id = CCIC_NOTIFY_ID_POWER_STATUS;
			pd_notifier.attach = pdic_attach;
			ccic_notifier_notify((CC_NOTI_TYPEDEF*)&pd_notifier, &pd_noti, pdic_attach);
		}
		else
			pd_noti.event = PDIC_NOTIFY_EVENT_CCIC_ATTACH;
	} else {
		pd_noti.sink_status.selected_pdo_num = 0;
		pd_noti.event = PDIC_NOTIFY_EVENT_DETACH;
	}
#else
	if(sm5507_data->attach)
	{
		/* PD notify */
		if(pdic_attach)
			pd_noti.event = PDIC_NOTIFY_EVENT_PD_SINK;
		else
			pd_noti.event = PDIC_NOTIFY_EVENT_CCIC_ATTACH;
	}
	else
	{ 
		pd_noti.sink_status.selected_pdo_num = 0;
		pd_noti.event = PDIC_NOTIFY_EVENT_DETACH;
	}
	pdic_notifier_call(pd_noti);
#endif
}

