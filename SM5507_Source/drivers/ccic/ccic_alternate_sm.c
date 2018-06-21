

#include <linux/ccic/sm5507.h>
#include <linux/ccic/ccic_alternate_sm.h>
#if defined(CONFIG_SWITCH)
#include <linux/switch.h>
#endif /* CONFIG_SWITCH */

#if defined(CONFIG_CCIC_ALTERNATE_MODE)
#if defined(CONFIG_SWITCH)
static struct switch_dev switch_dock = {
	.name = "ccic_dock",
};
#endif /* CONFIG_SWITCH */

/*
static char VDM_MSG_IRQ_State_Print[7][40] =
{
    {"bFLAG_Vdm_Reserve_b0"},
    {"bFLAG_Vdm_Discover_ID"},
    {"bFLAG_Vdm_Discover_SVIDs"},
    {"bFLAG_Vdm_Discover_MODEs"},
    {"bFLAG_Vdm_Enter_Mode"},
    {"bFLAG_Vdm_Exit_Mode"},
    {"bFLAG_Vdm_Attention"},
};
*/

int ccic_register_switch_device(int mode)
{
#ifdef CONFIG_SWITCH
	int ret = 0;
	if (mode) {
		ret = switch_dev_register(&switch_dock);
		if (ret < 0) {
			pr_err("%s: Failed to register dock switch(%d)\n",
				__func__, ret);
			return -ENODEV;
		}
	} else {
		switch_dev_unregister(&switch_dock);
	}
#endif /* CONFIG_SWITCH */
	return 0;
}

static void ccic_send_dock_intent(int type)
{
	pr_info("%s: CCIC dock type(%d)\n", __func__, type);
#ifdef CONFIG_SWITCH
	switch_set_state(&switch_dock, type);
#endif /* CONFIG_SWITCH */
}

void acc_detach_check(struct work_struct *wk)
{
	struct delayed_work *delay_work =
		container_of(wk, struct delayed_work, work);
	struct sm5507_data *sm5507_data =
		container_of(delay_work, struct sm5507_data, acc_detach_work);
	pr_info("%s: usbpd_data->pd_state : %d\n", __func__, sm5507_data->pd_state);
	if (sm5507_data->attach == 0) {
		ccic_send_dock_intent(CCIC_DOCK_DETACHED);
		sm5507_data->acc_type = CCIC_DOCK_DETACHED;
	}

}
static int process_check_accessory(void * data)
{
	struct sm5507_data *sm5507_data = data;

	// detect Gear VR
	if (sm5507_data->Vendor_ID == 0x04E8 && sm5507_data->Product_ID >= 0xA500 && sm5507_data->Product_ID <= 0xA505) {
		pr_info("%s : Samsung Gear VR connected.\n", __func__);
		if (sm5507_data->acc_type == CCIC_DOCK_DETACHED) {
			ccic_send_dock_intent(CCIC_DOCK_HMT);
			sm5507_data->acc_type = CCIC_DOCK_HMT;
		}
		return 1;
	}
	return 0;

}


static void process_discover_identity(void * data)
{
	struct sm5507_data *sm5507_data = data;
	struct i2c_client *i2c = sm5507_data->i2c;
	uint8_t REG_ADD = SM5507_REG_TX_PAYLOAD;
   	uint8_t ReadMSG[32] = {0,};
    u8 uVal;
    int ret = 0;

	// Message Type Definition
	U_DATA_MSG_ID_HEADER_Type     *DATA_MSG_ID = (U_DATA_MSG_ID_HEADER_Type *)&ReadMSG[8];
	U_PRODUCT_VDO_Type			  *DATA_MSG_PRODUCT = (U_PRODUCT_VDO_Type *)&ReadMSG[16];


    sm5507_i2c_write_byte(i2c, SM5507_REG_PDO_RD_REQ, 0x03); // Read VDM ID
    
     do {
         uVal = sm5507_i2c_read_byte(i2c, SM5507_REG_PL_SEL);
     } while( !(uVal & 0x80));


    ret = sm5507_multi_read_byte(i2c, REG_ADD, ReadMSG, 32);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s has i2c error.\n", __func__);
		return;
	}    

	sm5507_data->Vendor_ID = DATA_MSG_ID->BITS.USB_Vendor_ID;
	sm5507_data->Product_ID = DATA_MSG_PRODUCT->BITS.Product_ID;


    dev_info(&i2c->dev, "%s Vendor_ID : 0x%X, Product_ID : 0x%X\n", __func__, sm5507_data->Vendor_ID, sm5507_data->Product_ID);
    if (process_check_accessory(sm5507_data))
        dev_info(&i2c->dev, "%s : Samsung Accessory Connected.\n", __func__);

}

static void process_discover_svids(void * data)
{
	struct sm5507_data *sm5507_data = data;
	struct i2c_client *i2c = sm5507_data->i2c;
	uint8_t REG_ADD = SM5507_REG_TX_PAYLOAD;
   	uint8_t ReadMSG[32] = {0,};
    u8 uVal;
    int ret = 0;

    // Message Type Definition
    U_VDO1_Type                   *DATA_MSG_VDO1 = (U_VDO1_Type *)&ReadMSG[8];

    sm5507_i2c_write_byte(i2c, SM5507_REG_PDO_RD_REQ, 0x04); // Read VDM SVID
    
     do {
         uVal = sm5507_i2c_read_byte(i2c, SM5507_REG_PL_SEL);
     } while( !(uVal & 0x80));

    ret = sm5507_multi_read_byte(i2c, REG_ADD, ReadMSG, 32);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s has i2c error.\n", __func__);
		return;
	}

    sm5507_data->SVID_0 = DATA_MSG_VDO1->BITS.SVID_0;
    sm5507_data->SVID_1 = DATA_MSG_VDO1->BITS.SVID_1;

    dev_info(&i2c->dev, "%s SVID_0 : 0x%X, SVID_1 : 0x%X\n", __func__, sm5507_data->SVID_0, sm5507_data->SVID_1);

}

static void process_discover_modes(void * data)
{
	struct sm5507_data *sm5507_data = data;
	struct i2c_client *i2c = sm5507_data->i2c;
	uint8_t REG_ADD = SM5507_REG_TX_PAYLOAD;
   	uint8_t ReadMSG[32] = {0,};
    u8 uVal;
    int ret = 0;

    sm5507_i2c_write_byte(i2c, SM5507_REG_PDO_RD_REQ, 0x05); // Read VDM MODE
    
     do {
         uVal = sm5507_i2c_read_byte(i2c, SM5507_REG_PL_SEL);
     } while( !(uVal & 0x80));

	ret = sm5507_multi_read_byte(i2c, REG_ADD, ReadMSG, 32);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s has i2c error.\n", __func__);
		return;
	}

	dev_info(&i2c->dev, "%s\n", __func__);

}

static void process_enter_mode(void * data)
{
	struct sm5507_data *sm5507_data = data;
	struct i2c_client *i2c = sm5507_data->i2c;
	uint8_t REG_ADD = SM5507_REG_TX_PAYLOAD;
   	uint8_t ReadMSG[32] = {0,};
    u8 uVal;
    int ret = 0;

	// Message Type Definition
	U_DATA_MSG_VDM_HEADER_Type	  *DATA_MSG_VDM = (U_DATA_MSG_VDM_HEADER_Type *)&ReadMSG[4];

    sm5507_i2c_write_byte(i2c, SM5507_REG_PDO_RD_REQ, 0x08); // Read VDM Enter MODE
    
     do {
         uVal = sm5507_i2c_read_byte(i2c, SM5507_REG_PL_SEL);
     } while( !(uVal & 0x80));

	ret = sm5507_multi_read_byte(i2c, REG_ADD, ReadMSG, 32);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s has i2c error.\n", __func__);
		return;
	}

    if (DATA_MSG_VDM->BITS.VDM_command_type == 1)
        dev_info(&i2c->dev, "%s : EnterMode ACK.\n", __func__);
    else
        dev_info(&i2c->dev, "%s : EnterMode NAK.\n", __func__);

}

static void process_alternate_mode(void * data)
{
	struct sm5507_data *sm5507_data = data;
	struct i2c_client *i2c = sm5507_data->i2c;
	uint32_t mode = sm5507_data->alternate_state;

	if (mode) {
		dev_info(&i2c->dev, "%s, mode : 0x%x\n", __func__, mode);

		if (mode & VDM_DISCOVER_ID)
			process_discover_identity(sm5507_data);
		if (mode & VDM_DISCOVER_SVIDS)
			process_discover_svids(sm5507_data);
		if (mode & VDM_DISCOVER_MODES)
			process_discover_modes(sm5507_data);
		if (mode & VDM_ENTER_MODE)
			process_enter_mode(sm5507_data);
		sm5507_data->alternate_state = 0;
	}
}

void send_alternate_message(void * data, int cmd)
{
    /* To-Do : Not used */
}

void receive_alternate_message(void * data)
{
	struct sm5507_data *sm5507_data = data;
	struct i2c_client *i2c = sm5507_data->i2c;

	dev_info(&i2c->dev, "%s : %s\n",__func__, "FLAG_Vdm_Discover_ID");
    sm5507_data->alternate_state |= VDM_DISCOVER_ID;    
/*    u8 PDO_REQ;

    PDO_REQ = sm5507_i2c_read_byte(i2c, SM5507_REG_PDO_RD_REQ);

	if (PDO_REQ & 0x03) {
		dev_info(&i2c->dev, "%s : %s\n",__func__, "FLAG_Vdm_Discover_ID");
		sm5507_data->alternate_state |= VDM_DISCOVER_ID;
	}
	if (PDO_REQ & 0x04) {
		dev_info(&i2c->dev, "%s : %s\n",__func__, "FLAG_Vdm_Discover_SVIDs");
		sm5507_data->alternate_state |= VDM_DISCOVER_SVIDS;
	}
	if (PDO_REQ & 0x05) {
		dev_info(&i2c->dev, "%s : %s\n",__func__, "FLAG_Vdm_Discover_MODEs");
		sm5507_data->alternate_state |= VDM_DISCOVER_MODES;
	}
	if (PDO_REQ & 0x07) {
		dev_info(&i2c->dev, "%s : %s\n",__func__, "FLAG_Vdm_Enter_Mode");
		sm5507_data->alternate_state |= VDM_ENTER_MODE;
	}
	if (PDO_REQ & 0x08) {
		dev_info(&i2c->dev, "%s : %s\n",__func__, "FLAG_Vdm_Exit_Mode");
		sm5507_data->alternate_state |= VDM_EXIT_MODE;
	}    
*/
	process_alternate_mode(sm5507_data);
}

#endif


