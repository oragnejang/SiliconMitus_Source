

#include <linux/ccic/sm5507.h>
#if defined(CONFIG_BATTERY_NOTIFIER)
#include <linux/battery/battery_notifier.h>
#endif



struct sm5507_data *g_sm5507_data = NULL;

extern struct device *ccic_device;
extern struct pdic_notifier_struct pd_noti;

#if defined(CONFIG_DUAL_ROLE_USB_INTF)
static enum dual_role_property sm5507_dual_role_properties[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
};
#endif









#if defined(CONFIG_DUAL_ROLE_USB_INTF)
extern int sm5507_dual_role_get_local_prop(struct dual_role_phy_instance *dual_role,
				    enum dual_role_property prop, unsigned int *val);
extern int sm5507_dual_role_set_prop(struct dual_role_phy_instance *dual_role,
			      enum dual_role_property prop, const unsigned int *val);
extern int sm5507_dual_role_is_writeable(struct dual_role_phy_instance *drp,
				  enum dual_role_property prop);
extern void sm5507_role_swap_check(struct work_struct *wk);
#endif

#if defined(CONFIG_CCIC_ALTERNATE_MODE)
extern void acc_detach_check(struct work_struct *wk);
extern int ccic_register_switch_device(int mode);
#endif

/* support IRQ process */
extern void process_cc_attach(void * data, u8 *plug_attach_done);
extern void process_cc_water_det(void * data);
extern void process_cc_get_int_status(void *data, u8 plug_attach_done, u8 pdic_attach);
extern void process_cc_rid(void *data);
extern void process_pd(void *data, u8 pdic_attach);

// function definition
int sm5507_i2c_read_byte(const struct i2c_client *client, u8 command);
int sm5507_i2c_write_byte(const struct i2c_client *client, u8 command, u8 value);
int sm5507_multi_read_byte(const struct i2c_client *i2c, u16 reg, u8 *val, u16 size);
void sm5507_rprd_mode_change(struct sm5507_data *data, u8 mode);


extern int sm5507_flash(struct sm5507_data *usbpd_data, unsigned int input);
extern int sm5507_flash_fw(struct sm5507_data *data, unsigned int input);
//extern void sm5507_get_chip_swversion(struct sm5507_data *data, struct sm5507_version *version);


/**
 *  SM5507 Register control functions
 *  
 */
int sm5507_i2c_read_byte(const struct i2c_client *client, u8 command)
{
    int ret;
    int retry = 0;
    
    ret = i2c_smbus_read_byte_data(client, command);
    
    while(ret < 0){
        pr_err("%s:i2c err on reading reg(0x%x), retrying ...\n",
                    __func__, command);
        if(retry > 10)
        {
            pr_err("%s: retry count > 10 : failed !!\n", __func__);
            break;
        }
        msleep(100);
        ret = i2c_smbus_read_byte_data(client, command);
        retry ++;
    }

    return ret;
}

int sm5507_i2c_write_byte(const struct i2c_client *client, u8 command, u8 value)
{
    int ret;
    int retry = 0;
    int written = 0;
    
    ret = i2c_smbus_write_byte_data(client, command, value);
    
    while(ret < 0) {
        written = i2c_smbus_read_byte_data(client, command);
        if(written < 0) pr_err("%s:i2c err on reading reg(0x%x)\n",
                    __func__, command);
        msleep(100);
        ret = i2c_smbus_write_byte_data(client, command, value);
        retry ++;
    }
    return ret;
}

int sm5507_multi_read_byte(const struct i2c_client *i2c, u16 reg, u8 *val, u16 size)
{
	int ret; u8 wbuf[2];
	struct i2c_msg msg[2];
	struct sm5507_data *sm5507_data = i2c_get_clientdata(i2c);

	mutex_lock(&sm5507_data->i2c_mutex);
	msg[0].addr = i2c->addr;
	msg[0].flags = i2c->flags;
	msg[0].len = 2;
	msg[0].buf = wbuf;
	msg[1].addr = i2c->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size;
	msg[1].buf = val;

	wbuf[0] = (reg & 0xFF00) >> 8;
	wbuf[1] = (reg & 0xFF);

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		dev_err(&i2c->dev, "i2c read16 fail reg:0x%x error %d\n",
			reg, ret);
	mutex_unlock(&sm5507_data->i2c_mutex);

	return ret;
}


void sm5507_int_clear(struct sm5507_data *sm5507_data)
{
    struct i2c_client *i2c = sm5507_data->i2c;
    int intr1, intr2, intr3, intr4;

	/* read and clear interrupt status bits */
    intr1 = sm5507_i2c_read_byte(i2c, SM5507_REG_INT1);
    intr2 = sm5507_i2c_read_byte(i2c, SM5507_REG_INT2);    
    intr3 = sm5507_i2c_read_byte(i2c, SM5507_REG_INT3);
    intr4 = sm5507_i2c_read_byte(i2c, SM5507_REG_INT4);

    printk("%s\n",__func__);
}

static void sm5507_reset(struct sm5507_data *sm5507_data)
{
    struct i2c_client *i2c = sm5507_data->i2c;
    u8 W_DATA;

    W_DATA = 0x80; //soft Reset

    sm5507_i2c_write_byte(i2c, SM5507_REG_SYS_CTRL, W_DATA);
    printk("%s\n",__func__);

}

/**
 *  I2C device driver for SM5507 control functions  
 * 
 */
void sm5507_control_option_command(struct sm5507_data *data, int cmd)
{
    struct i2c_client *i2c = data->i2c;
    uint8_t REG_ADD;
    int i;
    u8 W_DATA;
    u8 R_DATA;

	printk("usb: %s cmd=0x%x\n", __func__, cmd);

    /* for Wake up*/
    for(i=0; i<5; i++){
        REG_ADD = 0x9;
        R_DATA = sm5507_i2c_read_byte(i2c, REG_ADD);   //dummy read
    }
    udelay(10);

    switch(cmd) {
        case 1: // Vconn control option command ON
            REG_ADD = 0x25;
            W_DATA = 0x80;        
            break;
        case 2: // Vconn control option command OFF
            REG_ADD = 0x25;        
            W_DATA = 0x00;        
            break;
        case 3: // Water Detect option command ON
            REG_ADD = 0x1D;        
            W_DATA = 0x54;        
            break;
        case 4: // Water Detect option command OFF
            REG_ADD = 0x1D;        
            W_DATA = 0x50;
            break;
    }

    sm5507_i2c_write_byte(i2c, REG_ADD, W_DATA);

}

void sm5507_manual_LPM(struct sm5507_data *data, int cmd)
{
	struct i2c_client *i2c = data->i2c;
	uint16_t REG_ADD;
	u8 W_DATA;
	u8 R_DATA = 0x00;
	int i;
	printk("usb: %s cmd=0x%x\n", __func__, cmd);


	/* for Wake up*/
	for(i=0; i<5; i++){
		REG_ADD = 0x9;
		R_DATA = sm5507_i2c_read_byte(i2c, REG_ADD);   //dummy read
	}
	udelay(10);

	W_DATA = cmd;
	REG_ADD = 0xD8;
	sm5507_i2c_write_byte(i2c, REG_ADD, W_DATA);

}

void sm5507_manual_JIGON(struct sm5507_data *data, int mode)
{
	struct i2c_client *i2c = data->i2c;
	uint16_t REG_ADD;
    int i;
	u8 W_DATA;
	u8 R_DATA = 0x00;


	printk("usb: %s mode=%s\n", __func__, mode? "High":"Low");

    if(mode)
        W_DATA = 0x0C; // JIGON High
    else
        W_DATA = 0x08; // JIGON Low
    /* for Wake up*/
    for(i=0; i<5; i++){
        REG_ADD = 0x9;
        R_DATA = sm5507_i2c_read_byte(i2c, REG_ADD);   //dummy read
    }
    udelay(10);

    REG_ADD = 0x1B;

    sm5507_i2c_write_byte(i2c, REG_ADD, W_DATA);

}

static void sm5507_new_toggling_control(struct sm5507_data *data, u8 mode)
{
	struct i2c_client *i2c = data->i2c;
    u8 W_DATA;

	pr_info("%s, mode=0x%x\n",__func__, mode);

    W_DATA = mode; // 0x20 : detach, 0x02 : SNK, 0x01 : SRC

    // Enter Manual Mode(MCU) 
    sm5507_i2c_write_byte(i2c, 0xF8, 0x27);
    sm5507_i2c_write_byte(i2c, 0xF8, 0x6D);

    sm5507_i2c_write_byte(i2c, 0xFE, 0x70); // Select Control Register Bank

    sm5507_i2c_write_byte(i2c, 0x10, W_DATA); // Write Control Data

    sm5507_i2c_write_byte(i2c, 0xFE, 0x00); // Release Control
    sm5507_i2c_write_byte(i2c, 0xF8, 0x00); // Exit Manual Mode
    
}

static void sm5507_toggling_control(struct sm5507_data *data, u8 mode)
{
	pr_info("%s, mode=0x%x\n",__func__, mode);
}


#if defined(CONFIG_DUAL_ROLE_USB_INTF)
void sm5507_rprd_mode_change(struct sm5507_data *data, u8 mode)
{
	pr_info("%s, mode=0x%x\n",__func__, mode);

	switch(mode)
	{
		case TYPE_C_ATTACH_DFP: // SRC
			sm5507_new_toggling_control(data, 0x20);
			msleep(1000);
			sm5507_new_toggling_control(data, 0x01);
		break;
		case TYPE_C_ATTACH_UFP: // SNK
			sm5507_new_toggling_control(data, 0x20);
			msleep(1000);
			sm5507_new_toggling_control(data, 0x02);
		break;
		case TYPE_C_ATTACH_DRP: // DRP
			sm5507_toggling_control(data, TYPE_C_ATTACH_DRP);
		break;	
	};
}
#endif


static irqreturn_t sm5507_irq_thread(int irq, void *data)
{
	struct sm5507_data *sm5507_data = data;
    struct i2c_client *i2c = sm5507_data->i2c;
	u8 plug_attach_done, status1, status2, status3, status4;
    u8 pdic_attach = 0;
    int intr1, intr2, intr3, intr4;

    dev_info(sm5507_data->dev, "%s - start\n", __func__);

	wake_lock_timeout(&sm5507_data->wlock, HZ);

	/* read and clear interrupt status bits */
    intr1 = sm5507_i2c_read_byte(i2c, SM5507_REG_INT1);
    intr2 = sm5507_i2c_read_byte(i2c, SM5507_REG_INT2);    
    intr3 = sm5507_i2c_read_byte(i2c, SM5507_REG_INT3);
    intr4 = sm5507_i2c_read_byte(i2c, SM5507_REG_INT4);

	pr_info("%s:%s intr[1:0x%x, 2:0x%x, 3:0x%x, 4:0x%x]\n", SM5507_CCIC_DEV_NAME, __func__,
			intr1, intr2, intr3, intr4);

    status1 = sm5507_i2c_read_byte(i2c, 0x09);
    status2 = sm5507_i2c_read_byte(i2c, 0x0A);    
    status3 = sm5507_i2c_read_byte(i2c, 0x0B);
    status4 = sm5507_i2c_read_byte(i2c, 0x0C);

    if (intr4 & INT_WATER_MASK || status3 & INT_WATER_MASK)
    {
		dev_info(&i2c->dev, "== WATER DETECT ==\n");
        sm5507_data->water_det = 1;    
    } else {
        sm5507_data->water_det = 0;
    }

    if (intr1 & INT_ATTACH_MASK || status1 & INT_ATTACH_MASK)
    {
        sm5507_data->attach= 1;
    } else if (intr1 & INT_DETACH_MASK || status1 & INT_DETACH_MASK) {
        sm5507_data->attach= 0;    
    }

	if(sm5507_data->water_det){
		process_cc_water_det(sm5507_data);
		goto water;
	}

	process_cc_attach(sm5507_data, &plug_attach_done);	

    if (intr2 & INT_PD_CONTRACT_MASK || status2 & INT_PD_CONTRACT_MASK)
    {
        pdic_attach = 1;
    } else {
        if (intr2 & INT_NON_PD_DEV_MASK || status2 & INT_NON_PD_DEV_MASK) {
            pdic_attach = 0;    
        }        
    }

    // when INT2 == 0x04(PD_INT)
	process_cc_get_int_status(sm5507_data, plug_attach_done, pdic_attach);


	process_pd(sm5507_data, pdic_attach);

    if (intr4 & INT_RID_INT_MASK || status4 & INT_RID_INT_MASK)
    {
        process_cc_rid(sm5507_data);
    }

water:
    dev_info(sm5507_data->dev, "%s - end\n", __func__);

	return IRQ_HANDLED;
}

static inline void _check_fw_version_update(struct sm5507_data *sm5507_data)
{
    struct sm5507_version chip_swver, fw_swver;
    u8 hw_ver = 0x1;
    sm5507_flash(sm5507_data, FLASH_WRITE);
    sm5507_flash_fw(sm5507_data, FLASH_WRITE);

  	sm5507_get_chip_swversion(sm5507_data, &chip_swver);
	pr_err("%s CHIP SWversion %2x, %2x - before\n", __func__,
	       chip_swver.main , chip_swver.boot);
    sm5507_get_fw_version(&fw_swver, chip_swver.boot);
    pr_err("%s SRC SWversion:%2x, %2x\n",__func__,
		fw_swver.main, fw_swver.boot);

    if (chip_swver.main < fw_swver.main)
    {
        sm5507_flash_fw(sm5507_data, FLASH_WRITE);
    }

	store_ccic_version(&hw_ver, &chip_swver.main, &chip_swver.boot);

    sm5507_data->firm_ver[2] = chip_swver.main;
    sm5507_data->firm_ver[3] = chip_swver.boot;    
}

#if defined(CONFIG_OF)
static int of_sm5507_ccic_dt(struct device *dev,
			       struct sm5507_data *sm5507_data)
{
	struct device_node *np = dev->of_node;
	int ret;

	sm5507_data->irq_gpio = of_get_named_gpio(np, "ccic,usbpd_int", 0);
	sm5507_data->redrv1_gpio = of_get_named_gpio(np, "ccic,redrv1", 0);
	sm5507_data->sda_gpio = of_get_named_gpio(np, "ccic,sm5507_sda", 0);
	sm5507_data->scl_gpio = of_get_named_gpio(np, "ccic,sm5507_scl", 0);

	np = of_find_all_nodes(NULL);
	ret = of_property_read_u32(np, "model_info-hw_rev", &sm5507_data->hw_rev);
	if (ret) {
		pr_info("%s: model_info-hw_rev is Empty\n", __func__);
		sm5507_data->hw_rev = 0;
	}

	dev_info(dev, "hw_rev:%03d irq=%d redrv1=%d sda=%d, scl=%d\n",
		sm5507_data->hw_rev,
		sm5507_data->irq_gpio, sm5507_data->redrv1_gpio,
		sm5507_data->sda_gpio, sm5507_data->scl_gpio);

	return 0;
}
#endif /* CONFIG_OF */

static int sm5507_probe(struct i2c_client *i2c,
			       const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(i2c->dev.parent);
	struct sm5507_data *sm5507_data;
	int ret = 0;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
	struct dual_role_phy_desc *desc;
	struct dual_role_phy_instance *dual_role;
#endif

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&i2c->dev, "i2c functionality check error\n");
		return -EIO;
	}
	sm5507_data = devm_kzalloc(&i2c->dev, sizeof(struct sm5507_data), GFP_KERNEL);
	if (!sm5507_data) {
		dev_err(&i2c->dev, "fail to allocate driver data\n");
		return -ENOMEM;
	}

#if defined(CONFIG_OF)
	if (i2c->dev.of_node)
		of_sm5507_ccic_dt(&i2c->dev, sm5507_data);
	else
		dev_err(&i2c->dev, "not found ccic dt!\n");
#endif

	ret = gpio_request(sm5507_data->irq_gpio, "sm5507_irq");
	if (ret) {
        dev_err(&i2c->dev, "fail to request gpio(irq-gpio) ret=%d\n", ret);
		goto err_free_irq_gpio;
    }
	if (gpio_is_valid(sm5507_data->redrv1_gpio)) {
		ret = gpio_request(sm5507_data->redrv1_gpio, "sm5507_redrv1");
		if (ret) {
			dev_err(&i2c->dev, "fail to request gpio(redrv1-gpio) ret=%d\n", ret);
            goto err_free_redriver_gpio;
        }
	}
    gpio_direction_input(sm5507_data->irq_gpio);
    sm5507_data->irq = gpio_to_irq(sm5507_data->irq_gpio);
    dev_info(&i2c->dev, "%s:IRQ NUM %d\n", __func__, sm5507_data->irq);

    sm5507_data->dev = &i2c->dev;
    sm5507_data->i2c = i2c;
    i2c_set_clientdata(i2c, sm5507_data);
    dev_set_drvdata(ccic_device, sm5507_data);
    device_init_wakeup(sm5507_data->dev, 1);
    pd_noti.pusbpd = sm5507_data;
	mutex_init(&sm5507_data->i2c_mutex);

	/* Init */
	sm5507_data->p_prev_rid = -1;
	sm5507_data->prev_rid = -1;
	sm5507_data->cur_rid = -1;
	sm5507_data->is_dr_swap = 0;
	sm5507_data->is_pr_swap = 0;
	sm5507_data->pd_state = 0;
	sm5507_data->func_state = 0;
	sm5507_data->data_role = 0;
	sm5507_data->is_host = 0;
	sm5507_data->is_client = 0;
	sm5507_data->manual_lpm_mode = 0;
	sm5507_data->water_det = 0;
	sm5507_data->try_state_change = 0;
    sm5507_data->attach = 0;
	wake_lock_init(&sm5507_data->wlock, WAKE_LOCK_SUSPEND, "sm5507-intr");

#if defined(CONFIG_CCIC_NOTIFIER)
	/* Create a work queue for the ccic irq thread */
	sm5507_data->ccic_wq = create_singlethread_workqueue("ccic_irq_event");
	 if (!sm5507_data->ccic_wq) {
		pr_err("%s failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err_free_redriver_gpio;
	 }
#endif

     _check_fw_version_update(sm5507_data);

#if defined(CONFIG_DUAL_ROLE_USB_INTF)
	desc = devm_kzalloc(&i2c->dev,
				 sizeof(struct dual_role_phy_desc), GFP_KERNEL);
	if (!desc) {
		pr_err("unable to allocate dual role descriptor\n");
		goto err_init_irq;
	}
	desc->name = "otg_default";
	desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
	desc->get_property = sm5507_dual_role_get_local_prop;
	desc->set_property = sm5507_dual_role_set_prop;
	desc->properties = sm5507_dual_role_properties;
	desc->num_properties = ARRAY_SIZE(sm5507_dual_role_properties);
	desc->property_is_writeable = sm5507_dual_role_is_writeable;
	dual_role = devm_dual_role_instance_register(&i2c->dev, desc);
	dual_role->drv_data = sm5507_data;
	sm5507_data->dual_role = dual_role;
	sm5507_data->desc = desc;
	init_completion(&sm5507_data->reverse_completion);
	INIT_DELAYED_WORK(&sm5507_data->role_swap_work, sm5507_role_swap_check);
#endif

#if defined(CONFIG_CCIC_ALTERNATE_MODE)
	sm5507_data->alternate_state = 0;
	sm5507_data->acc_type = 0;
	ccic_register_switch_device(1);
	INIT_DELAYED_WORK(&sm5507_data->acc_detach_work, acc_detach_check);
#endif

	ret = request_threaded_irq(sm5507_data->irq, NULL, sm5507_irq_thread,
		(IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT), "sm5507-ccic", sm5507_data);
	if (ret) {
		dev_err(&i2c->dev, "fail to request IRQ %d, error %d\n", sm5507_data->irq, ret);
		goto err_init_irq;
	}

	sm5507_int_clear(sm5507_data);

    g_sm5507_data = sm5507_data;

	return ret;

err_init_irq:
	if (sm5507_data->irq) {
		free_irq(sm5507_data->irq, sm5507_data);
		sm5507_data->irq = 0;
	}
err_free_redriver_gpio:
	gpio_free(sm5507_data->redrv1_gpio);
err_free_irq_gpio:
	wake_lock_destroy(&sm5507_data->wlock);
	gpio_free(sm5507_data->irq_gpio);
	kfree(sm5507_data);
	return ret;
}

static int sm5507_remove(struct i2c_client *i2c)
{
	struct sm5507_data *sm5507_data = dev_get_drvdata(ccic_device);

#if defined(CONFIG_DUAL_ROLE_USB_INTF)
	devm_dual_role_instance_unregister(sm5507_data->dev, sm5507_data->dual_role);
	devm_kfree(sm5507_data->dev, sm5507_data->desc);
#endif

#if defined(CONFIG_CCIC_ALTERNATE_MODE)
	ccic_register_switch_device(0);
#endif

	wake_lock_destroy(&sm5507_data->wlock);

	return 0;
}

static void sm5507_shutdown(struct i2c_client *i2c)
{
	struct sm5507_data *sm5507_data = i2c_get_clientdata(i2c);

	disable_irq(sm5507_data->irq);

	if ((sm5507_data->cur_rid != RID_523K) &&
	    (sm5507_data->cur_rid != RID_619K))
		sm5507_reset(sm5507_data);
}

#if defined(CONFIG_PM)
static int sm5507_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct sm5507_data *sm5507_data = i2c_get_clientdata(i2c);

#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
	pr_info("%s:%s\n", SM5507_CCIC_DEV_NAME, __func__);
#endif /* CONFIG_SAMSUNG_PRODUCT_SHIP */

	if (device_may_wakeup(dev))
		enable_irq_wake(sm5507_data->irq);

	disable_irq(sm5507_data->irq);

	return 0;
}

static int sm5507_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct sm5507_data *sm5507_data = i2c_get_clientdata(i2c);

#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
	pr_info("%s:%s\n", SM5507_CCIC_DEV_NAME, __func__);
#endif /* CONFIG_SAMSUNG_PRODUCT_SHIP */

	if (device_may_wakeup(dev))
		disable_irq_wake(sm5507_data->irq);

	enable_irq(sm5507_data->irq);

	return 0;
}
#else
#define sm5507_suspend	    NULL
#define sm5507_resume		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id sm5507_id[] = {
	{ SM5507_CCIC_DEV_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, sm5507_ccic_id);

#if defined(CONFIG_OF)
static struct of_device_id sm5507_i2c_dt_ids[] = {
	{ .compatible = "sec-sm5507,i2c" },
	{ }
};
#endif /* CONFIG_OF */

#if defined(CONFIG_PM)
const struct dev_pm_ops sm5507_pm = {
	.suspend = sm5507_suspend,
	.resume = sm5507_resume,
};
#endif /* CONFIG_PM */

static struct i2c_driver sm5507_ccic_driver = {
	.driver		= {
		.name	= SM5507_CCIC_DEV_NAME,
#if defined(CONFIG_PM)
		.pm	= &sm5507_pm,
#endif /* CONFIG_PM */
#if defined(CONFIG_OF)
		.of_match_table	= sm5507_i2c_dt_ids,
#endif /* CONFIG_OF */
	},
	.probe		= sm5507_probe,
	.remove		= sm5507_remove,
	.shutdown	= sm5507_shutdown,
	.id_table	= sm5507_id,
};

static int __init sm5507_ccic_init(void)
{
	return i2c_add_driver(&sm5507_ccic_driver);
}
module_init(sm5507_ccic_init);

static void __exit sm5507_ccic_exit(void)
{
	i2c_del_driver(&sm5507_ccic_driver);
}
module_exit(sm5507_ccic_exit);

MODULE_DESCRIPTION("SM5507 CCIC driver");
MODULE_LICENSE("GPL");

