#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>

//#include <mt-plat/mt_typedefs.h>
#include <mt-plat/charging.h>
#include <mach/mt_charging.h>
//#include <mt-plat/mt_gpio.h>

#include <sm5424.h>

extern kal_bool chargin_hw_init_done;

static struct i2c_client *new_client = NULL;

struct sm5424_info {
	//struct platform_device *pdev;
	struct power_supply psy;
	struct device *pdev;		/* the device structure */

	//i2c
	struct i2c_client *i2c;

	/* gpio */
	int nCHGEN;
	int nINT;

	/* interrupt */
	int irq;
	
	struct pinctrl *pin;
	struct pinctrl_state *boot;
	struct pinctrl_state *charging;
	struct pinctrl_state *not_charging;

	/* current */
	int vbuslimit; 		/* mA */
	int fastchg; 		/* mA */

	/* status */
	int status;
	int enable;

	/* lock */
	struct mutex i2c_lock;

	int cv_voltage;		/* in uV */
};

int sm5424_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct sm5424_info *sm5424 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&sm5424->i2c_lock);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&sm5424->i2c_lock);
	if (ret < 0) {
		printk("[%s] reg(0x%x), ret(%d)\n",__func__, reg, ret);		
		return ret;
	}
	*dest = (ret & 0xff);

    return 0;
}

int sm5424_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct sm5424_info *sm5424 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&sm5424->i2c_lock);
	ret = i2c_smbus_read_i2c_block_data(i2c, reg, count, buf);
	mutex_unlock(&sm5424->i2c_lock);
	if (ret < 0) {		
		return ret;
	}

	return 0;
}

int sm5424_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct sm5424_info *sm5424 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&sm5424->i2c_lock);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&sm5424->i2c_lock);
	if (ret < 0) {		
		printk("[%s]reg(0x%x), ret(%d)\n",__func__, reg, ret);		
	}

	return ret;
}

int sm5424_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct sm5424_info *sm5424 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&sm5424->i2c_lock);
	ret = i2c_smbus_write_i2c_block_data(i2c, reg, count, buf);
	mutex_unlock(&sm5424->i2c_lock);
	if (ret < 0)
		return ret;

	return 0;
}

int sm5424_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct sm5424_info *sm5424 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&sm5424->i2c_lock);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret >= 0) {
		u8 old_val = ret & 0xff;
		u8 new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	mutex_unlock(&sm5424->i2c_lock);
	
	return ret;
}

static int sm5424_gpio_init(struct sm5424_info *info)
{
	int err = 0;
	
	err = gpio_request(info->nCHGEN, "nCHGEN");
	if (err) {
		printk("[%s] failed requesting : gpio %s , err = %d\n",__func__, "nCHGEN", err);	
		return err;
	}
	
	err = gpio_request(info->nINT, "nINT");
	if (err) {
		printk("[%s] failed requesting : gpio %s , err = %d\n",__func__, "nINT", err);	
		return err;		
	}
	
	return 0;
}

static int sm5424_get_status(struct sm5424_info *info)
{

	u8 status2 = 0;
	u8 status = 0;
	
	sm5424_read_reg(info->i2c, SM5424_STATUS2, &status2);

	if (status2 & SM5424_STATUS2_CHGON) {
		status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		if (status2 & SM5424_STATUS2_TOPOFF)
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	printk("[%s] : STATUS2 : 0x%x, status = 0x%x\n",__func__, status2, status);

	return status;
}

/**
 * SM5424 Charger Register control functions.
 */
static inline u8 _calc_limit_current_offset_to_mA(unsigned short mA)
{
	unsigned char offset;

	if(mA < 375) {
		offset = 0x00;
	}else {
        offset = (((mA - 375) / 25)+1) & SM5424_VBUSCNTL_VBUSLIMIT_MASK;
	}

	return offset;
}

static inline unsigned short _calc_limit_current_mA_to_offset(u8 offset)
{
	unsigned short mA;

	if (offset == 0) {
		mA = 100;
	} else {
		mA = 375 + ((offset-1) * 25);
	}
	
	return mA;
}

static inline u8 _calc_chg_current_offset_to_mA(unsigned short mA)
{
    unsigned char offset;

    if (mA < 350) {
        offset = 0x00;
    } else {
        offset = ((mA - 350) / 50) & SM5424_CHGCNTL2_FASTCHG_MASK;
    }

    return offset;
}

static inline unsigned short _calc_chg_current_mA_to_offset(u8 offset)
{
	return 350 + (offset * 50);
}

static inline u8 _calc_bat_float_voltage_offset_to_uV(unsigned int uV)
{
	unsigned char offset;

	if (uV < 3990000) {
		offset = 0x0;     /* BATREG = 3.990000V */
	} else if (uV <= 4620000) {
		offset = ((uV - 3990000) / 10000) & SM5424_CHGCNTL3_BATREG_MASK;    /* BATREG = 3.990000 ~ 4.620000 in 0.010000V steps */
	} else {
        printk("[%s] : can't support BATREG at over voltage 4.620000V (mV=%d)\n", __func__, uV);
		offset = 0x15;    /* default Offset : 4.200000V */
	}

	return offset;
}

static inline unsigned short _calc_bat_float_voltage_mV_to_offset(u8 offset)
{
	return 3990 + (offset * 10);
}

static inline unsigned char _calc_topoff_current_offset_to_mA(unsigned short mA)
{
	unsigned char offset;

	if (mA < 100) {
		offset = 0x0;               /* Topoff = 100mA */
	} else if (mA < 475) {
		offset = ((mA - 100) / 25) & SM5424_CHGCNTL4_TOPOFF_MASK;   /* Topoff = 125mA ~ 450mA in 25mA steps */
	} else {
		offset = 0x0F;              /* Topoff = 475mA */
	}

	return offset;
}


static int sm5424_CHG_set_VBUSLIMIT(struct sm5424_info *info, unsigned short limit_mA)
{
    u8 offset = _calc_limit_current_offset_to_mA(limit_mA);

    sm5424_update_reg(info->i2c, SM5424_VBUSCNTL, ((offset & SM5424_VBUSCNTL_VBUSLIMIT_MASK) << SM5424_VBUSCNTL_VBUSLIMIT_SHIFT), (SM5424_VBUSCNTL_VBUSLIMIT_MASK << SM5424_VBUSCNTL_VBUSLIMIT_SHIFT));

    return 0;
}

static unsigned short sm5424_CHG_get_VBUSLIMIT(struct sm5424_info *info)
{
    u8 offset = 0x0;

    sm5424_read_reg(info->i2c, SM5424_VBUSCNTL, &offset);

    return _calc_limit_current_mA_to_offset(offset & SM5424_VBUSCNTL_VBUSLIMIT_MASK);
}

static int sm5424_CHG_set_FASTCHG(struct sm5424_info *info, unsigned short chg_mA)
{
    u8 offset = _calc_chg_current_offset_to_mA(chg_mA);

    sm5424_update_reg(info->i2c, SM5424_CHGCNTL2, ((offset & SM5424_CHGCNTL2_FASTCHG_MASK) << SM5424_CHGCNTL2_FASTCHG_SHIFT), (SM5424_CHGCNTL2_FASTCHG_MASK << SM5424_CHGCNTL2_FASTCHG_SHIFT));

    return 0;
}

static unsigned short sm5424_CHG_get_FASTCHG(struct sm5424_info *info)
{
    u8 offset = 0x0;

    sm5424_read_reg(info->i2c, SM5424_CHGCNTL2, &offset);

    return _calc_chg_current_mA_to_offset(offset & SM5424_CHGCNTL2_FASTCHG_MASK);
}

static int sm5424_CHG_set_BATREG(struct sm5424_info *info, unsigned int float_uV)
{
    u8 offset = _calc_bat_float_voltage_offset_to_uV(float_uV);

    sm5424_update_reg(info->i2c, SM5424_CHGCNTL3, ((offset & SM5424_CHGCNTL3_BATREG_MASK) << SM5424_CHGCNTL3_BATREG_SHIFT), (SM5424_CHGCNTL3_BATREG_MASK << SM5424_CHGCNTL3_BATREG_SHIFT));

    return 0;
}

static unsigned int sm5424_CHG_get_BATREG(struct sm5424_info *info)
{
    u8 offset = 0x0;

    sm5424_read_reg(info->i2c, SM5424_CHGCNTL3, &offset);

    return _calc_bat_float_voltage_mV_to_offset(offset & SM5424_CHGCNTL3_BATREG_MASK);
}

static int sm5424_CHG_set_TOPOFF(struct sm5424_info *info, unsigned short topoff_mA)
{
    u8 offset = _calc_topoff_current_offset_to_mA(topoff_mA);

    sm5424_update_reg(info->i2c, SM5424_CHGCNTL4, ((offset & SM5424_CHGCNTL4_TOPOFF_MASK) << SM5424_CHGCNTL4_TOPOFF_SHIFT), (SM5424_CHGCNTL4_TOPOFF_MASK << SM5424_CHGCNTL4_TOPOFF_SHIFT));

    return 0;
}

static int sm5424_CHG_set_AICLEN(struct sm5424_info *info, bool enable)
{
    sm5424_update_reg(info->i2c, SM5424_CHGCNTL1, ((enable & SM5424_CHGCNTL1_AICLEN_MASK) << SM5424_CHGCNTL1_AICLEN_SHIFT), (SM5424_CHGCNTL1_AICLEN_MASK << SM5424_CHGCNTL1_AICLEN_SHIFT));

    return 0;
}

static int sm5424_CHG_set_AICLTH(struct sm5424_info *info, u8 val)
{
    sm5424_update_reg(info->i2c, SM5424_CHGCNTL1, ((val & SM5424_CHGCNTL1_AICLTH_MASK) << SM5424_CHGCNTL1_AICLTH_SHIFT), (SM5424_CHGCNTL1_AICLTH_MASK << SM5424_CHGCNTL1_AICLTH_SHIFT));

    return 0;
}
static int sm5424_CHG_set_AUTOSTOP(struct sm5424_info *info, bool enable)
{
    sm5424_update_reg(info->i2c, SM5424_CNTL, ((enable & SM5424_CNTL_AUTOSTOP_MASK) << SM5424_CNTL_AUTOSTOP_SHIFT), (SM5424_CNTL_AUTOSTOP_MASK << SM5424_CNTL_AUTOSTOP_SHIFT));

    return 0;
}

static int sm5424_CHG_set_ENBOOST(struct sm5424_info *info, bool enable)
{
    sm5424_update_reg(info->i2c, SM5424_CNTL, ((enable & SM5424_CNTL_ENBOOST_MASK) << SM5424_CNTL_ENBOOST_SHIFT), (SM5424_CNTL_ENBOOST_MASK << SM5424_CNTL_ENBOOST_SHIFT));

    return 0;
}

static int sm5424_CHG_set_OTGCURRENT(struct sm5424_info *info, u8 val)
{
    sm5424_update_reg(info->i2c, SM5424_CNTL, ((val & SM5424_CHGCNTL5_OTGCURRENT_MASK) << SM5424_CHGCNTL5_OTGCURRENT_SHIFT), (SM5424_CHGCNTL5_OTGCURRENT_MASK << SM5424_CHGCNTL5_OTGCURRENT_SHIFT));

    return 0;
}

static int sm5424_CHG_set_BST_IQ3LIMIT(struct sm5424_info *info, u8 val)
{
    sm5424_update_reg(info->i2c, SM5424_CHGCNTL5, ((val & SM5424_CHGCNTL5_BST_IQ3LIMIT_MASK) << SM5424_CHGCNTL5_BST_IQ3LIMIT_SHIFT), (SM5424_CHGCNTL5_BST_IQ3LIMIT_MASK << SM5424_CHGCNTL5_BST_IQ3LIMIT_SHIFT));

    return 0;
}

static int sm5424_CHG_set_CHGEN(struct sm5424_info *info, int en)
{
	int rc = 0;

	printk("[%s] : Enable = <%d>\n",__func__, en);
	
	if (en) {
		pinctrl_select_state(info->pin, info->charging);
		sm5424_update_reg(info->i2c, SM5424_CNTL, ((en & SM5424_CNTL_CHGEN_MASK)<<SM5424_CNTL_CHGEN_SHIFT), (SM5424_CNTL_CHGEN_MASK<<SM5424_CNTL_CHGEN_SHIFT));		
	} else {
		pinctrl_select_state(info->pin, info->not_charging);
		sm5424_update_reg(info->i2c, SM5424_CNTL, ((en & SM5424_CNTL_CHGEN_MASK)<<SM5424_CNTL_CHGEN_SHIFT), (SM5424_CNTL_CHGEN_MASK<<SM5424_CNTL_CHGEN_SHIFT));		
	}
	
	printk("[%s] : nCHGEN = %d\n",__func__, gpio_get_value(info->nCHGEN));

	return rc;
}

static void sm5424_CHG_print_reg(struct sm5424_info *info)
{
    u8 regs[SM5424_REG_NUM] = {0x0, };
    int i;

	for (i = SM5424_INTMASK1; i < (SM5424_CHGCNTL6 + 1); i++)
	{
    	sm5424_read_reg(info->i2c, i, &regs[i]);
	}
	sm5424_read_reg(info->i2c, SM5424_DEVICEID, &regs[SM5424_CHGCNTL6+1]);
	
    printk("sm5720-charger: ");

    for (i = SM5424_INTMASK1; i < (SM5424_CHGCNTL6 + 1); ++i) {
        pr_info("0x%x:0x%x ", i, regs[i]);
    }
	printk("0x%x:0x%x ", SM5424_DEVICEID, regs[SM5424_CHGCNTL6+1]);

    pr_info("\n");
}

void sm5424_otg_enable(unsigned int enable)
{
	struct sm5424_info *info = i2c_get_clientdata(new_client);
		
	printk("[%s] : Enable = <%d>\n",__func__, enable);

	if (new_client != NULL) {
	    if (KAL_TRUE == enable) {			
			pinctrl_select_state(info->pin, info->not_charging);
			sm5424_update_reg(new_client, SM5424_CNTL, ((CHARGE_DIS & SM5424_CNTL_CHGEN_MASK)<<SM5424_CNTL_CHGEN_SHIFT), (SM5424_CNTL_CHGEN_MASK<<SM5424_CNTL_CHGEN_SHIFT));		
			sm5424_update_reg(new_client, SM5424_CNTL, ((ENBOOST_EN & SM5424_CNTL_ENBOOST_MASK)<<SM5424_CNTL_ENBOOST_SHIFT), (SM5424_CNTL_ENBOOST_MASK<<SM5424_CNTL_ENBOOST_SHIFT));
	    } else {   
			sm5424_update_reg(new_client, SM5424_CNTL, ((ENBOOST_DIS & SM5424_CNTL_ENBOOST_MASK)<<SM5424_CNTL_ENBOOST_SHIFT), (SM5424_CNTL_ENBOOST_MASK<<SM5424_CNTL_ENBOOST_SHIFT));
	    }
	} else {
		printk("[%s] : Enable = <%d>, new_client = NULL\n",__func__, enable);		
	}

	printk("[%s] : nCHGEN = %d\n",__func__, gpio_get_value(info->nCHGEN));			

	
}

static void sm5424_charger_initialize(struct sm5424_info *info)
{
	printk("charger initial hardware condition process start.\n");

	sm5424_write_reg(info->i2c, SM5424_INTMASK1, 0xFF);	
	sm5424_write_reg(info->i2c, SM5424_INTMASK2, 0xFF);	
	sm5424_write_reg(info->i2c, SM5424_INTMASK3, 0xFF);	

	sm5424_CHG_set_AICLTH(info, AICL_THRESHOLD_4_5_V);
    sm5424_CHG_set_AICLEN(info, AICL_EN);
	sm5424_CHG_set_BATREG(info, BATREG_4400mV);
	sm5424_CHG_set_TOPOFF(info, TOPOFF_100mA);

    sm5424_CHG_set_BST_IQ3LIMIT(info, BSTIQ3LIMIT_2P0A);

    sm5424_CHG_set_AUTOSTOP(info, AUTOSTOP_DIS);

	sm5424_CHG_print_reg(info);

	printk("charger initial hardware condition process done.\n");
}


static enum power_supply_property sm5424_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
};

static int sm5424_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sm5424_info *info = container_of(psy, struct sm5424_info, psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = sm5424_get_status(info);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = info->cv_voltage;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = info->vbuslimit;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = info->fastchg;		
		break;
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		val->intval = info->enable;		
		break;		
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int sm5424_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct sm5424_info *info = container_of(psy, struct sm5424_info, psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		switch (val->intval) {
		case POWER_SUPPLY_STATUS_CHARGING:
			info->enable = 1;
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			info->enable = 0;
			break;
		default:
			rc = -EINVAL;
			break;
		}		
		if (!rc)
			sm5424_CHG_set_CHGEN(info,info->enable);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		sm5424_CHG_set_BATREG(info, val->intval);
		info->cv_voltage = val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		sm5424_CHG_set_VBUSLIMIT(info,val->intval);
		info->vbuslimit = val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		sm5424_CHG_set_FASTCHG(info,val->intval);
		info->fastchg = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		sm5424_CHG_set_CHGEN(info,val->intval);
		info->enable= val->intval;
		break;		
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int sm5424_property_is_writeable(struct power_supply *psy,
				     enum power_supply_property psp)
{
	int rc = 0;
	
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static struct power_supply sm5424_psy = {
	.name = "sm5424",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties	= sm5424_props,
	.num_properties = ARRAY_SIZE(sm5424_props),
	.get_property	= sm5424_get_property,
	.set_property	= sm5424_set_property,
	.property_is_writeable = sm5424_property_is_writeable,
};

static int sm5424_parse_dt(struct device *dev, struct sm5424_info *info)
{
	struct device_node *np = dev->of_node;
	int rc;

	rc = of_property_read_u32(np, "nCHGEN" ,&info->nCHGEN);
	if (rc) {
		printk("[%s] : nCHGEN not defined.\n",__func__);
		return rc;
	}

	rc = of_property_read_u32(np, "nINT" ,&info->nINT);
	if (rc) {
		printk("[%s] : nINT not defined.",__func__);
		return rc;
	}

	info->pin = devm_pinctrl_get(dev);
	info->boot = pinctrl_lookup_state(info->pin, "default");
	info->charging = pinctrl_lookup_state(info->pin, "charging");
	info->not_charging = pinctrl_lookup_state(info->pin, "not_charging");

	return 0;
}

static irqreturn_t sm5424_irq_handler(struct sm5424_info *info)
{
	u8 int_value[3] = {0,};
	
	sm5424_read_reg(info->i2c, SM5424_INT1, &int_value[SM5424_INT1]);
	sm5424_read_reg(info->i2c, SM5424_INT2, &int_value[SM5424_INT2]);
	sm5424_read_reg(info->i2c, SM5424_INT3, &int_value[SM5424_INT3]);

	printk("[%s] INT1 : 0x%x, INT2 : 0x%x, INT3 : 0x%x\n",__func__,int_value[SM5424_INT1],int_value[SM5424_INT2],int_value[SM5424_INT3]);

	return IRQ_HANDLED;
}

static int sm5424_irq_init(struct sm5424_info *info)
{	
	int ret = 0;
	struct device_node *np = info->pdev->of_node;
	
	printk("[%s] : Start\n",__func__);
	
	if(np){
		info->irq = gpio_to_irq(info->nINT);

		gpio_direction_input(info->nINT);
		
		ret = request_threaded_irq(info->irq, NULL, (irq_handler_t)sm5424_irq_handler, IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_ONESHOT, "sm5424-irq", info);

		if(ret < 0){		    
			printk("[%s] : request_irq IRQ LINE NOT AVAILABLE!.\n",__func__);
		}
	}else{
		printk("[%s] : request_irq can not find  eint device node!.\n",__func__);	
		ret = -1;
	}

	printk("[%s] : Done\n",__func__);

	return ret;
}

static int sm5424_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sm5424_info *info;
	int rc;

	printk("[%s] : Start\n",__func__);

	info = (struct sm5424_info*)kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		printk("[%s] sm5424_info memory allocation failed.\n",__func__);
		return -ENOMEM;
	}

	/* initialize device info */
	info->i2c = client;
	info->pdev = &client->dev;
	info->cv_voltage = 4200000;
	info->status = POWER_SUPPLY_STATUS_UNKNOWN;

	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!new_client) {
		printk("[%s] new_client memory allocation failed.\n",__func__);
		return -ENOMEM;
	}

	new_client = client;

	/* read device tree */
	if (client->dev.of_node)
	{			
		rc = sm5424_parse_dt(&client->dev, info);
		if (rc) {			
			printk("[%s] cannot read from fdt.\n",__func__);
			return rc;
		}
	}

	/* initialize device */
	sm5424_gpio_init(info);

	mutex_init(&info->i2c_lock);

	i2c_set_clientdata(client, info);

	sm5424_charger_initialize(info);

	/* register class */
	info->psy = sm5424_psy;
	rc = power_supply_register(&client->dev, &info->psy);
	if (rc) {
		printk("[%s] power supply register failed.\n",__func__);		
		return rc;
	}

	rc = sm5424_irq_init(info);
	if (rc < 0) {
		printk("[%s] Error : can't initialize SM5424 irq.\n",__func__);
		return rc;
	}

	chargin_hw_init_done = KAL_TRUE;

	return 0;
}

static struct of_device_id sm5424_of_device_id[] = {
	{
		.compatible = "siliconmitus,sm5424",
	},
};

static struct i2c_driver sm5424_driver = {
	.probe		= sm5424_probe,
	.driver		= {
		.name	= "sm5424",
		.owner	= THIS_MODULE,
		.of_match_table = sm5424_of_device_id,
	},
};

static int __init sm5424_init(void)
{
	printk("[%s] init start with i2c DTS\n",__func__);

	if (i2c_add_driver(&sm5424_driver) != 0) {
		printk("[%s] failed to register sm5424 i2c driver.\n",__func__);
		return -ENODEV;
	} else {
		printk("[%s] Success to register sm5424 i2c driver.\n",__func__);
	}

	return 0;
}

static void __exit sm5424_exit(void)
{
	printk("[%s]\n",__func__);

	i2c_del_driver(&sm5424_driver);
}
module_init(sm5424_init);
module_exit(sm5424_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Siliconmitus SM5424 Driver");
