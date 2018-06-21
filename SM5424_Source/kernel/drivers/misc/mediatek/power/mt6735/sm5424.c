#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <linux/pinctrl/consumer.h>
#include <mt-plat/charging.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include "sm5424.h"

 /**********************************************************
   *
   *   [I2C Slave Setting]
   *
   *********************************************************/ 
static struct i2c_client *new_client;
static const struct i2c_device_id sm5424_i2c_id[] = { {"sm5424", 0}, {} }; 
int sm5424_irq = 0;
int sm5424_int = 0;
kal_bool chargin_hw_init_done = KAL_FALSE;

struct sm5424_info {
	struct device *pdev;		/* the device structure */
	struct i2c_client *i2c;		/* i2c */

	int nCHGEN;					/* gpio */
	int nINT;					/* gpio */

	/* interrupt */
	int irq;
	
	struct pinctrl *pin;
	struct pinctrl_state *boot;
	struct pinctrl_state *charging;
	struct pinctrl_state *not_charging;
};

static int sm5424_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);
 
#ifdef CONFIG_OF
 static const struct of_device_id sm5424_of_match[] = {
	 {.compatible = "siliconmitus,sm5424",},
	 {},
 };
 
 MODULE_DEVICE_TABLE(of, sm5424_of_match);
#endif
 
 static struct i2c_driver sm5424_driver = {
	 .driver = {
			.name = "sm5424",
#ifdef CONFIG_OF
			.of_match_table = sm5424_of_match,
#endif
			},
	 .probe = sm5424_driver_probe,
	 .id_table = sm5424_i2c_id,
 };
 
/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char sm5424_reg[SM5424_REG_NUM] = { 0 };

static DEFINE_MUTEX(sm5424_i2c_access);

int g_sm5424_hw_exist = 0;

/**********************************************************
  *
  *   [I2C Function For Read/Write sm5424]
  *
  *********************************************************/
int sm5424_read_byte(unsigned char cmd, unsigned char *returnData)
{
	 char cmd_buf[1] = { 0x00 };
	 char readData = 0;
	 int ret = 0;

	 mutex_lock(&sm5424_i2c_access);

	 /* new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG; */
	 new_client->ext_flag =
		 ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	 cmd_buf[0] = cmd;
	 ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	 if (ret < 0) {
		 /* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
		 new_client->ext_flag = 0;

		 mutex_unlock(&sm5424_i2c_access);
		 return 0;
	 }

	 readData = cmd_buf[0];
	 *returnData = readData;

	 /* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
	 new_client->ext_flag = 0;

	 mutex_unlock(&sm5424_i2c_access);
	 return 1;
}

int sm5424_write_byte(unsigned char cmd, unsigned char writeData)
{
	 char write_data[2] = { 0 };
	 int ret = 0;

	 mutex_lock(&sm5424_i2c_access);

	 write_data[0] = cmd;
	 write_data[1] = writeData;

	 new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	 ret = i2c_master_send(new_client, write_data, 2);
	 if (ret < 0) {
		 new_client->ext_flag = 0;
		 mutex_unlock(&sm5424_i2c_access);
		 return 0;
	 }

	 new_client->ext_flag = 0;
	 mutex_unlock(&sm5424_i2c_access);
	 return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int sm5424_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				   unsigned char SHIFT)
{
	 unsigned char sm5424_reg = 0;
	 int ret = 0;

	 battery_log(BAT_LOG_FULL, "--------------------------------------------------\n");

	 ret = sm5424_read_byte(RegNum, &sm5424_reg);

	 battery_log(BAT_LOG_FULL, "[sm5424_read_interface] Reg[%x]=0x%x\n", RegNum, sm5424_reg);

	 sm5424_reg &= (MASK << SHIFT);
	 *val = (sm5424_reg >> SHIFT);

	 battery_log(BAT_LOG_FULL, "[sm5424_read_interface] val=0x%x\n", *val);

	 return ret;
}

unsigned int sm5424_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
					 unsigned char SHIFT)
{
	 unsigned char sm5424_reg = 0;
	 int ret = 0;

	 battery_log(BAT_LOG_FULL, "--------------------------------------------------\n");

	 ret = sm5424_read_byte(RegNum, &sm5424_reg);
	 battery_log(BAT_LOG_FULL, "[sm5424_config_interface] Reg[%x]=0x%x\n", RegNum, sm5424_reg);

	 sm5424_reg &= ~(MASK << SHIFT);
	 sm5424_reg |= (val << SHIFT);

	 ret = sm5424_write_byte(RegNum, sm5424_reg);
	 battery_log(BAT_LOG_FULL, "[sm5424_config_interface] write Reg[%x]=0x%x\n", RegNum, sm5424_reg);

	 /* Check */
	 /* sm5424_read_byte(RegNum, &sm5424_reg); */
	 /* battery_log(BAT_LOG_FULL, "[sm5424_config_interface] Check Reg[%x]=0x%x\n", RegNum, sm5424_reg); */

	 return ret;
}

/* write one register directly */
unsigned int sm5424_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	 unsigned int ret = 0;

	 ret = sm5424_write_byte(RegNum, val);

	 return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* STATUS1---------------------------------------------------- */
 unsigned int sm5424_get_status1(void)
{
	 unsigned char val=0;
	 unsigned int ret=0; 
 
	 ret=sm5424_read_byte(SM5424_STATUS1,&val);
	 
	 return val;
}

/* STATUS2---------------------------------------------------- */
unsigned int sm5424_get_status2(void)
{
	 unsigned char val=0;
	 unsigned int ret=0; 

   ret=sm5424_read_byte(SM5424_STATUS2,&val);
	
   return val;
}

/* STATUS3---------------------------------------------------- */
unsigned int sm5424_get_status3(void)
{
	 unsigned char val=0;
	 unsigned int ret=0; 
 
	 ret=sm5424_read_byte(SM5424_STATUS3, &val);
	 
	 return val;
}

/* CNTL---------------------------------------------------- */ 
void sm5424_set_nENZCS(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CNTL),
						(unsigned char) (val),
						(unsigned char) (SM5424_CNTL_nENZCS_MASK),
						(unsigned char) (SM5424_CNTL_nENZCS_SHIFT)
		 );
}

void sm5424_set_ENI2CRESET(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CNTL),
						(unsigned char) (val),
						(unsigned char) (SM5424_CNTL_ENI2CRESET_MASK),
						(unsigned char) (SM5424_CNTL_ENI2CRESET_SHIFT)
		 );
} 

void sm5424_set_SUSPEND(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CNTL),
						(unsigned char) (val),
						(unsigned char) (SM5424_CNTL_SUSPEND_MASK),
						(unsigned char) (SM5424_CNTL_SUSPEND_SHIFT)
		 );
} 

void sm5424_set_RESET(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CNTL),
						(unsigned char) (val),
						(unsigned char) (SM5424_CNTL_RESET_MASK),
						(unsigned char) (SM5424_CNTL_RESET_SHIFT)
		 );
} 

void sm5424_set_ENBOOST(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CNTL),
						(unsigned char) (val),
						(unsigned char) (SM5424_CNTL_ENBOOST_MASK),
						(unsigned char) (SM5424_CNTL_ENBOOST_SHIFT)
		 );
}  

void sm5424_set_AUTOSTOP(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CNTL),
						(unsigned char) (val),
						(unsigned char) (SM5424_CNTL_AUTOSTOP_MASK),
						(unsigned char) (SM5424_CNTL_AUTOSTOP_SHIFT)
		 );
}  

void sm5424_set_CHGEN(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CNTL),
						(unsigned char) (val),
						(unsigned char) (SM5424_CNTL_CHGEN_MASK),
						(unsigned char) (SM5424_CNTL_CHGEN_SHIFT)
		 );
}  
/* VBUSCNTL---------------------------------------------------- */
void sm5424_set_VBUSLIMIT(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_VBUSCNTL),
						(unsigned char) (val),
						(unsigned char) (SM5424_VBUSCNTL_VBUSLIMIT_MASK),
						(unsigned char) (SM5424_VBUSCNTL_VBUSLIMIT_SHIFT)
		 );
}

/* CHGCNTL1---------------------------------------------------- */
void sm5424_set_AICLTH(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL1),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL1_AICLTH_MASK), 
						(unsigned char) (SM5424_CHGCNTL1_AICLTH_SHIFT)
		 );
}

void sm5424_set_AICLEN(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL1),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL1_AICLEN_MASK), 
						(unsigned char) (SM5424_CHGCNTL1_AICLEN_SHIFT)
		 );
}

void sm5424_set_DISLIMIT(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL1),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL1_DISLIMIT_MASK), 
						(unsigned char) (SM5424_CHGCNTL1_DISLIMIT_SHIFT)
		 );
}

void sm5424_set_PRECHG(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL1),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL1_PRECHG_MASK), 
						(unsigned char) (SM5424_CHGCNTL1_PRECHG_SHIFT)
		 );
}

/* CHGCNTL2---------------------------------------------------- */
void sm5424_set_FASTCHG(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL2),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL2_FASTCHG_MASK), 
						(unsigned char) (SM5424_CHGCNTL2_FASTCHG_SHIFT)
		 );
}

/* CHGCNTL3---------------------------------------------------- */
void sm5424_set_BATREG(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL3),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL3_BATREG_MASK), 
						(unsigned char) (SM5424_CHGCNTL3_BATREG_SHIFT)
		 );
}

/* CHGCNTL4---------------------------------------------------- */

void sm5424_set_Q2LIM(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL4),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL4_Q2LIM_MASK), 
						(unsigned char) (SM5424_CHGCNTL4_Q2LIM_SHIFT)
		 );
}

void sm5424_set_FREQSEL(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL4),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL4_FREQSEL_MASK), 
						(unsigned char) (SM5424_CHGCNTL4_FREQSEL_SHIFT)
		 );
}

void sm5424_set_TOPOFF(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL4),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL4_TOPOFF_MASK), 
						(unsigned char) (SM5424_CHGCNTL4_TOPOFF_SHIFT)
		 );
}

/* CHGCNTL5---------------------------------------------------- */
void sm5424_set_TOPOFFTIMER(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL5),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL5_TOPOFFTIMER_MASK), 
						(unsigned char) (SM5424_CHGCNTL5_TOPOFFTIMER_SHIFT)
		 );
}

void sm5424_set_FASTTIMER(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL5),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL5_FASTTIMER_MASK), 
						(unsigned char) (SM5424_CHGCNTL5_FASTTIMER_SHIFT)
		 );
}

void sm5424_set_OTGCURRENT(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL5),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL5_OTGCURRENT_MASK), 
						(unsigned char) (SM5424_CHGCNTL5_OTGCURRENT_SHIFT)
		 );
}

void sm5424_set_BST_IQ3LIMIT(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL5),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL5_BST_IQ3LIMIT_MASK), 
						(unsigned char) (SM5424_CHGCNTL5_BST_IQ3LIMIT_SHIFT)
		 );
}

/* CHGCNTL6---------------------------------------------------- */
void sm5424_set_VOTG(unsigned int val)
{
	 unsigned int ret = 0;

	 ret = sm5424_config_interface((unsigned char) (SM5424_CHGCNTL6),
						(unsigned char) (val),
						(unsigned char) (SM5424_CHGCNTL6_VOTG_MASK), 
						(unsigned char) (SM5424_CHGCNTL6_VOTG_SHIFT)
		 );
}

/* DEVICEID---------------------------------------------------- */
 unsigned int sm5424_get_REVISIONID(void)
{
	 unsigned char val=0;
	 unsigned int ret=0; 
	 
	 ret=sm5424_read_interface((unsigned char)(SM5424_DEVICEID), 
								(&val),
								(unsigned char)(SM5424_DEVICEID_REVISIONID_MASK),
								(unsigned char)(SM5424_DEVICEID_REVISIONID_SHIFT)
		);
	 
	 return val;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
void sm5424_hw_component_detect(void)
{
	 unsigned int ret = 0;
	 unsigned char val = 0;

	 ret = sm5424_read_interface(SM5424_DEVICEID, &val, SM5424_DEVICEID_REVISIONID_MASK, SM5424_DEVICEID_REVISIONID_SHIFT);

	 if (ret < 0)
		 g_sm5424_hw_exist = 0;
	 else
		 g_sm5424_hw_exist = 1;

	 battery_log(BAT_LOG_CRTI, "[sm5424_hw_component_detect] exist=%d, Reg[0x37]=0x%x\n", g_sm5424_hw_exist, val);
}

int is_sm5424_exist(void)
{
	 battery_log(BAT_LOG_CRTI, "[is_sm5424_exist] g_sm5424_hw_exist=%d\n", g_sm5424_hw_exist);

	 return g_sm5424_hw_exist;
}

void sm5424_dump_register(void)
{
	 int i = 0;

	 battery_log(BAT_LOG_FULL, "[sm5424] ");
	 for (i = SM5424_INTMASK1; i < SM5424_CHGCNTL6+1; i++) {
		 sm5424_read_byte(i, &sm5424_reg[i]);
		 battery_log(BAT_LOG_FULL, "[0x%x]=0x%x ", i, sm5424_reg[i]);
	 }
	 sm5424_read_byte(SM5424_DEVICEID, &sm5424_reg[SM5424_CHGCNTL6+1]);
	 battery_log(BAT_LOG_FULL, "[0x%x]=0x%x ", i, sm5424_reg[SM5424_CHGCNTL6+1]);
		 
	 battery_log(BAT_LOG_FULL, "\n");
}

static void sm5424_charger_initialize(void)
{
	 printk("charger initial hardware condition process start.\n");

	 sm5424_write_byte(SM5424_INTMASK1, 0xFF); 
	 sm5424_write_byte(SM5424_INTMASK2, 0xFF); 
	 sm5424_write_byte(SM5424_INTMASK3, 0xFF); 

	 sm5424_set_AICLTH(AICL_THRESHOLD_4_5_V);
	 sm5424_set_AICLEN(AICL_EN);
	 sm5424_set_BATREG(BATREG_4400mV);
	 sm5424_set_BST_IQ3LIMIT(BSTIQ3LIMIT_2P0A); 
	 sm5424_set_AUTOSTOP(AUTOSTOP_DIS); 
	 sm5424_set_TOPOFF(TOPOFF_100mA);

	 sm5424_dump_register();

	 printk("charger initial hardware condition process done.\n");
}

int sm5424_chg_enable(int en)
{
	 struct sm5424_info *info = i2c_get_clientdata(new_client);
	 int rc = 0;

	 printk("[%s] : Enable = <%d>\n",__func__, en);
	 
	 if (en) {
		pinctrl_select_state(info->pin, info->charging);
		sm5424_set_CHGEN(CHARGE_EN);
	 } else {
		 pinctrl_select_state(info->pin, info->not_charging);
		 sm5424_set_CHGEN(CHARGE_DIS);
	 }
	 
	 printk("[%s] : nCHGEN = %d\n",__func__, gpio_get_value(info->nCHGEN));

	 return rc;
}

void sm5424_otg_enable(unsigned int enable)
{			
	 printk("[%s] : Enable = <%d>\n",__func__, enable);


	if (new_client != NULL) {
		if (KAL_TRUE == enable) {			 
			sm5424_chg_enable(CHARGE_DIS);
			sm5424_set_ENBOOST(ENBOOST_EN);
		} else {
			sm5424_set_ENBOOST(ENBOOST_DIS);
		}
	} else {
		printk("[%s] : Enable = <%d>, new_client = NULL\n",__func__, enable);			
	}
}

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
	 
	 sm5424_read_byte(SM5424_INT1, &int_value[SM5424_INT1]);
	 sm5424_read_byte(SM5424_INT2, &int_value[SM5424_INT2]);
	 sm5424_read_byte(SM5424_INT3, &int_value[SM5424_INT3]);

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

static int sm5424_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	 struct sm5424_info *info;
	 int ret = 0; 

	 battery_log(BAT_LOG_CRTI, "[sm5424_driver_probe] START\n");

	 info = (struct sm5424_info*)kzalloc(sizeof(*info), GFP_KERNEL);
	 if (!info) {
		 printk("[%s] sm5424_info memory allocation failed.\n",__func__);
		 return -ENOMEM;
	 }
	 
	 /* initialize device info */
	 info->i2c = client;
	 info->pdev = &client->dev;

	 new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);

	 if (!new_client) {
		 ret = -ENOMEM;
		 goto exit;
	 }
	 memset(new_client, 0, sizeof(struct i2c_client));

	 new_client = client;

	 /* read device tree */
	 if (client->dev.of_node)
	 {			 
		 ret = sm5424_parse_dt(&client->dev, info);
		 if (ret) {			 
			 printk("[%s] cannot read from fdt.\n",__func__);
			 return ret;
		 }
	 }

	 /* --------------------- */
	 sm5424_hw_component_detect();	 	 
	 
	 sm5424_charger_initialize();

	 ret = sm5424_irq_init(info);
	 if (ret < 0) {
		 battery_log(BAT_LOG_CRTI, "[%s] Error : can't initialize SM5424 irq.\n",__func__);
		 goto exit;
	 }
	 
	 sm5424_dump_register();
	 chargin_hw_init_done = KAL_TRUE;

	 battery_log(BAT_LOG_CRTI, "[sm5424_driver_probe] DONE\n");

	 return 0;

exit:
	 return ret;

}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_sm5424 = 0;
static ssize_t show_sm5424_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	 battery_log(BAT_LOG_CRTI, "[show_sm5424_access] 0x%x\n", g_reg_value_sm5424);
	 return sprintf(buf, "%u\n", g_reg_value_sm5424);
}

static ssize_t store_sm5424_access(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	 int ret = 0;
	 char *pvalue = NULL, *addr, *val;
	 unsigned int reg_value = 0;
	 unsigned int reg_address = 0;

	 battery_log(BAT_LOG_CRTI, "[store_sm5424_access]\n");

	 if (buf != NULL && size != 0) {
		 battery_log(BAT_LOG_CRTI, "[store_sm5424_access] buf is %s and size is %zu\n", buf, size);
		 /*reg_address = kstrtoul(buf, 16, &pvalue);*/

		 pvalue = (char *)buf;
		 if (size > 3) {
			 addr = strsep(&pvalue, " ");
			 ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		 } else
			 ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);

		 if (size > 3) {
			 val = strsep(&pvalue, " ");
			 ret = kstrtou32(val, 16, (unsigned int *)&reg_value);

			 battery_log(BAT_LOG_CRTI,
				 "[store_sm5424_access] write sm5424 reg 0x%x with value 0x%x !\n",
				  reg_address, reg_value);
			 ret = sm5424_config_interface(reg_address, reg_value, 0xFF, 0x0);
		 } else {
			 ret = sm5424_read_interface(reg_address, &g_reg_value_sm5424, 0xFF, 0x0);
			 battery_log(BAT_LOG_CRTI,
				 "[store_sm5424_access] read sm5424 reg 0x%x with value 0x%x !\n",
				  reg_address, g_reg_value_sm5424);
			 battery_log(BAT_LOG_CRTI,
				 "[store_sm5424_access] Please use \"cat sm5424_access\" to get value\r\n");
		 }
	 }
	 return size;
}

static DEVICE_ATTR(sm5424_access, 0664, show_sm5424_access, store_sm5424_access);	 /* 664 */

static int sm5424_user_space_probe(struct platform_device *dev)
{
	 int ret_device_file = 0;

	 battery_log(BAT_LOG_CRTI, "******** sm5424_user_space_probe!! ********\n");

	 ret_device_file = device_create_file(&(dev->dev), &dev_attr_sm5424_access);

	 return 0;
}

struct platform_device sm5424_user_space_device = {
	 .name = "sm5424-user",
	 .id = -1,
};

static const struct of_device_id sm5424_platform_of_ids[] = {
	 {.compatible = "mediatek,mt6735-sm5424",},
	 {}
};

static struct platform_driver sm5424_user_space_driver = {
	 .probe = sm5424_user_space_probe,
	 .driver = {
			.name = "sm5424-user",
#ifdef CONFIG_OF
	 		.of_match_table = sm5424_platform_of_ids,
#endif					
			},		
};

static int __init sm5424_subsys_init(void)
{
	 int ret = 0;

	 if (i2c_add_driver(&sm5424_driver) != 0)
		 battery_log(BAT_LOG_CRTI, "[sm5424_init] failed to register sm5424 i2c driver.\n");
	 else
		 battery_log(BAT_LOG_CRTI, "[sm5424_init] Success to register sm5424 i2c driver.\n");

	 /* sm5424 user space access interface */
	 ret = platform_device_register(&sm5424_user_space_device);
	 if (ret) {
		 battery_log(BAT_LOG_CRTI, "****[sm5424_init] Unable to device register(%d)\n", ret);
		 return ret;
	 }
	 ret = platform_driver_register(&sm5424_user_space_driver);
	 if (ret) {
		 battery_log(BAT_LOG_CRTI, "****[sm5424_init] Unable to register driver (%d)\n", ret);
		 return ret;
	 }

	 return 0;
}

static void __exit sm5424_exit(void)
{
	 i2c_del_driver(&sm5424_driver);
}

//module_init(sm5424_init);
module_exit(sm5424_exit);
subsys_initcall(sm5424_subsys_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C sm5424 Driver"); 
