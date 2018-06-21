#include <platform/mt_typedefs.h>
#include <platform/mt_reg_base.h>
#include <platform/mt_i2c.h>   
#include <platform/mt_pmic.h>
#include <platform/sm5424.h>
#include <printf.h>
#include <cust_gpio_usage.h>
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>

#if !defined(CONFIG_POWER_EXT)
#include <platform/upmu_common.h>
#endif

int g_sm5424_log_en=2;

/**********************************************************
  *
  *   [I2C Slave Setting] 
  *
  *********************************************************/
#define sm5424_SLAVE_ADDR_WRITE   0x92
#define sm5424_SLAVE_ADDR_READ    0x93
#define PRECC_BATVOL 2000
/**********************************************************
  *
  *   [Global Variable] 
  *
  *********************************************************/
kal_uint8 sm5424_reg[SM5424_REG_NUM] = {0};

/**********************************************************
  *
  *   [I2C Function For Read/Write bq24158] 
  *
  *********************************************************/
#define SM5424_I2C_ID	I2C1
static struct mt_i2c_t sm5424_i2c;

kal_uint32 sm5424_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    sm5424_i2c.id = SM5424_I2C_ID;
    /* Since i2c will left shift 1 bit, we need to set SM5424 I2C address to >>1 */
    sm5424_i2c.addr = (sm5424_SLAVE_ADDR_WRITE >> 1);
    sm5424_i2c.mode = ST_MODE;
    sm5424_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&sm5424_i2c, write_data, len);

    if(I2C_OK != ret_code)
        dprintf(INFO, "%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}

kal_uint32 sm5424_read_byte (kal_uint8 addr, kal_uint8 *dataBuffer) 
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint16 len;
    *dataBuffer = addr;

    sm5424_i2c.id = SM5424_I2C_ID;
    /* Since i2c will left shift 1 bit, we need to set SM5424 I2C address to >>1 */
    sm5424_i2c.addr = (sm5424_SLAVE_ADDR_READ >> 1);
    sm5424_i2c.mode = ST_MODE;
    sm5424_i2c.speed = 100;
    len = 1;

    ret_code = i2c_write_read(&sm5424_i2c, dataBuffer, len, len);

    if(I2C_OK != ret_code)
        dprintf(INFO, "%s: i2c_read: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}

kal_uint32 sm5424_read_interface (kal_uint8 RegNum, kal_uint8 *val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 sm5424_reg = 0;
    int ret = 0;

    dprintf(INFO, "--------------------------------------------------LK\n");

    ret = sm5424_read_byte(RegNum, &sm5424_reg);
    dprintf(INFO, "[sm5424_read_interface] Reg[%x]=0x%x\n", RegNum, sm5424_reg);

    sm5424_reg &= (MASK << SHIFT);
    *val = (sm5424_reg >> SHIFT);	  

    if(g_sm5424_log_en>1)		  
	    dprintf(INFO, "%d\n", ret);

    return ret;
}

kal_uint32 sm5424_config_interface (kal_uint8 RegNum, kal_uint8 val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 sm5424_reg = 0;
    kal_uint32 ret = 0;

    dprintf(INFO, "--------------------------------------------------LK\n");

    ret = sm5424_read_byte(RegNum, &sm5424_reg);

    dprintf(INFO, "[sm5424_config_interface] Reg[%x]=0x%x\n", RegNum, sm5424_reg);

    sm5424_reg &= ~(MASK << SHIFT);
    sm5424_reg |= (val << SHIFT);

    ret = sm5424_write_byte(RegNum, sm5424_reg);

    dprintf(INFO, "[sm5424_config_interface] write Reg[%x]=0x%x\n", RegNum, sm5424_reg);

    // Check
    //sm5424_read_byte(RegNum, &sm5424_reg);
    //dprintf(INFO, "[sm5424_config_interface] Check Reg[%x]=0x%x\n", RegNum, sm5424_reg);

    if(g_sm5424_log_en>1)        
        dprintf(INFO, "%d\n", ret);

    return ret;
}

/**********************************************************
  *
  *   [Read / Write Function] 
  *
  *********************************************************/
/* STATUS1---------------------------------------------------- */
kal_uint32 sm5424_get_status1(void)
{
	kal_uint8 val=0;
	kal_uint32 ret=0; 
  
	ret=sm5424_read_byte(SM5424_STATUS1,&val);

	if(g_sm5424_log_en>1)        
		dprintf(CRITICAL, "%d\n", ret);
	
	return val;
}

/* STATUS2---------------------------------------------------- */
kal_uint32 sm5424_get_status2(void)
{
	kal_uint8 val=0;
	kal_uint32 ret=0; 

	ret=sm5424_read_byte(SM5424_STATUS2,&val);

	if(g_sm5424_log_en>1)        
		dprintf(CRITICAL, "%d\n", ret);	

	return val;
}
 
/* STATUS3---------------------------------------------------- */
kal_uint32 sm5424_get_status3(void)
{
	kal_uint8 val=0;
	kal_uint32 ret=0; 

	ret=sm5424_read_byte(SM5424_STATUS3, &val);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 

	return val;
}

/* CNTL---------------------------------------------------- */ 
void sm5424_set_nENZCS(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CNTL),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CNTL_nENZCS_MASK),
									(kal_uint8) (SM5424_CNTL_nENZCS_SHIFT)
		);
	
	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

void sm5424_set_ENI2CRESET(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CNTL),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CNTL_ENI2CRESET_MASK),
									(kal_uint8) (SM5424_CNTL_ENI2CRESET_SHIFT)
		);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
} 

void sm5424_set_SUSPEND(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CNTL),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CNTL_SUSPEND_MASK),
									(kal_uint8) (SM5424_CNTL_SUSPEND_SHIFT)
									);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
} 

void sm5424_set_RESET(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CNTL),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CNTL_RESET_MASK),
									(kal_uint8) (SM5424_CNTL_RESET_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
} 

void sm5424_set_ENBOOST(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CNTL),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CNTL_ENBOOST_MASK),
									(kal_uint8) (SM5424_CNTL_ENBOOST_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}	

void sm5424_set_AUTOSTOP(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CNTL),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CNTL_AUTOSTOP_MASK),
									(kal_uint8) (SM5424_CNTL_AUTOSTOP_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}	

void sm5424_set_CHGEN(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CNTL),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CNTL_CHGEN_MASK),
									(kal_uint8) (SM5424_CNTL_CHGEN_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}	
/* VBUSCNTL---------------------------------------------------- */
void sm5424_set_VBUSLIMIT(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_VBUSCNTL),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_VBUSCNTL_VBUSLIMIT_MASK),
									(kal_uint8) (SM5424_VBUSCNTL_VBUSLIMIT_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

/* CHGCNTL1---------------------------------------------------- */
void sm5424_set_AICLTH(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL1),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL1_AICLTH_MASK), 
									(kal_uint8) (SM5424_CHGCNTL1_AICLTH_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

void sm5424_set_AICLEN(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL1),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL1_AICLEN_MASK), 
									(kal_uint8) (SM5424_CHGCNTL1_AICLEN_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

void sm5424_set_DISLIMIT(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL1),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL1_DISLIMIT_MASK), 
									(kal_uint8) (SM5424_CHGCNTL1_DISLIMIT_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

void sm5424_set_PRECHG(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL1),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL1_PRECHG_MASK), 
									(kal_uint8) (SM5424_CHGCNTL1_PRECHG_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

/* CHGCNTL2---------------------------------------------------- */
void sm5424_set_FASTCHG(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL2),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL2_FASTCHG_MASK), 
									(kal_uint8) (SM5424_CHGCNTL2_FASTCHG_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

/* CHGCNTL3---------------------------------------------------- */
void sm5424_set_BATREG(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL3),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL3_BATREG_MASK), 
									(kal_uint8) (SM5424_CHGCNTL3_BATREG_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

/* CHGCNTL4---------------------------------------------------- */

void sm5424_set_Q2LIM(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL4),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL4_Q2LIM_MASK), 
									(kal_uint8) (SM5424_CHGCNTL4_Q2LIM_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

void sm5424_set_FREQSEL(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL4),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL4_FREQSEL_MASK), 
									(kal_uint8) (SM5424_CHGCNTL4_FREQSEL_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

void sm5424_set_TOPOFF(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL4),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL4_TOPOFF_MASK), 
									(kal_uint8) (SM5424_CHGCNTL4_TOPOFF_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

/* CHGCNTL5---------------------------------------------------- */
void sm5424_set_TOPOFFTIMER(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL5),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL5_TOPOFFTIMER_MASK), 
									(kal_uint8) (SM5424_CHGCNTL5_TOPOFFTIMER_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

void sm5424_set_FASTTIMER(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL5),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL5_FASTTIMER_MASK), 
									(kal_uint8) (SM5424_CHGCNTL5_FASTTIMER_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

void sm5424_set_OTGCURRENT(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL5),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL5_OTGCURRENT_MASK), 
									(kal_uint8) (SM5424_CHGCNTL5_OTGCURRENT_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

void sm5424_set_BST_IQ3LIMIT(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL5),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL5_BST_IQ3LIMIT_MASK), 
									(kal_uint8) (SM5424_CHGCNTL5_BST_IQ3LIMIT_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

/* CHGCNTL6---------------------------------------------------- */
void sm5424_set_VOTG(kal_uint32 val)
{
	kal_uint32 ret = 0;

	ret = sm5424_config_interface((kal_uint8) (SM5424_CHGCNTL6),
									(kal_uint8) (val),
									(kal_uint8) (SM5424_CHGCNTL6_VOTG_MASK), 
									(kal_uint8) (SM5424_CHGCNTL6_VOTG_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 
}

/* DEVICEID---------------------------------------------------- */
kal_uint32 sm5424_get_REVISIONID(void)
{
	kal_uint8 val=0;
	kal_uint32 ret=0; 

	ret=sm5424_read_interface((kal_uint8)(SM5424_DEVICEID), 
								(&val),
								(kal_uint8)(SM5424_DEVICEID_REVISIONID_MASK),
								(kal_uint8)(SM5424_DEVICEID_REVISIONID_SHIFT)
	);

	if(g_sm5424_log_en>1)		  
		dprintf(CRITICAL, "%d\n", ret); 

	return val;
}

/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
void sm5424_dump_register(void)
{
    int i=0;

    dprintf(CRITICAL, "sm5424_dump_register\r\n");
    for (i = SM5424_INTMASK1; i < SM5424_CHGCNTL6+1; i++)
    {
        sm5424_read_byte(i, &sm5424_reg[i]);
        dprintf(INFO, "[0x%x]=0x%x\r\n", i, sm5424_reg[i]);		  
    }
    sm5424_read_byte(i, &sm5424_reg[SM5424_CHGCNTL6+1]);
	dprintf(INFO, "[0x%x]=0x%x\r\n", i, sm5424_reg[SM5424_CHGCNTL6+1]); 	  	
}

void sm5424_hw_init(void)
{
    //pull CE low
    int reg_data = 0;

    printf("[sm5424] ChargerHwInit_sm5424\n" );

#if defined(GPIO_SWCHARGER_EN_PIN)
	//Enable Charger
    mt_set_gpio_mode(GPIO_SWCHARGER_EN_PIN,GPIO_MODE_GPIO);  
    mt_set_gpio_dir(GPIO_SWCHARGER_EN_PIN,GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_SWCHARGER_EN_PIN,GPIO_OUT_ZERO); 	
#endif	
	sm5424_set_CHGEN(CHARGE_EN);
    
    //You can customize INTMSK1/2/3 for your system
    //INTMSK1
    reg_data = 0xFF;
    sm5424_write_byte(SM5424_INTMASK1, &reg_data);
    //INTMSK2
    reg_data = 0xFF;
    sm5424_write_byte(SM5424_INTMASK2, &reg_data);
    //INTMSK3
    reg_data = 0xFF;
    sm5424_write_byte(SM5424_INTMASK3, &reg_data);

	sm5424_set_AICLTH(AICL_THRESHOLD_4_5_V);
	sm5424_set_AICLEN(AICL_EN);
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)	
	sm5424_set_BATREG(BATREG_4400mV);
#else
	sm5424_set_BATREG(BATREG_4400mV);
#endif
	sm5424_set_BST_IQ3LIMIT(BSTIQ3LIMIT_2P0A);
	sm5424_set_TOPOFF(TOPOFF_100mA);
#if defined(SM5424_TOPOFF_TIMER_SUPPORT)
	sm5424_set_AUTOSTOP(AUTOSTOP_EN); 
	sm5424_set_TOPOFFTIMER(TOPOFFTIMER_45MIN);	
#else
	sm5424_set_AUTOSTOP(AUTOSTOP_DIS); 
#endif

	sm5424_dump_register();

}

static CHARGER_TYPE g_chr_type_num = CHARGER_UNKNOWN;
int hw_charging_get_charger_type(void);

void sm5424_charging_enable(kal_uint32 bEnable)
{
    int temp_CC_value = 0;
    kal_int32 bat_val = 0;

    if(CHARGER_UNKNOWN == g_chr_type_num && KAL_TRUE == upmu_is_chr_det())
    {
        hw_charging_get_charger_type();
        dprintf(CRITICAL, "[BATTERY:sm5424] charger type: %d\n", g_chr_type_num);
    }

    bat_val = get_i_sense_volt(1);
    if (g_chr_type_num == STANDARD_CHARGER)
    {
        temp_CC_value = 2000;
        sm5424_set_VBUSLIMIT(VBUSLIMIT_2000mA); //IN current limit at 2A
        if(bat_val < PRECC_BATVOL)
            sm5424_set_PRECHG(PRECHG_450mA);  //Pre-Charging Current Limit at 500ma ->450mA
        else
            sm5424_set_FASTCHG(FASTCHG_2000mA);  //Fast Charging Current Limit at 2A
    }
    else if (g_chr_type_num == STANDARD_HOST \
        || g_chr_type_num == CHARGING_HOST \
        || g_chr_type_num == NONSTANDARD_CHARGER)
    {
        temp_CC_value = 450;
        sm5424_set_VBUSLIMIT(VBUSLIMIT_450mA); //IN current limit at 500mA
        sm5424_set_FASTCHG(FASTCHG_450mA);  //Fast Charging Current Limit at 500mA
    }
    else
    {
        dprintf(INFO, "[BATTERY:sm5424] Unknown charger type\n");
        temp_CC_value = 450;
        sm5424_set_VBUSLIMIT(VBUSLIMIT_450mA); //IN current limit at 500mA
        sm5424_set_FASTCHG(FASTCHG_450mA);  //Fast Charging Current Limit at 500mA
    }     	

    if(KAL_TRUE == bEnable)
    {
#if defined(GPIO_SWCHARGER_EN_PIN)    
		mt_set_gpio_mode(GPIO_SWCHARGER_EN_PIN,GPIO_MODE_GPIO);  
		mt_set_gpio_dir(GPIO_SWCHARGER_EN_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_SWCHARGER_EN_PIN,GPIO_OUT_ZERO);
#endif
		sm5424_set_CHGEN(CHARGE_EN);
    }
    else
    {
#if defined(GPIO_SWCHARGER_EN_PIN)    
		mt_set_gpio_mode(GPIO_SWCHARGER_EN_PIN,GPIO_MODE_GPIO);  
		mt_set_gpio_dir(GPIO_SWCHARGER_EN_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_SWCHARGER_EN_PIN,GPIO_OUT_ONE);    
#endif		
		sm5424_set_CHGEN(CHARGE_DIS);
    }

    dprintf(INFO, "[BATTERY:sm5424] sm5424_set_ac_current(), CC value(%dmA) \r\n", temp_CC_value);
    dprintf(INFO, "[BATTERY:sm5424] charger enable/disable %d !\r\n", bEnable);
}

#if defined(CONFIG_POWER_EXT)

int hw_charging_get_charger_type(void)
{
    g_chr_type_num = STANDARD_HOST;

    return STANDARD_HOST;
}

#else

extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern void mdelay (unsigned long msec);
extern kal_uint16 bc11_set_register_value(PMU_FLAGS_LIST_ENUM flagname,kal_uint32 val);

static void hw_bc11_dump_register(void)
{
/*
	kal_uint32 reg_val = 0;
	kal_uint32 reg_num = CHR_CON18;
	kal_uint32 i = 0;

	for(i=reg_num ; i<=CHR_CON19 ; i+=2)
	{
		reg_val = upmu_get_reg_value(i);
		dprintf(INFO, "Chr Reg[0x%x]=0x%x \r\n", i, reg_val);
	}	
*/
}

static void hw_bc11_init(void)
{
    mdelay(200);
    Charger_Detect_Init();

    //RG_bc11_BIAS_EN=1
    bc11_set_register_value(PMIC_RG_BC11_BIAS_EN,1);
    //RG_bc11_VSRC_EN[1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0);
    //RG_bc11_VREF_VTH = [1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0);
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0);
    //RG_bc11_IPU_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0);
    //RG_bc11_IPD_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0);
    //bc11_RST=1
    bc11_set_register_value(PMIC_RG_BC11_RST,1);
    //bc11_BB_CTRL=1
    bc11_set_register_value(PMIC_RG_BC11_BB_CTRL,1);

    mdelay(50);

    if(g_sm5424_log_en>1)
    {
        dprintf(INFO, "hw_bc11_init() \r\n");
        hw_bc11_dump_register();
    }	
}
 
static U32 hw_bc11_DCD(void)
{
    U32 wChargerAvail = 0;

    //RG_bc11_IPU_EN[1.0] = 10
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x2);  
    //RG_bc11_IPD_EN[1.0] = 01
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x1);
    //RG_bc11_VREF_VTH = [1:0]=01
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x1);
    //RG_bc11_CMP_EN[1.0] = 10
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x2);

    mdelay(80);

    wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

    if(g_sm5424_log_en>1)
    {
        dprintf(INFO, "hw_bc11_DCD() \r\n");
        hw_bc11_dump_register();
    }

    //RG_bc11_IPU_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0);
    //RG_bc11_IPD_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);
    //RG_bc11_VREF_VTH = [1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);

    return wChargerAvail;
}

static U32 hw_bc11_stepA1(void)
{
    U32 wChargerAvail = 0;
      
    //RG_bc11_IPD_EN[1.0] = 01
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x1);
    //RG_bc11_VREF_VTH = [1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
    //RG_bc11_CMP_EN[1.0] = 01
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);

    mdelay(80);

    wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

    if(g_sm5424_log_en>1)
    {
    	dprintf(INFO, "hw_bc11_stepA1() \r\n");
    	hw_bc11_dump_register();
    }

    //RG_bc11_IPD_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);

    return  wChargerAvail;
}


static U32 hw_bc11_stepA2(void)
{
    U32 wChargerAvail = 0;

    //RG_bc11_VSRC_EN[1.0] = 10 
    bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x2);
    //RG_bc11_IPD_EN[1:0] = 01
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x1);
    //RG_bc11_VREF_VTH = [1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
    //RG_bc11_CMP_EN[1.0] = 01
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);

    mdelay(80);

    wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

    if(g_sm5424_log_en)
    {
    	dprintf(INFO, "hw_bc11_stepB1() \r\n");
    	hw_bc11_dump_register();
    }

    //RG_bc11_VSRC_EN[1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x0);
    //RG_bc11_IPD_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);

    return  wChargerAvail;
}


static U32 hw_bc11_stepB2(void)
{
    U32 wChargerAvail = 0;
      
    //RG_bc11_IPU_EN[1:0]=10
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x2); 
    //RG_bc11_VREF_VTH = [1:0]=01
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x1);
    //RG_bc11_CMP_EN[1.0] = 01
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);

    mdelay(80);

    wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

    if(g_sm5424_log_en)
    {
    	dprintf(INFO, "hw_bc11_stepB2() \r\n");
    	hw_bc11_dump_register();
    }

    if (!wChargerAvail)
    {
        //RG_bc11_VSRC_EN[1.0] = 10 
        //mt6325_upmu_set_rg_bc11_vsrc_en(0x2);
        bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x2); 
    }
    //RG_bc11_IPU_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0); 
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0); 
    //RG_bc11_VREF_VTH = [1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0); 

    return  wChargerAvail;
}


static void hw_bc11_done(void)
{
    //RG_bc11_VSRC_EN[1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x0);
    //RG_bc11_VREF_VTH = [1:0]=0
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);
    //RG_bc11_IPU_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0);
    //RG_bc11_IPD_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
    //RG_bc11_BIAS_EN=0
    bc11_set_register_value(PMIC_RG_BC11_BIAS_EN,0x0);

    Charger_Detect_Release();

    if(g_sm5424_log_en>1)
    {
    	dprintf(INFO, "hw_bc11_done() \r\n");
    	hw_bc11_dump_register();
    }
}

int hw_charging_get_charger_type(void)
{
    if(CHARGER_UNKNOWN != g_chr_type_num)
        return g_chr_type_num;
#if 0
	  return STANDARD_HOST;
#else
    CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;
    
    /********* Step initial  ***************/         
    hw_bc11_init();
 
    /********* Step DCD ***************/  
    if(1 == hw_bc11_DCD())
    {
         /********* Step A1 ***************/
         if(1 == hw_bc11_stepA1())
         {             
             CHR_Type_num = APPLE_2_1A_CHARGER;
             dprintf(INFO, "step A1 : Apple 2.1A CHARGER!\r\n");
         }
         else
         {
             CHR_Type_num = NONSTANDARD_CHARGER;
             dprintf(INFO, "step A1 : Non STANDARD CHARGER!\r\n");
         }
    }
    else
    {
         /********* Step A2 ***************/
         if(1 == hw_bc11_stepA2())
         {
             /********* Step B2 ***************/
             if(1 == hw_bc11_stepB2())
             {
                 CHR_Type_num = STANDARD_CHARGER;
                 dprintf(INFO, "step B2 : STANDARD CHARGER!\r\n");
             }
             else
             {
                 CHR_Type_num = CHARGING_HOST;
                 dprintf(INFO, "step B2 :  Charging Host!\r\n");
             }
         }
         else
         {
             CHR_Type_num = STANDARD_HOST;
             dprintf(INFO, "step A2 : Standard USB Host!\r\n");
         }
    }
 
    /********* Finally setting *******************************/
    hw_bc11_done();

    g_chr_type_num = CHR_Type_num;

    return g_chr_type_num;
#endif    
}
#endif
