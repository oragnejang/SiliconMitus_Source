#include <linux/types.h>
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>

#include <linux/delay.h>
#include <linux/reboot.h>
#include <mt-plat/mt_boot.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#include <linux/power_supply.h>
#include <mt-plat/battery_common.h>
#ifdef CONFIG_MACH_LGE
#ifdef CONFIG_LGE_PM_AT_CMD_SUPPORT
#include <mach/board_lge.h>
#endif
#endif

// ============================================================ //
// define
// ============================================================ //
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

// ============================================================ //
// global variable
// ============================================================ //
kal_bool chargin_hw_init_done = KAL_FALSE;


// ============================================================ //
// internal variable
// ============================================================ //
static char* support_charger[] = {
	"sm5424",
};
static struct power_supply *external_charger = NULL;

static kal_bool charging_type_det_done = KAL_TRUE;
static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;

static unsigned int g_charging_enabled = 0;
static unsigned int g_charging_current = 0;
static unsigned int g_charging_current_limit = 0;
static unsigned int g_charging_voltage = 0;
static unsigned int g_charging_setting_chagned = 0;

const unsigned int VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	    BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V,
	    BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V,
	    BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	    BATTERY_VOLT_10_500000_V
};

// ============================================================ //
// extern variable
// ============================================================ //

// ============================================================ //
// extern function
// ============================================================ //
extern unsigned int upmu_get_reg_value(unsigned int reg);
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern void mt_power_off(void);

/* Internal APIs for PowerSupply */
static int is_property_support(enum power_supply_property prop)
{
	int support = 0;
	int i;

	if (!external_charger)
		return 0;

	if (!external_charger->get_property)
		return 0;

	for(i = 0; i < external_charger->num_properties; i++) {
		if (external_charger->properties[i] == prop) {
			support = 1;
			break;
		}
	}

	return support;
}

static int is_property_writeable(enum power_supply_property prop)
{
	if (!external_charger->set_property)
		return 0;

	if (!external_charger->property_is_writeable)
		return 0;

	return external_charger->property_is_writeable(external_charger, prop);
}

static int get_property(enum power_supply_property prop, int *data)
{
	union power_supply_propval val;
	int rc = 0;

	*(int*)data = STATUS_UNSUPPORTED;

	if(!is_property_support(prop))
		return 0;

	rc = external_charger->get_property(external_charger, prop, &val);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] failed to get property %d\n", prop);
		*(int*)data = 0;
		return rc;
	}

	*(int*)data = val.intval;

	battery_log(BAT_LOG_FULL, "[CHARGER] set property %d to %d\n", prop, val.intval);
	return rc;
}

static int set_property(enum power_supply_property prop, int data)
{
	union power_supply_propval val;
	int rc = 0;

	if (!is_property_writeable(prop))
		return 0;

	val.intval = data;
	rc = external_charger->set_property(external_charger, prop, &val);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] failed to set property %d\n", prop);
	}
	battery_log(BAT_LOG_FULL, "[CHARGER] set property %d to %d\n", prop, data);
	return rc;
}

static u32 charging_parameter_to_value(const u32 *parameter, const u32 array_size, const u32 val)
{
	u32 i;

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	battery_log(BAT_LOG_CRTI, "NO register value match. val=%d\r\n", val);
	/* TODO: ASSERT(0);      // not find the value */
	return 0;
}

static u32 bmt_find_closest_level(const u32 *pList, u32 number, u32 level)
{
	u32 i;
	u32 max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = true;
	else
		max_value_in_last_element = false;

	if (max_value_in_last_element == true) {
		for (i = (number - 1); i != 0; i--) {	/* max value in the last element */
			if (pList[i] <= level)
				return pList[i];
		}

		battery_log(BAT_LOG_CRTI, "Can't find closest level, small value first \r\n");
		return pList[0];
	}

	for (i = 0; i < number; i++) {	/* max value in the first element */
		if (pList[i] <= level)
			return pList[i];
	}

	battery_log(BAT_LOG_CRTI, "Can't find closest level, large value first \r\n");
	return pList[number - 1];
}

/* Charger Control Interface Handler */
static unsigned int charging_hw_init(void *data)
{
	static int hw_initialized = 0;

	if (hw_initialized)
		return STATUS_OK;

	bc11_set_register_value(PMIC_RG_USBDL_SET,0x0);//force leave USBDL mode
	bc11_set_register_value(PMIC_RG_USBDL_RST,0x1);//force leave USBDL mode

	hw_initialized = 1;
	battery_log(BAT_LOG_FULL, "[CHARGER] initialized.\n");

	return STATUS_OK;
}

static unsigned int charging_dump_register(void *data)
{
	/* nothing to do */
	return STATUS_OK;
}

static unsigned int charging_enable(void *data)
{
	int status;
	int enable = *(int*)(data);
	int rc = 0;

	if (enable == g_charging_enabled && !g_charging_setting_chagned)
		return STATUS_OK;

	/* Do not disable charging when battery disconnected */
	if (!enable && bc11_get_register_value(PMIC_RGS_BATON_UNDET))
		return STATUS_OK;

	if (enable) {
		status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	rc = set_property(POWER_SUPPLY_PROP_STATUS, status);
	if (rc) {
		battery_log(BAT_LOG_CRTI,
			"[CHARGER] failed to %s charging.(%d)\n",
			(enable ? "start" : "stop"), rc);
		return STATUS_UNSUPPORTED;
	}

	g_charging_enabled = enable;

	/* clear charging setting */
	if (!g_charging_enabled) {
		g_charging_current = 0;
		g_charging_current_limit = 0;
		g_charging_voltage = 0;
	}

	g_charging_setting_chagned = 0;

	battery_log(BAT_LOG_CRTI, "[CHARGER] %s charging.\n",
				(g_charging_enabled ? "start" : "stop"));

	return STATUS_OK;
}

static unsigned int charging_set_cv_voltage(void *data)
{
	int voltage = *(int*)(data);
	int rc = 0;

	if (voltage == g_charging_voltage)
		return STATUS_OK;

	rc = set_property(POWER_SUPPLY_PROP_VOLTAGE_MAX, voltage);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] Set CV Voltage failed.(%d)\n", rc);
		return STATUS_UNSUPPORTED;
	}

	g_charging_voltage = voltage;
	g_charging_setting_chagned = 1;

	battery_log(BAT_LOG_CRTI, "[CHARGER] Set CV Voltage to %duV\n", g_charging_voltage);

	return STATUS_OK;
}

static unsigned int charging_get_current(void *data)
{
	int cur;
	int rc = 0;

	if (!g_charging_enabled) {
		*(int*)data = 0;
		return STATUS_OK;
	}

	rc = get_property(POWER_SUPPLY_PROP_CURRENT_NOW, &cur);
	if (rc)
		*(int*)data = min(g_charging_current, g_charging_current_limit);
	else if (cur < 0)
		*(int*)data = min(g_charging_current, g_charging_current_limit);
	else
		*(int*)data = cur;

	/* match unit with CHR_CURRENT_ENUM */
	*(int*)data *= 100;

	return STATUS_OK;
}

static unsigned int charging_set_current(void *data)
{
	enum power_supply_property prop = POWER_SUPPLY_PROP_CURRENT_NOW;
	int cur = *(int*)(data);
	int rc = 0;

	/* convert unit to mA */
	cur = cur / 100;

	/* charging current & current limit is not separated */
	if (!is_property_writeable(POWER_SUPPLY_PROP_CURRENT_NOW)) {
		prop = POWER_SUPPLY_PROP_CURRENT_MAX;

		/* use current limit value here */
		if (cur > g_charging_current_limit) {
			cur = g_charging_current_limit;
		}
	}

	if (cur == g_charging_current)
		return STATUS_OK;

	rc = set_property(prop, cur);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] Set Current failed.(%d)\n", rc);
		return STATUS_UNSUPPORTED;
	}

	g_charging_current = cur;
	g_charging_setting_chagned = 1;

	battery_log(BAT_LOG_CRTI, "[CHARGER] Set Current to %dmA\n", g_charging_current);

	return STATUS_OK;
}

static unsigned int charging_set_input_current(void *data)
{
	int cur = *(int*)(data);
	int rc = 0;

	/* convert unit to mA */
	cur = cur / 100;

	if (cur == g_charging_current_limit)
		return STATUS_OK;

	rc = set_property(POWER_SUPPLY_PROP_CURRENT_MAX, cur);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] Set Current Limit failed.(%d)\n", rc);
		return STATUS_UNSUPPORTED;
	}

	g_charging_current_limit = cur;
	g_charging_setting_chagned = 1;

	battery_log(BAT_LOG_CRTI, "[CHARGER] Set Current Limit to %dmA\n", g_charging_current_limit);

	return STATUS_OK;
}

static unsigned int charging_get_charging_status(void *data)
{
	int voltage = *(int*)data;
	int status;
	int rc = 0;

	if (g_charger_type == CHARGER_UNKNOWN) {
		*(int*)data = 0;
		return STATUS_OK;
	}

	rc = get_property(POWER_SUPPLY_PROP_STATUS, &status);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] failed to get charging status.(%d)\n", rc);
		return STATUS_UNSUPPORTED;
	}

	/* if eoc check is not supported in charger ic, check battery voltage instead */
	if (status == STATUS_UNSUPPORTED) {
		/* battery voltage is invalid range */
		if (voltage > 5000) {
			*(int*)data = 0;
			return STATUS_OK;
		}

		if (voltage > RECHARGING_VOLTAGE)
			*(int*)data = 1;
		else
			*(int*)data = 0;

		return STATUS_OK;
	}

	if (status == POWER_SUPPLY_STATUS_FULL) {
		*(int*)data = 1;
	} else {
		*(int*)data = 0;
	}
	battery_log(BAT_LOG_FULL, "[CHARGER] End of Charging : %d\n", *(int*)data);

	return STATUS_OK;
}

static unsigned int charging_reset_watch_dog_timer(void *data)
{
	/* nothing to do */
	return STATUS_OK;
}

static unsigned int charging_set_hv_threshold(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int set_hv_voltage;
	unsigned int array_size;
	unsigned short register_value;
	unsigned int voltage = *(unsigned int *) (data);

	array_size = GETARRAYNUM(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size, set_hv_voltage);
	pmic_set_register_value(PMIC_RG_VCDT_HV_VTH, register_value);

	return status;
}

static unsigned int charging_get_hv_status(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;
	pr_notice("[charging_get_hv_status] charger ok for bring up.\n");
#else
	*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
#endif

	return status;
}

static unsigned int charging_get_battery_status(void *data)
{
	bc11_set_register_value(PMIC_BATON_TDET_EN,1);
	bc11_set_register_value(PMIC_RG_BATON_EN,1);

	*(kal_bool*)(data) = bc11_get_register_value(PMIC_RGS_BATON_UNDET);

	return STATUS_OK;
}

static unsigned int charging_get_charger_det_status(void *data)
{
	int online;
	int rc;

	if (bc11_get_register_value(PMIC_RGS_CHRDET) == 0) {
		*(kal_bool*)(data) = KAL_FALSE;
		g_charger_type = CHARGER_UNKNOWN;
	} else {
		*(kal_bool*)(data) = KAL_TRUE;
		rc = get_property(POWER_SUPPLY_PROP_ONLINE, &online);
		if (rc) {
			/* error reading online status. use pmic value */
			battery_log(BAT_LOG_CRTI, "[CHARGER] cannot read online.\n");
			return STATUS_OK;
		}

		if (!online) {
			/* OVP detected in external charger */
			*(kal_bool*)(data) = KAL_FALSE;
			g_charger_type = CHARGER_UNKNOWN;
		}
	}

	battery_log(BAT_LOG_FULL, "[CHARGER] g_charger_type = %d\n", g_charger_type);

	return STATUS_OK;
}

extern int hw_charging_get_charger_type(void);
#ifdef CONFIG_LGE_PM_AT_CMD_SUPPORT
extern int get_usb_type(void);
#endif

static unsigned int charging_get_charger_type(void *data)
{
	unsigned int status = STATUS_OK;
#ifdef CONFIG_LGE_PM_AT_CMD_SUPPORT
	usb_cable_type cable_type;
#endif
#if defined(CONFIG_POWER_EXT)
	*(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else

	if (g_charger_type != CHARGER_UNKNOWN && g_charger_type != WIRELESS_CHARGER)
	{
		*(CHARGER_TYPE*)(data) = g_charger_type;
		battery_log(BAT_LOG_CRTI, "[CHARGER] return %d!\n", g_charger_type);

		return status;
	}

	charging_type_det_done = KAL_FALSE;

	g_charger_type = hw_charging_get_charger_type();

#ifdef CONFIG_LGE_PM
#ifdef CONFIG_LGE_PM_AT_CMD_SUPPORT
	/* Workaround for MITS */
	if (g_charger_type == NONSTANDARD_CHARGER) {
		cable_type = get_usb_type();
		if (cable_type == LT_CABLE_56K ||
			cable_type == LT_CABLE_130K ||
			cable_type == LT_CABLE_910K) {
			g_charger_type = STANDARD_HOST;
		}
	}
#endif
	if (g_charger_type == NONSTANDARD_CHARGER) {
		msleep(500);
		g_charger_type = hw_charging_get_charger_type();
	}
#endif

	charging_type_det_done = KAL_TRUE;

	*(CHARGER_TYPE*)(data) = g_charger_type;
#endif

	return STATUS_OK;
}

static unsigned int charging_get_is_pcm_timer_trigger(void *data)
{
	unsigned int status = STATUS_OK;
#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
	/* skip code */
#else
	if(slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool*)(data) = KAL_TRUE;
	else
	*(kal_bool*)(data) = KAL_FALSE;

	battery_log(BAT_LOG_CRTI, "[CHARGER] slp_get_wake_reason=%d\n",
		slp_get_wake_reason());
#endif
	return status;
}

static unsigned int charging_set_platform_reset(void *data)
{
	battery_log(BAT_LOG_CRTI, "[CHARGER] charging_set_platform_reset\n");

	kernel_restart("battery service reboot system");

	return STATUS_OK;
}

static unsigned int charging_get_platform_boot_mode(void *data)
{
	*(unsigned int*)(data) = get_boot_mode();

	battery_log(BAT_LOG_CRTI, "[CHARGER] get_boot_mode=%d\n", get_boot_mode());

	return STATUS_OK;
}

static unsigned int charging_set_power_off(void *data)
{
	unsigned int status = STATUS_OK;

	battery_log(BAT_LOG_CRTI, "[CHARGER] charging_set_power_off\n");
	kernel_power_off();

	return status;
}

static unsigned int charging_get_power_source(void *data)
{
	unsigned int status = STATUS_OK;

	*(kal_bool *)data = KAL_FALSE;

	return status;
}
static unsigned int charging_get_csdac_full_flag(void *data)
{
	unsigned int status = STATUS_OK;

	*(kal_bool *)data = KAL_FALSE;

	return status;
}
static unsigned int charging_set_ta_current_pattern(void *data)
{
	unsigned int increase = *(unsigned int *) (data);
	unsigned int charging_status = KAL_FALSE;
	BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_200000_V;
	unsigned int fastchg = CHARGE_CURRENT_500_00_MA, vbuslimit = CHARGE_CURRENT_500_00_MA;
	unsigned int chr_en = KAL_FALSE;
	
	if (batt_cust_data.high_battery_voltage_support)
		cv_voltage = BATTERY_VOLT_04_400000_V;

	charging_get_charging_status(&charging_status);
	if (KAL_FALSE == charging_status) {
		charging_set_cv_voltage(&cv_voltage);	/* Set CV */
		fastchg = CHARGE_CURRENT_500_00_MA;
		charging_set_current(&fastchg);	/* Set charging current 500ma */
		charging_enable(&chr_en);	/* Enable Charging */
	}

	if (increase == KAL_TRUE) {
		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		msleep(85);
		
		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 1");
		msleep(85);
		
		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 1");
		msleep(85);

		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 2");
		msleep(85);
		
		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 2");
		msleep(85);
		
		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 3");
		msleep(281);

		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 3");
		msleep(85);

		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 4");
		msleep(281);

		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 4");
		msleep(85);

		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 5");
		msleep(281);

		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 5");
		msleep(85);

		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 6");
		msleep(485);

		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 6");
		msleep(50);

		battery_log(BAT_LOG_CRTI, "mtk_ta_increase() end\n");

		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		msleep(200);
	} else {
		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		msleep(85);

		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 1");
		msleep(281);

		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 1");
		msleep(85);

		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 2");
		msleep(281);

		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 2");
		msleep(85);

		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 3");
		msleep(281);

		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 3");
		msleep(85);

		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 4");
		msleep(85);

		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 4");
		msleep(85);

		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 5");
		msleep(85);

		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 5");
		msleep(85);

		vbuslimit = CHARGE_CURRENT_500_00_MA;
		charging_set_input_current(&vbuslimit);	/* 500mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 6");
		msleep(485);

		vbuslimit = CHARGE_CURRENT_100_00_MA;
		charging_set_input_current(&vbuslimit);	/* 100mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 6");
		msleep(50);
		
		vbuslimit = CHARGE_CURRENT_500_00_MA;
		battery_log(BAT_LOG_CRTI, "mtk_ta_decrease() end\n");
		charging_set_input_current(&vbuslimit);	/* 500mA */
	}

	return STATUS_OK;

}

static unsigned int charging_set_error_state(void *data)
{
	return STATUS_UNSUPPORTED;
}

static unsigned int(*charging_func[CHARGING_CMD_NUMBER]) (void *data);

/*
 * FUNCTION
 *		Internal_chr_control_handler
 *
 * DESCRIPTION
 *		 This function is called to set the charger hw
 *
 * CALLS
 *
 * PARAMETERS
 *		None
 *
 * RETURNS
 *
 *
 * GLOBALS AFFECTED
 *	   None
 */
signed int chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	signed int status;
	int i;

	static int chr_control_interface_init = 0;

	if (!chr_control_interface_init) {
		charging_func[CHARGING_CMD_INIT] = charging_hw_init;
		charging_func[CHARGING_CMD_DUMP_REGISTER] = charging_dump_register;
		charging_func[CHARGING_CMD_ENABLE] = charging_enable;
		charging_func[CHARGING_CMD_SET_CV_VOLTAGE] = charging_set_cv_voltage;
		charging_func[CHARGING_CMD_GET_CURRENT] = charging_get_current;
		charging_func[CHARGING_CMD_SET_CURRENT] = charging_set_current;
		charging_func[CHARGING_CMD_SET_INPUT_CURRENT] = charging_set_input_current;
		charging_func[CHARGING_CMD_GET_CHARGING_STATUS] = charging_get_charging_status;
		charging_func[CHARGING_CMD_RESET_WATCH_DOG_TIMER] = charging_reset_watch_dog_timer;
		charging_func[CHARGING_CMD_SET_HV_THRESHOLD] = charging_set_hv_threshold;
		charging_func[CHARGING_CMD_GET_HV_STATUS] = charging_get_hv_status;
		charging_func[CHARGING_CMD_GET_BATTERY_STATUS] = charging_get_battery_status;
		charging_func[CHARGING_CMD_GET_CHARGER_DET_STATUS] = charging_get_charger_det_status;
		charging_func[CHARGING_CMD_GET_CHARGER_TYPE] = charging_get_charger_type;
		charging_func[CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER] = charging_get_is_pcm_timer_trigger;
		charging_func[CHARGING_CMD_SET_PLATFORM_RESET] = charging_set_platform_reset;
		charging_func[CHARGING_CMD_GET_PLATFORM_BOOT_MODE] = charging_get_platform_boot_mode;
		charging_func[CHARGING_CMD_SET_POWER_OFF] = charging_set_power_off;
		charging_func[CHARGING_CMD_GET_POWER_SOURCE] = charging_get_power_source;
		charging_func[CHARGING_CMD_GET_CSDAC_FALL_FLAG] = charging_get_csdac_full_flag;
		charging_func[CHARGING_CMD_SET_TA_CURRENT_PATTERN] = charging_set_ta_current_pattern;
		charging_func[CHARGING_CMD_SET_ERROR_STATE] = charging_set_error_state;

		chr_control_interface_init = 1;
	}

	switch(cmd) {
	/* these commands does not need external_charger, so jump to do_cmd */
	case CHARGING_CMD_INIT:
	case CHARGING_CMD_RESET_WATCH_DOG_TIMER:
	case CHARGING_CMD_SET_HV_THRESHOLD:
	case CHARGING_CMD_GET_HV_STATUS:
	case CHARGING_CMD_GET_BATTERY_STATUS:
	case CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER:
	case CHARGING_CMD_SET_PLATFORM_RESET:
	case CHARGING_CMD_GET_PLATFORM_BOOT_MODE:
	case CHARGING_CMD_SET_POWER_OFF:
	case CHARGING_CMD_GET_POWER_SOURCE:
	case CHARGING_CMD_GET_CSDAC_FALL_FLAG:
	case CHARGING_CMD_SET_TA_CURRENT_PATTERN:
	case CHARGING_CMD_SET_ERROR_STATE:
		goto chr_control_interface_do_cmd;
		break;
	default:
		break;
	}

	if (!external_charger) {
		/* find charger */
		for (i = 0; i < ARRAY_SIZE(support_charger); i++) {
			external_charger = power_supply_get_by_name(support_charger[i]);
			if (external_charger) {
				battery_log(BAT_LOG_CRTI, "[CHARGER] found charger %s\n",
					support_charger[i]);
				break;
			}
		}

		/* if not found, cannot control charger */
		if (!external_charger) {
			battery_log(BAT_LOG_CRTI, "[CHARGER] failed to get external_charger.\n");
			return STATUS_UNSUPPORTED;
		}

		/* If power source is attached, assume that device is charing now */
		if (bc11_get_register_value(PMIC_RGS_CHRDET)) {
			g_charging_enabled = true;
		}
	}

chr_control_interface_do_cmd:
	status = charging_func[cmd](data);

	return status;
}

