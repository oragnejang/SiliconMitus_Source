
#ifndef __SM5507_H
#define __SM5507_H __FILE__

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/usb_notify.h>
#include <linux/wakelock.h>

#if defined(CONFIG_CCIC_NOTIFIER)
#include <linux/ccic/ccic_notifier.h>
#endif
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
#include <linux/usb/class-dual-role.h>
#endif

#define SM5507_CCIC_DEV_NAME  "ccic-sm5507"
#define CCIC_DEFAULT_UMS_SM5507_FW			"/sdcard/Firmware/usbpd/sm5507_fw.dat"


/* SM5507 register Address */
#define SM5507_REG_INT1	        0x01
#define SM5507_REG_INT2	        0x02
#define SM5507_REG_INT3	        0x03
#define SM5507_REG_INT4	        0x04
#define SM5507_REG_STATUS3      0x0B
#define SM5507_REG_FACTORY      0x0D
#define SM5507_REG_SYS_CTRL	    0x1B
#define SM5507_REG_CC_STATUS    0x20
#define SM5507_REG_PDO_CAP4     0x37
#define SM5507_REG_PDO_RD_REQ   0x46
#define SM5507_REG_PD_REQ	    0x47
#define SM5507_REG_RX_HEADER    0x90
#define SM5507_REG_PL_SEL       0x9F
#define SM5507_REG_TX_PAYLOAD   0xA0
#define SM5507_REG_TX_HEADER    0xBC
#define SM5507_REG_MTP_TMODE    0xE7
#define SM5507_REG_MTP_CTRL     0xE8
#define SM5507_REG_MTP_ADDR0    0xEA
#define SM5507_REG_MTP_ADDR1    0xEB
#define SM5507_REG_MTP_WDATA    0xEC
#define SM5507_REG_MTP_IUM      0xEE



/* PD Request Type */
#define PD_REQ_GET_SRC_CAP  0x03
#define PD_REQ_GET_SNK_CAP  0x04
#define PD_REQ_PR_SWAP      0x05
#define PD_REQ_DR_SWAP      0x06
#define PD_REQ_VCONN_SWAP   0x07

/* SM5507 Interrupt 1 register */
#define INT_WAKEUP_SHIFT	        5
#define INT_DETACH_SHIFT	        4
#define INT_ATTACH_SHIFT		    3
#define INT_ADC_DONE_SHIFT		    2
#define INT_VBUS_OFF_SHIFT		    1
#define INT_VBUSPOK_SHIFT		    0
#define INT_WAKEUP_MASK	            (0x1 << INT_WAKEUP_SHIFT)
#define INT_DETACH_MASK	        	(0x1 << INT_DETACH_SHIFT)
#define INT_ATTACH_MASK		    	(0x1 << INT_ATTACH_SHIFT)
#define INT_ADC_DONE_MASK			(0x1 << INT_ADC_DONE_SHIFT)
#define INT_VBUS_OFF_MASK			(0x1 << INT_VBUS_OFF_SHIFT)
#define INT_VBUSPOK_MASK		    (0x1 << INT_VBUSPOK_SHIFT)

/* SM5507 Interrupt 2 register */
#define INT_MUIC_SHIFT	            4
#define INT_VBUS_REQ_SHIFT	        3
#define INT_PD_INT_SHIFT		    2
#define INT_NON_PD_DEV_SHIFT	    1
#define INT_PD_CONTRACT_SHIFT	    0
#define INT_MUIC_MASK	            (0x1 << INT_MUIC_SHIFT)
#define INT_VBUS_REQ_MASK	       	(0x1 << INT_VBUS_REQ_SHIFT)
#define INT_PD_INT_MASK		    	(0x1 << INT_PD_INT_SHIFT)
#define INT_NON_PD_DEV_MASK			(0x1 << INT_NON_PD_DEV_SHIFT)
#define INT_PD_CONTRACT_MASK		(0x1 << INT_PD_CONTRACT_SHIFT)

/* SM5507 Interrupt 3 register */
#define INT_MTP_BOOT_FAIL_SHIFT	    7
#define INT_MTP_MAIN_FAIL_SHIFT	    6
#define INT_VCONN_OCP_SHIFT		    3
#define INT_CC2_OVP_SHIFT		    2
#define INT_CC1_OVP_SHIFT		    1
#define INT_WATER_SHIFT		        0
#define INT_MTP_BOOT_FAIL_MASK	    (0x1 << INT_MTP_BOOT_FAIL_SHIFT)
#define INT_MTP_MAIN_FAIL_MASK		(0x1 << INT_MTP_MAIN_FAIL_SHIFT)
#define INT_VCONN_OCP_MASK			(0x1 << INT_VCONN_OCP_SHIFT)
#define INT_CC2_OVP_MASK			(0x1 << INT_CC2_OVP_SHIFT)
#define INT_CC1_OVP_MASK			(0x1 << INT_CC1_OVP_SHIFT)
#define INT_WATER_MASK			    (0x1 << INT_WATER_SHIFT)

/* SM5507 Interrupt 4 register */
#define INT_USR_DEF_INT_SHIFT	    2
#define INT_ACC_INT_SHIFT		    1
#define INT_RID_INT_SHIFT		    0
#define INT_USR_DEF_INT_MASK	    (0x1 << INT_USR_DEF_INT_SHIFT)
#define INT_ACC_INT_MASK			(0x1 << INT_ACC_INT_SHIFT)
#define INT_RID_INT_MASK			(0x1 << INT_RID_INT_SHIFT)

struct sm5507_version {
    u8 main;
    u8 boot;
};

typedef struct
{
	uint16_t	Message_Type:4;
	uint16_t	Rsvd2_msg_header:1;
	uint16_t	Port_Data_Role:1;
	uint16_t	Specification_Revision:2;
	uint16_t	Port_Power_Role:1;
	uint16_t	Message_ID:3;
	uint16_t	Number_of_obj:3;
	uint16_t	Rsvd_msg_header:1;
} MSG_HEADER_Type;

typedef struct
{
	uint32_t    Maximum_Allow_Power:10;
	uint32_t    Minimum_Voltage:10;
	uint32_t    Maximum_Voltage:10;
	uint32_t    PDO_Parameter:2;
} SRC_BAT_SUPPLY_Typedef;

// ================== Capabilities Message ==============
// Source Capabilities
typedef struct
{
	uint32_t    Maximum_Current:10;
	uint32_t    Voltage_Unit:10;
	uint32_t    Peak_Current:2;
	uint32_t    Reserved:3;
	uint32_t    Data_Role_Swap:1;
	uint32_t    USB_Comm_Capable:1;
	uint32_t    Externally_POW:1;
	uint32_t    USB_Suspend_Support:1;
	uint32_t    Dual_Role_Power:1;
	uint32_t    PDO_Parameter:2;
}SRC_FIXED_SUPPLY_Typedef;

typedef struct
{
	uint32_t    Maximum_Current:10;
	uint32_t    Minimum_Voltage:10;
	uint32_t    Maximum_Voltage:10;
	uint32_t    PDO_Parameter:2;
}SRC_VAR_SUPPLY_Typedef;

// ================== Request Message ================

typedef struct
{
	uint32_t    Maximum_OP_Current:10;  // 10mA
	uint32_t    OP_Current:10;          // 10mA
	uint32_t    Reserved_1:4;           // Set to Zero
	uint32_t    No_USB_Suspend:1;
	uint32_t    USB_Comm_Capable:1;
	uint32_t    Capa_Mismatch:1;
	uint32_t    GiveBack_Flag:1;        // GiveBack Support set to 1
	uint32_t    Object_Position:3;      // 000 is reserved
	uint32_t    Reserved_2:1;
} REQUEST_FIXED_SUPPLY_STRUCT_Typedef;


typedef enum
{
	TYPE_C_DETACH = 0,
	TYPE_C_ATTACH_DFP = 1, // Host
	TYPE_C_ATTACH_UFP = 2, // Device
	TYPE_C_ATTACH_DRP = 3, // Dual role
} CCIC_MODE;

enum {
	FLASH_MODE_ENTER,
    FLASH_ERASE,
    FLASH_WRITE,
	FLASH_WRITE_BUILTIN,
	FLASH_WRITE_UMS,
	FLASH_MODE_EXIT	
};


typedef enum
{
	CLIENT_OFF = 0,
	CLIENT_ON = 1,
} CCIC_DEVICE_REASON;

typedef enum
{
	HOST_OFF = 0,
	HOST_ON_BY_RD = 1, // Rd detection
	HOST_ON_BY_RID000K = 2, // RID000K detection
} CCIC_HOST_REASON;


#if defined(CONFIG_CCIC_NOTIFIER)
struct ccic_state_work {
	struct work_struct ccic_work;
	int dest;
	int id;
	int attach;
	int event;
};
#endif

struct sm5507_data {
	struct device *dev;
	struct i2c_client *i2c;
#if defined(CONFIG_CCIC_NOTIFIER)
	struct workqueue_struct *ccic_wq;
#endif
	int irq;
	int irq_gpio;
    int redrv1_gpio;
    int sda_gpio;
    int scl_gpio;
    u32 hw_rev;     /* model hw revision info. */
	struct mutex i2c_mutex;
	u8 attach;
	u8 vbus_detach;
	struct wake_lock wlock;

	int p_prev_rid;
	int prev_rid;
	int cur_rid;
	int water_det;

	u8 firm_ver[4];

	int pd_state;
	uint32_t func_state;

	int is_host;
	int is_client;
	int is_dr_swap;
	int is_pr_swap;

	int plug_rprd_sel;
	uint32_t data_role;
#if defined(CONFIG_CCIC_ALTERNATE_MODE)
	uint32_t alternate_state;
	uint32_t acc_type;
	uint32_t Vendor_ID;
	uint32_t Product_ID;
	uint32_t SVID_0;
	uint32_t SVID_1;
	struct delayed_work acc_detach_work;
#endif
	int manual_lpm_mode;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
	struct dual_role_phy_instance *dual_role;
	struct dual_role_phy_desc *desc;
	struct completion reverse_completion;
	int power_role;
	int try_state_change;
	struct delayed_work role_swap_work;
#endif

};


extern void vbus_turn_on_ctrl(bool enable);
extern int sm5507_i2c_read_byte(const struct i2c_client *client, u8 command);
extern int sm5507_i2c_write_byte(const struct i2c_client *client, u8 command, u8 value);
extern int sm5507_multi_read_byte(const struct i2c_client *i2c, u16 reg, u8 *val, u16 size);
extern void sm5507_int_clear(struct sm5507_data *sm5507_data);
extern void sm5507_get_chip_swversion(struct sm5507_data *data, struct sm5507_version *version);
extern void sm5507_get_fw_version(struct sm5507_version *version, u8 boot_version);
extern void sm5507_manual_LPM(struct sm5507_data *data, int cmd);
extern void sm5507_manual_JIGON(struct sm5507_data *data, int mode);
extern void sm5507_control_option_command(struct sm5507_data *sm5507_data, int cmd);
extern void sm5507_rprd_mode_change(struct sm5507_data *data, u8 mode);

#endif  /* __SM5507_H */
