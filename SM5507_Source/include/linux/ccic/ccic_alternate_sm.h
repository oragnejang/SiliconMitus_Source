

#ifndef __LINUX_CCIC_ALTERNATE_MODE_SM_H__
#define __LINUX_CCIC_ALTERNATE_MODE_SM_H__



/* CCIC Dock Observer Callback parameter */
#if defined(CONFIG_CCIC_ALTERNATE_MODE)
enum {
	CCIC_DOCK_DETACHED	= 0,
	CCIC_DOCK_HMT		= 105,
	CCIC_DOCK_ABNORMAL	= 106,
};

enum VDM_MSG_IRQ_State {
	VDM_DISCOVER_ID		=	(1 << 0),
	VDM_DISCOVER_SVIDS	=	(1 << 1),
	VDM_DISCOVER_MODES	=	(1 << 2),
	VDM_ENTER_MODE		=	(1 << 3),
	VDM_EXIT_MODE 		=	(1 << 4),
};

#define GEAR_VR_DETACH_WAIT_MS 1000

typedef union
{
	uint32_t        DATA;
	struct
	{
		uint8_t     BDATA[4];
	}BYTES;
	struct
	{
		uint32_t    VDM_command:5,
				    Rsvd2_VDM_header:1,
				    VDM_command_type:2,
				    Object_Position:3,
				    Rsvd_VDM_header:2,
			    	Structured_VDM_Version:2,
				    VDM_Type:1,
				    Standard_Vendor_ID:16;
	}BITS;
}U_DATA_MSG_VDM_HEADER_Type;


typedef union
{
	uint32_t        DATA;
	struct
	{
		uint8_t     BDATA[4];
	}BYTES;
	struct
	{
		uint32_t    USB_Vendor_ID:16,
			    	Rsvd_ID_header:10,
				    Modal_Operation_Supported:1,
				    Product_Type:3,
				    Data_Capable_USB_Device:1,
				    Data_Capable_USB_Host:1;
	}BITS;
}U_DATA_MSG_ID_HEADER_Type;

typedef union
{
	uint32_t        DATA;
	struct
	{
		uint8_t     BDATA[4];
	}BYTES;
	struct
	{
		uint32_t    Device_Version:16,
			    	Product_ID:16;
	}BITS;
}U_PRODUCT_VDO_Type;

typedef union
{
	uint32_t        DATA;
	struct
	{
		uint8_t     BDATA[4];
	}BYTES;
	struct
	{
		uint32_t    SVID_1:16,
				    SVID_0:16;
	}BITS;
}U_VDO1_Type;


void receive_alternate_message(void * data);

#endif
#endif /* __LINUX_CCIC_ALTERNATE_MODE_SM_H__ */
