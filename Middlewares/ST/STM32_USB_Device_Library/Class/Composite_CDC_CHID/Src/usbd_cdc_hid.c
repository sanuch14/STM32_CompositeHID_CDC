/**
  ******************************************************************************
  * @file    usbd_cdc_hid.c
  * @author  Oleksandr Lisovyi
  * @version V0.0.1
  * @date    05-November-2016
  * @verbatim
  *      
  *          ===================================================================      
  *                                composite CDC_HID
  *          =================================================================== 
  *      
  * @endverbatim
  ******************************************************************************
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_hid.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"



/**
*********************************************
   CDC Device library callbacks
 *********************************************/
extern uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
extern uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
extern uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
extern uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
extern uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
extern uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev);
extern uint8_t  *USBD_CDC_GetFSCfgDesc (uint16_t *length);
extern uint8_t  *USBD_CDC_GetHSCfgDesc (uint16_t *length);
extern uint8_t  *USBD_CDC_GetOtherSpeedCfgDesc (uint16_t *length);
extern uint8_t  *USBD_CDC_GetDeviceQualifierDescriptor (uint16_t *length);

/*********************************************
   Custon HID Device library callbacks
*********************************************/
extern uint8_t  USBD_CUSTOM_HID_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
extern uint8_t  USBD_CUSTOM_HID_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
extern uint8_t  USBD_CUSTOM_HID_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
extern uint8_t  *USBD_CUSTOM_HID_GetCfgDesc (uint16_t *length);
extern uint8_t  *USBD_CUSTOM_HID_GetDeviceQualifierDesc (uint16_t *length);
extern uint8_t  USBD_CUSTOM_HID_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
extern uint8_t  USBD_CUSTOM_HID_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
extern uint8_t  USBD_CUSTOM_HID_EP0_RxReady (USBD_HandleTypeDef  *pdev);

/** @defgroup USBD_CDC_HID_Private_Variables
* @{
*/ 
static uint8_t USBD_CDC_HID_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_HID_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_HID_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_CDC_HID_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_HID_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_HID_EP0_RxReady (USBD_HandleTypeDef *pdev);
static uint8_t *USBD_CDC_HID_GetCfgDesc (uint16_t *length);
static uint8_t *USBD_CDC_HID_GetDeviceQualifierDesc (uint16_t *length);

//  (USB_MSC_CONFIG_DESC_SIZ -9 + USB_CDC_CONFIG_DESC_SIZ  + 8)



USBD_CDC_HID_ItfTypeDef CDC_HID = 
{
	&USBD_CustomHID_fops_FS,
	&USBD_Interface_fops_FS,
};



/* CDC_HID interface class callbacks structure */
USBD_ClassTypeDef  USBD_CDC_HID = 
{
	USBD_CDC_HID_Init,
	USBD_CDC_HID_DeInit,
	USBD_CDC_HID_Setup,
	NULL,                 /* EP0_TxSent, */
	USBD_CDC_HID_EP0_RxReady, /*EP0_RxReady*/ /* STATUS STAGE IN */
	USBD_CDC_HID_DataIn,    /*DataIn*/
	USBD_CDC_HID_DataOut,
	NULL,    /*SOF */
	NULL,
	NULL,     
	USBD_CDC_HID_GetCfgDesc,  
	USBD_CDC_HID_GetCfgDesc,    
	USBD_CDC_HID_GetCfgDesc, 
	USBD_CDC_HID_GetDeviceQualifierDesc,
};


/* USB CDC_HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_HID_CfgDesc[USB_CDC_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
	0x09, /* bLength: Configuration Descriptor size */
	USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
	USB_CDC_HID_CONFIG_DESC_SIZ,
	/* wTotalLength: Bytes returned */
	0x00,
	0x03,         /*bNumInterfaces: 3 interfaces (2 for CDC, 1 for HID)*/
	0x01,         /*bConfigurationValue: Configuration value*/
	0x00,         /*iConfiguration: Index of string descriptor describing
	the configuration*/
	0xC0,         /*bmAttributes: bus powered */
	0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
	/*9*/
	
	
	
	///////////////////////////////////////////////////////////////////////////////////////////////
    // IAD      Digitizer
     //  0x08,	// bLength: Interface Descriptor size
     //  0x0B,	// bDescriptorType: IAD
     //  0x00,	// bFirstInterface                    //starts from zero .. hence 0-1- and now 2
     //  0x01,	// bInterfaceCount                    //no of interfaces used in function. for above cdc it was 2
     //  0x00,   //*bInterfaceClass: HID*/
    //   0x00,   //*bInterfaceSubClass : 1=BOOT, 0=no boot*/
     //  0x00,   //*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
     //  0x02,   //*iInterface: Index of string descriptor*/
    ////////////////////////////////////////////////////////////////////////////////////////////////
 
	
	
	
	
	
	/************** Descriptor of CUSTOM HID interface ****************/
	0x09,         /*bLength: Interface Descriptor size*/
	USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
	HID_INTERFACE,         /*bInterfaceNumber: Number of Interface*/
	0x00,         /*bAlternateSetting: Alternate setting*/
	0x02,         /*bNumEndpoints*/
	0x03,         /*bInterfaceClass: CUSTOM_HID*/
	0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
	0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
	0,            /*iInterface: Index of string descriptor*/
	/*18*/
	
	/******************** Descriptor of CUSTOM_HID *************************/
	0x09,         /*bLength: CUSTOM_HID Descriptor size*/
	CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
	0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
	0x01,
	0x00,         /*bCountryCode: Hardware target country*/
	0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
	0x22,         /*bDescriptorType*/
	USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
	0x00,
	/*27*/
	
	/******************** Descriptor of Custom HID endpoints ********************/
	0x07,          /*bLength: Endpoint Descriptor size*/
	USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
	CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
	0x03,          /*bmAttributes: Interrupt endpoint*/
	CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 64 Byte max */
	0x00,
	0x01,          /*bInterval: Polling Interval (20 ms)*///0x20/////////////////////////////////////////////////////////
	/*34*/
	
	0x07,	         /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT,	/* bDescriptorType: */
	CUSTOM_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
	0x03,	/* bmAttributes: Interrupt endpoint */
	CUSTOM_HID_EPOUT_SIZE,	/* wMaxPacketSize: 2 Bytes max  */
	0x00,
	0x01,	/* bInterval: Polling Interval (20 ms) *//////////0x20/////////////////////////////////////////////////////////
	/*41*/
	
	
	/******** /IAD should be positioned just before the CDC interfaces ******
                IAD to associate the two CDC interfaces */
  
	0x08, /* bLength */
	0x0B, /* bDescriptorType */
	0x01, /* bFirstInterface */
	0x02, /* bInterfaceCount */
	0x02, /* bFunctionClass */
	0x02, /* bFunctionSubClass */
	0x01, /* bFunctionProtocol */
	0x02, /* iFunction (Index of string descriptor describing this function) */
	/*49*/
	
	/************************  CDC interface ********************************/
	0x09,   /* bLength: Interface Descriptor size */
	USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
	/* Interface descriptor type */
	CDC_INTERFACE0,   /* bInterfaceNumber: Number of Interface */
	0x00,   /* bAlternateSetting: Alternate setting */
	0x01,   /* bNumEndpoints: One endpoints used */
	0x02,   /* bInterfaceClass: Communication Interface Class */
	0x02,   /* bInterfaceSubClass: Abstract Control Model */
	0x01,   /* bInterfaceProtocol: Common AT commands */
	0x00,   /* iInterface: */
	/*58*/
	
	/*Header Functional Descriptor*/
	0x05,   /* bLength: Endpoint Descriptor size */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x00,   /* bDescriptorSubtype: Header Func Desc */
	0x10,   /* bcdCDC: spec release number */
	0x01,
	/*63*/
	
	/*Call Management Functional Descriptor*/
	0x05,   /* bFunctionLength */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x01,   /* bDescriptorSubtype: Call Management Func Desc */
	0x00,   /* bmCapabilities: D0+D1 */
	0x01,   /* bDataInterface: 1 */
	/*68*/
	
	/*ACM Functional Descriptor*/
	0x04,   /* bFunctionLength */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
	0x02,   /* bmCapabilities */
	/*72*/
	
	/*Union Functional Descriptor*/
	0x05,   /* bFunctionLength */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x06,   /* bDescriptorSubtype: Union func desc */
	0x00,   /* bMasterInterface: Communication class interface */
	0x01,   /* bSlaveInterface0: Data Class Interface */
	/*77*/
	
	/*Endpoint 2 Descriptor*/
	0x07,                           /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
	CDC_CMD_EP,                     /* bEndpointAddress */
	0x03,                           /* bmAttributes: Interrupt */
	LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
	HIBYTE(CDC_CMD_PACKET_SIZE),
	0x10,                           /* bInterval: */ 
	/*84*/
	
	/*---------------------------------------------------------------------------*/
  
	/*Data class interface descriptor*/
	0x09,   /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
	CDC_INTERFACE1,   /* bInterfaceNumber: Number of Interface */
	0x00,   /* bAlternateSetting: Alternate setting */
	0x02,   /* bNumEndpoints: Two endpoints used */
	0x0A,   /* bInterfaceClass: CDC */
	0x00,   /* bInterfaceSubClass: */
	0x00,   /* bInterfaceProtocol: */
	0x00,   /* iInterface: */
	/*93*/
	
	/*Endpoint OUT Descriptor*/
	0x07,   /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
	CDC_OUT_EP,                        /* bEndpointAddress */
	0x02,                              /* bmAttributes: Bulk */
	LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
	HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
	0x00,                              /* bInterval: ignore for Bulk transfer */
	/*100*/
	
	/*Endpoint IN Descriptor*/
	0x07,   /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
	CDC_IN_EP,                         /* bEndpointAddress */
	0x02,                              /* bmAttributes: Bulk */
	LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
	HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
	0x00                               /* bInterval: ignore for Bulk transfer */
	/*107*/
} ;


/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @brief  USBD_CDC_HID_Init
  *         Initialize the CDC and HID interfaces
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_HID_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	/* HID initialization */
	USBD_CUSTOM_HID_Init(pdev, cfgidx);
	/* CDC initialization */
	USBD_CDC_Init(pdev, cfgidx);
	
	return USBD_OK;
}

/**
  * @brief  USBD_CDC_HID_DeInit
  *         DeInitialize the CDC and HID interfaces
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_HID_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	/* HID initialization */
	USBD_CUSTOM_HID_DeInit(pdev, cfgidx);
	/* CDC initialization */
	USBD_CDC_DeInit(pdev, cfgidx);
	
	return USBD_OK;
}

/**
  * @brief  USBD_CDC_HID_Setup
  *         Handle the CDC and HID specific requests
  * @param  pdev: instance
  * @param  req: USBD requests
  * @retval status
*/
static uint8_t USBD_CDC_HID_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
	if (req->wIndex == HID_INTERFACE)
		return (USBD_CUSTOM_HID_Setup (pdev, req));
	else
		return (USBD_CDC_Setup(pdev, req));
}

/**
  * @brief  USBD_CDC_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
 */
static uint8_t USBD_CDC_HID_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	/*DataIN can be for CDC or HID */
	if (epnum == (CUSTOM_HID_EPIN_ADDR&~0x80))
		return (USBD_CUSTOM_HID_DataIn(pdev, epnum));
	else
		return (USBD_CDC_DataIn(pdev, epnum));
		
}

/**
  * @brief  USBD_CDC_HID_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_CDC_HID_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	/*DataOUT can be for CDC or HID */
	if (epnum == (CUSTOM_HID_EPOUT_ADDR&~0x80))
		return (USBD_CUSTOM_HID_DataOut(pdev, epnum));
	else
		return (USBD_CDC_DataOut(pdev, epnum));
}

/**
  * @brief  USBD_CDC_HID_EP0_RxReady
  *         Handles control request data.
  * @param  pdev: device instance
  * @retval status
*/
static uint8_t USBD_CDC_HID_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
	/**????**/
	USBD_CUSTOM_HID_EP0_RxReady(pdev);
	
	USBD_CDC_EP0_RxReady(pdev);
	
	return USBD_OK;
}

/**
  * @brief  USBD_CDC_HID_GetCfgDesc 
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_HID_GetCfgDesc (uint16_t *length)
{
	*length = sizeof (USBD_CDC_HID_CfgDesc);
	return USBD_CDC_HID_CfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t *USBD_CDC_HID_GetDeviceQualifierDesc (uint16_t *length)
{
	*length = sizeof (USBD_CDC_HID_DeviceQualifierDesc);
	return USBD_CDC_HID_DeviceQualifierDesc;
}



/**
* @brief  USBD_CDC_HID_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CUSTOMHID Interface callback
  * @retval status
  */
//uint8_t  USBD_CDC_HID_RegisterInterface  (USBD_HandleTypeDef   *pdev, USBD_CUSTOM_HID_ItfTypeDef *fops)
uint8_t  USBD_CDC_HID_RegisterInterface  (USBD_HandleTypeDef   *pdev)
{
	pdev->pUserData= &CDC_HID;
  USBD_CUSTOM_HID_RegisterInterface(pdev, &USBD_CustomHID_fops_FS);
	USBD_CDC_RegisterInterface(pdev, &USBD_Interface_fops_FS);
	
	return USBD_OK;
}


