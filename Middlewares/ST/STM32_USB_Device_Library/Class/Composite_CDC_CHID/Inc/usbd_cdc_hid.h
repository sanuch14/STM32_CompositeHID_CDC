/**
  ******************************************************************************
  * @file    usbd_cdc_hid.h
  * @author  Oleksandr Lisovyi
  * @version V1.0
  * @date    09-December-2016
  * @brief   header file for the usbd_cdc_hid.c file.
  ******************************************************************************
  */ 
  
/* Includes ------------------------------------------------------------------*/

#ifndef _USBD_CDC_HID_H_
#define _USBD_CDC_HID_H_

#include  "usbd_ioreq.h"
#include "usbd_customhid.h"
#include "usbd_cdc.h"

#define USB_CDC_HID_CONFIG_DESC_SIZ 	0x6B
#define HID_INTERFACE 					0x00
#define CDC_INTERFACE0 					0x01
#define CDC_INTERFACE1 					0x02

extern USBD_ClassTypeDef  USBD_CDC_HID;
#define USBD_CDC_HID_CLASS  &USBD_CDC_HID

typedef struct _USBD_CDC_HID_Itf
{
	USBD_CUSTOM_HID_ItfTypeDef 		*HID;
	USBD_CDC_ItfTypeDef						*CDC;
}USBD_CDC_HID_ItfTypeDef;

uint8_t  USBD_CDC_HID_RegisterInterface  (USBD_HandleTypeDef   *pdev);

#endif  /* _USBD_CDC_HID_H_*/
