/* 
*   BSD 2-Clause License
*   Copyright (c) 2022, LiteEMF
*   All rights reserved.
*   This software component is licensed by LiteEMF under BSD 2-Clause license,
*   the "License"; You may not use this file except in compliance with the
*   License. You may obtain a copy of the License at:
*       opensource.org/licenses/BSD-2-Clause
* 
*/


#ifndef _hw_config_h
#define _hw_config_h

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************************************
** Defined
*******************************************************************************************************/
#define LOG_ENABLE                      1
#define HEAP_ID							0

/******************************************************************************************************
** project
*******************************************************************************************************/
#define PROJECT_GAMEPAD                  1           //keyboard and mouse project

#if PROJECT_GAMEPAD
	/**********************************************************************************/
	#define DEV_TRPS_DEFAULT			BIT(TR_NULL)				/*产品传输层支持*/
	#define DEV_TYPES_DEFAULT			BIT(DEV_TYPE_NONE)
	#define HID_TYPES_DEFAULT			BIT(HID_TYPE_NONE)

	/**********************************************************************************/

	#define GAMEPAD1						1

	#if GAMEPAD1	
		#define APP_KEY_ENABLE			1
		#define APP_IMU_ENABLE			1
		#define IMU_SH3001_ID			0
		#define APP_JOYSTICK_ENABLE		1
		#define APP_RUMBLE_ENABLE		1
		#define APP_RGB_ENABLE			1
		#define APP_RGB_NUMS 			3
		#define API_WDT_ENABLE			1

		#define API_USBD_BIT_ENABLE			BIT(0)
		#define USBD_NUM					2
		// #define USBD_RAM_DISK_ENABLE		1
		// #define USBD_TYPE_SUPPORT 			(BIT_ENUM(DEV_TYPE_MSD))
		#define USBD_TYPE_SUPPORT 			BIT_ENUM(DEV_TYPE_HID)
		#define USBD_HID_SUPPORT			(BIT_ENUM(HID_TYPE_KB) | BIT_ENUM(HID_TYPE_MOUSE) | BIT_ENUM(HID_TYPE_CONSUMER))
		// #define USBD_HID_SUPPORT			(BIT_ENUM(HID_TYPE_KB))

		#define API_USBH_BIT_ENABLE			0 //BIT(0)
		#define USBH_TYPE_SUPPORT			(BIT_ENUM(DEV_TYPE_HID) | BIT_ENUM(DEV_TYPE_HUB))
		#define USBH_HID_SUPPORT			(BIT_ENUM(HID_TYPE_KB) | BIT_ENUM(HID_TYPE_MOUSE) | BIT_ENUM(HID_TYPE_CONSUMER))

		#define SW_VERSION                     	0x01
        #define DEFAULT_NAME			       	"gamepad"
        #define DEFAULT_MODEL					"GP_dev"
	#else

	#endif

#endif


#include "emf_config.h"
#include "hw_board.h"
#ifdef __cplusplus
}
#endif
#endif





