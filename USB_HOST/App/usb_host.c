/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_msc.h"


/* USER CODE BEGIN Includes */
#include <stdbool.h>
//#include "main.h"
#include "fatfs.h"
#include "debug_uart.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern FATFS USBHFatFS;    	/* File system object for USBH logical drive      */
extern FIL   USBHFile;      /* File object for USBH 						  */
extern char  USBHPath[4];	/* USBH logical drive path 						  */

bool         is_usbhfatfs_mounted = false;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */

  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_Start(&hUsbHostFS) != USBH_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */

  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * Background task
 */
void MX_USB_HOST_Process(void)
{
  /* USB Host Background task */
  USBH_Process(&hUsbHostFS);
}
/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
	/* USER CODE BEGIN CALL_BACK_1 */
	switch(id)
	{
		case HOST_USER_SELECT_CONFIGURATION : {

		} break;

		case HOST_USER_DISCONNECTION : {

			if (is_usbhfatfs_mounted == true) {
				if (FR_OK != f_unmount(USBHPath)) {
					printf("%s unmount failed.\r\n", USBHPath);
					Error_Handler();
				} else {
					is_usbhfatfs_mounted = false;
					printf("%s unmounted successfully.\r\n", USBHPath);
				}
			}

			Appli_state = APPLICATION_DISCONNECT;
		} break;

		case HOST_USER_CLASS_ACTIVE : {

			if (is_usbhfatfs_mounted != true) {
				BYTE option = 1; 									/**< Mount immediately */
				if (FR_OK != f_mount(&USBHFatFS, USBHPath, option)) {
					printf("%s mount failed.\r\n", USBHPath);
					Error_Handler();
				} else {
					is_usbhfatfs_mounted = true;
					printf("%s mounted successfully.\r\n", USBHPath);
				}
			}

			Appli_state = APPLICATION_READY;
		} break;

		case HOST_USER_CONNECTION : {
			is_usbhfatfs_mounted = false;
			Appli_state = APPLICATION_START;
		} break;

		default : {

		} break;
	}
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
