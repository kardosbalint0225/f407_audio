/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "dma.h"
#include "fatfs.h"
#include "rtc.h"
#include "usb_host.h"
//#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sysclk.h"
#include "audio.h"
#include "debug_uart.h"
#include "hmi.h"
#include "utils.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//void MX_GPIO_Init(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t audio_appdata[AUDIO_BUFFER_SIZE];
uint8_t audio_halfbuf[AUDIO_BUFFER_HALF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void audio_out_write_callback(uint8_t *address, uint32_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern ApplicationTypeDef 	Appli_state;
extern FATFS 				USBHFatFS;    	/* File system object for USBH logical drive */
extern FIL 					USBHFile;       /* File object for USBH */
extern char 				USBHPath[4];   	/* USBH logical drive path */
extern bool 				is_btn1_pushed;
extern bool    				is_usbhfatfs_mounted;
bool start = false;
volatile bool read_half = false;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_DMA_Init();
  MX_USB_HOST_Init();
  //MX_RTC_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */
  debug_uart_status_t debug_uart_state = debug_uart_init();
  if (DEBUG_UART_OK != debug_uart_state) {
	  Error_Handler();
  }

  printf("Hello.\r\n");
  printf("bello \r\n");
  printf("Some short text to test.\r\n");
  printf("It is important to place the newline character because the stdout is buffered.\r\n");
  printf("That was a longer text than before.\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  audio_status_t audio_status = audio_out_init();
  if (AUDIO_OK != audio_status) {
	  Error_Handler();
  }

  audio_out_hw_params_t hw_params = {
	  .rate     = 44100,
	  .channels = 2,
	  .format   = AUDIO_FORMAT_S16_LE,
  };

  if (AUDIO_OK != audio_out_set_hw_params(&hw_params)) {
	  Error_Handler();
  }

  if (AUDIO_OK != audio_out_set_write_callback(audio_out_write_callback)) {
	  Error_Handler();
  }

  for (uint32_t i = 0; i < AUDIO_BUFFER_SIZE; i++) {
	  audio_appdata[i] = 0;
  }

  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    if (!start && is_usbhfatfs_mounted) {
    	if (FR_OK != f_open(&USBHFile, "ilan_bluestone_-_sinai.raw", FA_READ)) {
    		  Error_Handler();
    	  }

    	  UINT br;

    	  if (FR_OK != f_read(&USBHFile, audio_appdata, AUDIO_BUFFER_SIZE, &br)) {
    		  Error_Handler();
    	  }

    	  if (AUDIO_OK != audio_out_write(audio_appdata, AUDIO_BUFFER_SIZE)) {
    		  Error_Handler();
    	  }

    	  if (AUDIO_OK != audio_out_start()) {
    	      Error_Handler();
    	  }
    	  start = true;
    	  read_half = true;
    }

    if (read_half) {
    	UINT br;
		if (FR_OK != f_read(&USBHFile, audio_halfbuf, AUDIO_BUFFER_HALF_SIZE, &br)) {
			Error_Handler();
		}
		read_half = false;
    }

    /* USER CODE BEGIN 3 */



  }
  /* USER CODE END 3 */
}


/* USER CODE BEGIN 4 */


void audio_out_write_callback(uint8_t *address, uint32_t size)
{
	read_half = true;

	for (uint32_t i = 0; i < size; i++) {
		address[i] = audio_halfbuf[i];
	}

	if (AUDIO_OK != audio_out_write(address, size)) {
		Error_Handler();
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
	  HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


