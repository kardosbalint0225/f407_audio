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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "audio_io.h"
#include "cs43l22.h"
#include "debug_uart.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_data_i2c[256];
uint8_t cs43l22_default[] = {
		CS43L22_ID_DEFAULT,
		POWER_CONTROL_1_DEFAULT,
		RESERVED_03H_DEFAULT,
		POWER_CONTROL_2_DEFAULT,
		CLOCKING_CONTROL_DEFAULT,
		INTERFACE_CONTROL_1_DEFAULT,
		INTERFACE_CONTROL_2_DEFAULT,
		PASSTHROUGH_A_SELECT_DEFAULT,
		PASSTHROUGH_B_SELECT_DEFAULT,
		ANALOG_ZC_AND_SR_SETTINGS_DEFAULT,
		RESERVED_0BH_DEFAULT,
		PASSTHROUGH_GANG_CONTROL_DEFAULT,
		PLAYBACK_CONTROL_1_DEFAULT,
		MISCELLANEOUS_CONTROL_DEFAULT,
		PLAYBACK_CONTROL_2_DEFAULT,
		RESERVED_10H_DEFAULT,
		RESERVED_11H_DEFAULT,
		RESERVED_12H_DEFAULT,
		RESERVED_13H_DEFAULT,
		PASSTHROUGH_A_VOLUME_DEFAULT,
		PASSTHROUGH_B_VOLUME_DEFAULT,
		RESERVED_16H_DEFAULT,
		RESERVED_17H_DEFAULT,
		RESERVED_18H_DEFAULT,
		RESERVED_19H_DEFAULT,
		PCMA_VOLUME_DEFAULT,
		PCMB_VOLUME_DEFAULT,
		BEEP_FREQUENCY_ON_TIME_DEFAULT,
		BEEP_VOLUME_OFF_TIME_DEFAULT,
		BEEP_TONE_CONFIG_DEFAULT,
		TONE_CONTROL_DEFAULT,
		MASTER_A_VOLUME_DEFAULT,
		MASTER_B_VOLUME_DEFAULT,
		HEADPHONE_A_VOLUME_DEFAULT,
		HEADPHONE_B_VOLUME_DEFAULT,
		SPEAKER_A_VOLUME_DEFAULT,
		SPEAKER_B_VOLUME_DEFAULT,
		CHANNEL_MIXER_AND_SWAP_DEFAULT,
		LIMITER_CONTROL_1_DEFAULT,
		LIMITER_CONTROL_2_DEFAULT,
		LIMITER_ATTACK_RATE_DEFAULT,
		RESERVED_2AH_DEFAULT,
		RESERVED_2BH_DEFAULT,
		RESERVED_2CH_DEFAULT,
		RESERVED_2DH_DEFAULT,
		OVERFLOW_AND_CLOCK_STATUS_DEFAULT,
		BATTERY_COMPENSATION_DEFAULT,
		VP_BATTERY_LEVEL_DEFAULT,
		SPEAKER_STATUS_DEFAULT,
		RESERVED_32H_DEFAULT,
		RESERVED_33H_DEFAULT,
		CHARGE_PUMP_FREQUENCY_DEFAULT
};
volatile uint8_t mismatch[10];
//extern volatile bool is_ready;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  audio_status_t status;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  status = audio_io_init();
  if (AUDIO_IO_OK != status) {
	  Error_Handler();
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  uint16_t size = sizeof(cs43l22_default);
  status = audio_io_read((0x80 | CS43L22_ID), rx_data_i2c, size, false);

  if (AUDIO_IO_OK != status) {
	  Error_Handler();
  }

  while(!audio_io_i2c_rx_cplt);

  int j = 0;
  for (int i = 0; i < size; i++) {
	  if (cs43l22_default[i] != rx_data_i2c[i]) {
		  mismatch[j] = i;
		  j++;
	  }
  }

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
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}


/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
