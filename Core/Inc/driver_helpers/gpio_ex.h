#ifndef __GPIO_EX_H
#define __GPIO_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "def.h"

#define GPIO_AF0_RTC_50Hz      			((uint8_t)0x00)  		/* RTC_50Hz Alternate Function mapping                       */
#define GPIO_AF0_MCO           			((uint8_t)0x00)  		/* MCO (MCO1 and MCO2) Alternate Function mapping            */
#define GPIO_AF0_TAMPER        			((uint8_t)0x00)  		/* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_AF0_SWJ           			((uint8_t)0x00)  		/* SWJ (SWD and JTAG) Alternate Function mapping             */
#define GPIO_AF0_TRACE         			((uint8_t)0x00)  		/* TRACE Alternate Function mapping                          */
#define GPIO_AF1_TIM1          			((uint8_t)0x01)  		/* TIM1 Alternate Function mapping */
#define GPIO_AF1_TIM2          			((uint8_t)0x01)  		/* TIM2 Alternate Function mapping */
#define GPIO_AF2_TIM3          			((uint8_t)0x02)  		/* TIM3 Alternate Function mapping */
#define GPIO_AF2_TIM4          			((uint8_t)0x02)  		/* TIM4 Alternate Function mapping */
#define GPIO_AF2_TIM5          			((uint8_t)0x02)  		/* TIM5 Alternate Function mapping */
#define GPIO_AF3_TIM8          			((uint8_t)0x03)  		/* TIM8 Alternate Function mapping  */
#define GPIO_AF3_TIM9          			((uint8_t)0x03)  		/* TIM9 Alternate Function mapping  */
#define GPIO_AF3_TIM10         			((uint8_t)0x03)  		/* TIM10 Alternate Function mapping */
#define GPIO_AF3_TIM11         			((uint8_t)0x03)  		/* TIM11 Alternate Function mapping */
#define GPIO_AF4_I2C1          			((uint8_t)0x04)  		/* I2C1 Alternate Function mapping */
#define GPIO_AF4_I2C2          			((uint8_t)0x04)  		/* I2C2 Alternate Function mapping */
#define GPIO_AF4_I2C3          			((uint8_t)0x04)  		/* I2C3 Alternate Function mapping */
#define GPIO_AF5_SPI1          			((uint8_t)0x05)  		/* SPI1 Alternate Function mapping        */
#define GPIO_AF5_SPI2          			((uint8_t)0x05)  		/* SPI2/I2S2 Alternate Function mapping   */
#define GPIO_AF5_I2S3ext       			((uint8_t)0x05)  		/* I2S3ext_SD Alternate Function mapping  */
#define GPIO_AF6_SPI3          			((uint8_t)0x06)  		/* SPI3/I2S3 Alternate Function mapping  */
#define GPIO_AF6_I2S2ext       			((uint8_t)0x06)  		/* I2S2ext_SD Alternate Function mapping */
#define GPIO_AF7_USART1        			((uint8_t)0x07)  		/* USART1 Alternate Function mapping     */
#define GPIO_AF7_USART2        			((uint8_t)0x07)  		/* USART2 Alternate Function mapping     */
#define GPIO_AF7_USART3        			((uint8_t)0x07)  		/* USART3 Alternate Function mapping     */
#define GPIO_AF7_I2S3ext       			((uint8_t)0x07)  		/* I2S3ext_SD Alternate Function mapping */
#define GPIO_AF8_UART4         			((uint8_t)0x08)  		/* UART4 Alternate Function mapping  */
#define GPIO_AF8_UART5         			((uint8_t)0x08)  		/* UART5 Alternate Function mapping  */
#define GPIO_AF8_USART6        			((uint8_t)0x08)  		/* USART6 Alternate Function mapping */
#define GPIO_AF9_CAN1          			((uint8_t)0x09)  		/* CAN1 Alternate Function mapping  */
#define GPIO_AF9_CAN2          			((uint8_t)0x09)  		/* CAN2 Alternate Function mapping  */
#define GPIO_AF9_TIM12         			((uint8_t)0x09)  		/* TIM12 Alternate Function mapping */
#define GPIO_AF9_TIM13         			((uint8_t)0x09)  		/* TIM13 Alternate Function mapping */
#define GPIO_AF9_TIM14         			((uint8_t)0x09)  		/* TIM14 Alternate Function mapping */
#define GPIO_AF10_OTG_FS        		((uint8_t)0x0A)  		/* OTG_FS Alternate Function mapping */
#define GPIO_AF10_OTG_HS        		((uint8_t)0x0A)  		/* OTG_HS Alternate Function mapping */
#define GPIO_AF11_ETH           		((uint8_t)0x0B)  		/* ETHERNET Alternate Function mapping */
#define GPIO_AF12_FSMC          		((uint8_t)0x0C)  		/* FSMC Alternate Function mapping                     */
#define GPIO_AF12_OTG_HS_FS     		((uint8_t)0x0C)  		/* OTG HS configured in FS, Alternate Function mapping */
#define GPIO_AF12_SDIO          		((uint8_t)0x0C)  		/* SDIO Alternate Function mapping                     */
#define GPIO_AF13_DCMI          		((uint8_t)0x0D)  		/* DCMI Alternate Function mapping */
#define GPIO_AF15_EVENTOUT      		((uint8_t)0x0F)  		/* EVENTOUT Alternate Function mapping */

  
#ifdef __cplusplus
}
#endif

#endif 

