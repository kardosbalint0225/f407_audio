//#ifdef TEST

#ifndef __RCC_H
#define __RCC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "def.h"
#include "rcc_ex.h"

typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                      This parameter can be a value of @ref RCC_Oscillator_Type                   */
  uint32_t HSEState;             /*!< The new state of the HSE.
                                      This parameter can be a value of @ref RCC_HSE_Config                        */
  uint32_t LSEState;             /*!< The new state of the LSE.
                                      This parameter can be a value of @ref RCC_LSE_Config                        */
  uint32_t HSIState;             /*!< The new state of the HSI.
                                      This parameter can be a value of @ref RCC_HSI_Config                        */
  uint32_t HSICalibrationValue;  /*!< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */
  uint32_t LSIState;             /*!< The new state of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Config                        */
  RCC_PLLInitTypeDef PLL;        /*!< PLL structure parameters                                                    */
}RCC_OscInitTypeDef;

typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured.
                                       This parameter can be a value of @ref RCC_System_Clock_Type      */
  uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCC_System_Clock_Source    */
  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_AHB_Clock_Source       */
  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */
  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */
}RCC_ClkInitTypeDef;

#define RCC_OSCILLATORTYPE_NONE             0x00000000U
#define RCC_OSCILLATORTYPE_HSE              0x00000001U
#define RCC_OSCILLATORTYPE_HSI              0x00000002U
#define RCC_OSCILLATORTYPE_LSE              0x00000004U
#define RCC_OSCILLATORTYPE_LSI              0x00000008U

#define RCC_HSE_OFF                         0x00000000U
#define RCC_HSE_ON                          RCC_CR_HSEON
#define RCC_HSE_BYPASS                      ((uint32_t)(RCC_CR_HSEBYP | RCC_CR_HSEON))

#define RCC_LSE_OFF                         0x00000000U
#define RCC_LSE_ON                          RCC_BDCR_LSEON
#define RCC_LSE_BYPASS                      ((uint32_t)(RCC_BDCR_LSEBYP | RCC_BDCR_LSEON))

#define RCC_HSI_OFF                         ((uint8_t)0x00)
#define RCC_HSI_ON                          ((uint8_t)0x01)

#define RCC_HSICALIBRATION_DEFAULT          0x10U         /* Default HSI calibration trimming value */

#define RCC_LSI_OFF                         ((uint8_t)0x00)
#define RCC_LSI_ON                          ((uint8_t)0x01)

#define RCC_PLL_NONE                        ((uint8_t)0x00)
#define RCC_PLL_OFF                         ((uint8_t)0x01)
#define RCC_PLL_ON                          ((uint8_t)0x02)

#define RCC_PLLP_DIV2                       0x00000002U
#define RCC_PLLP_DIV4                       0x00000004U
#define RCC_PLLP_DIV6                       0x00000006U
#define RCC_PLLP_DIV8                       0x00000008U

#define RCC_PLLSOURCE_HSI                   RCC_PLLCFGR_PLLSRC_HSI
#define RCC_PLLSOURCE_HSE                   RCC_PLLCFGR_PLLSRC_HSE

#define RCC_CLOCKTYPE_SYSCLK                0x00000001U
#define RCC_CLOCKTYPE_HCLK                  0x00000002U
#define RCC_CLOCKTYPE_PCLK1                 0x00000004U
#define RCC_CLOCKTYPE_PCLK2                 0x00000008U

#define RCC_SYSCLKSOURCE_HSI                RCC_CFGR_SW_HSI
#define RCC_SYSCLKSOURCE_HSE                RCC_CFGR_SW_HSE
#define RCC_SYSCLKSOURCE_PLLCLK             RCC_CFGR_SW_PLL
#define RCC_SYSCLKSOURCE_PLLRCLK            ((uint32_t)(RCC_CFGR_SW_0 | RCC_CFGR_SW_1))

#define RCC_SYSCLKSOURCE_STATUS_HSI         RCC_CFGR_SWS_HSI   /*!< HSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSE         RCC_CFGR_SWS_HSE   /*!< HSE used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK      RCC_CFGR_SWS_PLL   /*!< PLL used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLRCLK     ((uint32_t)(RCC_CFGR_SWS_0 | RCC_CFGR_SWS_1))   /*!< PLLR used as system clock */

#define RCC_SYSCLK_DIV1                     RCC_CFGR_HPRE_DIV1
#define RCC_SYSCLK_DIV2                     RCC_CFGR_HPRE_DIV2
#define RCC_SYSCLK_DIV4                     RCC_CFGR_HPRE_DIV4
#define RCC_SYSCLK_DIV8                     RCC_CFGR_HPRE_DIV8
#define RCC_SYSCLK_DIV16                    RCC_CFGR_HPRE_DIV16
#define RCC_SYSCLK_DIV64                    RCC_CFGR_HPRE_DIV64
#define RCC_SYSCLK_DIV128                   RCC_CFGR_HPRE_DIV128
#define RCC_SYSCLK_DIV256                   RCC_CFGR_HPRE_DIV256
#define RCC_SYSCLK_DIV512                   RCC_CFGR_HPRE_DIV512

#define RCC_HCLK_DIV1                       RCC_CFGR_PPRE1_DIV1
#define RCC_HCLK_DIV2                       RCC_CFGR_PPRE1_DIV2
#define RCC_HCLK_DIV4                       RCC_CFGR_PPRE1_DIV4
#define RCC_HCLK_DIV8                       RCC_CFGR_PPRE1_DIV8
#define RCC_HCLK_DIV16                      RCC_CFGR_PPRE1_DIV16

#define RCC_RTCCLKSOURCE_NO_CLK             0x00000000U
#define RCC_RTCCLKSOURCE_LSE                0x00000100U
#define RCC_RTCCLKSOURCE_LSI                0x00000200U
#define RCC_RTCCLKSOURCE_HSE_DIVX           0x00000300U
#define RCC_RTCCLKSOURCE_HSE_DIV2           0x00020300U
#define RCC_RTCCLKSOURCE_HSE_DIV3           0x00030300U
#define RCC_RTCCLKSOURCE_HSE_DIV4           0x00040300U
#define RCC_RTCCLKSOURCE_HSE_DIV5           0x00050300U
#define RCC_RTCCLKSOURCE_HSE_DIV6           0x00060300U
#define RCC_RTCCLKSOURCE_HSE_DIV7           0x00070300U
#define RCC_RTCCLKSOURCE_HSE_DIV8           0x00080300U
#define RCC_RTCCLKSOURCE_HSE_DIV9           0x00090300U
#define RCC_RTCCLKSOURCE_HSE_DIV10          0x000A0300U
#define RCC_RTCCLKSOURCE_HSE_DIV11          0x000B0300U
#define RCC_RTCCLKSOURCE_HSE_DIV12          0x000C0300U
#define RCC_RTCCLKSOURCE_HSE_DIV13          0x000D0300U
#define RCC_RTCCLKSOURCE_HSE_DIV14          0x000E0300U
#define RCC_RTCCLKSOURCE_HSE_DIV15          0x000F0300U
#define RCC_RTCCLKSOURCE_HSE_DIV16          0x00100300U
#define RCC_RTCCLKSOURCE_HSE_DIV17          0x00110300U
#define RCC_RTCCLKSOURCE_HSE_DIV18          0x00120300U
#define RCC_RTCCLKSOURCE_HSE_DIV19          0x00130300U
#define RCC_RTCCLKSOURCE_HSE_DIV20          0x00140300U
#define RCC_RTCCLKSOURCE_HSE_DIV21          0x00150300U
#define RCC_RTCCLKSOURCE_HSE_DIV22          0x00160300U
#define RCC_RTCCLKSOURCE_HSE_DIV23          0x00170300U
#define RCC_RTCCLKSOURCE_HSE_DIV24          0x00180300U
#define RCC_RTCCLKSOURCE_HSE_DIV25          0x00190300U
#define RCC_RTCCLKSOURCE_HSE_DIV26          0x001A0300U
#define RCC_RTCCLKSOURCE_HSE_DIV27          0x001B0300U
#define RCC_RTCCLKSOURCE_HSE_DIV28          0x001C0300U
#define RCC_RTCCLKSOURCE_HSE_DIV29          0x001D0300U
#define RCC_RTCCLKSOURCE_HSE_DIV30          0x001E0300U
#define RCC_RTCCLKSOURCE_HSE_DIV31          0x001F0300U

#define RCC_MCO1                            0x00000000U
#define RCC_MCO2                            0x00000001U

#define RCC_MCO1SOURCE_HSI                  0x00000000U
#define RCC_MCO1SOURCE_LSE                  RCC_CFGR_MCO1_0
#define RCC_MCO1SOURCE_HSE                  RCC_CFGR_MCO1_1
#define RCC_MCO1SOURCE_PLLCLK               RCC_CFGR_MCO1

#define RCC_MCODIV_1                        0x00000000U
#define RCC_MCODIV_2                        RCC_CFGR_MCO1PRE_2
#define RCC_MCODIV_3                        ((uint32_t)RCC_CFGR_MCO1PRE_0 | RCC_CFGR_MCO1PRE_2)
#define RCC_MCODIV_4                        ((uint32_t)RCC_CFGR_MCO1PRE_1 | RCC_CFGR_MCO1PRE_2)
#define RCC_MCODIV_5                        RCC_CFGR_MCO1PRE

#define RCC_IT_LSIRDY                       ((uint8_t)0x01)
#define RCC_IT_LSERDY                       ((uint8_t)0x02)
#define RCC_IT_HSIRDY                       ((uint8_t)0x04)
#define RCC_IT_HSERDY                       ((uint8_t)0x08)
#define RCC_IT_PLLRDY                       ((uint8_t)0x10)
#define RCC_IT_PLLI2SRDY                    ((uint8_t)0x20)
#define RCC_IT_CSS                          ((uint8_t)0x80)

#define RCC_FLAG_HSIRDY                     ((uint8_t)0x21)
#define RCC_FLAG_HSERDY                     ((uint8_t)0x31)
#define RCC_FLAG_PLLRDY                     ((uint8_t)0x39)
#define RCC_FLAG_PLLI2SRDY                  ((uint8_t)0x3B)
#define RCC_FLAG_LSERDY                     ((uint8_t)0x41)
#define RCC_FLAG_LSIRDY                     ((uint8_t)0x61)
#define RCC_FLAG_BORRST                     ((uint8_t)0x79)
#define RCC_FLAG_PINRST                     ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                     ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                     ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST                    ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST                    ((uint8_t)0x7E)
#define RCC_FLAG_LPWRRST                    ((uint8_t)0x7F)

#define __HAL_RCC_GPIOA_CLK_ENABLE()   
#define __HAL_RCC_GPIOB_CLK_ENABLE()   
#define __HAL_RCC_GPIOC_CLK_ENABLE()  
#define __HAL_RCC_GPIOH_CLK_ENABLE()  
#define __HAL_RCC_DMA1_CLK_ENABLE()  
#define __HAL_RCC_DMA2_CLK_ENABLE()  
#define __HAL_RCC_GPIOA_CLK_DISABLE()        
#define __HAL_RCC_GPIOB_CLK_DISABLE()        
#define __HAL_RCC_GPIOC_CLK_DISABLE()        
#define __HAL_RCC_GPIOH_CLK_DISABLE()        
#define __HAL_RCC_DMA1_CLK_DISABLE()         
#define __HAL_RCC_DMA2_CLK_DISABLE()         
#define __HAL_RCC_GPIOA_IS_CLK_ENABLED()        
#define __HAL_RCC_GPIOB_IS_CLK_ENABLED()        
#define __HAL_RCC_GPIOC_IS_CLK_ENABLED()        
#define __HAL_RCC_GPIOH_IS_CLK_ENABLED()        
#define __HAL_RCC_DMA1_IS_CLK_ENABLED()         
#define __HAL_RCC_DMA2_IS_CLK_ENABLED()         
#define __HAL_RCC_GPIOA_IS_CLK_DISABLED()       
#define __HAL_RCC_GPIOB_IS_CLK_DISABLED()       
#define __HAL_RCC_GPIOC_IS_CLK_DISABLED()       
#define __HAL_RCC_GPIOH_IS_CLK_DISABLED()       
#define __HAL_RCC_DMA1_IS_CLK_DISABLED()        
#define __HAL_RCC_DMA2_IS_CLK_DISABLED()        
#define __HAL_RCC_TIM5_CLK_ENABLE()     
#define __HAL_RCC_WWDG_CLK_ENABLE()     
#define __HAL_RCC_SPI2_CLK_ENABLE()     
#define __HAL_RCC_USART2_CLK_ENABLE()   
#define __HAL_RCC_I2C1_CLK_ENABLE()     
#define __HAL_RCC_I2C2_CLK_ENABLE()     
#define __HAL_RCC_PWR_CLK_ENABLE()     
#define __HAL_RCC_TIM5_CLK_DISABLE()   
#define __HAL_RCC_WWDG_CLK_DISABLE()   
#define __HAL_RCC_SPI2_CLK_DISABLE()   
#define __HAL_RCC_USART2_CLK_DISABLE() 
#define __HAL_RCC_I2C1_CLK_DISABLE()   
#define __HAL_RCC_I2C2_CLK_DISABLE()   
#define __HAL_RCC_PWR_CLK_DISABLE()    
#define __HAL_RCC_TIM5_IS_CLK_ENABLED()   
#define __HAL_RCC_WWDG_IS_CLK_ENABLED()   
#define __HAL_RCC_SPI2_IS_CLK_ENABLED()   
#define __HAL_RCC_USART2_IS_CLK_ENABLED() 
#define __HAL_RCC_I2C1_IS_CLK_ENABLED()   
#define __HAL_RCC_I2C2_IS_CLK_ENABLED()   
#define __HAL_RCC_PWR_IS_CLK_ENABLED()    
#define __HAL_RCC_TIM5_IS_CLK_DISABLED()   
#define __HAL_RCC_WWDG_IS_CLK_DISABLED()   
#define __HAL_RCC_SPI2_IS_CLK_DISABLED()   
#define __HAL_RCC_USART2_IS_CLK_DISABLED() 
#define __HAL_RCC_I2C1_IS_CLK_DISABLED()   
#define __HAL_RCC_I2C2_IS_CLK_DISABLED()   
#define __HAL_RCC_PWR_IS_CLK_DISABLED()    
#define __HAL_RCC_TIM1_CLK_ENABLE()     
#define __HAL_RCC_USART1_CLK_ENABLE()   
#define __HAL_RCC_USART6_CLK_ENABLE()   
#define __HAL_RCC_ADC1_CLK_ENABLE()     
#define __HAL_RCC_SPI1_CLK_ENABLE()     
#define __HAL_RCC_SYSCFG_CLK_ENABLE()   
#define __HAL_RCC_TIM9_CLK_ENABLE()     
#define __HAL_RCC_TIM11_CLK_ENABLE()    
#define __HAL_RCC_TIM1_CLK_DISABLE()   
#define __HAL_RCC_USART1_CLK_DISABLE() 
#define __HAL_RCC_USART6_CLK_DISABLE() 
#define __HAL_RCC_ADC1_CLK_DISABLE()   
#define __HAL_RCC_SPI1_CLK_DISABLE()   
#define __HAL_RCC_SYSCFG_CLK_DISABLE() 
#define __HAL_RCC_TIM9_CLK_DISABLE()   
#define __HAL_RCC_TIM11_CLK_DISABLE()  
#define __HAL_RCC_TIM1_IS_CLK_ENABLED()   
#define __HAL_RCC_USART1_IS_CLK_ENABLED() 
#define __HAL_RCC_USART6_IS_CLK_ENABLED() 
#define __HAL_RCC_ADC1_IS_CLK_ENABLED()   
#define __HAL_RCC_SPI1_IS_CLK_ENABLED()   
#define __HAL_RCC_SYSCFG_IS_CLK_ENABLED() 
#define __HAL_RCC_TIM9_IS_CLK_ENABLED()   
#define __HAL_RCC_TIM11_IS_CLK_ENABLED()  
#define __HAL_RCC_TIM1_IS_CLK_DISABLED()   
#define __HAL_RCC_USART1_IS_CLK_DISABLED() 
#define __HAL_RCC_USART6_IS_CLK_DISABLED() 
#define __HAL_RCC_ADC1_IS_CLK_DISABLED()   
#define __HAL_RCC_SPI1_IS_CLK_DISABLED()   
#define __HAL_RCC_SYSCFG_IS_CLK_DISABLED() 
#define __HAL_RCC_TIM9_IS_CLK_DISABLED()   
#define __HAL_RCC_TIM11_IS_CLK_DISABLED()  
#define __HAL_RCC_AHB1_FORCE_RESET()    
#define __HAL_RCC_GPIOA_FORCE_RESET()   
#define __HAL_RCC_GPIOB_FORCE_RESET()   
#define __HAL_RCC_GPIOC_FORCE_RESET()   
#define __HAL_RCC_GPIOH_FORCE_RESET()   
#define __HAL_RCC_DMA1_FORCE_RESET()    
#define __HAL_RCC_DMA2_FORCE_RESET()    
#define __HAL_RCC_AHB1_RELEASE_RESET()  
#define __HAL_RCC_GPIOA_RELEASE_RESET() 
#define __HAL_RCC_GPIOB_RELEASE_RESET() 
#define __HAL_RCC_GPIOC_RELEASE_RESET() 
#define __HAL_RCC_GPIOH_RELEASE_RESET() 
#define __HAL_RCC_DMA1_RELEASE_RESET()  
#define __HAL_RCC_DMA2_RELEASE_RESET()  
#define __HAL_RCC_APB1_FORCE_RESET()     
#define __HAL_RCC_TIM5_FORCE_RESET()     
#define __HAL_RCC_WWDG_FORCE_RESET()     
#define __HAL_RCC_SPI2_FORCE_RESET()     
#define __HAL_RCC_USART2_FORCE_RESET()   
#define __HAL_RCC_I2C1_FORCE_RESET()     
#define __HAL_RCC_I2C2_FORCE_RESET()     
#define __HAL_RCC_PWR_FORCE_RESET()      
#define __HAL_RCC_APB1_RELEASE_RESET()   
#define __HAL_RCC_TIM5_RELEASE_RESET()   
#define __HAL_RCC_WWDG_RELEASE_RESET()   
#define __HAL_RCC_SPI2_RELEASE_RESET()   
#define __HAL_RCC_USART2_RELEASE_RESET() 
#define __HAL_RCC_I2C1_RELEASE_RESET()   
#define __HAL_RCC_I2C2_RELEASE_RESET()   
#define __HAL_RCC_PWR_RELEASE_RESET()    
#define __HAL_RCC_APB2_FORCE_RESET()     
#define __HAL_RCC_TIM1_FORCE_RESET()     
#define __HAL_RCC_USART1_FORCE_RESET()   
#define __HAL_RCC_USART6_FORCE_RESET()   
#define __HAL_RCC_ADC_FORCE_RESET()      
#define __HAL_RCC_SPI1_FORCE_RESET()     
#define __HAL_RCC_SYSCFG_FORCE_RESET()   
#define __HAL_RCC_TIM9_FORCE_RESET()     
#define __HAL_RCC_TIM11_FORCE_RESET()    
#define __HAL_RCC_APB2_RELEASE_RESET()   
#define __HAL_RCC_TIM1_RELEASE_RESET()   
#define __HAL_RCC_USART1_RELEASE_RESET() 
#define __HAL_RCC_USART6_RELEASE_RESET() 
#define __HAL_RCC_ADC_RELEASE_RESET()    
#define __HAL_RCC_SPI1_RELEASE_RESET()   
#define __HAL_RCC_SYSCFG_RELEASE_RESET() 
#define __HAL_RCC_TIM9_RELEASE_RESET()   
#define __HAL_RCC_TIM11_RELEASE_RESET()  
#define __HAL_RCC_GPIOA_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_GPIOB_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_GPIOC_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_GPIOH_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_DMA1_CLK_SLEEP_ENABLE()     
#define __HAL_RCC_DMA2_CLK_SLEEP_ENABLE()     
#define __HAL_RCC_GPIOA_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_GPIOB_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_GPIOC_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_GPIOH_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_DMA1_CLK_SLEEP_DISABLE()    
#define __HAL_RCC_DMA2_CLK_SLEEP_DISABLE()    
#define __HAL_RCC_TIM5_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_WWDG_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_SPI2_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_USART2_CLK_SLEEP_ENABLE()  
#define __HAL_RCC_I2C1_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_I2C2_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_PWR_CLK_SLEEP_ENABLE()     
#define __HAL_RCC_TIM5_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_WWDG_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_SPI2_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_USART2_CLK_SLEEP_DISABLE() 
#define __HAL_RCC_I2C1_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_I2C2_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_PWR_CLK_SLEEP_DISABLE()    
#define __HAL_RCC_TIM1_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_USART1_CLK_SLEEP_ENABLE()  
#define __HAL_RCC_USART6_CLK_SLEEP_ENABLE()  
#define __HAL_RCC_ADC1_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_SPI1_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_SYSCFG_CLK_SLEEP_ENABLE()  
#define __HAL_RCC_TIM9_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_TIM11_CLK_SLEEP_ENABLE()   
#define __HAL_RCC_TIM1_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_USART1_CLK_SLEEP_DISABLE() 
#define __HAL_RCC_USART6_CLK_SLEEP_DISABLE() 
#define __HAL_RCC_ADC1_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_SPI1_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_SYSCFG_CLK_SLEEP_DISABLE() 
#define __HAL_RCC_TIM9_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_TIM11_CLK_SLEEP_DISABLE()  

#define __HAL_RCC_HSI_ENABLE() 
#define __HAL_RCC_HSI_DISABLE() 
#define __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(__HSICalibrationValue__) 
#define __HAL_RCC_LSI_ENABLE() 
#define __HAL_RCC_LSI_DISABLE() 
#define __HAL_RCC_HSE_CONFIG(__STATE__)                         
#define __HAL_RCC_LSE_CONFIG(__STATE__) 
#define __HAL_RCC_RTC_ENABLE() 
#define __HAL_RCC_RTC_DISABLE() 
#define __HAL_RCC_RTC_CLKPRESCALER(__RTCCLKSource__) 
#define __HAL_RCC_RTC_CONFIG(__RTCCLKSource__) 
#define __HAL_RCC_GET_RTC_SOURCE() 
#define  __HAL_RCC_GET_RTC_HSE_PRESCALER() 
#define __HAL_RCC_BACKUPRESET_FORCE() 
#define __HAL_RCC_BACKUPRESET_RELEASE() 
#define __HAL_RCC_PLL_ENABLE() 
#define __HAL_RCC_PLL_DISABLE() 
#define __HAL_RCC_PLL_PLLSOURCE_CONFIG(__PLLSOURCE__) 
#define __HAL_RCC_PLL_PLLM_CONFIG(__PLLM__) 
#define __HAL_RCC_SYSCLK_CONFIG(__RCC_SYSCLKSOURCE__) 
#define __HAL_RCC_GET_SYSCLK_SOURCE() 
#define __HAL_RCC_GET_PLL_OSCSOURCE()
#define __HAL_RCC_MCO1_CONFIG(__MCOCLKSOURCE__, __MCODIV__) 
#define __HAL_RCC_MCO2_CONFIG(__MCOCLKSOURCE__, __MCODIV__) 
#define __HAL_RCC_ENABLE_IT(__INTERRUPT__) 
#define __HAL_RCC_DISABLE_IT(__INTERRUPT__) 
#define __HAL_RCC_CLEAR_IT(__INTERRUPT__) 
#define __HAL_RCC_GET_IT(__INTERRUPT__) 
#define __HAL_RCC_CLEAR_RESET_FLAGS() 

#define RCC_FLAG_MASK  ((uint8_t)0x1FU)
#define __HAL_RCC_GET_FLAG(__FLAG__)

HAL_StatusTypeDef HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);

void     HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void     HAL_RCC_EnableCSS(void);
void     HAL_RCC_DisableCSS(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void     HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void     HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);

void HAL_RCC_NMI_IRQHandler(void);

void HAL_RCC_CSSCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* __RCC_H */

//#endif

