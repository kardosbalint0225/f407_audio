#ifdef TEST

#ifndef __RCC_EX_H
#define __RCC_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "def.h"

typedef struct
{
  uint32_t PLLState;   /*!< The new state of the PLL.
                            This parameter can be a value of @ref RCC_PLL_Config                      */
  uint32_t PLLSource;  /*!< RCC_PLLSource: PLL entry clock source.
                            This parameter must be a value of @ref RCC_PLL_Clock_Source               */
  uint32_t PLLM;       /*!< PLLM: Division factor for PLL VCO input clock.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 63    */
  uint32_t PLLN;       /*!< PLLN: Multiplication factor for PLL VCO output clock.
                            This parameter must be a number between Min_Data = 50 and Max_Data = 432 
                            except for STM32F411xE devices where the Min_Data = 192 */
  uint32_t PLLP;       /*!< PLLP: Division factor for main system clock (SYSCLK).
                            This parameter must be a value of @ref RCC_PLLP_Clock_Divider             */
  uint32_t PLLQ;       /*!< PLLQ: Division factor for OTG FS, SDIO and RNG clocks.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 15    */
}RCC_PLLInitTypeDef;

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) ||\
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)

typedef struct
{                        
  uint32_t PLLI2SN;    /*!< Specifies the multiplication factor for PLLI2S VCO output clock.
                            This parameter must be a number between Min_Data = 50 and Max_Data = 432
                            Except for STM32F411xE devices where the Min_Data = 192. 
                            This parameter will be used only when PLLI2S is selected as Clock Source I2S or SAI */
  uint32_t PLLI2SR;    /*!< Specifies the division factor for I2S clock.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 7. 
                            This parameter will be used only when PLLI2S is selected as Clock Source I2S or SAI */
}RCC_PLLI2SInitTypeDef;
 
typedef struct
{
  uint32_t PeriphClockSelection; /*!< The Extended Clock to be configured.
                                      This parameter can be a value of @ref RCCEx_Periph_Clock_Selection */
  RCC_PLLI2SInitTypeDef PLLI2S;  /*!< PLL I2S structure parameters.
                                      This parameter will be used only when PLLI2S is selected as Clock Source I2S or SAI */
  uint32_t RTCClockSelection;      /*!< Specifies RTC Clock Prescalers Selection.
                                       This parameter can be a value of @ref RCC_RTC_Clock_Source */
}RCC_PeriphCLKInitTypeDef;
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F401xC || STM32F401xE || STM32F411xE */

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx)|| defined(STM32F417xx) ||\
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) 

#define RCC_PERIPHCLK_I2S               0x00000001U
#define RCC_PERIPHCLK_RTC               0x00000002U
#define RCC_PERIPHCLK_PLLI2S            0x00000004U

#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F401xC || STM32F401xE || STM32F411xE */

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F469xx) || \
    defined(STM32F479xx) 

#define RCC_I2SCLKSOURCE_PLLI2S         0x00000000U
#define RCC_I2SCLKSOURCE_EXT            0x00000001U

#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx ||
          STM32F401xC || STM32F401xE || STM32F411xE || STM32F469xx || STM32F479xx */

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F446xx) || \
    defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || \
    defined(STM32F412Rx) || defined(STM32F413xx) || defined(STM32F423xx)

#define RCC_MCO2SOURCE_SYSCLK            0x00000000U
#define RCC_MCO2SOURCE_PLLI2SCLK         RCC_CFGR_MCO2_0
#define RCC_MCO2SOURCE_HSE               RCC_CFGR_MCO2_1
#define RCC_MCO2SOURCE_PLLCLK            RCC_CFGR_MCO2

#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx ||
          STM32F401xC || STM32F401xE || STM32F411xE || STM32F446xx || STM32F469xx || STM32F479xx || STM32F412Zx || STM32F412Vx ||
          STM32F412Rx || STM32F413xx | STM32F423xx */

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx)|| defined(STM32F417xx)

#define __HAL_RCC_BKPSRAM_CLK_ENABLE() 
#define __HAL_RCC_CCMDATARAMEN_CLK_ENABLE() 
#define __HAL_RCC_CRC_CLK_ENABLE()     
#define __HAL_RCC_GPIOD_CLK_ENABLE()   
#define __HAL_RCC_GPIOE_CLK_ENABLE()   
#define __HAL_RCC_GPIOI_CLK_ENABLE()   
#define __HAL_RCC_GPIOF_CLK_ENABLE()   
#define __HAL_RCC_GPIOG_CLK_ENABLE()   
#define __HAL_RCC_USB_OTG_HS_CLK_ENABLE() 
#define __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE()
#define __HAL_RCC_GPIOD_CLK_DISABLE()           
#define __HAL_RCC_GPIOE_CLK_DISABLE()           
#define __HAL_RCC_GPIOF_CLK_DISABLE()           
#define __HAL_RCC_GPIOG_CLK_DISABLE()           
#define __HAL_RCC_GPIOI_CLK_DISABLE()           
#define __HAL_RCC_USB_OTG_HS_CLK_DISABLE()      
#define __HAL_RCC_USB_OTG_HS_ULPI_CLK_DISABLE() 
#define __HAL_RCC_BKPSRAM_CLK_DISABLE()         
#define __HAL_RCC_CCMDATARAMEN_CLK_DISABLE()    
#define __HAL_RCC_CRC_CLK_DISABLE()             

#if defined(STM32F407xx)|| defined(STM32F417xx)

#define __HAL_RCC_ETHMAC_CLK_ENABLE()  
#define __HAL_RCC_ETHMACTX_CLK_ENABLE() 
#define __HAL_RCC_ETHMACRX_CLK_ENABLE() 
#define __HAL_RCC_ETHMACPTP_CLK_ENABLE()
#define __HAL_RCC_ETH_CLK_ENABLE()      
#define __HAL_RCC_ETHMAC_CLK_DISABLE()    
#define __HAL_RCC_ETHMACTX_CLK_DISABLE()  
#define __HAL_RCC_ETHMACRX_CLK_DISABLE()  
#define __HAL_RCC_ETHMACPTP_CLK_DISABLE() 
#define __HAL_RCC_ETH_CLK_DISABLE()       

#endif /* STM32F407xx || STM32F417xx */
  
#define __HAL_RCC_BKPSRAM_IS_CLK_ENABLED()          
#define __HAL_RCC_CCMDATARAMEN_IS_CLK_ENABLED()     
#define __HAL_RCC_CRC_IS_CLK_ENABLED()              
#define __HAL_RCC_GPIOD_IS_CLK_ENABLED()            
#define __HAL_RCC_GPIOE_IS_CLK_ENABLED()            
#define __HAL_RCC_GPIOI_IS_CLK_ENABLED()            
#define __HAL_RCC_GPIOF_IS_CLK_ENABLED()            
#define __HAL_RCC_GPIOG_IS_CLK_ENABLED()            
#define __HAL_RCC_USB_OTG_HS_IS_CLK_ENABLED()       
#define __HAL_RCC_USB_OTG_HS_ULPI_IS_CLK_ENABLED()  

#define __HAL_RCC_GPIOD_IS_CLK_DISABLED()           
#define __HAL_RCC_GPIOE_IS_CLK_DISABLED()           
#define __HAL_RCC_GPIOF_IS_CLK_DISABLED()           
#define __HAL_RCC_GPIOG_IS_CLK_DISABLED()           
#define __HAL_RCC_GPIOI_IS_CLK_DISABLED()           
#define __HAL_RCC_USB_OTG_HS_IS_CLK_DISABLED()      
#define __HAL_RCC_USB_OTG_HS_ULPI_IS_CLK_DISABLED() 
#define __HAL_RCC_BKPSRAM_IS_CLK_DISABLED()         
#define __HAL_RCC_CCMDATARAMEN_IS_CLK_DISABLED()    
#define __HAL_RCC_CRC_IS_CLK_DISABLED()             

#if defined(STM32F407xx)|| defined(STM32F417xx)

#define __HAL_RCC_ETHMAC_IS_CLK_ENABLED()     
#define __HAL_RCC_ETHMACTX_IS_CLK_ENABLED()   
#define __HAL_RCC_ETHMACRX_IS_CLK_ENABLED()   
#define __HAL_RCC_ETHMACPTP_IS_CLK_ENABLED()  
#define __HAL_RCC_ETH_IS_CLK_ENABLED()        
#define __HAL_RCC_ETHMAC_IS_CLK_DISABLED()    
#define __HAL_RCC_ETHMACTX_IS_CLK_DISABLED()  
#define __HAL_RCC_ETHMACRX_IS_CLK_DISABLED()  
#define __HAL_RCC_ETHMACPTP_IS_CLK_DISABLED() 
#define __HAL_RCC_ETH_IS_CLK_DISABLED()       

#endif /* STM32F407xx || STM32F417xx */

#define __HAL_RCC_USB_OTG_FS_CLK_ENABLE()  
#define __HAL_RCC_USB_OTG_FS_CLK_DISABLE() 
#define __HAL_RCC_RNG_CLK_ENABLE()    
#define __HAL_RCC_RNG_CLK_DISABLE()   

#if defined(STM32F407xx)|| defined(STM32F417xx) 

#define __HAL_RCC_DCMI_CLK_ENABLE()  
#define __HAL_RCC_DCMI_CLK_DISABLE() 

#endif /* STM32F407xx || STM32F417xx */

#define __HAL_RCC_USB_OTG_FS_IS_CLK_ENABLED()  
#define __HAL_RCC_USB_OTG_FS_IS_CLK_DISABLED() 
#define __HAL_RCC_RNG_IS_CLK_ENABLED()   
#define __HAL_RCC_RNG_IS_CLK_DISABLED()  

#if defined(STM32F407xx)|| defined(STM32F417xx) 
#define __HAL_RCC_DCMI_IS_CLK_ENABLED()  
#define __HAL_RCC_DCMI_IS_CLK_DISABLED() 
#endif /* STM32F407xx || STM32F417xx */

#define __HAL_RCC_FSMC_CLK_ENABLE()   
#define __HAL_RCC_FSMC_CLK_DISABLE() 
#define __HAL_RCC_FSMC_IS_CLK_ENABLED()   
#define __HAL_RCC_FSMC_IS_CLK_DISABLED()  

#define __HAL_RCC_TIM6_CLK_ENABLE()   
#define __HAL_RCC_TIM7_CLK_ENABLE()   
#define __HAL_RCC_TIM12_CLK_ENABLE()  
#define __HAL_RCC_TIM13_CLK_ENABLE()  
#define __HAL_RCC_TIM14_CLK_ENABLE()  
#define __HAL_RCC_USART3_CLK_ENABLE() 
#define __HAL_RCC_UART4_CLK_ENABLE()  
#define __HAL_RCC_UART5_CLK_ENABLE()  
#define __HAL_RCC_CAN1_CLK_ENABLE()   
#define __HAL_RCC_CAN2_CLK_ENABLE()   
#define __HAL_RCC_DAC_CLK_ENABLE()    
#define __HAL_RCC_TIM2_CLK_ENABLE()   
#define __HAL_RCC_TIM3_CLK_ENABLE()   
#define __HAL_RCC_TIM4_CLK_ENABLE()   
#define __HAL_RCC_SPI3_CLK_ENABLE()   
#define __HAL_RCC_I2C3_CLK_ENABLE()   
#define __HAL_RCC_TIM2_CLK_DISABLE()  
#define __HAL_RCC_TIM3_CLK_DISABLE()  
#define __HAL_RCC_TIM4_CLK_DISABLE()  
#define __HAL_RCC_SPI3_CLK_DISABLE()   
#define __HAL_RCC_I2C3_CLK_DISABLE()   
#define __HAL_RCC_TIM6_CLK_DISABLE()   
#define __HAL_RCC_TIM7_CLK_DISABLE()   
#define __HAL_RCC_TIM12_CLK_DISABLE()  
#define __HAL_RCC_TIM13_CLK_DISABLE()  
#define __HAL_RCC_TIM14_CLK_DISABLE()  
#define __HAL_RCC_USART3_CLK_DISABLE() 
#define __HAL_RCC_UART4_CLK_DISABLE()  
#define __HAL_RCC_UART5_CLK_DISABLE()  
#define __HAL_RCC_CAN1_CLK_DISABLE()   
#define __HAL_RCC_CAN2_CLK_DISABLE()   
#define __HAL_RCC_DAC_CLK_DISABLE()    

#define __HAL_RCC_TIM2_IS_CLK_ENABLED()   
#define __HAL_RCC_TIM3_IS_CLK_ENABLED()   
#define __HAL_RCC_TIM4_IS_CLK_ENABLED()   
#define __HAL_RCC_SPI3_IS_CLK_ENABLED()   
#define __HAL_RCC_I2C3_IS_CLK_ENABLED()   
#define __HAL_RCC_TIM6_IS_CLK_ENABLED()   
#define __HAL_RCC_TIM7_IS_CLK_ENABLED()   
#define __HAL_RCC_TIM12_IS_CLK_ENABLED()  
#define __HAL_RCC_TIM13_IS_CLK_ENABLED()  
#define __HAL_RCC_TIM14_IS_CLK_ENABLED()  
#define __HAL_RCC_USART3_IS_CLK_ENABLED() 
#define __HAL_RCC_UART4_IS_CLK_ENABLED()  
#define __HAL_RCC_UART5_IS_CLK_ENABLED()  
#define __HAL_RCC_CAN1_IS_CLK_ENABLED()   
#define __HAL_RCC_CAN2_IS_CLK_ENABLED()   
#define __HAL_RCC_DAC_IS_CLK_ENABLED()    

#define __HAL_RCC_TIM2_IS_CLK_DISABLED()   
#define __HAL_RCC_TIM3_IS_CLK_DISABLED()   
#define __HAL_RCC_TIM4_IS_CLK_DISABLED()   
#define __HAL_RCC_SPI3_IS_CLK_DISABLED()   
#define __HAL_RCC_I2C3_IS_CLK_DISABLED()   
#define __HAL_RCC_TIM6_IS_CLK_DISABLED()   
#define __HAL_RCC_TIM7_IS_CLK_DISABLED()   
#define __HAL_RCC_TIM12_IS_CLK_DISABLED()  
#define __HAL_RCC_TIM13_IS_CLK_DISABLED()  
#define __HAL_RCC_TIM14_IS_CLK_DISABLED()  
#define __HAL_RCC_USART3_IS_CLK_DISABLED() 
#define __HAL_RCC_UART4_IS_CLK_DISABLED()  
#define __HAL_RCC_UART5_IS_CLK_DISABLED()  
#define __HAL_RCC_CAN1_IS_CLK_DISABLED()   
#define __HAL_RCC_CAN2_IS_CLK_DISABLED()   
#define __HAL_RCC_DAC_IS_CLK_DISABLED()    
 
#define __HAL_RCC_TIM8_CLK_ENABLE()   
#define __HAL_RCC_ADC2_CLK_ENABLE()   
#define __HAL_RCC_ADC3_CLK_ENABLE()   
#define __HAL_RCC_SDIO_CLK_ENABLE()   
#define __HAL_RCC_SPI4_CLK_ENABLE()   
#define __HAL_RCC_TIM10_CLK_ENABLE()  

#define __HAL_RCC_SDIO_CLK_DISABLE()   
#define __HAL_RCC_SPI4_CLK_DISABLE()   
#define __HAL_RCC_TIM10_CLK_DISABLE()  
#define __HAL_RCC_TIM8_CLK_DISABLE()   
#define __HAL_RCC_ADC2_CLK_DISABLE()   
#define __HAL_RCC_ADC3_CLK_DISABLE()   

#define __HAL_RCC_SDIO_IS_CLK_ENABLED()      
#define __HAL_RCC_SPI4_IS_CLK_ENABLED()    
#define __HAL_RCC_TIM10_IS_CLK_ENABLED()   
#define __HAL_RCC_TIM8_IS_CLK_ENABLED()    
#define __HAL_RCC_ADC2_IS_CLK_ENABLED()    
#define __HAL_RCC_ADC3_IS_CLK_ENABLED()    
  
#define __HAL_RCC_SDIO_IS_CLK_DISABLED()   
#define __HAL_RCC_SPI4_IS_CLK_DISABLED()   
#define __HAL_RCC_TIM10_IS_CLK_DISABLED()  
#define __HAL_RCC_TIM8_IS_CLK_DISABLED()   
#define __HAL_RCC_ADC2_IS_CLK_DISABLED()   
#define __HAL_RCC_ADC3_IS_CLK_DISABLED()   

#define __HAL_RCC_GPIOD_FORCE_RESET()    
#define __HAL_RCC_GPIOE_FORCE_RESET()    
#define __HAL_RCC_GPIOF_FORCE_RESET()    
#define __HAL_RCC_GPIOG_FORCE_RESET()    
#define __HAL_RCC_GPIOI_FORCE_RESET()    
#define __HAL_RCC_ETHMAC_FORCE_RESET()   
#define __HAL_RCC_USB_OTG_HS_FORCE_RESET()   
#define __HAL_RCC_CRC_FORCE_RESET()     

#define __HAL_RCC_GPIOD_RELEASE_RESET()  
#define __HAL_RCC_GPIOE_RELEASE_RESET()  
#define __HAL_RCC_GPIOF_RELEASE_RESET()  
#define __HAL_RCC_GPIOG_RELEASE_RESET()  
#define __HAL_RCC_GPIOI_RELEASE_RESET()  
#define __HAL_RCC_ETHMAC_RELEASE_RESET() 
#define __HAL_RCC_USB_OTG_HS_RELEASE_RESET()  
#define __HAL_RCC_CRC_RELEASE_RESET()    

#define __HAL_RCC_AHB2_FORCE_RESET()         
#define __HAL_RCC_AHB2_RELEASE_RESET()       

#if defined(STM32F407xx)|| defined(STM32F417xx)  
#define __HAL_RCC_DCMI_FORCE_RESET()   
#define __HAL_RCC_DCMI_RELEASE_RESET() 
#endif /* STM32F407xx || STM32F417xx */
   
#define __HAL_RCC_USB_OTG_FS_FORCE_RESET()   
#define __HAL_RCC_USB_OTG_FS_RELEASE_RESET() 

#define __HAL_RCC_RNG_FORCE_RESET()    
#define __HAL_RCC_RNG_RELEASE_RESET()  
 
#define __HAL_RCC_AHB3_FORCE_RESET() 
#define __HAL_RCC_AHB3_RELEASE_RESET() 

#define __HAL_RCC_FSMC_FORCE_RESET()   
#define __HAL_RCC_FSMC_RELEASE_RESET() 

#define __HAL_RCC_TIM6_FORCE_RESET()     
#define __HAL_RCC_TIM7_FORCE_RESET()     
#define __HAL_RCC_TIM12_FORCE_RESET()    
#define __HAL_RCC_TIM13_FORCE_RESET()    
#define __HAL_RCC_TIM14_FORCE_RESET()    
#define __HAL_RCC_USART3_FORCE_RESET()   
#define __HAL_RCC_UART4_FORCE_RESET()    
#define __HAL_RCC_UART5_FORCE_RESET()    
#define __HAL_RCC_CAN1_FORCE_RESET()     
#define __HAL_RCC_CAN2_FORCE_RESET()     
#define __HAL_RCC_DAC_FORCE_RESET()      
#define __HAL_RCC_TIM2_FORCE_RESET()     
#define __HAL_RCC_TIM3_FORCE_RESET()     
#define __HAL_RCC_TIM4_FORCE_RESET()     
#define __HAL_RCC_SPI3_FORCE_RESET()     
#define __HAL_RCC_I2C3_FORCE_RESET()     

#define __HAL_RCC_TIM2_RELEASE_RESET()   
#define __HAL_RCC_TIM3_RELEASE_RESET()   
#define __HAL_RCC_TIM4_RELEASE_RESET()   
#define __HAL_RCC_SPI3_RELEASE_RESET()   
#define __HAL_RCC_I2C3_RELEASE_RESET()   
#define __HAL_RCC_TIM6_RELEASE_RESET()   
#define __HAL_RCC_TIM7_RELEASE_RESET()   
#define __HAL_RCC_TIM12_RELEASE_RESET()  
#define __HAL_RCC_TIM13_RELEASE_RESET()  
#define __HAL_RCC_TIM14_RELEASE_RESET()  
#define __HAL_RCC_USART3_RELEASE_RESET() 
#define __HAL_RCC_UART4_RELEASE_RESET()  
#define __HAL_RCC_UART5_RELEASE_RESET()  
#define __HAL_RCC_CAN1_RELEASE_RESET()   
#define __HAL_RCC_CAN2_RELEASE_RESET()   
#define __HAL_RCC_DAC_RELEASE_RESET()    

#define __HAL_RCC_TIM8_FORCE_RESET()   
#define __HAL_RCC_SDIO_FORCE_RESET()   
#define __HAL_RCC_SPI4_FORCE_RESET()   
#define __HAL_RCC_TIM10_FORCE_RESET()  
                                          
#define __HAL_RCC_SDIO_RELEASE_RESET() 
#define __HAL_RCC_SPI4_RELEASE_RESET() 
#define __HAL_RCC_TIM10_RELEASE_RESET()
#define __HAL_RCC_TIM8_RELEASE_RESET() 

#define __HAL_RCC_GPIOD_CLK_SLEEP_ENABLE()      
#define __HAL_RCC_GPIOE_CLK_SLEEP_ENABLE()      
#define __HAL_RCC_GPIOF_CLK_SLEEP_ENABLE()      
#define __HAL_RCC_GPIOG_CLK_SLEEP_ENABLE()      
#define __HAL_RCC_GPIOI_CLK_SLEEP_ENABLE()      
#define __HAL_RCC_SRAM2_CLK_SLEEP_ENABLE()      
#define __HAL_RCC_ETHMAC_CLK_SLEEP_ENABLE()     
#define __HAL_RCC_ETHMACTX_CLK_SLEEP_ENABLE()   
#define __HAL_RCC_ETHMACRX_CLK_SLEEP_ENABLE()   
#define __HAL_RCC_ETHMACPTP_CLK_SLEEP_ENABLE()  
#define __HAL_RCC_USB_OTG_HS_CLK_SLEEP_ENABLE()      
#define __HAL_RCC_USB_OTG_HS_ULPI_CLK_SLEEP_ENABLE()  
#define __HAL_RCC_CRC_CLK_SLEEP_ENABLE()      
#define __HAL_RCC_FLITF_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_SRAM1_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_BKPSRAM_CLK_SLEEP_ENABLE()  

#define __HAL_RCC_GPIOD_CLK_SLEEP_DISABLE()     
#define __HAL_RCC_GPIOE_CLK_SLEEP_DISABLE()     
#define __HAL_RCC_GPIOF_CLK_SLEEP_DISABLE()     
#define __HAL_RCC_GPIOG_CLK_SLEEP_DISABLE()     
#define __HAL_RCC_GPIOI_CLK_SLEEP_DISABLE()     
#define __HAL_RCC_SRAM2_CLK_SLEEP_DISABLE()     
#define __HAL_RCC_ETHMAC_CLK_SLEEP_DISABLE()    
#define __HAL_RCC_ETHMACTX_CLK_SLEEP_DISABLE()  
#define __HAL_RCC_ETHMACRX_CLK_SLEEP_DISABLE()  
#define __HAL_RCC_ETHMACPTP_CLK_SLEEP_DISABLE() 
#define __HAL_RCC_USB_OTG_HS_CLK_SLEEP_DISABLE()     
#define __HAL_RCC_USB_OTG_HS_ULPI_CLK_SLEEP_DISABLE() 
#define __HAL_RCC_CRC_CLK_SLEEP_DISABLE()       
#define __HAL_RCC_FLITF_CLK_SLEEP_DISABLE()     
#define __HAL_RCC_SRAM1_CLK_SLEEP_DISABLE()     
#define __HAL_RCC_BKPSRAM_CLK_SLEEP_DISABLE()   

#define __HAL_RCC_USB_OTG_FS_CLK_SLEEP_ENABLE()  
#define __HAL_RCC_USB_OTG_FS_CLK_SLEEP_DISABLE() 

#define __HAL_RCC_RNG_CLK_SLEEP_ENABLE()   
#define __HAL_RCC_RNG_CLK_SLEEP_DISABLE()  

#if defined(STM32F407xx)|| defined(STM32F417xx) 
#define __HAL_RCC_DCMI_CLK_SLEEP_ENABLE()  
#define __HAL_RCC_DCMI_CLK_SLEEP_DISABLE() 
#endif /* STM32F407xx || STM32F417xx */

#define __HAL_RCC_FSMC_CLK_SLEEP_ENABLE()  
#define __HAL_RCC_FSMC_CLK_SLEEP_DISABLE() 

#define __HAL_RCC_TIM6_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_TIM7_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_TIM12_CLK_SLEEP_ENABLE()   
#define __HAL_RCC_TIM13_CLK_SLEEP_ENABLE()   
#define __HAL_RCC_TIM14_CLK_SLEEP_ENABLE()   
#define __HAL_RCC_USART3_CLK_SLEEP_ENABLE()  
#define __HAL_RCC_UART4_CLK_SLEEP_ENABLE()   
#define __HAL_RCC_UART5_CLK_SLEEP_ENABLE()   
#define __HAL_RCC_CAN1_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_CAN2_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_DAC_CLK_SLEEP_ENABLE()     
#define __HAL_RCC_TIM2_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_TIM3_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_TIM4_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_SPI3_CLK_SLEEP_ENABLE()    
#define __HAL_RCC_I2C3_CLK_SLEEP_ENABLE()    

#define __HAL_RCC_TIM2_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_TIM3_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_TIM4_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_SPI3_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_I2C3_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_TIM6_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_TIM7_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_TIM12_CLK_SLEEP_DISABLE()  
#define __HAL_RCC_TIM13_CLK_SLEEP_DISABLE()  
#define __HAL_RCC_TIM14_CLK_SLEEP_DISABLE()  
#define __HAL_RCC_USART3_CLK_SLEEP_DISABLE() 
#define __HAL_RCC_UART4_CLK_SLEEP_DISABLE()  
#define __HAL_RCC_UART5_CLK_SLEEP_DISABLE()  
#define __HAL_RCC_CAN1_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_CAN2_CLK_SLEEP_DISABLE()   
#define __HAL_RCC_DAC_CLK_SLEEP_DISABLE()    

#define __HAL_RCC_TIM8_CLK_SLEEP_ENABLE() 
#define __HAL_RCC_ADC2_CLK_SLEEP_ENABLE() 
#define __HAL_RCC_ADC3_CLK_SLEEP_ENABLE() 
#define __HAL_RCC_SDIO_CLK_SLEEP_ENABLE() 
#define __HAL_RCC_SPI4_CLK_SLEEP_ENABLE() 
#define __HAL_RCC_TIM10_CLK_SLEEP_ENABLE()

#define __HAL_RCC_SDIO_CLK_SLEEP_DISABLE() 
#define __HAL_RCC_SPI4_CLK_SLEEP_DISABLE() 
#define __HAL_RCC_TIM10_CLK_SLEEP_DISABLE()
#define __HAL_RCC_TIM8_CLK_SLEEP_DISABLE() 
#define __HAL_RCC_ADC2_CLK_SLEEP_DISABLE() 
#define __HAL_RCC_ADC3_CLK_SLEEP_DISABLE() 

#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx */

#define __HAL_RCC_PLL_CONFIG(__RCC_PLLSource__, __PLLM__, __PLLN__, __PLLP__, __PLLQ__) 

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F446xx) || \
    defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || \
    defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)

#define __HAL_RCC_PLLI2S_ENABLE() 
#define __HAL_RCC_PLLI2S_DISABLE() 

#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx ||
          STM32F401xC || STM32F401xE || STM32F411xE || STM32F446xx || STM32F469xx || STM32F479xx || STM32F412Zx || STM32F412Vx || 
          STM32F412Rx || STM32F412Cx */

#define __HAL_RCC_PLLI2S_CONFIG(__PLLI2SN__, __PLLI2SR__)     

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) ||\
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) ||\
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F469xx) ||\
    defined(STM32F479xx)

#define __HAL_RCC_I2S_CONFIG(__SOURCE__) 
#define __HAL_RCC_GET_I2S_SOURCE() 
#endif /* STM32F40xxx || STM32F41xxx || STM32F42xxx || STM32F43xxx || STM32F469xx || STM32F479xx */
                                 
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);

uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);

#if defined(RCC_PLLI2S_SUPPORT)
HAL_StatusTypeDef HAL_RCCEx_EnablePLLI2S(RCC_PLLI2SInitTypeDef  *PLLI2SInit);
HAL_StatusTypeDef HAL_RCCEx_DisablePLLI2S(void);
#endif /* RCC_PLLI2S_SUPPORT */

#ifdef __cplusplus
}
#endif

#endif /* __RCC_EX_H */

#endif

