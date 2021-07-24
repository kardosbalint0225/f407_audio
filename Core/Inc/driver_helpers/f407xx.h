#ifndef __F407xx_H
#define __F407xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

#define __CM4_REV                 0x0001U  				/*!< Core revision r0p1                            */
#define __MPU_PRESENT             1U       				/*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4U       				/*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0U       				/*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1U       				/*!< FPU present                                   */

typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ********************************************************************************/
  NonMaskableInt_IRQn         = -14,    				/*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    				/*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    				/*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    				/*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     				/*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     				/*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     				/*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     				/*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **************************************************************************************/
  WWDG_IRQn                   = 0,      				/*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      				/*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      				/*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      				/*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      				/*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      				/*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      				/*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      				/*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      				/*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      				/*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     				/*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     				/*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     				/*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     				/*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     				/*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     				/*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     				/*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     				/*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     				/*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     				/*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     				/*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     				/*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     				/*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     				/*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     				/*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     				/*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     				/*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     				/*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     				/*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     				/*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     				/*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     				/*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     				/*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     				/*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     				/*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     				/*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     				/*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     				/*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     				/*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     				/*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     				/*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     				/*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     				/*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     				/*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     				/*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     				/*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     				/*!< TIM8 Capture Compare global interrupt                             */
  DMA1_Stream7_IRQn           = 47,     				/*!< DMA1 Stream7 Interrupt                                            */
  FSMC_IRQn                   = 48,     				/*!< FSMC global Interrupt                                             */
  SDIO_IRQn                   = 49,     				/*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     				/*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     				/*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     				/*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     				/*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     				/*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     				/*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     				/*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     				/*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     				/*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     				/*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     				/*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     				/*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     				/*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     				/*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     				/*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     				/*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     				/*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     				/*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     				/*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     				/*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     				/*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     				/*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     				/*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     				/*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     				/*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     				/*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     				/*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     				/*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     				/*!< DCMI global interrupt                                             */
  RNG_IRQn                    = 80,     				/*!< RNG global Interrupt                                              */
  FPU_IRQn                    = 81      				/*!< FPU global interrupt                                               */
} IRQn_Type;
/* Legacy define */
#define  HASH_RNG_IRQn      RNG_IRQn

#include "cm4.h"             /* Cortex-M4 processor and core peripherals */
//#include "system_stm32f4xx.h"
#include <stdint.h>

typedef struct
{
  __IO uint32_t SR;     								/*!< ADC status register,                         Address offset: 0x00 */
  __IO uint32_t CR1;    								/*!< ADC control register 1,                      Address offset: 0x04 */
  __IO uint32_t CR2;    								/*!< ADC control register 2,                      Address offset: 0x08 */
  __IO uint32_t SMPR1;  								/*!< ADC sample time register 1,                  Address offset: 0x0C */
  __IO uint32_t SMPR2;  								/*!< ADC sample time register 2,                  Address offset: 0x10 */
  __IO uint32_t JOFR1;  								/*!< ADC injected channel data offset register 1, Address offset: 0x14 */
  __IO uint32_t JOFR2;  								/*!< ADC injected channel data offset register 2, Address offset: 0x18 */
  __IO uint32_t JOFR3;  								/*!< ADC injected channel data offset register 3, Address offset: 0x1C */
  __IO uint32_t JOFR4;  								/*!< ADC injected channel data offset register 4, Address offset: 0x20 */
  __IO uint32_t HTR;    								/*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
  __IO uint32_t LTR;    								/*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
  __IO uint32_t SQR1;   								/*!< ADC regular sequence register 1,             Address offset: 0x2C */
  __IO uint32_t SQR2;   								/*!< ADC regular sequence register 2,             Address offset: 0x30 */
  __IO uint32_t SQR3;   								/*!< ADC regular sequence register 3,             Address offset: 0x34 */
  __IO uint32_t JSQR;   								/*!< ADC injected sequence register,              Address offset: 0x38*/
  __IO uint32_t JDR1;   								/*!< ADC injected data register 1,                Address offset: 0x3C */
  __IO uint32_t JDR2;   								/*!< ADC injected data register 2,                Address offset: 0x40 */
  __IO uint32_t JDR3;   								/*!< ADC injected data register 3,                Address offset: 0x44 */
  __IO uint32_t JDR4;   								/*!< ADC injected data register 4,                Address offset: 0x48 */
  __IO uint32_t DR;     								/*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_TypeDef;

typedef struct
{
  __IO uint32_t CSR;    								/*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
  __IO uint32_t CCR;    								/*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
  __IO uint32_t CDR;    								/*!< ADC common regular data register for dual
															AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;

typedef struct
{
  __IO uint32_t TIR;  									/*!< CAN TX mailbox identifier register */
  __IO uint32_t TDTR; 									/*!< CAN mailbox data length control and time stamp register */
  __IO uint32_t TDLR; 									/*!< CAN mailbox data low register */
  __IO uint32_t TDHR; 									/*!< CAN mailbox data high register */
} CAN_TxMailBox_TypeDef;

typedef struct
{
  __IO uint32_t RIR;  									/*!< CAN receive FIFO mailbox identifier register */
  __IO uint32_t RDTR; 									/*!< CAN receive FIFO mailbox data length control and time stamp register */
  __IO uint32_t RDLR; 									/*!< CAN receive FIFO mailbox data low register */
  __IO uint32_t RDHR; 									/*!< CAN receive FIFO mailbox data high register */
} CAN_FIFOMailBox_TypeDef;

typedef struct
{
  __IO uint32_t FR1; 									/*!< CAN Filter bank register 1 */
  __IO uint32_t FR2; 									/*!< CAN Filter bank register 1 */
} CAN_FilterRegister_TypeDef;

typedef struct
{
  __IO uint32_t              MCR;                 		/*!< CAN master control register,         Address offset: 0x00          */
  __IO uint32_t              MSR;                 		/*!< CAN master status register,          Address offset: 0x04          */
  __IO uint32_t              TSR;                 		/*!< CAN transmit status register,        Address offset: 0x08          */
  __IO uint32_t              RF0R;                		/*!< CAN receive FIFO 0 register,         Address offset: 0x0C          */
  __IO uint32_t              RF1R;                		/*!< CAN receive FIFO 1 register,         Address offset: 0x10          */
  __IO uint32_t              IER;                 		/*!< CAN interrupt enable register,       Address offset: 0x14          */
  __IO uint32_t              ESR;                 		/*!< CAN error status register,           Address offset: 0x18          */
  __IO uint32_t              BTR;                 		/*!< CAN bit timing register,             Address offset: 0x1C          */
  uint32_t                   RESERVED0[88];       		/*!< Reserved, 0x020 - 0x17F                                            */
  CAN_TxMailBox_TypeDef      sTxMailBox[3];       		/*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];     		/*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
  uint32_t                   RESERVED1[12];       		/*!< Reserved, 0x1D0 - 0x1FF                                            */
  __IO uint32_t              FMR;                 		/*!< CAN filter master register,          Address offset: 0x200         */
  __IO uint32_t              FM1R;                		/*!< CAN filter mode register,            Address offset: 0x204         */
  uint32_t                   RESERVED2;           		/*!< Reserved, 0x208                                                    */
  __IO uint32_t              FS1R;                		/*!< CAN filter scale register,           Address offset: 0x20C         */
  uint32_t                   RESERVED3;           		/*!< Reserved, 0x210                                                    */
  __IO uint32_t              FFA1R;               		/*!< CAN filter FIFO assignment register, Address offset: 0x214         */
  uint32_t                   RESERVED4;           		/*!< Reserved, 0x218                                                    */
  __IO uint32_t              FA1R;                		/*!< CAN filter activation register,      Address offset: 0x21C         */
  uint32_t                   RESERVED5[8];        		/*!< Reserved, 0x220-0x23F                                              */ 
  CAN_FilterRegister_TypeDef sFilterRegister[28]; 		/*!< CAN Filter Register,                 Address offset: 0x240-0x31C   */
} CAN_TypeDef;

typedef struct
{
  __IO uint32_t DR;         							/*!< CRC Data register,             Address offset: 0x00 */
  __IO uint8_t  IDR;        							/*!< CRC Independent data register, Address offset: 0x04 */
  uint8_t       RESERVED0;  							/*!< Reserved, 0x05                                      */
  uint16_t      RESERVED1;  							/*!< Reserved, 0x06                                      */
  __IO uint32_t CR;         							/*!< CRC Control register,          Address offset: 0x08 */
} CRC_TypeDef;

typedef struct
{
  __IO uint32_t CR;       								/*!< DAC control register,                                    Address offset: 0x00 */
  __IO uint32_t SWTRIGR;  								/*!< DAC software trigger register,                           Address offset: 0x04 */
  __IO uint32_t DHR12R1;  								/*!< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08 */
  __IO uint32_t DHR12L1;  								/*!< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C */
  __IO uint32_t DHR8R1;   								/*!< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10 */
  __IO uint32_t DHR12R2;  								/*!< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14 */
  __IO uint32_t DHR12L2;  								/*!< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18 */
  __IO uint32_t DHR8R2;   								/*!< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C */
  __IO uint32_t DHR12RD;  								/*!< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20 */
  __IO uint32_t DHR12LD;  								/*!< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24 */
  __IO uint32_t DHR8RD;   								/*!< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28 */
  __IO uint32_t DOR1;     								/*!< DAC channel1 data output register,                       Address offset: 0x2C */
  __IO uint32_t DOR2;     								/*!< DAC channel2 data output register,                       Address offset: 0x30 */
  __IO uint32_t SR;       								/*!< DAC status register,                                     Address offset: 0x34 */
} DAC_TypeDef;

typedef struct
{
  __IO uint32_t IDCODE;  								/*!< MCU device ID code,               Address offset: 0x00 */
  __IO uint32_t CR;      								/*!< Debug MCU configuration register, Address offset: 0x04 */
  __IO uint32_t APB1FZ;  								/*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
  __IO uint32_t APB2FZ;  								/*!< Debug MCU APB2 freeze register,   Address offset: 0x0C */
} DBGMCU_TypeDef;

typedef struct
{
  __IO uint32_t CR;       								/*!< DCMI control register 1,                       Address offset: 0x00 */
  __IO uint32_t SR;       								/*!< DCMI status register,                          Address offset: 0x04 */
  __IO uint32_t RISR;     								/*!< DCMI raw interrupt status register,            Address offset: 0x08 */
  __IO uint32_t IER;      								/*!< DCMI interrupt enable register,                Address offset: 0x0C */
  __IO uint32_t MISR;     								/*!< DCMI masked interrupt status register,         Address offset: 0x10 */
  __IO uint32_t ICR;      								/*!< DCMI interrupt clear register,                 Address offset: 0x14 */
  __IO uint32_t ESCR;     								/*!< DCMI embedded synchronization code register,   Address offset: 0x18 */
  __IO uint32_t ESUR;     								/*!< DCMI embedded synchronization unmask register, Address offset: 0x1C */
  __IO uint32_t CWSTRTR;  								/*!< DCMI crop window start,                        Address offset: 0x20 */
  __IO uint32_t CWSIZER;  								/*!< DCMI crop window size,                         Address offset: 0x24 */
  __IO uint32_t DR;       								/*!< DCMI data register,                            Address offset: 0x28 */
} DCMI_TypeDef;

typedef struct
{
  __IO uint32_t CR;     								/*!< DMA stream x configuration register      */
  __IO uint32_t NDTR;   								/*!< DMA stream x number of data register     */
  __IO uint32_t PAR;    								/*!< DMA stream x peripheral address register */
  __IO uint32_t M0AR;   								/*!< DMA stream x memory 0 address register   */
  __IO uint32_t M1AR;   								/*!< DMA stream x memory 1 address register   */
  __IO uint32_t FCR;    								/*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;

typedef struct
{
  __IO uint32_t LISR;   								/*!< DMA low interrupt status register,      Address offset: 0x00 */
  __IO uint32_t HISR;   								/*!< DMA high interrupt status register,     Address offset: 0x04 */
  __IO uint32_t LIFCR;  								/*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
  __IO uint32_t HIFCR;  								/*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;

typedef struct
{
  __IO uint32_t MACCR;
  __IO uint32_t MACFFR;
  __IO uint32_t MACHTHR;
  __IO uint32_t MACHTLR;
  __IO uint32_t MACMIIAR;
  __IO uint32_t MACMIIDR;
  __IO uint32_t MACFCR;
  __IO uint32_t MACVLANTR;             /*    8 */
  uint32_t      RESERVED0[2];
  __IO uint32_t MACRWUFFR;             /*   11 */
  __IO uint32_t MACPMTCSR;
  uint32_t      RESERVED1;
  __IO uint32_t MACDBGR;
  __IO uint32_t MACSR;                 /*   15 */
  __IO uint32_t MACIMR;
  __IO uint32_t MACA0HR;
  __IO uint32_t MACA0LR;
  __IO uint32_t MACA1HR;
  __IO uint32_t MACA1LR;
  __IO uint32_t MACA2HR;
  __IO uint32_t MACA2LR;
  __IO uint32_t MACA3HR;
  __IO uint32_t MACA3LR;               /*   24 */
  uint32_t      RESERVED2[40];
  __IO uint32_t MMCCR;                 /*   65 */
  __IO uint32_t MMCRIR;
  __IO uint32_t MMCTIR;
  __IO uint32_t MMCRIMR;
  __IO uint32_t MMCTIMR;               /*   69 */
  uint32_t      RESERVED3[14];
  __IO uint32_t MMCTGFSCCR;            /*   84 */
  __IO uint32_t MMCTGFMSCCR;
  uint32_t      RESERVED4[5];
  __IO uint32_t MMCTGFCR;
  uint32_t      RESERVED5[10];
  __IO uint32_t MMCRFCECR;
  __IO uint32_t MMCRFAECR;
  uint32_t      RESERVED6[10];
  __IO uint32_t MMCRGUFCR;
  uint32_t      RESERVED7[334];
  __IO uint32_t PTPTSCR;
  __IO uint32_t PTPSSIR;
  __IO uint32_t PTPTSHR;
  __IO uint32_t PTPTSLR;
  __IO uint32_t PTPTSHUR;
  __IO uint32_t PTPTSLUR;
  __IO uint32_t PTPTSAR;
  __IO uint32_t PTPTTHR;
  __IO uint32_t PTPTTLR;
  __IO uint32_t RESERVED8;
  __IO uint32_t PTPTSSR;
  uint32_t      RESERVED9[565];
  __IO uint32_t DMABMR;
  __IO uint32_t DMATPDR;
  __IO uint32_t DMARPDR;
  __IO uint32_t DMARDLAR;
  __IO uint32_t DMATDLAR;
  __IO uint32_t DMASR;
  __IO uint32_t DMAOMR;
  __IO uint32_t DMAIER;
  __IO uint32_t DMAMFBOCR;
  __IO uint32_t DMARSWTR;
  uint32_t      RESERVED10[8];
  __IO uint32_t DMACHTDR;
  __IO uint32_t DMACHRDR;
  __IO uint32_t DMACHTBAR;
  __IO uint32_t DMACHRBAR;
} ETH_TypeDef;

typedef struct
{
  __IO uint32_t IMR;    								/*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  __IO uint32_t EMR;    								/*!< EXTI Event mask register,                Address offset: 0x04 */
  __IO uint32_t RTSR;   								/*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  __IO uint32_t FTSR;   								/*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  __IO uint32_t SWIER;  								/*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  __IO uint32_t PR;     								/*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;

typedef struct
{
  __IO uint32_t ACR;      								/*!< FLASH access control register,   Address offset: 0x00 */
  __IO uint32_t KEYR;     								/*!< FLASH key register,              Address offset: 0x04 */
  __IO uint32_t OPTKEYR;  								/*!< FLASH option key register,       Address offset: 0x08 */
  __IO uint32_t SR;       								/*!< FLASH status register,           Address offset: 0x0C */
  __IO uint32_t CR;       								/*!< FLASH control register,          Address offset: 0x10 */
  __IO uint32_t OPTCR;    								/*!< FLASH option control register ,  Address offset: 0x14 */
  __IO uint32_t OPTCR1;   								/*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_TypeDef;

typedef struct
{
  __IO uint32_t BTCR[8];    							/*!< NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C */   
} FSMC_Bank1_TypeDef;

typedef struct
{
  __IO uint32_t BWTR[7];    							/*!< NOR/PSRAM write timing registers, Address offset: 0x104-0x11C */
} FSMC_Bank1E_TypeDef;
  
typedef struct
{
  __IO uint32_t PCR2;       							/*!< NAND Flash control register 2,                       Address offset: 0x60 */
  __IO uint32_t SR2;        							/*!< NAND Flash FIFO status and interrupt register 2,     Address offset: 0x64 */
  __IO uint32_t PMEM2;      							/*!< NAND Flash Common memory space timing register 2,    Address offset: 0x68 */
  __IO uint32_t PATT2;      							/*!< NAND Flash Attribute memory space timing register 2, Address offset: 0x6C */
  uint32_t      RESERVED0;  							/*!< Reserved, 0x70                                                            */
  __IO uint32_t ECCR2;      							/*!< NAND Flash ECC result registers 2,                   Address offset: 0x74 */
  uint32_t      RESERVED1;  							/*!< Reserved, 0x78                                                            */
  uint32_t      RESERVED2;  							/*!< Reserved, 0x7C                                                            */
  __IO uint32_t PCR3;       							/*!< NAND Flash control register 3,                       Address offset: 0x80 */
  __IO uint32_t SR3;        							/*!< NAND Flash FIFO status and interrupt register 3,     Address offset: 0x84 */
  __IO uint32_t PMEM3;      							/*!< NAND Flash Common memory space timing register 3,    Address offset: 0x88 */
  __IO uint32_t PATT3;      							/*!< NAND Flash Attribute memory space timing register 3, Address offset: 0x8C */
  uint32_t      RESERVED3;  							/*!< Reserved, 0x90                                                            */
  __IO uint32_t ECCR3;      							/*!< NAND Flash ECC result registers 3,                   Address offset: 0x94 */
} FSMC_Bank2_3_TypeDef;

typedef struct
{
  __IO uint32_t PCR4;       							/*!< PC Card  control register 4,                       Address offset: 0xA0 */
  __IO uint32_t SR4;        							/*!< PC Card  FIFO status and interrupt register 4,     Address offset: 0xA4 */
  __IO uint32_t PMEM4;      							/*!< PC Card  Common memory space timing register 4,    Address offset: 0xA8 */
  __IO uint32_t PATT4;      							/*!< PC Card  Attribute memory space timing register 4, Address offset: 0xAC */
  __IO uint32_t PIO4;       							/*!< PC Card  I/O space timing register 4,              Address offset: 0xB0 */
} FSMC_Bank4_TypeDef; 

typedef struct
{
  __IO uint32_t MODER;    								/*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   								/*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  								/*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    								/*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      								/*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      								/*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     								/*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  __IO uint32_t LCKR;     								/*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   								/*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

typedef struct
{
  __IO uint32_t MEMRMP;       							/*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  __IO uint32_t PMC;          							/*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  __IO uint32_t EXTICR[4];    							/*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t      RESERVED[2];  							/*!< Reserved, 0x18-0x1C                                                          */
  __IO uint32_t CMPCR;        							/*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;

typedef struct
{
  __IO uint32_t CR1;        							/*!< I2C Control register 1,     Address offset: 0x00 */
  __IO uint32_t CR2;        							/*!< I2C Control register 2,     Address offset: 0x04 */
  __IO uint32_t OAR1;       							/*!< I2C Own address register 1, Address offset: 0x08 */
  __IO uint32_t OAR2;       							/*!< I2C Own address register 2, Address offset: 0x0C */
  __IO uint32_t DR;         							/*!< I2C Data register,          Address offset: 0x10 */
  __IO uint32_t SR1;        							/*!< I2C Status register 1,      Address offset: 0x14 */
  __IO uint32_t SR2;        							/*!< I2C Status register 2,      Address offset: 0x18 */
  __IO uint32_t CCR;        							/*!< I2C Clock control register, Address offset: 0x1C */
  __IO uint32_t TRISE;      							/*!< I2C TRISE register,         Address offset: 0x20 */
} I2C_TypeDef;

typedef struct
{
  __IO uint32_t KR;   									/*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;   									/*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;  									/*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;   									/*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_TypeDef;

typedef struct
{
  __IO uint32_t CR;   									/*!< PWR power control register,        Address offset: 0x00 */
  __IO uint32_t CSR;  									/*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;

typedef struct
{
  __IO uint32_t CR;            							/*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       							/*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          							/*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           							/*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      							/*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHB2RSTR;      							/*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __IO uint32_t AHB3RSTR;      							/*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     							/*!< Reserved, 0x1C                                                                    */
  __IO uint32_t APB1RSTR;      							/*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      							/*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  							/*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       							/*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       							/*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       							/*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     							/*!< Reserved, 0x3C                                                                    */
  __IO uint32_t APB1ENR;       							/*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       							/*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  							/*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     							/*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __IO uint32_t AHB2LPENR;     							/*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __IO uint32_t AHB3LPENR;     							/*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     							/*!< Reserved, 0x5C                                                                    */
  __IO uint32_t APB1LPENR;     							/*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     							/*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  							/*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          							/*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           							/*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  							/*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         							/*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __IO uint32_t PLLI2SCFGR;    							/*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_TypeDef;

typedef struct
{
  __IO uint32_t TR;      								/*!< RTC time register,                                        Address offset: 0x00 */
  __IO uint32_t DR;      								/*!< RTC date register,                                        Address offset: 0x04 */
  __IO uint32_t CR;      								/*!< RTC control register,                                     Address offset: 0x08 */
  __IO uint32_t ISR;     								/*!< RTC initialization and status register,                   Address offset: 0x0C */
  __IO uint32_t PRER;    								/*!< RTC prescaler register,                                   Address offset: 0x10 */
  __IO uint32_t WUTR;    								/*!< RTC wakeup timer register,                                Address offset: 0x14 */
  __IO uint32_t CALIBR;  								/*!< RTC calibration register,                                 Address offset: 0x18 */
  __IO uint32_t ALRMAR;  								/*!< RTC alarm A register,                                     Address offset: 0x1C */
  __IO uint32_t ALRMBR;  								/*!< RTC alarm B register,                                     Address offset: 0x20 */
  __IO uint32_t WPR;     								/*!< RTC write protection register,                            Address offset: 0x24 */
  __IO uint32_t SSR;     								/*!< RTC sub second register,                                  Address offset: 0x28 */
  __IO uint32_t SHIFTR;  								/*!< RTC shift control register,                               Address offset: 0x2C */
  __IO uint32_t TSTR;    								/*!< RTC time stamp time register,                             Address offset: 0x30 */
  __IO uint32_t TSDR;    								/*!< RTC time stamp date register,                             Address offset: 0x34 */
  __IO uint32_t TSSSR;   								/*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
  __IO uint32_t CALR;    								/*!< RTC calibration register,                                 Address offset: 0x3C */
  __IO uint32_t TAFCR;   								/*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
  __IO uint32_t ALRMASSR;								/*!< RTC alarm A sub second register,                          Address offset: 0x44 */
  __IO uint32_t ALRMBSSR;								/*!< RTC alarm B sub second register,                          Address offset: 0x48 */
  uint32_t RESERVED7;    								/*!< Reserved, 0x4C                                                                 */
  __IO uint32_t BKP0R;   								/*!< RTC backup register 1,                                    Address offset: 0x50 */
  __IO uint32_t BKP1R;   								/*!< RTC backup register 1,                                    Address offset: 0x54 */
  __IO uint32_t BKP2R;   								/*!< RTC backup register 2,                                    Address offset: 0x58 */
  __IO uint32_t BKP3R;   								/*!< RTC backup register 3,                                    Address offset: 0x5C */
  __IO uint32_t BKP4R;   								/*!< RTC backup register 4,                                    Address offset: 0x60 */
  __IO uint32_t BKP5R;   								/*!< RTC backup register 5,                                    Address offset: 0x64 */
  __IO uint32_t BKP6R;   								/*!< RTC backup register 6,                                    Address offset: 0x68 */
  __IO uint32_t BKP7R;   								/*!< RTC backup register 7,                                    Address offset: 0x6C */
  __IO uint32_t BKP8R;   								/*!< RTC backup register 8,                                    Address offset: 0x70 */
  __IO uint32_t BKP9R;   								/*!< RTC backup register 9,                                    Address offset: 0x74 */
  __IO uint32_t BKP10R;  								/*!< RTC backup register 10,                                   Address offset: 0x78 */
  __IO uint32_t BKP11R;  								/*!< RTC backup register 11,                                   Address offset: 0x7C */
  __IO uint32_t BKP12R;  								/*!< RTC backup register 12,                                   Address offset: 0x80 */
  __IO uint32_t BKP13R;  								/*!< RTC backup register 13,                                   Address offset: 0x84 */
  __IO uint32_t BKP14R;  								/*!< RTC backup register 14,                                   Address offset: 0x88 */
  __IO uint32_t BKP15R;  								/*!< RTC backup register 15,                                   Address offset: 0x8C */
  __IO uint32_t BKP16R;  								/*!< RTC backup register 16,                                   Address offset: 0x90 */
  __IO uint32_t BKP17R;  								/*!< RTC backup register 17,                                   Address offset: 0x94 */
  __IO uint32_t BKP18R;  								/*!< RTC backup register 18,                                   Address offset: 0x98 */
  __IO uint32_t BKP19R;  								/*!< RTC backup register 19,                                   Address offset: 0x9C */
} RTC_TypeDef;

typedef struct
{
  __IO uint32_t POWER;                 					/*!< SDIO power control register,    Address offset: 0x00 */
  __IO uint32_t CLKCR;                 					/*!< SDI clock control register,     Address offset: 0x04 */
  __IO uint32_t ARG;                   					/*!< SDIO argument register,         Address offset: 0x08 */
  __IO uint32_t CMD;                   					/*!< SDIO command register,          Address offset: 0x0C */
  __IO const uint32_t  RESPCMD;        					/*!< SDIO command response register, Address offset: 0x10 */
  __IO const uint32_t  RESP1;          					/*!< SDIO response 1 register,       Address offset: 0x14 */
  __IO const uint32_t  RESP2;          					/*!< SDIO response 2 register,       Address offset: 0x18 */
  __IO const uint32_t  RESP3;          					/*!< SDIO response 3 register,       Address offset: 0x1C */
  __IO const uint32_t  RESP4;          					/*!< SDIO response 4 register,       Address offset: 0x20 */
  __IO uint32_t DTIMER;                					/*!< SDIO data timer register,       Address offset: 0x24 */
  __IO uint32_t DLEN;                  					/*!< SDIO data length register,      Address offset: 0x28 */
  __IO uint32_t DCTRL;                 					/*!< SDIO data control register,     Address offset: 0x2C */
  __IO const uint32_t  DCOUNT;         					/*!< SDIO data counter register,     Address offset: 0x30 */
  __IO const uint32_t  STA;            					/*!< SDIO status register,           Address offset: 0x34 */
  __IO uint32_t ICR;                   					/*!< SDIO interrupt clear register,  Address offset: 0x38 */
  __IO uint32_t MASK;                  					/*!< SDIO mask register,             Address offset: 0x3C */
  uint32_t      RESERVED0[2];          					/*!< Reserved, 0x40-0x44                                  */
  __IO const uint32_t  FIFOCNT;        					/*!< SDIO FIFO counter register,     Address offset: 0x48 */
  uint32_t      RESERVED1[13];         					/*!< Reserved, 0x4C-0x7C                                  */
  __IO uint32_t FIFO;                  					/*!< SDIO data FIFO register,        Address offset: 0x80 */
} SDIO_TypeDef;

typedef struct
{
  __IO uint32_t CR1;        							/*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  __IO uint32_t CR2;        							/*!< SPI control register 2,                             Address offset: 0x04 */
  __IO uint32_t SR;         							/*!< SPI status register,                                Address offset: 0x08 */
  __IO uint32_t DR;         							/*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t CRCPR;      							/*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  __IO uint32_t RXCRCR;     							/*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  __IO uint32_t TXCRCR;     							/*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  __IO uint32_t I2SCFGR;    							/*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  __IO uint32_t I2SPR;      							/*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef;

typedef struct
{
  __IO uint32_t CR1;         							/*!< TIM control register 1,              Address offset: 0x00 */
  __IO uint32_t CR2;         							/*!< TIM control register 2,              Address offset: 0x04 */
  __IO uint32_t SMCR;        							/*!< TIM slave mode control register,     Address offset: 0x08 */
  __IO uint32_t DIER;        							/*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __IO uint32_t SR;          							/*!< TIM status register,                 Address offset: 0x10 */
  __IO uint32_t EGR;         							/*!< TIM event generation register,       Address offset: 0x14 */
  __IO uint32_t CCMR1;       							/*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __IO uint32_t CCMR2;       							/*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __IO uint32_t CCER;        							/*!< TIM capture/compare enable register, Address offset: 0x20 */
  __IO uint32_t CNT;         							/*!< TIM counter register,                Address offset: 0x24 */
  __IO uint32_t PSC;         							/*!< TIM prescaler,                       Address offset: 0x28 */
  __IO uint32_t ARR;         							/*!< TIM auto-reload register,            Address offset: 0x2C */
  __IO uint32_t RCR;         							/*!< TIM repetition counter register,     Address offset: 0x30 */
  __IO uint32_t CCR1;        							/*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __IO uint32_t CCR2;        							/*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __IO uint32_t CCR3;        							/*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __IO uint32_t CCR4;        							/*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __IO uint32_t BDTR;        							/*!< TIM break and dead-time register,    Address offset: 0x44 */
  __IO uint32_t DCR;         							/*!< TIM DMA control register,            Address offset: 0x48 */
  __IO uint32_t DMAR;        							/*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  __IO uint32_t OR;          							/*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;

typedef struct
{
  __IO uint32_t SR;         							/*!< USART Status register,                   Address offset: 0x00 */
  __IO uint32_t DR;         							/*!< USART Data register,                     Address offset: 0x04 */
  __IO uint32_t BRR;        							/*!< USART Baud rate register,                Address offset: 0x08 */
  __IO uint32_t CR1;        							/*!< USART Control register 1,                Address offset: 0x0C */
  __IO uint32_t CR2;        							/*!< USART Control register 2,                Address offset: 0x10 */
  __IO uint32_t CR3;        							/*!< USART Control register 3,                Address offset: 0x14 */
  __IO uint32_t GTPR;       							/*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;

typedef struct
{
  __IO uint32_t CR;   									/*!< WWDG Control register,       Address offset: 0x00 */
  __IO uint32_t CFR;  									/*!< WWDG Configuration register, Address offset: 0x04 */
  __IO uint32_t SR;   									/*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;

typedef struct 
{
  __IO uint32_t CR;  									/*!< RNG control register, Address offset: 0x00 */
  __IO uint32_t SR;  									/*!< RNG status register,  Address offset: 0x04 */
  __IO uint32_t DR;  									/*!< RNG data register,    Address offset: 0x08 */
} RNG_TypeDef;

typedef struct
{
  __IO uint32_t GOTGCTL;              					/*!< USB_OTG Control and Status Register          000h */
  __IO uint32_t GOTGINT;              					/*!< USB_OTG Interrupt Register                   004h */
  __IO uint32_t GAHBCFG;              					/*!< Core AHB Configuration Register              008h */
  __IO uint32_t GUSBCFG;              					/*!< Core USB Configuration Register              00Ch */
  __IO uint32_t GRSTCTL;              					/*!< Core Reset Register                          010h */
  __IO uint32_t GINTSTS;              					/*!< Core Interrupt Register                      014h */
  __IO uint32_t GINTMSK;              					/*!< Core Interrupt Mask Register                 018h */
  __IO uint32_t GRXSTSR;              					/*!< Receive Sts Q Read Register                  01Ch */
  __IO uint32_t GRXSTSP;              					/*!< Receive Sts Q Read & POP Register            020h */
  __IO uint32_t GRXFSIZ;              					/*!< Receive FIFO Size Register                   024h */
  __IO uint32_t DIEPTXF0_HNPTXFSIZ;   					/*!< EP0 / Non Periodic Tx FIFO Size Register     028h */
  __IO uint32_t HNPTXSTS;             					/*!< Non Periodic Tx FIFO/Queue Sts reg           02Ch */
  uint32_t Reserved30[2];             					/*!< Reserved                                     030h */
  __IO uint32_t GCCFG;                					/*!< General Purpose IO Register                  038h */
  __IO uint32_t CID;                  					/*!< User ID Register                             03Ch */
  uint32_t  Reserved40[48];           					/*!< Reserved                                0x40-0xFF */
  __IO uint32_t HPTXFSIZ;             					/*!< Host Periodic Tx FIFO Size Reg               100h */
  __IO uint32_t DIEPTXF[0x0F];        					/*!< dev Periodic Transmit FIFO                        */
} USB_OTG_GlobalTypeDef;

typedef struct 
{
  __IO uint32_t DCFG;            						/*!< dev Configuration Register   800h */
  __IO uint32_t DCTL;            						/*!< dev Control Register         804h */
  __IO uint32_t DSTS;            						/*!< dev Status Register (RO)     808h */
  uint32_t Reserved0C;           						/*!< Reserved                     80Ch */
  __IO uint32_t DIEPMSK;         						/*!< dev IN Endpoint Mask         810h */
  __IO uint32_t DOEPMSK;         						/*!< dev OUT Endpoint Mask        814h */
  __IO uint32_t DAINT;           						/*!< dev All Endpoints Itr Reg    818h */
  __IO uint32_t DAINTMSK;        						/*!< dev All Endpoints Itr Mask   81Ch */
  uint32_t  Reserved20;          						/*!< Reserved                     820h */
  uint32_t Reserved9;            						/*!< Reserved                     824h */
  __IO uint32_t DVBUSDIS;        						/*!< dev VBUS discharge Register  828h */
  __IO uint32_t DVBUSPULSE;      						/*!< dev VBUS Pulse Register      82Ch */
  __IO uint32_t DTHRCTL;         						/*!< dev threshold                830h */
  __IO uint32_t DIEPEMPMSK;      						/*!< dev empty msk                834h */
  __IO uint32_t DEACHINT;        						/*!< dedicated EP interrupt       838h */
  __IO uint32_t DEACHMSK;        						/*!< dedicated EP msk             83Ch */
  uint32_t Reserved40;           						/*!< dedicated EP mask            840h */
  __IO uint32_t DINEP1MSK;       						/*!< dedicated EP mask            844h */
  uint32_t  Reserved44[15];      						/*!< Reserved                 844-87Ch */
  __IO uint32_t DOUTEP1MSK;      						/*!< dedicated EP msk             884h */
} USB_OTG_DeviceTypeDef;

typedef struct 
{
  __IO uint32_t DIEPCTL;           						/*!< dev IN Endpoint Control Reg    900h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;             						/*!< Reserved                       900h + (ep_num * 20h) + 04h */
  __IO uint32_t DIEPINT;           						/*!< dev IN Endpoint Itr Reg        900h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;             						/*!< Reserved                       900h + (ep_num * 20h) + 0Ch */
  __IO uint32_t DIEPTSIZ;          						/*!< IN Endpoint Txfer Size         900h + (ep_num * 20h) + 10h */
  __IO uint32_t DIEPDMA;           						/*!< IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h */
  __IO uint32_t DTXFSTS;           						/*!< IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h */
  uint32_t Reserved18;             						/*!< Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch */
} USB_OTG_INEndpointTypeDef;

typedef struct 
{
  __IO uint32_t DOEPCTL;       							/*!< dev OUT Endpoint Control Reg           B00h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;         							/*!< Reserved                               B00h + (ep_num * 20h) + 04h */
  __IO uint32_t DOEPINT;       							/*!< dev OUT Endpoint Itr Reg               B00h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;         							/*!< Reserved                               B00h + (ep_num * 20h) + 0Ch */
  __IO uint32_t DOEPTSIZ;      							/*!< dev OUT Endpoint Txfer Size            B00h + (ep_num * 20h) + 10h */
  __IO uint32_t DOEPDMA;       							/*!< dev OUT Endpoint DMA Address           B00h + (ep_num * 20h) + 14h */
  uint32_t Reserved18[2];      							/*!< Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch */
} USB_OTG_OUTEndpointTypeDef;

typedef struct 
{
  __IO uint32_t HCFG;             						/*!< Host Configuration Register          400h */
  __IO uint32_t HFIR;             						/*!< Host Frame Interval Register         404h */
  __IO uint32_t HFNUM;            						/*!< Host Frame Nbr/Frame Remaining       408h */
  uint32_t Reserved40C;           						/*!< Reserved                             40Ch */
  __IO uint32_t HPTXSTS;          						/*!< Host Periodic Tx FIFO/ Queue Status  410h */
  __IO uint32_t HAINT;            						/*!< Host All Channels Interrupt Register 414h */
  __IO uint32_t HAINTMSK;         						/*!< Host All Channels Interrupt Mask     418h */
} USB_OTG_HostTypeDef;						
						
typedef struct						
{						
  __IO uint32_t HCCHAR;           						/*!< Host Channel Characteristics Register    500h */
  __IO uint32_t HCSPLT;           						/*!< Host Channel Split Control Register      504h */
  __IO uint32_t HCINT;            						/*!< Host Channel Interrupt Register          508h */
  __IO uint32_t HCINTMSK;         						/*!< Host Channel Interrupt Mask Register     50Ch */
  __IO uint32_t HCTSIZ;           						/*!< Host Channel Transfer Size Register      510h */
  __IO uint32_t HCDMA;            						/*!< Host Channel DMA Address Register        514h */
  uint32_t Reserved[2];           						/*!< Reserved                                      */
} USB_OTG_HostChannelTypeDef;

#define FLASH_BASE            			0x08000000UL 	/*!< FLASH(up to 1 MB) base address in the alias region                         */
#define CCMDATARAM_BASE       			0x10000000UL 	/*!< CCM(core coupled memory) data RAM(64 KB) base address in the alias region  */
#define SRAM1_BASE            			0x20000000UL 	/*!< SRAM1(112 KB) base address in the alias region                              */
#define SRAM2_BASE            			0x2001C000UL 	/*!< SRAM2(16 KB) base address in the alias region                              */
#define PERIPH_BASE           			0x40000000UL 	/*!< Peripheral base address in the alias region                                */
#define BKPSRAM_BASE          			0x40024000UL 	/*!< Backup SRAM(4 KB) base address in the alias region                         */
#define FSMC_R_BASE           			0xA0000000UL 	/*!< FSMC registers base address                                                */
#define SRAM1_BB_BASE         			0x22000000UL 	/*!< SRAM1(112 KB) base address in the bit-band region                          */
#define SRAM2_BB_BASE         			0x22380000UL 	/*!< SRAM2(16 KB) base address in the bit-band region                           */
#define PERIPH_BB_BASE        			0x42000000UL 	/*!< Peripheral base address in the bit-band region                             */
#define BKPSRAM_BB_BASE       			0x42480000UL 	/*!< Backup SRAM(4 KB) base address in the bit-band region                      */
#define FLASH_END             			0x080FFFFFUL 	/*!< FLASH end address                                                          */
#define FLASH_OTP_BASE        			0x1FFF7800UL 	/*!< Base address of : (up to 528 Bytes) embedded FLASH OTP Area                */
#define FLASH_OTP_END         			0x1FFF7A0FUL 	/*!< End address of : (up to 528 Bytes) embedded FLASH OTP Area                 */
#define CCMDATARAM_END        			0x1000FFFFUL 	/*!< CCM data RAM end address                                                   */
			
/* Legacy defines */			
#define SRAM_BASE             			SRAM1_BASE
#define SRAM_BB_BASE          			SRAM1_BB_BASE
			
/*!< Peripheral memory map */			
#define APB1PERIPH_BASE       			PERIPH_BASE
#define APB2PERIPH_BASE       			(PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       			(PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       			(PERIPH_BASE + 0x10000000UL)
			
/*!< APB1 peripherals */			
#define TIM2_BASE             			(APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             			(APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             			(APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             			(APB1PERIPH_BASE + 0x0C00UL)
#define TIM6_BASE             			(APB1PERIPH_BASE + 0x1000UL)
#define TIM7_BASE             			(APB1PERIPH_BASE + 0x1400UL)
#define TIM12_BASE            			(APB1PERIPH_BASE + 0x1800UL)
#define TIM13_BASE            			(APB1PERIPH_BASE + 0x1C00UL)
#define TIM14_BASE            			(APB1PERIPH_BASE + 0x2000UL)
#define RTC_BASE              			(APB1PERIPH_BASE + 0x2800UL)
#define WWDG_BASE             			(APB1PERIPH_BASE + 0x2C00UL)
#define IWDG_BASE             			(APB1PERIPH_BASE + 0x3000UL)
#define I2S2ext_BASE          			(APB1PERIPH_BASE + 0x3400UL)
#define SPI2_BASE             			(APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             			(APB1PERIPH_BASE + 0x3C00UL)
#define I2S3ext_BASE          			(APB1PERIPH_BASE + 0x4000UL)
#define USART2_BASE           			(APB1PERIPH_BASE + 0x4400UL)
#define USART3_BASE           			(APB1PERIPH_BASE + 0x4800UL)
#define UART4_BASE            			(APB1PERIPH_BASE + 0x4C00UL)
#define UART5_BASE            			(APB1PERIPH_BASE + 0x5000UL)
#define I2C1_BASE             			(APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             			(APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             			(APB1PERIPH_BASE + 0x5C00UL)
#define CAN1_BASE             			(APB1PERIPH_BASE + 0x6400UL)
#define CAN2_BASE             			(APB1PERIPH_BASE + 0x6800UL)
#define PWR_BASE              			(APB1PERIPH_BASE + 0x7000UL)
#define DAC_BASE              			(APB1PERIPH_BASE + 0x7400UL)
			
/*!< APB2 peripherals */			
#define TIM1_BASE             			(APB2PERIPH_BASE + 0x0000UL)
#define TIM8_BASE             			(APB2PERIPH_BASE + 0x0400UL)
#define USART1_BASE           			(APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           			(APB2PERIPH_BASE + 0x1400UL)
#define ADC1_BASE             			(APB2PERIPH_BASE + 0x2000UL)
#define ADC2_BASE             			(APB2PERIPH_BASE + 0x2100UL)
#define ADC3_BASE             			(APB2PERIPH_BASE + 0x2200UL)
#define ADC123_COMMON_BASE    			(APB2PERIPH_BASE + 0x2300UL)
/* Legacy define */			
#define ADC_BASE              			 ADC123_COMMON_BASE
#define SDIO_BASE             			(APB2PERIPH_BASE + 0x2C00UL)
#define SPI1_BASE             			(APB2PERIPH_BASE + 0x3000UL)
#define SYSCFG_BASE           			(APB2PERIPH_BASE + 0x3800UL)
#define EXTI_BASE             			(APB2PERIPH_BASE + 0x3C00UL)
#define TIM9_BASE             			(APB2PERIPH_BASE + 0x4000UL)
#define TIM10_BASE            			(APB2PERIPH_BASE + 0x4400UL)
#define TIM11_BASE            			(APB2PERIPH_BASE + 0x4800UL)
			
/*!< AHB1 peripherals */			
#define GPIOA_BASE            			(AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            			(AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            			(AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            			(AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            			(AHB1PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE            			(AHB1PERIPH_BASE + 0x1400UL)
#define GPIOG_BASE            			(AHB1PERIPH_BASE + 0x1800UL)
#define GPIOH_BASE            			(AHB1PERIPH_BASE + 0x1C00UL)
#define GPIOI_BASE            			(AHB1PERIPH_BASE + 0x2000UL)
#define CRC_BASE              			(AHB1PERIPH_BASE + 0x3000UL)
#define RCC_BASE              			(AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_R_BASE          			(AHB1PERIPH_BASE + 0x3C00UL)
#define DMA1_BASE             			(AHB1PERIPH_BASE + 0x6000UL)
#define DMA1_Stream0_BASE     			(DMA1_BASE + 0x010UL)
#define DMA1_Stream1_BASE     			(DMA1_BASE + 0x028UL)
#define DMA1_Stream2_BASE     			(DMA1_BASE + 0x040UL)
#define DMA1_Stream3_BASE     			(DMA1_BASE + 0x058UL)
#define DMA1_Stream4_BASE     			(DMA1_BASE + 0x070UL)
#define DMA1_Stream5_BASE     			(DMA1_BASE + 0x088UL)
#define DMA1_Stream6_BASE     			(DMA1_BASE + 0x0A0UL)
#define DMA1_Stream7_BASE     			(DMA1_BASE + 0x0B8UL)
#define DMA2_BASE             			(AHB1PERIPH_BASE + 0x6400UL)
#define DMA2_Stream0_BASE     			(DMA2_BASE + 0x010UL)
#define DMA2_Stream1_BASE     			(DMA2_BASE + 0x028UL)
#define DMA2_Stream2_BASE     			(DMA2_BASE + 0x040UL)
#define DMA2_Stream3_BASE     			(DMA2_BASE + 0x058UL)
#define DMA2_Stream4_BASE     			(DMA2_BASE + 0x070UL)
#define DMA2_Stream5_BASE     			(DMA2_BASE + 0x088UL)
#define DMA2_Stream6_BASE     			(DMA2_BASE + 0x0A0UL)
#define DMA2_Stream7_BASE     			(DMA2_BASE + 0x0B8UL)
#define ETH_BASE              			(AHB1PERIPH_BASE + 0x8000UL)
#define ETH_MAC_BASE          			(ETH_BASE)
#define ETH_MMC_BASE          			(ETH_BASE + 0x0100UL)
#define ETH_PTP_BASE          			(ETH_BASE + 0x0700UL)
#define ETH_DMA_BASE          			(ETH_BASE + 0x1000UL)
			
/*!< AHB2 peripherals */			
#define DCMI_BASE             			(AHB2PERIPH_BASE + 0x50000UL)
#define RNG_BASE              			(AHB2PERIPH_BASE + 0x60800UL)

/*!< FSMC Bankx registers base address */
#define FSMC_Bank1_R_BASE     			(FSMC_R_BASE + 0x0000UL)
#define FSMC_Bank1E_R_BASE    			(FSMC_R_BASE + 0x0104UL)
#define FSMC_Bank2_3_R_BASE   			(FSMC_R_BASE + 0x0060UL)
#define FSMC_Bank4_R_BASE     			(FSMC_R_BASE + 0x00A0UL)


/*!< Debug MCU registers base address */
#define DBGMCU_BASE           			0xE0042000UL
/*!< USB registers base address */
#define USB_OTG_HS_PERIPH_BASE          0x40040000UL
#define USB_OTG_FS_PERIPH_BASE          0x50000000UL
										
#define USB_OTG_GLOBAL_BASE             0x000UL
#define USB_OTG_DEVICE_BASE             0x800UL
#define USB_OTG_IN_ENDPOINT_BASE        0x900UL
#define USB_OTG_OUT_ENDPOINT_BASE       0xB00UL
#define USB_OTG_EP_REG_SIZE             0x20UL
#define USB_OTG_HOST_BASE               0x400UL
#define USB_OTG_HOST_PORT_BASE          0x440UL
#define USB_OTG_HOST_CHANNEL_BASE       0x500UL
#define USB_OTG_HOST_CHANNEL_SIZE       0x20UL
#define USB_OTG_PCGCCTL_BASE            0xE00UL
#define USB_OTG_FIFO_BASE               0x1000UL
#define USB_OTG_FIFO_SIZE               0x1000UL

#define UID_BASE                     	0x1FFF7A10UL           /*!< Unique device ID register base address */
#define FLASHSIZE_BASE               	0x1FFF7A22UL           /*!< FLASH Size register base address       */
#define PACKAGE_BASE                 	0x1FFF7BF0UL           /*!< Package size register base address     */
 
#define TIM2                			((TIM_TypeDef *) TIM2_BASE)
#define TIM3                			((TIM_TypeDef *) TIM3_BASE)
#define TIM4                			((TIM_TypeDef *) TIM4_BASE)
#define TIM5                			((TIM_TypeDef *) TIM5_BASE)
#define TIM6                			((TIM_TypeDef *) TIM6_BASE)
#define TIM7                			((TIM_TypeDef *) TIM7_BASE)
#define TIM12               			((TIM_TypeDef *) TIM12_BASE)
#define TIM13               			((TIM_TypeDef *) TIM13_BASE)
#define TIM14               			((TIM_TypeDef *) TIM14_BASE)
#define RTC                 			((RTC_TypeDef *) RTC_BASE)
#define WWDG                			((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                			((IWDG_TypeDef *) IWDG_BASE)
#define I2S2ext             			((SPI_TypeDef *) I2S2ext_BASE)
#define SPI2                			((SPI_TypeDef *) SPI2_BASE)
#define SPI3                			((SPI_TypeDef *) SPI3_BASE)
#define I2S3ext             			((SPI_TypeDef *) I2S3ext_BASE)
#define USART2              			((USART_TypeDef *) USART2_BASE)
#define USART3              			((USART_TypeDef *) USART3_BASE)
#define UART4               			((USART_TypeDef *) UART4_BASE)
#define UART5               			((USART_TypeDef *) UART5_BASE)
#define I2C1                			((I2C_TypeDef *) I2C1_BASE)
#define I2C2                			((I2C_TypeDef *) I2C2_BASE)
#define I2C3                			((I2C_TypeDef *) I2C3_BASE)
#define CAN1                			((CAN_TypeDef *) CAN1_BASE)
#define CAN2                			((CAN_TypeDef *) CAN2_BASE)
#define PWR                 			((PWR_TypeDef *) PWR_BASE)
#define DAC1                			((DAC_TypeDef *) DAC_BASE)
#define DAC                 			((DAC_TypeDef *) DAC_BASE) /* Kept for legacy purpose */
#define TIM1                			((TIM_TypeDef *) TIM1_BASE)
#define TIM8                			((TIM_TypeDef *) TIM8_BASE)
#define USART1              			((USART_TypeDef *) USART1_BASE)
#define USART6              			((USART_TypeDef *) USART6_BASE)
#define ADC1                			((ADC_TypeDef *) ADC1_BASE)
#define ADC2                			((ADC_TypeDef *) ADC2_BASE)
#define ADC3                			((ADC_TypeDef *) ADC3_BASE)
#define ADC123_COMMON       			((ADC_Common_TypeDef *) ADC123_COMMON_BASE)
/* Legacy define */			
#define ADC                 			 ADC123_COMMON
#define SDIO                			((SDIO_TypeDef *) SDIO_BASE)
#define SPI1                			((SPI_TypeDef *) SPI1_BASE)
#define SYSCFG              			((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                			((EXTI_TypeDef *) EXTI_BASE)
#define TIM9                			((TIM_TypeDef *) TIM9_BASE)
#define TIM10               			((TIM_TypeDef *) TIM10_BASE)
#define TIM11               			((TIM_TypeDef *) TIM11_BASE)
#define GPIOA               			((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               			((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               			((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               			((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               			((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               			((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               			((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               			((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI               			((GPIO_TypeDef *) GPIOI_BASE)
#define CRC                 			((CRC_TypeDef *) CRC_BASE)
#define RCC                 			((RCC_TypeDef *) RCC_BASE)
#define FLASH               			((FLASH_TypeDef *) FLASH_R_BASE)
#define DMA1                			((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Stream0        			((DMA_Stream_TypeDef *) DMA1_Stream0_BASE)
#define DMA1_Stream1        			((DMA_Stream_TypeDef *) DMA1_Stream1_BASE)
#define DMA1_Stream2        			((DMA_Stream_TypeDef *) DMA1_Stream2_BASE)
#define DMA1_Stream3        			((DMA_Stream_TypeDef *) DMA1_Stream3_BASE)
#define DMA1_Stream4        			((DMA_Stream_TypeDef *) DMA1_Stream4_BASE)
#define DMA1_Stream5        			((DMA_Stream_TypeDef *) DMA1_Stream5_BASE)
#define DMA1_Stream6        			((DMA_Stream_TypeDef *) DMA1_Stream6_BASE)
#define DMA1_Stream7        			((DMA_Stream_TypeDef *) DMA1_Stream7_BASE)
#define DMA2                			((DMA_TypeDef *) DMA2_BASE)
#define DMA2_Stream0        			((DMA_Stream_TypeDef *) DMA2_Stream0_BASE)
#define DMA2_Stream1        			((DMA_Stream_TypeDef *) DMA2_Stream1_BASE)
#define DMA2_Stream2        			((DMA_Stream_TypeDef *) DMA2_Stream2_BASE)
#define DMA2_Stream3        			((DMA_Stream_TypeDef *) DMA2_Stream3_BASE)
#define DMA2_Stream4        			((DMA_Stream_TypeDef *) DMA2_Stream4_BASE)
#define DMA2_Stream5        			((DMA_Stream_TypeDef *) DMA2_Stream5_BASE)
#define DMA2_Stream6        			((DMA_Stream_TypeDef *) DMA2_Stream6_BASE)
#define DMA2_Stream7        			((DMA_Stream_TypeDef *) DMA2_Stream7_BASE)
#define ETH                 			((ETH_TypeDef *) ETH_BASE)  
#define DCMI                			((DCMI_TypeDef *) DCMI_BASE)
#define RNG                 			((RNG_TypeDef *) RNG_BASE)
#define FSMC_Bank1          			((FSMC_Bank1_TypeDef *) FSMC_Bank1_R_BASE)
#define FSMC_Bank1E         			((FSMC_Bank1E_TypeDef *) FSMC_Bank1E_R_BASE)
#define FSMC_Bank2_3        			((FSMC_Bank2_3_TypeDef *) FSMC_Bank2_3_R_BASE)
#define FSMC_Bank4          			((FSMC_Bank4_TypeDef *) FSMC_Bank4_R_BASE)
#define DBGMCU              			((DBGMCU_TypeDef *) DBGMCU_BASE)
#define USB_OTG_FS          			((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)
#define USB_OTG_HS          			((USB_OTG_GlobalTypeDef *) USB_OTG_HS_PERIPH_BASE)


/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DMA_SxCR register  *****************/
#define DMA_SxCR_CHSEL_Pos       (25U)                                         
#define DMA_SxCR_CHSEL_Msk       (0x7UL << DMA_SxCR_CHSEL_Pos)                  /*!< 0x0E000000 */
#define DMA_SxCR_CHSEL           DMA_SxCR_CHSEL_Msk                            
#define DMA_SxCR_CHSEL_0         0x02000000U                                   
#define DMA_SxCR_CHSEL_1         0x04000000U                                   
#define DMA_SxCR_CHSEL_2         0x08000000U                                   
#define DMA_SxCR_MBURST_Pos      (23U)                                         
#define DMA_SxCR_MBURST_Msk      (0x3UL << DMA_SxCR_MBURST_Pos)                 /*!< 0x01800000 */
#define DMA_SxCR_MBURST          DMA_SxCR_MBURST_Msk                           
#define DMA_SxCR_MBURST_0        (0x1UL << DMA_SxCR_MBURST_Pos)                 /*!< 0x00800000 */
#define DMA_SxCR_MBURST_1        (0x2UL << DMA_SxCR_MBURST_Pos)                 /*!< 0x01000000 */
#define DMA_SxCR_PBURST_Pos      (21U)                                         
#define DMA_SxCR_PBURST_Msk      (0x3UL << DMA_SxCR_PBURST_Pos)                 /*!< 0x00600000 */
#define DMA_SxCR_PBURST          DMA_SxCR_PBURST_Msk                           
#define DMA_SxCR_PBURST_0        (0x1UL << DMA_SxCR_PBURST_Pos)                 /*!< 0x00200000 */
#define DMA_SxCR_PBURST_1        (0x2UL << DMA_SxCR_PBURST_Pos)                 /*!< 0x00400000 */
#define DMA_SxCR_CT_Pos          (19U)                                         
#define DMA_SxCR_CT_Msk          (0x1UL << DMA_SxCR_CT_Pos)                     /*!< 0x00080000 */
#define DMA_SxCR_CT              DMA_SxCR_CT_Msk                               
#define DMA_SxCR_DBM_Pos         (18U)                                         
#define DMA_SxCR_DBM_Msk         (0x1UL << DMA_SxCR_DBM_Pos)                    /*!< 0x00040000 */
#define DMA_SxCR_DBM             DMA_SxCR_DBM_Msk                              
#define DMA_SxCR_PL_Pos          (16U)                                         
#define DMA_SxCR_PL_Msk          (0x3UL << DMA_SxCR_PL_Pos)                     /*!< 0x00030000 */
#define DMA_SxCR_PL              DMA_SxCR_PL_Msk                               
#define DMA_SxCR_PL_0            (0x1UL << DMA_SxCR_PL_Pos)                     /*!< 0x00010000 */
#define DMA_SxCR_PL_1            (0x2UL << DMA_SxCR_PL_Pos)                     /*!< 0x00020000 */
#define DMA_SxCR_PINCOS_Pos      (15U)                                         
#define DMA_SxCR_PINCOS_Msk      (0x1UL << DMA_SxCR_PINCOS_Pos)                 /*!< 0x00008000 */
#define DMA_SxCR_PINCOS          DMA_SxCR_PINCOS_Msk                           
#define DMA_SxCR_MSIZE_Pos       (13U)                                         
#define DMA_SxCR_MSIZE_Msk       (0x3UL << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00006000 */
#define DMA_SxCR_MSIZE           DMA_SxCR_MSIZE_Msk                            
#define DMA_SxCR_MSIZE_0         (0x1UL << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00002000 */
#define DMA_SxCR_MSIZE_1         (0x2UL << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00004000 */
#define DMA_SxCR_PSIZE_Pos       (11U)                                         
#define DMA_SxCR_PSIZE_Msk       (0x3UL << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00001800 */
#define DMA_SxCR_PSIZE           DMA_SxCR_PSIZE_Msk                            
#define DMA_SxCR_PSIZE_0         (0x1UL << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00000800 */
#define DMA_SxCR_PSIZE_1         (0x2UL << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00001000 */
#define DMA_SxCR_MINC_Pos        (10U)                                         
#define DMA_SxCR_MINC_Msk        (0x1UL << DMA_SxCR_MINC_Pos)                   /*!< 0x00000400 */
#define DMA_SxCR_MINC            DMA_SxCR_MINC_Msk                             
#define DMA_SxCR_PINC_Pos        (9U)                                          
#define DMA_SxCR_PINC_Msk        (0x1UL << DMA_SxCR_PINC_Pos)                   /*!< 0x00000200 */
#define DMA_SxCR_PINC            DMA_SxCR_PINC_Msk                             
#define DMA_SxCR_CIRC_Pos        (8U)                                          
#define DMA_SxCR_CIRC_Msk        (0x1UL << DMA_SxCR_CIRC_Pos)                   /*!< 0x00000100 */
#define DMA_SxCR_CIRC            DMA_SxCR_CIRC_Msk                             
#define DMA_SxCR_DIR_Pos         (6U)                                          
#define DMA_SxCR_DIR_Msk         (0x3UL << DMA_SxCR_DIR_Pos)                    /*!< 0x000000C0 */
#define DMA_SxCR_DIR             DMA_SxCR_DIR_Msk                              
#define DMA_SxCR_DIR_0           (0x1UL << DMA_SxCR_DIR_Pos)                    /*!< 0x00000040 */
#define DMA_SxCR_DIR_1           (0x2UL << DMA_SxCR_DIR_Pos)                    /*!< 0x00000080 */
#define DMA_SxCR_PFCTRL_Pos      (5U)                                          
#define DMA_SxCR_PFCTRL_Msk      (0x1UL << DMA_SxCR_PFCTRL_Pos)                 /*!< 0x00000020 */
#define DMA_SxCR_PFCTRL          DMA_SxCR_PFCTRL_Msk                           
#define DMA_SxCR_TCIE_Pos        (4U)                                          
#define DMA_SxCR_TCIE_Msk        (0x1UL << DMA_SxCR_TCIE_Pos)                   /*!< 0x00000010 */
#define DMA_SxCR_TCIE            DMA_SxCR_TCIE_Msk                             
#define DMA_SxCR_HTIE_Pos        (3U)                                          
#define DMA_SxCR_HTIE_Msk        (0x1UL << DMA_SxCR_HTIE_Pos)                   /*!< 0x00000008 */
#define DMA_SxCR_HTIE            DMA_SxCR_HTIE_Msk                             
#define DMA_SxCR_TEIE_Pos        (2U)                                          
#define DMA_SxCR_TEIE_Msk        (0x1UL << DMA_SxCR_TEIE_Pos)                   /*!< 0x00000004 */
#define DMA_SxCR_TEIE            DMA_SxCR_TEIE_Msk                             
#define DMA_SxCR_DMEIE_Pos       (1U)                                          
#define DMA_SxCR_DMEIE_Msk       (0x1UL << DMA_SxCR_DMEIE_Pos)                  /*!< 0x00000002 */
#define DMA_SxCR_DMEIE           DMA_SxCR_DMEIE_Msk                            
#define DMA_SxCR_EN_Pos          (0U)                                          
#define DMA_SxCR_EN_Msk          (0x1UL << DMA_SxCR_EN_Pos)                     /*!< 0x00000001 */
#define DMA_SxCR_EN              DMA_SxCR_EN_Msk                               

/* Legacy defines */
#define DMA_SxCR_ACK_Pos         (20U)                                         
#define DMA_SxCR_ACK_Msk         (0x1UL << DMA_SxCR_ACK_Pos)                    /*!< 0x00100000 */
#define DMA_SxCR_ACK             DMA_SxCR_ACK_Msk                              

/********************  Bits definition for DMA_SxCNDTR register  **************/
#define DMA_SxNDT_Pos            (0U)                                          
#define DMA_SxNDT_Msk            (0xFFFFUL << DMA_SxNDT_Pos)                    /*!< 0x0000FFFF */
#define DMA_SxNDT                DMA_SxNDT_Msk                                 
#define DMA_SxNDT_0              (0x0001UL << DMA_SxNDT_Pos)                    /*!< 0x00000001 */
#define DMA_SxNDT_1              (0x0002UL << DMA_SxNDT_Pos)                    /*!< 0x00000002 */
#define DMA_SxNDT_2              (0x0004UL << DMA_SxNDT_Pos)                    /*!< 0x00000004 */
#define DMA_SxNDT_3              (0x0008UL << DMA_SxNDT_Pos)                    /*!< 0x00000008 */
#define DMA_SxNDT_4              (0x0010UL << DMA_SxNDT_Pos)                    /*!< 0x00000010 */
#define DMA_SxNDT_5              (0x0020UL << DMA_SxNDT_Pos)                    /*!< 0x00000020 */
#define DMA_SxNDT_6              (0x0040UL << DMA_SxNDT_Pos)                    /*!< 0x00000040 */
#define DMA_SxNDT_7              (0x0080UL << DMA_SxNDT_Pos)                    /*!< 0x00000080 */
#define DMA_SxNDT_8              (0x0100UL << DMA_SxNDT_Pos)                    /*!< 0x00000100 */
#define DMA_SxNDT_9              (0x0200UL << DMA_SxNDT_Pos)                    /*!< 0x00000200 */
#define DMA_SxNDT_10             (0x0400UL << DMA_SxNDT_Pos)                    /*!< 0x00000400 */
#define DMA_SxNDT_11             (0x0800UL << DMA_SxNDT_Pos)                    /*!< 0x00000800 */
#define DMA_SxNDT_12             (0x1000UL << DMA_SxNDT_Pos)                    /*!< 0x00001000 */
#define DMA_SxNDT_13             (0x2000UL << DMA_SxNDT_Pos)                    /*!< 0x00002000 */
#define DMA_SxNDT_14             (0x4000UL << DMA_SxNDT_Pos)                    /*!< 0x00004000 */
#define DMA_SxNDT_15             (0x8000UL << DMA_SxNDT_Pos)                    /*!< 0x00008000 */

/********************  Bits definition for DMA_SxFCR register  ****************/ 
#define DMA_SxFCR_FEIE_Pos       (7U)                                          
#define DMA_SxFCR_FEIE_Msk       (0x1UL << DMA_SxFCR_FEIE_Pos)                  /*!< 0x00000080 */
#define DMA_SxFCR_FEIE           DMA_SxFCR_FEIE_Msk                            
#define DMA_SxFCR_FS_Pos         (3U)                                          
#define DMA_SxFCR_FS_Msk         (0x7UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000038 */
#define DMA_SxFCR_FS             DMA_SxFCR_FS_Msk                              
#define DMA_SxFCR_FS_0           (0x1UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000008 */
#define DMA_SxFCR_FS_1           (0x2UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000010 */
#define DMA_SxFCR_FS_2           (0x4UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000020 */
#define DMA_SxFCR_DMDIS_Pos      (2U)                                          
#define DMA_SxFCR_DMDIS_Msk      (0x1UL << DMA_SxFCR_DMDIS_Pos)                 /*!< 0x00000004 */
#define DMA_SxFCR_DMDIS          DMA_SxFCR_DMDIS_Msk                           
#define DMA_SxFCR_FTH_Pos        (0U)                                          
#define DMA_SxFCR_FTH_Msk        (0x3UL << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000003 */
#define DMA_SxFCR_FTH            DMA_SxFCR_FTH_Msk                             
#define DMA_SxFCR_FTH_0          (0x1UL << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000001 */
#define DMA_SxFCR_FTH_1          (0x2UL << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000002 */

/********************  Bits definition for DMA_LISR register  *****************/ 
#define DMA_LISR_TCIF3_Pos       (27U)                                         
#define DMA_LISR_TCIF3_Msk       (0x1UL << DMA_LISR_TCIF3_Pos)                  /*!< 0x08000000 */
#define DMA_LISR_TCIF3           DMA_LISR_TCIF3_Msk                            
#define DMA_LISR_HTIF3_Pos       (26U)                                         
#define DMA_LISR_HTIF3_Msk       (0x1UL << DMA_LISR_HTIF3_Pos)                  /*!< 0x04000000 */
#define DMA_LISR_HTIF3           DMA_LISR_HTIF3_Msk                            
#define DMA_LISR_TEIF3_Pos       (25U)                                         
#define DMA_LISR_TEIF3_Msk       (0x1UL << DMA_LISR_TEIF3_Pos)                  /*!< 0x02000000 */
#define DMA_LISR_TEIF3           DMA_LISR_TEIF3_Msk                            
#define DMA_LISR_DMEIF3_Pos      (24U)                                         
#define DMA_LISR_DMEIF3_Msk      (0x1UL << DMA_LISR_DMEIF3_Pos)                 /*!< 0x01000000 */
#define DMA_LISR_DMEIF3          DMA_LISR_DMEIF3_Msk                           
#define DMA_LISR_FEIF3_Pos       (22U)                                         
#define DMA_LISR_FEIF3_Msk       (0x1UL << DMA_LISR_FEIF3_Pos)                  /*!< 0x00400000 */
#define DMA_LISR_FEIF3           DMA_LISR_FEIF3_Msk                            
#define DMA_LISR_TCIF2_Pos       (21U)                                         
#define DMA_LISR_TCIF2_Msk       (0x1UL << DMA_LISR_TCIF2_Pos)                  /*!< 0x00200000 */
#define DMA_LISR_TCIF2           DMA_LISR_TCIF2_Msk                            
#define DMA_LISR_HTIF2_Pos       (20U)                                         
#define DMA_LISR_HTIF2_Msk       (0x1UL << DMA_LISR_HTIF2_Pos)                  /*!< 0x00100000 */
#define DMA_LISR_HTIF2           DMA_LISR_HTIF2_Msk                            
#define DMA_LISR_TEIF2_Pos       (19U)                                         
#define DMA_LISR_TEIF2_Msk       (0x1UL << DMA_LISR_TEIF2_Pos)                  /*!< 0x00080000 */
#define DMA_LISR_TEIF2           DMA_LISR_TEIF2_Msk                            
#define DMA_LISR_DMEIF2_Pos      (18U)                                         
#define DMA_LISR_DMEIF2_Msk      (0x1UL << DMA_LISR_DMEIF2_Pos)                 /*!< 0x00040000 */
#define DMA_LISR_DMEIF2          DMA_LISR_DMEIF2_Msk                           
#define DMA_LISR_FEIF2_Pos       (16U)                                         
#define DMA_LISR_FEIF2_Msk       (0x1UL << DMA_LISR_FEIF2_Pos)                  /*!< 0x00010000 */
#define DMA_LISR_FEIF2           DMA_LISR_FEIF2_Msk                            
#define DMA_LISR_TCIF1_Pos       (11U)                                         
#define DMA_LISR_TCIF1_Msk       (0x1UL << DMA_LISR_TCIF1_Pos)                  /*!< 0x00000800 */
#define DMA_LISR_TCIF1           DMA_LISR_TCIF1_Msk                            
#define DMA_LISR_HTIF1_Pos       (10U)                                         
#define DMA_LISR_HTIF1_Msk       (0x1UL << DMA_LISR_HTIF1_Pos)                  /*!< 0x00000400 */
#define DMA_LISR_HTIF1           DMA_LISR_HTIF1_Msk                            
#define DMA_LISR_TEIF1_Pos       (9U)                                          
#define DMA_LISR_TEIF1_Msk       (0x1UL << DMA_LISR_TEIF1_Pos)                  /*!< 0x00000200 */
#define DMA_LISR_TEIF1           DMA_LISR_TEIF1_Msk                            
#define DMA_LISR_DMEIF1_Pos      (8U)                                          
#define DMA_LISR_DMEIF1_Msk      (0x1UL << DMA_LISR_DMEIF1_Pos)                 /*!< 0x00000100 */
#define DMA_LISR_DMEIF1          DMA_LISR_DMEIF1_Msk                           
#define DMA_LISR_FEIF1_Pos       (6U)                                          
#define DMA_LISR_FEIF1_Msk       (0x1UL << DMA_LISR_FEIF1_Pos)                  /*!< 0x00000040 */
#define DMA_LISR_FEIF1           DMA_LISR_FEIF1_Msk                            
#define DMA_LISR_TCIF0_Pos       (5U)                                          
#define DMA_LISR_TCIF0_Msk       (0x1UL << DMA_LISR_TCIF0_Pos)                  /*!< 0x00000020 */
#define DMA_LISR_TCIF0           DMA_LISR_TCIF0_Msk                            
#define DMA_LISR_HTIF0_Pos       (4U)                                          
#define DMA_LISR_HTIF0_Msk       (0x1UL << DMA_LISR_HTIF0_Pos)                  /*!< 0x00000010 */
#define DMA_LISR_HTIF0           DMA_LISR_HTIF0_Msk                            
#define DMA_LISR_TEIF0_Pos       (3U)                                          
#define DMA_LISR_TEIF0_Msk       (0x1UL << DMA_LISR_TEIF0_Pos)                  /*!< 0x00000008 */
#define DMA_LISR_TEIF0           DMA_LISR_TEIF0_Msk                            
#define DMA_LISR_DMEIF0_Pos      (2U)                                          
#define DMA_LISR_DMEIF0_Msk      (0x1UL << DMA_LISR_DMEIF0_Pos)                 /*!< 0x00000004 */
#define DMA_LISR_DMEIF0          DMA_LISR_DMEIF0_Msk                           
#define DMA_LISR_FEIF0_Pos       (0U)                                          
#define DMA_LISR_FEIF0_Msk       (0x1UL << DMA_LISR_FEIF0_Pos)                  /*!< 0x00000001 */
#define DMA_LISR_FEIF0           DMA_LISR_FEIF0_Msk                            

/********************  Bits definition for DMA_HISR register  *****************/ 
#define DMA_HISR_TCIF7_Pos       (27U)                                         
#define DMA_HISR_TCIF7_Msk       (0x1UL << DMA_HISR_TCIF7_Pos)                  /*!< 0x08000000 */
#define DMA_HISR_TCIF7           DMA_HISR_TCIF7_Msk                            
#define DMA_HISR_HTIF7_Pos       (26U)                                         
#define DMA_HISR_HTIF7_Msk       (0x1UL << DMA_HISR_HTIF7_Pos)                  /*!< 0x04000000 */
#define DMA_HISR_HTIF7           DMA_HISR_HTIF7_Msk                            
#define DMA_HISR_TEIF7_Pos       (25U)                                         
#define DMA_HISR_TEIF7_Msk       (0x1UL << DMA_HISR_TEIF7_Pos)                  /*!< 0x02000000 */
#define DMA_HISR_TEIF7           DMA_HISR_TEIF7_Msk                            
#define DMA_HISR_DMEIF7_Pos      (24U)                                         
#define DMA_HISR_DMEIF7_Msk      (0x1UL << DMA_HISR_DMEIF7_Pos)                 /*!< 0x01000000 */
#define DMA_HISR_DMEIF7          DMA_HISR_DMEIF7_Msk                           
#define DMA_HISR_FEIF7_Pos       (22U)                                         
#define DMA_HISR_FEIF7_Msk       (0x1UL << DMA_HISR_FEIF7_Pos)                  /*!< 0x00400000 */
#define DMA_HISR_FEIF7           DMA_HISR_FEIF7_Msk                            
#define DMA_HISR_TCIF6_Pos       (21U)                                         
#define DMA_HISR_TCIF6_Msk       (0x1UL << DMA_HISR_TCIF6_Pos)                  /*!< 0x00200000 */
#define DMA_HISR_TCIF6           DMA_HISR_TCIF6_Msk                            
#define DMA_HISR_HTIF6_Pos       (20U)                                         
#define DMA_HISR_HTIF6_Msk       (0x1UL << DMA_HISR_HTIF6_Pos)                  /*!< 0x00100000 */
#define DMA_HISR_HTIF6           DMA_HISR_HTIF6_Msk                            
#define DMA_HISR_TEIF6_Pos       (19U)                                         
#define DMA_HISR_TEIF6_Msk       (0x1UL << DMA_HISR_TEIF6_Pos)                  /*!< 0x00080000 */
#define DMA_HISR_TEIF6           DMA_HISR_TEIF6_Msk                            
#define DMA_HISR_DMEIF6_Pos      (18U)                                         
#define DMA_HISR_DMEIF6_Msk      (0x1UL << DMA_HISR_DMEIF6_Pos)                 /*!< 0x00040000 */
#define DMA_HISR_DMEIF6          DMA_HISR_DMEIF6_Msk                           
#define DMA_HISR_FEIF6_Pos       (16U)                                         
#define DMA_HISR_FEIF6_Msk       (0x1UL << DMA_HISR_FEIF6_Pos)                  /*!< 0x00010000 */
#define DMA_HISR_FEIF6           DMA_HISR_FEIF6_Msk                            
#define DMA_HISR_TCIF5_Pos       (11U)                                         
#define DMA_HISR_TCIF5_Msk       (0x1UL << DMA_HISR_TCIF5_Pos)                  /*!< 0x00000800 */
#define DMA_HISR_TCIF5           DMA_HISR_TCIF5_Msk                            
#define DMA_HISR_HTIF5_Pos       (10U)                                         
#define DMA_HISR_HTIF5_Msk       (0x1UL << DMA_HISR_HTIF5_Pos)                  /*!< 0x00000400 */
#define DMA_HISR_HTIF5           DMA_HISR_HTIF5_Msk                            
#define DMA_HISR_TEIF5_Pos       (9U)                                          
#define DMA_HISR_TEIF5_Msk       (0x1UL << DMA_HISR_TEIF5_Pos)                  /*!< 0x00000200 */
#define DMA_HISR_TEIF5           DMA_HISR_TEIF5_Msk                            
#define DMA_HISR_DMEIF5_Pos      (8U)                                          
#define DMA_HISR_DMEIF5_Msk      (0x1UL << DMA_HISR_DMEIF5_Pos)                 /*!< 0x00000100 */
#define DMA_HISR_DMEIF5          DMA_HISR_DMEIF5_Msk                           
#define DMA_HISR_FEIF5_Pos       (6U)                                          
#define DMA_HISR_FEIF5_Msk       (0x1UL << DMA_HISR_FEIF5_Pos)                  /*!< 0x00000040 */
#define DMA_HISR_FEIF5           DMA_HISR_FEIF5_Msk                            
#define DMA_HISR_TCIF4_Pos       (5U)                                          
#define DMA_HISR_TCIF4_Msk       (0x1UL << DMA_HISR_TCIF4_Pos)                  /*!< 0x00000020 */
#define DMA_HISR_TCIF4           DMA_HISR_TCIF4_Msk                            
#define DMA_HISR_HTIF4_Pos       (4U)                                          
#define DMA_HISR_HTIF4_Msk       (0x1UL << DMA_HISR_HTIF4_Pos)                  /*!< 0x00000010 */
#define DMA_HISR_HTIF4           DMA_HISR_HTIF4_Msk                            
#define DMA_HISR_TEIF4_Pos       (3U)                                          
#define DMA_HISR_TEIF4_Msk       (0x1UL << DMA_HISR_TEIF4_Pos)                  /*!< 0x00000008 */
#define DMA_HISR_TEIF4           DMA_HISR_TEIF4_Msk                            
#define DMA_HISR_DMEIF4_Pos      (2U)                                          
#define DMA_HISR_DMEIF4_Msk      (0x1UL << DMA_HISR_DMEIF4_Pos)                 /*!< 0x00000004 */
#define DMA_HISR_DMEIF4          DMA_HISR_DMEIF4_Msk                           
#define DMA_HISR_FEIF4_Pos       (0U)                                          
#define DMA_HISR_FEIF4_Msk       (0x1UL << DMA_HISR_FEIF4_Pos)                  /*!< 0x00000001 */
#define DMA_HISR_FEIF4           DMA_HISR_FEIF4_Msk                            

/********************  Bits definition for DMA_LIFCR register  ****************/ 
#define DMA_LIFCR_CTCIF3_Pos     (27U)                                         
#define DMA_LIFCR_CTCIF3_Msk     (0x1UL << DMA_LIFCR_CTCIF3_Pos)                /*!< 0x08000000 */
#define DMA_LIFCR_CTCIF3         DMA_LIFCR_CTCIF3_Msk                          
#define DMA_LIFCR_CHTIF3_Pos     (26U)                                         
#define DMA_LIFCR_CHTIF3_Msk     (0x1UL << DMA_LIFCR_CHTIF3_Pos)                /*!< 0x04000000 */
#define DMA_LIFCR_CHTIF3         DMA_LIFCR_CHTIF3_Msk                          
#define DMA_LIFCR_CTEIF3_Pos     (25U)                                         
#define DMA_LIFCR_CTEIF3_Msk     (0x1UL << DMA_LIFCR_CTEIF3_Pos)                /*!< 0x02000000 */
#define DMA_LIFCR_CTEIF3         DMA_LIFCR_CTEIF3_Msk                          
#define DMA_LIFCR_CDMEIF3_Pos    (24U)                                         
#define DMA_LIFCR_CDMEIF3_Msk    (0x1UL << DMA_LIFCR_CDMEIF3_Pos)               /*!< 0x01000000 */
#define DMA_LIFCR_CDMEIF3        DMA_LIFCR_CDMEIF3_Msk                         
#define DMA_LIFCR_CFEIF3_Pos     (22U)                                         
#define DMA_LIFCR_CFEIF3_Msk     (0x1UL << DMA_LIFCR_CFEIF3_Pos)                /*!< 0x00400000 */
#define DMA_LIFCR_CFEIF3         DMA_LIFCR_CFEIF3_Msk                          
#define DMA_LIFCR_CTCIF2_Pos     (21U)                                         
#define DMA_LIFCR_CTCIF2_Msk     (0x1UL << DMA_LIFCR_CTCIF2_Pos)                /*!< 0x00200000 */
#define DMA_LIFCR_CTCIF2         DMA_LIFCR_CTCIF2_Msk                          
#define DMA_LIFCR_CHTIF2_Pos     (20U)                                         
#define DMA_LIFCR_CHTIF2_Msk     (0x1UL << DMA_LIFCR_CHTIF2_Pos)                /*!< 0x00100000 */
#define DMA_LIFCR_CHTIF2         DMA_LIFCR_CHTIF2_Msk                          
#define DMA_LIFCR_CTEIF2_Pos     (19U)                                         
#define DMA_LIFCR_CTEIF2_Msk     (0x1UL << DMA_LIFCR_CTEIF2_Pos)                /*!< 0x00080000 */
#define DMA_LIFCR_CTEIF2         DMA_LIFCR_CTEIF2_Msk                          
#define DMA_LIFCR_CDMEIF2_Pos    (18U)                                         
#define DMA_LIFCR_CDMEIF2_Msk    (0x1UL << DMA_LIFCR_CDMEIF2_Pos)               /*!< 0x00040000 */
#define DMA_LIFCR_CDMEIF2        DMA_LIFCR_CDMEIF2_Msk                         
#define DMA_LIFCR_CFEIF2_Pos     (16U)                                         
#define DMA_LIFCR_CFEIF2_Msk     (0x1UL << DMA_LIFCR_CFEIF2_Pos)                /*!< 0x00010000 */
#define DMA_LIFCR_CFEIF2         DMA_LIFCR_CFEIF2_Msk                          
#define DMA_LIFCR_CTCIF1_Pos     (11U)                                         
#define DMA_LIFCR_CTCIF1_Msk     (0x1UL << DMA_LIFCR_CTCIF1_Pos)                /*!< 0x00000800 */
#define DMA_LIFCR_CTCIF1         DMA_LIFCR_CTCIF1_Msk                          
#define DMA_LIFCR_CHTIF1_Pos     (10U)                                         
#define DMA_LIFCR_CHTIF1_Msk     (0x1UL << DMA_LIFCR_CHTIF1_Pos)                /*!< 0x00000400 */
#define DMA_LIFCR_CHTIF1         DMA_LIFCR_CHTIF1_Msk                          
#define DMA_LIFCR_CTEIF1_Pos     (9U)                                          
#define DMA_LIFCR_CTEIF1_Msk     (0x1UL << DMA_LIFCR_CTEIF1_Pos)                /*!< 0x00000200 */
#define DMA_LIFCR_CTEIF1         DMA_LIFCR_CTEIF1_Msk                          
#define DMA_LIFCR_CDMEIF1_Pos    (8U)                                          
#define DMA_LIFCR_CDMEIF1_Msk    (0x1UL << DMA_LIFCR_CDMEIF1_Pos)               /*!< 0x00000100 */
#define DMA_LIFCR_CDMEIF1        DMA_LIFCR_CDMEIF1_Msk                         
#define DMA_LIFCR_CFEIF1_Pos     (6U)                                          
#define DMA_LIFCR_CFEIF1_Msk     (0x1UL << DMA_LIFCR_CFEIF1_Pos)                /*!< 0x00000040 */
#define DMA_LIFCR_CFEIF1         DMA_LIFCR_CFEIF1_Msk                          
#define DMA_LIFCR_CTCIF0_Pos     (5U)                                          
#define DMA_LIFCR_CTCIF0_Msk     (0x1UL << DMA_LIFCR_CTCIF0_Pos)                /*!< 0x00000020 */
#define DMA_LIFCR_CTCIF0         DMA_LIFCR_CTCIF0_Msk                          
#define DMA_LIFCR_CHTIF0_Pos     (4U)                                          
#define DMA_LIFCR_CHTIF0_Msk     (0x1UL << DMA_LIFCR_CHTIF0_Pos)                /*!< 0x00000010 */
#define DMA_LIFCR_CHTIF0         DMA_LIFCR_CHTIF0_Msk                          
#define DMA_LIFCR_CTEIF0_Pos     (3U)                                          
#define DMA_LIFCR_CTEIF0_Msk     (0x1UL << DMA_LIFCR_CTEIF0_Pos)                /*!< 0x00000008 */
#define DMA_LIFCR_CTEIF0         DMA_LIFCR_CTEIF0_Msk                          
#define DMA_LIFCR_CDMEIF0_Pos    (2U)                                          
#define DMA_LIFCR_CDMEIF0_Msk    (0x1UL << DMA_LIFCR_CDMEIF0_Pos)               /*!< 0x00000004 */
#define DMA_LIFCR_CDMEIF0        DMA_LIFCR_CDMEIF0_Msk                         
#define DMA_LIFCR_CFEIF0_Pos     (0U)                                          
#define DMA_LIFCR_CFEIF0_Msk     (0x1UL << DMA_LIFCR_CFEIF0_Pos)                /*!< 0x00000001 */
#define DMA_LIFCR_CFEIF0         DMA_LIFCR_CFEIF0_Msk                          

/********************  Bits definition for DMA_HIFCR  register  ****************/ 
#define DMA_HIFCR_CTCIF7_Pos     (27U)                                         
#define DMA_HIFCR_CTCIF7_Msk     (0x1UL << DMA_HIFCR_CTCIF7_Pos)                /*!< 0x08000000 */
#define DMA_HIFCR_CTCIF7         DMA_HIFCR_CTCIF7_Msk                          
#define DMA_HIFCR_CHTIF7_Pos     (26U)                                         
#define DMA_HIFCR_CHTIF7_Msk     (0x1UL << DMA_HIFCR_CHTIF7_Pos)                /*!< 0x04000000 */
#define DMA_HIFCR_CHTIF7         DMA_HIFCR_CHTIF7_Msk                          
#define DMA_HIFCR_CTEIF7_Pos     (25U)                                         
#define DMA_HIFCR_CTEIF7_Msk     (0x1UL << DMA_HIFCR_CTEIF7_Pos)                /*!< 0x02000000 */
#define DMA_HIFCR_CTEIF7         DMA_HIFCR_CTEIF7_Msk                          
#define DMA_HIFCR_CDMEIF7_Pos    (24U)                                         
#define DMA_HIFCR_CDMEIF7_Msk    (0x1UL << DMA_HIFCR_CDMEIF7_Pos)               /*!< 0x01000000 */
#define DMA_HIFCR_CDMEIF7        DMA_HIFCR_CDMEIF7_Msk                         
#define DMA_HIFCR_CFEIF7_Pos     (22U)                                         
#define DMA_HIFCR_CFEIF7_Msk     (0x1UL << DMA_HIFCR_CFEIF7_Pos)                /*!< 0x00400000 */
#define DMA_HIFCR_CFEIF7         DMA_HIFCR_CFEIF7_Msk                          
#define DMA_HIFCR_CTCIF6_Pos     (21U)                                         
#define DMA_HIFCR_CTCIF6_Msk     (0x1UL << DMA_HIFCR_CTCIF6_Pos)                /*!< 0x00200000 */
#define DMA_HIFCR_CTCIF6         DMA_HIFCR_CTCIF6_Msk                          
#define DMA_HIFCR_CHTIF6_Pos     (20U)                                         
#define DMA_HIFCR_CHTIF6_Msk     (0x1UL << DMA_HIFCR_CHTIF6_Pos)                /*!< 0x00100000 */
#define DMA_HIFCR_CHTIF6         DMA_HIFCR_CHTIF6_Msk                          
#define DMA_HIFCR_CTEIF6_Pos     (19U)                                         
#define DMA_HIFCR_CTEIF6_Msk     (0x1UL << DMA_HIFCR_CTEIF6_Pos)                /*!< 0x00080000 */
#define DMA_HIFCR_CTEIF6         DMA_HIFCR_CTEIF6_Msk                          
#define DMA_HIFCR_CDMEIF6_Pos    (18U)                                         
#define DMA_HIFCR_CDMEIF6_Msk    (0x1UL << DMA_HIFCR_CDMEIF6_Pos)               /*!< 0x00040000 */
#define DMA_HIFCR_CDMEIF6        DMA_HIFCR_CDMEIF6_Msk                         
#define DMA_HIFCR_CFEIF6_Pos     (16U)                                         
#define DMA_HIFCR_CFEIF6_Msk     (0x1UL << DMA_HIFCR_CFEIF6_Pos)                /*!< 0x00010000 */
#define DMA_HIFCR_CFEIF6         DMA_HIFCR_CFEIF6_Msk                          
#define DMA_HIFCR_CTCIF5_Pos     (11U)                                         
#define DMA_HIFCR_CTCIF5_Msk     (0x1UL << DMA_HIFCR_CTCIF5_Pos)                /*!< 0x00000800 */
#define DMA_HIFCR_CTCIF5         DMA_HIFCR_CTCIF5_Msk                          
#define DMA_HIFCR_CHTIF5_Pos     (10U)                                         
#define DMA_HIFCR_CHTIF5_Msk     (0x1UL << DMA_HIFCR_CHTIF5_Pos)                /*!< 0x00000400 */
#define DMA_HIFCR_CHTIF5         DMA_HIFCR_CHTIF5_Msk                          
#define DMA_HIFCR_CTEIF5_Pos     (9U)                                          
#define DMA_HIFCR_CTEIF5_Msk     (0x1UL << DMA_HIFCR_CTEIF5_Pos)                /*!< 0x00000200 */
#define DMA_HIFCR_CTEIF5         DMA_HIFCR_CTEIF5_Msk                          
#define DMA_HIFCR_CDMEIF5_Pos    (8U)                                          
#define DMA_HIFCR_CDMEIF5_Msk    (0x1UL << DMA_HIFCR_CDMEIF5_Pos)               /*!< 0x00000100 */
#define DMA_HIFCR_CDMEIF5        DMA_HIFCR_CDMEIF5_Msk                         
#define DMA_HIFCR_CFEIF5_Pos     (6U)                                          
#define DMA_HIFCR_CFEIF5_Msk     (0x1UL << DMA_HIFCR_CFEIF5_Pos)                /*!< 0x00000040 */
#define DMA_HIFCR_CFEIF5         DMA_HIFCR_CFEIF5_Msk                          
#define DMA_HIFCR_CTCIF4_Pos     (5U)                                          
#define DMA_HIFCR_CTCIF4_Msk     (0x1UL << DMA_HIFCR_CTCIF4_Pos)                /*!< 0x00000020 */
#define DMA_HIFCR_CTCIF4         DMA_HIFCR_CTCIF4_Msk                          
#define DMA_HIFCR_CHTIF4_Pos     (4U)                                          
#define DMA_HIFCR_CHTIF4_Msk     (0x1UL << DMA_HIFCR_CHTIF4_Pos)                /*!< 0x00000010 */
#define DMA_HIFCR_CHTIF4         DMA_HIFCR_CHTIF4_Msk                          
#define DMA_HIFCR_CTEIF4_Pos     (3U)                                          
#define DMA_HIFCR_CTEIF4_Msk     (0x1UL << DMA_HIFCR_CTEIF4_Pos)                /*!< 0x00000008 */
#define DMA_HIFCR_CTEIF4         DMA_HIFCR_CTEIF4_Msk                          
#define DMA_HIFCR_CDMEIF4_Pos    (2U)                                          
#define DMA_HIFCR_CDMEIF4_Msk    (0x1UL << DMA_HIFCR_CDMEIF4_Pos)               /*!< 0x00000004 */
#define DMA_HIFCR_CDMEIF4        DMA_HIFCR_CDMEIF4_Msk                         
#define DMA_HIFCR_CFEIF4_Pos     (0U)                                          
#define DMA_HIFCR_CFEIF4_Msk     (0x1UL << DMA_HIFCR_CFEIF4_Pos)                /*!< 0x00000001 */
#define DMA_HIFCR_CFEIF4         DMA_HIFCR_CFEIF4_Msk                          

/******************  Bit definition for DMA_SxPAR register  ********************/
#define DMA_SxPAR_PA_Pos         (0U)                                          
#define DMA_SxPAR_PA_Msk         (0xFFFFFFFFUL << DMA_SxPAR_PA_Pos)             /*!< 0xFFFFFFFF */
#define DMA_SxPAR_PA             DMA_SxPAR_PA_Msk                              /*!< Peripheral Address */

/******************  Bit definition for DMA_SxM0AR register  ********************/
#define DMA_SxM0AR_M0A_Pos       (0U)                                          
#define DMA_SxM0AR_M0A_Msk       (0xFFFFFFFFUL << DMA_SxM0AR_M0A_Pos)           /*!< 0xFFFFFFFF */
#define DMA_SxM0AR_M0A           DMA_SxM0AR_M0A_Msk                            /*!< Memory Address */

/******************  Bit definition for DMA_SxM1AR register  ********************/
#define DMA_SxM1AR_M1A_Pos       (0U)                                          
#define DMA_SxM1AR_M1A_Msk       (0xFFFFFFFFUL << DMA_SxM1AR_M1A_Pos)           /*!< 0xFFFFFFFF */
#define DMA_SxM1AR_M1A           DMA_SxM1AR_M1A_Msk                            /*!< Memory Address */


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32F407xx_H */

