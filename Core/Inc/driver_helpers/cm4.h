#ifndef __CM4_H
#define __CM4_H

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define __CORTEX_M                (4U)                                   /*!< Cortex-M Core */

#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */

typedef union
{
  struct
  {
    uint32_t _reserved0:16;              /*!< bit:  0..15  Reserved */
    uint32_t GE:4;                       /*!< bit: 16..19  Greater than or Equal flags */
    uint32_t _reserved1:7;               /*!< bit: 20..26  Reserved */
    uint32_t Q:1;                        /*!< bit:     27  Saturation condition flag */
    uint32_t V:1;                        /*!< bit:     28  Overflow condition code flag */
    uint32_t C:1;                        /*!< bit:     29  Carry condition code flag */
    uint32_t Z:1;                        /*!< bit:     30  Zero condition code flag */
    uint32_t N:1;                        /*!< bit:     31  Negative condition code flag */
  } b;                                   /*!< Structure used for bit  access */
  uint32_t w;                            /*!< Type      used for word access */
} APSR_Type;

typedef union
{
  struct
  {
    uint32_t ISR:9;                      /*!< bit:  0.. 8  Exception number */
    uint32_t _reserved0:23;              /*!< bit:  9..31  Reserved */
  } b;                                   /*!< Structure used for bit  access */
  uint32_t w;                            /*!< Type      used for word access */
} IPSR_Type;

typedef union
{
  struct
  {
    uint32_t ISR:9;                      /*!< bit:  0.. 8  Exception number */
    uint32_t _reserved0:1;               /*!< bit:      9  Reserved */
    uint32_t ICI_IT_1:6;                 /*!< bit: 10..15  ICI/IT part 1 */
    uint32_t GE:4;                       /*!< bit: 16..19  Greater than or Equal flags */
    uint32_t _reserved1:4;               /*!< bit: 20..23  Reserved */
    uint32_t T:1;                        /*!< bit:     24  Thumb bit */
    uint32_t ICI_IT_2:2;                 /*!< bit: 25..26  ICI/IT part 2 */
    uint32_t Q:1;                        /*!< bit:     27  Saturation condition flag */
    uint32_t V:1;                        /*!< bit:     28  Overflow condition code flag */
    uint32_t C:1;                        /*!< bit:     29  Carry condition code flag */
    uint32_t Z:1;                        /*!< bit:     30  Zero condition code flag */
    uint32_t N:1;                        /*!< bit:     31  Negative condition code flag */
  } b;                                   /*!< Structure used for bit  access */
  uint32_t w;                            /*!< Type      used for word access */
} xPSR_Type;

typedef union
{
  struct
  {
    uint32_t nPRIV:1;                    /*!< bit:      0  Execution privilege in Thread mode */
    uint32_t SPSEL:1;                    /*!< bit:      1  Stack to be used */
    uint32_t FPCA:1;                     /*!< bit:      2  FP extension active flag */
    uint32_t _reserved0:29;              /*!< bit:  3..31  Reserved */
  } b;                                   /*!< Structure used for bit  access */
  uint32_t w;                            /*!< Type      used for word access */
} CONTROL_Type;

typedef struct
{
  __IOM uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
  __IOM uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RSERVED1[24U];
  __IOM uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
  __IOM uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
  __IOM uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
  __IOM uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
  __OM  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;

typedef struct
{
  __IM  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __IOM uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __IOM uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __IOM uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __IOM uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __IOM uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __IOM uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __IOM uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __IOM uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __IOM uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __IOM uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __IOM uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __IOM uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __IOM uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __IM  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __IM  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __IM  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __IM  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __IM  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
        uint32_t RESERVED0[5U];
  __IOM uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;

typedef struct
{
        uint32_t RESERVED0[1U];
  __IM  uint32_t ICTR;                   /*!< Offset: 0x004 (R/ )  Interrupt Controller Type Register */
  __IOM uint32_t ACTLR;                  /*!< Offset: 0x008 (R/W)  Auxiliary Control Register */
} SCnSCB_Type;

typedef struct
{
  __IOM uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  __IOM uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  __IOM uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  __IM  uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;

typedef struct
{
  __OM  union
  {
    __OM  uint8_t    u8;                 /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 8-bit */
    __OM  uint16_t   u16;                /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 16-bit */
    __OM  uint32_t   u32;                /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 32-bit */
  }  PORT [32U];                         /*!< Offset: 0x000 ( /W)  ITM Stimulus Port Registers */
        uint32_t RESERVED0[864U];
  __IOM uint32_t TER;                    /*!< Offset: 0xE00 (R/W)  ITM Trace Enable Register */
        uint32_t RESERVED1[15U];
  __IOM uint32_t TPR;                    /*!< Offset: 0xE40 (R/W)  ITM Trace Privilege Register */
        uint32_t RESERVED2[15U];
  __IOM uint32_t TCR;                    /*!< Offset: 0xE80 (R/W)  ITM Trace Control Register */
        uint32_t RESERVED3[29U];
  __OM  uint32_t IWR;                    /*!< Offset: 0xEF8 ( /W)  ITM Integration Write Register */
  __IM  uint32_t IRR;                    /*!< Offset: 0xEFC (R/ )  ITM Integration Read Register */
  __IOM uint32_t IMCR;                   /*!< Offset: 0xF00 (R/W)  ITM Integration Mode Control Register */
        uint32_t RESERVED4[43U];
  __OM  uint32_t LAR;                    /*!< Offset: 0xFB0 ( /W)  ITM Lock Access Register */
  __IM  uint32_t LSR;                    /*!< Offset: 0xFB4 (R/ )  ITM Lock Status Register */
        uint32_t RESERVED5[6U];
  __IM  uint32_t PID4;                   /*!< Offset: 0xFD0 (R/ )  ITM Peripheral Identification Register #4 */
  __IM  uint32_t PID5;                   /*!< Offset: 0xFD4 (R/ )  ITM Peripheral Identification Register #5 */
  __IM  uint32_t PID6;                   /*!< Offset: 0xFD8 (R/ )  ITM Peripheral Identification Register #6 */
  __IM  uint32_t PID7;                   /*!< Offset: 0xFDC (R/ )  ITM Peripheral Identification Register #7 */
  __IM  uint32_t PID0;                   /*!< Offset: 0xFE0 (R/ )  ITM Peripheral Identification Register #0 */
  __IM  uint32_t PID1;                   /*!< Offset: 0xFE4 (R/ )  ITM Peripheral Identification Register #1 */
  __IM  uint32_t PID2;                   /*!< Offset: 0xFE8 (R/ )  ITM Peripheral Identification Register #2 */
  __IM  uint32_t PID3;                   /*!< Offset: 0xFEC (R/ )  ITM Peripheral Identification Register #3 */
  __IM  uint32_t CID0;                   /*!< Offset: 0xFF0 (R/ )  ITM Component  Identification Register #0 */
  __IM  uint32_t CID1;                   /*!< Offset: 0xFF4 (R/ )  ITM Component  Identification Register #1 */
  __IM  uint32_t CID2;                   /*!< Offset: 0xFF8 (R/ )  ITM Component  Identification Register #2 */
  __IM  uint32_t CID3;                   /*!< Offset: 0xFFC (R/ )  ITM Component  Identification Register #3 */
} ITM_Type;

typedef struct
{
  __IOM uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  Control Register */
  __IOM uint32_t CYCCNT;                 /*!< Offset: 0x004 (R/W)  Cycle Count Register */
  __IOM uint32_t CPICNT;                 /*!< Offset: 0x008 (R/W)  CPI Count Register */
  __IOM uint32_t EXCCNT;                 /*!< Offset: 0x00C (R/W)  Exception Overhead Count Register */
  __IOM uint32_t SLEEPCNT;               /*!< Offset: 0x010 (R/W)  Sleep Count Register */
  __IOM uint32_t LSUCNT;                 /*!< Offset: 0x014 (R/W)  LSU Count Register */
  __IOM uint32_t FOLDCNT;                /*!< Offset: 0x018 (R/W)  Folded-instruction Count Register */
  __IM  uint32_t PCSR;                   /*!< Offset: 0x01C (R/ )  Program Counter Sample Register */
  __IOM uint32_t COMP0;                  /*!< Offset: 0x020 (R/W)  Comparator Register 0 */
  __IOM uint32_t MASK0;                  /*!< Offset: 0x024 (R/W)  Mask Register 0 */
  __IOM uint32_t FUNCTION0;              /*!< Offset: 0x028 (R/W)  Function Register 0 */
        uint32_t RESERVED0[1U];
  __IOM uint32_t COMP1;                  /*!< Offset: 0x030 (R/W)  Comparator Register 1 */
  __IOM uint32_t MASK1;                  /*!< Offset: 0x034 (R/W)  Mask Register 1 */
  __IOM uint32_t FUNCTION1;              /*!< Offset: 0x038 (R/W)  Function Register 1 */
        uint32_t RESERVED1[1U];
  __IOM uint32_t COMP2;                  /*!< Offset: 0x040 (R/W)  Comparator Register 2 */
  __IOM uint32_t MASK2;                  /*!< Offset: 0x044 (R/W)  Mask Register 2 */
  __IOM uint32_t FUNCTION2;              /*!< Offset: 0x048 (R/W)  Function Register 2 */
        uint32_t RESERVED2[1U];
  __IOM uint32_t COMP3;                  /*!< Offset: 0x050 (R/W)  Comparator Register 3 */
  __IOM uint32_t MASK3;                  /*!< Offset: 0x054 (R/W)  Mask Register 3 */
  __IOM uint32_t FUNCTION3;              /*!< Offset: 0x058 (R/W)  Function Register 3 */
} DWT_Type;

typedef struct
{
  __IM  uint32_t SSPSR;                  /*!< Offset: 0x000 (R/ )  Supported Parallel Port Size Register */
  __IOM uint32_t CSPSR;                  /*!< Offset: 0x004 (R/W)  Current Parallel Port Size Register */
        uint32_t RESERVED0[2U];
  __IOM uint32_t ACPR;                   /*!< Offset: 0x010 (R/W)  Asynchronous Clock Prescaler Register */
        uint32_t RESERVED1[55U];
  __IOM uint32_t SPPR;                   /*!< Offset: 0x0F0 (R/W)  Selected Pin Protocol Register */
        uint32_t RESERVED2[131U];
  __IM  uint32_t FFSR;                   /*!< Offset: 0x300 (R/ )  Formatter and Flush Status Register */
  __IOM uint32_t FFCR;                   /*!< Offset: 0x304 (R/W)  Formatter and Flush Control Register */
  __IM  uint32_t FSCR;                   /*!< Offset: 0x308 (R/ )  Formatter Synchronization Counter Register */
        uint32_t RESERVED3[759U];
  __IM  uint32_t TRIGGER;                /*!< Offset: 0xEE8 (R/ )  TRIGGER Register */
  __IM  uint32_t FIFO0;                  /*!< Offset: 0xEEC (R/ )  Integration ETM Data */
  __IM  uint32_t ITATBCTR2;              /*!< Offset: 0xEF0 (R/ )  ITATBCTR2 */
        uint32_t RESERVED4[1U];
  __IM  uint32_t ITATBCTR0;              /*!< Offset: 0xEF8 (R/ )  ITATBCTR0 */
  __IM  uint32_t FIFO1;                  /*!< Offset: 0xEFC (R/ )  Integration ITM Data */
  __IOM uint32_t ITCTRL;                 /*!< Offset: 0xF00 (R/W)  Integration Mode Control */
        uint32_t RESERVED5[39U];
  __IOM uint32_t CLAIMSET;               /*!< Offset: 0xFA0 (R/W)  Claim tag set */
  __IOM uint32_t CLAIMCLR;               /*!< Offset: 0xFA4 (R/W)  Claim tag clear */
        uint32_t RESERVED7[8U];
  __IM  uint32_t DEVID;                  /*!< Offset: 0xFC8 (R/ )  TPIU_DEVID */
  __IM  uint32_t DEVTYPE;                /*!< Offset: 0xFCC (R/ )  TPIU_DEVTYPE */
} TPI_Type;

typedef struct
{
  __IM  uint32_t TYPE;                   /*!< Offset: 0x000 (R/ )  MPU Type Register */
  __IOM uint32_t CTRL;                   /*!< Offset: 0x004 (R/W)  MPU Control Register */
  __IOM uint32_t RNR;                    /*!< Offset: 0x008 (R/W)  MPU Region RNRber Register */
  __IOM uint32_t RBAR;                   /*!< Offset: 0x00C (R/W)  MPU Region Base Address Register */
  __IOM uint32_t RASR;                   /*!< Offset: 0x010 (R/W)  MPU Region Attribute and Size Register */
  __IOM uint32_t RBAR_A1;                /*!< Offset: 0x014 (R/W)  MPU Alias 1 Region Base Address Register */
  __IOM uint32_t RASR_A1;                /*!< Offset: 0x018 (R/W)  MPU Alias 1 Region Attribute and Size Register */
  __IOM uint32_t RBAR_A2;                /*!< Offset: 0x01C (R/W)  MPU Alias 2 Region Base Address Register */
  __IOM uint32_t RASR_A2;                /*!< Offset: 0x020 (R/W)  MPU Alias 2 Region Attribute and Size Register */
  __IOM uint32_t RBAR_A3;                /*!< Offset: 0x024 (R/W)  MPU Alias 3 Region Base Address Register */
  __IOM uint32_t RASR_A3;                /*!< Offset: 0x028 (R/W)  MPU Alias 3 Region Attribute and Size Register */
} MPU_Type;

typedef struct
{
        uint32_t RESERVED0[1U];
  __IOM uint32_t FPCCR;                  /*!< Offset: 0x004 (R/W)  Floating-Point Context Control Register */
  __IOM uint32_t FPCAR;                  /*!< Offset: 0x008 (R/W)  Floating-Point Context Address Register */
  __IOM uint32_t FPDSCR;                 /*!< Offset: 0x00C (R/W)  Floating-Point Default Status Control Register */
  __IM  uint32_t MVFR0;                  /*!< Offset: 0x010 (R/ )  Media and FP Feature Register 0 */
  __IM  uint32_t MVFR1;                  /*!< Offset: 0x014 (R/ )  Media and FP Feature Register 1 */
} FPU_Type;

typedef struct
{
  __IOM uint32_t DHCSR;                  /*!< Offset: 0x000 (R/W)  Debug Halting Control and Status Register */
  __OM  uint32_t DCRSR;                  /*!< Offset: 0x004 ( /W)  Debug Core Register Selector Register */
  __IOM uint32_t DCRDR;                  /*!< Offset: 0x008 (R/W)  Debug Core Register Data Register */
  __IOM uint32_t DEMCR;                  /*!< Offset: 0x00C (R/W)  Debug Exception and Monitor Control Register */
} CoreDebug_Type;

/* Memory mapping of Core Hardware */
#define SCS_BASE            			(0xE000E000UL)                            		/*!< System Control Space Base Address */
#define ITM_BASE            			(0xE0000000UL)                            		/*!< ITM Base Address */
#define DWT_BASE            			(0xE0001000UL)                            		/*!< DWT Base Address */
#define TPI_BASE            			(0xE0040000UL)                            		/*!< TPI Base Address */
#define CoreDebug_BASE      			(0xE000EDF0UL)                            		/*!< Core Debug Base Address */
#define SysTick_BASE        			(SCS_BASE +  0x0010UL)                    		/*!< SysTick Base Address */
#define NVIC_BASE           			(SCS_BASE +  0x0100UL)                    		/*!< NVIC Base Address */
#define SCB_BASE            			(SCS_BASE +  0x0D00UL)                    		/*!< System Control Block Base Address */
					
#define SCnSCB              			((SCnSCB_Type    *)     SCS_BASE      )   		/*!< System control Register not in SCB */
#define SCB                 			((SCB_Type       *)     SCB_BASE      )   		/*!< SCB configuration struct */
#define SysTick             			((SysTick_Type   *)     SysTick_BASE  )   		/*!< SysTick configuration struct */
#define NVIC                			((NVIC_Type      *)     NVIC_BASE     )   		/*!< NVIC configuration struct */
#define ITM                 			((ITM_Type       *)     ITM_BASE      )   		/*!< ITM configuration struct */
#define DWT                 			((DWT_Type       *)     DWT_BASE      )   		/*!< DWT configuration struct */
#define TPI                 			((TPI_Type       *)     TPI_BASE      )   		/*!< TPI configuration struct */
#define CoreDebug           			((CoreDebug_Type *)     CoreDebug_BASE)   		/*!< Core Debug configuration struct */
#define MPU_BASE          				(SCS_BASE +  0x0D90UL)                    		/*!< Memory Protection Unit */
#define MPU               				((MPU_Type       *)     MPU_BASE      )   		/*!< Memory Protection Unit */
#define FPU_BASE            			(SCS_BASE +  0x0F30UL)                    		/*!< Floating Point Unit */
#define FPU                 			((FPU_Type       *)     FPU_BASE      )   		/*!< Floating Point Unit */


#ifdef __cplusplus
}
#endif

#endif 

