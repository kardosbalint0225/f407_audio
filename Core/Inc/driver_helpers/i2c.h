#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "def.h"
#include "dma.h"

typedef struct
{
  uint32_t ClockSpeed;       							/*!< Specifies the clock frequency.
															This parameter must be set to a value lower than 400kHz */
							
  uint32_t DutyCycle;        							/*!< Specifies the I2C fast mode duty cycle.
															This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */
							
  uint32_t OwnAddress1;      							/*!< Specifies the first device own address.
															This parameter can be a 7-bit or 10-bit address. */
							
  uint32_t AddressingMode;   							/*!< Specifies if 7-bit or 10-bit addressing mode is selected.
															This parameter can be a value of @ref I2C_addressing_mode */
							
  uint32_t DualAddressMode;  							/*!< Specifies if dual addressing mode is selected.
															This parameter can be a value of @ref I2C_dual_addressing_mode */
							
  uint32_t OwnAddress2;      							/*!< Specifies the second device own address if dual addressing mode is selected
															This parameter can be a 7-bit address. */
							
  uint32_t GeneralCallMode;  							/*!< Specifies if general call mode is selected.
															This parameter can be a value of @ref I2C_general_call_addressing_mode */
							
  uint32_t NoStretchMode;    							/*!< Specifies if nostretch mode is selected.
															This parameter can be a value of @ref I2C_nostretch_mode */

} I2C_InitTypeDef;

typedef enum
{
  HAL_I2C_STATE_RESET             = 0x00U,   			/*!< Peripheral is not yet Initialized         */
  HAL_I2C_STATE_READY             = 0x20U,   			/*!< Peripheral Initialized and ready for use  */
  HAL_I2C_STATE_BUSY              = 0x24U,   			/*!< An internal process is ongoing            */
  HAL_I2C_STATE_BUSY_TX           = 0x21U,   			/*!< Data Transmission process is ongoing      */
  HAL_I2C_STATE_BUSY_RX           = 0x22U,   			/*!< Data Reception process is ongoing         */
  HAL_I2C_STATE_LISTEN            = 0x28U,   			/*!< Address Listen Mode is ongoing            */
  HAL_I2C_STATE_BUSY_TX_LISTEN    = 0x29U,   			/*!< Address Listen Mode and Data Transmission
															process is ongoing                         */
  HAL_I2C_STATE_BUSY_RX_LISTEN    = 0x2AU,   			/*!< Address Listen Mode and Data Reception
															process is ongoing                         */
  HAL_I2C_STATE_ABORT             = 0x60U,   			/*!< Abort user request ongoing                */
  HAL_I2C_STATE_TIMEOUT           = 0xA0U,   			/*!< Timeout state                             */
  HAL_I2C_STATE_ERROR             = 0xE0U    			/*!< Error                                     */

} HAL_I2C_StateTypeDef;

typedef enum
{
  HAL_I2C_MODE_NONE               = 0x00U,   			/*!< No I2C communication on going             */
  HAL_I2C_MODE_MASTER             = 0x10U,   			/*!< I2C communication is in Master Mode       */
  HAL_I2C_MODE_SLAVE              = 0x20U,   			/*!< I2C communication is in Slave Mode        */
  HAL_I2C_MODE_MEM                = 0x40U    			/*!< I2C communication is in Memory Mode       */

} HAL_I2C_ModeTypeDef;

typedef struct
{
  I2C_TypeDef                *Instance;      			/*!< I2C registers base address               */
  I2C_InitTypeDef            Init;           			/*!< I2C communication parameters             */
  uint8_t                    *pBuffPtr;      			/*!< Pointer to I2C transfer buffer           */
  uint16_t                   XferSize;       			/*!< I2C transfer size                        */
  __IO uint16_t              XferCount;      			/*!< I2C transfer counter                     */
  __IO uint32_t              XferOptions;    			/*!< I2C transfer options                     */
  __IO uint32_t              PreviousState;  			/*!< I2C communication Previous state and mode
															context for internal usage               */
  DMA_HandleTypeDef          *hdmatx;        			/*!< I2C Tx DMA handle parameters             */
  DMA_HandleTypeDef          *hdmarx;        			/*!< I2C Rx DMA handle parameters             */
  HAL_LockTypeDef            Lock;           			/*!< I2C locking object                       */
  __IO HAL_I2C_StateTypeDef  State;          			/*!< I2C communication state                  */
  __IO HAL_I2C_ModeTypeDef   Mode;           			/*!< I2C communication mode                   */
  __IO uint32_t              ErrorCode;      			/*!< I2C Error code                           */
  __IO uint32_t              Devaddress;     			/*!< I2C Target device address                */
  __IO uint32_t              Memaddress;     			/*!< I2C Target memory address                */
  __IO uint32_t              MemaddSize;     			/*!< I2C Target memory address  size          */
  __IO uint32_t              EventCount;     			/*!< I2C Event counter                        */

} I2C_HandleTypeDef;

#define I2C_DUTYCYCLE_2                			0x00000000U
#define I2C_DUTYCYCLE_16_9             			I2C_CCR_DUTY
#define I2C_ADDRESSINGMODE_7BIT        			0x00004000U
#define I2C_ADDRESSINGMODE_10BIT       			(I2C_OAR1_ADDMODE | 0x00004000U)
#define I2C_DUALADDRESS_DISABLE        			0x00000000U
#define I2C_DUALADDRESS_ENABLE         			I2C_OAR2_ENDUAL
#define I2C_GENERALCALL_DISABLE        			0x00000000U
#define I2C_GENERALCALL_ENABLE         			I2C_CR1_ENGC
#define I2C_NOSTRETCH_DISABLE          			0x00000000U
#define I2C_NOSTRETCH_ENABLE           			I2C_CR1_NOSTRETCH

#include "i2c_ex.h"

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);

void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);

HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
HAL_I2C_ModeTypeDef HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c);
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *hi2c);


#ifdef __cplusplus
}
#endif

#endif 

