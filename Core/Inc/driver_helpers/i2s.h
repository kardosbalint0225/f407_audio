#ifdef TEST

#ifndef __I2S_H
#define __I2S_H

#ifdef __cplusplus
extern "C" {
#endif

#include "def.h"
#include "dma.h"

typedef struct
{
  uint32_t Mode;                /*!< Specifies the I2S operating mode.
                                     This parameter can be a value of @ref I2S_Mode */
  uint32_t Standard;            /*!< Specifies the standard used for the I2S communication.
                                     This parameter can be a value of @ref I2S_Standard */
  uint32_t DataFormat;          /*!< Specifies the data format for the I2S communication.
                                     This parameter can be a value of @ref I2S_Data_Format */
  uint32_t MCLKOutput;          /*!< Specifies whether the I2S MCLK output is enabled or not.
                                     This parameter can be a value of @ref I2S_MCLK_Output */
  uint32_t AudioFreq;           /*!< Specifies the frequency selected for the I2S communication.
                                     This parameter can be a value of @ref I2S_Audio_Frequency */
  uint32_t CPOL;                /*!< Specifies the idle state of the I2S clock.
                                     This parameter can be a value of @ref I2S_Clock_Polarity */
  uint32_t ClockSource;         /*!< Specifies the I2S Clock Source.
                                     This parameter can be a value of @ref I2S_Clock_Source */
  uint32_t FullDuplexMode;      /*!< Specifies the I2S FullDuplex mode.
                                     This parameter can be a value of @ref I2S_FullDuplex_Mode */
} I2S_InitTypeDef;

typedef enum
{
  HAL_I2S_STATE_RESET      = 0x00U,  /*!< I2S not yet initialized or disabled                */
  HAL_I2S_STATE_READY      = 0x01U,  /*!< I2S initialized and ready for use                  */
  HAL_I2S_STATE_BUSY       = 0x02U,  /*!< I2S internal process is ongoing                    */
  HAL_I2S_STATE_BUSY_TX    = 0x03U,  /*!< Data Transmission process is ongoing               */
  HAL_I2S_STATE_BUSY_RX    = 0x04U,  /*!< Data Reception process is ongoing                  */
  HAL_I2S_STATE_BUSY_TX_RX = 0x05U,  /*!< Data Transmission and Reception process is ongoing */
  HAL_I2S_STATE_TIMEOUT    = 0x06U,  /*!< I2S timeout state                                  */
  HAL_I2S_STATE_ERROR      = 0x07U   /*!< I2S error state                                    */
} HAL_I2S_StateTypeDef;

typedef struct __I2S_HandleTypeDef
{
  SPI_TypeDef                *Instance;    /*!< I2S registers base address */
  I2S_InitTypeDef            Init;         /*!< I2S communication parameters */
  uint16_t                   *pTxBuffPtr;  /*!< Pointer to I2S Tx transfer buffer */
  __IO uint16_t              TxXferSize;   /*!< I2S Tx transfer size */
  __IO uint16_t              TxXferCount;  /*!< I2S Tx transfer Counter */
  uint16_t                   *pRxBuffPtr;  /*!< Pointer to I2S Rx transfer buffer */
  __IO uint16_t              RxXferSize;   /*!< I2S Rx transfer size */
  __IO uint16_t              RxXferCount;  /*!< I2S Rx transfer counter
                                              (This field is initialized at the
                                               same value as transfer size at the
                                               beginning of the transfer and
                                               decremented when a sample is received
                                               NbSamplesReceived = RxBufferSize-RxBufferCount) */
  void (*IrqHandlerISR)(struct __I2S_HandleTypeDef *hi2s);         /*!< I2S function pointer on IrqHandler   */
  DMA_HandleTypeDef          *hdmatx;      /*!< I2S Tx DMA handle parameters */
  DMA_HandleTypeDef          *hdmarx;      /*!< I2S Rx DMA handle parameters */
  __IO HAL_LockTypeDef       Lock;         /*!< I2S locking object */
  __IO HAL_I2S_StateTypeDef  State;        /*!< I2S communication state */
  __IO uint32_t              ErrorCode;    /*!< I2S Error code
                                                This parameter can be a value of @ref I2S_Error */

#if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
  void (* TxCpltCallback)(struct __I2S_HandleTypeDef *hi2s);             /*!< I2S Tx Completed callback          */
  void (* RxCpltCallback)(struct __I2S_HandleTypeDef *hi2s);             /*!< I2S Rx Completed callback          */
  void (* TxRxCpltCallback)(struct __I2S_HandleTypeDef *hi2s);           /*!< I2S TxRx Completed callback        */
  void (* TxHalfCpltCallback)(struct __I2S_HandleTypeDef *hi2s);         /*!< I2S Tx Half Completed callback     */
  void (* RxHalfCpltCallback)(struct __I2S_HandleTypeDef *hi2s);         /*!< I2S Rx Half Completed callback     */
  void (* TxRxHalfCpltCallback)(struct __I2S_HandleTypeDef *hi2s);       /*!< I2S TxRx Half Completed callback   */
  void (* ErrorCallback)(struct __I2S_HandleTypeDef *hi2s);              /*!< I2S Error callback                 */
  void (* MspInitCallback)(struct __I2S_HandleTypeDef *hi2s);            /*!< I2S Msp Init callback              */
  void (* MspDeInitCallback)(struct __I2S_HandleTypeDef *hi2s);          /*!< I2S Msp DeInit callback            */

#endif  /* USE_HAL_I2S_REGISTER_CALLBACKS */
} I2S_HandleTypeDef;

#if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)

typedef enum
{
  HAL_I2S_TX_COMPLETE_CB_ID             = 0x00U,    /*!< I2S Tx Completed callback ID         */
  HAL_I2S_RX_COMPLETE_CB_ID             = 0x01U,    /*!< I2S Rx Completed callback ID         */
  HAL_I2S_TX_RX_COMPLETE_CB_ID          = 0x02U,    /*!< I2S TxRx Completed callback ID       */
  HAL_I2S_TX_HALF_COMPLETE_CB_ID        = 0x03U,    /*!< I2S Tx Half Completed callback ID    */
  HAL_I2S_RX_HALF_COMPLETE_CB_ID        = 0x04U,    /*!< I2S Rx Half Completed callback ID    */
  HAL_I2S_TX_RX_HALF_COMPLETE_CB_ID     = 0x05U,    /*!< I2S TxRx Half Completed callback ID  */
  HAL_I2S_ERROR_CB_ID                   = 0x06U,    /*!< I2S Error callback ID                */
  HAL_I2S_MSPINIT_CB_ID                 = 0x07U,    /*!< I2S Msp Init callback ID             */
  HAL_I2S_MSPDEINIT_CB_ID               = 0x08U     /*!< I2S Msp DeInit callback ID           */

} HAL_I2S_CallbackIDTypeDef;

typedef  void (*pI2S_CallbackTypeDef)(I2S_HandleTypeDef *hi2s); /*!< pointer to an I2S callback function */

#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */

#define HAL_I2S_ERROR_NONE               (0x00000000U)  /*!< No error                    */
#define HAL_I2S_ERROR_TIMEOUT            (0x00000001U)  /*!< Timeout error               */
#define HAL_I2S_ERROR_OVR                (0x00000002U)  /*!< OVR error                   */
#define HAL_I2S_ERROR_UDR                (0x00000004U)  /*!< UDR error                   */
#define HAL_I2S_ERROR_DMA                (0x00000008U)  /*!< DMA transfer error          */
#define HAL_I2S_ERROR_PRESCALER          (0x00000010U)  /*!< Prescaler Calculation error */
#if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
#define HAL_I2S_ERROR_INVALID_CALLBACK   (0x00000020U)  /*!< Invalid Callback error      */
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
#define HAL_I2S_ERROR_BUSY_LINE_RX       (0x00000040U)  /*!< Busy Rx Line error          */

#define I2S_MODE_SLAVE_TX                (0x00000000U)
#define I2S_MODE_SLAVE_RX                (SPI_I2SCFGR_I2SCFG_0)
#define I2S_MODE_MASTER_TX               (SPI_I2SCFGR_I2SCFG_1)
#define I2S_MODE_MASTER_RX               ((SPI_I2SCFGR_I2SCFG_0 | SPI_I2SCFGR_I2SCFG_1))

#define I2S_STANDARD_PHILIPS             (0x00000000U)
#define I2S_STANDARD_MSB                 (SPI_I2SCFGR_I2SSTD_0)
#define I2S_STANDARD_LSB                 (SPI_I2SCFGR_I2SSTD_1)
#define I2S_STANDARD_PCM_SHORT           ((SPI_I2SCFGR_I2SSTD_0 | SPI_I2SCFGR_I2SSTD_1))
#define I2S_STANDARD_PCM_LONG            ((SPI_I2SCFGR_I2SSTD_0 | SPI_I2SCFGR_I2SSTD_1 | SPI_I2SCFGR_PCMSYNC))

#define I2S_DATAFORMAT_16B               (0x00000000U)
#define I2S_DATAFORMAT_16B_EXTENDED      (SPI_I2SCFGR_CHLEN)
#define I2S_DATAFORMAT_24B               ((SPI_I2SCFGR_CHLEN | SPI_I2SCFGR_DATLEN_0))
#define I2S_DATAFORMAT_32B               ((SPI_I2SCFGR_CHLEN | SPI_I2SCFGR_DATLEN_1))

#define I2S_MCLKOUTPUT_ENABLE            (SPI_I2SPR_MCKOE)
#define I2S_MCLKOUTPUT_DISABLE           (0x00000000U)

#define I2S_AUDIOFREQ_192K               (192000U)
#define I2S_AUDIOFREQ_96K                (96000U)
#define I2S_AUDIOFREQ_48K                (48000U)
#define I2S_AUDIOFREQ_44K                (44100U)
#define I2S_AUDIOFREQ_32K                (32000U)
#define I2S_AUDIOFREQ_22K                (22050U)
#define I2S_AUDIOFREQ_16K                (16000U)
#define I2S_AUDIOFREQ_11K                (11025U)
#define I2S_AUDIOFREQ_8K                 (8000U)
#define I2S_AUDIOFREQ_DEFAULT            (2U)

#define I2S_FULLDUPLEXMODE_DISABLE       (0x00000000U)
#define I2S_FULLDUPLEXMODE_ENABLE        (0x00000001U)

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) ||     defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) ||     defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F469xx) ||     defined(STM32F479xx)
#define I2S_CLOCK_PLL                    (0x00000000U)
#define I2S_CLOCK_EXTERNAL               (0x00000001U)
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx ||
          STM32F401xC || STM32F401xE || STM32F411xE || STM32F469xx || STM32F479xx */


#include "i2s_ex.h"

HAL_StatusTypeDef HAL_I2S_Init(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DeInit(I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);

#if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
HAL_StatusTypeDef HAL_I2S_RegisterCallback(I2S_HandleTypeDef *hi2s, HAL_I2S_CallbackIDTypeDef CallbackID,
                                           pI2S_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_I2S_UnRegisterCallback(I2S_HandleTypeDef *hi2s, HAL_I2S_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */

HAL_StatusTypeDef HAL_I2S_Transmit(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2S_Receive(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);

HAL_StatusTypeDef HAL_I2S_Transmit_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
void HAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s);

HAL_StatusTypeDef HAL_I2S_Transmit_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2S_DMAPause(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAResume(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAStop(I2S_HandleTypeDef *hi2s);

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s);

HAL_I2S_StateTypeDef HAL_I2S_GetState(I2S_HandleTypeDef *hi2s);
uint32_t HAL_I2S_GetError(I2S_HandleTypeDef *hi2s);

#ifdef __cplusplus
}
#endif

#endif /* __I2S_H */

#endif


