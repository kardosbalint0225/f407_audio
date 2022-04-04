#include "debug_uart.h"
#include <stdio.h>
#include <errno.h>
#include <stdbool.h>
#include "fifo.h"

#ifdef TEST
	#include "cortex.h"
	#include "dma.h"
	#include "gpio.h"
	#include "hal.h"
	#include "uart.h"
    #include "rcc.h"
#else
	#include "stm32f4xx_hal.h"
    #include <sys/unistd.h>
#endif

UART_HandleTypeDef huart;
DMA_HandleTypeDef hdma_uart_tx;
static debug_uart_err_t debug_uart_error;
static bool uart_dma_tx_status_busy;
static fifo_t uart_tx_fifo;
static uint8_t uart_tx_buffer[512] __attribute__((aligned(512)));

static HAL_StatusTypeDef UARTx_Init(void);
static HAL_StatusTypeDef UARTx_DeInit(void);
static void DMA_Init(void);
static void DMA_DeInit(void);
static void UARTx_ErrorHandler(void);

int _write(int file, char *ptr, int len);

void UARTx_MspInit(UART_HandleTypeDef* huart);
void UARTx_MspDeInit(UART_HandleTypeDef* huart);
void UARTx_TxCpltCallback(UART_HandleTypeDef *huart);
void UARTx_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef TEST
void debug_uart_error_reset(void)
{
	debug_uart_error.w = 0;
}
#endif

int _write(int file, char *ptr, int len)
{
    HAL_StatusTypeDef status;
    int retc;

    if ((STDOUT_FILENO != file) && (STDERR_FILENO != file)) {
        errno = EBADF;
        return -1;
    }

    if (false == uart_dma_tx_status_busy) {

        for (int i = 0; i < len; i++) {
            uart_tx_buffer[i] = ptr[i];
        }

        status = HAL_UART_Transmit_DMA(&huart, uart_tx_buffer, (uint16_t)len);
        uart_dma_tx_status_busy = true;
        
        if (HAL_OK == status) {
            retc = len;
        } else {
            retc = -1;
            debug_uart_error.dmatx = 1;
        }

    } else {

        if (false == fifo_is_full(&uart_tx_fifo)) {
            fifo_push(&uart_tx_fifo, (uint8_t *)ptr, (uint32_t)len);
            retc = len;
        } else {
            retc = -1;
            debug_uart_error.fifo_overrun = 1;
        }
    }
    
	return retc;
}

debug_uart_status_t debug_uart_init(void)
{
    HAL_StatusTypeDef status;
    debug_uart_status_t retc;
    uart_dma_tx_status_busy = false;

    fifo_init(&uart_tx_fifo);

    for (uint32_t i = 0; i < FIFO_ELEMENT_MAX_SIZE; i++) {
        uart_tx_buffer[i] = 0;
    }

    DMA_Init();
    status = UARTx_Init();

    retc = (HAL_OK != status) ? (DEBUG_UART_INIT_ERROR) : (DEBUG_UART_OK);
    return retc;
}

debug_uart_status_t debug_uart_deinit(void)
{
    HAL_StatusTypeDef status;
    debug_uart_status_t retc;

    DMA_DeInit();
    status = UARTx_DeInit();

    retc = (HAL_OK != status) ? (DEBUG_UART_DEINIT_ERROR) : (DEBUG_UART_OK);
    return retc;
}

static HAL_StatusTypeDef UARTx_Init(void)
{
    HAL_StatusTypeDef status;
    debug_uart_error.w = 0;

    huart.Instance          = USARTx;
    huart.Init.BaudRate     = UARTx_BAUDRATE;
    huart.Init.WordLength   = UART_WORDLENGTH_8B;
    huart.Init.StopBits     = UART_STOPBITS_1;
    huart.Init.Parity       = UART_PARITY_NONE;
    huart.Init.Mode         = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_STATE_RESET != HAL_UART_GetState(&huart)) {
        debug_uart_error.state = 1;
    }

    HAL_UART_RegisterCallback(&huart, HAL_UART_MSPINIT_CB_ID, UARTx_MspInit);
    HAL_UART_RegisterCallback(&huart, HAL_UART_MSPDEINIT_CB_ID, UARTx_MspDeInit);

    if (HAL_OK != HAL_UART_Init(&huart)) {
        debug_uart_error.init = 1;
    }

    HAL_UART_RegisterCallback(&huart, HAL_UART_TX_COMPLETE_CB_ID, UARTx_TxCpltCallback);
    HAL_UART_RegisterCallback(&huart, HAL_UART_ERROR_CB_ID, UARTx_ErrorCallback);
    HAL_UART_UnRegisterCallback(&huart, HAL_UART_TX_HALFCOMPLETE_CB_ID);

    status = (0 != debug_uart_error.w) ? (HAL_ERROR) : (HAL_OK);

	return status;
}

static HAL_StatusTypeDef UARTx_DeInit(void)
{
    HAL_StatusTypeDef status;

    HAL_UART_UnRegisterCallback(&huart, HAL_UART_TX_COMPLETE_CB_ID);
    HAL_UART_UnRegisterCallback(&huart, HAL_UART_ERROR_CB_ID);

    if (HAL_OK != HAL_UART_DeInit(&huart)) {
        debug_uart_error.deinit = 1;
    }

    HAL_UART_UnRegisterCallback(&huart, HAL_UART_MSPINIT_CB_ID);
    HAL_UART_UnRegisterCallback(&huart, HAL_UART_MSPDEINIT_CB_ID);

    if (HAL_UART_STATE_RESET != HAL_UART_GetState(&huart)) {
        debug_uart_error.state = 1;
    }

    status = (0 != debug_uart_error.w) ? (HAL_ERROR) : (HAL_OK);

	return status;
}

static void DMA_Init(void)
{
    __UARTxTX_DMA_CLK_ENABLE();
    HAL_NVIC_SetPriority(UARTxTX_DMA_Stream_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UARTxTX_DMA_Stream_IRQn);
}

static void DMA_DeInit(void)
{
    HAL_NVIC_DisableIRQ(UARTxTX_DMA_Stream_IRQn);
}

void UARTx_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    __UARTx_TX_GPIO_CLK_ENABLE();
    __UARTx_RX_GPIO_CLK_ENABLE();
    
    GPIO_InitStruct.Pin       = UARTx_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = UARTx_TX_RX_AF;
    HAL_GPIO_Init(UARTx_TX_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = UARTx_RX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = UARTx_TX_RX_AF;
    HAL_GPIO_Init(UARTx_RX_GPIO_PORT, &GPIO_InitStruct);

    __UARTx_CLK_ENABLE();
    __UARTx_FORCE_RESET();
    __UARTx_RELEASE_RESET(); 

    hdma_uart_tx.Instance                 = UARTxTX_DMA_STREAM;
    hdma_uart_tx.Init.Channel             = UARTxTX_DMA_CHANNEL;
    hdma_uart_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_uart_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart_tx.Init.Mode                = DMA_NORMAL;
    hdma_uart_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_uart_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_uart_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma_uart_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_uart_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    if (HAL_OK != HAL_DMA_Init(&hdma_uart_tx)) {
        debug_uart_error.dmatx = 1;
    }

    __HAL_LINKDMA(huart, hdmatx, hdma_uart_tx);

    HAL_NVIC_SetPriority(USARTx_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

void UARTx_MspDeInit(UART_HandleTypeDef* huart)
{
    __UARTx_CLK_DISABLE();

    HAL_GPIO_DeInit(UARTx_TX_GPIO_PORT, UARTx_TX_PIN);
    HAL_GPIO_DeInit(UARTx_RX_GPIO_PORT, UARTx_RX_PIN);

    if (HAL_OK != HAL_DMA_DeInit(&hdma_uart_tx)) {
        debug_uart_error.dmatx = 1;
    }

    HAL_NVIC_DisableIRQ(USARTx_IRQn);
}

void UARTx_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (false == fifo_is_empty(&uart_tx_fifo)) {
        uint16_t len = (uint16_t)fifo_pull(&uart_tx_fifo, uart_tx_buffer);
        HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, uart_tx_buffer, len);

        if (HAL_OK != status) {
            debug_uart_error.dmatx = 1;
        }

    } else {
        uart_dma_tx_status_busy = false;
    }
}

void UARTx_ErrorCallback(UART_HandleTypeDef *huart)
{
    UARTx_ErrorHandler();
}

static void UARTx_ErrorHandler(void)
{
    UARTx_DeInit();
    UARTx_Init();
}

