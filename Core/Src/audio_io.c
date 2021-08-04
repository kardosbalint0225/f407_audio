#include "audio_io.h"

#ifdef TEST
	#include "cortex.h"
	#include "dma.h"
	#include "gpio.h"
	#include "hal.h"
	#include "i2c.h"
#else
	#include "stm32f4xx_hal.h"
#endif

volatile bool is_ready = false;

I2C_HandleTypeDef hi2c;
DMA_HandleTypeDef hdma_i2c_tx;
DMA_HandleTypeDef hdma_i2c_rx;

static audio_io_err_t audio_io_error;

static HAL_StatusTypeDef I2Cx_Init(void);
static HAL_StatusTypeDef I2Cx_DeInit(void);

static void I2Cx_ErrorHandler(void);
static void DMA_Init(void);
static void DMA_DeInit(void);
static void audio_io_reset(void);
static void audio_io_reset_pin_init(void);
static void audio_io_reset_pin_deinit(void);

void I2Cx_MspInit(I2C_HandleTypeDef *hi2c);
void I2Cx_MspDeInit(I2C_HandleTypeDef *hi2c);
void I2Cx_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void I2Cx_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void I2Cx_ErrorCallback(I2C_HandleTypeDef *hi2c);

#ifdef TEST
void audio_io_error_reset(void)
{
	audio_io_error.i2c.w = 0;
}
#endif

audio_status_t audio_io_init(void)
{	
	HAL_StatusTypeDef status;
	audio_status_t retc;

	audio_io_reset_pin_init();
	audio_io_reset();
	
	DMA_Init();
	
	status = I2Cx_Init();
	retc = (HAL_OK != status) ? (AUDIO_IO_I2C_INIT_ERROR) : (AUDIO_IO_OK);
	
	return retc;
}

static void audio_io_reset_pin_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__AUDIO_IO_RESET_GPIO_CLK_ENABLE();
	
	GPIO_InitStruct.Pin   = AUDIO_IO_RESET_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;		
	HAL_GPIO_Init(AUDIO_IO_RESET_PORT, &GPIO_InitStruct);
}

static void audio_io_reset_pin_deinit(void)
{
	HAL_GPIO_DeInit(AUDIO_IO_RESET_PORT, AUDIO_IO_RESET_PIN);
	__AUDIO_IO_RESET_GPIO_CLK_DISABLE();
}

static void audio_io_reset(void)
{	
	HAL_GPIO_WritePin(AUDIO_IO_RESET_PORT, AUDIO_IO_RESET_PIN, GPIO_PIN_RESET);		/* Power Down the codec */
	HAL_Delay(5); 																	/* Wait for a delay to ensure registers erasing */
	HAL_GPIO_WritePin(AUDIO_IO_RESET_PORT, AUDIO_IO_RESET_PIN, GPIO_PIN_SET);		/* Power on the codec */
}

static void DMA_Init(void)
{
	__I2CxTX_DMA_CLK_ENABLE();
	__I2CxRX_DMA_CLK_ENABLE();
	HAL_NVIC_SetPriority(DMA_I2CxTX_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA_I2CxTX_Stream_IRQn);
	HAL_NVIC_SetPriority(DMA_I2CxRX_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA_I2CxRX_Stream_IRQn);
}

static HAL_StatusTypeDef I2Cx_Init(void)
{	
	HAL_StatusTypeDef status;
	uint32_t trials  = 3;
	uint32_t timeout = 1;

	audio_io_error.i2c.w = 0;

	hi2c.Instance             = I2Cx;
	hi2c.Init.ClockSpeed      = I2Cx_CLK_SPEED;
	hi2c.Init.DutyCycle       = I2C_DUTYCYCLE_2;
	hi2c.Init.OwnAddress1     = 0;							//I2cHandle.Init.OwnAddress1 = 0x33;
	hi2c.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c.Init.OwnAddress2     = 0;
	hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;	
	
	if (HAL_I2C_STATE_RESET != HAL_I2C_GetState(&hi2c)) {
		audio_io_error.i2c.state = 1;
	}
	
	HAL_I2C_RegisterCallback(&hi2c, HAL_I2C_MSPINIT_CB_ID, I2Cx_MspInit);
	HAL_I2C_RegisterCallback(&hi2c, HAL_I2C_MSPDEINIT_CB_ID, I2Cx_MspDeInit);

	if (HAL_OK != HAL_I2C_Init(&hi2c)) {
		audio_io_error.i2c.init = 1;
	}
	
	HAL_I2C_RegisterCallback(&hi2c, HAL_I2C_MASTER_TX_COMPLETE_CB_ID, I2Cx_MasterTxCpltCallback);
	HAL_I2C_RegisterCallback(&hi2c, HAL_I2C_MEM_RX_COMPLETE_CB_ID, I2Cx_MemRxCpltCallback);
	HAL_I2C_RegisterCallback(&hi2c, HAL_I2C_ERROR_CB_ID, I2Cx_ErrorCallback);

	if (HAL_OK != HAL_I2C_IsDeviceReady(&hi2c, AUDIO_IO_DEVICE_ADDRESS, trials, timeout)) {
		audio_io_error.i2c.ready = 1;
	}

	status = (0 != audio_io_error.i2c.w) ? (HAL_ERROR) : (HAL_OK);

	return status;
}

void I2Cx_MspInit(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__I2Cx_SCL_GPIO_CLK_ENABLE();
	__I2Cx_SDA_GPIO_CLK_ENABLE();
	
	GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
	HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin       = I2Cx_SDA_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
	HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);

	__I2Cx_CLK_ENABLE();		/* I2Cx clock enable 						*/
	__I2Cx_FORCE_RESET();		/* Force the I2C peripheral clock reset 	*/
	__I2Cx_RELEASE_RESET();		/* Release the I2C peripheral clock reset 	*/
	
	hdma_i2c_tx.Instance                 = I2CxTX_DMA_STREAM;
	hdma_i2c_tx.Init.Channel             = I2CxTX_DMA_CHANNEL;
	hdma_i2c_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	hdma_i2c_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_i2c_tx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_i2c_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_i2c_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_i2c_tx.Init.Mode                = DMA_NORMAL;
	hdma_i2c_tx.Init.Priority            = DMA_PRIORITY_LOW;
	hdma_i2c_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	hdma_i2c_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_1QUARTERFULL;
	hdma_i2c_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
	hdma_i2c_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;

	hdma_i2c_rx.Instance                 = I2CxRX_DMA_STREAM;
	hdma_i2c_rx.Init.Channel             = I2CxRX_DMA_CHANNEL;
	hdma_i2c_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	hdma_i2c_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_i2c_rx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_i2c_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_i2c_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_i2c_rx.Init.Mode                = DMA_NORMAL;
	hdma_i2c_rx.Init.Priority            = DMA_PRIORITY_LOW;
	hdma_i2c_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	hdma_i2c_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_1QUARTERFULL;
	hdma_i2c_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
	hdma_i2c_rx.Init.PeriphBurst         = DMA_PBURST_SINGLE;

	if (HAL_OK != HAL_DMA_Init(&hdma_i2c_tx)) {
		audio_io_error.i2c.dmatx = 1;
	}

	__HAL_LINKDMA(hi2c, hdmatx, hdma_i2c_tx);

	if (HAL_OK != HAL_DMA_Init(&hdma_i2c_rx)) {
		audio_io_error.i2c.dmarx = 1;
	}

	__HAL_LINKDMA(hi2c, hdmarx, hdma_i2c_rx);
}

audio_status_t audio_io_deinit(void)
{
	HAL_StatusTypeDef status;
	audio_status_t retc;

	audio_io_reset_pin_deinit();
	
	status = I2Cx_DeInit();
	
	DMA_DeInit();
	
	retc = (HAL_OK != status) ? (AUDIO_IO_I2C_DEINIT_ERROR) : (AUDIO_IO_OK);

	return retc;
}

static void DMA_DeInit(void)
{
	HAL_NVIC_DisableIRQ(DMA_I2CxTX_Stream_IRQn);
	HAL_NVIC_DisableIRQ(DMA_I2CxRX_Stream_IRQn);
	__I2CxTX_DMA_CLK_DISABLE();
	__I2CxRX_DMA_CLK_DISABLE();
}

static HAL_StatusTypeDef I2Cx_DeInit(void)
{
	HAL_StatusTypeDef status;
	
	HAL_I2C_UnRegisterCallback(&hi2c, HAL_I2C_MASTER_TX_COMPLETE_CB_ID);
	HAL_I2C_UnRegisterCallback(&hi2c, HAL_I2C_MEM_RX_COMPLETE_CB_ID);
	HAL_I2C_UnRegisterCallback(&hi2c, HAL_I2C_ERROR_CB_ID);

	if (HAL_OK != HAL_I2C_DeInit(&hi2c)) {
		audio_io_error.i2c.deinit = 1;
	}
	
	HAL_I2C_UnRegisterCallback(&hi2c, HAL_I2C_MSPINIT_CB_ID);
	HAL_I2C_UnRegisterCallback(&hi2c, HAL_I2C_MSPDEINIT_CB_ID);
	
	if (HAL_I2C_STATE_RESET != HAL_I2C_GetState(&hi2c)) {
		audio_io_error.i2c.state = 1;
	}
	
	status = (0 != audio_io_error.i2c.w) ? (HAL_ERROR) : (HAL_OK);

	return status;
}

void I2Cx_MspDeInit(I2C_HandleTypeDef *hi2c)
{
	__I2Cx_CLK_DISABLE();

    HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
    HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);

    if (HAL_OK != HAL_DMA_DeInit(&hdma_i2c_tx)) {
		audio_io_error.i2c.dmatx = 1;
    }

	if (HAL_OK != HAL_DMA_DeInit(&hdma_i2c_rx)) {
		audio_io_error.i2c.dmarx = 1;
    }
}


void I2Cx_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}

void I2Cx_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	is_ready = true;
}

void I2Cx_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	I2Cx_ErrorHandler();
}

static void I2Cx_ErrorHandler(void)
{
	I2Cx_DeInit();
	I2Cx_Init();
}

audio_status_t audio_io_read(uint8_t register_address, uint8_t *data, uint8_t size, bool blocking)
{
	HAL_StatusTypeDef status;
	audio_status_t retc;
	uint16_t address      = (uint16_t)register_address;
	uint16_t address_size = sizeof(uint8_t);
	uint16_t data_size    = (uint16_t)size;
	uint32_t timeout      = 10;

	is_ready = false;

	if (true == blocking) {
		status = HAL_I2C_Mem_Read(&hi2c, AUDIO_IO_DEVICE_ADDRESS, address, address_size, data, data_size, timeout);
	} else {
		status = HAL_I2C_Mem_Read_DMA(&hi2c, AUDIO_IO_DEVICE_ADDRESS, address, address_size, data, data_size);
	}

	retc = (HAL_OK != status) ? (AUDIO_IO_I2C_READ_ERROR) : (AUDIO_IO_OK);

	return retc;
}

