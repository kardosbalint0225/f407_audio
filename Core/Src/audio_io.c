#include "audio_io.h"

#ifdef TEST
	#include "cortex.h"
	#include "dma.h"
	#include "gpio.h"
	#include "hal.h"
	#include "i2c.h"
	#include "i2s.h"
	#include "rcc.h"
#else
	#include "stm32f4xx_hal.h"
#endif

static const uint32_t PLLI2Sx_CLK   = 1000000U;
static const uint32_t PLLI2Sx_N[11] = { 256U,   429U,   384U,   213U,   429U,   424U,   213U,   271U,   344U,   271U,   344U};
static const uint32_t PLLI2Sx_R[11] = {   5U,     4U,     5U,     2U,     4U,     3U,     2U,     2U,     7U,     2U,     2U};
static const uint32_t I2Sx_Fs[11]   = {8000U, 11025U, 12000U, 16000U, 22050U, 24000U, 32000U, 44100U, 48000U, 88200U, 96000U};

volatile bool audio_io_i2c_rx_cplt = false;
volatile bool audio_io_i2c_tx_cplt = false;

I2C_HandleTypeDef hi2c;
I2S_HandleTypeDef hi2s;
DMA_HandleTypeDef hdma_i2c_tx;
DMA_HandleTypeDef hdma_i2c_rx;
DMA_HandleTypeDef hdma_i2s_tx;

static audio_out_ll_t audio_out_ll = {
	.hw_params = {
		.standard        = 0,
		.data_format     = 0,
		.audio_frequency = 0,
	},
	.cb_params = {
		.write_callback  = NULL,
		.m0_buffer       = NULL,
		.m1_buffer       = NULL,
	},
};

static audio_io_err_t audio_io_error;

static HAL_StatusTypeDef I2Cx_Init(void);
static HAL_StatusTypeDef I2Cx_DeInit(void);
static HAL_StatusTypeDef I2Sx_Init(audio_out_ll_hw_params_t *haout);
static HAL_StatusTypeDef I2Sx_DeInit(void);

static void I2Cx_ErrorHandler(void);
static void I2Sx_ErrorHandler(void);
static void DMA_Init(void);
static void DMA_DeInit(void);
static void audio_io_reset(void);
static void audio_io_reset_pin_init(void);
static void audio_io_reset_pin_deinit(void);

void I2Cx_MspInit(I2C_HandleTypeDef *hi2c);
void I2Cx_MspDeInit(I2C_HandleTypeDef *hi2c);
void I2Cx_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void I2Cx_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void I2Cx_ErrorCallback(I2C_HandleTypeDef *hi2c);
void I2Sx_MspInit(I2S_HandleTypeDef *hi2s);
void I2Sx_MspDeInit(I2S_HandleTypeDef *hi2s);
void I2Sx_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void I2Sx_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void I2Sx_ErrorCallback(I2S_HandleTypeDef *hi2s);

uint32_t find(const uint32_t *array, const uint32_t size, const uint32_t value);

#ifdef TEST
void audio_io_error_reset(void)
{
	audio_io_error.i2c.w = 0;
	audio_io_error.i2s.w = 0;
}

void audio_io_get_error(uint32_t *i2c, uint32_t *i2s)
{
	*i2c = audio_io_error.i2c.w;
	*i2s = audio_io_error.i2s.w;
}

void audio_out_ll_set_params(audio_out_ll_t *haout)
{
	audio_out_ll.hw_params.standard        = haout->hw_params.standard;
	audio_out_ll.hw_params.data_format     = haout->hw_params.data_format;
	audio_out_ll.hw_params.audio_frequency = haout->hw_params.audio_frequency;
	audio_out_ll.cb_params.write_callback  = haout->cb_params.write_callback;
	audio_out_ll.cb_params.m0_buffer		= haout->cb_params.m0_buffer;
	audio_out_ll.cb_params.m1_buffer       = haout->cb_params.m1_buffer;
}
#endif

/**< ****************************************************************************************************************************** */
/**< DMA initialization/deinitialization																					        */
/**< ****************************************************************************************************************************** */
static void DMA_Init(void)
{
	__I2CxTX_DMA_CLK_ENABLE();
	__I2CxRX_DMA_CLK_ENABLE();
	HAL_NVIC_SetPriority(I2CxTX_DMA_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2CxTX_DMA_Stream_IRQn);
	HAL_NVIC_SetPriority(I2CxRX_DMA_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2CxRX_DMA_Stream_IRQn);

	__I2SxTX_DMA_CLK_ENABLE();
	HAL_NVIC_SetPriority(I2SxTX_DMA_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2SxTX_DMA_Stream_IRQn);
}

static void DMA_DeInit(void)
{
	HAL_NVIC_DisableIRQ(I2CxTX_DMA_Stream_IRQn);
	HAL_NVIC_DisableIRQ(I2CxRX_DMA_Stream_IRQn);
	HAL_NVIC_DisableIRQ(I2SxTX_DMA_Stream_IRQn);
}


/**< ****************************************************************************************************************************** */
/**< Audio I/O initialization 																								        */
/**< ****************************************************************************************************************************** */
audio_status_t audio_io_init(void)
{	
	HAL_StatusTypeDef status;
	audio_status_t retc;

	audio_io_reset_pin_init();
	audio_io_reset();
	
	DMA_Init();
	
	status = I2Cx_Init();
	retc = (HAL_OK != status) ? (AUDIO_IO_INIT_ERROR) : (AUDIO_IO_OK);
	
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

static void audio_io_reset(void)
{	
	HAL_GPIO_WritePin(AUDIO_IO_RESET_PORT, AUDIO_IO_RESET_PIN, GPIO_PIN_RESET);		/* Power Down the codec */
	HAL_Delay(5); 																	/* Wait for a delay to ensure registers erasing */
	HAL_GPIO_WritePin(AUDIO_IO_RESET_PORT, AUDIO_IO_RESET_PIN, GPIO_PIN_SET);		/* Power on the codec */
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
	hi2c.Init.OwnAddress1     = 0;							// OwnAddress1 = 0x33;
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
	
	HAL_I2C_RegisterCallback(&hi2c, HAL_I2C_MEM_TX_COMPLETE_CB_ID, I2Cx_MemTxCpltCallback);
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

	if (HAL_DMA_STATE_RESET != HAL_DMA_GetState(&hdma_i2c_tx)) {
		audio_io_error.i2c.dmatx_state = 1;
	}

	if (HAL_OK != HAL_DMA_Init(&hdma_i2c_tx)) {
		audio_io_error.i2c.dmatx = 1;
	}

	__HAL_LINKDMA(hi2c, hdmatx, hdma_i2c_tx);

	if (HAL_DMA_STATE_RESET != HAL_DMA_GetState(&hdma_i2c_rx)) {
		audio_io_error.i2c.dmarx_state = 1;
	}

	if (HAL_OK != HAL_DMA_Init(&hdma_i2c_rx)) {
		audio_io_error.i2c.dmarx = 1;
	}

	__HAL_LINKDMA(hi2c, hdmarx, hdma_i2c_rx);
}


/**< ****************************************************************************************************************************** */
/**< Audio I/O deinitialization 																								    */
/**< ****************************************************************************************************************************** */
audio_status_t audio_io_deinit(void)
{
	HAL_StatusTypeDef status;
	audio_status_t retc;

	audio_io_reset_pin_deinit();
	
	status = I2Cx_DeInit();
	
	DMA_DeInit();
	
	retc = (HAL_OK != status) ? (AUDIO_IO_DEINIT_ERROR) : (AUDIO_IO_OK);

	return retc;
}

static void audio_io_reset_pin_deinit(void)
{
	HAL_GPIO_DeInit(AUDIO_IO_RESET_PORT, AUDIO_IO_RESET_PIN);
}

static HAL_StatusTypeDef I2Cx_DeInit(void)
{
	HAL_StatusTypeDef status;
	
	HAL_I2C_UnRegisterCallback(&hi2c, HAL_I2C_MEM_TX_COMPLETE_CB_ID);
	HAL_I2C_UnRegisterCallback(&hi2c, HAL_I2C_MEM_RX_COMPLETE_CB_ID);
	HAL_I2C_UnRegisterCallback(&hi2c, HAL_I2C_ERROR_CB_ID);

	if (HAL_OK != HAL_I2C_DeInit(&hi2c)) {
		audio_io_error.i2c.deinit = 1;
	}
	
	HAL_I2C_UnRegisterCallback(&hi2c, HAL_I2C_MSPINIT_CB_ID);
	HAL_I2C_UnRegisterCallback(&hi2c, HAL_I2C_MSPDEINIT_CB_ID);
	
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


/**< ****************************************************************************************************************************** */
/**< Audio I/O Callbacks         																								    */
/**< ****************************************************************************************************************************** */
void I2Cx_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	audio_io_i2c_tx_cplt = true;
}

void I2Cx_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	audio_io_i2c_rx_cplt = true;
}

void I2Cx_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	I2Cx_ErrorHandler();
}

static void I2Cx_ErrorHandler(void)
{
	I2Cx_DeInit();
	//I2Cx_Init();
	//error_callback;
}


/**< ****************************************************************************************************************************** */
/**< Audio I/O Read/Write 																								            */
/**< ****************************************************************************************************************************** */
audio_status_t audio_io_read(uint8_t register_address, uint8_t *data, uint8_t size, bool blocking)
{
	HAL_StatusTypeDef status;
	audio_status_t retc;
	uint16_t address      = (uint16_t)register_address;
	uint16_t address_size = sizeof(uint8_t);
	uint16_t data_size    = (uint16_t)size;
	uint32_t timeout      = 10;

	if (true == blocking) {
		status = HAL_I2C_Mem_Read(&hi2c, AUDIO_IO_DEVICE_ADDRESS, address, address_size, data, data_size, timeout);
	} else {
		audio_io_i2c_rx_cplt = false;
		status = HAL_I2C_Mem_Read_DMA(&hi2c, AUDIO_IO_DEVICE_ADDRESS, address, address_size, data, data_size);
	}

	retc = (HAL_OK != status) ? (AUDIO_IO_READ_ERROR) : (AUDIO_IO_OK);

	return retc;
}

audio_status_t audio_io_write(uint8_t register_address, uint8_t *data, uint8_t size, bool blocking)
{
	HAL_StatusTypeDef status;
	audio_status_t retc;
	uint16_t address      = (uint16_t)register_address;
	uint16_t address_size = sizeof(uint8_t);
	uint16_t data_size    = (uint16_t)size;
	uint32_t timeout      = 10;

	if (true == blocking) {
		status = HAL_I2C_Mem_Write(&hi2c, AUDIO_IO_DEVICE_ADDRESS, address, address_size, data, data_size, timeout);
	} else {
		audio_io_i2c_tx_cplt = false;
		status = HAL_I2C_Mem_Write_DMA(&hi2c, AUDIO_IO_DEVICE_ADDRESS, address, address_size, data, data_size);
	}

	retc = (HAL_OK != status) ? (AUDIO_IO_WRITE_ERROR) : (AUDIO_IO_OK);

	return retc;
}


/**< ****************************************************************************************************************************** */
/**< Audio Output Stream initialization 																						    */
/**< ****************************************************************************************************************************** */
audio_status_t audio_out_init(audio_out_ll_hw_params_t *haout)
{
	HAL_StatusTypeDef status;
	audio_status_t retc;

	status = I2Sx_Init(haout);
	retc = (HAL_OK != status) ? (AUDIO_OUT_INIT_ERROR) : (AUDIO_IO_OK);

	return retc;
}

uint32_t find(const uint32_t *array, const uint32_t size, const uint32_t value)
{
	uint32_t index = 0xFFFFFFFF;

	for (uint32_t i = 0; i < size; i++) {
		if (array[i] == value) {
			index = i;
		}
	}

	return index;
}

static HAL_StatusTypeDef I2Sx_Init(audio_out_ll_hw_params_t *haout)
{
	HAL_StatusTypeDef status;
	audio_io_error.i2s.w = 0;

	audio_out_ll.hw_params.standard        = haout->standard;
	audio_out_ll.hw_params.data_format     = haout->data_format;
	audio_out_ll.hw_params.audio_frequency = haout->audio_frequency;

	hi2s.Init.Standard       = audio_out_ll.hw_params.standard;
	hi2s.Init.DataFormat     = audio_out_ll.hw_params.data_format;
	hi2s.Init.AudioFreq      = audio_out_ll.hw_params.audio_frequency;
	hi2s.Instance            = I2Sx;
	hi2s.Init.Mode           = I2S_MODE_MASTER_TX;
	hi2s.Init.MCLKOutput     = I2S_MCLKOUTPUT_ENABLE;
	hi2s.Init.CPOL           = I2S_CPOL_LOW;
	hi2s.Init.ClockSource    = I2S_CLOCK_PLL;
	hi2s.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;

	if (HAL_I2S_STATE_RESET != HAL_I2S_GetState(&hi2s)) {
		audio_io_error.i2s.state = 1;
	}

	HAL_I2S_RegisterCallback(&hi2s, HAL_I2S_MSPINIT_CB_ID, I2Sx_MspInit);
	HAL_I2S_RegisterCallback(&hi2s, HAL_I2S_MSPDEINIT_CB_ID, I2Sx_MspDeInit);

	if (HAL_OK != HAL_I2S_Init(&hi2s)) {
		audio_io_error.i2s.init = 1;
	}

	HAL_I2S_RegisterCallback(&hi2s, HAL_I2S_TX_HALF_COMPLETE_CB_ID, I2Sx_TxHalfCpltCallback);
	HAL_I2S_RegisterCallback(&hi2s, HAL_I2S_TX_COMPLETE_CB_ID, I2Sx_TxCpltCallback);
	HAL_I2S_RegisterCallback(&hi2s, HAL_I2S_ERROR_CB_ID, I2Sx_ErrorCallback);

	status = (0 != audio_io_error.i2s.w) ? (HAL_ERROR) : (HAL_OK);

	return status;
}

void I2Sx_MspInit(I2S_HandleTypeDef *hi2s)
{
	RCC_PLLI2SInitTypeDef RCC_PLLI2SInit;
	GPIO_InitTypeDef GPIO_InitStruct;
	HAL_StatusTypeDef status;

	uint32_t index = find(I2Sx_Fs, sizeof(I2Sx_Fs)/sizeof(uint32_t), audio_out_ll.hw_params.audio_frequency);

	if (0xFFFFFFFF == index) {
		audio_io_error.i2s.invalid_fs = 1;
		audio_out_ll.hw_params.audio_frequency = I2Sx_Fs[0];
		index = 0;
	}

	RCC_PLLI2SInit.PLLI2SN = PLLI2Sx_N[index]; 
	RCC_PLLI2SInit.PLLI2SR = PLLI2Sx_R[index];
	
	if (HAL_OK != HAL_RCCEx_EnablePLLI2S(&RCC_PLLI2SInit)) {
		audio_io_error.i2s.plli2s = 1;
	}

	__I2Sx_MCLK_GPIO_CLK_ENABLE();
	__I2Sx_SCLK_GPIO_CLK_ENABLE();
	__I2Sx_SDIN_GPIO_CLK_ENABLE();
	__I2Sx_LRCK_GPIO_CLK_ENABLE();

	GPIO_InitStruct.Pin       = I2Sx_MCLK_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = I2Sx_GPIO_AF;
	HAL_GPIO_Init(I2Sx_MCLK_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin       = I2Sx_SCLK_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = I2Sx_GPIO_AF;
	HAL_GPIO_Init(I2Sx_SCLK_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin       = I2Sx_SDIN_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = I2Sx_GPIO_AF;
	HAL_GPIO_Init(I2Sx_SDIN_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin       = I2Sx_LRCK_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = I2Sx_GPIO_AF;
	HAL_GPIO_Init(I2Sx_LRCK_GPIO_PORT, &GPIO_InitStruct);

	__I2Sx_CLK_ENABLE();
	__I2Sx_FORCE_RESET();
	__I2Sx_RELEASE_RESET();

    hdma_i2s_tx.Instance                 = I2SxTX_DMA_STREAM;
	hdma_i2s_tx.Init.Channel             = I2SxTX_DMA_CHANNEL;
	hdma_i2s_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	hdma_i2s_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_i2s_tx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_i2s_tx.Init.PeriphDataAlignment = (I2S_DATAFORMAT_16B == audio_out_ll.hw_params.data_format) ? (DMA_PDATAALIGN_HALFWORD) : (DMA_PDATAALIGN_WORD);
	hdma_i2s_tx.Init.MemDataAlignment    = (I2S_DATAFORMAT_16B == audio_out_ll.hw_params.data_format) ? (DMA_MDATAALIGN_HALFWORD) : (DMA_MDATAALIGN_WORD);
	hdma_i2s_tx.Init.Mode                = DMA_NORMAL; 
	hdma_i2s_tx.Init.Priority            = DMA_PRIORITY_HIGH;
	hdma_i2s_tx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
	hdma_i2s_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	hdma_i2s_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
	hdma_i2s_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;

	if (HAL_DMA_STATE_RESET != HAL_DMA_GetState(&hdma_i2s_tx)) {
		audio_io_error.i2s.dmatx_state = 1;
	}

	if (HAL_OK != HAL_DMA_Init(&hdma_i2s_tx)) {
		audio_io_error.i2s.dmatx = 1;
	}

	__HAL_LINKDMA(hi2s, hdmatx, hdma_i2s_tx);
}


/**< ****************************************************************************************************************************** */
/**< Audio Output Stream deinitialization 																						    */
/**< ****************************************************************************************************************************** */
audio_status_t audio_out_deinit(void)
{
	HAL_StatusTypeDef status;
	audio_status_t retc;

	status = I2Sx_DeInit();
	retc = (HAL_OK != status) ? (AUDIO_OUT_DEINIT_ERROR) : (AUDIO_IO_OK);

	return retc;
}

static HAL_StatusTypeDef I2Sx_DeInit(void)
{
	HAL_StatusTypeDef status;
	
	HAL_I2S_UnRegisterCallback(&hi2s, HAL_I2S_TX_COMPLETE_CB_ID);
	HAL_I2S_UnRegisterCallback(&hi2s, HAL_I2S_TX_HALF_COMPLETE_CB_ID);
	HAL_I2S_UnRegisterCallback(&hi2s, HAL_I2S_ERROR_CB_ID);

	if (HAL_OK != HAL_I2S_DeInit(&hi2s)) {
		audio_io_error.i2s.deinit = 1;
	}
	
	HAL_I2S_UnRegisterCallback(&hi2s, HAL_I2S_MSPINIT_CB_ID);
	HAL_I2S_UnRegisterCallback(&hi2s, HAL_I2S_MSPDEINIT_CB_ID);
	
	status = (0 != audio_io_error.i2s.w) ? (HAL_ERROR) : (HAL_OK);

	return status;
}

void I2Sx_MspDeInit(I2S_HandleTypeDef *hi2s)
{
	if (HAL_OK != HAL_RCCEx_DisablePLLI2S()) {
		audio_io_error.i2s.plli2s = 1;
	}

	__I2Sx_CLK_ENABLE();

	HAL_GPIO_DeInit(I2Sx_MCLK_GPIO_PORT, I2Sx_MCLK_PIN);
	HAL_GPIO_DeInit(I2Sx_SCLK_GPIO_PORT, I2Sx_SCLK_PIN);
	HAL_GPIO_DeInit(I2Sx_SDIN_GPIO_PORT, I2Sx_SDIN_PIN);
	HAL_GPIO_DeInit(I2Sx_LRCK_GPIO_PORT, I2Sx_LRCK_PIN);

	if (HAL_OK != HAL_DMA_DeInit(&hdma_i2s_tx)) {
		audio_io_error.i2s.dmatx = 1;
	}
}


/**< ****************************************************************************************************************************** */
/**< Audio Output Stream Callbacks																								    */
/**< ****************************************************************************************************************************** */
void I2Sx_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	audio_out_ll.cb_params.write_callback(audio_out_ll.cb_params.m0_buffer, AUDIO_OUT_TX_HALF_COMPLETE_CB_ID);
}

void I2Sx_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	audio_out_ll.cb_params.write_callback(audio_out_ll.cb_params.m1_buffer, AUDIO_OUT_TX_COMPLETE_CB_ID);
}

void I2Sx_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
	I2Sx_ErrorHandler();
}

static void I2Sx_ErrorHandler(void)
{
	HAL_I2S_DMAStop(&hi2s);
	I2Sx_DeInit();
	//I2Sx_Init();
	//error_callback;
}

/**< ****************************************************************************************************************************** */
/**< Audio Output Stream Write																									    */
/**< 	When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S											*/
/**<    configuration phase, the Size parameter means the number of 16-bit data length												*/
/**<    in the transaction and when a 24-bit data frame or a 32-bit data frame is selected											*/
/**<    the Size parameter means the number of 16-bit data length.																	*/
/**< ****************************************************************************************************************************** */
audio_status_t audio_out_write(uint16_t *data, const uint16_t size)
{
	HAL_StatusTypeDef status = HAL_I2S_Transmit_DMA(&hi2s, data, size);
	audio_status_t retc = (HAL_OK != status) ? (AUDIO_OUT_WRITE_ERROR) : (AUDIO_IO_OK);	

	return retc;
}

audio_status_t audio_out_pause(void)
{
	HAL_StatusTypeDef status = HAL_I2S_DMAPause(&hi2s);
	audio_status_t retc = (HAL_OK != status) ? (AUDIO_OUT_PAUSE_ERROR) : (AUDIO_IO_OK);

	return retc;
}

audio_status_t audio_out_resume(void)
{
	HAL_StatusTypeDef status = HAL_I2S_DMAResume(&hi2s);
	audio_status_t retc = (HAL_OK != status) ? (AUDIO_OUT_RESUME_ERROR) : (AUDIO_IO_OK);

	return retc;
}

audio_status_t audio_out_stop(void)
{
	HAL_StatusTypeDef status = HAL_I2S_DMAStop(&hi2s);
	audio_status_t retc = (HAL_OK != status) ? (AUDIO_OUT_STOP_ERROR) : (AUDIO_IO_OK);

	return retc;
}

bool audio_out_ll_set_cb_params(audio_out_ll_cb_params_t *cb_params)
{
	bool retc;

	if ((NULL != cb_params)                 && \
	    (NULL != cb_params->write_callback) && \
		(NULL != cb_params->m0_buffer)      && \
		(NULL != cb_params->m1_buffer)) 	{

		audio_out_ll.cb_params.write_callback = cb_params->write_callback;
		audio_out_ll.cb_params.m0_buffer      = cb_params->m0_buffer;
		audio_out_ll.cb_params.m1_buffer      = cb_params->m1_buffer;
		retc = true;	
		
	} else {
		audio_out_ll.cb_params.write_callback = NULL;
		audio_out_ll.cb_params.m0_buffer      = NULL;
		audio_out_ll.cb_params.m1_buffer      = NULL;
		audio_io_error.i2s.bad_params = 1;
		retc = false;
	}

	return retc;
}

