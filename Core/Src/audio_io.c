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

static I2C_HandleTypeDef hi2c;
static DMA_HandleTypeDef hdma_i2c_tx;

static void DMA_Init(void);
static HAL_StatusTypeDef I2Cx_Init(void);
static HAL_StatusTypeDef I2Cx_DeInit(void);
static HAL_StatusTypeDef I2Cx_MspInit(void);
static void I2Cx_MspDeInit(void);
static void audio_io_reset(void);
static void audio_io_reset_pin_init(void);
static void audio_io_reset_pin_deinit(void);


audio_status_t audio_io_init(void)
{	
	audio_io_reset_pin_init();
	audio_io_reset();
	
	DMA_Init();
	
	if (HAL_OK != I2Cx_Init()) {
		return AUDIO_IO_I2C_INIT_ERROR;
	}
	
	return AUDIO_IO_OK;
}

static void audio_io_reset_pin_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__AUDIO_IO_RESET_GPIO_CLK_ENABLE();
	
	GPIO_InitStruct.Pin 	  = AUDIO_IO_RESET_PIN;
	GPIO_InitStruct.Mode 	  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull 	  = GPIO_NOPULL;
	GPIO_InitStruct.Speed 	  = GPIO_SPEED_FREQ_LOW;		
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
	__I2C_TX_DMAx_CLK_ENABLE();
	HAL_NVIC_SetPriority(DMAx_Streamy_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMAx_Streamy_IRQn);
}

static HAL_StatusTypeDef I2Cx_Init(void)
{	
	uint32_t trials  = 3;
	uint32_t timeout = 1;
	uint16_t address = AUDIO_IO_DEVICE_I2C_ADDRESS;

	hi2c.Instance 			  = I2Cx;
	hi2c.Init.ClockSpeed 	  = I2Cx_CLK_SPEED;
	hi2c.Init.DutyCycle 	  = I2C_DUTYCYCLE_2;
	hi2c.Init.OwnAddress1 	  = 0;							//I2cHandle.Init.OwnAddress1 = 0x33;
	hi2c.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c.Init.OwnAddress2     = 0;
	hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;	
	
	if (HAL_I2C_STATE_RESET != HAL_I2C_GetState(&hi2c)) {
		return HAL_ERROR;
	}
	
	if (HAL_OK != I2Cx_MspInit()) {
		return HAL_ERROR;
	}
	
	if (HAL_OK != HAL_I2C_Init(&hi2c)) {
		return HAL_ERROR;
	}
	
	if (HAL_OK != HAL_I2C_IsDeviceReady(&hi2c, address, trials, timeout)) {
		return HAL_ERROR;
	}
		
	return HAL_OK;
}

static HAL_StatusTypeDef I2Cx_MspInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__I2Cx_SCL_GPIO_CLK_ENABLE();
	__I2Cx_SDA_GPIO_CLK_ENABLE();
	
	GPIO_InitStruct.Pin 	  = I2Cx_SCL_PIN;
	GPIO_InitStruct.Mode 	  = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull 	  = GPIO_NOPULL;
	GPIO_InitStruct.Speed 	  = GPIO_SPEED_FREQ_LOW;		
	GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
	HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin 	  = I2Cx_SDA_PIN;
	GPIO_InitStruct.Mode 	  = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull 	  = GPIO_NOPULL;
	GPIO_InitStruct.Speed 	  = GPIO_SPEED_FREQ_LOW;	
	GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
	HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);

	__I2Cx_CLK_ENABLE();							/* I2Cx clock enable 						*/	
	__I2Cx_FORCE_RESET();							/* Force the I2C peripheral clock reset 	*/
	__I2Cx_RELEASE_RESET();							/* Release the I2C peripheral clock reset 	*/
	
	hdma_i2c_tx.Instance 	  			 = I2C_TX_DMAx_STREAMy;
	hdma_i2c_tx.Init.Channel  			 = I2C_TX_DMA_CHANNELx;
	hdma_i2c_tx.Init.Direction 			 = DMA_MEMORY_TO_PERIPH;
	hdma_i2c_tx.Init.PeriphInc 			 = DMA_PINC_DISABLE;
	hdma_i2c_tx.Init.MemInc    			 = DMA_MINC_ENABLE;
	hdma_i2c_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_i2c_tx.Init.MemDataAlignment 	 = DMA_MDATAALIGN_BYTE;
	hdma_i2c_tx.Init.Mode 				 = DMA_NORMAL;
	hdma_i2c_tx.Init.Priority 			 = DMA_PRIORITY_LOW;
	hdma_i2c_tx.Init.FIFOMode 			 = DMA_FIFOMODE_DISABLE;
	hdma_i2c_tx.Init.FIFOThreshold 		 = DMA_FIFO_THRESHOLD_1QUARTERFULL;
	hdma_i2c_tx.Init.MemBurst 			 = DMA_MBURST_SINGLE;
	hdma_i2c_tx.Init.PeriphBurst 		 = DMA_PBURST_SINGLE;
	
	if (HAL_OK != HAL_DMA_Init(&hdma_i2c_tx)) {
		return HAL_ERROR;
	}
	
	__HAL_LINKDMA(&hi2c, hdmatx, hdma_i2c_tx);
	HAL_NVIC_SetPriority(I2Cx_EV_IRQn, 0, 0);		/* I2Cx Event interrupt Init */
	HAL_NVIC_EnableIRQ(I2Cx_EV_IRQn);
	HAL_NVIC_SetPriority(I2Cx_ER_IRQn, 0, 0);		/* I2Cx Error interrupt Init */
	HAL_NVIC_EnableIRQ(I2Cx_ER_IRQn);
		
	return HAL_OK;
}

audio_status_t audio_io_deinit(void)
{
	audio_io_reset_pin_deinit();
	
	audio_status_t status;
	
	if (HAL_OK != I2Cx_DeInit()) {
		status = AUDIO_IO_I2C_DEINIT_ERROR;
	} else {
		if (HAL_I2C_STATE_RESET != HAL_I2C_GetState(&hi2c)) {
			status = AUDIO_IO_I2C_DEINIT_ERROR;
		} else {
			status = AUDIO_IO_OK;
		}
	}
	
	return status;
}

static HAL_StatusTypeDef I2Cx_DeInit(void)
{
	I2Cx_MspDeInit();
	return HAL_I2C_DeInit(&hi2c);
}

static void I2Cx_MspDeInit(void)
{
	HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
	HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);

	__I2Cx_SCL_GPIO_CLK_DISABLE();
	__I2Cx_SDA_GPIO_CLK_DISABLE();

	HAL_NVIC_DisableIRQ(I2Cx_EV_IRQn);		/* I2Cx Event interrupt Deinit */
	HAL_NVIC_DisableIRQ(I2Cx_ER_IRQn);		/* I2Cx Error interrupt Deinit */
}





