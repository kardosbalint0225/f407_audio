#ifndef AUDIO_IO_H
#define AUDIO_IO_H

#include <stdint.h>
#include <stdbool.h>

/**< ****************************************************************************************************************************** */
/**< I2C peripheral configuration defines (control interface of the audio codec)													*/
/**< ****************************************************************************************************************************** */
#ifdef TEST
	#define I2Cx                            		I2C1
	#define __I2Cx_CLK_ENABLE()               		
	#define __I2Cx_CLK_DISABLE()               		
	#define __I2Cx_SCL_GPIO_CLK_ENABLE()  			
	#define __I2Cx_SDA_GPIO_CLK_ENABLE()  			
	#define __I2Cx_SCL_GPIO_CLK_DISABLE()  			
	#define __I2Cx_SDA_GPIO_CLK_DISABLE()  			
	#define I2Cx_SCL_SDA_AF                 		GPIO_AF4_I2C1
	#define I2Cx_SCL_GPIO_PORT          			GPIOB
	#define I2Cx_SDA_GPIO_PORT          			GPIOB
	#define I2Cx_SCL_PIN                    		GPIO_PIN_6
	#define I2Cx_SDA_PIN                    		GPIO_PIN_9
	#define I2Cx_CLK_SPEED                          100000	/* I2C clock speed configuration (in Hz) */

	#define __I2Cx_FORCE_RESET()              		
	#define __I2Cx_RELEASE_RESET()            		

//	#define I2Cx_EV_IRQn                    		I2C1_EV_IRQn
//	#define I2Cx_ER_IRQn                    		I2C1_ER_IRQn
	
	#define AUDIO_IO_RESET_PORT						GPIOD
	#define AUDIO_IO_RESET_PIN						GPIO_PIN_4
	#define __AUDIO_IO_RESET_GPIO_CLK_ENABLE()  	
	#define __AUDIO_IO_RESET_GPIO_CLK_DISABLE()  	
	
	#define I2CxTX_DMA_STREAM						DMA1_Stream7	//DMA1_Stream7
	#define I2CxTX_DMA_CHANNEL						DMA_CHANNEL_1
	#define __I2CxTX_DMA_CLK_ENABLE()
	#define __I2CxTX_DMA_CLK_DISABLE()
	#define I2CxTX_DMA_Stream_IRQn					DMA1_Stream7_IRQn

	#define I2CxRX_DMA_STREAM						DMA1_Stream0	//DMA1_Stream5
	#define I2CxRX_DMA_CHANNEL						DMA_CHANNEL_1
	#define __I2CxRX_DMA_CLK_ENABLE()
	#define __I2CxRX_DMA_CLK_DISABLE()
	#define I2CxRX_DMA_Stream_IRQn					DMA1_Stream0_IRQn

	#define I2Sx									SPI2
	
#else
	#define I2Cx                            		I2C1
	#define __I2Cx_CLK_ENABLE()               		__HAL_RCC_I2C1_CLK_ENABLE()
	#define __I2Cx_CLK_DISABLE()               		__HAL_RCC_I2C1_CLK_DISABLE()
	#define __I2Cx_SCL_GPIO_CLK_ENABLE()  			__HAL_RCC_GPIOB_CLK_ENABLE()
	#define __I2Cx_SDA_GPIO_CLK_ENABLE()  			__HAL_RCC_GPIOB_CLK_ENABLE()
	#define __I2Cx_SCL_GPIO_CLK_DISABLE()  			__HAL_RCC_GPIOB_CLK_DISABLE()
	#define __I2Cx_SDA_GPIO_CLK_DISABLE()  			__HAL_RCC_GPIOB_CLK_DISABLE()
	#define I2Cx_SCL_SDA_AF                 		GPIO_AF4_I2C1
	#define I2Cx_SCL_GPIO_PORT          			GPIOB
	#define I2Cx_SDA_GPIO_PORT          			GPIOB
	#define I2Cx_SCL_PIN                    		GPIO_PIN_6
	#define I2Cx_SDA_PIN                    		GPIO_PIN_9
	#define I2Cx_CLK_SPEED                          100000	/* I2C clock speed configuration (in Hz) */

	#define __I2Cx_FORCE_RESET()              		__HAL_RCC_I2C1_FORCE_RESET()
	#define __I2Cx_RELEASE_RESET()            		__HAL_RCC_I2C1_RELEASE_RESET()

//	#define I2Cx_EV_IRQn                    		I2C1_EV_IRQn
//	#define I2Cx_ER_IRQn                    		I2C1_ER_IRQn
	
	#define AUDIO_IO_RESET_PORT						GPIOD
	#define AUDIO_IO_RESET_PIN						GPIO_PIN_4
	#define __AUDIO_IO_RESET_GPIO_CLK_ENABLE()  	__HAL_RCC_GPIOD_CLK_ENABLE()
	#define __AUDIO_IO_RESET_GPIO_CLK_DISABLE()  	__HAL_RCC_GPIOD_CLK_DISABLE()
	
	#define I2CxTX_DMA_STREAM						DMA1_Stream7	//DMA1_Stream6
	#define I2CxTX_DMA_CHANNEL						DMA_CHANNEL_1
	#define __I2CxTX_DMA_CLK_ENABLE()				__HAL_RCC_DMA1_CLK_ENABLE()
	#define __I2CxTX_DMA_CLK_DISABLE()				__HAL_RCC_DMA1_CLK_DISABLE()
	#define I2CxTX_DMA_Stream_IRQn					DMA1_Stream7_IRQn

	#define I2CxRX_DMA_STREAM						DMA1_Stream0	//DMA1_Stream5
	#define I2CxRX_DMA_CHANNEL						DMA_CHANNEL_1
	#define __I2CxRX_DMA_CLK_ENABLE()				__HAL_RCC_DMA1_CLK_ENABLE()
	#define __I2CxRX_DMA_CLK_DISABLE()				__HAL_RCC_DMA1_CLK_DISABLE()
	#define I2CxRX_DMA_Stream_IRQn					DMA1_Stream0_IRQn

	#define I2Sx									SPI2
	
#endif
		// + i2s instance
		// + gpio reset pin
typedef enum {
	AUDIO_IO_OK = 0,
	AUDIO_IO_I2C_INIT_ERROR,
	AUDIO_IO_I2C_DEINIT_ERROR,
	AUDIO_IO_DMA_INIT_ERROR,
	AUDIO_IO_I2C_READ_ERROR,
	AUDIO_IO_I2C_WRITE_ERROR,
} audio_status_t;

typedef enum {
	AUDIO_IO_DEVICE_ADDRESS = 0x94U,	// (1 0 0 1 0 1 AD0=0) << 1 
} audio_io_device_address_t;

typedef union {
	struct {
		uint32_t state  : 1;
		uint32_t init   : 1;
		uint32_t deinit : 1;
		uint32_t ready  : 1;
		uint32_t dmatx  : 1;
		uint32_t dmarx  : 1;
		uint32_t        : 25;
	};
	uint32_t w;
} audio_io_i2c_err_t;

typedef struct {

} audio_io_dma_err_t;

typedef struct {

} audio_io_i2s_err_t;

typedef struct {
	audio_io_i2c_err_t i2c;
} audio_io_err_t;

/*
 * stm32f4xx_hal_i2s.h
 *
 * standard:        - I2S_STANDARD_PHILIPS
 *                  - I2S_STANDARD_MSB
 *                  - I2S_STANDARD_LSB
 *                  - I2S_STANDARD_PCM_SHORT
 *                  - I2S_STANDARD_PCM_LONG
 * 
 * data_format:     - I2S_DATAFORMAT_16B
 *                  - I2S_DATAFORMAT_16B_EXTENDED
 *                  - I2S_DATAFORMAT_24B
 *                  - I2S_DATAFORMAT_32B
 * 
 * audio_frequency: - I2S_AUDIOFREQ_192K
 *                  - I2S_AUDIOFREQ_96K
 *                  - I2S_AUDIOFREQ_48K
 *                  - I2S_AUDIOFREQ_44K
 *                  - I2S_AUDIOFREQ_32K
 *                  - I2S_AUDIOFREQ_22K
 *                  - I2S_AUDIOFREQ_16K
 *                  - I2S_AUDIOFREQ_11K
 *                  - I2S_AUDIOFREQ_8K
 *                  - I2S_AUDIOFREQ_DEFAULT
 */
typedef struct {
	uint32_t standard;
	uint32_t data_format;
	uint32_t audio_frequency;
} audio_stream_t;

/**< ****************************************************************************************************************************** */
/**< Audio Control Port I/O functions																								*/
/**< ****************************************************************************************************************************** */
audio_status_t audio_io_init(void);
audio_status_t audio_io_deinit(void);
//void audio_io_error();
//void audio_io_reset(void);
audio_status_t audio_io_read(uint8_t register_address, uint8_t *data, uint8_t size, bool blocking);
audio_status_t audio_io_write(uint8_t register_address, uint8_t *data, uint8_t size, bool blocking);

#ifdef TEST
void audio_io_error_reset(void);
#endif

/**< ****************************************************************************************************************************** */
/**< Audio Stream functions            																								*/
/**< ****************************************************************************************************************************** */
audio_status_t audio_stream_interface_init(audio_stream_t *audio_stream);
audio_status_t audio_stream_interface_deinit(void);
//void 	audio_stream_write(const uint8_t *buffer, const uint32_t size);


#endif // AUDIO_IO_H
