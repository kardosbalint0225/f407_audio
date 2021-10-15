#ifndef AUDIO_IO_H
#define AUDIO_IO_H

#include <stdint.h>
#include <stdbool.h>

/**< ****************************************************************************************************************************** */
/**< I2C peripheral configuration defines (control interface of the audio codec)													*/
/**< ****************************************************************************************************************************** */
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
#define I2Cx_CLK_SPEED                          100000U	/* I2C clock speed configuration (in Hz) */

#define __I2Cx_FORCE_RESET()              		__HAL_RCC_I2C1_FORCE_RESET()
#define __I2Cx_RELEASE_RESET()            		__HAL_RCC_I2C1_RELEASE_RESET()

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

/**< ****************************************************************************************************************************** */
/**< GPIO configuration defines (reset pin of the audio codec)																		*/
/**< ****************************************************************************************************************************** */
#define AUDIO_IO_RESET_PORT						GPIOD
#define AUDIO_IO_RESET_PIN						GPIO_PIN_4
#define __AUDIO_IO_RESET_GPIO_CLK_ENABLE()  	__HAL_RCC_GPIOD_CLK_ENABLE()
#define __AUDIO_IO_RESET_GPIO_CLK_DISABLE()  	__HAL_RCC_GPIOD_CLK_DISABLE()

/**< ****************************************************************************************************************************** */
/**< I2S peripheral configuration defines (stream interface of the audio codec)														*/
/**< ****************************************************************************************************************************** */
#define I2Sx									SPI3
#define __I2Sx_CLK_ENABLE()               		__HAL_RCC_SPI3_CLK_ENABLE()		
#define __I2Sx_CLK_DISABLE()					__HAL_RCC_SPI3_CLK_DISABLE()
#define __I2Sx_MCLK_GPIO_CLK_ENABLE()			__HAL_RCC_GPIOC_CLK_ENABLE()	
#define __I2Sx_SCLK_GPIO_CLK_ENABLE()			__HAL_RCC_GPIOC_CLK_ENABLE()
#define __I2Sx_SDIN_GPIO_CLK_ENABLE()			__HAL_RCC_GPIOC_CLK_ENABLE()
#define __I2Sx_LRCK_GPIO_CLK_ENABLE()			__HAL_RCC_GPIOA_CLK_ENABLE()
#define __I2Sx_MCLK_GPIO_CLK_DISABLE()			__HAL_RCC_GPIOC_CLK_DISABLE()
#define __I2Sx_SCLK_GPIO_CLK_DISABLE()			__HAL_RCC_GPIOC_CLK_DISABLE()
#define __I2Sx_SDIN_GPIO_CLK_DISABLE()			__HAL_RCC_GPIOC_CLK_DISABLE()
#define __I2Sx_LRCK_GPIO_CLK_DISABLE()			__HAL_RCC_GPIOA_CLK_DISABLE()

#define I2Sx_GPIO_AF                 			GPIO_AF6_SPI3
#define I2Sx_MCLK_GPIO_PORT          			GPIOC
#define I2Sx_SCLK_GPIO_PORT          			GPIOC
#define I2Sx_SDIN_GPIO_PORT          			GPIOC
#define I2Sx_LRCK_GPIO_PORT          			GPIOA
#define I2Sx_MCLK_PIN                    		GPIO_PIN_7
#define I2Sx_SCLK_PIN                    		GPIO_PIN_10
#define I2Sx_SDIN_PIN                    		GPIO_PIN_12
#define I2Sx_LRCK_PIN                    		GPIO_PIN_4

#define __I2Sx_FORCE_RESET()              		__HAL_RCC_SPI3_FORCE_RESET()
#define __I2Sx_RELEASE_RESET()            		__HAL_RCC_SPI3_RELEASE_RESET()

#define I2SxTX_DMA_STREAM						DMA1_Stream5	
#define I2SxTX_DMA_CHANNEL						DMA_CHANNEL_0
#define __I2SxTX_DMA_CLK_ENABLE()				__HAL_RCC_DMA1_CLK_ENABLE()
#define __I2SxTX_DMA_CLK_DISABLE()				__HAL_RCC_DMA1_CLK_DISABLE()
#define I2SxTX_DMA_Stream_IRQn					DMA1_Stream5_IRQn

#define	AUDIO_OUT_STANDARD_PHILIPS          	(0x00000000U)
#define AUDIO_OUT_STANDARD_LEFT_JUSTIFIED   	(0x00000010U)
#define AUDIO_OUT_STANDARD_RIGHT_JUSTIFIED  	(0x00000020U)
#define AUDIO_OUT_STANDARD_DSP_MODE         	(0x00000030U)
#define	AUDIO_OUT_DATAFORMAT_16B            	(0x00000000U)
#define	AUDIO_OUT_DATAFORMAT_24B            	(0x00000003U)
#define	AUDIO_OUT_DATAFORMAT_32B            	(0x00000005U)

typedef enum {
	AUDIO_IO_OK = 0,
	AUDIO_IO_INIT_ERROR,
	AUDIO_IO_DEINIT_ERROR,
	AUDIO_IO_READ_ERROR,
	AUDIO_IO_WRITE_ERROR,
	AUDIO_OUT_INIT_ERROR,
	AUDIO_OUT_DEINIT_ERROR,
	AUDIO_OUT_WRITE_ERROR,
	AUDIO_OUT_PAUSE_ERROR,
	AUDIO_OUT_RESUME_ERROR,
	AUDIO_OUT_STOP_ERROR,
} audio_status_t;

typedef enum {
	AUDIO_IO_DEVICE_ADDRESS = 0x94U,	// (1 0 0 1 0 1 AD0=0) << 1 
} audio_io_device_address_t;

typedef union {
	struct {
		uint32_t state       : 1;
		uint32_t init        : 1;
		uint32_t deinit      : 1;
		uint32_t ready       : 1;
		uint32_t dmatx       : 1;
		uint32_t dmarx       : 1;
		uint32_t dmatx_state : 1;
		uint32_t dmarx_state : 1;
		uint32_t             : 24;
	};
	uint32_t w;
} audio_io_i2c_err_t;

typedef union {
	struct {
		uint32_t state       : 1;
		uint32_t init        : 1;
		uint32_t deinit      : 1;		
		uint32_t dmatx       : 1;	
		uint32_t dmatx_state : 1;
		uint32_t invalid_fs  : 1;
		uint32_t plli2s		 : 1;
		uint32_t bad_params  : 1;	
		uint32_t             : 24;
	};
	uint32_t w;
} audio_io_i2s_err_t;

typedef struct {
	audio_io_i2c_err_t i2c;
	audio_io_i2s_err_t i2s;
} audio_io_err_t;

typedef enum {
	AUDIO_OUT_TX_HALF_COMPLETE_CB_ID,
	AUDIO_OUT_TX_COMPLETE_CB_ID,
} audio_out_cb_id_t;

typedef void (*audio_out_write_callback_t)(uint16_t *address, const audio_out_cb_id_t callback_id);

/*
 * stm32f4xx_hal_i2s.h
 *
 * standard:        - I2S_STANDARD_PHILIPS
 *                  - I2S_STANDARD_MSB
 *                  - I2S_STANDARD_LSB
 *                  - I2S_STANDARD_PCM_SHORT
 *                  - I2S_STANDARD_PCM_LONG
 * 
 * data_format:     - I2S_DATAFORMAT_16B			(16 bit)
 *                  - I2S_DATAFORMAT_16B_EXTENDED	(32 bit)
 *                  - I2S_DATAFORMAT_24B			(32 bit)
 *                  - I2S_DATAFORMAT_32B			(32 bit)
 * 
 * audio_frequency: - I2S_AUDIOFREQ_192K			(192000U)
 *                  - I2S_AUDIOFREQ_96K				(96000U)
 *                  - I2S_AUDIOFREQ_48K				(48000U)
 *                  - I2S_AUDIOFREQ_44K				(44100U)
 *                  - I2S_AUDIOFREQ_32K				(32000U)
 *                  - I2S_AUDIOFREQ_22K				(22050U)
 *                  - I2S_AUDIOFREQ_16K				(16000U)
 *                  - I2S_AUDIOFREQ_11K				(11025U)
 *                  - I2S_AUDIOFREQ_8K				(8000U)
 *                  - I2S_AUDIOFREQ_DEFAULT			(2U)
 */

typedef struct {
	uint32_t standard;
	uint32_t data_format;
	uint32_t audio_frequency;
} audio_out_hw_params_t;

typedef struct {
	audio_out_write_callback_t write_callback;
	uint16_t *m0_buffer;
	uint16_t *m1_buffer;
} audio_out_cb_params_t;

typedef struct {
	audio_out_hw_params_t hw_params;
	audio_out_cb_params_t cb_params;
} audio_out_t;


#ifdef TEST
void audio_io_error_reset(void);
void audio_io_get_error(uint32_t *i2c, uint32_t *i2s);
void audio_out_set_params(audio_out_t *haout);
#endif

typedef struct {
	audio_status_t (*init)(void);
	audio_status_t (*deinit)(void);
	audio_status_t (*read)(uint8_t register_address, uint8_t *data, uint8_t size, bool blocking);
	audio_status_t (*write)(uint8_t register_address, uint8_t *data, uint8_t size, bool blocking);
} audio_io_if_t;

typedef struct {
	audio_status_t (*init)(audio_out_t *haout);
	audio_status_t (*deinit)(void);
	audio_status_t (*write)(uint16_t *data, const uint16_t size);
	audio_status_t (*pause)(void);
	audio_status_t (*resume)(void);
	audio_status_t (*stop)(void);
} audio_out_if_t;


/**< ****************************************************************************************************************************** */
/**< Audio Control Port I/O functions																								*/
/**< ****************************************************************************************************************************** */
audio_status_t audio_io_init(void);
audio_status_t audio_io_deinit(void);
audio_status_t audio_io_read(uint8_t register_address, uint8_t *data, uint8_t size, bool blocking);
audio_status_t audio_io_write(uint8_t register_address, uint8_t *data, uint8_t size, bool blocking);

/**< ****************************************************************************************************************************** */
/**< Audio Output Stream functions            																						*/
/**< ****************************************************************************************************************************** */
audio_status_t audio_out_init(audio_out_t *haout);
audio_status_t audio_out_deinit(void);
audio_status_t audio_out_write(uint16_t *data, const uint16_t size);
audio_status_t audio_out_pause(void);
audio_status_t audio_out_resume(void);
audio_status_t audio_out_stop(void);


#endif // AUDIO_IO_H


