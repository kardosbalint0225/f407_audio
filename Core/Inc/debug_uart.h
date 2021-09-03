#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdint.h>

/**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */

#ifdef TEST
	#define USARTx                            		USART2
	#define __UARTx_CLK_ENABLE()               		//__HAL_RCC_USART2_CLK_ENABLE()
	#define __UARTx_CLK_DISABLE()              		//__HAL_RCC_USART2_CLK_DISABLE()
	#define __UARTx_TX_GPIO_CLK_ENABLE()  			//__HAL_RCC_GPIOA_CLK_ENABLE()
	#define __UARTx_RX_GPIO_CLK_ENABLE()  			//__HAL_RCC_GPIOA_CLK_ENABLE()
	#define __UARTx_TX_GPIO_CLK_DISABLE()  			//__HAL_RCC_GPIOA_CLK_DISABLE()
	#define __UARTx_RX_GPIO_CLK_DISABLE()  			//__HAL_RCC_GPIOA_CLK_DISABLE()
	#define UARTx_TX_RX_AF                  		GPIO_AF7_USART2
	#define UARTx_TX_GPIO_PORT          			GPIOA
	#define UARTx_RX_GPIO_PORT          			GPIOA
	#define UARTx_TX_PIN                    		GPIO_PIN_2
	#define UARTx_RX_PIN                    		GPIO_PIN_3
	#define UARTx_BAUDRATE                          115200	

	#define __UARTx_FORCE_RESET()              		//__HAL_RCC_USART2_FORCE_RESET()
	#define __UARTx_RELEASE_RESET()            		//__HAL_RCC_USART2_RELEASE_RESET()
	
	#define UARTxTX_DMA_STREAM						DMA1_Stream6
	#define UARTxTX_DMA_CHANNEL						DMA_CHANNEL_4
	#define __UARTxTX_DMA_CLK_ENABLE()				//__HAL_RCC_DMA1_CLK_ENABLE()		
	#define __UARTxTX_DMA_CLK_DISABLE()				//__HAL_RCC_DMA1_CLK_DISABLE()
	#define UARTxTX_DMA_Stream_IRQn					DMA1_Stream6_IRQn
	
	#define USARTx_IRQn 							USART2_IRQn
#else
	#define USARTx                            		USART2
	#define __UARTx_CLK_ENABLE()               		__HAL_RCC_USART2_CLK_ENABLE()
	#define __UARTx_CLK_DISABLE()              		__HAL_RCC_USART2_CLK_DISABLE()
	#define __UARTx_TX_GPIO_CLK_ENABLE()  			__HAL_RCC_GPIOA_CLK_ENABLE()
	#define __UARTx_RX_GPIO_CLK_ENABLE()  			__HAL_RCC_GPIOA_CLK_ENABLE()
	#define __UARTx_TX_GPIO_CLK_DISABLE()  			__HAL_RCC_GPIOA_CLK_DISABLE()
	#define __UARTx_RX_GPIO_CLK_DISABLE()  			__HAL_RCC_GPIOA_CLK_DISABLE()
	#define UARTx_TX_RX_AF                  		GPIO_AF7_USART2
	#define UARTx_TX_GPIO_PORT          			GPIOA
	#define UARTx_RX_GPIO_PORT          			GPIOA
	#define UARTx_TX_PIN                    		GPIO_PIN_2
	#define UARTx_RX_PIN                    		GPIO_PIN_3
	#define UARTx_BAUDRATE                          115200	

	#define __UARTx_FORCE_RESET()              		__HAL_RCC_USART2_FORCE_RESET()
	#define __UARTx_RELEASE_RESET()            		__HAL_RCC_USART2_RELEASE_RESET()
	
	#define UARTxTX_DMA_STREAM						DMA1_Stream6
	#define UARTxTX_DMA_CHANNEL						DMA_CHANNEL_4
	#define __UARTxTX_DMA_CLK_ENABLE()				__HAL_RCC_DMA1_CLK_ENABLE()		
	#define __UARTxTX_DMA_CLK_DISABLE()				__HAL_RCC_DMA1_CLK_DISABLE()
	#define UARTxTX_DMA_Stream_IRQn					DMA1_Stream6_IRQn	

	#define USARTx_IRQn 							USART2_IRQn
#endif

typedef enum {
	DEBUG_UART_OK           = 0U,
	DEBUG_UART_INIT_ERROR   = 1U,
	DEBUG_UART_DEINIT_ERROR = 2U,
} debug_uart_status_t;

typedef union {
	struct {
		uint32_t state        : 1;
		uint32_t init         : 1;
		uint32_t deinit       : 1;
		uint32_t ready        : 1;
		uint32_t dmatx        : 1;
		uint32_t dmarx        : 1;
		uint32_t fifo_overrun : 1;
		uint32_t              : 24;
	};
	uint32_t w;
} debug_uart_err_t;   

#ifdef TEST
void debug_uart_error_reset(void);
#endif

debug_uart_status_t debug_uart_init(void);
debug_uart_status_t debug_uart_deinit(void);


#endif // DEBUG_UART_H
