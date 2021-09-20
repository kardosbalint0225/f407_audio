#ifdef TEST

#include "unity.h"
#include "fifo.h"
#include "debug_uart.h"
#include "mock_cortex.h"
#include "mock_gpio.h"
#include "mock_hal.h"
#include "mock_dma.h"
#include "mock_uart.h"

#include <string.h>

extern int _write(int file, char *ptr, int len);

void setUp(void)
{
}

void tearDown(void)
{
}

void test_debug_uart_init_for_ok(void)
{
    debug_uart_error_reset();

    HAL_UART_RegisterCallback_IgnoreAndReturn(HAL_OK);
    HAL_UART_UnRegisterCallback_IgnoreAndReturn(HAL_OK);
    HAL_NVIC_SetPriority_Expect(UARTxTX_DMA_Stream_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ_Expect(UARTxTX_DMA_Stream_IRQn);

    HAL_UART_GetState_ExpectAnyArgsAndReturn(HAL_UART_STATE_RESET);
    HAL_UART_Init_ExpectAnyArgsAndReturn(HAL_OK);
    
    debug_uart_status_t actual = debug_uart_init();
    debug_uart_status_t expected = DEBUG_UART_OK;

    TEST_ASSERT_EQUAL(expected, actual);
}

void test_debug_uart_init_for_busy_state_error(void)
{
    debug_uart_error_reset();

    HAL_UART_RegisterCallback_IgnoreAndReturn(HAL_OK);
    HAL_UART_UnRegisterCallback_IgnoreAndReturn(HAL_OK);
    HAL_NVIC_SetPriority_Expect(UARTxTX_DMA_Stream_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ_Expect(UARTxTX_DMA_Stream_IRQn);

    HAL_UART_GetState_ExpectAnyArgsAndReturn(HAL_UART_STATE_BUSY);
    HAL_UART_Init_ExpectAnyArgsAndReturn(HAL_OK);
    
    debug_uart_status_t actual = debug_uart_init();
    debug_uart_status_t expected = DEBUG_UART_INIT_ERROR;

    TEST_ASSERT_EQUAL(expected, actual);
}

void test_debug_uart_init_for_hal_init_error(void)
{
    debug_uart_error_reset();

    HAL_UART_RegisterCallback_IgnoreAndReturn(HAL_OK);
    HAL_UART_UnRegisterCallback_IgnoreAndReturn(HAL_OK);
    HAL_NVIC_SetPriority_Expect(UARTxTX_DMA_Stream_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ_Expect(UARTxTX_DMA_Stream_IRQn);

    HAL_UART_GetState_ExpectAnyArgsAndReturn(HAL_UART_STATE_RESET);
    HAL_UART_Init_ExpectAnyArgsAndReturn(HAL_ERROR);
    
    debug_uart_status_t actual = debug_uart_init();
    debug_uart_status_t expected = DEBUG_UART_INIT_ERROR;

    TEST_ASSERT_EQUAL(expected, actual);
}

void test_debug_uart_deinit_for_ok(void)
{
    debug_uart_error_reset();

    HAL_UART_UnRegisterCallback_IgnoreAndReturn(HAL_OK);
    HAL_NVIC_DisableIRQ_Expect(UARTxTX_DMA_Stream_IRQn);

    HAL_UART_DeInit_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_UART_GetState_ExpectAnyArgsAndReturn(HAL_UART_STATE_RESET);
    
    debug_uart_status_t actual = debug_uart_deinit();
    debug_uart_status_t expected = DEBUG_UART_OK;

    TEST_ASSERT_EQUAL(expected, actual);
}

void test_debug_uart_deinit_for_hal_deinit_error(void)
{
    debug_uart_error_reset();

    HAL_UART_UnRegisterCallback_IgnoreAndReturn(HAL_OK);
    HAL_NVIC_DisableIRQ_Expect(UARTxTX_DMA_Stream_IRQn);

    HAL_UART_DeInit_ExpectAnyArgsAndReturn(HAL_ERROR);
    HAL_UART_GetState_ExpectAnyArgsAndReturn(HAL_UART_STATE_RESET);
    
    debug_uart_status_t actual = debug_uart_deinit();
    debug_uart_status_t expected = DEBUG_UART_DEINIT_ERROR;

    TEST_ASSERT_EQUAL(expected, actual);
}

void test_debug_uart_deinit_for_busy_state_error(void)
{
    debug_uart_error_reset();

    HAL_UART_UnRegisterCallback_IgnoreAndReturn(HAL_OK);
    HAL_NVIC_DisableIRQ_Expect(UARTxTX_DMA_Stream_IRQn);

    HAL_UART_DeInit_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_UART_GetState_ExpectAnyArgsAndReturn(HAL_UART_STATE_BUSY);
    
    debug_uart_status_t actual = debug_uart_deinit();
    debug_uart_status_t expected = DEBUG_UART_DEINIT_ERROR;

    TEST_ASSERT_EQUAL(expected, actual);
}

void test_write(void)
{
    char str[] = "some string.\r\n";
    int len = strlen(str);

    HAL_UART_Transmit_DMA_ExpectAnyArgsAndReturn(HAL_OK);

    int actual = _write(STDOUT_FILENO, str, len);
    int expected = len;

    TEST_ASSERT_EQUAL(expected, actual);
}

#endif // TEST
