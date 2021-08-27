#ifdef TEST

#include "unity.h"

#include "audio_io.h"
#include "mock_i2c.h"
#include "mock_cortex.h"
#include "mock_gpio.h"
#include "mock_hal.h"
#include "mock_dma.h"

static void reset_gpio_init(void)
{
	HAL_GPIO_Init_ExpectAnyArgs();
}

static void reset_gpio_deinit(void)
{
	HAL_GPIO_DeInit_Expect(AUDIO_IO_RESET_PORT, AUDIO_IO_RESET_PIN);
}

static void audio_io_reset(void)
{
	HAL_GPIO_WritePin_Expect(AUDIO_IO_RESET_PORT, AUDIO_IO_RESET_PIN, GPIO_PIN_RESET);
	HAL_Delay_Expect(5);
	HAL_GPIO_WritePin_Expect(AUDIO_IO_RESET_PORT, AUDIO_IO_RESET_PIN, GPIO_PIN_SET);
}

static void dma_init(void)
{
	HAL_NVIC_SetPriority_Expect(I2CxTX_DMA_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ_Expect(I2CxTX_DMA_Stream_IRQn);
	HAL_NVIC_SetPriority_Expect(I2CxRX_DMA_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ_Expect(I2CxRX_DMA_Stream_IRQn);
}

static void dma_deinit(void)
{
	HAL_NVIC_DisableIRQ_Expect(I2CxTX_DMA_Stream_IRQn);
	HAL_NVIC_DisableIRQ_Expect(I2CxRX_DMA_Stream_IRQn);
}

static void i2c_gpio_init(void)
{
	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_GPIO_Init_ExpectAnyArgs();
}

static void i2c_gpio_deinit(void)
{
	HAL_GPIO_DeInit_Expect(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
	HAL_GPIO_DeInit_Expect(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);
}

void setUp(void)
{
}

void tearDown(void)
{
}

/*
 *  Audio IO Init tests
 */
void test_audio_io_init_for_ok(void)
{
	audio_io_error_reset();
	reset_gpio_init();
	audio_io_reset();
	dma_init();
	HAL_I2C_RegisterCallback_IgnoreAndReturn(HAL_OK);

	HAL_I2C_GetState_ExpectAnyArgsAndReturn(HAL_I2C_STATE_RESET);
	HAL_I2C_Init_ExpectAnyArgsAndReturn(HAL_OK);
	HAL_I2C_IsDeviceReady_ExpectAnyArgsAndReturn(HAL_OK);
	
	audio_status_t actual;
	audio_status_t expected = AUDIO_IO_OK;
	actual = audio_io_init();
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

void test_audio_io_init_for_busy_state_error(void)
{
	audio_io_error_reset();
	reset_gpio_init();
	audio_io_reset();
	dma_init();
	HAL_I2C_RegisterCallback_IgnoreAndReturn(HAL_OK);
	
	HAL_I2C_GetState_ExpectAnyArgsAndReturn(HAL_I2C_STATE_BUSY);
	HAL_I2C_Init_ExpectAnyArgsAndReturn(HAL_OK);
	HAL_I2C_IsDeviceReady_ExpectAnyArgsAndReturn(HAL_OK);

	audio_status_t actual;
	audio_status_t expected = AUDIO_IO_I2C_INIT_ERROR;
	actual = audio_io_init();
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

void test_audio_io_init_for_device_not_ready_error(void)
{
	audio_io_error_reset();
	reset_gpio_init();
	audio_io_reset();
	dma_init();
	HAL_I2C_RegisterCallback_IgnoreAndReturn(HAL_OK);
	
	HAL_I2C_GetState_ExpectAnyArgsAndReturn(HAL_I2C_STATE_RESET);
	HAL_I2C_Init_ExpectAnyArgsAndReturn(HAL_OK);
	HAL_I2C_IsDeviceReady_ExpectAnyArgsAndReturn(HAL_BUSY);
	
	audio_status_t actual;
	audio_status_t expected = AUDIO_IO_I2C_INIT_ERROR;
	actual = audio_io_init();
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

void test_audio_io_init_for_ll_init_error(void)
{
	audio_io_error_reset();
	reset_gpio_init();
	audio_io_reset();
	dma_init();
	HAL_I2C_RegisterCallback_IgnoreAndReturn(HAL_OK);
	
	HAL_I2C_GetState_ExpectAnyArgsAndReturn(HAL_I2C_STATE_RESET);
	HAL_I2C_Init_ExpectAnyArgsAndReturn(HAL_ERROR);
	HAL_I2C_IsDeviceReady_ExpectAnyArgsAndReturn(HAL_OK);
	
	audio_status_t actual;
	audio_status_t expected = AUDIO_IO_I2C_INIT_ERROR;
	actual = audio_io_init();
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

/*
 *  Audio IO DeInit tests
 */
void test_audio_io_deinit_for_ok(void)
{	
	audio_io_error_reset();
	reset_gpio_deinit();
	HAL_I2C_UnRegisterCallback_IgnoreAndReturn(HAL_OK);

	HAL_I2C_DeInit_ExpectAnyArgsAndReturn(HAL_OK);
	HAL_I2C_GetState_ExpectAnyArgsAndReturn(HAL_I2C_STATE_RESET);
	
	dma_deinit();
	
	audio_status_t actual;
	audio_status_t expected = AUDIO_IO_OK;
	actual = audio_io_deinit();
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

void test_audio_io_deinit_for_ll_deinit_error(void)
{	
	audio_io_error_reset();
	reset_gpio_deinit();
	HAL_I2C_UnRegisterCallback_IgnoreAndReturn(HAL_OK);

	HAL_I2C_DeInit_ExpectAnyArgsAndReturn(HAL_ERROR);
	HAL_I2C_GetState_ExpectAnyArgsAndReturn(HAL_I2C_STATE_RESET);
	
	dma_deinit();
	
	audio_status_t actual;
	audio_status_t expected = AUDIO_IO_I2C_DEINIT_ERROR;
	actual = audio_io_deinit();
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

void test_audio_io_deinit_for_state_error(void)
{	
	audio_io_error_reset();
	reset_gpio_deinit();
	HAL_I2C_UnRegisterCallback_IgnoreAndReturn(HAL_OK);
	
	HAL_I2C_DeInit_ExpectAnyArgsAndReturn(HAL_OK);
	HAL_I2C_GetState_ExpectAnyArgsAndReturn(HAL_I2C_STATE_BUSY);

	dma_deinit();
	
	audio_status_t actual;
	audio_status_t expected = AUDIO_IO_I2C_DEINIT_ERROR;
	actual = audio_io_deinit();
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

#endif // TEST
