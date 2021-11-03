#ifdef TEST

#include "unity.h"

#include "audio_io.h"
#include "mock_i2c.h"
#include "mock_cortex.h"
#include "mock_gpio.h"
#include "mock_hal.h"
#include "mock_dma.h"
#include "mock_i2s.h"
#include "mock_rcc.h"
#include "mock_rcc_ex.h"

static void reset_gpio_init(void)
{
	HAL_GPIO_Init_ExpectAnyArgs();
}

static void reset_gpio_deinit(void)
{
	HAL_GPIO_DeInit_Expect(AUDIO_IO_RESET_PORT, AUDIO_IO_RESET_PIN);
}

//static void audio_io_reset(void)
//{
//	HAL_GPIO_WritePin_Expect(AUDIO_IO_RESET_PORT, AUDIO_IO_RESET_PIN, GPIO_PIN_RESET);
//	HAL_Delay_Expect(5);
//	HAL_GPIO_WritePin_Expect(AUDIO_IO_RESET_PORT, AUDIO_IO_RESET_PIN, GPIO_PIN_SET);
//}

static void dma_init(void)
{
	HAL_NVIC_SetPriority_Expect(I2CxTX_DMA_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ_Expect(I2CxTX_DMA_Stream_IRQn);
	HAL_NVIC_SetPriority_Expect(I2CxRX_DMA_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ_Expect(I2CxRX_DMA_Stream_IRQn);
	HAL_NVIC_SetPriority_Expect(I2SxTX_DMA_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ_Expect(I2SxTX_DMA_Stream_IRQn);
}

static void dma_deinit(void)
{
	HAL_NVIC_DisableIRQ_Expect(I2CxTX_DMA_Stream_IRQn);
	HAL_NVIC_DisableIRQ_Expect(I2CxRX_DMA_Stream_IRQn);
	HAL_NVIC_DisableIRQ_Expect(I2SxTX_DMA_Stream_IRQn);
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
//	reset_gpio_init();
//	audio_io_reset();
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
//	reset_gpio_init();
//	audio_io_reset();
	dma_init();
	HAL_I2C_RegisterCallback_IgnoreAndReturn(HAL_OK);
	
	HAL_I2C_GetState_ExpectAnyArgsAndReturn(HAL_I2C_STATE_BUSY);
	HAL_I2C_Init_ExpectAnyArgsAndReturn(HAL_OK);
	HAL_I2C_IsDeviceReady_ExpectAnyArgsAndReturn(HAL_OK);

	audio_status_t actual;
	audio_status_t expected = AUDIO_IO_INIT_ERROR;
	actual = audio_io_init();
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

void test_audio_io_init_for_device_not_ready_error(void)
{
	audio_io_error_reset();
//	reset_gpio_init();
//	audio_io_reset();
	dma_init();
	HAL_I2C_RegisterCallback_IgnoreAndReturn(HAL_OK);
	
	HAL_I2C_GetState_ExpectAnyArgsAndReturn(HAL_I2C_STATE_RESET);
	HAL_I2C_Init_ExpectAnyArgsAndReturn(HAL_OK);
	HAL_I2C_IsDeviceReady_ExpectAnyArgsAndReturn(HAL_BUSY);
	
	audio_status_t actual;
	audio_status_t expected = AUDIO_IO_INIT_ERROR;
	actual = audio_io_init();
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

void test_audio_io_init_for_ll_init_error(void)
{
	audio_io_error_reset();
//	reset_gpio_init();
//	audio_io_reset();
	dma_init();
	HAL_I2C_RegisterCallback_IgnoreAndReturn(HAL_OK);
	
	HAL_I2C_GetState_ExpectAnyArgsAndReturn(HAL_I2C_STATE_RESET);
	HAL_I2C_Init_ExpectAnyArgsAndReturn(HAL_ERROR);
	HAL_I2C_IsDeviceReady_ExpectAnyArgsAndReturn(HAL_OK);
	
	audio_status_t actual;
	audio_status_t expected = AUDIO_IO_INIT_ERROR;
	actual = audio_io_init();
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

/*
 *  Audio IO DeInit tests
 */
void test_audio_io_deinit_for_ok(void)
{	
	audio_io_error_reset();
//	reset_gpio_deinit();
	HAL_I2C_UnRegisterCallback_IgnoreAndReturn(HAL_OK);

	HAL_I2C_DeInit_ExpectAnyArgsAndReturn(HAL_OK);
	
	dma_deinit();
	
	audio_status_t actual;
	audio_status_t expected = AUDIO_IO_OK;
	actual = audio_io_deinit();
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

void test_audio_io_deinit_for_ll_deinit_error(void)
{	
	audio_io_error_reset();
//	reset_gpio_deinit();
	HAL_I2C_UnRegisterCallback_IgnoreAndReturn(HAL_OK);

	HAL_I2C_DeInit_ExpectAnyArgsAndReturn(HAL_ERROR);
	
	dma_deinit();
	
	audio_status_t actual;
	audio_status_t expected = AUDIO_IO_DEINIT_ERROR;
	actual = audio_io_deinit();
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

/*
 *  Audio Out Init tests
 */
uint16_t audio_buffer[1024];

void audio_callback(uint16_t *data, const audio_out_cb_id_t callback_id)
{

}

void test_audio_out_init_for_ok(void)
{
	audio_io_error_reset();

	audio_out_ll_hw_params_t hw_params = {
		.standard        = I2S_STANDARD_PHILIPS,
		.data_format     = I2S_DATAFORMAT_16B,
		.audio_frequency = I2S_AUDIOFREQ_96K,
	};

	HAL_I2S_RegisterCallback_IgnoreAndReturn(HAL_OK);
	HAL_I2S_GetState_ExpectAnyArgsAndReturn(HAL_I2S_STATE_RESET);
	HAL_I2S_Init_ExpectAnyArgsAndReturn(HAL_OK);
	
	audio_status_t actual = audio_out_ll_init(&hw_params);
	audio_status_t expected = AUDIO_IO_OK;
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

void test_audio_out_init_for_i2s_busy_state_error(void)
{
	audio_io_error_reset();

	audio_out_ll_hw_params_t hw_params = {
		.standard        = I2S_STANDARD_PHILIPS,
		.data_format     = I2S_DATAFORMAT_16B,
		.audio_frequency = I2S_AUDIOFREQ_96K,
	};

	HAL_I2S_RegisterCallback_IgnoreAndReturn(HAL_OK);
	HAL_I2S_GetState_ExpectAnyArgsAndReturn(HAL_I2S_STATE_BUSY);
	HAL_I2S_Init_ExpectAnyArgsAndReturn(HAL_OK);
	
	audio_status_t actual = audio_out_ll_init(&hw_params);
	audio_status_t expected = AUDIO_OUT_INIT_ERROR;
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

void test_audio_out_init_for_i2s_init_error(void)
{
	audio_io_error_reset();

	audio_out_ll_hw_params_t hw_params = {
		.standard        = I2S_STANDARD_PHILIPS,
		.data_format     = I2S_DATAFORMAT_16B,
		.audio_frequency = I2S_AUDIOFREQ_96K,
	};

	HAL_I2S_RegisterCallback_IgnoreAndReturn(HAL_OK);
	HAL_I2S_GetState_ExpectAnyArgsAndReturn(HAL_I2S_STATE_RESET);
	HAL_I2S_Init_ExpectAnyArgsAndReturn(HAL_ERROR);
	
	audio_status_t actual = audio_out_ll_init(&hw_params);
	audio_status_t expected = AUDIO_OUT_INIT_ERROR;
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}


/*
 *  Audio Out DeInit tests
 */
void test_audio_out_deinit_for_ok(void)
{
	audio_io_error_reset();

	HAL_I2S_UnRegisterCallback_IgnoreAndReturn(HAL_OK);
	HAL_I2S_DeInit_ExpectAnyArgsAndReturn(HAL_OK);

	audio_status_t actual = audio_out_ll_deinit();
	audio_status_t expected = AUDIO_IO_OK;
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

void test_audio_out_deinit_for_i2s_deinit_error(void)
{
	audio_io_error_reset();

	HAL_I2S_UnRegisterCallback_IgnoreAndReturn(HAL_OK);
	HAL_I2S_DeInit_ExpectAnyArgsAndReturn(HAL_ERROR);

	audio_status_t actual = audio_out_ll_deinit();
	audio_status_t expected = AUDIO_OUT_DEINIT_ERROR;
	TEST_ASSERT_EQUAL((int)expected, (int)actual);
}

/*
 *  find tests
 */
extern uint32_t find(const uint32_t *array, const uint32_t size, const uint32_t value);

void test_find_for_item_presence(void)
{
	const uint32_t I2Sx_Fs[11]   = {8000U, 11025U, 12000U, 16000U, 22050U, 24000U, 32000U, 44100U, 48000U, 88200U, 96000U};
	uint32_t actual = find(I2Sx_Fs, sizeof(I2Sx_Fs)/sizeof(uint32_t), 44100);
	uint32_t expected = 7;
	TEST_ASSERT_EQUAL(expected, actual);
}

void test_find_for_item_absence(void)
{
	const uint32_t I2Sx_Fs[11]   = {8000U, 11025U, 12000U, 16000U, 22050U, 24000U, 32000U, 44100U, 48000U, 88200U, 96000U};
	uint32_t actual = find(I2Sx_Fs, sizeof(I2Sx_Fs)/sizeof(uint32_t), 33333);
	uint32_t expected = 0xFFFFFFFF;
	TEST_ASSERT_EQUAL(expected, actual);
}

/*
 *   Msp tests
 */
extern void I2Cx_MspInit(I2C_HandleTypeDef *hi2c);
extern void I2Cx_MspDeInit(I2C_HandleTypeDef *hi2c);

void test_i2cx_mspinit_for_ok(void)
{
	I2C_HandleTypeDef hi2c;
	audio_io_error_reset();

	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_DMA_GetState_ExpectAnyArgsAndReturn(HAL_DMA_STATE_RESET);
	HAL_DMA_Init_ExpectAnyArgsAndReturn(HAL_OK);
	HAL_DMA_GetState_ExpectAnyArgsAndReturn(HAL_DMA_STATE_RESET);
	HAL_DMA_Init_ExpectAnyArgsAndReturn(HAL_OK);

	I2Cx_MspInit(&hi2c);

	uint32_t actual_i2c_error;
	uint32_t actual_i2s_error;
	uint32_t expected_i2c_error = 0;
	uint32_t expected_i2s_error = 0;

	audio_io_get_error(&actual_i2c_error, &actual_i2s_error);
	TEST_ASSERT_EQUAL(expected_i2c_error, actual_i2c_error);
	TEST_ASSERT_EQUAL(expected_i2s_error, actual_i2s_error);
}

void test_i2cx_mspinit_for_error(void)
{
	I2C_HandleTypeDef hi2c;
	audio_io_error_reset();

	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_DMA_GetState_ExpectAnyArgsAndReturn(HAL_DMA_STATE_RESET);
	HAL_DMA_Init_ExpectAnyArgsAndReturn(HAL_ERROR);
	HAL_DMA_GetState_ExpectAnyArgsAndReturn(HAL_DMA_STATE_RESET);
	HAL_DMA_Init_ExpectAnyArgsAndReturn(HAL_OK);

	I2Cx_MspInit(&hi2c);

	uint32_t actual_i2c_error;
	uint32_t actual_i2s_error;
	uint32_t expected_i2c_error = 0;
	uint32_t expected_i2s_error = 0;

	audio_io_get_error(&actual_i2c_error, &actual_i2s_error);
	TEST_ASSERT_GREATER_THAN(expected_i2c_error, actual_i2c_error);
	TEST_ASSERT_EQUAL(expected_i2s_error, actual_i2s_error);
}

void test_i2cx_mspdeinit_for_ok(void)
{
	I2C_HandleTypeDef hi2c;
	audio_io_error_reset();

	HAL_GPIO_DeInit_ExpectAnyArgs();
	HAL_GPIO_DeInit_ExpectAnyArgs();
	HAL_DMA_DeInit_ExpectAnyArgsAndReturn(HAL_OK);
	HAL_DMA_DeInit_ExpectAnyArgsAndReturn(HAL_OK);

	I2Cx_MspDeInit(&hi2c);

	uint32_t actual_i2c_error;
	uint32_t actual_i2s_error;
	uint32_t expected_i2c_error = 0;
	uint32_t expected_i2s_error = 0;

	audio_io_get_error(&actual_i2c_error, &actual_i2s_error);

	TEST_ASSERT_EQUAL(expected_i2c_error, actual_i2c_error);
	TEST_ASSERT_EQUAL(expected_i2s_error, actual_i2s_error);
}

void test_i2cx_mspdeinit_for_error(void)
{
	I2C_HandleTypeDef hi2c;
	audio_io_error_reset();

	HAL_GPIO_DeInit_ExpectAnyArgs();
	HAL_GPIO_DeInit_ExpectAnyArgs();
	HAL_DMA_DeInit_ExpectAnyArgsAndReturn(HAL_OK);
	HAL_DMA_DeInit_ExpectAnyArgsAndReturn(HAL_ERROR);

	I2Cx_MspDeInit(&hi2c);

	uint32_t actual_i2c_error;
	uint32_t actual_i2s_error;
	uint32_t expected_i2c_error = 0;
	uint32_t expected_i2s_error = 0;

	audio_io_get_error(&actual_i2c_error, &actual_i2s_error);

	TEST_ASSERT_GREATER_THAN(expected_i2c_error, actual_i2c_error);
	TEST_ASSERT_EQUAL(expected_i2s_error, actual_i2s_error);
}

extern void I2Sx_MspInit(I2S_HandleTypeDef *hi2s);
extern void I2Sx_MspDeInit(I2S_HandleTypeDef *hi2s);

void test_i2sx_mspinit_for_ok(void)
{
	I2S_HandleTypeDef hi2s;
	audio_io_error_reset();

	audio_out_ll_hw_params_t hw_params = {
		.standard        = I2S_STANDARD_PHILIPS,
		.data_format     = I2S_DATAFORMAT_16B,
		.audio_frequency = I2S_AUDIOFREQ_96K,
	};

	audio_out_ll_set_hw_params(&hw_params);

	HAL_RCCEx_EnablePLLI2S_ExpectAnyArgsAndReturn(HAL_OK);
	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_DMA_GetState_ExpectAnyArgsAndReturn(HAL_DMA_STATE_RESET);
	HAL_DMA_Init_ExpectAnyArgsAndReturn(HAL_OK);

	I2Sx_MspInit(&hi2s);

	uint32_t actual_i2c_error;
	uint32_t actual_i2s_error;
	uint32_t expected_i2c_error = 0;
	uint32_t expected_i2s_error = 0;

	audio_io_get_error(&actual_i2c_error, &actual_i2s_error);

	TEST_ASSERT_EQUAL(expected_i2c_error, actual_i2c_error);
	TEST_ASSERT_EQUAL(expected_i2s_error, actual_i2s_error);
}

void test_i2sx_mspinit_for_error(void)
{
	I2S_HandleTypeDef hi2s;
	audio_io_error_reset();

	audio_out_ll_hw_params_t hw_params = {
		.standard        = I2S_STANDARD_PHILIPS,
		.data_format     = I2S_DATAFORMAT_16B,
		.audio_frequency = 0,
	};

	audio_out_ll_set_hw_params(&hw_params);

	HAL_RCCEx_EnablePLLI2S_ExpectAnyArgsAndReturn(HAL_OK);
	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_GPIO_Init_ExpectAnyArgs();
	HAL_DMA_GetState_ExpectAnyArgsAndReturn(HAL_DMA_STATE_RESET);
	HAL_DMA_Init_ExpectAnyArgsAndReturn(HAL_OK);

	I2Sx_MspInit(&hi2s);

	uint32_t actual_i2c_error;
	uint32_t actual_i2s_error;
	uint32_t expected_i2c_error = 0;
	uint32_t expected_i2s_error = 0;

	audio_io_get_error(&actual_i2c_error, &actual_i2s_error);

	TEST_ASSERT_EQUAL(expected_i2c_error, actual_i2c_error);
	TEST_ASSERT_GREATER_THAN(expected_i2s_error, actual_i2s_error);
}

void test_i2sx_mspdeinit_for_ok(void)
{
	I2S_HandleTypeDef hi2s;
	audio_io_error_reset();

	HAL_RCCEx_DisablePLLI2S_ExpectAndReturn(HAL_OK);
	HAL_GPIO_DeInit_ExpectAnyArgs();
	HAL_GPIO_DeInit_ExpectAnyArgs();
	HAL_GPIO_DeInit_ExpectAnyArgs();
	HAL_GPIO_DeInit_ExpectAnyArgs();
	HAL_DMA_DeInit_ExpectAnyArgsAndReturn(HAL_OK);

	I2Sx_MspDeInit(&hi2s);

	uint32_t actual_i2c_error;
	uint32_t actual_i2s_error;
	uint32_t expected_i2c_error = 0;
	uint32_t expected_i2s_error = 0;

	audio_io_get_error(&actual_i2c_error, &actual_i2s_error);

	TEST_ASSERT_EQUAL(expected_i2c_error, actual_i2c_error);
	TEST_ASSERT_EQUAL(expected_i2s_error, actual_i2s_error);
}

void test_i2sx_mspdeinit_for_error(void)
{
	I2S_HandleTypeDef hi2s;
	audio_io_error_reset();

	HAL_RCCEx_DisablePLLI2S_ExpectAndReturn(HAL_ERROR);
	HAL_GPIO_DeInit_ExpectAnyArgs();
	HAL_GPIO_DeInit_ExpectAnyArgs();
	HAL_GPIO_DeInit_ExpectAnyArgs();
	HAL_GPIO_DeInit_ExpectAnyArgs();
	HAL_DMA_DeInit_ExpectAnyArgsAndReturn(HAL_ERROR);

	I2Sx_MspDeInit(&hi2s);

	uint32_t actual_i2c_error;
	uint32_t actual_i2s_error;
	uint32_t expected_i2c_error = 0;
	uint32_t expected_i2s_error = 0;

	audio_io_get_error(&actual_i2c_error, &actual_i2s_error);

	TEST_ASSERT_EQUAL(expected_i2c_error, actual_i2c_error);
	TEST_ASSERT_GREATER_THAN(expected_i2s_error, actual_i2s_error);
}

void test_defines(void)
{
	TEST_ASSERT_EQUAL(I2S_STANDARD_PHILIPS,   AUDIO_OUT_STANDARD_PHILIPS);
	TEST_ASSERT_EQUAL(I2S_STANDARD_MSB,       AUDIO_OUT_STANDARD_LEFT_JUSTIFIED);
	TEST_ASSERT_EQUAL(I2S_STANDARD_LSB,       AUDIO_OUT_STANDARD_RIGHT_JUSTIFIED);
	TEST_ASSERT_EQUAL(I2S_STANDARD_PCM_SHORT, AUDIO_OUT_STANDARD_DSP_MODE);
	TEST_ASSERT_EQUAL(I2S_DATAFORMAT_16B,     AUDIO_OUT_DATAFORMAT_16B);
	TEST_ASSERT_EQUAL(I2S_DATAFORMAT_24B,     AUDIO_OUT_DATAFORMAT_24B);
	TEST_ASSERT_EQUAL(I2S_DATAFORMAT_32B,     AUDIO_OUT_DATAFORMAT_32B);
}

#endif // TEST


