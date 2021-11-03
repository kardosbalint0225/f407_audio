#ifdef TEST

#include "unity.h"

#include "cs43l22.h"
#include "audio_io.h"
#include "mock_i2c.h"
#include "mock_cortex.h"
#include "mock_gpio.h"
#include "mock_hal.h"
#include "mock_dma.h"
#include "mock_i2s.h"
#include "mock_rcc.h"
#include "mock_rcc_ex.h"
//#include "mock_audio_io.h"

extern uint32_t sfind(const int32_t *array, const uint32_t size, const int32_t value);

void setUp(void)
{
}

void tearDown(void)
{
}

void test_cs43l22_map_increment_type_endianness(void)
{
    /**< Little Endian */
    cs43l22_map_t map = {
        .address = CS43L22_ID,  // 0b00000001
        .INCR    = 1,           // 0b10000000
    };                          // 0x81

    uint8_t expected = 0x81;
    TEST_ASSERT_EQUAL(expected, map.byte);
}

void test_beep_generator_volume(void)
{
    const int32_t beep_volume[32] = { 
	     -6,  -4,  -2,   0,   2,   4,   6, -56, -54, -52, -50, 
	    -48, -46, -44, -42, -40, -38, -36, -34, -32, -30, -28, 
	    -26, -24, -22, -20, -18, -16, -14, -12, -10, -8 
    };

    for (uint8_t i = 0; i < 128; i++) {
        beep_generator_t beep = {
            .volume = i,
        };

        uint8_t beep_volume_index;
        beep.volume = (beep.volume > 110) ? 110 : beep.volume;
        int32_t volume = (int32_t)(0.29*((float)beep.volume));
        volume = (-2)*(28 - volume);
        
        beep_volume_index = (uint8_t)sfind(beep_volume, sizeof(beep_volume)/sizeof(int32_t), volume);

        TEST_ASSERT_EQUAL(volume, beep_volume[beep_volume_index]);
    }
}

void test_beep_generator_register_config(void)
{
    const int32_t beep_volume[32] = { 
	     -6,  -4,  -2,   0,   2,   4,   6, -56, -54, -52, -50, 
	    -48, -46, -44, -42, -40, -38, -36, -34, -32, -30, -28, 
	    -26, -24, -22, -20, -18, -16, -14, -12, -10, -8 
    };

    beep_generator_t beep;
    beep.frequency   = BEEP_GENERATOR_FREQUENCY_A5_PITCH;
    beep.on_time     = BEEP_GENERATOR_ON_TIME_3_20_S;
    beep.off_time    = BEEP_GENERATOR_OFF_TIME_6_60_S;
    beep.volume      = 31; // %
    beep.mix_disable = BEEP_GENERATOR_MIX_ENABLED;

    uint8_t beep_volume_index;
    beep.volume = (beep.volume > 110) ? 110 : beep.volume;
    int32_t volume = (int32_t)(0.29*((float)beep.volume));
    volume = (-2)*(28 - volume);
        
    beep_volume_index = (uint8_t)sfind(beep_volume, sizeof(beep_volume)/sizeof(int32_t), volume);

    uint8_t beep_generator_register_config[3] = {0, 0, 0};

    beep_generator_register_config[0] = beep.frequency    | beep.on_time;
    beep_generator_register_config[1] = beep.off_time     | beep_volume_index;
    beep_generator_register_config[2] &= 0x1F;
    beep_generator_register_config[2] |= beep.mix_disable | BEEP_GENERATOR_OCCURENCE_OFF;

    TEST_ASSERT_EQUAL(0b01101001, beep_generator_register_config[0]);
    TEST_ASSERT_EQUAL(0b10001111, beep_generator_register_config[1]);
    TEST_ASSERT_EQUAL(0b00000000, beep_generator_register_config[2]);

    ////////////////////////////////////////////////////////////////////////////////////////////////

    beep.frequency   = BEEP_GENERATOR_FREQUENCY_E6_PITCH;
    beep.on_time     = BEEP_GENERATOR_ON_TIME_780_MS;
    beep.off_time    = BEEP_GENERATOR_OFF_TIME_8_05_S;
    beep.volume      = 59; // %
    beep.mix_disable = BEEP_GENERATOR_MIX_DISABLED;

    beep.volume       = (beep.volume > 110) ? 110 : beep.volume;
    volume            = (int32_t)(0.29*((float)beep.volume));
    volume            = (-2)*(28 - volume);
    beep_volume_index = (uint8_t)sfind(beep_volume, sizeof(beep_volume)/sizeof(int32_t), volume);

    beep_generator_register_config[0] = 0;
    beep_generator_register_config[1] = 0;
    beep_generator_register_config[2] = 0;

    beep_generator_register_config[0] = beep.frequency    | beep.on_time;
    beep_generator_register_config[1] = beep.off_time     | beep_volume_index;
    beep_generator_register_config[2] &= 0x1F;
    beep_generator_register_config[2] |= beep.mix_disable | BEEP_GENERATOR_OCCURENCE_OFF;

    TEST_ASSERT_EQUAL(0b10100010, beep_generator_register_config[0]);
    TEST_ASSERT_EQUAL(0b10111000, beep_generator_register_config[1]);
    TEST_ASSERT_EQUAL(0b00100000, beep_generator_register_config[2]);
}

void test_beep_generator_init_for_ok(void)
{
    uint8_t register_values[52] = {
        0xE0U, 0x01U, 0x00U, 0x05U, 0xA0U, 0x00U, 0x00U, 0x01U, 0x01U, 0x05U, 0x00U, 0x00U,
	    0x60U, 0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
	    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x88U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
	    0x00U, 0x00U, 0x00U, 0x7FU, 0x00U, 0x00U, 0x00U, 0x00U,	0x00U, 0x00U, 0x00U, 0x00U,
	    0x00U, 0x00U, 0x00U, 0x50U,
    };

    HAL_NVIC_SetPriority_Expect(I2CxTX_DMA_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ_Expect(I2CxTX_DMA_Stream_IRQn);
	HAL_NVIC_SetPriority_Expect(I2CxRX_DMA_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ_Expect(I2CxRX_DMA_Stream_IRQn);
    HAL_NVIC_SetPriority_Expect(I2SxTX_DMA_Stream_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ_Expect(I2SxTX_DMA_Stream_IRQn);
    HAL_I2C_RegisterCallback_IgnoreAndReturn(HAL_OK);
    HAL_I2C_GetState_ExpectAnyArgsAndReturn(HAL_I2C_STATE_RESET);
    HAL_I2C_Init_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_I2C_IsDeviceReady_ExpectAnyArgsAndReturn(HAL_OK);

    HAL_I2C_Mem_Read_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_I2C_Mem_Read_ReturnArrayThruPtr_pData(register_values, 52);
    HAL_I2C_Mem_Read_IgnoreArg_hi2c();
    HAL_I2C_Mem_Read_IgnoreArg_DevAddress();
    HAL_I2C_Mem_Read_IgnoreArg_MemAddress();
    HAL_I2C_Mem_Read_IgnoreArg_MemAddSize();
    HAL_I2C_Mem_Read_IgnoreArg_Size();
    HAL_I2C_Mem_Read_IgnoreArg_Timeout();
    
    HAL_I2S_RegisterCallback_IgnoreAndReturn(HAL_OK);
    HAL_I2S_GetState_ExpectAnyArgsAndReturn(HAL_I2S_STATE_RESET);
    HAL_I2S_Init_ExpectAnyArgsAndReturn(HAL_OK);


    cs43l22_if_t interface = {
        .io = {
            .init         = audio_io_init,
            .deinit       = audio_io_deinit,
            .read         = audio_io_read,
            .write        = audio_io_write,
            .reset_init   = audio_io_reset_init,
            .reset_deinit = audio_io_reset_deinit,
            .reset        = audio_io_reset,
        },

        .out = {
            .init         = audio_out_ll_init,
            .deinit       = audio_out_ll_deinit,
            .write        = audio_out_ll_write,
            .pause        = audio_out_ll_pause,
            .resume       = audio_out_ll_resume,
            .stop         = audio_out_ll_stop,
        },
    };

    cs43l22_status_t status = cs43l22_init(&interface);
    TEST_ASSERT_EQUAL(CS43L22_OK, status);

    HAL_I2C_Mem_Read_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_I2C_Mem_Write_ExpectAnyArgsAndReturn(HAL_OK);

    beep_generator_t beep;
    beep.frequency   = BEEP_GENERATOR_FREQUENCY_A5_PITCH;
    beep.on_time     = BEEP_GENERATOR_ON_TIME_3_20_S;
    beep.off_time    = BEEP_GENERATOR_OFF_TIME_6_60_S;
    beep.volume      = 31; // %
    beep.mix_disable = BEEP_GENERATOR_MIX_ENABLED;

    cs43l22_status_t actual = beep_generator_init(&beep);

    TEST_ASSERT_EQUAL(CS43L22_OK, actual);
}

#endif // TEST
