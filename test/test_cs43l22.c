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


void test_volume_calculation(void)
{
    for (uint8_t i = 0; i < 240; i++) {
        float   volume  = ((float)i)*0.5;

        float   p       = (volume > 111.7) ? (1.117) : (0.01*volume);
        uint8_t mst_vol = (uint8_t)(204*p + 52.5);

        printf("volume: %.2f%%, value: %d, p: %.3f\n", volume, mst_vol, p);
    }
    
}


#endif // TEST
