#ifdef TEST

#include "unity.h"

#include "cs43l22.h"
#include "mock_audio_io.h"

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

#endif // TEST
