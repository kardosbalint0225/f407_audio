#ifdef TEST

#include "unity.h"

#include "audio.h"
#include "audio_io.h"
#include "cs43l22.h"
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

void test_audio_NeedToImplement(void)
{
    TEST_IGNORE_MESSAGE("Need to Implement audio");
}

#endif // TEST
