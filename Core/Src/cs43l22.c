#include "cs43l22.h"

#ifdef TEST
	#include "hal.h"
#else
	#include "stm32f4xx_hal.h"
#endif

static cs43l22_err_t cs43l22_error;

static audio_io_if_t cs43l22_io = {
    .init         = audio_io_init,
    .deinit       = audio_io_deinit,
    .read         = audio_io_read,
    .write        = audio_io_write,
    .reset_init   = audio_io_reset_init,
    .reset_deinit = audio_io_reset_deinit,
    .reset        = audio_io_reset,
};

static const uint8_t cs43l22_default_register_values[52] = {
    0xE0U, 0x01U, 0x00U, 0x05U, 0xA0U, 0x00U, 0x00U, 0x01U, 0x01U, 0x05U, 0x00U, 0x00U, 0x60U, 
    0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 
    0x00U, 0x00U, 0x00U, 0x00U, 0x88U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 
    0x7FU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x50U,
};

static const uint8_t cs43l22_default_register_value_masks[52] = {
    0xF8U, 0xFFU, 0x00U, 0xFFU, 0xFFU, 0xDFU, 0x48U, 0x0FU, 0x0FU, 0x0FU, 0x00U, 0x80U, 0xFFU,
    0xFFU, 0xFFU, 0x00U, 0x00U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0x00U, 0x00U, 0x00U, 0x00U, 0xFFU,
    0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xF0U, 0xFFU,
    0xFFU, 0x3FU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0xCFU, 0x00U, 0x00U, 0x00U, 0x00U, 0xF0U,
};


static void power_up(cs43l22_power_state_t power_state);
static void power_down(cs43l22_power_state_t power_state);
static void load_init_sequence(void);
static void verify_default_state(void);
static void load_register_settings(void);
static void verified_io_write(const uint8_t address, uint8_t *data);


#ifdef TEST
void cs43l22_error_reset(void)
{
    cs43l22_error.io.w  = 0;
}

void cs43l22_get_error(uint32_t *io)
{
    *io  = cs43l22_error.io.w;
}

#endif

codec_status_t cs43l22_init(void)
{
    cs43l22_error.io.w  = 0;

    cs43l22_io.reset_init();
    power_up(CS43L22_POWER_STATE_RESET);

    codec_status_t retc = (0 != cs43l22_error.io.w) ? (CODEC_INIT_ERROR) : (CODEC_OK);
    return retc;
}

static void power_up(cs43l22_power_state_t power_state)
{
    switch (power_state)
    {
        case CS43L22_POWER_STATE_RESET : {

            /**< ******************************************************************************************* */
            /**< 1. Hold |RESET low until the power supplies are stable.                                     */
            /**< ******************************************************************************************* */
            cs43l22_io.reset(AUDIO_IO_RESET_STATE_LOW);
            HAL_Delay(1);
            /**< ******************************************************************************************* */

            /**< ******************************************************************************************* */
            /**< 2. Bring |RESET high.                                                                       */
            /**< ******************************************************************************************* */
            cs43l22_io.reset(AUDIO_IO_RESET_STATE_HIGH);
            HAL_Delay(5);

            if (AUDIO_IO_OK != cs43l22_io.init()) {
                cs43l22_error.io.init = 1;
            }

            verify_default_state();
            /**< ******************************************************************************************* */

            /**< ******************************************************************************************* */
            /**< 3. The default state of the "Power Ctl. 1" register (0x02) is 0x01.                         */
            /**<    Load the desired register settings while keeping the "Power Ctl 1" register set to 0x01. */
            /**< ******************************************************************************************* */
            load_register_settings();
            /**< ******************************************************************************************* */

            /**< ******************************************************************************************* */
            /**< 4. Load the required initialization settings listed in Section 4.11.                        */
            /**< ******************************************************************************************* */
            load_init_sequence();
            /**< ******************************************************************************************* */

        } break;

        case CS43L22_POWER_STATE_STANDBY : {

            /**< ******************************************************************************************* */
            /**< 6. Set the Power Ctl 1 register (0x02) to 0x9E                                              */
            /**< ******************************************************************************************* */
            uint8_t power_ctl_1 = 0x9E;
            verified_io_write(POWER_CONTROL_1, &power_ctl_1);
            /**< ******************************************************************************************* */

        } break; 

        default : {

        } break;
    }

    if (0 != cs43l22_error.io.w) {
        cs43l22_error.io.power_up = 1;
    }
}

static void verify_default_state(void)
{
    uint8_t cs43l22_register_values[52];
    const uint8_t size = sizeof(cs43l22_register_values)/sizeof(uint8_t);
    cs43l22_map_t map = {
        .address = CS43L22_ID,
        .INCR    = 1,
    };

    // Read all the registers, then mask out the reserved bits then compare them to the default values
    if (AUDIO_IO_OK != cs43l22_io.read(map.byte, cs43l22_register_values, size, true)) {
        cs43l22_error.io.read = 1;
    }

    for (uint8_t i = 0; i < size; i++) {
        uint8_t expected = cs43l22_default_register_values[i] & cs43l22_default_register_value_masks[i];
        uint8_t actual   = cs43l22_register_values[i]         & cs43l22_default_register_value_masks[i];
        if (actual != expected) {
            cs43l22_error.io.default_state = 1;
        }
    }
}

static void load_register_settings(void)
{
    uint8_t address[14]  = {
        CLOCKING_CONTROL,
		INTERFACE_CONTROL_1,
		PASSTHROUGH_A_SELECT,
		PASSTHROUGH_B_SELECT,
		ANALOG_ZC_AND_SR_SETTINGS,
        MISCELLANEOUS_CONTROL,
		PCMA_VOLUME,
		PCMB_VOLUME,
		MASTER_A_VOLUME,
        MASTER_B_VOLUME,
		HEADPHONE_A_VOLUME,
		HEADPHONE_B_VOLUME,
		SPEAKER_A_VOLUME,
        SPEAKER_B_VOLUME
    };
    
    uint8_t settings[14];
    uint8_t size = sizeof(settings)/sizeof(uint8_t);

    for (uint8_t i = 0; i < size; i++) {
        if (AUDIO_IO_OK != cs43l22_io.read(address[i], &settings[i], 1, true)) {
            cs43l22_error.io.read = 1;
        }
    }

    settings[0]  = 0x81;    // Clocking Control: enable Auto-Detect and MCLKDIV2
    settings[1]  = 0x04;	// Interface Control 1: I2S DACDIF
    settings[2] &= 0xF0;    // Passthrough A Select: No inputs selected
    settings[3] &= 0xF0;    // Passthrough B Select: No inputs selected
    settings[4] &= 0xF0;    // Analog Zero-Cross and Soft-Ramp: disabled
    settings[5] &= 0x3D;    // Miscellaneous Controls: disable PASSTHRUx and digital soft ramp
    settings[5] |= 0x30;    // Miscellaneous Controls: mute analog passthrough
    settings[6]  = 0x00;    // PCMA Volume: 0 dB
    settings[7]  = 0x00;    // PCMB Volume: 0 dB
    settings[8]  = 0x00;    // MSTA Volume: 0 dB
    settings[9]  = 0x00;    // MSTB Volume: 0 dB
    settings[10]  = 0x00;    // HPA Volume:  0 dB
    settings[11] = 0x00;    // HPB Volume:  0 dB
    settings[12] = 0x01;    // SPKA Volume: Muted
    settings[13] = 0x01;    // SPKB Volume: Muted

    for (uint8_t i = 0; i < size; i++) {
        verified_io_write(address[i], &settings[i]);
    }

    if (0 != cs43l22_error.io.w) {
        cs43l22_error.io.register_settings = 1;
    }
}

static void verified_io_write(const uint8_t address, uint8_t *data)
{
    uint8_t expected = *data;
    uint8_t actual;

    if (AUDIO_IO_OK != cs43l22_io.write(address, data, 1, true)) {
        cs43l22_error.io.write = 1;
    }

    if (AUDIO_IO_OK != cs43l22_io.read(address, &actual, 1, true)) {
        cs43l22_error.io.read = 1;
    }    
    
    if (actual != expected) {
        cs43l22_error.io.verified_write = 1;
    } 
}

static void load_init_sequence(void)
{
    uint8_t address[5] = {0x00, 0x47, 0x32, 0x32, 0x00};
    uint8_t data[5]    = {0x99, 0x80, 0x00, 0x00, 0x00};
    uint8_t size       = 1;

    if (AUDIO_IO_OK != cs43l22_io.read(address[2], &data[2], size, true)) {
        cs43l22_error.io.read = 1;
    }

    data[3] = data[2];
    data[2] |= 0x80;
    data[3] &= 0x7F;

    for (uint8_t i = 0; i < 5; i++) {
        verified_io_write(address[i], &data[i]);
    }

    if (0 != cs43l22_error.io.w) {
        cs43l22_error.io.init_sequence = 1;
    }
}

codec_status_t cs43l22_deinit(void)
{
    power_down(CS43L22_POWER_STATE_RESET);
    codec_status_t retc = (0 != cs43l22_error.io.w) ? (CODEC_DEINIT_ERROR) : (CODEC_OK);
    return retc;
}

static void power_down(cs43l22_power_state_t power_state)
{
    /**< ****************************************************************************************************** */
    /**< 1. Mute the DAC and PWM outputs                                                                      */
    /**< ****************************************************************************************************** */
    uint8_t playback_ctl_2;
    if (AUDIO_IO_OK != cs43l22_io.read(PLAYBACK_CONTROL_2, &playback_ctl_2, 1, true)) {
        cs43l22_error.io.read = 1;
    }

    playback_ctl_2 &= 0x0F;
    playback_ctl_2 |= 0xF0;

    verified_io_write(PLAYBACK_CONTROL_2, &playback_ctl_2);
    /**< ****************************************************************************************************** */

    /**< ****************************************************************************************************** */
    /**< 2. Disable soft ramp and zero cross volume transitions                                                 */
    /**< ****************************************************************************************************** */
    uint8_t analog_zc_and_sr;
    if (AUDIO_IO_OK != cs43l22_io.read(ANALOG_ZC_AND_SR_SETTINGS, &analog_zc_and_sr, 1, true)) {
        cs43l22_error.io.read = 1;
    }

    analog_zc_and_sr &= 0xF0;

    verified_io_write(ANALOG_ZC_AND_SR_SETTINGS, &analog_zc_and_sr);        

    uint8_t misc_ctl;
    if (AUDIO_IO_OK != cs43l22_io.read(MISCELLANEOUS_CONTROL, &misc_ctl, 1, true)) {
        cs43l22_error.io.read = 1;
    }

    misc_ctl &= 0xFC;

    verified_io_write(MISCELLANEOUS_CONTROL, &misc_ctl);
    /**< ****************************************************************************************************** */

    /**< ****************************************************************************************************** */
    /**< 3. Set the "Power Ctl 1" register (0x02) to 0x9F                                                       */
    /**< ****************************************************************************************************** */
    uint8_t power_ctl_1 = 0x9F;
    verified_io_write(POWER_CONTROL_1, &power_ctl_1);
    /**< ****************************************************************************************************** */

    /**< ****************************************************************************************************** */
    /**< 4. Wait at least 100 us                                                                                */
    /**< ****************************************************************************************************** */
    HAL_Delay(1);
    /**< ****************************************************************************************************** */

    /**< ****************************************************************************************************** */
    /**< 5. MCLK may be removed at this time                                                                    */
    /**< ****************************************************************************************************** */
    // ...
    /**< ****************************************************************************************************** */

    /**< ****************************************************************************************************** */
    /**< 6. To achieve the lowest operating quiescent current, bring RESET low.                                 */ 
    /**<    All control port registers will be reset to their default state                                     */
    /**< ****************************************************************************************************** */
    if (CS43L22_POWER_STATE_RESET == power_state) {
        cs43l22_io.reset(AUDIO_IO_RESET_STATE_LOW);
        if (AUDIO_IO_OK != cs43l22_io.deinit()) {
            cs43l22_error.io.deinit = 1;
        }
    }
    /**< ****************************************************************************************************** */

    if (0 != cs43l22_error.io.w) {
        cs43l22_error.io.power_down = 1;
    }
}

codec_status_t cs43l22_set_hw_params(audio_out_ll_hw_params_t *hw_params)
{
    /**< 5. Apply MCLK at the appropriate frequency, as discussed in Section 4.6.   */
    /**<    SCLK may be applied or set to master at any time;                       */ 
    /**<    LRCK may only be applied or set to master while the PDN bit is set to 1 */
    if (NULL == hw_params) {
        cs43l22_error.io.bad_params = 1;
    }

    power_down(CS43L22_POWER_STATE_STANDBY);
    HAL_Delay(1);
    power_up(CS43L22_POWER_STATE_STANDBY);
    HAL_Delay(1);

    codec_status_t retc = (0 != cs43l22_error.io.w) ? (CODEC_HW_PARAMS_ERROR) : (CODEC_OK);
    return retc;
}

codec_status_t cs43l22_play(void)
{
    cs43l22_set_mute(CODEC_MUTE_DISABLE);
    power_up(CS43L22_POWER_STATE_STANDBY);
    codec_status_t retc = (0 != cs43l22_error.io.w) ? (CODEC_PLAY_ERROR) : (CODEC_OK);
    return retc;
}

codec_status_t cs43l22_pause(void)
{
    cs43l22_set_mute(CODEC_MUTE_ENABLE);
    power_down(CS43L22_POWER_STATE_STANDBY);
    codec_status_t retc = (0 != cs43l22_error.io.w) ? (CODEC_PAUSE_ERROR) : (CODEC_OK);
    return retc;
}

codec_status_t cs43l22_resume(void)
{
    cs43l22_set_mute(CODEC_MUTE_DISABLE);
    power_up(CS43L22_POWER_STATE_STANDBY);
    codec_status_t retc = (0 != cs43l22_error.io.w) ? (CODEC_RESUME_ERROR) : (CODEC_OK);
    return retc;
}

codec_status_t cs43l22_stop(void)
{
    cs43l22_set_mute(CODEC_MUTE_ENABLE);
    power_down(CS43L22_POWER_STATE_STANDBY);
    codec_status_t retc = (0 != cs43l22_error.io.w) ? (CODEC_STOP_ERROR) : (CODEC_OK);
    return retc;
}

codec_status_t cs43l22_set_volume(float volume)
{
    float   p       = (111.7 > volume) ? (1.117) : (0.01*volume);
    uint8_t mst_vol = (uint8_t)(204*p + 52.5);
    
    verified_io_write(MASTER_A_VOLUME, &mst_vol);
    verified_io_write(MASTER_B_VOLUME, &mst_vol);

    codec_status_t retc = (0 != cs43l22_error.io.w) ? (CODEC_VOLUME_ERROR) : (CODEC_OK);
    return retc;
}

codec_status_t cs43l22_set_mute(codec_mute_t mute)
{
    uint8_t mst_mute;
    if (AUDIO_IO_OK != cs43l22_io.read(PLAYBACK_CONTROL_1, &mst_mute, 1, true)) {
        cs43l22_error.io.read = 1;
    }

    switch (mute)
    {
        case CODEC_MUTE_ENABLE : {
            mst_mute |= 0x03; // MSTBMUTE = 1, MSTAMUTE = 1
        } break;

        case CODEC_MUTE_DISABLE : {
            mst_mute &= 0xFC; // MSTBMUTE = 0, MSTAMUTE = 0
        } break;

        default : break;
    }

    verified_io_write(PLAYBACK_CONTROL_1, &mst_mute);

    codec_status_t retc = (0 != cs43l22_error.io.w) ? (CODEC_MUTE_ERROR) : (CODEC_OK);
    return retc;
}


