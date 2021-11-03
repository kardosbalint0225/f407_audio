#include "cs43l22.h"

#ifdef TEST
	#include "hal.h"
#else
	#include "stm32f4xx_hal.h"
#endif

static cs43l22_err_t cs43l22_error;

static audio_io_if_t cs43l22_io = {
    .init         = NULL,
    .deinit       = NULL,
    .read         = NULL,
    .write        = NULL,
    .reset_init   = NULL,
    .reset_deinit = NULL,
    .reset        = NULL,
};

static audio_out_ll_if_t cs43l22_out = {
    .init         = NULL,
    .deinit       = NULL,
    .write        = NULL,
    .pause        = NULL,
    .resume       = NULL,
    .stop         = NULL,
};

static const uint8_t cs43l22_default_register_values[52] = {
    0xE0U,			/**< Chip I.D. and Revision Register (Address 01h) (Read Only) 	*/
	0x01U,			/**< Power Control 1 (Address 02h) 								*/
	0x00U,			/**< Reserved 													*/
	0x05U,			/**< Power Control 2 (Address 04h) 								*/
	0xA0U,			/**< Clocking Control (Address 05h) 							*/
	0x00U,			/**< Interface Control 1 (Address 06h) 							*/
	0x00U,			/**< Interface Control 2 (Address 07h) 							*/
	0x01U,			/**< Passthrough x Select: PassA (Address 08h) 					*/
	0x01U,			/**< Passthrough x Select: PassB (Address 09h) 					*/
	0x05U,			/**< Analog ZC and SR Settings (Address 0Ah) 					*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Passthrough Gang Control (Address 0Ch) 					*/
	0x60U,			/**< Playback Control 1 (Address 0Dh) 							*/
	0x02U,			/**< Miscellaneous Controls (Address 0Eh) 						*/
	0x00U,			/**< Playback Control 2 (Address 0Fh) 							*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Passthrough x Volume: PASSAVOL (Address 14h)  				*/
	0x00U,			/**< Passthrough x Volume: PASSBVOL (Address 15h)     			*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< PCMx Volume: PCMA (Address 1Ah)  							*/
	0x00U,			/**< PCMx Volume: PCMB (Address 1Bh) 							*/
	0x00U,			/**< Beep Frequency & On Time (Address 1Ch) 					*/
	0x00U,			/**< Beep Volume & Off Time (Address 1Dh) 						*/
	0x00U,			/**< Beep & Tone Configuration (Address 1Eh) 					*/
	0x88U,			/**< Tone Control (Address 1Fh) 								*/
	0x00U,			/**< Master Volume Control: MSTA (Address 20h)  				*/
	0x00U,			/**< Master Volume Control: MSTB (Address 21h) 					*/
	0x00U,			/**< Headphone Volume Control: HPA (Address 22h)  				*/
	0x00U,			/**< Headphone Volume Control: HPB (Address 23h) 				*/
	0x00U,			/**< Speaker Volume Control: SPKA (Address 24h) 				*/
	0x00U,			/**< Speaker Volume Control: SPKB (Address 25h) 				*/
	0x00U,			/**< PCM Channel Swap (Address 26h) 							*/
	0x00U,			/**< Limiter Control 1, Min/Max Thresholds (Address 27h) 		*/
	0x7FU,			/**< Limiter Control 2, Release Rate (Address 28h) 				*/
	0x00U,			/**< Limiter Attack Rate (Address 29h) 							*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Status (Address 2Eh) (Read Only) 							*/
	0x00U,			/**< Battery Compensation (Address 2Fh) 						*/
	0x00U,			/**< VP Battery Level (Address 30h) (Read Only) 				*/
	0x00U,			/**< Speaker Status (Address 31h) (Read Only) 					*/
	0x00U,			/**< Reserved 													*/
	0x00U,			/**< Reserved 													*/
	0x50U,			/**< Charge Pump Frequency (Address 34h) 						*/          
};

static const uint8_t cs43l22_default_register_value_masks[52] = {
    0xF8U,          // 0:  revision level varies
    0xFFU,          // 1:
    0x00U,          // 2:  ?  
    0xFFU,          // 3:
    0xFFU,          // 4:
    0xDFU,          // 5:  bit5 reserved 
    0x48U,          // 6:  bit6,3 valid only
    0x0FU,          // 7:  bit7..4 reserved  
    0x0FU,          // 8:  bit7..4 reserved
    0x0FU,          // 9:  bit7..4 reserved
    0x00U,          // 10: ? 
    0x80U,          // 11: bit7 valid only
    0xFFU,          // 12:
    0xFFU,          // 13: 
    0xFFU,          // 14: 
    0x00U,          // 15: ? 
    0x00U,          // 16: ?
    0x00U,          // 17: ?
    0x00U,          // 18: ?
    0xFFU,          // 19:
    0xFFU,          // 20:
    0x00U,          // 21: ?
    0x00U,          // 22: ?
    0x00U,          // 23: ?
    0x00U,          // 24: ?
    0xFFU,          // 25:
    0xFFU,          // 26:
    0xFFU,          // 27:
    0xFFU,          // 28:
    0xFFU,          // 29:
    0xFFU,          // 30:
    0xFFU,          // 31:
    0xFFU,          // 32:
    0xFFU,          // 33:
    0xFFU,          // 34:
    0xFFU,          // 35:
    0xFFU,          // 36:
    0xF0U,          // 37: bit3..0 reserved
    0xFFU,          // 38:
    0xFFU,          // 39:
    0x3FU,          // 40: bit7..6 reserved
    0x00U,          // 41: ?
    0x00U,          // 42: ?
    0x00U,          // 43: ?
    0x00U,          // 44: ?
    0x00U,          // 45: read-only
    0xCFU,          // 46: bit5..4 reserved
    0x00U,          // 47: read-only
    0x00U,          // 48: read-only
    0x00U,          // 49: ?
    0x00U,          // 50: ?
    0xF0U,          // 51: bit3..0 reserved
};

void audio_out_write_callback(uint16_t *address, const audio_out_cb_id_t callback_id);

static bool is_audio_io_if_valid(audio_io_if_t *io);
static bool is_audio_out_if_valid(audio_out_ll_if_t *out); 

uint32_t sfind(const int32_t *array, const uint32_t size, const int32_t value);

static void mask(uint8_t *value, const uint8_t *mask, const uint8_t size);
static bool compare(const uint8_t *a1, const uint8_t *a2, const uint8_t size);
static cs43l22_status_t cs43l22_power_up(cs43l22_power_state_t power_state);
static cs43l22_status_t cs43l22_power_down(cs43l22_power_state_t power_state);
static cs43l22_status_t cs43l22_load_init_sequence(void);
static cs43l22_status_t cs43l22_verify_default_state(void);
static cs43l22_status_t cs43l22_load_register_settings(void);
static bool cs43l22_verified_write(const uint8_t address, uint8_t *data);


#ifdef TEST
void cs43l22_error_reset(void)
{
    cs43l22_error.io.w  = 0;
    cs43l22_error.out.w = 0;
}

void cs43l22_get_error(uint32_t *io, uint32_t *out)
{
    *io  = cs43l22_error.io.w;
    *out = cs43l22_error.out.w;
}

#endif


void audio_out_write_callback(uint16_t *address, const audio_out_cb_id_t callback_id)
{
    switch (callback_id)
    {
        case AUDIO_OUT_TX_HALF_COMPLETE_CB_ID : {
            // read()
        } break;

        case AUDIO_OUT_TX_COMPLETE_CB_ID : {
            // audio_status_t audio_out_ll_write(uint16_t *data, const uint16_t size);
            // read()
        } break;

        default : {
            // error()
        } break;
    }
}

static bool is_audio_io_if_valid(audio_io_if_t *io) 
{
    bool retc;

    if ((NULL == io)               || \
        (NULL == io->init)         || \
        (NULL == io->deinit)       || \
        (NULL == io->read)         || \
        (NULL == io->write)        || \
        (NULL == io->reset_init)   || \
        (NULL == io->reset_deinit) || \
        (NULL == io->reset))       {

        retc = false;
    } else {
        retc = true;
    }

    return retc;
}

static bool is_audio_out_if_valid(audio_out_ll_if_t *out)
{
    bool retc;

    if ((NULL == out)         || \
        (NULL == out->init)   || \
        (NULL == out->deinit) || \
        (NULL == out->write)  || \
        (NULL == out->pause)  || \
        (NULL == out->resume) || \
        (NULL == out->stop))  {
        
        retc = false;
    } else {
        retc = true;
    }

    return retc;
}


cs43l22_status_t cs43l22_init(cs43l22_if_t *interface)
{
    cs43l22_error.io.w  = 0;
    cs43l22_error.out.w = 0;

    if (false == is_audio_io_if_valid(&interface->io)) {
        cs43l22_error.io.bad_params = 1;
    }

    if (false == is_audio_out_if_valid(&interface->out)) {
        cs43l22_error.out.bad_params = 1;
    }

    cs43l22_io.init         = interface->io.init;  
    cs43l22_io.deinit       = interface->io.deinit;
    cs43l22_io.read         = interface->io.read;
    cs43l22_io.write        = interface->io.write;
    cs43l22_io.reset_init   = interface->io.reset_init;
    cs43l22_io.reset_deinit = interface->io.reset_deinit;
    cs43l22_io.reset        = interface->io.reset;
    cs43l22_out.init        = interface->out.init; 
    cs43l22_out.deinit      = interface->out.deinit;
    cs43l22_out.write       = interface->out.write;
    cs43l22_out.pause       = interface->out.pause;
    cs43l22_out.resume      = interface->out.resume;
    cs43l22_out.stop        = interface->out.stop;

    cs43l22_io.reset_init();

    if (CS43L22_OK != cs43l22_power_up(CS43L22_POWER_STATE_RESET)) {
        cs43l22_error.io.power_up = 1;
    }

}

static cs43l22_status_t cs43l22_power_up(cs43l22_power_state_t power_state)
{
    switch (power_state)
    {
        case CS43L22_POWER_STATE_RESET : {

            /**< ****************************************************************************************************** */
            /**< 1. Hold |RESET low until the power supplies are stable.                                                */
            /**< ****************************************************************************************************** */
            cs43l22_io.reset(AUDIO_IO_RESET_STATE_LOW);
            HAL_Delay(1);
            /**< ****************************************************************************************************** */

            /**< ****************************************************************************************************** */
            /**< 2. Bring |RESET high.                                                                                  */
            /**< ****************************************************************************************************** */
            cs43l22_io.reset(AUDIO_IO_RESET_STATE_HIGH);
            HAL_Delay(5);

            if (AUDIO_IO_OK != cs43l22_io.init()) {
                cs43l22_error.io.init = 1;
            }

            if (CS43L22_OK != cs43l22_verify_default_state()) {
                cs43l22_error.io.def_state = 1;
            }
            /**< ****************************************************************************************************** */

            /**< ****************************************************************************************************** */
            /**< 3. The default state of the "Power Ctl. 1" register (0x02) is 0x01.                                    */
            /**<    Load the desired register settings while keeping the "Power Ctl 1" register set to 0x01.            */
            /**< ****************************************************************************************************** */
            if (CS43L22_OK != cs43l22_load_register_settings()) {
                cs43l22_error.io.reg_set = 1;
            }
            /**< ****************************************************************************************************** */

            /**< ****************************************************************************************************** */
            /**< 4. Load the required initialization settings listed in Section 4.11.                                   */
            /**< ****************************************************************************************************** */
            if (CS43L22_OK != cs43l22_load_init_sequence()) {
                cs43l22_error.io.init_seq = 1;
            }
            /**< ****************************************************************************************************** */

        } break;

        case CS43L22_POWER_STATE_STANDBY : {

            /**< ****************************************************************************************************** */
            /**< 6. Set the “Power Ctl 1” register (0x02) to 0x9E                                                       */
            /**< ****************************************************************************************************** */
            uint8_t power_ctl_1 = 0x9E;
            if (AUDIO_IO_OK != cs43l22_io.write(POWER_CONTROL_1, &power_ctl_1, 1, true)) {
                cs43l22_error.io.write = 1;
            }
            /**< ****************************************************************************************************** */

        } break; 

        default : {

        } break;
    }
    
    cs43l22_status_t retc = ((0 != cs43l22_error.io.w) || (0 != cs43l22_error.out.w)) ? (CS43L22_POWER_UP_ERROR) : (CS43L22_OK);

    return retc;
}

static cs43l22_status_t cs43l22_verify_default_state(void)
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

    mask(cs43l22_register_values, cs43l22_default_register_value_masks, size);

    bool retc = compare(cs43l22_default_register_values, cs43l22_register_values, size);
    cs43l22_status_t status = (false == retc) ? (CS43L22_DEFAULT_STATE_ERROR) : (CS43L22_OK);
    return status;
}

static cs43l22_status_t cs43l22_load_register_settings(void)
{
    uint8_t address[13]  = {
        CLOCKING_CONTROL,      PASSTHROUGH_A_SELECT,     PASSTHROUGH_B_SELECT, ANALOG_ZC_AND_SR_SETTINGS, 
        MISCELLANEOUS_CONTROL, PCMA_VOLUME,              PCMB_VOLUME,          MASTER_A_VOLUME,      
        MASTER_B_VOLUME,       HEADPHONE_A_VOLUME,       HEADPHONE_B_VOLUME,   SPEAKER_A_VOLUME,         
        SPEAKER_B_VOLUME
    };
    
    uint8_t settings[13];
    uint8_t size = sizeof(settings)/sizeof(uint8_t);

    for (uint8_t i = 0; i < size; i++) {
        if (AUDIO_IO_OK != cs43l22_io.read(address[i], &settings[i], 1, true)) {
            cs43l22_error.io.read = 1;
        }
    }

    settings[0] |= 0x81;    // Clocking Control: enable Auto-Detect and MCLKDIV2
    settings[1] &= 0xF0;    // Passthrough A Select: No inputs selected
    settings[2] &= 0xF0;    // Passthrough B Select: No inputs selected
    settings[3] &= 0xF0;    // Analog Zero-Cross and Soft-Ramp: disabled
    settings[4] &= 0x3D;    // Miscellaneous Controls: disable PASSTHRUx and digital soft ramp
    settings[4] |= 0x30;    // Miscellaneous Controls: mute analog passthrough
    settings[5]  = 0x00;    // PCMA Volume: 0 dB
    settings[6]  = 0x00;    // PCMB Volume: 0 dB
    settings[7]  = 0x00;    // MSTA Volume: 0 dB
    settings[8]  = 0x00;    // MSTB Volume: 0 dB
    settings[9]  = 0x00;    // HPA Volume:  0 dB
    settings[10] = 0x00;    // HPB Volume:  0 dB
    settings[11] = 0x01;    // SPKA Volume: Muted
    settings[12] = 0x01;    // SPKB Volume: Muted

    for (uint8_t i = 0; i < size; i++) {
        if (true != cs43l22_verified_write(address[i], &settings[i])) {
            cs43l22_error.io.verify = 1;
        }
    }

    cs43l22_status_t retc = (0 != cs43l22_error.io.w) ? (CS43L22_REGISTER_SETTINGS_ERROR) : (CS43L22_OK);

    return retc;
}

static bool cs43l22_verified_write(const uint8_t address, uint8_t *data)
{
    uint8_t expected = *data;
    uint8_t actual;

    if (AUDIO_IO_OK != cs43l22_io.write(address, data, 1, true)) {
        cs43l22_error.io.write = 1;
    }

    if (AUDIO_IO_OK != cs43l22_io.read(address, &actual, 1, true)) {
        cs43l22_error.io.read = 1;
    }    
    
    bool retc = (expected != actual) ? (false) : (true);
    return retc;   
}

static cs43l22_status_t cs43l22_load_init_sequence(void)
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
        if (AUDIO_IO_OK != cs43l22_io.write(address[i], &data[i], size, true)) {
            cs43l22_error.io.write = 1;
        }
    }

    cs43l22_status_t retc = (0 != cs43l22_error.io.w) ? (CS43L22_INIT_SEQUENCE_ERROR) : (CS43L22_OK);

    return retc;
}

static void mask(uint8_t *value, const uint8_t *mask, const uint8_t size)
{
    for (uint8_t i = 0; i < size; i++) {
        value[i] &= mask[i];
    }
}

static bool compare(const uint8_t *a1, const uint8_t *a2, const uint8_t size)
{
    bool retc = true;

    for (uint8_t i = 0; i < size; i++) {
        if (a1[i] != a2[i]) {
            retc = false;
        }
    }

    return retc;
}

static cs43l22_status_t cs43l22_power_down(cs43l22_power_state_t power_state)
{
    /**< ****************************************************************************************************** */
    /**< 1. Mute the DAC’s and PWM outputs                                                                      */
    /**< ****************************************************************************************************** */
    uint8_t playback_ctl_2;
    if (AUDIO_IO_OK != cs43l22_io.read(PLAYBACK_CONTROL_2, &playback_ctl_2, 1, true)) {
        cs43l22_error.io.read = 1;
    }

    playback_ctl_2 &= 0x0F;
    playback_ctl_2 |= 0xF0;

    if (AUDIO_IO_OK != cs43l22_io.write(PLAYBACK_CONTROL_2, &playback_ctl_2, 1, true)) {
        cs43l22_error.io.write = 1;
    }
    /**< ****************************************************************************************************** */

    /**< ****************************************************************************************************** */
    /**< 2. Disable soft ramp and zero cross volume transitions                                                 */
    /**< ****************************************************************************************************** */
    uint8_t analog_zc_and_sr;
    if (AUDIO_IO_OK != cs43l22_io.read(ANALOG_ZC_AND_SR_SETTINGS, &analog_zc_and_sr, 1, true)) {
        cs43l22_error.io.read = 1;
    }

    analog_zc_and_sr &= 0xF0;

    if (AUDIO_IO_OK != cs43l22_io.write(ANALOG_ZC_AND_SR_SETTINGS, &analog_zc_and_sr, 1, true)) {
        cs43l22_error.io.read = 1;
    }

    uint8_t misc_ctl;
    if (AUDIO_IO_OK != cs43l22_io.read(MISCELLANEOUS_CONTROL, &misc_ctl, 1, true)) {
        cs43l22_error.io.read = 1;
    }

    misc_ctl &= 0xFC;

    if (AUDIO_IO_OK != cs43l22_io.write(MISCELLANEOUS_CONTROL, &misc_ctl, 1, true)) {
        cs43l22_error.io.read = 1;
    }
    /**< ****************************************************************************************************** */

    /**< ****************************************************************************************************** */
    /**< 3. Set the "Power Ctl 1" register (0x02) to 0x9F                                                       */
    /**< ****************************************************************************************************** */
    uint8_t power_ctl_1 = 0x9F;
    if (AUDIO_IO_OK != cs43l22_io.write(POWER_CONTROL_1, &power_ctl_1, 1, true)) {
        cs43l22_error.io.write = 1;
    }
    /**< ****************************************************************************************************** */

    /**< ****************************************************************************************************** */
    /**< 4. Wait at least 100 us                                                                                */
    /**< ****************************************************************************************************** */
    HAL_Delay(1);
    /**< ****************************************************************************************************** */

    /**< ****************************************************************************************************** */
    /**< 5. MCLK may be removed at this time                                                                    */
    /**< ****************************************************************************************************** */
    /**< ****************************************************************************************************** */

    /**< ****************************************************************************************************** */
    /**< 6. To achieve the lowest operating quiescent current, bring RESET low.                                 */ 
    /**<    All control port registers will be reset to their default state                                     */
    /**< ****************************************************************************************************** */
    if (CS43L22_POWER_STATE_RESET == power_state) {
        cs43l22_io.reset(AUDIO_IO_RESET_STATE_LOW);
    }
    /**< ****************************************************************************************************** */

    cs43l22_status_t retc = (0 != cs43l22_error.io.w) ? (CS43L22_POWER_DOWN_ERROR) : (CS43L22_OK);

    return retc;
}


cs43l22_status_t cs43l22_set_hw_params(audio_out_ll_hw_params_t *hw_params)
{
    /**< 5. Apply MCLK at the appropriate frequency, as discussed in Section 4.6.   */
    /**<    SCLK may be applied or set to master at any time;                       */ 
    /**<    LRCK may only be applied or set to master while the PDN bit is set to 1 */
    if (NULL == hw_params) {
        cs43l22_error.out.bad_params = 1;
    }

    if (AUDIO_IO_OK != cs43l22_out.init(hw_params)) {
        cs43l22_error.out.init = 1;
    }
}

cs43l22_status_t cs43l22_set_cb_params(audio_out_ll_cb_params_t *cb_params)
{

}

cs43l22_status_t cs43l22_play()
{

}

cs43l22_status_t cs43l22_pause(void)
{
    //uint8_t power_down = 0x01;
    
    //if (AUDIO_IO_OK != cs43l22_io.write(POWER_CONTROL_1, &power_down, 1, true)) {
    //    cs43l22_error.io.write = 1;
    //}

    if (AUDIO_IO_OK != cs43l22_out.pause()) {
        cs43l22_error.out.pause = 1;
    }

    cs43l22_status_t retc = ((0 != cs43l22_error.io.w) || (0 != cs43l22_error.out.w)) ? (CS43L22_PAUSE_ERROR) : (CS43L22_OK);

    return retc;
}

cs43l22_status_t cs43l22_resume(void)
{

}

cs43l22_status_t cs43l22_stop(void)
{

}


/**< ****************************************************************************************************** */
/**< 4.2.1 Beep Generator																					*/
/**< ****************************************************************************************************** */
static const int32_t beep_volume[32] = { 
	 -6,  -4,  -2,   0,   2,   4,   6, -56, -54, -52, -50, 
	-48, -46, -44, -42, -40, -38, -36, -34, -32, -30, -28, 
	-26, -24, -22, -20, -18, -16, -14, -12, -10, -8 
};

cs43l22_status_t beep_generator_init(beep_generator_t *beep)
{
    uint8_t beep_generator_register_config[3];
    uint8_t beep_volume_index;
    uint8_t size = sizeof(beep_generator_register_config)/sizeof(uint8_t);
    audio_status_t status; 

    audio_out_ll_hw_params_t hw_params = {
        .standard        = AUDIO_OUT_STANDARD_LEFT_JUSTIFIED,
        .data_format     = AUDIO_OUT_DATAFORMAT_16B,
        .audio_frequency = 48000,
    };

    cs43l22_map_t beep_map = {
        .address = BEEP_FREQUENCY_ON_TIME,
        .INCR    = 1,
    };
    
    if (NULL == beep) {
        cs43l22_error.out.bad_params = 1;
    }
    
    if (AUDIO_IO_OK != cs43l22_out.init(&hw_params)) {
        cs43l22_error.out.init = 1;
    }
    
    if (AUDIO_IO_OK != cs43l22_io.read(beep_map.byte, beep_generator_register_config, size, true)) {
        cs43l22_error.io.read = 1;
    }

    beep->volume = (beep->volume > 110) ? 110 : beep->volume;   // saturate to avoid invalid gain setting

    int32_t volume = (int32_t)(0.29*((float)beep->volume));
    volume = (-2)*(28 - volume);
    
    beep_volume_index = (uint8_t)sfind(beep_volume, sizeof(beep_volume)/sizeof(int32_t), volume);

    beep_generator_register_config[0] = beep->frequency    | beep->on_time;
    beep_generator_register_config[1] = beep->off_time     | beep_volume_index;
    beep_generator_register_config[2] &= 0x1F;
    beep_generator_register_config[2] |= beep->mix_disable | BEEP_GENERATOR_OCCURENCE_OFF;
    
    if (AUDIO_IO_OK != cs43l22_io.write(beep_map.byte, beep_generator_register_config, size, true)) {
        cs43l22_error.io.write = 1;
    }

    cs43l22_status_t retc = ((0 != cs43l22_error.io.w) || (0 != cs43l22_error.out.w)) ? (CS43L22_BEEP_GENERATOR_INIT_ERROR) : (CS43L22_OK);

    return retc;
}

uint32_t sfind(const int32_t *array, const uint32_t size, const int32_t value)
{
	uint32_t index = 0xFFFFFFFF;

	for (uint32_t i = 0; i < size; i++) {
		if (array[i] == value) {
			index = i;
		}
	}

	return index;
}

// beep_generator_start(occurence)
// beep_generator_stop

cs43l22_status_t beep_generator_deinit(void)
{

}


