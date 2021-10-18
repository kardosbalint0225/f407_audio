#include "cs43l22.h"

static cs43l22_err_t cs43l22_error;

static audio_io_if_t cs43l22_io = {
    .init   = NULL,
    .deinit = NULL,
    .read   = NULL,
    .write  = NULL,
};

static audio_out_if_t cs43l22_out = {
    .init   = NULL,
    .deinit = NULL,
    .write  = NULL,
    .pause  = NULL,
    .resume = NULL,
    .stop   = NULL,
};

static const uint8_t cs43l22_default_register_values[52] = {
    CS43L22_ID_DEFAULT,
    POWER_CONTROL_1_DEFAULT,
    RESERVED_03H_DEFAULT,
    POWER_CONTROL_2_DEFAULT,
    CLOCKING_CONTROL_DEFAULT,
    INTERFACE_CONTROL_1_DEFAULT,
    INTERFACE_CONTROL_2_DEFAULT,
    PASSTHROUGH_A_SELECT_DEFAULT,
    PASSTHROUGH_B_SELECT_DEFAULT,
    ANALOG_ZC_AND_SR_SETTINGS_DEFAULT,
    RESERVED_0BH_DEFAULT,
    PASSTHROUGH_GANG_CONTROL_DEFAULT,
    PLAYBACK_CONTROL_1_DEFAULT,
    MISCELLANEOUS_CONTROL_DEFAULT,
    PLAYBACK_CONTROL_2_DEFAULT,
    RESERVED_10H_DEFAULT,
    RESERVED_11H_DEFAULT,
    RESERVED_12H_DEFAULT,
    RESERVED_13H_DEFAULT,
    PASSTHROUGH_A_VOLUME_DEFAULT,
    PASSTHROUGH_B_VOLUME_DEFAULT,
    RESERVED_16H_DEFAULT,
    RESERVED_17H_DEFAULT,
    RESERVED_18H_DEFAULT,
    RESERVED_19H_DEFAULT,
    PCMA_VOLUME_DEFAULT,
    PCMB_VOLUME_DEFAULT,
    BEEP_FREQUENCY_ON_TIME_DEFAULT,
    BEEP_VOLUME_OFF_TIME_DEFAULT,
    BEEP_TONE_CONFIG_DEFAULT,
    TONE_CONTROL_DEFAULT,
    MASTER_A_VOLUME_DEFAULT,
    MASTER_B_VOLUME_DEFAULT,
    HEADPHONE_A_VOLUME_DEFAULT,
    HEADPHONE_B_VOLUME_DEFAULT,
    SPEAKER_A_VOLUME_DEFAULT,
    SPEAKER_B_VOLUME_DEFAULT,
    CHANNEL_MIXER_AND_SWAP_DEFAULT,
    LIMITER_CONTROL_1_DEFAULT,
    LIMITER_CONTROL_2_DEFAULT,
    LIMITER_ATTACK_RATE_DEFAULT,
    RESERVED_2AH_DEFAULT,
    RESERVED_2BH_DEFAULT,
    RESERVED_2CH_DEFAULT,
    RESERVED_2DH_DEFAULT,
    OVERFLOW_AND_CLOCK_STATUS_DEFAULT,
    BATTERY_COMPENSATION_DEFAULT,
    VP_BATTERY_LEVEL_DEFAULT,
    SPEAKER_STATUS_DEFAULT,
    RESERVED_32H_DEFAULT,
    RESERVED_33H_DEFAULT,
    CHARGE_PUMP_FREQUENCY_DEFAULT,
};

void audio_out_write_callback(uint16_t *address, const audio_out_cb_id_t callback_id);

static bool is_audio_io_if_valid(audio_io_if_t *io);
static bool is_audio_out_if_valid(audio_out_if_t *out); 

static cs43l22_status_t chip_id_and_revision(id_revision_reg_t *chip_id);
static cs43l22_status_t power_control_1(power_ctl_1_reg_t *power_ctl);
static cs43l22_status_t power_control_2(power_ctl_2_reg_t *power_ctl);
static cs43l22_status_t clocking_control(clocking_ctl_reg_t *clocking_ctl);
static cs43l22_status_t interface_control_1(interface_ctl_1_reg_t *interface_ctl);
static cs43l22_status_t interface_control_2(interface_ctl_2_reg_t *interface_ctl);
static cs43l22_status_t passthrough_a_select(passthrough_sel_reg_t *passa_sel);
static cs43l22_status_t passthrough_b_select(passthrough_sel_reg_t *passb_sel);
static cs43l22_status_t analog_zc_and_sr_settings(analog_zc_sr_settings_reg_t *analog_zc_sr_settings);
static cs43l22_status_t passthrough_gang_control(passthrough_gang_ctl_reg_t *passthrough_gang_ctl);
static cs43l22_status_t playback_control_1(playback_ctl_1_reg_t *playback_ctl);
static cs43l22_status_t miscellaneous_control(misc_ctl_reg_t *misc_ctl);
static cs43l22_status_t playback_control_2(playback_ctl_2_reg_t *playback_ctl);
static cs43l22_status_t beep_frequency_and_on_time(beep_freq_on_time_reg_t *beep_freq_on_time);
static cs43l22_status_t beep_volume_and_off_time(beep_vol_off_time_reg_t *beep_vol_off_time);
static cs43l22_status_t beep_and_tone_config(beep_tone_config_reg_t *beep_tone_config);
static cs43l22_status_t tone_control(tone_control_reg_t *tone_ctl);
static cs43l22_status_t passthrough_a_volume(uint8_t *passa_vol);
static cs43l22_status_t passthrough_b_volume(uint8_t *passb_vol);
static cs43l22_status_t pcma_volume(pcmx_vol_reg_t *pcma_vol);
static cs43l22_status_t pcmb_volume(pcmx_vol_reg_t *pcmb_vol);
static cs43l22_status_t master_a_volume(uint8_t *msta_vol);
static cs43l22_status_t master_b_volume(uint8_t *mstb_vol);
static cs43l22_status_t headphone_a_volume(uint8_t *hp_a_vol);
static cs43l22_status_t headphone_b_volume(uint8_t *hp_b_vol);
static cs43l22_status_t speaker_a_volume(uint8_t *spkr_a_vol);
static cs43l22_status_t speaker_b_volume(uint8_t *spkr_b_vol);
static cs43l22_status_t channel_mixer_and_swap(channel_mixer_swap_reg_t *ch_mixer_swap);
static cs43l22_status_t limiter_control_1(limiter_ctl_1_reg_t *limiter_ctl);
static cs43l22_status_t limiter_control_2(limiter_ctl_2_reg_t *limiter_ctl);
static cs43l22_status_t limiter_attack_rate(limiter_attack_rate_reg_t *limiter_ctl);
static cs43l22_status_t overflow_and_clock_status(overflow_clock_status_reg_t *ofc_status);
static cs43l22_status_t battery_compensation(battery_comp_reg_t *battery_comp);
static cs43l22_status_t vp_battery_level(uint8_t *battery_level);
static cs43l22_status_t speaker_status(speaker_status_reg_t *spkr_status);
static cs43l22_status_t charge_pump_frequency(charge_pump_freq_reg_t *charge_pump_freq);

static cs43l22_status_t set_power_control_1(power_ctl_1_reg_t *power_ctl);
static cs43l22_status_t set_power_control_2(power_ctl_2_reg_t *power_ctl);
static cs43l22_status_t set_clocking_control(clocking_ctl_reg_t *clocking_ctl);
static cs43l22_status_t set_interface_control_1(interface_ctl_1_reg_t *interface_ctl);
static cs43l22_status_t set_interface_control_2(interface_ctl_2_reg_t *interface_ctl);
static cs43l22_status_t set_passthrough_a_select(passthrough_sel_reg_t *passa_sel);
static cs43l22_status_t set_passthrough_b_select(passthrough_sel_reg_t *passb_sel);
static cs43l22_status_t set_analog_zc_and_sr_settings(analog_zc_sr_settings_reg_t *analog_zc_sr_settings);
static cs43l22_status_t set_passthrough_gang_control(passthrough_gang_ctl_reg_t *passthrough_gang_ctl);
static cs43l22_status_t set_playback_control_1(playback_ctl_1_reg_t *playback_ctl);
static cs43l22_status_t set_miscellaneous_control(misc_ctl_reg_t *misc_ctl);
static cs43l22_status_t set_playback_control_2(playback_ctl_2_reg_t *playback_ctl);
static cs43l22_status_t set_beep_frequency_and_on_time(beep_freq_on_time_reg_t *beep_freq_on_time);
static cs43l22_status_t set_beep_volume_and_off_time(beep_vol_off_time_reg_t *beep_vol_off_time);
static cs43l22_status_t set_beep_and_tone_config(beep_tone_config_reg_t *beep_tone_config);
static cs43l22_status_t set_tone_control(tone_control_reg_t *tone_ctl);
static cs43l22_status_t set_passthrough_a_volume(uint8_t *passa_vol);
static cs43l22_status_t set_passthrough_b_volume(uint8_t *passb_vol);
static cs43l22_status_t set_pcma_volume(pcmx_vol_reg_t *pcma_vol);
static cs43l22_status_t set_pcmb_volume(pcmx_vol_reg_t *pcmb_vol);
static cs43l22_status_t set_master_a_volume(uint8_t *msta_vol);
static cs43l22_status_t set_master_b_volume(uint8_t *mstb_vol);
static cs43l22_status_t set_headphone_a_volume(uint8_t *hp_a_vol);
static cs43l22_status_t set_headphone_b_volume(uint8_t *hp_b_vol);
static cs43l22_status_t set_speaker_a_volume(uint8_t *spkr_a_vol);
static cs43l22_status_t set_speaker_b_volume(uint8_t *spkr_b_vol);
static cs43l22_status_t set_channel_mixer_and_swap(channel_mixer_swap_reg_t *ch_mixer_swap);
static cs43l22_status_t set_limiter_control_1(limiter_ctl_1_reg_t *limiter_ctl);
static cs43l22_status_t set_limiter_control_2(limiter_ctl_2_reg_t *limiter_ctl);
static cs43l22_status_t set_limiter_attack_rate(limiter_attack_rate_reg_t *limiter_ctl);
static cs43l22_status_t set_battery_compensation(battery_comp_reg_t *battery_comp);
static cs43l22_status_t set_charge_pump_frequency(charge_pump_freq_reg_t *charge_pump_freq);


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
            //audio_status_t audio_out_write(uint16_t *data, const uint16_t size);
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

    if ((NULL == io)         || \
        (NULL == io->init)   || \
        (NULL == io->deinit) || \
        (NULL == io->read)   || \
        (NULL == io->write)) {

        retc = false;
    } else {
        retc = true;
    }

    return retc;
}

static bool is_audio_out_if_valid(audio_out_if_t *out)
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

/**< ****************************************************************************************************************************** */
/**<                                                                                                                                */
/**< Register reader/writer functions																								*/
/**<                                                                                                                                */
/**< ****************************************************************************************************************************** */

/**< 7.1	Chip I.D. and Revision Register (Address 01h) (Read Only) 	                                                            */
static cs43l22_status_t chip_id_and_revision(id_revision_reg_t *chip_id)    
{
    cs43l22_map_t map = {
        .address = CS43L22_ID,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &chip_id->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< ****************************************************************************************************************************** */
/**< System control functions	                                                                                                    */	
/**< ****************************************************************************************************************************** */

/**< 7.2	Power Control 1 (Address 02h) 																	                        */
static cs43l22_status_t set_power_control_1(power_ctl_1_reg_t *power_ctl)
{
    cs43l22_map_t map = {
        .address = POWER_CONTROL_1,
        .INCR    = 0,
    };

    audio_status_t status = cs43l22_io.write(map.byte, &power_ctl->byte, 1, false);
	cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t power_control_1(power_ctl_1_reg_t *power_ctl)
{
    cs43l22_map_t map = {
        .address = POWER_CONTROL_1,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &power_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}


/**< 7.3	Power Control 2 (Address 04h) 																	                        */
static cs43l22_status_t set_power_control_2(power_ctl_2_reg_t *power_ctl)
{
    cs43l22_map_t map = {
        .address = POWER_CONTROL_2,
        .INCR    = 0,
    };

    audio_status_t status = cs43l22_io.write(map.byte, &power_ctl->byte, 1, false);
	cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t power_control_2(power_ctl_2_reg_t *power_ctl)
{
    cs43l22_map_t map = {
        .address = POWER_CONTROL_2,
        .INCR    = 0,
    };

    audio_status_t status = cs43l22_io.read(map.byte, &power_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.4	Clocking Control (Address 05h)																	                        */
static cs43l22_status_t set_clocking_control(clocking_ctl_reg_t *clocking_ctl)
{
    cs43l22_map_t map = {
        .address = CLOCKING_CONTROL,
        .INCR    = 0,
    };

    audio_status_t status = cs43l22_io.write(map.byte, &clocking_ctl->byte, 1, false);
	cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t clocking_control(clocking_ctl_reg_t *clocking_ctl)
{
    cs43l22_map_t map = {
        .address = CLOCKING_CONTROL,
        .INCR    = 0,
    };
	audio_status_t status = cs43l22_io.read(map.byte, &clocking_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.5 	Interface Control 1 (Address 06h)																                           */
static cs43l22_status_t set_interface_control_1(interface_ctl_1_reg_t *interface_ctl)
{
    cs43l22_map_t map = {
        .address = INTERFACE_CONTROL_1,
        .INCR    = 0,
    };

    audio_status_t status = cs43l22_io.write(map.byte, &interface_ctl->byte, 1, false);
	cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t interface_control_1(interface_ctl_1_reg_t *interface_ctl)
{
    cs43l22_map_t map = {
        .address = INTERFACE_CONTROL_1,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &interface_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.6 	Interface Control 2 (Address 07h)																                            */
static cs43l22_status_t set_interface_control_2(interface_ctl_2_reg_t *interface_ctl)
{
    cs43l22_map_t map = {
        .address = INTERFACE_CONTROL_2,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &interface_ctl->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t interface_control_2(interface_ctl_2_reg_t *interface_ctl)
{
    cs43l22_map_t map = {
        .address = INTERFACE_CONTROL_2,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &interface_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.7 	Passthrough x Select: PassA (Address 08h), PassB (Address 09h)									                            */
static cs43l22_status_t set_passthrough_a_select(passthrough_sel_reg_t *passa_sel)
{	
    cs43l22_map_t map = {
        .address = PASSTHROUGH_A_SELECT,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &passa_sel->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}	

static cs43l22_status_t passthrough_a_select(passthrough_sel_reg_t *passa_sel)
{	
    cs43l22_map_t map = {
        .address = PASSTHROUGH_A_SELECT,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &passa_sel->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;	
}	

static cs43l22_status_t set_passthrough_b_select(passthrough_sel_reg_t *passb_sel)
{
    cs43l22_map_t map = {
        .address = PASSTHROUGH_B_SELECT,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &passb_sel->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}	

static cs43l22_status_t passthrough_b_select(passthrough_sel_reg_t *passb_sel)
{
    cs43l22_map_t map = {
        .address = PASSTHROUGH_B_SELECT,
        .INCR    = 0,
    };
    
    audio_status_t status = cs43l22_io.read(map.byte, &passb_sel->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;		
}	

/**< 7.8 	Analog ZC and SR Settings (Address 0Ah)															                            */
static cs43l22_status_t set_analog_zc_and_sr_settings(analog_zc_sr_settings_reg_t *analog_zc_sr_settings)
{
    cs43l22_map_t map = {
        .address = ANALOG_ZC_AND_SR_SETTINGS,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &analog_zc_sr_settings->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t analog_zc_and_sr_settings(analog_zc_sr_settings_reg_t *analog_zc_sr_settings)
{
    cs43l22_map_t map = {
        .address = ANALOG_ZC_AND_SR_SETTINGS,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &analog_zc_sr_settings->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.9 	Passthrough Gang Control (Address 0Ch)															                            */
static cs43l22_status_t set_passthrough_gang_control(passthrough_gang_ctl_reg_t *passthrough_gang_ctl)
{
    cs43l22_map_t map = {
        .address = PASSTHROUGH_GANG_CONTROL,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &passthrough_gang_ctl->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t passthrough_gang_control(passthrough_gang_ctl_reg_t *passthrough_gang_ctl)
{
    cs43l22_map_t map = {
        .address = PASSTHROUGH_GANG_CONTROL,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &passthrough_gang_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.10 	Playback Control 1 (Address 0Dh)																                            */
static cs43l22_status_t set_playback_control_1(playback_ctl_1_reg_t *playback_ctl)
{
    cs43l22_map_t map = {
        .address = PLAYBACK_CONTROL_1,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &playback_ctl->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t playback_control_1(playback_ctl_1_reg_t *playback_ctl)
{
    cs43l22_map_t map = {
        .address = PLAYBACK_CONTROL_1,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &playback_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.11 	Miscellaneous Controls (Address 0Eh)															                            */
static cs43l22_status_t set_miscellaneous_control(misc_ctl_reg_t *misc_ctl)
{
    cs43l22_map_t map = {
        .address = MISCELLANEOUS_CONTROL,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &misc_ctl->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t miscellaneous_control(misc_ctl_reg_t *misc_ctl)
{
    cs43l22_map_t map = {
        .address = MISCELLANEOUS_CONTROL,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &misc_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.12 	Playback Control 2 (Address 0Fh)																                            */
static cs43l22_status_t set_playback_control_2(playback_ctl_2_reg_t *playback_ctl)
{
    cs43l22_map_t map = {
        .address = PLAYBACK_CONTROL_2,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &playback_ctl->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}	

static cs43l22_status_t playback_control_2(playback_ctl_2_reg_t *playback_ctl)
{
    cs43l22_map_t map = {
        .address = PLAYBACK_CONTROL_2,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &playback_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}	


/**< ****************************************************************************************************************************** */
/**< Beep generation and Tone control functions                                                                                     */
/**< ****************************************************************************************************************************** */

/**< 7.15 	Beep Frequency & On Time (Address 1Ch)															                        */
static cs43l22_status_t set_beep_frequency_and_on_time(beep_freq_on_time_reg_t *beep_freq_on_time)
{
    cs43l22_map_t map = {
        .address = BEEP_FREQUENCY_ON_TIME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &beep_freq_on_time->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t beep_frequency_and_on_time(beep_freq_on_time_reg_t *beep_freq_on_time)
{
    cs43l22_map_t map = {
        .address = BEEP_FREQUENCY_ON_TIME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &beep_freq_on_time->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.16 	Beep Volume & Off Time (Address 1Dh)															                        */
static cs43l22_status_t set_beep_volume_and_off_time(beep_vol_off_time_reg_t *beep_vol_off_time)
{
    cs43l22_map_t map = {
        .address = BEEP_VOLUME_OFF_TIME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &beep_vol_off_time->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t beep_volume_and_off_time(beep_vol_off_time_reg_t *beep_vol_off_time)
{
    cs43l22_map_t map = {
        .address = BEEP_VOLUME_OFF_TIME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &beep_vol_off_time->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.17 	Beep & Tone Configuration (Address 1Eh)															                        */		
static cs43l22_status_t set_beep_and_tone_config(beep_tone_config_reg_t *beep_tone_config)
{
    cs43l22_map_t map = {
        .address = BEEP_TONE_CONFIG,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &beep_tone_config->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t beep_and_tone_config(beep_tone_config_reg_t *beep_tone_config)
{
    cs43l22_map_t map = {
        .address = BEEP_TONE_CONFIG,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &beep_tone_config->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.18 	Tone Control (Address 1Fh)																		                          */
static cs43l22_status_t set_tone_control(tone_control_reg_t *tone_ctl)
{
    cs43l22_map_t map = {
        .address = TONE_CONTROL,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &tone_ctl->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}     

static cs43l22_status_t tone_control(tone_control_reg_t *tone_ctl)
{
    cs43l22_map_t map = {
        .address = TONE_CONTROL,
        .INCR    = 0,
    };
    
	audio_status_t status = cs43l22_io.read(map.byte, &tone_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}


/**< ****************************************************************************************************************************** */
/**< Volume control functions 	                                                                                                    */	
/**< ****************************************************************************************************************************** */

/**< 7.13 	Passthrough x Volume: PASSAVOL (Address 14h) & PASSBVOL (Address 15h) 							                        */
static cs43l22_status_t set_passthrough_a_volume(uint8_t *passa_vol)
{
    cs43l22_map_t map = {
        .address = PASSTHROUGH_A_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, passa_vol, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t passthrough_a_volume(uint8_t *passa_vol)
{	
    cs43l22_map_t map = {
        .address = PASSTHROUGH_A_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, passa_vol, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t set_passthrough_b_volume(uint8_t *passb_vol)
{	
    cs43l22_map_t map = {
        .address = PASSTHROUGH_B_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, passb_vol, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t passthrough_b_volume(uint8_t *passb_vol)
{	
    cs43l22_map_t map = {
        .address = PASSTHROUGH_B_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, passb_vol, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.14 	PCMx Volume: PCMA (Address 1Ah) & PCMB (Address 1Bh)											                        */
static cs43l22_status_t set_pcma_volume(pcmx_vol_reg_t *pcma_vol)
{
    cs43l22_map_t map = {
        .address = PCMA_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &pcma_vol->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t pcma_volume(pcmx_vol_reg_t *pcma_vol)
{
    cs43l22_map_t map = {
        .address = PCMA_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &pcma_vol->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t set_pcmb_volume(pcmx_vol_reg_t *pcmb_vol)
{
    cs43l22_map_t map = {
        .address = PCMB_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &pcmb_vol->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t pcmb_volume(pcmx_vol_reg_t *pcmb_vol)
{
    cs43l22_map_t map = {
        .address = PCMB_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &pcmb_vol->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.19 	Master Volume Control: MSTA (Address 20h) & MSTB (Address 21h)									                        */
static cs43l22_status_t set_master_a_volume(uint8_t *msta_vol)
{
    cs43l22_map_t map = {
        .address = MASTER_A_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, msta_vol, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t master_a_volume(uint8_t *msta_vol)
{
    cs43l22_map_t map = {
        .address = MASTER_A_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, msta_vol, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t set_master_b_volume(uint8_t *mstb_vol)
{
    cs43l22_map_t map = {
        .address = MASTER_B_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, mstb_vol, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t master_b_volume(uint8_t *mstb_vol)
{
    cs43l22_map_t map = {
        .address = MASTER_B_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, mstb_vol, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.20 	Headphone Volume Control: HPA (Address 22h) & HPB (Address 23h)									                        */
static cs43l22_status_t set_headphone_a_volume(uint8_t *hp_a_vol)
{
    cs43l22_map_t map = {
        .address = HEADPHONE_A_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, hp_a_vol, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;		
}

static cs43l22_status_t headphone_a_volume(uint8_t *hp_a_vol)
{
    cs43l22_map_t map = {
        .address = HEADPHONE_A_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, hp_a_vol, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;		
}

static cs43l22_status_t set_headphone_b_volume(uint8_t *hp_b_vol)
{
    cs43l22_map_t map = {
        .address = HEADPHONE_B_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, hp_b_vol, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t headphone_b_volume(uint8_t *hp_b_vol)
{
    cs43l22_map_t map = {
        .address = HEADPHONE_B_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, hp_b_vol, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.21 	Speaker Volume Control: SPKA (Address 24h) & SPKB (Address 25h)									                        */
static cs43l22_status_t set_speaker_a_volume(uint8_t *spkr_a_vol)
{
    cs43l22_map_t map = {
        .address = SPEAKER_A_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, spkr_a_vol, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
} 	

static cs43l22_status_t speaker_a_volume(uint8_t *spkr_a_vol)
{
    cs43l22_map_t map = {
        .address = SPEAKER_A_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, spkr_a_vol, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
} 		

static cs43l22_status_t set_speaker_b_volume(uint8_t *spkr_b_vol)
{
    cs43l22_map_t map = {
        .address = SPEAKER_B_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, spkr_b_vol, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
} 
	
static cs43l22_status_t speaker_b_volume(uint8_t *spkr_b_vol)
{
    cs43l22_map_t map = {
        .address = SPEAKER_B_VOLUME,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, spkr_b_vol, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.22 	PCM Channel Swap (Address 26h)																	                        */
static cs43l22_status_t set_channel_mixer_and_swap(channel_mixer_swap_reg_t *ch_mixer_swap)
{
    cs43l22_map_t map = {
        .address = CHANNEL_MIXER_AND_SWAP,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &ch_mixer_swap->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
} 

static cs43l22_status_t channel_mixer_and_swap(channel_mixer_swap_reg_t *ch_mixer_swap)
{
    cs43l22_map_t map = {
        .address = CHANNEL_MIXER_AND_SWAP,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &ch_mixer_swap->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
} 


/**< ****************************************************************************************************************************** */
/**< Limiter control functions	                                                                                                    */		
/**< ****************************************************************************************************************************** */

/**< 7.23 	Limiter Control 1, Min/Max Thresholds (Address 27h)												                        */
static cs43l22_status_t set_limiter_control_1(limiter_ctl_1_reg_t *limiter_ctl)
{
    cs43l22_map_t map = {
        .address = LIMITER_CONTROL_1,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &limiter_ctl->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t limiter_control_1(limiter_ctl_1_reg_t *limiter_ctl)
{
    cs43l22_map_t map = {
        .address = LIMITER_CONTROL_1,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &limiter_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.24 	Limiter Control 2, Release Rate (Address 28h)													                        */
static cs43l22_status_t set_limiter_control_2(limiter_ctl_2_reg_t *limiter_ctl)
{
    cs43l22_map_t map = {
        .address = LIMITER_CONTROL_2,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &limiter_ctl->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t limiter_control_2(limiter_ctl_2_reg_t *limiter_ctl)
{
    cs43l22_map_t map = {
        .address = LIMITER_CONTROL_2,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &limiter_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.25 	Limiter Attack Rate (Address 29h)																                        */
static cs43l22_status_t set_limiter_attack_rate(limiter_attack_rate_reg_t *limiter_ctl)
{
    cs43l22_map_t map = {
        .address = LIMITER_ATTACK_RATE,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &limiter_ctl->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}	

static cs43l22_status_t limiter_attack_rate(limiter_attack_rate_reg_t *limiter_ctl)
{
    cs43l22_map_t map = {
        .address = LIMITER_ATTACK_RATE,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &limiter_ctl->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}		


/**< ****************************************************************************************************************************** */ 
/**< System status functions	                                                                                                    */
/**< ****************************************************************************************************************************** */

/**< 7.26 	Status (Address 2Eh) (Read Only)																                        */
static cs43l22_status_t overflow_and_clock_status(overflow_clock_status_reg_t *ofc_status)
{
    cs43l22_map_t map = {
        .address = OVERFLOW_AND_CLOCK_STATUS,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &ofc_status->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.27 	Battery Compensation (Address 2Fh)																                        */
static cs43l22_status_t set_battery_compensation(battery_comp_reg_t *battery_comp)
{
    cs43l22_map_t map = {
        .address = BATTERY_COMPENSATION,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &battery_comp->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t battery_compensation(battery_comp_reg_t *battery_comp)
{
    cs43l22_map_t map = {
        .address = BATTERY_COMPENSATION,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &battery_comp->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.28 	VP Battery Level (Address 30h) (Read Only)														                        */
static cs43l22_status_t vp_battery_level(uint8_t *battery_level)
{
    cs43l22_map_t map = {
        .address = VP_BATTERY_LEVEL,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, battery_level, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.29 	Speaker Status (Address 31h) (Read Only)														                        */
static cs43l22_status_t speaker_status(speaker_status_reg_t *spkr_status)
{
    cs43l22_map_t map = {
        .address = SPEAKER_STATUS,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &spkr_status->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}

/**< 7.30 	Charge Pump Frequency (Address 34h)																                        */
static cs43l22_status_t set_charge_pump_frequency(charge_pump_freq_reg_t *charge_pump_freq)
{
    cs43l22_map_t map = {
        .address = CHARGE_PUMP_FREQUENCY,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.write(map.byte, &charge_pump_freq->byte, 1, false);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_WRITE_ERROR) : (CS43L22_OK);
    return retc;
}

static cs43l22_status_t charge_pump_frequency(charge_pump_freq_reg_t *charge_pump_freq)
{
    cs43l22_map_t map = {
        .address = CHARGE_PUMP_FREQUENCY,
        .INCR    = 0,
    };

	audio_status_t status = cs43l22_io.read(map.byte, &charge_pump_freq->byte, 1, true);
    cs43l22_status_t retc = (AUDIO_IO_OK != status) ? (CS43L22_IO_READ_ERROR) : (CS43L22_OK);
    return retc;
}





cs43l22_status_t cs43l22_init(cs43l22_if_t *interface)
{
    audio_status_t status;
    cs43l22_error.io.w  = 0;
    cs43l22_error.out.w = 0;

    if (false == is_audio_io_if_valid(&interface->io)) {
        cs43l22_error.io.bad_params = 1;
    }

    if (false == is_audio_out_if_valid(&interface->out)) {
        cs43l22_error.out.bad_params = 1;
    }

    cs43l22_io.init    = interface->io.init;  
    cs43l22_io.deinit  = interface->io.deinit;
    cs43l22_io.read    = interface->io.read;
    cs43l22_io.write   = interface->io.write;
    cs43l22_out.init   = interface->out.init; 
    cs43l22_out.deinit = interface->out.deinit;
    cs43l22_out.write  = interface->out.write;
    cs43l22_out.pause  = interface->out.pause;
    cs43l22_out.resume = interface->out.resume;
    cs43l22_out.stop   = interface->out.stop;

    if (AUDIO_IO_OK != cs43l22_io.init()) {
        cs43l22_error.io.init = 1;
    }

    uint8_t cs43l22_register_values[52];
    cs43l22_map_t map = {
        .address = CS43L22_ID,
        .INCR    = 1,
    };

    if (AUDIO_IO_OK != cs43l22_io.read(map.byte, cs43l22_register_values, sizeof(cs43l22_register_values)/sizeof(uint8_t), true)) {
        cs43l22_error.io.read = 1;
    }

}

cs43l22_status_t cs43l22_set_hw_params(audio_out_ll_hw_params_t *haout)
{
    
}


////////////////////////////////////////////////////////////////////////////////////////////////////

/**< ****************************************************************************************************** */
/**< 4.2.1 Beep Generator																					*/
/**< ****************************************************************************************************** */

const int8_t beep_generator_volume[32] = { 
	 -6,  -4,  -2,   0,   2,   4,   6, -56, -54, -52, -50, 
	-48, -46, -44, -42, -40, -38, -36, -34, -32, -30, -28, 
	-26, -24, -22, -20, -18, -16, -14, -12, -10, -8 
};

cs43l22_status_t beep_generator_init(beep_generator_t *beep)
{
    audio_out_ll_hw_params_t hw_params = {
        .standard        = AUDIO_OUT_STANDARD_LEFT_JUSTIFIED,
        .data_format     = AUDIO_OUT_DATAFORMAT_16B,
        .audio_frequency = 48000,
    };

    audio_status_t status = audio_out_init(&hw_params);
}

cs43l22_status_t beep_generator_deinit(void)
{

}


