#include "cs43l22.h"



const int32_t supported_rates[] = {
	
};

/**< ****************************************************************************************************************************** */
/**< Register reader functions																										*/
/**< ****************************************************************************************************************************** */


/**
  * @brief  Reads chip ID and revision register value from the CS43L22
  * @param  chip_id: where the value is stored 
  * @retval none
  */
static void chip_id_and_revision(id_revision_reg_t *chip_id)
{
	chip_id->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, CS43L22_ID);
}


/**< System control functions	*/	

/**
  * @brief  Reads power control 1 register value from the CS43L22
  * @param  power_ctl: where the value is stored
  * @retval none
  */			
static void power_control_1(power_ctl_1_reg_t *power_ctl)
{
	power_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, POWER_CONTROL_1);
}
	
/**
  * @brief  Reads power control 2 register value from the CS43L22
  * @param  power_ctl: where the value is stored
  * @retval none
  */	
static void power_control_2(power_ctl_2_reg_t *power_ctl)
{
	power_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, POWER_CONTROL_2);
}

/**
  * @brief  Reads clocking control register value from the CS43L22
  * @param  clocking_ctl: where the value is stored
  * @retval none
  */	
static void clocking_control(clocking_ctl_reg_t *clocking_ctl)
{
	clocking_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, CLOCKING_CONTROL);
}

/**
  * @brief  Reads interface control 1 register value from the CS43L22
  * @param  interface_ctl: where the value is stored
  * @retval none
  */			
static void interface_control_1(interface_ctl_1_reg_t *interface_ctl)
{
	interface_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, INTERFACE_CONTROL_1);
}

/**
  * @brief  Reads interface control 2 register value from the CS43L22
  * @param  interface_ctl: where the value is stored
  * @retval none
  */
static void interface_control_2(interface_ctl_2_reg_t *interface_ctl)
{
	interface_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, INTERFACE_CONTROL_2);
}

/**
  * @brief  Reads passthrough a select register value from the CS43L22
  * @param  passa_sel: where the value is stored
  * @retval none
  */
static void passthrough_a_select(passthrough_sel_reg_t *passa_sel)
{	
	passa_sel->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, PASSTHROUGH_A_SELECT);	
}	

/**
  * @brief  Reads passthrough b select register value from the CS43L22
  * @param  passb_sel: where the value is stored
  * @retval none
  */
static void passthrough_b_select(passthrough_sel_reg_t *passb_sel)
{
	passb_sel->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, PASSTHROUGH_B_SELECT);		
}	

/**
  * @brief  Reads analog zero-cross and softramp settings register value from the CS43L22
  * @param  analog_zc_sr_settings: where the value is stored
  * @retval none
  */
static void analog_zc_and_sr_settings(analog_zc_sr_settings_reg_t *analog_zc_sr_settings)
{
	analog_zc_sr_settings->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, ANALOG_ZC_AND_SR_SETTINGS);
}

/**
  * @brief  Reads passthrough gang control register value from the CS43L22
  * @param  passthrough_gang_ctl: where the value is stored
  * @retval none
  */
static void passthrough_gang_control(passthrough_gang_ctl_reg_t *passthrough_gang_ctl)
{
	passthrough_gang_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, PASSTHROUGH_GANG_CONTROL);
}
	
/**
  * @brief  Reads playback control 1 register value from the CS43L22
  * @param  playback_ctl: where the value is stored
  * @retval none
  */	
static void playback_control_1(playback_ctl_1_reg_t *playback_ctl)
{
	playback_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, PLAYBACK_CONTROL_1);
}

/**
  * @brief  Reads playback control 2 register value from the CS43L22
  * @param  playback_ctl: where the value is stored
  * @retval none
  */	
static void playback_control_2(playback_ctl_2_reg_t *playback_ctl)
{
	playback_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, PLAYBACK_CONTROL_2);
}	
			
/**
  * @brief  Reads miscellaneous control register value from the CS43L22
  * @param  misc_ctl: where the value is stored
  * @retval none
  */			
static void miscellaneous_control(misc_ctl_reg_t *misc_ctl)
{
	misc_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, MISCELLANEOUS_CONTROL);
}

		
/**< Volume control functions 	*/	

/**
  * @brief  Reads passthrough a volume register value from the CS43L22
  * @param  passa_vol: where the value is stored
  * @retval none
  */
static void passthrough_a_volume(uint32_t *passa_vol)
{	
	*passa_vol = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, PASSTHROUGH_A_VOLUME);
}

/**
  * @brief  Reads passthrough b volume register value from the CS43L22
  * @param  passb_vol: where the value is stored
  * @retval none
  */
static void passthrough_b_volume(uint32_t *passb_vol)
{	
	*passb_vol = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, PASSTHROUGH_B_VOLUME);
}
	
/**
  * @brief  Reads pcma volume register value from the CS43L22
  * @param  pcma_vol: where the value is stored
  * @retval none
  */	
static void pcma_volume(pcmx_vol_reg_t *pcma_vol)
{
	pcma_vol->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, PCMA_VOLUME);
}

/**
  * @brief  Reads pcmb volume register value from the CS43L22
  * @param  pcmb_vol: where the value is stored
  * @retval none
  */	
static void pcmb_volume(pcmx_vol_reg_t *pcmb_vol)
{
	pcmb_vol->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, PCMB_VOLUME);
}

/**
  * @brief  Reads msta volume register value from the CS43L22
  * @param  msta_vol: where the value is stored
  * @retval none
  */ 		
static void master_a_volume(uint32_t *msta_vol)
{
	*msta_vol = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, MASTER_A_VOLUME);
}

/**
  * @brief  Reads mstb volume register value from the CS43L22
  * @param  mstb_vol: where the value is stored
  * @retval none
  */ 		
static void master_b_volume(uint32_t *mstb_vol)
{
	*mstb_vol = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, MASTER_B_VOLUME);	
}

/**
  * @brief  Reads headphone a volume register value from the CS43L22
  * @param  hp_a_vol: where the value is stored
  * @retval none
  */			 			
static void headphone_a_volume(uint32_t *hp_a_vol)
{
	*hp_a_vol = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, HEADPHONE_A_VOLUME);		
}

/**
  * @brief  Reads headphone b volume register value from the CS43L22
  * @param  hp_b_vol: where the value is stored
  * @retval none
  */			 			
static void headphone_b_volume(uint32_t *hp_b_vol)
{
	*hp_b_vol = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, HEADPHONE_B_VOLUME);
}

/**
  * @brief  Reads speaker a volume register value from the CS43L22
  * @param  spkr_a_vol: where the value is stored
  * @retval none
  */			
static void speaker_a_volume(uint32_t *spkr_a_vol)
{
	*spkr_a_vol = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, SPEAKER_A_VOLUME);
} 						

/**
  * @brief  Reads speaker b volume register value from the CS43L22
  * @param  spkr_b_vol: where the value is stored
  * @retval none
  */			
static void speaker_b_volume(uint32_t *spkr_b_vol)
{
	*spkr_b_vol = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, SPEAKER_B_VOLUME);
}

/**< Beep generation and Tone control functions */

/**
  * @brief  Reads beep frequency and on time register value from the CS43L22
  * @param  beep_freq_on_time: where the value is stored
  * @retval none
  */
static void beep_frequency_and_on_time(beep_freq_on_time_reg_t *beep_freq_on_time)
{
	beep_freq_on_time->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, BEEP_FREQUENCY_ON_TIME);
}

/**
  * @brief  Reads beep volume and off time register value from the CS43L22
  * @param  beep_vol_off_time: where the value is stored
  * @retval none
  */
static void beep_volume_and_off_time(beep_vol_off_time_reg_t *beep_vol_off_time)
{
	beep_vol_off_time->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, BEEP_VOLUME_OFF_TIME);
}

/**
  * @brief  Reads beep and tone config register value from the CS43L22
  * @param  beep_tone_config: where the value is stored
  * @retval none
  */		
static void beep_and_tone_config(beep_tone_config_reg_t *beep_tone_config)
{
	beep_tone_config->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, BEEP_TONE_CONFIG);
}
 			
/**
  * @brief  Reads tone control register value from the CS43L22
  * @param  tone_ctl: where the value is stored
  * @retval none
  */			
static void tone_control(tone_control_reg_t *tone_ctl)
{
	tone_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, TONE_CONTROL);
} 					
  			
/**
  * @brief  Reads channel mixer and swap register value from the CS43L22
  * @param  ch_mixer_swap: where the value is stored
  * @retval none
  */			
static void channel_mixer_and_swap(channel_mixer_swap_reg_t *ch_mixer_swap)
{
	ch_mixer_swap->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, CHANNEL_MIXER_AND_SWAP);
} 


/**< Limiter control functions	*/		

/**
  * @brief  Reads limiter control 1 register value from the CS43L22
  * @param  limiter_ctl: where the value is stored
  * @retval none
  */
static void limiter_control_1(limiter_ctl_1_reg_t *limiter_ctl)
{
	limiter_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, LIMITER_CONTROL_1);
}
 			
/**
  * @brief  Reads limiter control 2 register value from the CS43L22
  * @param  limiter_ctl: where the value is stored
  * @retval none
  */			
static void limiter_control_2(limiter_ctl_2_reg_t *limiter_ctl)
{
	limiter_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, LIMITER_CONTROL_2);
}

/**
  * @brief  Reads limiter attack rate register value from the CS43L22
  * @param  limiter_ctl: where the value is stored
  * @retval none
  */ 			
static void limiter_attack_rate(limiter_attack_rate_reg_t *limiter_ctl)
{
	limiter_ctl->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, LIMITER_ATTACK_RATE);
}		
 
 
/**< System status functions	*/

/**
  * @brief  Reads overflow and clock status register value from the CS43L22
  * @param  status: where the value is stored
  * @retval none
  */
static void overflow_and_clock_status(overflow_clock_status_reg_t *status)
{
	status->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, OVERFLOW_AND_CLOCK_STATUS);
}
 	
/**
  * @brief  Reads battery compensation register value from the CS43L22
  * @param  battery_comp: where the value is stored
  * @retval none
  */	
static void battery_compensation(battery_comp_reg_t *battery_comp)
{
	battery_comp->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, BATTERY_COMPENSATION);
}

/**
  * @brief  Reads battery level register value from the CS43L22
  * @param  battery_level: where the value is stored
  * @retval none
  */ 		
static void vp_battery_level(uint32_t *battery_level)
{
	*battery_level = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, VP_BATTERY_LEVEL);
}

/**
  * @brief  Reads speaker status register value from the CS43L22
  * @param  spkr_status: where the value is stored
  * @retval none
  */
static void speaker_status(speaker_status_reg_t *spkr_status)
{
	spkr_status->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, SPEAKER_STATUS);
}

/**
  * @brief  Reads charge pump frequency register value from the CS43L22
  * @param  charge_pump_freq: where the value is stored
  * @retval none
  */
static void charge_pump_frequency(charge_pump_freq_reg_t *charge_pump_freq)
{
	charge_pump_freq->w = (uint32_t)audio_io_read(CS43L22_CHIP_ADDRESS, CHARGE_PUMP_FREQUENCY);
}


/**< ****************************************************************************************************************************** */
/**< Register writer functions																										*/
/**< ****************************************************************************************************************************** */


/**< System control functions	*/	

/**
  * @brief  Writes power control 1 register value to the CS43L22
  * @param  power_ctl: value to write
  * @retval none
  */			
static void set_power_control_1(power_ctl_1_reg_t *power_ctl)
{
	uint8_t value = (uint8_t)power_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, POWER_CONTROL_1, &value);
}
	
/**
  * @brief  Writes power control 2 register value to the CS43L22
  * @param  power_ctl: value to write
  * @retval none
  */	
static void set_power_control_2(power_ctl_2_reg_t *power_ctl)
{
	uint8_t value = (uint8_t)power_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, POWER_CONTROL_2, &value);
}

/**
  * @brief  Writes clocking control register value to the CS43L22
  * @param  clocking_ctl: value to write
  * @retval none
  */	
static void set_clocking_control(clocking_ctl_reg_t *clocking_ctl)
{
	uint8_t value = (uint8_t)clocking_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, CLOCKING_CONTROL, &value);
}

/**
  * @brief  Writes interface control 1 register value to the CS43L22
  * @param  interface_ctl: value to write
  * @retval none
  */			
static void set_interface_control_1(interface_ctl_1_reg_t *interface_ctl)
{
	uint8_t value = (uint8_t)interface_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, INTERFACE_CONTROL_1, &value);
}

/**
  * @brief  Writes interface control 2 register value to the CS43L22
  * @param  interface_ctl: value to write
  * @retval none
  */
static void set_interface_control_2(interface_ctl_2_reg_t *interface_ctl)
{
	uint8_t value = (uint8_t)interface_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, INTERFACE_CONTROL_2, &value);
}

/**
  * @brief  Writes passthrough a select register value to the CS43L22
  * @param  passa_sel: value to write
  * @retval none
  */
static void set_passthrough_a_select(passthrough_sel_reg_t *passa_sel)
{	
	uint8_t value = (uint8_t)passa_sel->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, PASSTHROUGH_A_SELECT, &value);	
}	

/**
  * @brief  Writes passthrough b select register value to the CS43L22
  * @param  passb_sel: value to write
  * @retval none
  */
static void set_passthrough_b_select(passthrough_sel_reg_t *passb_sel)
{
	uint8_t value = (uint8_t)passb_sel->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, PASSTHROUGH_B_SELECT, &value);		
}	

/**
  * @brief  Writes analog zero-cross and softramp settings register value to the CS43L22
  * @param  analog_zc_sr_settings: value to write
  * @retval none
  */
static void set_analog_zc_and_sr_settings(analog_zc_sr_settings_reg_t *analog_zc_sr_settings)
{
	uint8_t value = (uint8_t)analog_zc_sr_settings->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, ANALOG_ZC_AND_SR_SETTINGS, &value);
}

/**
  * @brief  Writes passthrough gang control register value to the CS43L22
  * @param  passthrough_gang_ctl: value to write
  * @retval none
  */
static void set_passthrough_gang_control(passthrough_gang_ctl_reg_t *passthrough_gang_ctl)
{
	uint8_t value = (uint8_t)passthrough_gang_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, PASSTHROUGH_GANG_CONTROL, &value);
}
	
/**
  * @brief  Writes playback control 1 register value to the CS43L22
  * @param  playback_ctl: value to write
  * @retval none
  */	
static void set_playback_control_1(playback_ctl_1_reg_t *playback_ctl)
{
	uint8_t value = (uint8_t)playback_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, PLAYBACK_CONTROL_1, &value);
}

/**
  * @brief  Writes playback control 2 register value to the CS43L22
  * @param  playback_ctl: value to write
  * @retval none
  */	
static void set_playback_control_2(playback_ctl_2_reg_t *playback_ctl)
{
	uint8_t value = (uint8_t)playback_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, PLAYBACK_CONTROL_2, &value);
}	
			
/**
  * @brief  Writes miscellaneous control register value to the CS43L22
  * @param  misc_ctl: value to write
  * @retval none
  */			
static void set_miscellaneous_control(misc_ctl_reg_t *misc_ctl)
{
	uint8_t value = (uint8_t)misc_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, MISCELLANEOUS_CONTROL, &value);
}

		
/**< Volume control functions 	*/	

/**
  * @brief  Writes passthrough a volume register value to the CS43L22
  * @param  passa_vol: value to write
  * @retval none
  */
static void set_passthrough_a_volume(uint32_t *passa_vol)
{
	uint8_t value = (uint8_t)*passa_vol;	
	audio_io_write(CS43L22_CHIP_ADDRESS, PASSTHROUGH_A_VOLUME, &value);
}

/**
  * @brief  Writes passthrough b volume register value to the CS43L22
  * @param  passb_vol: value to write
  * @retval none
  */
static void set_passthrough_b_volume(uint32_t *passb_vol)
{	
	uint8_t value = (uint8_t)*passb_vol;
	audio_io_write(CS43L22_CHIP_ADDRESS, PASSTHROUGH_B_VOLUME, &value);
}
	
/**
  * @brief  Writes pcma volume register value to the CS43L22
  * @param  pcma_vol: value to write
  * @retval none
  */	
static void set_pcma_volume(pcmx_vol_reg_t *pcma_vol)
{
	uint8_t value = (uint8_t)pcma_vol->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, PCMA_VOLUME, &value);
}

/**
  * @brief  Writes pcmb volume register value to the CS43L22
  * @param  pcmb_vol: value to write
  * @retval none
  */	
static void set_pcmb_volume(pcmx_vol_reg_t *pcmb_vol)
{
	uint8_t value = (uint8_t)pcmb_vol->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, PCMB_VOLUME, &value);
}

/**
  * @brief  Writes msta volume register value to the CS43L22
  * @param  msta_vol: value to write
  * @retval none
  */ 		
static void set_master_a_volume(uint32_t *msta_vol)
{
	uint8_t value = (uint8_t)*msta_vol;
	audio_io_write(CS43L22_CHIP_ADDRESS, MASTER_A_VOLUME, &value);
}

/**
  * @brief  Writes mstb volume register value to the CS43L22
  * @param  mstb_vol: value to write
  * @retval none
  */ 		
static void set_master_b_volume(uint32_t *mstb_vol)
{
	uint8_t value = (uint8_t)*mstb_vol;
	audio_io_write(CS43L22_CHIP_ADDRESS, MASTER_B_VOLUME, &value);	
}

/**
  * @brief  Writes headphone a volume register value to the CS43L22
  * @param  hp_a_vol: value to write
  * @retval none
  */			 			
static void set_headphone_a_volume(uint32_t *hp_a_vol)
{
	uint8_t value = (uint8_t)*hp_a_vol;
	audio_io_write(CS43L22_CHIP_ADDRESS, HEADPHONE_A_VOLUME, &value);		
}

/**
  * @brief  Writes headphone b volume register value to the CS43L22
  * @param  hp_b_vol: value to write
  * @retval none
  */			 			
static void set_headphone_b_volume(uint32_t *hp_b_vol)
{
	uint8_t value = (uint8_t)*hp_b_vol;
	audio_io_write(CS43L22_CHIP_ADDRESS, HEADPHONE_B_VOLUME, &value);
}

/**
  * @brief  Writes speaker a volume register value to the CS43L22
  * @param  spkr_a_vol: value to write
  * @retval none
  */			
static void set_speaker_a_volume(uint32_t *spkr_a_vol)
{
	uint8_t value = (uint8_t)*spkr_a_vol;
	audio_io_write(CS43L22_CHIP_ADDRESS, SPEAKER_A_VOLUME, &value);
} 						

/**
  * @brief  Writes speaker b volume register value to the CS43L22
  * @param  spkr_b_vol: value to write
  * @retval none
  */			
static void set_speaker_b_volume(uint32_t *spkr_b_vol)
{
	uint8_t value = (uint8_t)*spkr_b_vol;
	audio_io_write(CS43L22_CHIP_ADDRESS, SPEAKER_B_VOLUME, &value);
}

/**< Beep generation and Tone control functions */

/**
  * @brief  Writes beep frequency and on time register value to the CS43L22
  * @param  beep_freq_on_time: value to write
  * @retval none
  */
static void set_beep_frequency_and_on_time(beep_freq_on_time_reg_t *beep_freq_on_time)
{
	uint8_t value = (uint8_t)beep_freq_on_time->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, BEEP_FREQUENCY_ON_TIME, &value);
}

/**
  * @brief  Writes beep volume and off time register value to the CS43L22
  * @param  beep_vol_off_time: value to write
  * @retval none
  */
static void set_beep_volume_and_off_time(beep_vol_off_time_reg_t *beep_vol_off_time)
{
	uint8_t value = (uint8_t)beep_vol_off_time->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, BEEP_VOLUME_OFF_TIME, &value);
}

/**
  * @brief  Writes beep and tone config register value to the CS43L22
  * @param  beep_tone_config: value to write
  * @retval none
  */		
static void set_beep_and_tone_config(beep_tone_config_reg_t *beep_tone_config)
{
	uint8_t value = (uint8_t)beep_tone_config->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, BEEP_TONE_CONFIG, &value);
}
 			
/**
  * @brief  Writes tone control register value to the CS43L22
  * @param  tone_ctl: value to write
  * @retval none
  */			
static void set_tone_control(tone_control_reg_t *tone_ctl)
{
	uint8_t value = (uint8_t)tone_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, TONE_CONTROL, &value);
} 					
  			
/**
  * @brief  Writes channel mixer and swap register value to the CS43L22
  * @param  ch_mixer_swap: value to write
  * @retval none
  */			
static void set_channel_mixer_and_swap(channel_mixer_swap_reg_t *ch_mixer_swap)
{
	uint8_t value = (uint8_t)ch_mixer_swap->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, CHANNEL_MIXER_AND_SWAP, &value);
} 


/**< Limiter control functions	*/		

/**
  * @brief  Writes limiter control 1 register value to the CS43L22
  * @param  limiter_ctl: value to write
  * @retval none
  */
static void set_limiter_control_1(limiter_ctl_1_reg_t *limiter_ctl)
{
	uint8_t value = (uint8_t)limiter_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, LIMITER_CONTROL_1, &value);
}
 			
/**
  * @brief  Writes limiter control 2 register value to the CS43L22
  * @param  limiter_ctl: value to write
  * @retval none
  */			
static void set_limiter_control_2(limiter_ctl_2_reg_t *limiter_ctl)
{
	uint8_t value = (uint8_t)limiter_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, LIMITER_CONTROL_2, &value);
}

/**
  * @brief  Writes limiter attack rate register value to the CS43L22
  * @param  limiter_ctl: value to write
  * @retval none
  */ 			
static void set_limiter_attack_rate(limiter_attack_rate_reg_t *limiter_ctl)
{
	uint8_t value = (uint8_t)limiter_ctl->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, LIMITER_ATTACK_RATE, &value);
}		
 
 
/**< System status functions	*/
 	
/**
  * @brief  Writes battery compensation register value to the CS43L22
  * @param  battery_comp: value to write
  * @retval none
  */	
static void set_battery_compensation(battery_comp_reg_t *battery_comp)
{
	uint8_t value = (uint8_t)battery_comp->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, BATTERY_COMPENSATION, &value);
}

/**
  * @brief  Writes charge pump frequency register value to the CS43L22
  * @param  charge_pump_freq: value to write
  * @retval none
  */
static void set_charge_pump_frequency(charge_pump_freq_reg_t *charge_pump_freq)
{
	uint8_t value = (uint8_t)charge_pump_freq->w;
	audio_io_write(CS43L22_CHIP_ADDRESS, CHARGE_PUMP_FREQUENCY, &value);
}


/**< ****************************************************************************************************************************** */
/**< CS43L22 public functions																										*/
/**< ****************************************************************************************************************************** */

/*
cs43l22_deinit()
cs43l22_reset()
cs43l22_play()
cs43l22_pause()
cs43l22_resume()
cs43l22_stop()
*/

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t cs43l22_init(cs43l22_desc_t *desc)
{
	
}

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t cs43l22_set_rate(cs43l22_desc_t *desc, uint32_t *rate)
{
	
}

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t cs43l22_set_rate_near(cs43l22_desc_t *desc, uint32_t *rate)
{
	
}

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t cs43l22_set_format(cs43l22_desc_t *desc, const uint32_t *format)
{
	
}

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t cs43l22_set_channels(cs43l22_desc_t *desc, const uint32_t *channels)
{
	
}

