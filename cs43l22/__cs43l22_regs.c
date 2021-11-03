/**< ****************************************************************************************************************************** */
/**<                                                                                                                                */
/**< Register reader/writer functions																								*/
/**<                                                                                                                                */
/**< ****************************************************************************************************************************** */
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

/**< ****************************************************************************************************** */
/**< Registers																								*/
/**< ****************************************************************************************************** */

/**< 7.1	Chip I.D. and Revision Register (Address 01h) (Read Only) 										*/
typedef union { 
	struct {
		uint8_t REVID		: 3;			/**< 7.1.2	Chip Revision (Read Only) 							*/
		uint8_t CHIPID 		: 5;			/**< 7.1.1	Chip I.D. (Read Only) 								*/
	};
	uint8_t byte;
} id_revision_reg_t;

/**< 7.2	Power Control 1 (Address 02h) 																	*/
typedef union { 
	struct {
		uint8_t PDN			: 8;			/**< 7.2.1	Power Down 											*/
	};
	uint8_t byte;
} power_ctl_1_reg_t;

/**< 7.3	Power Control 2 (Address 04h) 																	*/
typedef union {
	struct {		
		uint8_t PDN_SPKA	: 2;			/**< Speaker A Power Status										*/
		uint8_t PDN_SPKB	: 2;			/**< Speaker B Power Status										*/
		uint8_t PDN_HPA		: 2;			/**< Headphone A Power Status									*/
		uint8_t PDN_HPB		: 2;			/**< Headphone B Power Status									*/
	};
	uint8_t byte;
} power_ctl_2_reg_t;

/**< 7.4	Clocking Control (Address 05h)																	*/
typedef union {
	struct {
		uint8_t MCLKDIV2	: 1;			/**< 7.4.6	MCLK Divide By 2 									*/
		uint8_t RATIO		: 2;			/**< 7.4.5	Internal MCLK/LRCK Ratio 							*/
		uint8_t VIDEOCLK	: 1;			/**< 7.4.4	27 MHz Video Clock 									*/
		uint8_t GROUP_32k	: 1;			/**< 7.4.3	32kHz Sample Rate Group 							*/
		uint8_t SPEED		: 2;			/**< 7.4.2	Speed Mode 											*/
		uint8_t AUTO		: 1;			/**< 7.4.1	Auto-Detect 										*/
	};
	uint8_t byte;
} clocking_ctl_reg_t;

/**< 7.5 	Interface Control 1 (Address 06h)																*/
typedef union {
	struct {
		uint8_t AWL			: 2;			/**< 7.5.5 Audio Word Length 									*/
		uint8_t DACDIF		: 2;			/**< 7.5.4 DAC Interface Format									*/
		uint8_t DSP			: 1;			/**< 7.5.3 DSP Mode												*/
		uint8_t Reserved	: 1;			/**< Reserved													*/
		uint8_t INV_SCLK 	: 1;			/**< 7.5.2 SCLK Polarity										*/
		uint8_t MS			: 1;			/**< 7.5.1 Master/Slave Mode									*/
	};
	uint8_t byte;
} interface_ctl_1_reg_t;

/**< 7.6 	Interface Control 2 (Address 07h)																*/
typedef union {
	struct {
		uint8_t Reserved_3 	: 3;			/**< Reserved													*/
		uint8_t INV_SWCH	: 1;			/**< 7.6.2 Speaker/Headphone Switch Invert 						*/
		uint8_t Reserved_2 	: 2;			/**< Reserved													*/
		uint8_t SCLK_MCLK	: 1;			/**< 7.6.1 SCLK equals MCLK				 						*/
		uint8_t Reserved_1 	: 1;			/**< Reserved													*/
	};
	uint8_t byte;
} interface_ctl_2_reg_t;

/**< 7.7 	Passthrough x Select: PassA (Address 08h), PassB (Address 09h)									*/
typedef union {
	struct {
		uint8_t PASSxSEL	: 4;			/**< 7.7.1 Passthrough Input Channel Mapping					*/
		uint8_t Reserved 	: 4;			/**< Reserved													*/
	};
	uint8_t byte;
} passthrough_sel_reg_t;

/**< 7.8 	Analog ZC and SR Settings (Address 0Ah)															*/
typedef union { 
	struct {
		uint8_t ANLGZCA		: 1;
		uint8_t ANLGSFTA 	: 1;
		uint8_t ANLGZCB		: 1;			/**< 7.8.2 Ch. x Analog Zero Cross								*/	
		uint8_t ANLGSFTB 	: 1;			/**< 7.8.1 Ch. x Analog Soft Ramp								*/
		uint8_t Reserved 	: 4;			/**< Reserved													*/
	};
	uint8_t byte;
} analog_zc_sr_settings_reg_t;

/**< 7.9 	Passthrough Gang Control (Address 0Ch)															*/
typedef union {
	struct {
		uint8_t Reserved 	: 7;
		uint8_t PASSB_A		: 1;			/**< 7.9.1 Passthrough Channel B=A Gang Control					*/	
	};
	uint8_t byte;
} passthrough_gang_ctl_reg_t;

/**< 7.10 	Playback Control 1 (Address 0Dh)																*/
typedef union {
	struct {
		uint8_t MSTAMUTE	: 1;			/**< 7.10.4 Master Playback A Mute								*/
		uint8_t MSTBMUTE	: 1;			/**< 7.10.4 Master Playback B Mute								*/
		uint8_t INV_PCMA	: 1;			/**< 7.10.3 Invert PCM A Signal Polarity						*/
		uint8_t INV_PCMB	: 1;			/**< 7.10.3 Invert PCM B Signal Polarity						*/
		uint8_t PLYBCKB_A	: 1;			/**< 7.10.2 Playback Volume Setting B=A							*/
		uint8_t HPGAIN		: 3;			/**< 7.10.1 Headphone Analog Gain								*/
	};
	uint8_t byte;
} playback_ctl_1_reg_t;

/**< 7.11 	Miscellaneous Controls (Address 0Eh)															*/
typedef union {
	struct {
		uint8_t DIGZC		: 1;			/**< 7.11.6 Digital Zero Cross									*/
		uint8_t DIGSFT		: 1;			/**< 7.11.5 Digital Soft Ramp									*/
		uint8_t DEEMPH		: 1;			/**< 7.11.4 HP/Speaker De-Emphasis								*/
		uint8_t FREEZE		: 1;			/**< 7.11.3 Freeze Registers									*/
		uint8_t PASSAMUTE	: 1;			/**< 7.11.2 Passthrough Mute A									*/
		uint8_t PASSBMUTE	: 1;			/**< 7.11.2 Passthrough Mute B									*/
		uint8_t PASSTHRUA	: 1;			/**< 7.11.1 Passthrough Analog A								*/
		uint8_t PASSTHRUB	: 1;			/**< 7.11.1 Passthrough Analog B								*/
	};
	uint8_t byte;
} misc_ctl_reg_t;

/**< 7.12 	Playback Control 2 (Address 0Fh)																*/
typedef union {
	struct {
		uint8_t MUTE50_50	: 1;			/**< 7.12.6 Speaker Mute 50/50 Control							*/
		uint8_t SPKMONO		: 1;			/**< 7.12.5 Speaker MONO Control								*/
		uint8_t SPKSWAP		: 1;			/**< 7.12.4 Speaker Channel Swap								*/
		uint8_t SPKB_A		: 1;			/**< 7.12.3 Speaker Volume Setting B=A							*/
		uint8_t SPKAMUTE	: 1;			/**< 7.12.2 Speaker Mute A										*/
		uint8_t SPKBMUTE	: 1;			/**< 7.12.2 Speaker Mute B										*/
		uint8_t HPAMUTE		: 1;			/**< 7.12.1 Headphone Mute A									*/
		uint8_t HPBMUTE		: 1;			/**< 7.12.1 Headphone Mute B									*/
	};
	uint8_t byte;
} playback_ctl_2_reg_t;

/**< 7.13 	Passthrough x Volume: PASSAVOL (Address 14h) & PASSBVOL (Address 15h) 							*/

/**< 7.14 	PCMx Volume: PCMA (Address 1Ah) & PCMB (Address 1Bh)											*/
typedef union {
	struct {
		uint8_t PCMxVOL		: 7;			/**< 7.14.2 PCM Channel x Volume								*/
		uint8_t PCMxMUTE	: 1;			/**< 7.14.1 PCM Channel x Mute									*/
	};
	uint8_t byte;
} pcmx_vol_reg_t;

/**< 7.15 	Beep Frequency & On Time (Address 1Ch)															*/
typedef union {
	struct {
		uint8_t ONTIME		: 4;			/**< 7.15.2 Beep On Time										*/
		uint8_t FREQ		: 4;			/**< 7.15.1 Beep Frequency										*/
	};
	uint8_t byte;
} beep_freq_on_time_reg_t; 

/**< 7.16 	Beep Volume & Off Time (Address 1Dh)															*/
typedef union {
	struct {
		uint8_t BEEPVOL	: 5;				/**< 7.16.2 Beep Volume											*/
		uint8_t OFFTIME	: 3;				/**< 7.16.1 Beep Off Time										*/
	};
	uint8_t byte;
} beep_vol_off_time_reg_t;

/**< 7.17 	Beep & Tone Configuration (Address 1Eh)															*/
typedef union {
	struct {
		uint8_t TCEN		: 1;			/**< 7.17.5 Tone Control Enable 								*/
		uint8_t BASSCF		: 2;			/**< 7.17.4 Bass Corner Frequency								*/
		uint8_t TREBCF		: 2;			/**< 7.17.3 Treble Corner Frequency								*/
		uint8_t BEEPMIXDIS	: 1;			/**< 7.17.2 Beep Mix Disable									*/
		uint8_t BEEP		: 2;			/**< 7.17.1 Beep Configuration									*/
	};
	uint8_t byte;
} beep_tone_config_reg_t;

/**< 7.18 	Tone Control (Address 1Fh)																		*/
typedef union {
	struct {
		uint8_t BASS		: 4;			/**< 7.18.2 Bass Gain 											*/
		uint8_t TREB		: 4;			/**< 7.18.1 Treble Gain											*/
	};
	uint8_t byte;
} tone_control_reg_t;

/**< 7.19 	Master Volume Control: MSTA (Address 20h) & MSTB (Address 21h)									*/
/**< 7.20 	Headphone Volume Control: HPA (Address 22h) & HPB (Address 23h)									*/
/**< 7.21 	Speaker Volume Control: SPKA (Address 24h) & SPKB (Address 25h)									*/

/**< 7.22 	PCM Channel Swap (Address 26h)																	*/
typedef union {
	struct {
		uint8_t Reserved	: 4;
		uint8_t PCMBSWP		: 2;
		uint8_t PCMASWP		: 2;			/**< 7.22.1 PCM Channel Swap 									*/
	};
	uint8_t byte;
} channel_mixer_swap_reg_t;

/**< 7.23 	Limiter Control 1, Min/Max Thresholds (Address 27h)												*/
typedef union {
	struct {
		uint8_t LIMZCDIS	: 1;			/**< 7.23.4 Limiter Zero Cross Disable							*/
		uint8_t LIMSRDIS	: 1;			/**< 7.23.3 Limiter Soft Ramp Disable							*/
		uint8_t CUSH		: 3;			/**< 7.23.2 Limiter Cushion Threshold 							*/
		uint8_t LMAX		: 3;			/**< 7.23.1 Limiter Maximum Threshold							*/
	};
	uint8_t byte;
} limiter_ctl_1_reg_t;

/**< 7.24 	Limiter Control 2, Release Rate (Address 28h)													*/
typedef union {
	struct {
		uint8_t LIMRRATE	: 6;			/**< 7.24.3 Limiter Release Rate								*/
		uint8_t LIMIT_ALL	: 1;			/**< 7.24.2 Peak Signal Limit All Channels						*/
		uint8_t LIMIT 		: 1;			/**< 7.24.1 Peak Detect and Limiter								*/
	};
	uint8_t byte;
} limiter_ctl_2_reg_t;

/**< 7.25 	Limiter Attack Rate (Address 29h)																*/
typedef union {
	struct {
		uint8_t RATE		: 6;			/**< 7.25.1 Limiter Attack Rate 								*/
		uint8_t Reserved	: 2;			/**< Reserved													*/
	} b;
	uint8_t byte;
} limiter_attack_rate_reg_t;

/**< 7.26 	Status (Address 2Eh) (Read Only)																*/
typedef union {
	struct {
		uint8_t Reserved_2	: 1;			/**< Reserved													*/
		uint8_t PCMBOVFL	: 1;			/**< */                                                 	    
		uint8_t PCMAOVFL	: 1;			/**< 7.26.3 PCMx Overflow (Read Only)							*/
		uint8_t DSPBOVFL	: 1;			/**< */                                                     
		uint8_t DSPAOVFL	: 1;			/**< 7.26.2 DSP Engine Overflow (Read Only)						*/
		uint8_t SPCLKERR	: 1;			/**< 7.26.1 Serial Port Clock Error (Read Only)					*/
		uint8_t Reserved_1	: 1;			/**< Reserved													*/
	};
	uint8_t byte;
} overflow_clock_status_reg_t;

/**< 7.27 	Battery Compensation (Address 2Fh)																*/
typedef union {
	struct {
		uint8_t VPREF		: 4;			/**< 7.27.3 VP Reference										*/
		uint8_t Reserved	: 2;			/**< Reserved													*/
		uint8_t VPMONITOR	: 1;			/**< 7.27.2 VP Monitor											*/
		uint8_t BATTCMP		: 1;			/**< 7.27.1 Battery Compensation								*/
	};
	uint8_t byte;
} battery_comp_reg_t;

/**< 7.28 	VP Battery Level (Address 30h) (Read Only)														*/

/**< 7.29 	Speaker Status (Address 31h) (Read Only)														*/
typedef union {
	struct {
		uint8_t Reserved_2	: 3;			/**< Reserved													*/
		uint8_t SPKR_HP		: 1;			/**< 7.29.2 SPKR/HP Pin Status (Read Only)						*/
		uint8_t SPKBSHRT	: 1;			/**< 7.29.1 Speaker Current Load Status (Read Only)				*/
		uint8_t SPKASHRT	: 1;			/**< 7.29.1 Speaker Current Load Status (Read Only)				*/
		uint8_t Reserved_1	: 2;			/**< Reserved													*/
	};
	uint8_t byte;
} speaker_status_reg_t;

/**< 7.30 	Charge Pump Frequency (Address 34h)																*/
typedef union {
	struct {
		uint8_t Reserved	: 4;			/**< Reserved													*/
		uint8_t FREQ		: 4;			/**< 7.30.1 Charge Pump Frequency								*/
	};
	uint8_t byte;
} charge_pump_freq_reg_t;

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


