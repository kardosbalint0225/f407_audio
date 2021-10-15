#ifndef CS43L22_H
#define CS43L22_H

#include "audio_io.h"

typedef enum {
	CS43L22_CHIP_ADDRESS 				= 0x94U						/**< I2C address						*/
																	/**< A0 is logical LOW (DGND)			*/
} cs43l22_chip_address_t;

/**< ********************************************************************************************************************** */
/**< CS43L22 Register addresses 																							*/
/**< ********************************************************************************************************************** */
typedef enum {
	CS43L22_ID							= 0x01U,			/**< Chip I.D. and Revision Register (Address 01h) (Read Only) 	*/
	POWER_CONTROL_1 					= 0x02U,			/**< Power Control 1 (Address 02h) 								*/
	RESERVED_03H 						= 0x03U,			/**< Reserved 													*/
	POWER_CONTROL_2 					= 0x04U,			/**< Power Control 2 (Address 04h) 								*/
	CLOCKING_CONTROL 					= 0x05U,			/**< Clocking Control (Address 05h) 							*/
	INTERFACE_CONTROL_1 				= 0x06U,			/**< Interface Control 1 (Address 06h) 							*/
	INTERFACE_CONTROL_2 				= 0x07U,			/**< Interface Control 2 (Address 07h) 							*/
	PASSTHROUGH_A_SELECT 				= 0x08U,			/**< Passthrough x Select: PassA (Address 08h) 					*/
	PASSTHROUGH_B_SELECT 				= 0x09U,			/**< Passthrough x Select: PassB (Address 09h) 					*/
	ANALOG_ZC_AND_SR_SETTINGS 			= 0x0AU,			/**< Analog ZC and SR Settings (Address 0Ah) 					*/
	RESERVED_0BH 						= 0x0BU,			/**< Reserved 													*/
	PASSTHROUGH_GANG_CONTROL 			= 0x0CU,			/**< Passthrough Gang Control (Address 0Ch) 					*/
	PLAYBACK_CONTROL_1 					= 0x0DU,			/**< Playback Control 1 (Address 0Dh) 							*/
	MISCELLANEOUS_CONTROL 				= 0x0EU,			/**< Miscellaneous Controls (Address 0Eh) 						*/
	PLAYBACK_CONTROL_2 					= 0x0FU,			/**< Playback Control 2 (Address 0Fh) 							*/
	RESERVED_10H 						= 0x10U,			/**< Reserved 													*/
	RESERVED_11H 						= 0x11U,			/**< Reserved 													*/
	RESERVED_12H 						= 0x12U,			/**< Reserved 													*/
	RESERVED_13H 						= 0x13U,			/**< Reserved 													*/
	PASSTHROUGH_A_VOLUME 				= 0x14U,			/**< Passthrough x Volume: PASSAVOL (Address 14h)  				*/
	PASSTHROUGH_B_VOLUME 				= 0x15U,			/**< Passthrough x Volume: PASSBVOL (Address 15h)     			*/
	RESERVED_16H 						= 0x16U,			/**< Reserved 													*/
	RESERVED_17H 						= 0x17U,			/**< Reserved 													*/
	RESERVED_18H 						= 0x18U,			/**< Reserved 													*/
	RESERVED_19H 						= 0x19U,			/**< Reserved 													*/
	PCMA_VOLUME 						= 0x1AU,			/**< PCMx Volume: PCMA (Address 1Ah)  							*/
	PCMB_VOLUME 						= 0x1BU,			/**< PCMx Volume: PCMB (Address 1Bh) 							*/
	BEEP_FREQUENCY_ON_TIME 				= 0x1CU,			/**< Beep Frequency & On Time (Address 1Ch) 					*/
	BEEP_VOLUME_OFF_TIME 				= 0x1DU,			/**< Beep Volume & Off Time (Address 1Dh) 						*/
	BEEP_TONE_CONFIG 					= 0x1EU,			/**< Beep & Tone Configuration (Address 1Eh) 					*/
	TONE_CONTROL 						= 0x1FU,			/**< Tone Control (Address 1Fh) 								*/
	MASTER_A_VOLUME 					= 0x20U,			/**< Master Volume Control: MSTA (Address 20h)  				*/
	MASTER_B_VOLUME 					= 0x21U,			/**< Master Volume Control: MSTB (Address 21h) 					*/
	HEADPHONE_A_VOLUME 					= 0x22U,			/**< Headphone Volume Control: HPA (Address 22h)  				*/
	HEADPHONE_B_VOLUME 					= 0x23U,			/**< Headphone Volume Control: HPB (Address 23h) 				*/
	SPEAKER_A_VOLUME 					= 0x24U,			/**< Speaker Volume Control: SPKA (Address 24h) 				*/
	SPEAKER_B_VOLUME 					= 0x25U,			/**< Speaker Volume Control: SPKB (Address 25h) 				*/
	CHANNEL_MIXER_AND_SWAP 				= 0x26U,			/**< PCM Channel Swap (Address 26h) 							*/
	LIMITER_CONTROL_1 					= 0x27U,			/**< Limiter Control 1, Min/Max Thresholds (Address 27h) 		*/
	LIMITER_CONTROL_2 					= 0x28U,			/**< Limiter Control 2, Release Rate (Address 28h) 				*/
	LIMITER_ATTACK_RATE 				= 0x29U,			/**< Limiter Attack Rate (Address 29h) 							*/
	RESERVED_2AH 						= 0x2AU,			/**< Reserved 													*/
	RESERVED_2BH 						= 0x2BU,			/**< Reserved 													*/
	RESERVED_2CH 						= 0x2CU,			/**< Reserved 													*/
	RESERVED_2DH 						= 0x2DU,			/**< Reserved 													*/
	OVERFLOW_AND_CLOCK_STATUS 			= 0x2EU,			/**< Status (Address 2Eh) (Read Only) 							*/
	BATTERY_COMPENSATION 				= 0x2FU,			/**< Battery Compensation (Address 2Fh) 						*/
	VP_BATTERY_LEVEL 					= 0x30U,			/**< VP Battery Level (Address 30h) (Read Only) 				*/
	SPEAKER_STATUS 						= 0x31U,			/**< Speaker Status (Address 31h) (Read Only) 					*/
	RESERVED_32H 						= 0x32U,			/**< Reserved 													*/
	RESERVED_33H 						= 0x33U,			/**< Reserved 													*/
	CHARGE_PUMP_FREQUENCY 				= 0x34U				/**< Charge Pump Frequency (Address 34h) 						*/
} cs43l22_reg_addr_t;

/**< ********************************************************************************************************************** */
/**< CS43L22 Default Register values 																						*/
/**< ********************************************************************************************************************** */
typedef enum {
	CS43L22_ID_DEFAULT					= 0xE0U,			/**< Chip I.D. and Revision Register (Address 01h) (Read Only) 	*/
	POWER_CONTROL_1_DEFAULT 			= 0x01U,			/**< Power Control 1 (Address 02h) 								*/
	RESERVED_03H_DEFAULT 				= 0x07U,			/**< Reserved 													*/
	POWER_CONTROL_2_DEFAULT 			= 0x05U,			/**< Power Control 2 (Address 04h) 								*/
	CLOCKING_CONTROL_DEFAULT 			= 0xA0U,			/**< Clocking Control (Address 05h) 							*/
	INTERFACE_CONTROL_1_DEFAULT 		= 0x00U,			/**< Interface Control 1 (Address 06h) 							*/
	INTERFACE_CONTROL_2_DEFAULT 		= 0x00U,			/**< Interface Control 2 (Address 07h) 							*/
	PASSTHROUGH_A_SELECT_DEFAULT 		= 0x81U,			/**< Passthrough x Select: PassA (Address 08h) 					*/
	PASSTHROUGH_B_SELECT_DEFAULT 		= 0x81U,			/**< Passthrough x Select: PassB (Address 09h) 					*/
	ANALOG_ZC_AND_SR_SETTINGS_DEFAULT 	= 0xA5U,			/**< Analog ZC and SR Settings (Address 0Ah) 					*/
	RESERVED_0BH_DEFAULT 				= 0x00U,			/**< Reserved 													*/
	PASSTHROUGH_GANG_CONTROL_DEFAULT 	= 0x00U,			/**< Passthrough Gang Control (Address 0Ch) 					*/
	PLAYBACK_CONTROL_1_DEFAULT 			= 0x60U,			/**< Playback Control 1 (Address 0Dh) 							*/
	MISCELLANEOUS_CONTROL_DEFAULT 		= 0x02U,			/**< Miscellaneous Controls (Address 0Eh) 						*/
	PLAYBACK_CONTROL_2_DEFAULT 			= 0x00U,			/**< Playback Control 2 (Address 0Fh) 							*/
	RESERVED_10H_DEFAULT 				= 0x00U,			/**< Reserved 													*/
	RESERVED_11H_DEFAULT 				= 0x00U,			/**< Reserved 													*/
	RESERVED_12H_DEFAULT 				= 0x00U,			/**< Reserved 													*/
	RESERVED_13H_DEFAULT 				= 0x00U,			/**< Reserved 													*/
	PASSTHROUGH_A_VOLUME_DEFAULT 		= 0x00U,			/**< Passthrough x Volume: PASSAVOL (Address 14h)  				*/
	PASSTHROUGH_B_VOLUME_DEFAULT 		= 0x00U,			/**< Passthrough x Volume: PASSBVOL (Address 15h)     			*/
	RESERVED_16H_DEFAULT 				= 0x00U,			/**< Reserved 													*/
	RESERVED_17H_DEFAULT 				= 0x00U,			/**< Reserved 													*/
	RESERVED_18H_DEFAULT 				= 0x80U,			/**< Reserved 													*/
	RESERVED_19H_DEFAULT 				= 0x80U,			/**< Reserved 													*/
	PCMA_VOLUME_DEFAULT 				= 0x00U,			/**< PCMx Volume: PCMA (Address 1Ah)  							*/
	PCMB_VOLUME_DEFAULT 				= 0x00U,			/**< PCMx Volume: PCMB (Address 1Bh) 							*/
	BEEP_FREQUENCY_ON_TIME_DEFAULT 		= 0x00U,			/**< Beep Frequency & On Time (Address 1Ch) 					*/
	BEEP_VOLUME_OFF_TIME_DEFAULT 		= 0x00U,			/**< Beep Volume & Off Time (Address 1Dh) 						*/
	BEEP_TONE_CONFIG_DEFAULT 			= 0x00U,			/**< Beep & Tone Configuration (Address 1Eh) 					*/
	TONE_CONTROL_DEFAULT 				= 0x88U,			/**< Tone Control (Address 1Fh) 								*/
	MASTER_A_VOLUME_DEFAULT 			= 0x00U,			/**< Master Volume Control: MSTA (Address 20h)  				*/
	MASTER_B_VOLUME_DEFAULT 			= 0x00U,			/**< Master Volume Control: MSTB (Address 21h) 					*/
	HEADPHONE_A_VOLUME_DEFAULT 			= 0x00U,			/**< Headphone Volume Control: HPA (Address 22h)  				*/
	HEADPHONE_B_VOLUME_DEFAULT 			= 0x00U,			/**< Headphone Volume Control: HPB (Address 23h) 				*/
	SPEAKER_A_VOLUME_DEFAULT 			= 0x00U,			/**< Speaker Volume Control: SPKA (Address 24h) 				*/
	SPEAKER_B_VOLUME_DEFAULT 			= 0x00U,			/**< Speaker Volume Control: SPKB (Address 25h) 				*/
	CHANNEL_MIXER_AND_SWAP_DEFAULT 		= 0x00U,			/**< PCM Channel Swap (Address 26h) 							*/
	LIMITER_CONTROL_1_DEFAULT 			= 0x00U,			/**< Limiter Control 1, Min/Max Thresholds (Address 27h) 		*/
	LIMITER_CONTROL_2_DEFAULT 			= 0x7FU,			/**< Limiter Control 2, Release Rate (Address 28h) 				*/
	LIMITER_ATTACK_RATE_DEFAULT 		= 0x00U,			/**< Limiter Attack Rate (Address 29h) 							*/
	RESERVED_2AH_DEFAULT 				= 0x00U,			/**< Reserved 													*/
	RESERVED_2BH_DEFAULT 				= 0x3FU,			/**< Reserved 													*/
	RESERVED_2CH_DEFAULT 				= 0x00U,			/**< Reserved 													*/
	RESERVED_2DH_DEFAULT 				= 0x00U,			/**< Reserved 													*/
	OVERFLOW_AND_CLOCK_STATUS_DEFAULT 	= 0x00U,			/**< Status (Address 2Eh) (Read Only) 							*/
	BATTERY_COMPENSATION_DEFAULT 		= 0x00U,			/**< Battery Compensation (Address 2Fh) 						*/
	VP_BATTERY_LEVEL_DEFAULT 			= 0x00U,			/**< VP Battery Level (Address 30h) (Read Only) 				*/
	SPEAKER_STATUS_DEFAULT 				= 0x00U,			/**< Speaker Status (Address 31h) (Read Only) 					*/
	RESERVED_32H_DEFAULT 				= 0x3BU,			/**< Reserved 													*/
	RESERVED_33H_DEFAULT 				= 0x00U,			/**< Reserved 													*/
	CHARGE_PUMP_FREQUENCY_DEFAULT 		= 0x5FU				/**< Charge Pump Frequency (Address 34h) 						*/
} cs43l22_default_reg_val_t;

/**< ****************************************************************************************************** */
/**< 5.1.1 Memory Address Pointer (MAP)																		*/
/**< ****************************************************************************************************** */
typedef union {
	struct {
		uint8_t address		: 7;
		uint8_t INCR		: 1;			/**< 5.1.1.1 Map Increment (INCR)								*/
	};
	uint8_t byte;
} cs43l22_map_t;

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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**< ****************************************************************************************************** */
/**< 4.2.1 Beep Generator																					*/
/**< ****************************************************************************************************** */
typedef struct {
	uint32_t frequency;
	uint32_t on_time;
	uint32_t off_time;
	uint32_t volume;
	uint32_t occurence;
	uint32_t mix_disable;
} beep_generator_t;

/**< 4.2.1.1 frequency																						*/
typedef enum {
	BEEP_GENERATOR_FREQUENCY_C4_PITCH  = 0x00U,
	BEEP_GENERATOR_FREQUENCY_C5_PITCH  = 0x10U,
	BEEP_GENERATOR_FREQUENCY_D5_PITCH  = 0x20U,
	BEEP_GENERATOR_FREQUENCY_E5_PITCH  = 0x30U,
	BEEP_GENERATOR_FREQUENCY_F5_PITCH  = 0x40U,
	BEEP_GENERATOR_FREQUENCY_G5_PITCH  = 0x50U,
	BEEP_GENERATOR_FREQUENCY_A5_PITCH  = 0x60U,
	BEEP_GENERATOR_FREQUENCY_B5_PITCH  = 0x70U,
	BEEP_GENERATOR_FREQUENCY_C6_PITCH  = 0x80U,
	BEEP_GENERATOR_FREQUENCY_D6_PITCH  = 0x90U,
	BEEP_GENERATOR_FREQUENCY_E6_PITCH  = 0xA0U,
	BEEP_GENERATOR_FREQUENCY_F6_PITCH  = 0xB0U,
	BEEP_GENERATOR_FREQUENCY_G6_PITCH  = 0xC0U,
	BEEP_GENERATOR_FREQUENCY_A6_PITCH  = 0xD0U,
	BEEP_GENERATOR_FREQUENCY_B6_PITCH  = 0xE0U,
	BEEP_GENERATOR_FREQUENCY_C7_PITCH  = 0xF0U,
} beep_generator_frequency_t;

/**< 4.2.1.2 on time																						*/
typedef enum {
	BEEP_GENERATOR_ON_TIME_86_MS       = 0x00U,
	BEEP_GENERATOR_ON_TIME_430_MS      = 0x01U,
	BEEP_GENERATOR_ON_TIME_780_MS      = 0x02U,
	BEEP_GENERATOR_ON_TIME_1_20_S      = 0x03U,
	BEEP_GENERATOR_ON_TIME_1_50_S      = 0x04U,
	BEEP_GENERATOR_ON_TIME_1_80_S      = 0x05U,
	BEEP_GENERATOR_ON_TIME_2_20_S      = 0x06U,
	BEEP_GENERATOR_ON_TIME_2_50_S      = 0x07U,
	BEEP_GENERATOR_ON_TIME_2_80_S      = 0x08U,
	BEEP_GENERATOR_ON_TIME_3_20_S      = 0x09U,
	BEEP_GENERATOR_ON_TIME_3_50_S      = 0x0AU,
	BEEP_GENERATOR_ON_TIME_3_80_S      = 0x0BU,
	BEEP_GENERATOR_ON_TIME_4_20_S      = 0x0CU,
	BEEP_GENERATOR_ON_TIME_4_50_S      = 0x0DU,
	BEEP_GENERATOR_ON_TIME_4_80_S      = 0x0EU,
	BEEP_GENERATOR_ON_TIME_5_20_S      = 0x0FU,
} beep_generator_on_time_t;

/**< 4.2.1.3 off time																						*/
typedef enum {
	BEEP_GENERATOR_OFF_TIME_1_23_S     = 0x00U,
	BEEP_GENERATOR_OFF_TIME_2_58_S     = 0x20U,
	BEEP_GENERATOR_OFF_TIME_3_90_S     = 0x40U,
	BEEP_GENERATOR_OFF_TIME_5_20_S     = 0x60U,
	BEEP_GENERATOR_OFF_TIME_6_60_S     = 0x80U,
	BEEP_GENERATOR_OFF_TIME_8_05_S     = 0xA0U,
	BEEP_GENERATOR_OFF_TIME_9_35_S     = 0xC0U,
	BEEP_GENERATOR_OFF_TIME_10_8_S     = 0xE0U,
} beep_generator_off_time_t;

/**< 4.2.1.4 volume																							*/
typedef enum {
	BEEP_GENERATOR_VOLUME_STEP_SIZE    = 2,
	BEEP_GENERATOR_VOLUME_MIN          = -56,
	BEEP_GENERATOR_VOLUME_MAX          = 6,
} beep_generator_volume_t;

/**< 4.2.1.5 occurence																						*/
typedef enum {
	BEEP_GENERATOR_OCCURENCE_OFF       = 0x00,
	BEEP_GENERATOR_OCCURENCE_SINGLE    = 0x40,
	BEEP_GENERATOR_OCCURENCE_MULTIPLE  = 0x80,
	BEEP_GENERATOR_OCCURENCE_CONTINOUS = 0xC0,
} beep_generator_occurence_t;

/**< 4.2.1.6 mix disable																					*/
typedef enum {
	BEEP_GENERATOR_MIX_ENABLED         = 0x00,
	BEEP_GENERATOR_MIX_DISABLED        = 0x20,
} beep_generator_mix_disable_t;

/**< ****************************************************************************************************** */

typedef struct {
	audio_io_if_t io;
	audio_out_if_t out;
} cs43l22_if_t;

typedef union {
	struct {
		uint32_t bad_params : 1;
		uint32_t init       : 1;
		uint32_t read       : 1;
		uint32_t 		    : 29;
	};
	uint32_t w;
} cs43l22_io_err_t;

typedef union {
	struct {
		uint32_t bad_params : 1;
		uint32_t 		    : 31;
	};
	uint32_t w;
} cs43l22_out_err_t;

typedef struct {
	cs43l22_io_err_t io;
	cs43l22_out_err_t out;
} cs43l22_err_t;

typedef enum {
	CS43L22_OK,
	CS43L22_INIT_ERROR,
	CS43L22_DEINIT_ERROR,
	CS43L22_IO_READ_ERROR,
	CS43L22_IO_WRITE_ERROR,
} cs43l22_status_t;

typedef struct {
	audio_out_write_callback_t callback;
	uint16_t *m0;
	uint16_t *m1;
} cs43l22_handle_t;

#ifdef TEST
void cs43l22_error_reset(void);
void cs43l22_get_error(uint32_t *io, uint32_t *out);
#endif

cs43l22_status_t cs43l22_init(cs43l22_if_t *interface, cs43l22_handle_t *handle);
cs43l22_status_t cs43l22_deinit(void);
cs43l22_status_t cs43l22_set_hw_params(audio_out_t *haout);

#endif // CS43L22_H
