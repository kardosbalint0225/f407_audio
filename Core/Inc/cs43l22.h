#ifndef CS43L22_H
#define CS43L22_H

#include "audio_io.h"

typedef enum {
	CODEC_OK,
	CODEC_INIT_ERROR,
	CODEC_DEINIT_ERROR,
	CODEC_PLAY_ERROR,
	CODEC_PAUSE_ERROR,
	CODEC_RESUME_ERROR,
	CODEC_STOP_ERROR,
	CODEC_VOLUME_ERROR,
	CODEC_MUTE_ERROR,
	CODEC_HW_PARAMS_ERROR,
} codec_status_t;

typedef enum {
	CODEC_MUTE_ENABLE,
	CODEC_MUTE_DISABLE,
} codec_mute_t;

typedef enum {
	CS43L22_POWER_STATE_RESET,
	CS43L22_POWER_STATE_STANDBY,
} cs43l22_power_state_t;

typedef enum {
	CS43L22_CHIP_ADDRESS 				= 0x94U,	/**< I2C address						                        */
													/**< A0 is logical LOW (DGND)			                        */
} cs43l22_chip_address_t;

/**< ************************************************************************************************************** */
/**< CS43L22 Register addresses 																					*/
/**< ************************************************************************************************************** */
typedef enum {
	CS43L22_ID							= 0x01U,	/**< Chip I.D. and Revision Register (Address 01h) (Read Only) 	*/
	POWER_CONTROL_1 					= 0x02U,	/**< Power Control 1 (Address 02h) 								*/
	RESERVED_03H 						= 0x03U,	/**< Reserved 													*/
	POWER_CONTROL_2 					= 0x04U,	/**< Power Control 2 (Address 04h) 								*/
	CLOCKING_CONTROL 					= 0x05U,	/**< Clocking Control (Address 05h) 							*/
	INTERFACE_CONTROL_1 				= 0x06U,	/**< Interface Control 1 (Address 06h) 							*/
	INTERFACE_CONTROL_2 				= 0x07U,	/**< Interface Control 2 (Address 07h) 							*/
	PASSTHROUGH_A_SELECT 				= 0x08U,	/**< Passthrough x Select: PassA (Address 08h) 					*/
	PASSTHROUGH_B_SELECT 				= 0x09U,	/**< Passthrough x Select: PassB (Address 09h) 					*/
	ANALOG_ZC_AND_SR_SETTINGS 			= 0x0AU,	/**< Analog ZC and SR Settings (Address 0Ah) 					*/
	RESERVED_0BH 						= 0x0BU,	/**< Reserved 													*/
	PASSTHROUGH_GANG_CONTROL 			= 0x0CU,	/**< Passthrough Gang Control (Address 0Ch) 					*/
	PLAYBACK_CONTROL_1 					= 0x0DU,	/**< Playback Control 1 (Address 0Dh) 							*/
	MISCELLANEOUS_CONTROL 				= 0x0EU,	/**< Miscellaneous Controls (Address 0Eh) 						*/
	PLAYBACK_CONTROL_2 					= 0x0FU,	/**< Playback Control 2 (Address 0Fh) 							*/
	RESERVED_10H 						= 0x10U,	/**< Reserved 													*/
	RESERVED_11H 						= 0x11U,	/**< Reserved 													*/
	RESERVED_12H 						= 0x12U,	/**< Reserved 													*/
	RESERVED_13H 						= 0x13U,	/**< Reserved 													*/
	PASSTHROUGH_A_VOLUME 				= 0x14U,	/**< Passthrough x Volume: PASSAVOL (Address 14h)  				*/
	PASSTHROUGH_B_VOLUME 				= 0x15U,	/**< Passthrough x Volume: PASSBVOL (Address 15h)     			*/
	RESERVED_16H 						= 0x16U,	/**< Reserved 													*/
	RESERVED_17H 						= 0x17U,	/**< Reserved 													*/
	RESERVED_18H 						= 0x18U,	/**< Reserved 													*/
	RESERVED_19H 						= 0x19U,	/**< Reserved 													*/
	PCMA_VOLUME 						= 0x1AU,	/**< PCMx Volume: PCMA (Address 1Ah)  							*/
	PCMB_VOLUME 						= 0x1BU,	/**< PCMx Volume: PCMB (Address 1Bh) 							*/
	BEEP_FREQUENCY_ON_TIME 				= 0x1CU,	/**< Beep Frequency & On Time (Address 1Ch) 					*/
	BEEP_VOLUME_OFF_TIME 				= 0x1DU,	/**< Beep Volume & Off Time (Address 1Dh) 						*/
	BEEP_TONE_CONFIG 					= 0x1EU,	/**< Beep & Tone Configuration (Address 1Eh) 					*/
	TONE_CONTROL 						= 0x1FU,	/**< Tone Control (Address 1Fh) 								*/
	MASTER_A_VOLUME 					= 0x20U,	/**< Master Volume Control: MSTA (Address 20h)  				*/
	MASTER_B_VOLUME 					= 0x21U,	/**< Master Volume Control: MSTB (Address 21h) 					*/
	HEADPHONE_A_VOLUME 					= 0x22U,	/**< Headphone Volume Control: HPA (Address 22h)  				*/
	HEADPHONE_B_VOLUME 					= 0x23U,	/**< Headphone Volume Control: HPB (Address 23h) 				*/
	SPEAKER_A_VOLUME 					= 0x24U,	/**< Speaker Volume Control: SPKA (Address 24h) 				*/
	SPEAKER_B_VOLUME 					= 0x25U,	/**< Speaker Volume Control: SPKB (Address 25h) 				*/
	CHANNEL_MIXER_AND_SWAP 				= 0x26U,	/**< PCM Channel Swap (Address 26h) 							*/
	LIMITER_CONTROL_1 					= 0x27U,	/**< Limiter Control 1, Min/Max Thresholds (Address 27h) 		*/
	LIMITER_CONTROL_2 					= 0x28U,	/**< Limiter Control 2, Release Rate (Address 28h) 				*/
	LIMITER_ATTACK_RATE 				= 0x29U,	/**< Limiter Attack Rate (Address 29h) 							*/
	RESERVED_2AH 						= 0x2AU,	/**< Reserved 													*/
	RESERVED_2BH 						= 0x2BU,	/**< Reserved 													*/
	RESERVED_2CH 						= 0x2CU,	/**< Reserved 													*/
	RESERVED_2DH 						= 0x2DU,	/**< Reserved 													*/
	OVERFLOW_AND_CLOCK_STATUS 			= 0x2EU,	/**< Status (Address 2Eh) (Read Only) 							*/
	BATTERY_COMPENSATION 				= 0x2FU,	/**< Battery Compensation (Address 2Fh) 						*/
	VP_BATTERY_LEVEL 					= 0x30U,	/**< VP Battery Level (Address 30h) (Read Only) 				*/
	SPEAKER_STATUS 						= 0x31U,	/**< Speaker Status (Address 31h) (Read Only) 					*/
	RESERVED_32H 						= 0x32U,	/**< Reserved 													*/
	RESERVED_33H 						= 0x33U,	/**< Reserved 													*/
	CHARGE_PUMP_FREQUENCY 				= 0x34U,	/**< Charge Pump Frequency (Address 34h) 						*/
} cs43l22_reg_addr_t;

typedef union {
	struct {
		uint8_t address		: 7;
		uint8_t INCR		: 1;			/**< 5.1.1.1 Map Increment (INCR) */
	};
	uint8_t byte;
} cs43l22_map_t;

typedef union {
	struct {
		uint32_t bad_params        : 1;
		uint32_t init              : 1;
		uint32_t deinit            : 1;
		uint32_t read              : 1;
		uint32_t write             : 1;
		uint32_t default_state     : 1;
		uint32_t verified_write    : 1;
		uint32_t init_sequence     : 1;
		uint32_t register_settings : 1;
		uint32_t power_up          : 1;
		uint32_t power_down        : 1;
		uint32_t 		           : 21;
	};
	uint32_t w;
} cs43l22_io_err_t;

typedef struct {
	cs43l22_io_err_t io;
} cs43l22_err_t;


#ifdef TEST
void cs43l22_error_reset(void);
void cs43l22_get_error(uint32_t *io);
#endif

codec_status_t cs43l22_init(void);
codec_status_t cs43l22_deinit(void);
codec_status_t cs43l22_set_hw_params(audio_out_ll_hw_params_t *hw_params);
codec_status_t cs43l22_play(void);
codec_status_t cs43l22_pause(void);
codec_status_t cs43l22_resume(void);
codec_status_t cs43l22_stop(void);
codec_status_t cs43l22_set_volume(float volume);
codec_status_t cs43l22_set_mute(codec_mute_t mute);


#endif // CS43L22_H
