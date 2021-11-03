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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**< ****************************************************************************************************** */
/**< 4.2.1 Beep Generator																					*/
/**< ****************************************************************************************************** */
typedef struct {
	uint8_t frequency;
	uint8_t on_time;
	uint8_t off_time;
	uint8_t volume;			// 0..110 %, 3.44 % step size in percentages | 0 dB = 100 % .. -56 dB = 0 %, 6 dB = 110 %
	uint8_t mix_disable;
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
typedef enum {
	CS43L22_POWER_STATE_RESET,
	CS43L22_POWER_STATE_STANDBY,
} cs43l22_power_state_t;

typedef struct {
	audio_io_if_t io;
	audio_out_ll_if_t out;
} cs43l22_if_t;

typedef union {
	struct {
		uint32_t bad_params : 1;
		uint32_t init       : 1;
		uint32_t read       : 1;
		uint32_t write      : 1;
		uint32_t def_state  : 1;
		uint32_t verify     : 1;
		uint32_t init_seq   : 1;
		uint32_t reg_set    : 1;
		uint32_t power_up   : 1;
		uint32_t 		    : 23;
	};
	uint32_t w;
} cs43l22_io_err_t;

typedef union {
	struct {
		uint32_t bad_params : 1;
		uint32_t init       : 1;
		uint32_t pause      : 1;
		uint32_t 		    : 29;
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
	CS43L22_INIT_SEQUENCE_ERROR,
	CS43L22_REGISTER_SETTINGS_ERROR,
	CS43L22_VERIFIED_WRITE_ERROR,
	CS43L22_POWER_UP_ERROR,
	CS43L22_POWER_DOWN_ERROR,
	CS43L22_DEFAULT_STATE_ERROR,
	CS43L22_BEEP_GENERATOR_INIT_ERROR,
	CS43L22_PAUSE_ERROR,
} cs43l22_status_t;

#ifdef TEST
void cs43l22_error_reset(void);
void cs43l22_get_error(uint32_t *io, uint32_t *out);
#endif

cs43l22_status_t cs43l22_init(cs43l22_if_t *interface);
cs43l22_status_t cs43l22_deinit(void);
cs43l22_status_t cs43l22_set_hw_params(audio_out_ll_hw_params_t *hw_params);

cs43l22_status_t beep_generator_init(beep_generator_t *beep);


#endif // CS43L22_H
