#include "audio_io.h"


																	/**< SPKR/HP pin is logical low (DGND)	*/

typedef enum {
	CS43L22_CHIP_ADDRESS 				= 0x94						/**< I2C address						*/
																	/**< A0 is logical LOW (DGND)			*/
} cs43l22_chip_address_t;

/**< ****************************************************************************************************************************** */
/**< Register addresses 																											*/
/**< ****************************************************************************************************************************** */
typedef enum {
	CS43L22_ID							= 0x01,				/**< Chip I.D. and Revision Register (Address 01h) (Read Only) 	*/
	POWER_CONTROL_1 					= 0x02,				/**< Power Control 1 (Address 02h) 								*/
	RESERVED_03H 						= 0x03,				/**< Reserved 													*/
	POWER_CONTROL_2 					= 0x04,				/**< Power Control 2 (Address 04h) 								*/
	CLOCKING_CONTROL 					= 0x05,				/**< Clocking Control (Address 05h) 							*/
	INTERFACE_CONTROL_1 				= 0x06,				/**< Interface Control 1 (Address 06h) 							*/
	INTERFACE_CONTROL_2 				= 0x07,				/**< Interface Control 2 (Address 07h) 							*/
	PASSTHROUGH_A_SELECT 				= 0x08,				/**< Passthrough x Select: PassA (Address 08h) 					*/
	PASSTHROUGH_B_SELECT 				= 0x09,				/**< Passthrough x Select: PassB (Address 09h) 					*/
	ANALOG_ZC_AND_SR_SETTINGS 			= 0x0A,				/**< Analog ZC and SR Settings (Address 0Ah) 					*/
	RESERVED_0BH 						= 0x0B,				/**< Reserved 													*/
	PASSTHROUGH_GANG_CONTROL 			= 0x0C,				/**< Passthrough Gang Control (Address 0Ch) 					*/
	PLAYBACK_CONTROL_1 					= 0x0D,				/**< Playback Control 1 (Address 0Dh) 							*/
	MISCELLANEOUS_CONTROL 				= 0x0E,				/**< Miscellaneous Controls (Address 0Eh) 						*/
	PLAYBACK_CONTROL_2 					= 0x0F,				/**< Playback Control 2 (Address 0Fh) 							*/
	RESERVED_10H 						= 0x10,				/**< Reserved 													*/
	RESERVED_11H 						= 0x11,				/**< Reserved 													*/
	RESERVED_12H 						= 0x12,				/**< Reserved 													*/
	RESERVED_13H 						= 0x13,				/**< Reserved 													*/
	PASSTHROUGH_A_VOLUME 				= 0x14,				/**< Passthrough x Volume: PASSAVOL (Address 14h)  				*/
	PASSTHROUGH_B_VOLUME 				= 0x15,				/**< Passthrough x Volume: PASSBVOL (Address 15h)     			*/
	RESERVED_16H 						= 0x16,				/**< Reserved 													*/
	RESERVED_17H 						= 0x17,				/**< Reserved 													*/
	RESERVED_18H 						= 0x18,				/**< Reserved 													*/
	RESERVED_19H 						= 0x19,				/**< Reserved 													*/
	PCMA_VOLUME 						= 0x1A,				/**< PCMx Volume: PCMA (Address 1Ah)  							*/
	PCMB_VOLUME 						= 0x1B,				/**< PCMx Volume: PCMB (Address 1Bh) 							*/
	BEEP_FREQUENCY_ON_TIME 				= 0x1C,				/**< Beep Frequency & On Time (Address 1Ch) 					*/
	BEEP_VOLUME_OFF_TIME 				= 0x1D,				/**< Beep Volume & Off Time (Address 1Dh) 						*/
	BEEP_TONE_CONFIG 					= 0x1E,				/**< Beep & Tone Configuration (Address 1Eh) 					*/
	TONE_CONTROL 						= 0x1F,				/**< Tone Control (Address 1Fh) 								*/
	MASTER_A_VOLUME 					= 0x20,				/**< Master Volume Control: MSTA (Address 20h)  				*/
	MASTER_B_VOLUME 					= 0x21,				/**< Master Volume Control: MSTB (Address 21h) 					*/
	HEADPHONE_A_VOLUME 					= 0x22,				/**< Headphone Volume Control: HPA (Address 22h)  				*/
	HEADPHONE_B_VOLUME 					= 0x23,				/**< Headphone Volume Control: HPB (Address 23h) 				*/
	SPEAKER_A_VOLUME 					= 0x24,				/**< Speaker Volume Control: SPKA (Address 24h) 				*/
	SPEAKER_B_VOLUME 					= 0x25,				/**< Speaker Volume Control: SPKB (Address 25h) 				*/
	CHANNEL_MIXER_AND_SWAP 				= 0x26,				/**< PCM Channel Swap (Address 26h) 							*/
	LIMITER_CONTROL_1 					= 0x27,				/**< Limiter Control 1, Min/Max Thresholds (Address 27h) 		*/
	LIMITER_CONTROL_2 					= 0x28,				/**< Limiter Control 2, Release Rate (Address 28h) 				*/
	LIMITER_ATTACK_RATE 				= 0x29,				/**< Limiter Attack Rate (Address 29h) 							*/
	RESERVED_2AH 						= 0x2A,				/**< Reserved 													*/
	RESERVED_2BH 						= 0x2B,				/**< Reserved 													*/
	RESERVED_2CH 						= 0x2C,				/**< Reserved 													*/
	RESERVED_2DH 						= 0x2D,				/**< Reserved 													*/
	OVERFLOW_AND_CLOCK_STATUS 			= 0x2E,				/**< Status (Address 2Eh) (Read Only) 							*/
	BATTERY_COMPENSATION 				= 0x2F,				/**< Battery Compensation (Address 2Fh) 						*/
	VP_BATTERY_LEVEL 					= 0x30,				/**< VP Battery Level (Address 30h) (Read Only) 				*/
	SPEAKER_STATUS 						= 0x31,				/**< Speaker Status (Address 31h) (Read Only) 					*/
	RESERVED_32H 						= 0x32,				/**< Reserved 													*/
	RESERVED_33H 						= 0x33,				/**< Reserved 													*/
	CHARGE_PUMP_FREQUENCY 				= 0x34				/**< Charge Pump Frequency (Address 34h) 						*/
} register_address_t;


/**< ****************************************************************************************************************************** */
/**< Register settings																												*/
/**< ****************************************************************************************************************************** */
typedef enum {
	
	/**< 7.1.1 Chip I.D. (Read Only)						*/
	CS43L22_DEVICE_ID							= 0b11100,
	
	/**< 7.1.2 Chip Revision (Read Only)					*/
	CS43L22_REVISION_LEVEL_A0 					= 0b000,			/**< Revision level: A0 										*/
	CS43L22_REVISION_LEVEL_A1 					= 0b001,			/**< Revision level: A1 										*/
	CS43L22_REVISION_LEVEL_B0 					= 0b010,			/**< Revision level: B0 										*/
	CS43L22_REVISION_LEVEL_B1 					= 0b011,			/**< Revision level: B1 										*/
	
	/**< 7.2.1 Power Down									*/
	CS43L22_STATUS_POWERED_DOWN_1 				= 0b00000001,		/**< Powered Down - same as setting 1001 1111					*/
	CS43L22_STATUS_POWERED_UP					= 0b10011110,		/**< Powered Up													*/
	CS43L22_STATUS_POWERED_DOWN_2				= 0b10011111,		/**< Powered Down - same as setting 0000 0001					*/
	
	/**< 7.3.1	Headphone Power Control						*/
	CS43L22_HEADPHONE_CHANNEL_ON_SPK_HP_SW_LO	= 0b00,				/**< Headphone channel is ON when the SPK/HP_SW pin, 6, is LO. 	*/
	CS43L22_HEADPHONE_CHANNEL_ON_SPK_HP_SW_HI	= 0b01,				/**< Headphone channel is ON when the SPK/HP_SW pin, 6, is HI. 	*/
	CS43L22_HEADPHONE_CHANNEL_ALWAYS_ON			= 0b10,				/**< Headphone channel is always ON. 							*/
	CS43L22_HEADPHONE_CHANNEL_ALWAYS_OFF		= 0b11,				/**< Headphone channel is always OFF. 							*/
	
	/**< 7.3.2	Speaker Power Control 						*/
	CS43L22_SPEAKER_CHANNEL_ON_SPK_HP_SW_LO		= 0b00,				/**< Speaker channel is ON when the SPK/HP_SW pin, 6, is LO.	*/
	CS43L22_SPEAKER_CHANNEL_ON_SPK_HP_SW_HI		= 0b01,				/**< Speaker channel is ON when the SPK/HP_SW pin, 6, is HI.	*/
	CS43L22_SPEAKER_CHANNEL_ALWAYS_ON			= 0b10,				/**< Speaker channel is always ON. 								*/
	CS43L22_SPEAKER_CHANNEL_ALWAYS_OFF			= 0b11,				/**< Speaker channel is always OFF. 							*/
	
	/**< 7.4.1	Auto-Detect 								*/
	/**< 7.5.3 	DSP Mode									*/
	/**< 7.8.1 	Ch. x Analog Soft Ramp						*/
	/**< 7.8.2 	Ch. x Analog Zero Cross 					*/
	/**< 7.9.1 	Passthrough Channel B=A Gang Control		*/
	/**< 7.10.2 Playback Volume Setting B=A					*/
	/**< 7.11.1 Passthrough Analog							*/
	/**< 7.11.2 Passthrough Mute							*/
	/**< 7.11.3 Freeze Registers							*/
	/**< 7.11.4 HP/Speaker De-Emphasis						*/
	/**< 7.11.5 Digital Soft Ramp							*/
	/**< 7.11.6 Digital Zero Cross							*/
	/**< 7.12.1 Headphone Mute 								*/
	/**< 7.12.2 Speaker Mute 								*/
	/**< 7.12.3 Speaker Volume Setting B=A					*/
	/**< 7.12.5 Speaker MONO Control						*/
	/**< 7.12.6 Speaker Mute 50/50 Control					*/
	/**< 7.14.1 PCM Channel x Mute							*/
	/**< 7.17.5 Tone Control Enable							*/
	/**< 7.24.1 Peak Detect and Limiter						*/
	/**< 7.24.2 Peak Signal Limit All Channels 				*/
	/**< 7.27.1 Battery Compensation						*/
	/**< 7.27.2 VP Monitor									*/
	CS43L22_DISABLED							= 0b0,
	CS43L22_ENABLED								= 0b1,
	
	/**< 7.17.2 Beep Mix Disable							*/
	CS43L22_BEEP_MIX_DISABLED 					= 0b1,
	CS43L22_BEEP_MIX_ENABLED 					= 0b0,
	/**< 7.23.3 Limiter Soft Ramp Disable					*/
	CS43L22_LIMITER_SOFT_RAMP_DISABLED 			= 0b1,
	CS43L22_LIMITER_SOFT_RAMP_ENABLED 			= 0b0,
	/**< 7.23.4 Limiter Zero Cross Disable					*/
	CS43L22_LIMITER_ZERO_CROSS_DISABLED 		= 0b1,
	CS43L22_LIMITER_ZERO_CROSS_ENABLED 			= 0b0,
	
	/**< 7.5.2 	SCLK Polarity								*/
	/**< 7.6.2 	Speaker/Headphone Switch Invert				*/
	/**< 7.10.3 Invert PCM Signal Polarity					*/
	/**< 7.10.4 Master Playback Mute						*/
	CS43L22_NOT_INVERTED 						= 0b0,
	CS43L22_INVERTED 							= 0b1,
	
	/**< 7.6.1 SCLK equals MCLK	(Master mode)				*/
	CS43L22_RETIMED 							= 0b0,				/**< Re-timed signal, synchronously derived from MCLK		*/
	CS43L22_NON_RETIMED 						= 0b1,				/**< Non-retimed, MCLK signal 								*/
	
	/**< 7.12.4 Speaker Channel Swap						*/
	CS43L22_CHANNEL_A							= 0b0,
	CS43L22_CHANNEL_B							= 0b1,
	
	/**< 7.4.2	Speed Mode 	(Slave Mode)					*/
	CS43L22_DOUBLE_SPEED_MODE 					= 0b00,				/**< Double-Speed Mode (DSM - 50 kHz -100 kHz Fs) 			*/
	CS43L22_SINGLE_SPEED_MODE 					= 0b01,				/**< Single-Speed Mode (SSM - 4 kHz -50 kHz Fs) 			*/
	CS43L22_HALF_SPEED_MODE   					= 0b10,				/**< Half-Speed Mode (HSM - 12.5kHz -25 kHz Fs) 			*/
	CS43L22_QUARTER_SPEED_MODE					= 0b11,				/**< Quarter-Speed Mode (QSM - 4 kHz -12.5 kHz Fs) 			*/
		
	/**< 7.4.2	Speed Mode 	(Master Mode)					*/	
	CS43L22_MCLK_LRCK_RATIO_512 				= 0b00,				/**< MCLK/LRCK Ratio 512									*/
	CS43L22_MCLK_LRCK_RATIO_256 				= 0b01,				/**< MCLK/LRCK Ratio 256									*/
	CS43L22_MCLK_LRCK_RATIO_128 				= 0b10,				/**< MCLK/LRCK Ratio 128									*/
	//CS43L22_MCLK_LRCK_RATIO_128 				= 0b11,				/**< MCLK/LRCK Ratio 128									*/
		
	/**< 7.4.3 	32kHz Sample Rate Group						*/	
	CS43L22_8_16_32_KHZ_SAMPLE_RATE 			= 0b1,				/**< 8 kHz, 16 kHz or 32 kHz sample rate? Yes				*/
		
	/**< 7.4.4	27 MHz Video Clock 							*/	
	CS43L22_27_MHZ_MCLK							= 0b1,				/**< 27 MHz MCLK? Yes										*/
		
	/**< 7.4.5 	Internal MCLK/LRCK Ratio					*/	
	CS43L22_INTERNAL_MCLK_LRCK_RATIO_128 		= 0b00,				/**< MCLK/LRCK Ratio  128									*/
	CS43L22_INTERNAL_MCLK_LRCK_RATIO_125 		= 0b01,				/**< MCLK/LRCK Ratio  125									*/
	CS43L22_INTERNAL_MCLK_LRCK_RATIO_132 		= 0b10,				/**< MCLK/LRCK Ratio  132									*/
	CS43L22_INTERNAL_MCLK_LRCK_RATIO_136 		= 0b11,				/**< MCLK/LRCK Ratio  136									*/
	CS43L22_SCLK_LRCK_RATIO_64 					= 0b00,				/**< SCLK/LRCK Ratio in Master Mode 64						*/
	CS43L22_SCLK_LRCK_RATIO_62 					= 0b01,				/**< SCLK/LRCK Ratio in Master Mode 62						*/
	CS43L22_SCLK_LRCK_RATIO_66 					= 0b10,				/**< SCLK/LRCK Ratio in Master Mode 66						*/
	CS43L22_SCLK_LRCK_RATIO_68 					= 0b11,				/**< SCLK/LRCK Ratio in Master Mode 68						*/
	
	/**< 7.4.6	MCLK Divide By 2 							*/
	CS43L22_MCLK_DIVIDED_BY_2					= 0b1,				/**< Divides the input MCLK by 2 prior to all internal circuitry. */

	/**< 7.5.1 	Master/Slave Mode							*/
	CS43L22_SLAVE_MODE  						= 0b0,				/**< Slave (input ONLY) 									*/
	CS43L22_MASTER_MODE 						= 0b1,				/**< Master (output ONLY) 									*/
			
	/**< 7.5.4 DAC Interface Format							*/
	CS43L22_INTERFACE_LEFT_JUSTIFIED  			= 0b00,				/**< Left Justified, up to 24-bit data 						*/
	CS43L22_INTERFACE_I2S						= 0b01,				/**< I2S, up to 24-bit data 								*/
	CS43L22_INTERFACE_RIGHT_JUSTIFIED			= 0b10,				/**< Right Justified 										*/
	CS43L22_INTERFACE_RESERVED					= 0b11,				/**< Reserved 												*/
	
	/**< 7.5.5 Audio Word Length							*/
	CS43L22_DSP_MODE_32BIT 						= 0b00,
	CS43L22_DSP_MODE_24BIT 						= 0b01,
	CS43L22_DSP_MODE_20BIT 						= 0b10,
	CS43L22_DSP_MODE_16BIT 						= 0b11,
	
	CS43L22_RIGHT_JUSTIFIED_24BIT 				= 0b00,
	CS43L22_RIGHT_JUSTIFIED_20BIT 				= 0b01,
	CS43L22_RIGHT_JUSTIFIED_18BIT 				= 0b10,
	CS43L22_RIGHT_JUSTIFIED_16BIT 				= 0b11,
		
	/**< 7.7.1 Passthrough Input Channel Mapping			*/
	CS43L22_NO_INPUTS_SELECTED 					= 0b00000,
	CS43L22_AIN1X 								= 0b00001,
	CS43L22_AIN2X 								= 0b00010,
	CS43L22_AIN3X 								= 0b00100,
	CS43L22_AIN4X 								= 0b01000,
	
	/**< 7.10.1 Headphone Analog Gain						*/
	CS43L22_HEADPHONE_ANALOG_GAIN_0_3959		= 0b000,
	CS43L22_HEADPHONE_ANALOG_GAIN_0_4571		= 0b001,
	CS43L22_HEADPHONE_ANALOG_GAIN_0_5111		= 0b010,
	CS43L22_HEADPHONE_ANALOG_GAIN_0_6047		= 0b011,
	CS43L22_HEADPHONE_ANALOG_GAIN_0_7099		= 0b100,
	CS43L22_HEADPHONE_ANALOG_GAIN_0_8399		= 0b101,
	CS43L22_HEADPHONE_ANALOG_GAIN_1_0000		= 0b110,
	CS43L22_HEADPHONE_ANALOG_GAIN_1_1430		= 0b111,
	
	/**< 7.15.1 Beep Frequency								*/
	CS43L22_C4_PITCH							= 0b0000,
	CS43L22_C5_PITCH							= 0b0001,
	CS43L22_D5_PITCH							= 0b0010,
	CS43L22_E5_PITCH							= 0b0011,
	CS43L22_F5_PITCH							= 0b0100,
	CS43L22_G5_PITCH							= 0b0101,
	CS43L22_A5_PITCH							= 0b0110,
	CS43L22_B5_PITCH							= 0b0111,
	CS43L22_C6_PITCH							= 0b1000,
	CS43L22_D6_PITCH							= 0b1001,
	CS43L22_E6_PITCH							= 0b1010,
	CS43L22_F6_PITCH							= 0b1011,
	CS43L22_G6_PITCH							= 0b1100,
	CS43L22_A6_PITCH							= 0b1101,
	CS43L22_B6_PITCH							= 0b1110,
	CS43L22_C7_PITCH							= 0b1111,
	
	/**< 7.15.2 Beep On Time								*/
	CS43L22_86_MS_ON_TIME						= 0b0000,
	CS43L22_430_MS_ON_TIME						= 0b0001,
	CS43L22_780_MS_ON_TIME						= 0b0010,
	CS43L22_1_20_S_ON_TIME						= 0b0011,
	CS43L22_1_50_S_ON_TIME						= 0b0100,
	CS43L22_1_80_S_ON_TIME						= 0b0101,
	CS43L22_2_20_S_ON_TIME						= 0b0110,
	CS43L22_2_50_S_ON_TIME						= 0b0111,
	CS43L22_2_80_S_ON_TIME						= 0b1000,
	CS43L22_3_20_S_ON_TIME						= 0b1001,
	CS43L22_3_50_S_ON_TIME						= 0b1010,
	CS43L22_3_80_S_ON_TIME						= 0b1011,
	CS43L22_4_20_S_ON_TIME						= 0b1100,
	CS43L22_4_50_S_ON_TIME						= 0b1101,
	CS43L22_4_80_S_ON_TIME						= 0b1110,
	CS43L22_5_20_S_ON_TIME						= 0b1111,
	
	/**< 7.16.1 Beep Off Time								*/
	CS43L22_1_23_S_OFF_TIME						= 0b000,
	CS43L22_2_58_S_OFF_TIME						= 0b001,
	CS43L22_3_90_S_OFF_TIME						= 0b010,
	CS43L22_5_20_S_OFF_TIME						= 0b011,
	CS43L22_6_60_S_OFF_TIME						= 0b100,
	CS43L22_8_05_S_OFF_TIME						= 0b101,
	CS43L22_9_35_S_OFF_TIME						= 0b110,
	CS43L22_10_80_S_OFF_TIME					= 0b111,
	
	/**< 7.17.1 Beep Configuration							*/
	CS43L22_BEEP_OCCURRENCE_OFF 				= 0b00,
	CS43L22_BEEP_OCCURRENCE_SINGLE				= 0b01,
	CS43L22_BEEP_OCCURRENCE_MULTIPLE			= 0b10,
	CS43L22_BEEP_OCCURRENCE_CONTINOUS			= 0b11,
	
	/**< 7.17.3 Treble Corner Frequency						*/
	CS43L22_TREBLE_CORNER_FREQUENCY_5_KHZ		= 0b00,
	CS43L22_TREBLE_CORNER_FREQUENCY_7_KHZ		= 0b01,
	CS43L22_TREBLE_CORNER_FREQUENCY_10_KHZ		= 0b10,
	CS43L22_TREBLE_CORNER_FREQUENCY_15_KHZ		= 0b11,
	
	/**< 7.17.4 Bass Corner Frequency						*/
	CS43L22_BASS_CORNER_FREQUENCY_50_HZ			= 0b00,
	CS43L22_BASS_CORNER_FREQUENCY_100_HZ    	= 0b01,
	CS43L22_BASS_CORNER_FREQUENCY_200_HZ    	= 0b10,
	CS43L22_BASS_CORNER_FREQUENCY_250_HZ    	= 0b11,
	
	/**< 7.22.1 PCM Channel Swap 							*/
	CS43L22_PCMA_CHANNEL_SWAP_LEFT				= 0b00,
	CS43L22_PCMA_CHANNEL_SWAP_RIGHT				= 0b11,
	CS43L22_PCMB_CHANNEL_SWAP_LEFT				= 0b11,
	CS43L22_PCMB_CHANNEL_SWAP_RIGHT				= 0b00,
	CS43L22_PCMx_CHANNEL_MIX					= 0b01,
		
	/**< 7.29.1 Speaker Current Load Status (Read Only)		*/
	CS43L22_NO_OVERLOAD_DETECTED				= 0b0,
	CS43L22_OVERLOAD_DETECTED					= 0b1,
	
	/**< 7.29.2 SPKR/HP Pin Status (Read Only)				*/
	CS43L22_SPKR_HP_PIN_LOW						= 0b0,
	CS43L22_SPKR_HP_PIN_HIGH					= 0b1,
	
} register_settings_t;

/**< ****************************************************************************************************** */
/**< 5.1.1 Memory Address Pointer (MAP)																		*/
/**< ****************************************************************************************************** */
typedef union {
	struct {
		uint8_t INCR		: 1;			/**< 5.1.1.1 Map Increment (INCR)								*/
		uint8_t address		: 7;
	} b;
	uint8_t w;
} map_t;


/**< ****************************************************************************************************** */
/**< Registers																								*/
/**< ****************************************************************************************************** */

/**< 7.1	Chip I.D. and Revision Register (Address 01h) (Read Only) 										*/
typedef union { 
	struct {
		uint32_t CHIPID 	: 5;			/**< 7.1.1	Chip I.D. (Read Only) 								*/
		uint32_t REVID 		: 3;			/**< 7.1.2	Chip Revision (Read Only) 							*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} id_revision_reg_t;				


/**< 7.2	Power Control 1 (Address 02h) 																	*/
typedef union { 
	struct {
		uint32_t PDN		: 8;			/**< 7.2.1	Power Down 											*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} power_ctl_1_reg_t;		
			

/**< 7.3	Power Control 2 (Address 04h) 																	*/
typedef union {
	struct {
		uint32_t PDN_HPB	: 2;			/**< Headphone B Power Status									*/
		uint32_t PDN_HPA	: 2;			/**< Headphone A Power Status									*/
		uint32_t PDN_SPKB	: 2;			/**< Speaker B Power Status										*/
		uint32_t PDN_SPKA	: 2;			/**< Speaker A Power Status										*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} power_ctl_2_reg_t;		


/**< 7.4	Clocking Control (Address 05h)																	*/
typedef union {
	struct {
		uint32_t MCLKDIV2	: 1;			/**< 7.4.6	MCLK Divide By 2 									*/
		uint32_t RATIO		: 2;			/**< 7.4.5	Internal MCLK/LRCK Ratio 							*/
		uint32_t VIDEOCLK	: 1;			/**< 7.4.4	27 MHz Video Clock 									*/
		uint32_t GROUP_32k	: 1;			/**< 7.4.3	32kHz Sample Rate Group 							*/
		uint32_t SPEED		: 2;			/**< 7.4.2	Speed Mode 											*/
		uint32_t AUTO		: 1;			/**< 7.4.1	Auto-Detect 										*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} clocking_ctl_reg_t;


/**< 7.5 	Interface Control 1 (Address 06h)																*/
typedef union {
	struct {
		uint32_t AWL		: 2;			/**< 7.5.5 Audio Word Length 									*/
		uint32_t DACDIF		: 2;			/**< 7.5.4 DAC Interface Format									*/
		uint32_t DSP		: 1;			/**< 7.5.3 DSP Mode												*/
		uint32_t Reserved	: 1;			/**< Reserved													*/
		uint32_t INV_SCLK 	: 1;			/**< 7.5.2 SCLK Polarity										*/
		uint32_t MS			: 1;			/**< 7.5.1 Master/Slave Mode									*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} interface_ctl_1_reg_t;


/**< 7.6 	Interface Control 2 (Address 07h)																*/
typedef union {
	struct {
		uint32_t Reserved_3 : 3;			/**< Reserved													*/
		uint32_t INV_SWCH	: 1;			/**< 7.6.2 Speaker/Headphone Switch Invert 						*/
		uint32_t Reserved_2 : 2;			/**< Reserved													*/
		uint32_t SCLK_MCLK	: 1;			/**< 7.6.1 SCLK equals MCLK				 						*/
		uint32_t Reserved_1 : 1;			/**< Reserved													*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} interface_ctl_2_reg_t;


/**< 7.7 	Passthrough x Select: PassA (Address 08h), PassB (Address 09h)									*/
typedef union {
	struct {
		uint32_t PASSxSEL	: 5;			/**< 7.7.1 Passthrough Input Channel Mapping					*/
		uint32_t Reserved 	: 3;			/**< Reserved													*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} passthrough_sel_reg_t;


/**< 7.8 	Analog ZC and SR Settings (Address 0Ah)															*/
typedef union { 
	struct {
		uint32_t ANLGZCA	: 1;
		uint32_t ANLGSFTA 	: 1;
		uint32_t ANLGZCB	: 1;				/**< 7.8.2 Ch. x Analog Zero Cross							*/	
		uint32_t ANLGSFTB 	: 1;				/**< 7.8.1 Ch. x Analog Soft Ramp							*/
		uint32_t Reserved 	: 4;				/**< Reserved												*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} analog_zc_sr_settings_reg_t;


/**< 7.9 	Passthrough Gang Control (Address 0Ch)															*/
typedef union {
	struct {
		uint32_t Reserved 	: 7;
		uint32_t PASSB_A	: 1;				/**< 7.9.1 Passthrough Channel B=A Gang Control				*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} passthrough_gang_ctl_reg_t;


/**< 7.10 	Playback Control 1 (Address 0Dh)																*/
typedef union {
	struct {
		uint32_t MSTAMUTE	: 1;				/**< 7.10.4 Master Playback A Mute							*/
		uint32_t MSTBMUTE	: 1;				/**< 7.10.4 Master Playback B Mute							*/
		uint32_t INV_PCMA	: 1;				/**< 7.10.3 Invert PCM A Signal Polarity					*/
		uint32_t INV_PCMB	: 1;				/**< 7.10.3 Invert PCM B Signal Polarity					*/
		uint32_t PLYBCKB_A	: 1;				/**< 7.10.2 Playback Volume Setting B=A						*/
		uint32_t HPGAIN		: 3;				/**< 7.10.1 Headphone Analog Gain							*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} playback_ctl_1_reg_t;


/**< 7.11 	Miscellaneous Controls (Address 0Eh)															*/
typedef union {
	struct {
		uint32_t DIGZC		: 1;				/**< 7.11.6 Digital Zero Cross								*/
		uint32_t DIGSFT		: 1;				/**< 7.11.5 Digital Soft Ramp								*/
		uint32_t DEEMPH		: 1;				/**< 7.11.4 HP/Speaker De-Emphasis							*/
		uint32_t FREEZE		: 1;				/**< 7.11.3 Freeze Registers								*/
		uint32_t PASSAMUTE	: 1;				/**< 7.11.2 Passthrough Mute A								*/
		uint32_t PASSBMUTE	: 1;				/**< 7.11.2 Passthrough Mute B								*/
		uint32_t PASSTHRUA	: 1;				/**< 7.11.1 Passthrough Analog A							*/
		uint32_t PASSTHRUB	: 1;				/**< 7.11.1 Passthrough Analog B							*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} misc_ctl_reg_t;


/**< 7.12 	Playback Control 2 (Address 0Fh)																*/
typedef union {
	struct {
		uint32_t MUTE50_50	: 1;				/**< 7.12.6 Speaker Mute 50/50 Control						*/
		uint32_t SPKMONO	: 1;				/**< 7.12.5 Speaker MONO Control							*/
		uint32_t SPKSWAP	: 1;				/**< 7.12.4 Speaker Channel Swap							*/
		uint32_t SPKB_A		: 1;				/**< 7.12.3 Speaker Volume Setting B=A						*/
		uint32_t SPKAMUTE	: 1;				/**< 7.12.2 Speaker Mute A									*/
		uint32_t SPKBMUTE	: 1;				/**< 7.12.2 Speaker Mute B									*/
		uint32_t HPAMUTE	: 1;				/**< 7.12.1 Headphone Mute A								*/
		uint32_t HPBMUTE	: 1;				/**< 7.12.1 Headphone Mute B								*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} playback_ctl_2_reg_t;


/**< 7.13 	Passthrough x Volume: PASSAVOL (Address 14h) & PASSBVOL (Address 15h) 							*/


/**< 7.14 	PCMx Volume: PCMA (Address 1Ah) & PCMB (Address 1Bh)											*/
typedef union {
	struct {
		uint32_t PCMxVOL	: 7;				/**< 7.14.2 PCM Channel x Volume							*/
		uint32_t PCMxMUTE	: 1;				/**< 7.14.1 PCM Channel x Mute								*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} pcmx_vol_reg_t;


/**< 7.15 	Beep Frequency & On Time (Address 1Ch)															*/
typedef union {
	struct {
		uint32_t ONTIME		: 4;				/**< 7.15.2 Beep On Time									*/
		uint32_t FREQ		: 4;				/**< 7.15.1 Beep Frequency									*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} beep_freq_on_time_reg_t; 


/**< 7.16 	Beep Volume & Off Time (Address 1Dh)															*/
typedef union {
	struct {
		uint32_t BEEPVOL	: 5;				/**< 7.16.2 Beep Volume										*/
		uint32_t OFFTIME	: 3;				/**< 7.16.1 Beep Off Time									*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} beep_vol_off_time_reg_t;


/**< 7.17 	Beep & Tone Configuration (Address 1Eh)															*/
typedef union {
	struct {
		uint32_t TCEN		: 1;				/**< 7.17.5 Tone Control Enable 							*/
		uint32_t BASSCF		: 2;				/**< 7.17.4 Bass Corner Frequency							*/
		uint32_t TREBCF		: 2;				/**< 7.17.3 Treble Corner Frequency							*/
		uint32_t BEEPMIXDIS	: 1;				/**< 7.17.2 Beep Mix Disable								*/
		uint32_t BEEP		: 2;				/**< 7.17.1 Beep Configuration								*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} beep_tone_config_reg_t;


/**< 7.18 	Tone Control (Address 1Fh)																		*/
typedef union {
	struct {
		uint32_t BASS		: 4;				/**< 7.18.2 Bass Gain 										*/
		uint32_t TREB		: 4;				/**< 7.18.1 Treble Gain										*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} tone_control_reg_t;


/**< 7.19 	Master Volume Control: MSTA (Address 20h) & MSTB (Address 21h)									*/


/**< 7.20 	Headphone Volume Control: HPA (Address 22h) & HPB (Address 23h)									*/


/**< 7.21 	Speaker Volume Control: SPKA (Address 24h) & SPKB (Address 25h)									*/


/**< 7.22 	PCM Channel Swap (Address 26h)																	*/
typedef union {
	struct {
		uint32_t Reserved	: 4;
		uint32_t PCMBSWP	: 2;
		uint32_t PCMASWP	: 2;				/**< 7.22.1 PCM Channel Swap 								*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} channel_mixer_swap_reg_t;


/**< 7.23 	Limiter Control 1, Min/Max Thresholds (Address 27h)												*/
typedef union {
	struct {
		uint32_t LIMZCDIS	: 1;				/**< 7.23.4 Limiter Zero Cross Disable						*/
		uint32_t LIMSRDIS	: 1;				/**< 7.23.3 Limiter Soft Ramp Disable						*/
		uint32_t CUSH		: 3;				/**< 7.23.2 Limiter Cushion Threshold 						*/
		uint32_t LMAX		: 3;				/**< 7.23.1 Limiter Maximum Threshold						*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} limiter_ctl_1_reg_t;


/**< 7.24 	Limiter Control 2, Release Rate (Address 28h)													*/
typedef union {
	struct {
		uint32_t LIMRRATE	: 6;				/**< 7.24.3 Limiter Release Rate							*/
		uint32_t LIMIT_ALL	: 1;				/**< 7.24.2 Peak Signal Limit All Channels					*/
		uint32_t LIMIT 		: 1;				/**< 7.24.1 Peak Detect and Limiter							*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} limiter_ctl_2_reg_t;


/**< 7.25 	Limiter Attack Rate (Address 29h)																*/
typedef union {
	struct {
		uint32_t RATE		: 6;				/**< 7.25.1 Limiter Attack Rate 							*/
		uint32_t Reserved	: 2;				/**< Reserved												*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} limiter_attack_rate_reg_t;


/**< 7.26 	Status (Address 2Eh) (Read Only)																*/
typedef union {
	struct {
		uint32_t Reserved_2	: 1;				/**< Reserved												*/
		uint32_t PCMBOVFL	: 1;				/**< */                                                     
		uint32_t PCMAOVFL	: 1;				/**< 7.26.3 PCMx Overflow (Read Only)						*/
		uint32_t DSPBOVFL	: 1;				/**< */                                                     
		uint32_t DSPAOVFL	: 1;				/**< 7.26.2 DSP Engine Overflow (Read Only)					*/
		uint32_t SPCLKERR	: 1;				/**< 7.26.1 Serial Port Clock Error (Read Only)				*/
		uint32_t Reserved_1	: 1;				/**< Reserved												*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} overflow_clock_status_reg_t;


/**< 7.27 	Battery Compensation (Address 2Fh)																*/
typedef union {
	struct {
		uint32_t VPREF		: 4;				/**< 7.27.3 VP Reference									*/
		uint32_t Reserved	: 2;				/**< Reserved												*/
		uint32_t VPMONITOR	: 1;				/**< 7.27.2 VP Monitor										*/
		uint32_t BATTCMP	: 1;				/**< 7.27.1 Battery Compensation							*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} battery_comp_reg_t;


/**< 7.28 	VP Battery Level (Address 30h) (Read Only)														*/


/**< 7.29 	Speaker Status (Address 31h) (Read Only)														*/
typedef union {
	struct {
		uint32_t Reserved_2	: 3;				/**< Reserved												*/
		uint32_t SPKR_HP	: 1;				/**< 7.29.2 SPKR/HP Pin Status (Read Only)					*/
		uint32_t SPKBSHRT	: 1;				/**< 7.29.1 Speaker Current Load Status (Read Only)			*/
		uint32_t SPKASHRT	: 1;				/**< 7.29.1 Speaker Current Load Status (Read Only)			*/
		uint32_t Reserved_1	: 2;				/**< Reserved												*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} speaker_status_reg_t;


/**< 7.30 	Charge Pump Frequency (Address 34h)																*/
typedef union {
	struct {
		uint32_t Reserved	: 4;				/**< Reserved												*/
		uint32_t FREQ		: 4;				/**< 7.30.1 Charge Pump Frequency							*/
		uint32_t _reserved0 : 24;
	} b;
	uint32_t w;
} charge_pump_freq_reg_t;


/**< ****************************************************************************************************************************** */
/**< CS43L22 Codec descriptor																										*/
/**< ****************************************************************************************************************************** */
typedef struct {
	id_revision_reg_t 					ID;
	power_ctl_1_reg_t 					PDNCTL1;
	power_ctl_2_reg_t 					PDNCTL2;
	clocking_ctl_reg_t 					CLKCTL;
	interface_ctl_1_reg_t 				IFCTL1;
	interface_ctl_2_reg_t 				IFCTL2;
	passthrough_sel_reg_t 				PASSASEL;
	passthrough_sel_reg_t 				PASSBSEL;
	analog_zc_sr_settings_reg_t			ANLGZCSFT;
	passthrough_gang_ctl_reg_t			PASSGCTL;
	playback_ctl_1_reg_t				PLYBCKCTL1;
	misc_ctl_reg_t						MISCCTL;
	playback_ctl_2_reg_t				PLYBCKCTL2;
	uint32_t							PASSAVOL;
	uint32_t							PASSBVOL;
	pcmx_vol_reg_t						PCMA;
	pcmx_vol_reg_t						PCMB;
	beep_freq_on_time_reg_t				BEEPFOT;		
	beep_vol_off_time_reg_t				BEEPVOT;
	beep_tone_config_reg_t				BEEPCFG;
	tone_control_reg_t					TONECTL;
	uint32_t							MSTAVOL;
	uint32_t							MSTBVOL;
	uint32_t							HPAVOL;
	uint32_t							HPBVOL;
	uint32_t							SPKAVOL;
	uint32_t							SPKBVOL;
	channel_mixer_swap_reg_t			PCMxSWP;
	limiter_ctl_1_reg_t					LIMCTL1;
	limiter_ctl_2_reg_t					LIMCTL2;
	limiter_attack_rate_reg_t			LIMARATE;
	overflow_clock_status_reg_t 		OFCLKSTAT;
	battery_comp_reg_t					BATTCOMP;
	uint32_t							VPLVL;
	speaker_status_reg_t				SPKRSTAT;
	charge_pump_freq_reg_t				CHGFREQ;
	
} cs43l22_desc_t;


/**< ****************************************************************************************************************************** */
/**< Supported audio channel configurations																							*/
/**< ****************************************************************************************************************************** */
typedef enum {
	CS43L22_CHANNELS_MONO = 0,
	CS43L22_CHANNELS_STEREO,
	CS43L22_CHANNELS_NOTSUPPORTED
} cs43l22_channels_t;

/**< ****************************************************************************************************************************** */
/**< Supported audio formats																										*/
/**< ****************************************************************************************************************************** */
typedef enum {
	CS43L22_FORMAT_S8 = 0, 
	CS43L22_FORMAT_U8, 
	CS43L22_FORMAT_S16_LE,
	CS43L22_FORMAT_S16_BE, 
	CS43L22_FORMAT_U16_LE, 
	CS43L22_FORMAT_U16_BE, 
	CS43L22_FORMAT_S24_LE,
	CS43L22_FORMAT_S24_BE, 
	CS43L22_FORMAT_U24_LE, 
	CS43L22_FORMAT_U24_BE, 
	CS43L22_FORMAT_S32_LE,
	CS43L22_FORMAT_S32_BE, 
	CS43L22_FORMAT_U32_LE, 
	CS43L22_FORMAT_U32_BE, 
	CS43L22_FORMAT_FLOAT_LE,
	CS43L22_FORMAT_FLOAT_BE,
	CS43L22_FORMAT_NOTSUPPORTED
} cs43l22_format_t;

typedef enum {
	CS43L22_OK = 0,
	CS43L22_ERROR,
	CS43L22_NOTSUPPORTED
} cs43l22_retv_t;

/**< ****************************************************************************************************************************** */
/**< Codec functions																												*/
/**< ****************************************************************************************************************************** */
uint32_t cs43l22_set_rate(cs43l22_desc_t *desc, uint32_t *rate);
uint32_t cs43l22_set_rate_near(cs43l22_desc_t *desc, uint32_t *rate);
uint32_t cs43l22_set_format(cs43l22_desc_t *desc, const uint32_t *format);
uint32_t cs43l22_set_channels(cs43l22_desc_t *desc, const uint32_t *channels);
uint32_t cs43l22_init(cs43l22_desc_t *desc);

/*
cs43l22_deinit();
cs43l22_reset();
cs43l22_play();
cs43l22_pause();
cs43l22_resume();
cs43l22_stop();
*/




