#include <stdint.h>
#include "audio_io.h"
#include "cs43l22.h"


/**< ****************************************************************************************************************************** */
/**< Supported audio channel configurations																							*/
/**< ****************************************************************************************************************************** */
//typedef enum {
//	AUDIO_CHANNELS_MONO = 0,
//	AUDIO_CHANNELS_STEREO,
//	AUDIO_CHANNELS_NOTSUPPORTED
//} audio_channels_t;

typedef cs43l22_channels_t audio_channels_t;

/**< ****************************************************************************************************************************** */
/**< Supported audio access modes																									*/
/**< ****************************************************************************************************************************** */
typedef enum {
	AUDIO_ACCESS_INTERLEAVED = 0,
	AUDIO_ACCESS_NONINTERLEAVED,
	AUDIO_ACCESS_NOTSUPPORTED
} audio_access_t;


/**< ****************************************************************************************************************************** */
/**< Supported audio formats																										*/
/**< ****************************************************************************************************************************** */
//typedef enum {
//	AUDIO_FORMAT_S8 = 0,
//	AUDIO_FORMAT_U8,
//	AUDIO_FORMAT_S16_LE,
//	AUDIO_FORMAT_S16_BE,
//	AUDIO_FORMAT_U16_LE,
//	AUDIO_FORMAT_U16_BE,
//	AUDIO_FORMAT_S24_LE,
//	AUDIO_FORMAT_S24_BE,
//	AUDIO_FORMAT_U24_LE,
//	AUDIO_FORMAT_U24_BE,
//	AUDIO_FORMAT_S32_LE,
//	AUDIO_FORMAT_S32_BE,
//	AUDIO_FORMAT_U32_LE,
//	AUDIO_FORMAT_U32_BE,
//	AUDIO_FORMAT_FLOAT_LE,
//	AUDIO_FORMAT_FLOAT_BE,
//	AUDIO_FORMAT_NOTSUPPORTED
//} audio_format_t;

typedef cs43l22_format_t audio_format_t;


/**< ****************************************************************************************************************************** */
/**< Audio states																													*/
/**< ****************************************************************************************************************************** */
typedef enum {
	AUDIO_STATE_OPEN = 0, 
	AUDIO_STATE_SETUP, 
	AUDIO_STATE_PREPARED, 
	AUDIO_STATE_RUNNING,
	AUDIO_STATE_XRUN, 
	AUDIO_STATE_DRAINING, 
	AUDIO_STATE_PAUSED, 
	AUDIO_STATE_SUSPENDED,
	AUDIO_STATE_DISCONNECTED,
} audio_state_t;


/**< ****************************************************************************************************************************** */
/**< Audio handler typedef																											*/
/**< ****************************************************************************************************************************** */
typedef struct {
	audio_channels_t	channels;
	audio_access_t   	access;
	audio_format_t	 	format;
	uint32_t		 	rate;
	uint8_t 		 	*buffer;
	uint32_t		 	buffer_size;
	uint32_t		 	period_size;
	uint32_t			periods;
	uint32_t			frame_size;
} audio_opt_t;


/**< ****************************************************************************************************************************** */
/**< Return values																													*/
/**< ****************************************************************************************************************************** */
typedef enum {
	AUDIO_OK = 0,
	AUDIO_ERROR,
	AUDIO_NOTSUPPORTED
} audio_retv_t;


/**< ****************************************************************************************************************************** */
/**< Audio functions																												*/
/**< ****************************************************************************************************************************** */
uint32_t audio_init(audio_opt_t *opt);
uint32_t audio_set_format(audio_opt_t *opt, const uint32_t *format);
uint32_t audio_set_rate_near(audio_opt_t *opt, uint32_t *rate);
uint32_t audio_set_rate(audio_opt_t *opt, uint32_t *rate);
uint32_t audio_set_channels(audio_opt_t *opt, const uint32_t *channels);

uint32_t audio_set_access(audio_opt_t *opt, const audio_access_t access);
uint32_t audio_set_period_size(audio_opt_t *opt, const uint32_t period_size);
uint32_t audio_set_buffer_size(audio_opt_t *opt, const uint32_t buffer_size);
uint32_t audio_set_periods(audio_opt_t *opt, const uint32_t periods);


