#ifndef AUDIO_H
#define AUDIO_H

#include "cs43l22.h"

#define AUDIO_BUFFER_SIZE        (1024U)
#define AUDIO_BUFFER_HALF_SIZE   (AUDIO_BUFFER_SIZE/2)

typedef struct {
	codec_status_t (*init)(void);
	codec_status_t (*deinit)(void);
	codec_status_t (*set_hw_params)(audio_out_ll_hw_params_t *hw_params);
	codec_status_t (*play)(void);
	codec_status_t (*pause)(void);
	codec_status_t (*resume)(void);
	codec_status_t (*stop)(void);
	codec_status_t (*set_volume)(float volume);
	codec_status_t (*set_mute)(codec_mute_t mute);
} codec_if_t;

typedef enum {
	AUDIO_OK = 0,
	AUDIO_OUT_INIT_ERROR,
	AUDIO_OUT_DEINIT_ERROR,
	AUDIO_OUT_START_ERROR,
	AUDIO_OUT_PAUSE_ERROR,
	AUDIO_OUT_RESUME_ERROR,
	AUDIO_OUT_STOP_ERROR,
	AUDIO_OUT_WRITE_ERROR,
	AUDIO_OUT_VOLUME_ERROR,
	AUDIO_OUT_MUTE_ERROR,
	AUDIO_OUT_HW_PARAMS_ERROR,
	AUDIO_OUT_CALLBACK_ERROR,
} audio_status_t;

typedef enum {
	AUDIO_FORMAT_S8 = 0,
	AUDIO_FORMAT_U8,
	AUDIO_FORMAT_S16_LE,
	AUDIO_FORMAT_S16_BE,
	AUDIO_FORMAT_U16_LE,
	AUDIO_FORMAT_U16_BE,
	AUDIO_FORMAT_S24_LE,
	AUDIO_FORMAT_S24_BE,
	AUDIO_FORMAT_U24_LE,
	AUDIO_FORMAT_U24_BE,
	AUDIO_FORMAT_S32_LE,
	AUDIO_FORMAT_S32_BE,
	AUDIO_FORMAT_U32_LE,
	AUDIO_FORMAT_U32_BE,
	AUDIO_FORMAT_FLOAT_LE,
	AUDIO_FORMAT_FLOAT_BE,
} audio_format_t;

typedef enum {
	AUDIO_CHANNELS_MONO   = 0x01U,
	AUDIO_CHANNELS_STEREO = 0x02U,
} audio_channels_t;

typedef enum {
	AUDIO_ACCESS_INTERLEAVED = 0,
	AUDIO_ACCESS_NONINTERLEAVED,
} audio_access_t;

typedef struct {
	uint32_t rate;
	uint32_t channels;
//	uint32_t access;
	uint32_t format;
} audio_out_hw_params_t;

typedef union {
	struct {
		uint32_t bad_params  : 1;
		uint32_t init        : 1;
		uint32_t deinit      : 1;
		uint32_t pause       : 1;
		uint32_t resume      : 1;
		uint32_t stop        : 1;
		uint32_t play        : 1;
		uint32_t volume      : 1;
		uint32_t mute        : 1;
		uint32_t hw_params   : 1;
		uint32_t             : 22;
	};
	uint32_t w;
} audio_codec_err_t;

typedef union {
	struct {
		uint32_t bad_params  : 1;
		uint32_t cb_params   : 1;
		uint32_t init        : 1;
		uint32_t deinit      : 1;
		uint32_t start       : 1;
		uint32_t pause       : 1;
		uint32_t resume      : 1;
		uint32_t stop        : 1;
		uint32_t write       : 1;
		uint32_t             : 23;
	};
	uint32_t w;
} audio_out_err_t;

typedef struct {
	audio_codec_err_t codec;
	audio_out_err_t   out;
} audio_err_t;

typedef void (*audio_out_write_callback_t)(uint32_t size);

audio_status_t audio_out_init(void);
audio_status_t audio_out_deinit(void);
audio_status_t audio_out_start(void);
audio_status_t audio_out_write(uint8_t *data, const uint32_t size);
audio_status_t audio_out_pause(void);
audio_status_t audio_out_resume(void);
audio_status_t audio_out_stop(void);
audio_status_t audio_out_set_volume(float volume);
audio_status_t audio_out_set_mute(codec_mute_t mute);
audio_status_t audio_out_set_hw_params(audio_out_hw_params_t *hw_params);
audio_status_t audio_out_set_write_callback(audio_out_write_callback_t callback);

#endif // AUDIO_H


