#ifndef AUDIO_H
#define AUDIO_H

#include "cs43l22.h"

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

void audio_out_init(void);

#endif // AUDIO_H
