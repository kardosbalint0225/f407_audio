#include "audio.h"
#include <stddef.h>

static codec_if_t codec = {
	.init          = cs43l22_init,
	.deinit        = cs43l22_deinit,
	.set_hw_params = cs43l22_set_hw_params,
	.play          = cs43l22_play,
	.pause         = cs43l22_pause,
	.resume        = cs43l22_resume,
	.stop          = cs43l22_stop,
	.set_volume    = cs43l22_set_volume,
	.set_mute      = cs43l22_set_mute,
};

static audio_err_t audio_error;
static audio_out_hw_params_t audio_out_hw_params = {
	.rate     = 0,
	.channels = 0,
//	.access   = 0,
	.format   = 0,
};

uint8_t audio_buffer[AUDIO_BUFFER_SIZE] __attribute__((aligned(16)));
static uint8_t *active_period = NULL;

static uint32_t convert_audio_format_to_size(audio_format_t format);
static uint32_t convert_audio_format_to_data_format(audio_format_t format);

audio_out_write_callback_t write_callback = NULL;

void audio_out_ll_write_callback(uint16_t *address, const audio_out_ll_cb_id_t callback_id)
{
    switch (callback_id)
    {
        case AUDIO_OUT_LL_TX_HALF_COMPLETE_CB_ID : {
            
			active_period = audio_buffer;
			write_callback(audio_buffer, AUDIO_BUFFER_HALF_SIZE);

        } break;

        case AUDIO_OUT_LL_TX_COMPLETE_CB_ID : {

			active_period = &audio_buffer[AUDIO_BUFFER_HALF_SIZE];

			const uint16_t size = AUDIO_BUFFER_SIZE / sizeof(uint16_t);
        	if (AUDIO_OUT_LL_OK != audio_out_ll_write((uint16_t *)audio_buffer, size)) {
        		while(1);
            }

        	write_callback(&audio_buffer[AUDIO_BUFFER_HALF_SIZE], AUDIO_BUFFER_HALF_SIZE);

        } break;

        default : {

            // error()

        } break;
    }
}

audio_status_t audio_out_init(void)
{
	audio_error.codec.w = 0;
	audio_error.out.w   = 0;
	active_period       = audio_buffer;

	for (uint32_t i = 0; i < AUDIO_BUFFER_SIZE; i++) {
		audio_buffer[i] = 0;
	}

	if (CODEC_OK != codec.init()) {
		audio_error.codec.init = 1;
	}

	audio_out_ll_cb_params_t cb_params;
	cb_params.m0_buffer      = (uint16_t *)audio_buffer;
	cb_params.m1_buffer      = (uint16_t *)&audio_buffer[AUDIO_BUFFER_HALF_SIZE];
	cb_params.write_callback = audio_out_ll_write_callback;

	if (AUDIO_OUT_LL_OK != audio_out_ll_set_cb_params(&cb_params)) {
		audio_error.out.cb_params = 1;
	}

	audio_status_t retc = ((0 != audio_error.codec.w) || (0 != audio_error.out.w)) ? (AUDIO_OUT_INIT_ERROR) : (AUDIO_OK);
	return retc;
}

audio_status_t audio_out_deinit(void)
{
	if (CODEC_OK != codec.deinit()) {
		audio_error.codec.deinit = 1;
	}

	if (AUDIO_OUT_LL_OK != audio_out_ll_deinit()) {
		audio_error.out.deinit = 1;
	}

	audio_status_t retc = ((0 != audio_error.codec.w) || (0 != audio_error.out.w)) ? (AUDIO_OUT_DEINIT_ERROR) : (AUDIO_OK);
	return retc;
}

audio_status_t audio_out_write(uint8_t *data, const uint32_t size)
{
	if (size > AUDIO_BUFFER_SIZE) {
		audio_error.out.write = 1;
	}

//	uint8_t channels = audio_out_hw_params.channels;
//	uint32_t format  = convert_audio_format_to_size(audio_out_hw_params.format);

	/* Does not support 24bit and mono yet */
	for (uint32_t i = 0; i < size; i++) {
		//audio_buffer[i] = data[i];
		active_period[i] = data[i];
	}
	
	audio_status_t retc = (0 != audio_error.out.w) ? (AUDIO_OUT_WRITE_ERROR) : (AUDIO_OK);
	return retc;
}

audio_status_t audio_out_start(void)
{
	const uint16_t size = AUDIO_BUFFER_SIZE / sizeof(uint16_t);
	if (AUDIO_OUT_LL_OK != audio_out_ll_write((uint16_t *)audio_buffer, size)) {
		audio_error.out.start = 1;
	}

	if (CODEC_OK != codec.play()) {
		audio_error.codec.play = 1;
	}

	audio_status_t retc = (0 != audio_error.out.w) ? (AUDIO_OUT_START_ERROR) : (AUDIO_OK);
	return retc;
}

static uint32_t convert_audio_format_to_size(audio_format_t format)
{
	uint32_t size;

	switch (format)
	{
		case AUDIO_FORMAT_S8       : 
		case AUDIO_FORMAT_U8       : size = 1;
		                             break;
		case AUDIO_FORMAT_S16_LE   : 
		case AUDIO_FORMAT_S16_BE   : 
        case AUDIO_FORMAT_U16_LE   : 
		case AUDIO_FORMAT_U16_BE   : size = 2;
		                             break;
	    case AUDIO_FORMAT_S24_LE   : 
		case AUDIO_FORMAT_S24_BE   :
		case AUDIO_FORMAT_U24_LE   :
		case AUDIO_FORMAT_U24_BE   : size = 3;
		                             break;
		case AUDIO_FORMAT_S32_LE   :
		case AUDIO_FORMAT_S32_BE   :
		case AUDIO_FORMAT_U32_LE   :
		case AUDIO_FORMAT_U32_BE   : size = 4;
		                             break;
		case AUDIO_FORMAT_FLOAT_LE :
		case AUDIO_FORMAT_FLOAT_BE : size = 4;
		                             break;
		default                    : size = 2;
		                             break;

	}

	return size;
}

static uint32_t convert_audio_format_to_data_format(audio_format_t format)
{
	uint32_t data_format;

	switch (format)
	{
		case AUDIO_FORMAT_S8       : 
		case AUDIO_FORMAT_U8       : data_format = AUDIO_OUT_LL_DATAFORMAT_16B;
		                             break;
		case AUDIO_FORMAT_S16_LE   : 
		case AUDIO_FORMAT_S16_BE   : 
        case AUDIO_FORMAT_U16_LE   : 
		case AUDIO_FORMAT_U16_BE   : data_format = AUDIO_OUT_LL_DATAFORMAT_16B;
		                             break;
	    case AUDIO_FORMAT_S24_LE   : 
		case AUDIO_FORMAT_S24_BE   :
		case AUDIO_FORMAT_U24_LE   :
		case AUDIO_FORMAT_U24_BE   : data_format = AUDIO_OUT_LL_DATAFORMAT_24B;
		                             break;
		case AUDIO_FORMAT_S32_LE   :
		case AUDIO_FORMAT_S32_BE   :
		case AUDIO_FORMAT_U32_LE   :
		case AUDIO_FORMAT_U32_BE   : data_format = AUDIO_OUT_LL_DATAFORMAT_32B;
		                             break;
		case AUDIO_FORMAT_FLOAT_LE :
		case AUDIO_FORMAT_FLOAT_BE : data_format = AUDIO_OUT_LL_DATAFORMAT_32B;
		                             break;
		default                    : data_format = AUDIO_OUT_LL_DATAFORMAT_16B;
		                             break;

	}

	return data_format;
}

audio_status_t audio_out_set_hw_params(audio_out_hw_params_t *hw_params)
{
	if (NULL == hw_params) {
		audio_error.codec.bad_params = 1;
	}

	audio_out_ll_hw_params_t ll_hw_params = {
		.audio_frequency = hw_params->rate,
		.standard        = AUDIO_OUT_LL_STANDARD_I2S,
        .data_format     = convert_audio_format_to_data_format(hw_params->format),
	};

	if (AUDIO_OUT_LL_OK != audio_out_ll_init(&ll_hw_params)) {
		audio_error.out.init = 1;
	}

	if (CODEC_OK != codec.set_hw_params(&ll_hw_params)) {
		audio_error.codec.hw_params = 1;
	}

	audio_out_hw_params.rate     = hw_params->rate;
	audio_out_hw_params.channels = hw_params->channels;
//	audio_out_hw_params.access   = hw_params->access;
	audio_out_hw_params.format   = hw_params->format;

	audio_status_t retc = ((0 != audio_error.codec.w) || (0 != audio_error.out.w)) ? (AUDIO_OUT_HW_PARAMS_ERROR) : (AUDIO_OK);
	return retc;
}

audio_status_t audio_out_pause(void)
{
	if (AUDIO_OUT_LL_OK != audio_out_ll_pause()) {
		audio_error.out.pause = 1;
	}

	if (CODEC_OK != codec.pause()) {
		audio_error.codec.pause = 1;
	}

	audio_status_t retc = ((0 != audio_error.codec.w) || (0 != audio_error.out.w)) ? (AUDIO_OUT_PAUSE_ERROR) : (AUDIO_OK);
	return retc;
}

audio_status_t audio_out_resume(void)
{
	if (CODEC_OK != codec.resume()) {
		audio_error.codec.resume = 1;
	}

	if (AUDIO_OUT_LL_OK != audio_out_ll_resume()) {
		audio_error.out.resume = 1;
	}

	audio_status_t retc = ((0 != audio_error.codec.w) || (0 != audio_error.out.w)) ? (AUDIO_OUT_RESUME_ERROR) : (AUDIO_OK);
	return retc;
}

audio_status_t audio_out_stop(void)
{
	if (CODEC_OK != codec.stop()) {
		audio_error.codec.stop = 1;
	}

	if (AUDIO_OUT_LL_OK != audio_out_ll_stop()) {
		audio_error.out.stop = 1;
	}

	if (AUDIO_OUT_LL_OK != audio_out_ll_deinit()) {
		audio_error.out.deinit = 1;
	}

	audio_status_t retc = ((0 != audio_error.codec.w) || (0 != audio_error.out.w)) ? (AUDIO_OUT_STOP_ERROR) : (AUDIO_OK);
	return retc;
}

audio_status_t audio_out_set_volume(float volume)
{
	if (CODEC_OK != codec.set_volume(volume)) {
		audio_error.codec.volume = 1;
	}

	audio_status_t retc = (0 != audio_error.codec.w) ? (AUDIO_OUT_VOLUME_ERROR) : (AUDIO_OK);
	return retc;
}

audio_status_t audio_out_set_mute(codec_mute_t mute)
{
	if (CODEC_OK != codec.set_mute(mute)) {
		audio_error.codec.mute = 1;
	}

	audio_status_t retc = (0 != audio_error.codec.w) ? (AUDIO_OUT_MUTE_ERROR) : (AUDIO_OK);
	return retc;
}

audio_status_t audio_out_set_write_callback(audio_out_write_callback_t callback)
{
	if (NULL == callback) {
		audio_error.out.bad_params = 1;
	}

	write_callback = callback;

	audio_status_t retc = (0 != audio_error.out.w) ? (AUDIO_OUT_CALLBACK_ERROR) : (AUDIO_OK);
	return retc;
}

