#include "audio.h"

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

uint16_t audio_buffer[1024];

void audio_out_ll_write_callback(uint16_t *address, const audio_out_ll_cb_id_t callback_id)
{
    switch (callback_id)
    {
        case AUDIO_OUT_TX_HALF_COMPLETE_CB_ID : {

            // read()

        } break;

        case AUDIO_OUT_TX_COMPLETE_CB_ID : {

        	// read()

        	if (AUDIO_IO_OK != audio_out_ll_write(&audio_buffer[0], 1024)) {
            	while (1);
            }

        } break;

        default : {

            // error()

        } break;
    }
}

void audio_out_init(void)
{
	audio_out_ll_hw_params_t hw_params;
	hw_params.audio_frequency = 44100;
	hw_params.data_format     = AUDIO_OUT_DATAFORMAT_16B;
	hw_params.standard        = AUDIO_OUT_STANDARD_LEFT_JUSTIFIED;

	if (CODEC_OK != codec.init()) {
		while (1);
	}

	if (AUDIO_IO_OK != audio_out_ll_init(&hw_params)) {
		while (1);
	}

	if (CODEC_OK != codec.set_hw_params(&hw_params)) {
		while (1);
	}

	for (uint32_t i = 0; i < 1024; i++) {
		audio_buffer[i] = 0;
	}

	audio_out_ll_cb_params_t cb_params;
	cb_params.m0_buffer      = &audio_buffer[0];
	cb_params.m1_buffer      = &audio_buffer[512];
	cb_params.write_callback = audio_out_ll_write_callback;

	if (true != audio_out_ll_set_cb_params(&cb_params)) {
		while (1);
	}

	//if (AUDIO_IO_OK != audio_out_ll_write(&audio_buffer[0], 1024)) {
	//	while (1);
	//}
}

