#include "audio_io.h"
#include "cs43l22.h"

void audio_out_write_callback(uint16_t *address, const audio_out_cb_id_t callback_id)
{
    switch (callback_id)
    {
        case AUDIO_OUT_TX_HALF_COMPLETE_CB_ID : {
            // read()
        } break;

        case AUDIO_OUT_TX_COMPLETE_CB_ID : {
            //audio_status_t audio_out_write(uint16_t *data, const uint16_t size);
            // read()
        } break;

        default : {
            // error()
        } break;
    }
}


