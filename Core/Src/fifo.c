#include "fifo.h"

void fifo_init(fifo_t *fifo)
{
    for (uint32_t i = 0; i < FIFO_DEPTH; i++) {
        for (uint32_t j = 0; j < FIFO_ELEMENT_MAX_SIZE; j++) {
            fifo->data[i].element[j] = 0;
        }
    }

    for (uint32_t i = 0; i < FIFO_DEPTH; i++) {
        fifo->size[i] = 0;
    }

    fifo->r = 0;
    fifo->w = 0;
    fifo->count = 0;
}

bool fifo_is_empty(fifo_t *fifo)
{
    bool ret = (0 == fifo->count) ? (true) : (false);
    return ret;
}

bool fifo_is_full(fifo_t *fifo)
{
    bool ret = (FIFO_DEPTH == fifo->count) ? (true) : (false);
    return ret;
}

uint32_t fifo_count(fifo_t *fifo)
{
    return (fifo->count);
}

void fifo_push(fifo_t *fifo, uint8_t *ptr, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        fifo->data[fifo->w].element[i] = (uint8_t)ptr[i];
    }

    fifo->size[fifo->w] = (uint32_t)len;
    fifo->w = fifo->w + 1;
    fifo->count = fifo->count + 1;
}

uint32_t fifo_pull(fifo_t *fifo, uint8_t *ptr)
{
    uint32_t len = fifo->size[fifo->r];

    for (uint32_t i = 0; i < len; i++) {
        ptr[i] = fifo->data[fifo->r].element[i];
    }

    fifo->size[fifo->r] = 0;
    fifo->r = fifo->r + 1;
    fifo->count = fifo->count - 1;

    return len;
}

