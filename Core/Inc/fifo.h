#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>
#include <stdbool.h>

typedef struct fifo_element {
	uint8_t element[128] __attribute__((aligned(128)));
} fifo_element_t;

typedef struct {
	fifo_element_t data[16];
	uint32_t size[16];
	uint32_t r : 4;
	uint32_t   : 28;
	uint32_t w : 4;
	uint32_t   : 28;
	uint32_t count;
} fifo_t;

typedef enum {
	FIFO_ELEMENT_MAX_SIZE = 128U,
	FIFO_DEPTH            = 16U,
} fifo_const_t;

void     fifo_init(fifo_t *fifo);
bool     fifo_is_empty(fifo_t *fifo);
bool     fifo_is_full(fifo_t *fifo);
uint32_t fifo_count(fifo_t *fifo);
void     fifo_push(fifo_t *fifo, uint8_t *ptr, uint32_t len);
uint32_t fifo_pull(fifo_t *fifo, uint8_t *ptr);

#endif // FIFO_H
