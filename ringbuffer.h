#ifndef RINGBUFFER_H
#define RINGBUFFER_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
void ringbuffer_rst(void);
void ringbuffer_add(uint8_t *dat, unsigned len);
unsigned ringbuffer_get(uint8_t *dst, unsigned len);
unsigned ringbuffer_overflow(void);
#ifdef __cplusplus
}
#endif
#endif
