#include "ringbuffer.h"
#define SIZE 1024

static unsigned next = 0;
static unsigned used = 0;
static unsigned overflow = 0;
static uint8_t data[SIZE];

void ringbuffer_add(uint8_t *dat, unsigned len) {
    if (next + len > SIZE) {
        unsigned partial = SIZE - next;
        ringbuffer_add(dat, partial);
        dat += partial;
        len -= partial;
        next = 0;
    }
    for (unsigned i = 0; i < len; i++) {
        data[next + i] = *dat;
        dat++;
    }
    used += len;
    next += len;
    if (used > SIZE) {
        overflow += used - SIZE;
        used = SIZE;
    }
}

unsigned ringbuffer_get(uint8_t *dst, unsigned len) {
    if (len > used) {
        len = used;
    }
    unsigned ret = len;
    if (len > next) {
        unsigned partial = len - next;
        ringbuffer_get(dst, partial);
        dst += partial;
        len -= partial;
    }
    for (unsigned i = next - len; i < next; i++) {
        *dst = data[i];
        dst++;
    }
    used -= len;
    return ret;
}

void ringbuffer_rst(void) {
    next = used = overflow = 0;
}

unsigned ringbuffer_overflow(void) {
    unsigned v = overflow;
    overflow = 0;
    return v;
}
