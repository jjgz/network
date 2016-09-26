#ifndef _RECV_H
#define _RECV_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "buffer.h"
#include "common.h"

#define DBG_LENGTH 24

typedef enum {
    NR_QUERY_STATS,
    NR_GRABBER_GRABBING,
    NR_GRABBER_LIFTING,
    NR_GRABBER_GRABBED,
    NR_GRABBER_LIFTED,
    NR_PATH,
    NR_STOP,
    NR_ADC,
    NR_ROTATION,
    NR_ULTRA_SENSOR,
    NR_PATH_GRAB_FINISH,
    NR_STOP_ACK,
    NR_INVALID_ERROR,
    NR_HELLO_RESPONSE,
    NR_REQ_NAME,
    NR_HELLO_JOSH,
    NR_REQ_HELLO_GEORDON_JOSH,
} NRType;

typedef union {
    MSGQueryStats query_stats;
    MSGPath path;
} NRUnion;

typedef struct {
    NRType type;
    NRUnion data;
} NRMessage;
bool network_recv_add_buffer_from_isr(CharBuffer *buffer);

void network_recv_init();
void network_recv_task();

#endif /* _RECV_H */
