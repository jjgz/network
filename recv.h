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
    NR_INVALID_ERROR,
    NR_QUERY_STATS,
    NR_REQ_NAME,
            NR_REQ_MOVEMENT,
            NR_REQ_JOSH_POINTS,
            NR_REQ_JOE_POINTS,
            NR_REQ_STOPPED,
            NR_REQ_IN_POS,
            NR_REQ_EDGE_DETECT,
            NR_REQ_EDGE_DROPPED,
            NR_REQ_DISTANCE,
            NR_REQ_GRABBED,
            NR_REQ_DROPPED,
			NR_JF,
			NR_JE,
			NR_CF,
			NR_CE,
			NR_CT,
} NRType;

typedef struct {
    bool dummy;
    MSGPoints points;
} NRUnion;

typedef struct {
    NRType type;
    NRUnion data;
} NRMessage;
bool network_recv_add_buffer_from_isr(CharBuffer *buffer);

void network_recv_init();
void network_recv_task();

#endif /* _RECV_H */
