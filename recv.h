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
    NR_MOVEMENT,
    NR_STOPPED,
    NR_IN_POS,
    NR_EDGE_DETECT,
    NR_EDGE_DROPPED,
    NR_GRABBED,
    NR_DROPPED,
    NR_DISTANCE,
    NR_DEBUG_JOE_TREAD,
} NRType;

typedef union {
    MSGPoint point;
    MSGMovement movement;
	double distance;
    TimerDebug tmr;
    bool answer;
    bool left_mvmnt;
    bool right_mvmnt;
    MSGDebugJoeTread debug_joe_tread;
} NRUnion;

typedef struct {
    NRType type;
    NRUnion data;
} NRMessage;
bool network_recv_add_buffer_from_isr(CharBuffer *buffer);
void network_recv_init();
void network_recv_task();
void jsmn_prim(NRMessage msg, CharBuffer buff);

#endif /* _RECV_H */
