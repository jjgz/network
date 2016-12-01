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
    NR_INITIALIZE,
    NR_REQ_NAME,
    NR_REQ_MOVEMENT,
    NR_REQ_STOPPED,
    NR_REQ_IN_POS,
    NR_REQ_EDGE_DETECT,
    NR_REQ_EDGE_DROPPED,
    NR_REQ_DISTANCE,
    NR_REQ_GRABBED,
    NR_REQ_DROPPED,
    NR_MOVEMENT,
    NR_STOPPED,
    NR_IN_POS,
    NR_EDGE_DETECT,
    NR_EDGE_DROPPED,
    NR_GRABBED,
    NR_DROPPED,
    NR_DISTANCE,
    NR_DEBUG_JOE_TREAD,
    NR_TEST_RESET,
    NR_TEST_ROTATE,
    NR_TEST_MOVE,
    NR_TEST_ROW,
    NR_GD_REQ_HALF_ROW,
    NR_GD_REQ_PING,
    NR_REQ_JOSH_POINTS,
    NR_HALF_ROW,
    NR_REQ_HALF_ROW,
    NR_JC_REQ_HALF_ROW,
    NR_JC_HALF_ROW,
    NR_GD_BUILD,
    NR_GD_FINISH,
    NR_GD_ALIGNED,
    NR_DEBUG_JG_ULTRA,
    NR_SENSORS,
} NRType;

typedef union {
    MSGPoint point;
    MSGInitialize initialization;
    MSGMovement movement;
    uint8_t w_array[64];
	double distance;
    TimerJGDebug tm3r;
    SensorReading ult_photo;    
    MSGDebugJoeTread debug_joe_tread;
    uint8_t rotate_val;
    uint8_t move_val;
    bool answer;
    TimerDebug tmr;
    uint8_t half_row;
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
