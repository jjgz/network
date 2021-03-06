#ifndef _SEND_H
#define _SEND_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "buffer.h"
#include "common.h"

typedef enum {
    NS_NETSTATS,
    NS_SEND_NAME_JOSH,
    NS_SEND_NAME_ZACH,
    NS_SEND_NAME_GEO,
    NS_SEND_NAME_JOE,
    NS_REQ_MOVEMENT,
    NS_MOVEMENT,
    NS_REQ_ASSUMED,
    NS_ASSUMED,
    NS_REQ_TARGETS,
    NS_TARGETS,
    NS_REQ_HALF_ROW,
    NS_HALF_ROW,
    NS_REQ_STOPPED,
    NS_STOPPED,
    NS_REQ_IN_POS,
    NS_IN_POS,
    NS_REQ_EDGE_DETECT_LEFT,
            NS_REQ_EDGE_DETECT_RIGHT,
    NS_EDGE_DETECT,
    NS_REQ_EDGE_DROPPED,
    NS_EDGE_DROPPED,
    NS_REQ_DISTANCE,
    NS_DISTANCE,
    NS_REQ_GRABBED,
    NS_GRABBED,
    NS_REQ_DROPPED,
    NS_DROPPED,
    NS_DEBUG_OC,
    NS_DEBUG_JOE_DISTANCE,
    NS_TMR,
    NS_DEBUG_GEORDON_ADC,
    NS_DEBUG_GEORDON_STR,
    NS_GD_HALF_ROW,
    NS_GD_PING,
    NS_PWM,
    NS_ROVER_DATA,
    NS_TEST_ROW,
    NS_TEST_REQ_GRABBED,
    NS_TEST_REQ_DROPPED,
    NS_JOSH_REQ_POINTS,
    NS_JC_REQ_HALF_ROW,
    NS_JG_REQ_ULTRA,
    NS_JG_REQ_SCAN_DONE,
            NS_REQ_PROXIMITY,
            NS_PROXIMITY,
} NSType;

typedef union {
    MSGNetstats netstats;
    MSGMovement movement;
    MSGProximity proximity;
    uint8_t w_array[64];
    MSGPoint point;
    bool answer;
    //double left_speed;
    //double right_speed;
    //int distance;
    ldr_move ldr_m;
    TimerJGDebug tm3r;
    //bool left_mvmnt;
    //bool right_mvmnt;
    double distance;
    TimerDebug tmr;
    unsigned adc_reading;
    rover_debug rd;
    const char *dbstr;
    uint8_t row_req;
} NSUnion;

typedef struct {
    NSType type;
    NSUnion data;
} NSMessage;

void network_send_add_message(NSMessage *message);
void network_send_add_message_isr(NSMessage *message);

void network_send_init();
void network_send_task();


#endif /* _SEND_H */
