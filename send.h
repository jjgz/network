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
    NS_REQ_STOPPED,
    NS_STOPPED,
    NS_REQ_IN_POS,
    NS_IN_POS,
    NS_REQ_EDGE_DETECT,
    NS_EDGE_DETECT,
    NS_REQ_EDGE_DROPPED,
    NS_EDGE_DROPPED,
    NS_REQ_DISTANCE,
    NS_DISTANCE,
    NS_REQ_GRABBED,
    NS_GRABBED,
    NS_REQ_DROPPED,
    NS_DROPPED,
    NS_TMR,
    NS_DEBUG_GEORDON_ADC,
    NS_PWM,
    NS_ROVER_DATA,
    NS_TEST_ROW,
} NSType;

typedef union {
    MSGNetstats netstats;
    MSGMovement movement;
    weight_array w_array[64];
    MSGPoint point;
    bool answer;
    double distance;
    TimerDebug tmr;
    unsigned adc_reading;
    rover_debug rd;
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
