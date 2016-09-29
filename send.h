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
    NS_SEND_NAME_ZAC,
    NS_SEND_NAME_GEO,
    NS_SEND_NAME_JOE,
} NSType;

typedef union {
    MSGNetstats netstats;
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
