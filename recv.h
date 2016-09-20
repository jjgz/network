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

typedef enum {
    NR_QUERY_STATS
} NRType;

typedef union {
    MSGQueryStats query_stats;
} NRUnion;

typedef struct {
    NRType type;
    NRUnion data;
} NRMessage;

bool network_recv_add_buffer_from_isr(CharBuffer *buffer);

void network_recv_init();
void network_recv_task();

#endif /* _RECV_H */
