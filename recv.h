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
} NRType;

typedef union {
    bool dummy;
} NRUnion;

typedef struct {
    NRType type;
    NRUnion data;
} NRMessage;
bool network_recv_add_buffer_from_isr(CharBuffer *buffer);

void network_recv_init();
void network_recv_task();

#endif /* _RECV_H */
