#ifndef _NETWORK_COMMON_H
#define _NETWORK_COMMON_H

typedef struct {
    unsigned numGoodMessagesRecved;
    unsigned numCommErrors;
    unsigned numJSONRequestsRecved;
    unsigned numJSONResponsesRecved;
    unsigned numJSONRequestsSent;
    unsigned numJSONResponsesSent;
} MSGNetstats;

typedef struct {
    float x, y, v, angle, av;
} MSGWorld;


typedef struct {
    uint8_t x;
    uint8_t y;
} MSGPoint;

#endif
