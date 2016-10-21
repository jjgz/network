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
} MSGMovement;


typedef struct {
    uint8_t x;
    uint8_t y;
} MSGPoint;

typedef struct{
    uint32_t tmr3;
    uint32_t tmr4;
    double l_spd;
    double r_spd;
    int left_spd_avg;
    int right_spd_avg;
}TimerDebug;

typedef struct{
    bool left;
    bool right;
}MSGDebugJoeTread;

#endif
