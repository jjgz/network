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
    uint32_t l_spd;
    uint32_t r_spd;
    uint32_t tmr_state;
}TimerJGDebug;

typedef struct{
    bool left;
    bool right;
}MSGDebugJoeTread;

typedef struct{
    uint8_t weight;
}weight_array;

typedef struct{
    uint32_t tmr3;
    uint32_t tmr4;
    uint32_t speed_left;
    uint32_t speed_right;
    double left_error;
    double right_error;
}TimerDebug;

typedef enum{
    NORTH = 1,
    SOUTH,
    EAST,
    WEST,
}orientation;

typedef struct{
    orientation ori;
    MSGPoint point;
    uint32_t target;    
}rover_debug;
#endif
