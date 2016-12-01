#ifndef _NETWORK_COMMON_H
#define _NETWORK_COMMON_H

typedef struct {
    float x, y;
} BasicPoint;

typedef struct {
    unsigned numGoodMessagesRecved;
    unsigned numCommErrors;
    unsigned numJSONRequestsRecved;
    unsigned numJSONResponsesRecved;
    unsigned numJSONRequestsSent;
    unsigned numJSONResponsesSent;
} MSGNetstats;

typedef struct {
    /// Number of targets.
    unsigned nt;
    /// Number of targets.
    BasicPoint ra;
    /// Number of arena border points.
    unsigned nb;
    /// Arena borders.
    BasicPoint points[14];
} MSGInitialize;

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

typedef struct{
    double ultra;
    double l_photo;
    double r_photo;
    double prev_lphoto;
    double prev_rphoto;
}SensorReading;

typedef struct{
    double dist_x;
    double dist_y;
    double turn_angle;
    double angle_var;
    double move_var;
}ldr_move;
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

typedef struct {
    double left_ir, right_ir;
} MSGProximity;

#endif
