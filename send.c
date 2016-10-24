// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

#include "send.h"
#include "int_wifly.h"
#include "int_adc.h"
#include "debug.h"
#include "processing.h"
#include <stdio.h>

#define NETWORK_SEND_QUEUE_LEN 16
#define MESSAGE_BUF_SIZE 512
#define TOTAL_MESSAGE_BUFFS 4
rover my_rover;
QueueHandle_t network_send_queue;

char messagebuffs[TOTAL_MESSAGE_BUFFS][MESSAGE_BUF_SIZE];
char *messagebuff;
unsigned choose_buff;

void network_send_add_message(NSMessage *message) {
    xQueueSendToBack(network_send_queue, message, portMAX_DELAY);
}

void next_messagebuff() {
    messagebuff = messagebuffs[(choose_buff++) % TOTAL_MESSAGE_BUFFS];
}

void network_send_init() {
    messagebuff = messagebuffs[0];
    choose_buff = 0;
    network_send_queue = xQueueCreate(NETWORK_SEND_QUEUE_LEN, sizeof(NSMessage));
    int_adc_init();
}

void network_send_task() {
    NSMessage message;
    CharBuffer buffer;
    while (1) {
        xQueueReceive(network_send_queue, &message, portMAX_DELAY);
        switch (message.type) {
            case NS_NETSTATS: {
                MSGNetstats *netstats = &message.data.netstats;
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"Netstats\":{\"myName\":\"%s\",\"numGoodMessagesRecved\":%d,\"numCommErrors\":%d,\"numJSONRequestsRecved\":%d,\"numJSONResponsesRecved\":%d,\"numJSONRequestsSent\":%d,\"numJSONResponsesSent\":%d}}",
                        MY_NAME,
                        netstats->numGoodMessagesRecved,
                        netstats->numCommErrors,
                        netstats->numJSONRequestsRecved,
                        netstats->numJSONResponsesRecved,
                        netstats->numJSONRequestsSent,
                        netstats->numJSONResponsesSent);
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            } break;
            case NS_SEND_NAME_JOSH:
            {
                my_rover.bools.got_name = true;
                buffer.buff = "\"NameJosh\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_SEND_NAME_ZACH:
            {
                buffer.buff = "\"NameZach\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;            
            case NS_SEND_NAME_GEO:
            {
                buffer.buff = "\"NameGeordon\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_SEND_NAME_JOE:
            {
                buffer.buff = "\"NameJoe\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_REQ_MOVEMENT:
            {
                buffer.buff = "\"ReqMovement\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_MOVEMENT:
            {
                MSGMovement *movement = &message.data.movement;
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"Movement\":{\"x\":%f,\"y\":%f,\"v\":%f,\"angle\":%f,\"av\":%f}}",
                        movement->x,
                        movement->y,
                        movement->v,
                        movement->angle,
                        movement->av);
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_JOE_REQ_POINTS:
            {
                buffer.buff = "\"JoeReqPoints\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_JF:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"JF\":%d}",
                        (uint32_t)message.data.point.x + 128 * (uint32_t)message.data.point.y);
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_JE:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"JE\":%d}",
                        (uint32_t)message.data.point.x + 128 * (uint32_t)message.data.point.y);
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_JOSH_REQ_POINTS:
            {
                buffer.buff = "\"JoshReqPoints\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_CF:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"CF\":%d}",
                        (uint32_t)message.data.point.x + 128 * (uint32_t)message.data.point.y);
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_CE:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"CE\":%d}",
                        (uint32_t)message.data.point.x + 128 * (uint32_t)message.data.point.y);
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_CT:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"CT\":%d}",
                        (uint32_t)message.data.point.x + 128 * (uint32_t)message.data.point.y);
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_REQ_STOPPED:
            {
                buffer.buff = "\"ReqStopped\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_STOPPED:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"Stopped\":%s}",
                        message.data.answer ? "true" : "false");
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_REQ_IN_POS:
            {
                buffer.buff = "\"ReqInPosition\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_IN_POS:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"InPosition\":%s}",
                        message.data.answer ? "true" : "false");
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_REQ_EDGE_DETECT:
            {
                buffer.buff = "\"ReqEdgeDetect\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_EDGE_DETECT:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"EdgeDetect\":%s}",
                        message.data.answer ? "true" : "false");
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_REQ_EDGE_DROPPED:
            {
                buffer.buff = "\"ReqEdgeDropped\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_EDGE_DROPPED:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"EdgeDropped\":%s}",
                        message.data.answer ? "true" : "false");
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_REQ_DISTANCE:
            {
                buffer.buff = "\"ReqDistance\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_DISTANCE:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"Distance\":%f}",
                        message.data.distance);
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_REQ_GRABBED:
            {
                buffer.buff = "\"ReqGrabbed\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_GRABBED:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"Grabbed\":%s}",
                        message.data.answer ? "true" : "false");
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_REQ_DROPPED:
            {
                buffer.buff = "\"ReqDropped\"";
                buffer.length = strlen(buffer.buff);
                wifly_int_send_buffer(&buffer);
            }break;
            case NS_DROPPED:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"Dropped\":%s}",
                        message.data.answer ? "true" : "false");
                
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_PWM:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"PDebugJosh\":[%u, %u, %u, %u, %u, %u]}", 
                        message.data.tmr.speed_left, 
                        message.data.tmr.speed_right,
                        message.data.tmr.tmr4,
                        message.data.tmr.tmr3,
                        message.data.tmr.speed_left,
                        message.data.tmr.speed_right);
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
            case NS_ROVER_DATA:
            {
                buffer.buff = messagebuff;
                buffer.length = sprintf(messagebuff, "{\"ADebugJosh\":[%u, %u, %u, %u]}", 
                        message.data.rd.point.x, 
                        message.data.rd.point.y,
                        message.data.rd.ori,
                        message.data.rd.target);
                if (buffer.length > 0) {
                    wifly_int_send_buffer(&buffer);
                    next_messagebuff();
                } else {
                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                }
            }break;
//            case NS_TMR:
//            {
//                buffer.buff = messagebuff;
//                buffer.length = sprintf(messagebuff, "{\"TDebugJosh\":[%u, %u]}", message.data.tmr.tmr3, message.data.tmr.tmr4);
//                if (buffer.length > 0) {
//                    wifly_int_send_buffer(&buffer);
//                    next_messagebuff();
//                } else {
//                    SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
//                }
//                
//            }break;
            default:
                break;
        }
    }
    
}
