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

#include "network_recv.h"
#include "processing.h"
#include "debug.h"
#include "network/jsmn/jsmn.h"
#include <string.h>
#define NETWORK_RECV_QUEUE_LEN 14
#define cmp_str_token(sname, token) sizeof(sname)-1 == base_str_len && !strncmp(sname, buffer.buff + recv_tokens[token].start, sizeof(sname)-1)

QueueHandle_t network_recv_queue;

jsmn_parser recv_parser;
jsmntok_t recv_tokens[64];
Point recv_point_ring_buffers[PROCESSING_QUEUE_LEN+2][20];
unsigned recv_ring_buffer_pos;

bool network_recv_add_buffer_from_isr(CharBuffer *buffer) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    // Attempt add the buffer from the isr to the queue.
    if (xQueueSendToBackFromISR(network_recv_queue, buffer, &higher_priority_task_woken)) {
        // If a higher priority task was waiting for something on the queue, switch to it.
        portEND_SWITCHING_ISR(higher_priority_task_woken);
        return true;
    // We didn't receive a buffer.
    } else {
        // Indicate on LD4 that we lost a packet.
        // NOTE: LD4 conflicts with SDA2 (I2C).
        SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
        return false;
    }
    return false;
}

void network_recv_init() {
    network_recv_queue = xQueueCreate(NETWORK_RECV_QUEUE_LEN, sizeof(CharBuffer));
    recv_ring_buffer_pos = 0;
    wifly_int_init();
}

void network_recv_task() {
    CharBuffer buffer;
    NRMessage message;
    int recv_json_length = 0;
    while (1) {
        jsmn_init(&recv_parser);
        xQueueReceive(network_recv_queue, &buffer, portMAX_DELAY);
        // Parse the JSON into objects.
        // TODO: Parse from JSON.
        recv_json_length = jsmn_parse(&recv_parser, buffer.buff, buffer.length, recv_tokens, sizeof(recv_tokens)/sizeof(recv_tokens[0]));
        if(recv_json_length <= 0)
        {
             message.type = NR_INVALID_ERROR;
             processing_add_recvmsg(&message);
             SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
             continue;
        }
        else
        {
            //if it equals "hearbeat"
            switch(recv_tokens[0].type)
            {
                case JSMN_STRING:
                {
                    const char recv_req_netstats[] = "ReqNetstats";
                    const char recv_heartbeat[] = "Heartbeat";
                    const char recv_req_name[] = "ReqName";
                    const char recv_req_movement[] = "ReqMovement";
                    const char recv_req_joe_points[] = "JoeReqPoints";
                    const char recv_req_josh_points[] = "JoshReqPoints";
                    const char recv_req_stopped[] = "ReqStopped";
                    const char recv_req_in_position[] = "ReqInPosition";
                    const char recv_req_EdgeDetect[] = "ReqEdgeDetect";
                    const char recv_req_EdgeDropped[] = "ReqEdgeDropped";
                    const char recv_req_Distance[] = "ReqDistance";
                    const char recv_req_Grabbed[] = "ReqGrabbed";
                    const char recv_req_Dropped[] = "ReqDropped";

                    int base_str_len = recv_tokens[0].end - recv_tokens[0].start;
                     if(cmp_str_token(recv_heartbeat, 0)) {
                     } else if(cmp_str_token(recv_req_netstats, 0)) {
                         message.type = NR_QUERY_STATS;
                         processing_add_recvmsg(&message);
                     } else if(cmp_str_token(recv_req_name, 0)) {
                         message.type = NR_REQ_NAME;
                         processing_add_recvmsg(&message);
                     } else if(cmp_str_token(recv_req_movement, 0)) {
                         message.type = NR_REQ_MOVEMENT;
                         processing_add_recvmsg(&message);
                     } else if(cmp_str_token(recv_req_joe_points, 0)) {
                         message.type = NR_REQ_JOE_POINTS;
                         processing_add_recvmsg(&message);
                     } else if(cmp_str_token(recv_req_josh_points, 0)) {
                         message.type = NR_REQ_JOSH_POINTS;
                         processing_add_recvmsg(&message);
                     } else if(cmp_str_token(recv_req_stopped, 0)) {
                         message.type = NR_REQ_STOPPED;
                         processing_add_recvmsg(&message);
                     } else if(cmp_str_token(recv_req_in_position, 0)) {
                         message.type = NR_REQ_IN_POS;
                         processing_add_recvmsg(&message);
                     } else if(cmp_str_token(recv_req_EdgeDetect, 0)) {
                         message.type = NR_REQ_EDGE_DETECT;
                         processing_add_recvmsg(&message);
                     } else if(cmp_str_token(recv_req_EdgeDropped, 0)) {
                         message.type = NR_REQ_EDGE_DROPPED;
                         processing_add_recvmsg(&message);
                     } else if(cmp_str_token(recv_req_Distance, 0)) {
                         message.type = NR_REQ_DISTANCE;
                         processing_add_recvmsg(&message);
                     } else if(cmp_str_token(recv_req_Grabbed, 0)) {
                         message.type = NR_REQ_GRABBED;
                         processing_add_recvmsg(&message);
                     } else if(cmp_str_token(recv_req_Dropped, 0)) {
                         message.type = NR_REQ_DROPPED;
                         processing_add_recvmsg(&message);
                     }
                     else {
                         message.type = NR_INVALID_ERROR;
                         processing_add_recvmsg(&message);
                     }
                } break;

                case JSMN_OBJECT:
                {
                    const char recv_path[] = "Path";
                    if(recv_tokens[1].type == JSMN_STRING)
                    {
                        int base_str_len = recv_tokens[1].end - recv_tokens[1].start;
                        
                        if(cmp_str_token("Movement", 1))//path
                        {
                         //debug_loc(DEBUG_RECV_GRABBING)
                           if(recv_tokens[2].type == JSMN_OBJECT)
                           {
                               int i;
                               for(i = 0; i < 5; i++)
                               {
                                   if(recv_tokens[3+i*2].type != JSMN_STRING ||
                                           recv_tokens[4+i*2].type != JSMN_PRIMITIVE)
                                   {
                                       message.type = NR_INVALID_ERROR;
                                       processing_add_recvmsg(&message);
                                       continue;
                                   }   
                               }
                               if(!cmp_str_token("x", 3) || !cmp_str_token("y", 5) || !cmp_str_token("v", 7) || !cmp_str_token("angle", 9) || !cmp_str_token("av", 11))
                               {
                                    message.type = NR_INVALID_ERROR;
                                    processing_add_recvmsg(&message);                                    
                               }
                               else
                               {
                                    message.type = NR_MOVEMENT;  
                                    message.movement.x = atof(buffer.buff + recv_tokens[4].start);
                                    message.movement.y = atof(buffer.buff + recv_tokens[6].start);
                                    message.movement.v = atof(buffer.buff + recv_tokens[8].start);
                                    message.movement.angle = atof(buffer.buff + recv_tokens[10].start);
                                    message.movement.av = atof(buffer.buff + recv_tokens[12].start);
                                    processing_add_recvmsg(&message);
                               }
                           }
                           else
                           {
                              message.type = NR_INVALID_ERROR;
                              processing_add_recvmsg(&message);
                           }
                        }
                        else if(cmp_str_token("JF", 1))
                        {
                           if(recv_tokens[2].type == JSMN_PRIMITIVE){
                               message.type = NR_JF; 
                               jsmn_prim(message, buffer);   
                           }
                           
                           else
                           {
                              message.type = NR_INVALID_ERROR;
                              processing_add_recvmsg(&message);
                           }
                        }
                        else if(cmp_str_token("JE", 1))
                        {
                           if(recv_tokens[2].type == JSMN_PRIMITIVE){
                               message.type = NR_JE; 
                               jsmn_prim(message, buffer);   
                           }
                           
                           else
                           {
                              message.type = NR_INVALID_ERROR;
                              processing_add_recvmsg(&message);
                           }
                        }
                        else if(cmp_str_token("CF", 1))
                        {
                           if(recv_tokens[2].type == JSMN_PRIMITIVE){
                               message.type = NR_CF; 
                               jsmn_prim(message, buffer);   
                           }
                           
                           else
                           {
                              message.type = NR_INVALID_ERROR;
                              processing_add_recvmsg(&message);
                           }
                        }
                        else if(cmp_str_token("CE", 1))
                        {
                           if(recv_tokens[2].type == JSMN_PRIMITIVE){
                               message.type = NR_CE; 
                               jsmn_prim(message, buffer);   
                           }
                           
                           else
                           {
                              message.type = NR_INVALID_ERROR;
                              processing_add_recvmsg(&message);
                           }
                        }
                        else if(cmp_str_token("CT", 1))
                        {
                           if(recv_tokens[2].type == JSMN_PRIMITIVE){
                               message.type = NR_CT; 
                               jsmn_prim(message, buffer);   
                           }
                           
                           else
                           {
                              message.type = NR_INVALID_ERROR;
                              processing_add_recvmsg(&message);
                           }
                        }
                        else
                        {
                           message.type = NR_INVALID_ERROR;
                           processing_add_recvmsg(&message);
                        }
                    }
                    else
                    {
                        message.type = NR_INVALID_ERROR;
                        processing_add_recvmsg(&message);
                    }
                } break;

                default:
                    break;
            }//end of switch
        }
      // Assume the object is a stat query.
    }
}

void jsmn_prim(NRMessage msg, CharBuffer buff)
{
    char *point_string = buff.buff + recv_tokens[2].start;
    long xy = atol(point_string);
    msg.data.point.x = xy % 128;
    msg.data.point.y = xy / 128;
    processing_add_recvmsg(&msg);
}