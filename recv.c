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
const char recv_heartbeat[] = "Heartbeat";
const char recv_req_netstats[] = "RequestNetstats";
const char recv_grabber_grabbing[] = "ConfirmGrabberGrabbing";
const char recv_grabber_lifting[] = "ConfirmGrabberLifting";
const char recv_grabber_grabbed[] = "ConfirmGrabberGrabbed";
const char recv_grabber_lifted[] = "ConfirmGrabberLifted";
const char recv_path[] = "PATH";
const char recv_stop[] = "STOP";
const char recv_points[] = "points";
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
        if(recv_json_length < 0)
        {
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
                     int base_str_len = recv_tokens[0].end - recv_tokens[0].start;
                     if(cmp_str_token(recv_heartbeat, 0))
                     {
                         //dont do anything on a heartbeat
                        // debug_loc(DEBUG_RECV_HEARTBEAT);
                     }
                     else if(cmp_str_token(recv_req_netstats, 0))
                     {
                        // debug_loc(DEBUG_RECV_NETSTAT);
                         message.type = NR_QUERY_STATS;
                         processing_add_recvmsg(&message);
                     }
                     else if(cmp_str_token(recv_grabber_grabbing, 0))
                     {
                         //debug_loc(DEBUG_RECV_GRABBING);
                         message.type = NR_GRABBER_GRABBING;
                         processing_add_recvmsg(&message);
                     }
                     else if(cmp_str_token(recv_grabber_lifting, 0))
                     {
                         //debug_loc(DEBUG_RECV_LIFTING);
                         message.type = NR_GRABBER_LIFTING;
                         processing_add_recvmsg(&message);
                     }
                     else if(cmp_str_token(recv_grabber_grabbed, 0))
                     {
                         //debug_loc(DEBUG_RECV_GRABBING);
                         message.type = NR_GRABBER_GRABBED;
                         processing_add_recvmsg(&message);
                     }
                     else if(cmp_str_token(recv_grabber_lifted, 0))
                     {
                         //debug_loc(DEBUG_RECV_LIFTING);
                         message.type = NR_GRABBER_LIFTED;
                         processing_add_recvmsg(&message);
                     }
                     else if(cmp_str_token(recv_stop, 0))
                     {
                         //debug_loc(DEBUG_RECV_LIFTING);
                         message.type = NR_STOP;
                         processing_add_recvmsg(&message);
                     }
                     else
                     {
                         SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                     }
                } break;
                
                case JSMN_OBJECT:
                {
                    if(recv_tokens[1].type == JSMN_STRING)
                    {
                        int base_str_len = recv_tokens[1].end - recv_tokens[1].start;
                        if(cmp_str_token(recv_path, 1))//path
                        {
                         //debug_loc(DEBUG_RECV_GRABBING)
                           if(recv_tokens[2].type == JSMN_OBJECT)
                           {
                               if(cmp_str_token(recv_points, 3)) //if points
                               {
                                   if(recv_tokens[4].type == JSMN_ARRAY)
                                   {
                                       int point_amount = recv_tokens[4].size/3;
                                       int i;
                                       for(i = 0; i < point_amount;i++)
                                       {
                                           if(recv_tokens[5+i*3].type == JSMN_ARRAY)
                                           {
                                               if(recv_tokens[6+i*3].type == JSMN_PRIMITIVE && recv_tokens[7+i*3].type == JSMN_PRIMITIVE)
                                               {
                                                   recv_point_ring_buffers[recv_ring_buffer_pos][i].x = atof(buffer.buff + recv_tokens[6+i*3].start);
                                                   recv_point_ring_buffers[recv_ring_buffer_pos][i].y = atof(buffer.buff + recv_tokens[7+i*3].start);
                                               }
                                               else
                                               {
                                                   SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                                               }
                                           }
                                           else
                                           {
                                               SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                                           }
                                       }
                                       message.data.path.points.buff = recv_point_ring_buffers[recv_ring_buffer_pos];
                                       message.data.path.points.length = point_amount;
                                       message.type = NR_PATH;
                                       recv_ring_buffer_pos++;
                                       recv_ring_buffer_pos %= PROCESSING_QUEUE_LEN+2;
                                       processing_add_recvmsg(&message);
                                   }
                                   else
                                   {
                                       SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                                   }
                               }
                               else
                               {
                                   SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                               }
                           }
                           else
                           {
                               SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                           }
                        }   
                        else
                        {
                            SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);   
                        }
                    }
                    else
                    {
                         SYS_PORTS_PinWrite(0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
                    }
                } break;
            }
        }
      // Assume the object is a stat query.
    }
}
