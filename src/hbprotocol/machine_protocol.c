/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2018 Simon Hailes <btsimonh@googlemail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
* usage:
* call void protocol_byte( unsigned char byte ); with incoming bytes from main.call
* will call protocol_process_message when message received (protocol.c)
* call protocol_post to send a message
*/

#include "protocol_private.h"

#include <string.h>
#include <stdlib.h>





static unsigned char mpGetTxByte(MACHINE_PROTOCOL_TX_BUFFER *buf);
static char mpGetTxMsg(MACHINE_PROTOCOL_TX_BUFFER *buf, unsigned char *dest);
static void mpPutTx(MACHINE_PROTOCOL_TX_BUFFER *buf, unsigned char value);

//////////////////////////////////////////////////////////////////




#define PROTOCOL_STATE_IDLE 0
#define PROTOCOL_STATE_WAIT_LEN 1
#define PROTOCOL_STATE_WAIT_CI 2
#define PROTOCOL_STATE_WAIT_END 3
#define PROTOCOL_STATE_BADCHAR 4

#define PROTOCOL_ACK_TX_IDLE 0
#define PROTOCOL_ACK_TX_WAITING 1



// private to us
static void protocol_send_nack(int (*send_serial_data)( unsigned char *data, int len ), unsigned char CI, unsigned char som);
static void protocol_send_ack(int (*send_serial_data)( unsigned char *data, int len ), unsigned char CI);
static int protocol_send(PROTOCOL_STAT *s, PROTOCOL_MSG2 *msg);
static void protocol_send_raw(int (*send_serial_data)( unsigned char *data, int len ), PROTOCOL_MSG2 *msg);


// called from main.c
// externed in protocol.h
void protocol_byte(PROTOCOL_STAT *s, unsigned char byte ){

    switch(s->state){
    case PROTOCOL_STATE_BADCHAR:
    case PROTOCOL_STATE_IDLE:
        if ((byte == PROTOCOL_SOM_ACK) || (byte == PROTOCOL_SOM_NOACK)){
            s->curr_msg.SOM = byte;
            s->last_char_time = protocol_GetTick();
            s->state = PROTOCOL_STATE_WAIT_CI;
            s->CS = 0;
        } else {
            if (s->allow_ascii){
                //////////////////////////////////////////////////////
                // if the byte was NOT SOM (02), then treat it as an
                // ascii protocol byte.  BOTH protocol can co-exist
                ascii_byte(s, byte );
                //////////////////////////////////////////////////////
            } else {
                s->last_char_time = protocol_GetTick();
                s->state = PROTOCOL_STATE_BADCHAR;
            }
        }
        break;

    case PROTOCOL_STATE_WAIT_CI:
        s->last_char_time = protocol_GetTick();
        s->curr_msg.CI = byte;
        s->CS += byte;
        s->state = PROTOCOL_STATE_WAIT_LEN;
        break;

    case PROTOCOL_STATE_WAIT_LEN:
        s->last_char_time = protocol_GetTick();
        s->curr_msg.len = byte;
        s->count = 0;
        s->CS += byte;
        s->state = PROTOCOL_STATE_WAIT_END;
        break;

    case PROTOCOL_STATE_WAIT_END:
        s->last_char_time = protocol_GetTick();
        s->curr_msg.bytes[s->count++] = byte;
        s->CS += byte;

        if (s->count == s->curr_msg.len+1){
            s->last_char_time = 0;

            switch(s->curr_msg.SOM) {
            case PROTOCOL_SOM_ACK:
                switch(s->curr_msg.bytes[0]) {
                case PROTOCOL_CMD_ACK:
                    if (s->send_state == PROTOCOL_ACK_TX_WAITING){
                        if (s->curr_msg.CI == s->ack.curr_send_msg.CI){
                            s->ack.last_send_time = 0;
                            s->send_state = PROTOCOL_ACK_TX_IDLE;
                            // if we got ack, then try to send a next message
                            int txcount = mpTxQueued(&s->ack.TxBuffer);
                            if (txcount){
                                // send from tx queue
                                protocol_send(s, NULL);
                            }
                        } else {
                            // ignore
                            s->ack.counters.unwantedacks++;
                            // 'if an ACK is received which contains a CI different to the last sent message, the ACK will be ignored. (?? implications??)'
                        }
                    } else {
                        // ignore, sort of:
                        // 'if an ACK is received which contains the same CI as the last ACK, the ACK will be ignored.'
                        s->ack.counters.unwantedacks++;
                    }
                    break;

                case PROTOCOL_CMD_NACK:
                    // 'If an end receives a NACK, it should resend the last message with the same CI, up to 2 retries'
                    if (s->send_state == PROTOCOL_ACK_TX_WAITING){
                        // ignore CI
                        if (s->ack.retries > 0){
                            s->ack.counters.txRetries++;
                            protocol_send_raw(s->send_serial_data, &s->ack.curr_send_msg);
                            s->ack.last_send_time = protocol_GetTick();
                            s->ack.retries--;
                        } else {
                            s->send_state = PROTOCOL_ACK_TX_IDLE;
                            s->ack.counters.txFailed++;
                            // if we run out of retries, then try to send a next message
                            int txcount = mpTxQueued(&s->ack.TxBuffer);
                            if (txcount){
                                // send from tx queue
                                protocol_send(s, NULL);
                            }
                        }
                    } else {
                        // unsolicited NACK received?
                        s->ack.counters.unwantednacks++;
                        // ignore
                    }
                    break;
                default:
                    if (s->CS != 0){
                        // Checksum invalid. Complain and Discard.
                        protocol_send_nack(s->send_serial_data, s->curr_msg.CI, s->curr_msg.SOM);
                        break;
                    }

                    if (s->ack.lastRXCI == s->curr_msg.CI) {
                        // 'if a message is received with the same CI as the last received message, ACK will be sent, but the message discarded.'
                        protocol_send_ack(s->send_serial_data, s->curr_msg.CI);
                        break;
                    }

                    if( s->curr_msg.CI < s->ack.lastRXCI) {
                        // CI is smaller than a received message. Probably Overflow of CI. (Other case would resending of previous lost message. Shouldn't happen.)
                        s->ack.lastRXCI = s->ack.lastRXCI - 256; // Max value of char is 255
                    }

                    // Add to rxMissing Counter, when CIs where skipped
                    s->ack.counters.rxMissing += s->curr_msg.CI - (s->ack.lastRXCI + 1);

                    // process message
                    protocol_send_ack(s->send_serial_data, s->curr_msg.CI);
                    s->ack.lastRXCI = s->curr_msg.CI;
                    s->ack.counters.rx++;
                    protocol_process_message(s, &(s->curr_msg));
                    break;
                }
                break;

            case PROTOCOL_SOM_NOACK:
                switch(s->curr_msg.bytes[0]) {
                case PROTOCOL_CMD_ACK:
                    // We shouldn't get ACKs in the NoACK protocol..
                    s->noack.counters.unwantedacks++;
                    break;

                case PROTOCOL_CMD_NACK:
                    // 'If an end receives a NACK, it should resend the last message with the same CI, up to 2 retries'
                    if (s->noack.retries > 0){
                        s->noack.counters.txRetries++;
                        protocol_send_raw(s->send_serial_data, &s->noack.curr_send_msg);
                        s->noack.retries--;
                    } else {
                        s->noack.counters.txFailed++;
                        // if we run out of retries, then try to send a next message
                        int txcount = mpTxQueued(&s->noack.TxBuffer);
                        if (txcount){
                            // send from tx queue
                            protocol_send(s, NULL);
                        }
                    }
                    break;
                default:
                    if (s->CS != 0){
                        // Checksum invalid. Complain and Discard.
                        protocol_send_nack(s->send_serial_data, s->curr_msg.CI, s->curr_msg.SOM);
                        break;
                    }

                    if (s->noack.lastRXCI == s->curr_msg.CI) {
                        // if a message is received with the same CI as the last received message, the message is discarded.
                        break;
                    }

                    if( s->curr_msg.CI < s->noack.lastRXCI) {
                        // CI is smaller than a received message. Probably Overflow of CI. (Other case would resending of previous lost message. Shouldn't happen.)
                        s->noack.lastRXCI = s->noack.lastRXCI - 256; // Max value of char is 255
                    }

                    // Add to rxMissing Counter, when CIs where skipped
                    s->noack.counters.rxMissing += s->curr_msg.CI - (s->noack.lastRXCI + 1);

                    // process message
                    s->noack.lastRXCI = s->curr_msg.CI;
                    s->noack.counters.rx++;
                    protocol_process_message(s, &(s->curr_msg));
                    break;
                }
                break;
            }
            s->state = PROTOCOL_STATE_IDLE;
        }
        break;
    }
}


// private
void protocol_send_nack(int (*send_serial_data)( unsigned char *data, int len ), unsigned char CI, unsigned char som){

    // Enforce valid SOM, otherwise Message is discarded by protocol_send_raw.
    // PROTOCOL_SOM_ACK is chosen to ensure backwards compatibilty.
    // If ANY SOM could be send, it would be identified as a badchar, which would trigger a NACK with the same SOM.
    // That's an infinite loop of NACKs with invalid SOM.
    if(som != PROTOCOL_SOM_ACK && som != PROTOCOL_SOM_NOACK) {
        som = PROTOCOL_SOM_ACK;
    }

    char tmp[] = { som, CI, 1, PROTOCOL_CMD_NACK, 0 };
    protocol_send_raw(send_serial_data, (PROTOCOL_MSG2 *)tmp);
}

// private
void protocol_send_ack(int (*send_serial_data)( unsigned char *data, int len ), unsigned char CI){
    char tmp[] = { PROTOCOL_SOM_ACK, CI, 1, PROTOCOL_CMD_ACK, 0 };
    protocol_send_raw(send_serial_data, (PROTOCOL_MSG2 *)tmp);
}


// called to send a message.
// just supply len|bytes - no SOM, CI, or CS
// returns:
//  -1 - cannot even queue
//  0 sent immediately
//  1 queued for later TX
int protocol_post(PROTOCOL_STAT *s, PROTOCOL_MSG2 *msg){


    if(msg->SOM == PROTOCOL_SOM_ACK) {
        int txcount = mpTxQueued(&s->ack.TxBuffer);
        if ((s->send_state != PROTOCOL_ACK_TX_WAITING) && !txcount){

            return protocol_send(s, msg);
        }

        // add to tx queue
        int total = msg->len + 1; // +1 len

        if (txcount + total >= MACHINE_PROTOCOL_TX_BUFFER_SIZE-2) {
            s->ack.TxBuffer.overflow++;
            return -1;
        }

        char *src = (char *) &(msg->len);
        for (int i = 0; i < total; i++) {
            mpPutTx(&s->ack.TxBuffer, *(src++));
        }

        return 1; // added to queue

    } else if(msg->SOM == PROTOCOL_SOM_NOACK) {
        return protocol_send(s, msg);
    }
    return -1;
}


// private
// note: if NULL in, send one message from queue
int protocol_send(PROTOCOL_STAT *s, PROTOCOL_MSG2 *msg){
    if(msg) {
        if(msg->SOM == PROTOCOL_SOM_ACK && s->send_state == PROTOCOL_ACK_TX_WAITING) {
            // Tried to Send Message which requires ACK, but a message is still pending.
            return -1;

        } else if(msg->SOM == PROTOCOL_SOM_ACK && s->send_state == PROTOCOL_ACK_TX_IDLE) {
            // Idling (not waiting for ACK), send the Message directly
            memcpy(&s->ack.curr_send_msg, msg, 1 + 1 + 1 + msg->len); // SOM + CI + Len + Payload
            s->ack.curr_send_msg.CI = ++(s->ack.lastTXCI);
            s->ack.counters.tx++;
            protocol_send_raw(s->send_serial_data, &s->ack.curr_send_msg);
            s->send_state = PROTOCOL_ACK_TX_WAITING;
            s->ack.last_send_time = protocol_GetTick();
            s->ack.retries = 2;
            return 0;

        } else if (msg->SOM == PROTOCOL_SOM_NOACK) {
            // Send Message without ACK immediately
            memcpy(&s->noack.curr_send_msg, msg, 1 + 1 + 1 + msg->len); // SOM + CI + Len + Payload
            s->noack.curr_send_msg.CI = ++(s->noack.lastTXCI);
            s->noack.counters.tx++;
            protocol_send_raw(s->send_serial_data, &s->noack.curr_send_msg);
            return 0;

        } else {
            // Shouldn't happen, unknowm SOM
            return -1;
        }
    } else {
        // No Message was given, work on Buffers then..

        if(s->send_state == PROTOCOL_STATE_IDLE && mpTxQueued(&s->ack.TxBuffer)) {
            // Make sure we are not waiting for another ACK. Check if There is something in the buffer.
            mpGetTxMsg(&s->ack.TxBuffer, &s->ack.curr_send_msg.len);
            s->ack.curr_send_msg.SOM = PROTOCOL_SOM_ACK;
            s->ack.curr_send_msg.CI = ++(s->ack.lastTXCI);
            s->ack.counters.tx++;
            protocol_send_raw(s->send_serial_data, &s->ack.curr_send_msg);
            s->send_state = PROTOCOL_ACK_TX_WAITING;
            s->ack.last_send_time = protocol_GetTick();
            s->ack.retries = 2;
            return 0;

        } else {
            // Do the other queue
            int ismsg = mpGetTxMsg(&s->noack.TxBuffer, &s->noack.curr_send_msg.len);
            if (ismsg){
                // Queue has message waiting
                s->noack.curr_send_msg.SOM = PROTOCOL_SOM_NOACK;
                s->noack.curr_send_msg.CI = ++(s->noack.lastTXCI);
                s->noack.counters.tx++;
                protocol_send_raw(s->send_serial_data, &s->noack.curr_send_msg);
                return 0;
            }
        }
    }
    return -1; // nothing to send
}


// private
void protocol_send_raw(int (*send_serial_data)( unsigned char *data, int len ), PROTOCOL_MSG2 *msg){

    // Enforce validity of SOM.
    if(msg->SOM == PROTOCOL_SOM_ACK || msg->SOM == PROTOCOL_SOM_NOACK) {
    unsigned char CS = 0;
    int i;
    CS -= msg->CI;
    CS -= msg->len;

    for (i = 0; i < msg->len; i++){
        CS -= msg->bytes[i];
    }
    msg->bytes[i] = CS;
    send_serial_data((unsigned char *) msg, msg->len+4);
}
}


// called regularly from main.c
// externed from protocol.h
void protocol_tick(PROTOCOL_STAT *s){
    s->last_tick_time = protocol_GetTick();


    if(s->send_state == PROTOCOL_ACK_TX_WAITING) {
        if ((s->last_tick_time - s->ack.last_send_time) > s->timeout1){
            // 'If an end does not receive an ACK response within (TIMEOUT1), it should resend the last message with the same CI, up to 2 retries'
            if (s->ack.retries > 0){
                s->ack.counters.txRetries++;
                protocol_send_raw(s->send_serial_data, &s->ack.curr_send_msg);
                s->ack.last_send_time = protocol_GetTick();
                s->ack.retries--;
            } else {
                // if we run out of retries, then try to send a next message
                s->send_state = PROTOCOL_ACK_TX_IDLE;
            }
        }
    }

    // send from tx queue
    protocol_send(s, NULL);

    switch(s->state){
        case PROTOCOL_STATE_IDLE:
            break;
        case PROTOCOL_STATE_BADCHAR:
            // 'normally, characters received BETWEEN messages which are not SOM should be treated as ASCII commands.'
            // 'implement a mode where non SOM characters between messages cause (TIMEOUT2) to be started,
            // resulting in a _NACK with CI of last received message + 1_.'
            if ((s->last_tick_time - s->last_char_time) > s->timeout2){
                protocol_send_nack(s->send_serial_data, s->curr_msg.CI+1, s->curr_msg.SOM);
                s->last_char_time = 0;
                s->state = PROTOCOL_STATE_IDLE;
            }
            break;
        default:
            // in a message
            // 'In receive, if in a message (SOM has been received) and the time since the last character
            // exceeds (TIMEOUT2), the incomming message should be discarded,
            // and a NACK should be sent with the CI of the message in progress or zero if no CI received yet'
            if ((s->last_tick_time - s->last_char_time) > s->timeout2){
                protocol_send_nack(s->send_serial_data, s->curr_msg.CI, s->curr_msg.SOM);
                s->last_char_time = 0;
                s->state = PROTOCOL_STATE_IDLE;
            }
            break;
    }


    // process subscriptions
    int len = sizeof(s->subscriptions)/sizeof(s->subscriptions[0]);
    int index = 0;

    // Check if subscription exists.
    for (index = 0; index < len; index++) {
        if( s->subscriptions[index].code != 0 && s->subscriptions[index].count != 0 ) {

            // Check if message is due
            if( s->subscriptions[index].next_send_time <= s->last_tick_time) {

                //If so, pretend that a Read request has arrived.

                PROTOCOL_MSG2 newMsg;
                memset((void*)&newMsg,0x00,sizeof(PROTOCOL_MSG2));
                PROTOCOL_BYTES_READVALS *readvals = (PROTOCOL_BYTES_READVALS *) &(newMsg.bytes);

                readvals->cmd  = PROTOCOL_CMD_READVAL;
                readvals->code = s->subscriptions[index].code;
                newMsg.SOM = s->subscriptions[index].som;
                newMsg.len = sizeof(readvals->cmd) + sizeof(readvals->code) + 1; // 1 for Checksum

                protocol_process_message(s, &newMsg);


                //reschedule job
                s->subscriptions[index].next_send_time = s->last_tick_time + s->subscriptions[index].period;
                if(s->subscriptions[index].count > 0) {
                    s->subscriptions[index].count = s->subscriptions[index].count - 1;
                }

            }


        }
    }

}


int mpTxQueued(MACHINE_PROTOCOL_TX_BUFFER *buf){
    if (buf->head != buf->tail){
        int count = buf->head - buf->tail;
        if (count < 0){
            count += MACHINE_PROTOCOL_TX_BUFFER_SIZE;
        }
        return count;
    }
    return 0;
}

unsigned char mpGetTxByte(MACHINE_PROTOCOL_TX_BUFFER *buf){
    short t = -1;
    if (buf->head != buf->tail){
        t = buf->buff[buf->tail];
        buf->tail = ((buf->tail + 1 ) % MACHINE_PROTOCOL_TX_BUFFER_SIZE);
    }
    return t;
}

char mpGetTxMsg(MACHINE_PROTOCOL_TX_BUFFER *buf, unsigned char *dest){
    if (mpTxQueued(buf)) {
        unsigned char len = *(dest++) = mpGetTxByte(buf); // len of bytes to follow
        for (int i = 0; i < len; i++) {
            *(dest++) = mpGetTxByte(buf); // data
        }
        return 1; // we got a message
    }
    return 0;
}

void mpPutTx(MACHINE_PROTOCOL_TX_BUFFER *buf, unsigned char value){
    buf->buff[buf->head] = value;
    buf->head = ((buf->head + 1 ) % MACHINE_PROTOCOL_TX_BUFFER_SIZE);
}