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

#include "protocol_private.h"
#include <string.h>
#include <stdlib.h>


///////////////////////////////////////////////////////
// Function Pointers to system functions
//////////////////////////////////////////////////////////

// Need to be assigned to functions "real" system fucntions
uint32_t noTick(void) { return 0; };
uint32_t (*protocol_GetTick)() = noTick;

void noDelay(uint32_t Delay) {};
void (*protocol_Delay)(uint32_t Delay) = noDelay;

void noReset(void) {};
void (*protocol_SystemReset)() = noReset;



static int initialised_functions = 0;

// forward declaration
extern PARAMSTAT *params[];
//////////////////////////////////////////////
// variables you want to read/write here. Only common used ones, specific ones below.

// Default temporary storage for received values
unsigned char contentbuf[sizeof( ((PROTOCOL_BYTES_WRITEVALS *)0)->content )];

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
// Default function, wipes receive memory before writing (and readresponse is just a differenct type of writing)


void fn_preWriteClear ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_PRE_WRITE:
        case FN_TYPE_PRE_READRESPONSE:
            // ensure clear in case of short write
            memset(param->ptr, 0, param->len);
            break;
    }
}


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x00 version

static int version = 1;


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x22 SubscribeData

void fn_SubscribeData ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {

    fn_preWriteClear(s, param, fn_type, content, len); // Wipes memory before write (and readresponse is just a differenct type of writing)

    switch (fn_type) {

        case FN_TYPE_POST_WRITE:
        case FN_TYPE_POST_READRESPONSE:

            // Check if length of received data is plausible.
            if(len != sizeof(PROTOCOL_SUBSCRIBEDATA)) {
                break;
            }

            int subscriptions_len = sizeof(s->subscriptions)/sizeof(s->subscriptions[0]);
            int index = 0;

            // Check if subscription already exists for this code
            for (index = 0; index < subscriptions_len; index++) {
                if(s->subscriptions[index].code == ((PROTOCOL_SUBSCRIBEDATA*) content)->code) {
                    break;
                }
            }

            // If code was not found, look for vacant subscription slot
            if(index == subscriptions_len) {
                for (index = 0; index < subscriptions_len; index++) {
                    // NOTE: if you set a count of 0, or the count runs out, then
                    // the subscription will be overwritten later -
                    // i.e. you effectively delete it....
                    if( s->subscriptions[index].code == 0 || s->subscriptions[index].count == 0 ) {
                        break;
                    }
                }
            }

            // Fill in new subscription when possible; Plausibility check for period
            if(index < subscriptions_len && ((PROTOCOL_SUBSCRIBEDATA*) content)->period >= 10) {
                s->subscriptions[index] = *((PROTOCOL_SUBSCRIBEDATA*) content);
                //char tmp[100];
                //sprintf(tmp, "subscription added at %d for 0x%x, period %d, count %d, som %d\n", index, ((SUBSCRIBEDATA*) (param->ptr))->code, ((SUBSCRIBEDATA*) (param->ptr))->period, ((SUBSCRIBEDATA*) (param->ptr))->count, ((SUBSCRIBEDATA*) (param->ptr))->som);
                //consoleLog(tmp);
            } else {
                // TODO. Inform sender??
                // consoleLog("no subscriptions available\n");
            }
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x22 and 0x23 ProtocolcountData

PROTOCOLCOUNT ProtocolcountData =  { .rx = 0 };

void fn_ProtocolcountDataSum ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {

    fn_preWriteClear(s, param, fn_type, content, len); // Wipes memory before write (and readresponse is just a differenct type of writing)

    switch (fn_type) {

        case FN_TYPE_PRE_READ:
            ProtocolcountData.rx                  = s->ack.counters.rx                  + s->noack.counters.rx;
            ProtocolcountData.rxMissing           = s->ack.counters.rxMissing           + s->noack.counters.rxMissing;
            ProtocolcountData.tx                  = s->ack.counters.tx                  + s->noack.counters.tx;
            ProtocolcountData.txFailed            = s->ack.counters.txFailed            + s->noack.counters.txFailed;
            ProtocolcountData.txRetries           = s->ack.counters.txRetries           + s->noack.counters.txRetries;
            ProtocolcountData.unwantedacks        = s->ack.counters.unwantedacks        + s->noack.counters.unwantedacks;
            ProtocolcountData.unknowncommands     = s->ack.counters.unknowncommands     + s->noack.counters.unknowncommands;
            ProtocolcountData.unplausibleresponse = s->ack.counters.unplausibleresponse + s->noack.counters.unplausibleresponse;
            ProtocolcountData.unwantednacks       = s->ack.counters.unwantednacks       + s->noack.counters.unwantednacks;
            break;

        case FN_TYPE_POST_WRITE:
            s->ack.counters = ProtocolcountData;
            s->noack.counters = ProtocolcountData;
            break;
    }
}

void fn_ProtocolcountDataAck ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {

    fn_preWriteClear(s, param, fn_type, content, len); // Wipes memory before write (and readresponse is just a differenct type of writing)

    switch (fn_type) {
        case FN_TYPE_PRE_READ:
            ProtocolcountData = s->ack.counters;
            break;

        case FN_TYPE_POST_WRITE:
            s->ack.counters = ProtocolcountData;
            break;
    }
}

void fn_ProtocolcountDataNoack ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {

    fn_preWriteClear(s, param, fn_type, content, len); // Wipes memory before write (and readresponse is just a differenct type of writing)

    switch (fn_type) {
        case FN_TYPE_PRE_READ:
            ProtocolcountData = s->noack.counters;
            break;

        case FN_TYPE_POST_WRITE:
            s->noack.counters = ProtocolcountData;
            break;
    }
}


////////////////////////////////////////////////////////////
// allows read of parameter descritpions and variable length
// accepts (unsigned char first, unsigned char count) in read message!
// data returned
typedef struct tag_descriptions {
    unsigned char first;
    unsigned char count_read;
    char descriptions[251];
} DESCRIPTIONS;
DESCRIPTIONS paramstat_descriptions;
typedef struct tag_description {
    unsigned char len;    // overall length of THIS
    unsigned char code;   // code from params
    unsigned char var_len;// length of variable referenced
    unsigned char var_type;// UI_NONE or UI_SHORT for the moment
    char description[249];// nul term description
    // could be expanded here, as len at the top.
    // but 'description' above will be shorter than 249, so strucutre variabls can;t be used?
} DESCRIPTION;

void fn_paramstat_descriptions ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ){
    switch (fn_type) {
        case FN_TYPE_PRE_READ: {
            // content[0] is the first entry to read.
            // content[1] is max count of entries to read
            // we prepare a buffer here, an MODIFY the paramstat length to tell it how much to send! (evil!?).
            int first = 0;
            int count = 100;
            if (len > 0) {
                first = content[0];
            }
            if (len > 1) {
                count = content[1];
            }

            if (first >= (sizeof(params)/sizeof(params[0]))) {
                params[0]->len = 0;
                return;
            }
            if (first + count >= (sizeof(params)/sizeof(params[0]))){
                count = (sizeof(params)/sizeof(params[0])) - first;
            }

            // now loop over requested entries until no more will fit in buffer,
            // informing on start and count done.
            int actual_count = 0;
            int len_out = 0;
            char *p = paramstat_descriptions.descriptions;
            for (int i = first; i < first+count; i++){
                if(params[i] != NULL) {
                    int desc_len = 0;
                    if (params[i]->description) {
                        desc_len = strlen(params[i]->description);
                    }
                    DESCRIPTION *d = (DESCRIPTION *)p;
                    if (len_out+sizeof(*d)-sizeof(d->description)+desc_len+1 > sizeof(paramstat_descriptions.descriptions)){
                        break;
                    }
                    d->len = sizeof(*d)-sizeof(d->description)+desc_len+1;
                    d->code = params[i]->code;
                    d->var_len = params[i]->len;
                    d->var_type = params[i]->ui_type;
                    if (desc_len) {
                        strcpy(d->description, params[i]->description);
                    } else {
                        d->description[0] = 0;
                    }
                    p += d->len;
                    len_out = p - paramstat_descriptions.descriptions;
                    actual_count++;
                }
            }
            paramstat_descriptions.first = first;
            paramstat_descriptions.count_read = actual_count;
            param->len = sizeof(DESCRIPTIONS) - sizeof(paramstat_descriptions.descriptions) + len_out;
            break;
        }

        case FN_TYPE_POST_READ:
            param->len = 0; // reset to zero
            break;
    }
}



////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

// NOTE: Don't start uistr with 'a'
PARAMSTAT *params[256];


PARAMSTAT initialparams[] = {
    // Protocol Relevant Parameters
    { 0xFF, "descriptions",            NULL,  UI_NONE,  &paramstat_descriptions, 0,                         PARAM_R,  fn_paramstat_descriptions },
    { 0x00, "version",                 NULL,  UI_LONG,  &version,           sizeof(int),                    PARAM_R,  NULL },
    { 0x22, "subscribe data",          NULL,  UI_NONE,  &contentbuf,        sizeof(PROTOCOL_SUBSCRIBEDATA), PARAM_RW, fn_SubscribeData },
    { 0x23, "protocol stats ack+noack",NULL,  UI_NONE,  &ProtocolcountData, sizeof(PROTOCOLCOUNT),          PARAM_RW, fn_ProtocolcountDataSum },
    { 0x24, "protocol stats ack",      NULL,  UI_NONE,  &ProtocolcountData, sizeof(PROTOCOLCOUNT),          PARAM_RW, fn_ProtocolcountDataAck },
    { 0x25, "protocol stats noack",    NULL,  UI_NONE,  &ProtocolcountData, sizeof(PROTOCOLCOUNT),          PARAM_RW, fn_ProtocolcountDataNoack },

    // Sensor (Hoverboard mode)
    { 0x01, "sensor data",             NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_SENSOR_FRAME),          PARAM_R,  NULL },

    // Control and Measurements
    { 0x02, "hall data",               NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_HALL_DATA_STRUCT),      PARAM_R,  NULL },
    { 0x03, "speed control mm/s",      NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_SPEED_DATA),            PARAM_RW, fn_preWriteClear },
    { 0x04, "hall position mm",        NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_POSN),                  PARAM_RW, fn_preWriteClear },
    { 0x05, "position control increment mm",NULL,UI_NONE,&contentbuf,sizeof(PROTOCOL_POSN_INCR),             PARAM_RW, fn_preWriteClear },
    { 0x06, "position control mm",     NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_POSN_DATA),             PARAM_RW, fn_preWriteClear },
    { 0x07, "hall position steps",     NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_POSN),                  PARAM_RW, fn_preWriteClear },
    { 0x08, "electrical measurements", NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_ELECTRICAL_PARAMS),     PARAM_R,  NULL },
    { 0x09, "enable motors",           NULL,  UI_CHAR,  &contentbuf, sizeof(uint8_t),                        PARAM_RW, fn_preWriteClear },
    { 0x0A, "disable poweroff timer",  NULL,  UI_CHAR,  &contentbuf, sizeof(uint8_t),                        PARAM_RW, fn_preWriteClear },
    { 0x0B, "enable console logs",     NULL,  UI_CHAR,  &contentbuf, sizeof(uint8_t ),                       PARAM_RW, fn_preWriteClear },
    { 0x0C, "read/clear xyt position", NULL,  UI_3LONG, &contentbuf, sizeof(PROTOCOL_INTEGER_XYT_POSN),      PARAM_RW, fn_preWriteClear },
    { 0x0D, "PWM control",             NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_PWM_DATA),              PARAM_RW, fn_preWriteClear },
    { 0x0E, "simpler PWM",             NULL,  UI_2LONG, &contentbuf, sizeof( ((PROTOCOL_PWM_DATA *)0)->pwm), PARAM_RW, fn_preWriteClear },
    { 0x21, "buzzer",                  NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_BUZZER_DATA),           PARAM_RW, fn_preWriteClear },

    // Flash Storage
    { 0x80, "flash magic",             "m",   UI_SHORT, &contentbuf, sizeof(short),                 PARAM_RW, fn_preWriteClear },  // write this with CURRENT_MAGIC to commit to flash

    { 0x81, "posn kp x 100",           "pkp", UI_SHORT, &contentbuf, sizeof(short),                 PARAM_RW, fn_preWriteClear },
    { 0x82, "posn ki x 100",           "pki", UI_SHORT, &contentbuf, sizeof(short),                 PARAM_RW, fn_preWriteClear }, // pid params for Position
    { 0x83, "posn kd x 100",           "pkd", UI_SHORT, &contentbuf, sizeof(short),                 PARAM_RW, fn_preWriteClear },
    { 0x84, "posn pwm lim",            "pl",  UI_SHORT, &contentbuf, sizeof(short),                 PARAM_RW, fn_preWriteClear }, // e.g. 200

    { 0x85, "speed kp x 100",          "skp", UI_SHORT, &contentbuf, sizeof(short),                 PARAM_RW, fn_preWriteClear },
    { 0x86, "speed ki x 100",          "ski", UI_SHORT, &contentbuf, sizeof(short),                 PARAM_RW, fn_preWriteClear }, // pid params for Speed
    { 0x87, "speed kd x 100",          "skd", UI_SHORT, &contentbuf, sizeof(short),                 PARAM_RW, fn_preWriteClear },
    { 0x88, "speed pwm incr lim",      "sl",  UI_SHORT, &contentbuf, sizeof(short),                 PARAM_RW, fn_preWriteClear }, // e.g. 20
    { 0x89, "max current limit x 100", "cl",  UI_SHORT, &contentbuf, sizeof(short),                 PARAM_RW, fn_preWriteClear }, // by default 1500 (=15 amps), limited by DC_CUR_LIMIT
    { 0xA0, "hoverboard enable",       "he",  UI_SHORT, &contentbuf, sizeof(short),                 PARAM_RW, fn_preWriteClear } // e.g. 20
};


/////////////////////////////////////////////
// Set entry in params
int setParam(PARAMSTAT *param) {

    if(param == NULL) return 1;   // Failure, got NULL pointer

    // Check if len can actually be received
    if( param->len > sizeof( ((PROTOCOL_BYTES_WRITEVALS *)0)->content ) ) {
        return 1;                 // Too long, Failure
    }

    if( param->code < (sizeof(params)/sizeof(params[0])) ) {
        params[param->code] = param;
        return 0; // Successfully assigned
    }

    return 1; // Failure, index too big.
}

int setParams( PARAMSTAT params[], int len) {
    int error = 0;
    for (int i = 0; i < len; i++) {
        error += setParam(&params[i]);
    }
    return error;
}

/////////////////////////////////////////////
// Change variable at runtime
int setParamVariable(unsigned char code, char ui_type, void *ptr, int len, char rw) {

    // Check if len can actually be received
    if( len > sizeof( ((PROTOCOL_BYTES_WRITEVALS *)0)->content ) ) {
        return 1;                           // Too long, Failure
    }

    if( code < (sizeof(params)/sizeof(params[0])) ) {
        if(params[code] != NULL) {
            params[code]->ui_type = ui_type;
            params[code]->ptr = ptr;
            params[code]->len = len;
            params[code]->rw = rw;
            return 0;                       // Success
        }
    }
    return 1;                               // Not found, Failure
}

/////////////////////////////////////////////
// Register new function handler at runtime
int setParamHandler(unsigned char code, PARAMSTAT_FN callback) {

    if( code < (sizeof(params)/sizeof(params[0])) ) {
        if(params[code] == NULL) return 1;
        params[code]->fn = callback;
        return 0; // Successfully assigned
    }

    return 1; // Failure, index too big.
}

/////////////////////////////////////////////
// get param function handler
PARAMSTAT_FN getParamHandler(unsigned char code) {

    if( code < (sizeof(params)/sizeof(params[0])) ) {
        if(params[code] != NULL) {
            return params[code]->fn;
        }
    }

    return NULL;
}

/////////////////////////////////////////////
// initialize protocol
// called from main.c
int nosend( unsigned char *data, int len ){ return 0; };
int protocol_init(PROTOCOL_STAT *s) {
    memset(s, 0, sizeof(*s));
    s->timeout1 = 500;
    s->timeout2 = 100;
    s->allow_ascii = 1;
    s->send_serial_data = nosend;
    s->send_serial_data_wait = nosend;

    int error = 0;
    if (!initialised_functions) {
        error += setParams(initialparams, sizeof(initialparams)/sizeof(initialparams[0]));
        initialised_functions = 1;
        // yes, may be called multiple times, but checks internally.
        ascii_init();
    }

    return error;
}

/////////////////////////////////////////////
// a complete machineprotocol message has been
// received without error
void protocol_process_message(PROTOCOL_STAT *s, PROTOCOL_MSG2 *msg) {
    PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;

    switch (writevals->cmd){
        case PROTOCOL_CMD_READVAL: {
            if( writevals->code < (sizeof(params)/sizeof(params[0])) ) {
                if(params[writevals->code] != NULL) {
                    if (params[writevals->code]->fn) params[writevals->code]->fn( s, params[writevals->code], FN_TYPE_PRE_READ, writevals->content, msg->len-2 ); // NOTE: re-uses the msg object (part of stats)
                    unsigned char *src = params[writevals->code]->ptr;
                    for (int j = 0; j < params[writevals->code]->len; j++){
                        writevals->content[j] = *(src++);
                    }
                    msg->len = 1+1+params[writevals->code]->len;  // command + code + data len only
                    writevals->cmd = PROTOCOL_CMD_READVALRESPONSE; // mark as response
                    // send back with 'read' command plus data like write.
                    protocol_post(s, msg);
                    if (params[writevals->code]->fn) params[writevals->code]->fn( s, params[writevals->code], FN_TYPE_POST_READ, writevals->content, msg->len-2 );
                    break;
                }
            }
            // parameter code not found
            msg->len = 1+1; // cmd + code only
            writevals->cmd = PROTOCOL_CMD_READVALRESPONSE; // mark as response
            // send back with 'read' command plus data like write.
            protocol_post(s, msg);
            break;
        }

        case PROTOCOL_CMD_READVALRESPONSE: {
            if( writevals->code < (sizeof(params)/sizeof(params[0])) ) {
                if(params[writevals->code] != NULL) {
                    if (params[writevals->code]->fn) params[writevals->code]->fn( s, params[writevals->code], FN_TYPE_PRE_READRESPONSE, writevals->content, msg->len-2 );

                    unsigned char *dest = params[writevals->code]->ptr;
                    // ONLY copy what we have, else we're stuffing random data in.
                    // e.g. is setting posn, structure is 8 x 4 bytes,
                    // but we often only want to set the first 8
                    for (int j = 0; ((j < params[writevals->code]->len) && (j < (msg->len-2))); j++){
                        *(dest++) = writevals->content[j];
                    }
                    if (params[writevals->code]->fn) params[writevals->code]->fn( s, params[writevals->code], FN_TYPE_POST_READRESPONSE, writevals->content,  msg->len-2 );
                    break;
                }
            }
            // parameter code not found
            if(msg->SOM == PROTOCOL_SOM_ACK) {
                s->ack.counters.unplausibleresponse++;
            } else {
                s->noack.counters.unplausibleresponse++;
            }
            break;
        }

        case PROTOCOL_CMD_WRITEVALRESPONSE:{
            if( writevals->code < (sizeof(params)/sizeof(params[0])) ) {
                if(params[writevals->code] != NULL) {
                    break;
                }
            }
            // parameter code not found
            if(msg->SOM == PROTOCOL_SOM_ACK) {
                s->ack.counters.unplausibleresponse++;
            } else {
                s->noack.counters.unplausibleresponse++;
            }
            break;
        }

        case PROTOCOL_CMD_WRITEVAL:{
            if( writevals->code < (sizeof(params)/sizeof(params[0])) ) {
                if(params[writevals->code] != NULL) {
                    if (params[writevals->code]->fn) params[writevals->code]->fn( s, params[writevals->code], FN_TYPE_PRE_WRITE, writevals->content, msg->len-2 );
                    // NOTE: re-uses the msg object (part of stats)
                    unsigned char *dest = params[writevals->code]->ptr;

                    // ONLY copy what we have, else we're stuffing random data in.
                    // e.g. is setting posn, structure is 8 x 4 bytes,
                    // but we often only want to set the first 8
                    for (int j = 0; ((j < params[writevals->code]->len) && (j < (msg->len-2))); j++){
                        *(dest++) = writevals->content[j];
                    }
                    msg->len = 1+1+1; // cmd+code+'1' only
                    writevals->cmd = PROTOCOL_CMD_WRITEVALRESPONSE; // mark as response
                    writevals->content[0] = 1; // say we wrote it
                    // send back with 'write' command with no data.
                    protocol_post(s, msg);
                    if (params[writevals->code]->fn) params[writevals->code]->fn( s, params[writevals->code], FN_TYPE_POST_WRITE, writevals->content, msg->len-2 );
                    break;
                }
            }
            // parameter code not found
            msg->len = 1+1+1; // cmd +code +'0' only
            writevals->cmd = PROTOCOL_CMD_WRITEVALRESPONSE; // mark as response
            writevals->content[0] = 0; // say we did not write it
            // send back with 'write' command plus data like write.
            protocol_post(s, msg);
            break;
        }

        case PROTOCOL_CMD_REBOOT:
            //protocol_send_ack(); // we no longer ack from here
            protocol_Delay(500);
            protocol_SystemReset();
            break;

        case PROTOCOL_CMD_TEST:
            // just send it back!
            writevals->cmd = PROTOCOL_CMD_TESTRESPONSE;
            // note: original 'bytes' sent back, so leave len as is
            protocol_post(s, msg);
            // post second immediately to test buffering
            // protocol_post(s, msg);
            break;

        case PROTOCOL_CMD_UNKNOWN:
            // Do nothing, otherwise endless loop is entered.
            if(msg->SOM == PROTOCOL_SOM_ACK) {
                s->ack.counters.unknowncommands++;
            } else {
                s->noack.counters.unknowncommands++;
            }
            break;

        default:
            if(msg->SOM == PROTOCOL_SOM_ACK) {
                s->ack.counters.unknowncommands++;
            } else {
                s->noack.counters.unknowncommands++;
            }            writevals->cmd = PROTOCOL_CMD_UNKNOWN;
            msg->len = 1;
            protocol_post(s, msg);
        break;
    }
}
