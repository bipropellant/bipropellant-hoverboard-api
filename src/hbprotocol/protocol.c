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


//////////////////////////////////////////////
// variables you want to read/write here. Only common used ones, specific ones below.

// Default temporary storage for received values
unsigned char contentbuf[sizeof( ((PROTOCOL_MSG3full *)0)->content )];

void protocol_process_ReadValue(PROTOCOL_STAT *s, PROTOCOL_MSG3full *msg) {
    if(msg) {
        unsigned char *src = s->params[msg->code]->ptr;
        for (int j = 0; j < s->params[msg->code]->len; j++){
            msg->content[j] = *(src++);
        }
    }
}

void protocol_process_ReadAndSendValue(PROTOCOL_STAT *s, PROTOCOL_MSG3full *msg) {
    if(msg) {
        protocol_process_ReadValue(s, msg);

        PROTOCOL_MSG3full newMsg;
        memcpy(&newMsg, msg, sizeof(PROTOCOL_MSG3full));

        newMsg.lenPayload = s->params[msg->code]->len;  // command + code + data len only
        newMsg.cmd = PROTOCOL_CMD_READVALRESPONSE; // mark as response
        // send back with 'read' command plus data like write.
        protocol_post(s, &newMsg);
    }
}

void protocol_process_WriteValue(PROTOCOL_STAT *s, PROTOCOL_MSG3full *msg) {
    if(msg) {
        unsigned char *dest = s->params[msg->code]->ptr;
        // ONLY copy what we have, else we're stuffing random data in.
        // e.g. is setting posn, structure is 8 x 4 bytes,
        // but we often only want to set the first 8
        for (int j = 0; ((j < s->params[msg->code]->len) && (j < (msg->lenPayload))); j++){
            *(dest++) = msg->content[j];
        }
    }
}

void protocol_process_cmdWritevalAndRespond(PROTOCOL_STAT *s, PROTOCOL_MSG3full *msg) {
    if(msg) {

        protocol_process_WriteValue(s, msg);

        PROTOCOL_MSG3full newMsg;
        memcpy(&newMsg, msg, sizeof(PROTOCOL_MSG3full));

        newMsg.lenPayload = 1; // 1 only
        newMsg.cmd = PROTOCOL_CMD_WRITEVALRESPONSE; // mark as response
        newMsg.content[0] = 1; // say we wrote it
        // send back with 'write' command with no data.
        protocol_post(s, &newMsg);
    }
}


void fn_defaultProcessing ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            protocol_process_ReadAndSendValue(s, msg);
            break;
        case PROTOCOL_CMD_SILENTREAD:
            protocol_process_ReadValue(s, msg);
            break;
        case PROTOCOL_CMD_READVALRESPONSE:
            protocol_process_WriteValue(s, msg);
            break;
        case PROTOCOL_CMD_WRITEVALRESPONSE:
            // Should never get here..
            break;
        case PROTOCOL_CMD_WRITEVAL:
            protocol_process_cmdWritevalAndRespond(s, msg);
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
// Default function, wipes receive memory before writing (and readresponse is just a differenct type of writing)


void fn_defaultProcessingPreWriteClear ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
            // ensure clear in case of short write
            memset(param->ptr, 0, param->len);
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
// Default function, wipes receive memory before writing (and readresponse is just a differenct type of writing)


void fn_defaultProcessingReadOnly ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
            fn_defaultProcessing(s, param, cmd, msg);
            break;

        case PROTOCOL_CMD_READVALRESPONSE:
        case PROTOCOL_CMD_WRITEVALRESPONSE:
        case PROTOCOL_CMD_WRITEVAL:
            // Do nothing
            break;
    }

}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0xFE version

static uint32_t version = 3;

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x27 Ping
// Sends back the received Message as a readresponse. (Sender can send a timestamp to measure latency)

void fn_ping ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {

    switch (cmd) {

        case PROTOCOL_CMD_WRITEVAL:
        {
            if(msg) {
                msg->cmd = PROTOCOL_CMD_READVALRESPONSE;
                protocol_post(s, msg);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x22 SubscribeData

void fn_SubscribeData ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {

    fn_defaultProcessingPreWriteClear(s, param, cmd, msg); // Wipes memory before write (and readresponse is just a differenct type of writing)

    switch (cmd) {

        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
        {
            if(msg) {

                // Check if length of received data is plausible.
                if(msg->lenPayload != sizeof(PROTOCOL_SUBSCRIBEDATA)) {
                    break;
                }

                int subscriptions_len = sizeof(s->subscriptions)/sizeof(s->subscriptions[0]);
                int index = 0;

                // Check if subscription already exists for this code
                for (index = 0; index < subscriptions_len; index++) {
                    if(s->subscriptions[index].code == ((PROTOCOL_SUBSCRIBEDATA*) msg->content)->code) {
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
                if(index < subscriptions_len && ((PROTOCOL_SUBSCRIBEDATA*) msg->content)->period >= 10) {
                    s->subscriptions[index] = *((PROTOCOL_SUBSCRIBEDATA*) msg->content);
                    //char tmp[100];
                    //sprintf(tmp, "subscription added at %d for 0x%x, period %d, count %d, som %d\n", index, ((SUBSCRIBEDATA*) (param->ptr))->code, ((SUBSCRIBEDATA*) (param->ptr))->period, ((SUBSCRIBEDATA*) (param->ptr))->count, ((SUBSCRIBEDATA*) (param->ptr))->som);
                    //consoleLog(tmp);
                } else {
                    // TODO. Inform sender??
                    // consoleLog("no subscriptions available\n");
                }
            }
            break;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x22 and 0x23 ProtocolcountData

PROTOCOLCOUNT ProtocolcountData =  { .rx = 0 };

void fn_ProtocolcountDataSum ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {

    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
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
    }

    fn_defaultProcessingPreWriteClear(s, param, cmd, msg); // Wipes memory before write (and readresponse is just a differenct type of writing)

    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
            s->ack.counters = ProtocolcountData;
            s->noack.counters = ProtocolcountData;
            break;
    }
}

void fn_ProtocolcountDataAck ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {

    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
            ProtocolcountData = s->ack.counters;
            break;
    }

    fn_defaultProcessingPreWriteClear(s, param, cmd, msg); // Wipes memory before write (and readresponse is just a differenct type of writing)

    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
            s->ack.counters = ProtocolcountData;
            break;
    }

}

void fn_ProtocolcountDataNoack ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {

    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
            ProtocolcountData = s->noack.counters;
            break;
    }

    fn_defaultProcessingPreWriteClear(s, param, cmd, msg); // Wipes memory before write (and readresponse is just a differenct type of writing)

    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
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


void fn_paramstat_descriptions ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
        {
            if(msg) {
                // content[0] is the first entry to read.
                // content[1] is max count of entries to read
                // we prepare a buffer here, an MODIFY the paramstat length to tell it how much to send! (evil!?).
                int first = 0;
                int count = 100;
                if (msg->lenPayload > 0) {
                    first = msg->content[0];
                }
                if (msg->lenPayload > 1) {
                    count = msg->content[1];
                }

                if (first >= (sizeof(s->params)/sizeof(s->params[0]))) {
                    s->params[0]->len = 0;
                    return;
                }
                if (first + count >= (sizeof(s->params)/sizeof(s->params[0]))){
                    count = (sizeof(s->params)/sizeof(s->params[0])) - first;
                }

                // now loop over requested entries until no more will fit in buffer,
                // informing on start and count done.
                int actual_count = 0;
                int len_out = 0;
                char *p = paramstat_descriptions.descriptions;
                for (int i = first; i < first+count; i++){
                    if(s->params[i] != NULL) {
                        int desc_len = 0;
                        if (s->params[i]->description) {
                            desc_len = strlen(s->params[i]->description);
                        }
                        DESCRIPTION *d = (DESCRIPTION *)p;
                        if (len_out+sizeof(*d)-sizeof(d->description)+desc_len+1 > sizeof(paramstat_descriptions.descriptions)){
                            break;
                        }
                        d->len = sizeof(*d)-sizeof(d->description)+desc_len+1;
                        d->code = s->params[i]->code;
                        d->var_len = s->params[i]->len;
                        d->var_type = s->params[i]->ui_type;
                        if (desc_len) {
                            strcpy(d->description, s->params[i]->description);
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
            }
            break;
        }
    }

    fn_defaultProcessingReadOnly(s, param, cmd, msg);

    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
            if(msg) {
            param->len = 0; // reset to zero
            }
            break;
    }
}


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

// NOTE: Don't start uistr with 'a'

const static PARAMSTAT initialparams[] = {
    // 0x00 is reserved, can not be used.

    // Protocol Relevant Parameters
    { 0xFF, "descriptions",            NULL,  UI_NONE,  &paramstat_descriptions, 0,                         fn_paramstat_descriptions },
    { 0xFE, "version",                 NULL,  UI_LONG,  &version,           sizeof(uint32_t),               fn_defaultProcessingReadOnly },
    { 0x22, "subscribe data",          NULL,  UI_NONE,  &contentbuf,        sizeof(PROTOCOL_SUBSCRIBEDATA), fn_SubscribeData },
    { 0x23, "protocol stats ack+noack",NULL,  UI_NONE,  &ProtocolcountData, sizeof(PROTOCOLCOUNT),          fn_ProtocolcountDataSum },
    { 0x24, "protocol stats ack",      NULL,  UI_NONE,  &ProtocolcountData, sizeof(PROTOCOLCOUNT),          fn_ProtocolcountDataAck },
    { 0x25, "protocol stats noack",    NULL,  UI_NONE,  &ProtocolcountData, sizeof(PROTOCOLCOUNT),          fn_ProtocolcountDataNoack },
    { 0x26, "text",                    NULL,  UI_NONE,  &contentbuf,        sizeof(contentbuf),             fn_defaultProcessing },
    { 0x27, "ping",                    NULL,  UI_NONE,  &contentbuf,        sizeof(contentbuf),             fn_ping },

    // Sensor (Hoverboard mode)
    { 0x01, "sensor data",             NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_SENSOR_FRAME),          fn_defaultProcessingReadOnly },

    // Control and Measurements
    { 0x02, "hall data",               NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_HALL_DATA_STRUCT),      fn_defaultProcessingReadOnly },
    { 0x03, "speed control mm/s",      NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_SPEED_DATA),            fn_defaultProcessingPreWriteClear },
    { 0x04, "hall position mm",        NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_POSN),                  fn_defaultProcessingPreWriteClear },
    { 0x05, "position control increment mm",NULL,UI_NONE,&contentbuf,sizeof(PROTOCOL_POSN_INCR),             fn_defaultProcessingPreWriteClear },
    { 0x06, "position control mm",     NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_POSN_DATA),             fn_defaultProcessingPreWriteClear },
    { 0x07, "hall position steps",     NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_POSN),                  fn_defaultProcessingPreWriteClear },
    { 0x08, "electrical measurements", NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_ELECTRICAL_PARAMS),     fn_defaultProcessingReadOnly },
    { 0x09, "enable motors",           NULL,  UI_CHAR,  &contentbuf, sizeof(uint8_t),                        fn_defaultProcessingPreWriteClear },
    { 0x0A, "disable poweroff timer",  NULL,  UI_CHAR,  &contentbuf, sizeof(uint8_t),                        fn_defaultProcessingPreWriteClear },
    { 0x0B, "enable console logs",     NULL,  UI_CHAR,  &contentbuf, sizeof(uint8_t ),                       fn_defaultProcessingPreWriteClear },
    { 0x0C, "read/clear xyt position", NULL,  UI_3LONG, &contentbuf, sizeof(PROTOCOL_INTEGER_XYT_POSN),      fn_defaultProcessingPreWriteClear },
    { 0x0D, "PWM control",             NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_PWM_DATA),              fn_defaultProcessingPreWriteClear },
    { 0x0E, "simpler PWM",             NULL,  UI_2LONG, &contentbuf, sizeof( ((PROTOCOL_PWM_DATA *)0)->pwm), fn_defaultProcessingPreWriteClear },
    { 0x21, "buzzer",                  NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_BUZZER_DATA),           fn_defaultProcessingPreWriteClear },

    // Flash Storage
    { 0x80, "flash magic",             "m",   UI_SHORT, &contentbuf, sizeof(short),                 fn_defaultProcessingPreWriteClear },  // write this with CURRENT_MAGIC to commit to flash

    { 0x81, "posn kp x 100",           "pkp", UI_SHORT, &contentbuf, sizeof(short),                 fn_defaultProcessingPreWriteClear },
    { 0x82, "posn ki x 100",           "pki", UI_SHORT, &contentbuf, sizeof(short),                 fn_defaultProcessingPreWriteClear }, // pid params for Position
    { 0x83, "posn kd x 100",           "pkd", UI_SHORT, &contentbuf, sizeof(short),                 fn_defaultProcessingPreWriteClear },
    { 0x84, "posn pwm lim",            "pl",  UI_SHORT, &contentbuf, sizeof(short),                 fn_defaultProcessingPreWriteClear }, // e.g. 200

    { 0x85, "speed kp x 100",          "skp", UI_SHORT, &contentbuf, sizeof(short),                 fn_defaultProcessingPreWriteClear },
    { 0x86, "speed ki x 100",          "ski", UI_SHORT, &contentbuf, sizeof(short),                 fn_defaultProcessingPreWriteClear }, // pid params for Speed
    { 0x87, "speed kd x 100",          "skd", UI_SHORT, &contentbuf, sizeof(short),                 fn_defaultProcessingPreWriteClear },
    { 0x88, "speed pwm incr lim",      "sl",  UI_SHORT, &contentbuf, sizeof(short),                 fn_defaultProcessingPreWriteClear }, // e.g. 20
    { 0x89, "max current limit x 100", "cl",  UI_SHORT, &contentbuf, sizeof(short),                 fn_defaultProcessingPreWriteClear }, // by default 1500 (=15 amps), limited by DC_CUR_LIMIT
    { 0x90, "adc settings",            NULL,  UI_NONE,  &contentbuf, sizeof(PROTOCOL_ADC_SETTINGS), fn_defaultProcessingPreWriteClear },
    { 0xA0, "hoverboard enable",       "he",  UI_SHORT, &contentbuf, sizeof(short),                 fn_defaultProcessingPreWriteClear } // e.g. 20
};


/////////////////////////////////////////////
// Set entry in params
int setParam(PROTOCOL_STAT *s, PARAMSTAT *param) {

    if(param == NULL) return 1;   // Failure, got NULL pointer

    // Check if len can actually be received
    if( param->len > sizeof( ((PROTOCOL_MSG3full *)0)->content ) ) {
        return 1;                 // Too long, Failure
    }

    if( param->code < (sizeof(s->params)/sizeof(s->params[0])) ) {
        s->params[param->code] = param;
        return 0; // Successfully assigned
    }

    return 1; // Failure, index too big.
}


int setParams(PROTOCOL_STAT *s, PARAMSTAT params[], int len) {
    int error = 0;
    for (int i = 0; i < len; i++) {
        error += setParam(s, &params[i]);
    }
    return error;
}


int setParamsCopy(PROTOCOL_STAT *s, const PARAMSTAT params[], int len) {
    int error = 0;
    for (int i = 0; i < len; i++) {
        PARAMSTAT *newParam;
        newParam = (PARAMSTAT *) malloc( sizeof(PARAMSTAT) );
        memcpy(newParam, &params[i], sizeof(PARAMSTAT));
        error += setParam(s, newParam);
    }
    return error;
}

/////////////////////////////////////////////
// Change variable at runtime
int setParamVariable(PROTOCOL_STAT *s, unsigned char code, char ui_type, void *ptr, int len) {

    // Check if len can actually be received
    if( len > sizeof( ((PROTOCOL_MSG3full *)0)->content ) ) {
        return 1;                           // Too long, Failure
    }

    if( code < (sizeof(s->params)/sizeof(s->params[0])) ) {
        if(s->params[code] != NULL) {
            s->params[code]->ui_type = ui_type;
            s->params[code]->ptr = ptr;
            s->params[code]->len = len;
            return 0;                       // Success
        }
    }
    return 1;                               // Not found, Failure
}

/////////////////////////////////////////////
// Register new function handler at runtime
int setParamHandler(PROTOCOL_STAT *s, unsigned char code, PARAMSTAT_FN callback) {

    if( code < (sizeof(s->params)/sizeof(s->params[0])) ) {
        if(s->params[code] == NULL) return 1;
        s->params[code]->fn = callback;
        return 0; // Successfully assigned
    }

    return 1; // Failure, index too big.
}

/////////////////////////////////////////////
// get param function handler
PARAMSTAT_FN getParamHandler(PROTOCOL_STAT *s, unsigned char code) {

    if( code < (sizeof(s->params)/sizeof(s->params[0])) ) {
        if(s->params[code] != NULL) {
            return s->params[code]->fn;
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
    s->ack.lastTXCI = 1;
    s->ack.lastRXCI = 1;
    s->noack.lastTXCI = 1;
    s->noack.lastRXCI = 1;

    // Initialize params array
    int error = 0;
    if (!s->initialised_functions) {
        error += setParamsCopy(s, initialparams, sizeof(initialparams)/sizeof(initialparams[0]));
        s->initialised_functions = 1;
        // yes, may be called multiple times, but checks internally.
        ascii_init(s);
    }


    // Send version as "welcome" message over protocol.
    // First try will fail, as send_serial_data is set to dummy function. But the next try should
    // succeed as it is expected to set send_serial_data to something useful before protocol_tick is called.
    PROTOCOL_MSG3full newMsg;
    memset((void*)&newMsg,0x00,sizeof(PROTOCOL_MSG3full));

    uint32_t *writeint = (uint32_t *) newMsg.content;

    newMsg.SOM = PROTOCOL_SOM_ACK;              // Retry to send when no ACK was received
    newMsg.lenPayload = sizeof(uint32_t);

    newMsg.cmd  = PROTOCOL_CMD_READVALRESPONSE; // Pretend someone requested this.
    newMsg.code = 0xFE;                         // 0xFE for version
    *writeint   = version;

    protocol_post(s, &newMsg);

    return error;
}

/////////////////////////////////////////////
// a complete machineprotocol message has been
// received without error
void protocol_process_message(PROTOCOL_STAT *s, PROTOCOL_MSG3full *msg) {

    switch (msg->cmd){
        case PROTOCOL_CMD_SILENTREAD: {
            if( msg->code < (sizeof(s->params)/sizeof(s->params[0])) ) {
                if(s->params[msg->code] != NULL) {
                    if (s->params[msg->code]->fn) s->params[msg->code]->fn( s, s->params[msg->code], msg->cmd, msg ); // NOTE: re-uses the msg object (part of stats)
                    break;
                }
            }
            break;
        }

        case PROTOCOL_CMD_READVAL: {
            if( msg->code < (sizeof(s->params)/sizeof(s->params[0])) ) {
                if(s->params[msg->code] != NULL) {
                    if (s->params[msg->code]->fn) s->params[msg->code]->fn( s, s->params[msg->code], msg->cmd, msg ); // NOTE: re-uses the msg object (part of stats)
                    break;
                }
            }
            // parameter code not found
            msg->lenPayload = 0;
            msg->cmd = PROTOCOL_CMD_READVALRESPONSE; // mark as response
            // send back with 'read' command plus data like write.
            protocol_post(s, msg);
            break;
        }

        case PROTOCOL_CMD_READVALRESPONSE: {
            if( msg->code < (sizeof(s->params)/sizeof(s->params[0])) ) {
                if(s->params[msg->code] != NULL) {
                    if (s->params[msg->code]->fn) s->params[msg->code]->fn( s, s->params[msg->code], msg->cmd, msg ); // NOTE: re-uses the msg object (part of stats)
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
            if( msg->code < (sizeof(s->params)/sizeof(s->params[0])) ) {
                if(s->params[msg->code] != NULL) {
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
            if( msg->code < (sizeof(s->params)/sizeof(s->params[0])) ) {
                if(s->params[msg->code] != NULL) {
                    if (s->params[msg->code]->fn) s->params[msg->code]->fn( s, s->params[msg->code], msg->cmd, msg ); // NOTE: re-uses the msg object (part of stats)
                    break;
                }
            }
            // parameter code not found
            msg->lenPayload = 1; // '0' only
            msg->cmd = PROTOCOL_CMD_WRITEVALRESPONSE; // mark as response
            msg->content[0] = 0; // say we did not write it
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
            msg->cmd = PROTOCOL_CMD_TESTRESPONSE;
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
            }            msg->cmd = PROTOCOL_CMD_UNKNOWN;
            msg->lenPayload = 0;
            protocol_post(s, msg);
        break;
    }
}
