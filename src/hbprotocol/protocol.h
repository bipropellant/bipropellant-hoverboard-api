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
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//// control structures used in firmware
#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_PROTOCOL_POSN_DATA {
    // these get set
    long wanted_posn_mm[2];

    // configurations/constants
    int posn_max_speed; // max speed in this mode
    int posn_min_speed; // minimum speed (to get wheels moving)

    // just so it can be read back
    long posn_diff_mm[2];
    long posn_speed_demand[2];
} PROTOCOL_POSN_DATA;
#pragma pack(pop)


#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_PROTOCOL_SPEED_DATA {
    // these get set
    long wanted_speed_mm_per_sec[2];

    // configurations/constants
    int speed_max_power; // max speed in this mode
    int speed_min_power; // minimum speed (to get wheels moving)
    int speed_minimum_speed; // below this, we don't ask it to do anything

    // just so it can be read back
    long speed_diff_mm_per_sec[2];
    long speed_power_demand[2];
} PROTOCOL_SPEED_DATA;
#pragma pack(pop)


#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_PROTOCOL_PWM_DATA {
    // these get set
    long pwm[2];

    // configurations/constants
    int speed_max_power; // max speed in this mode
    int speed_min_power; // minimum speed (to get wheels moving)
    int speed_minimum_pwm; // below this, we don't ask it to do anything
} PROTOCOL_PWM_DATA;
#pragma pack(pop)



#pragma pack(push, 4)  // long and float are 4 byte each
typedef struct tag_PROTOCOL_HALL_DATA_STRUCT{
    long HallPosn; // 90 per revolution
    long HallSpeed; // speed part calibrated to speed demand value

    float HallPosnMultiplier; // m per hall segment

    long HallPosn_lastread; // posn offset set via protocol in raw value
    long HallPosn_mm; // posn in mm
    long HallPosn_mm_lastread; // posn offset set via protocol in mm
    long HallSpeed_mm_per_s; // speed in m/s

    unsigned long HallTimeDiff;
    unsigned long HallSkipped;
} PROTOCOL_HALL_DATA_STRUCT;
#pragma pack(pop)

#pragma pack(push, 4) // all used types (float and int) are 4 bytes

typedef struct tag_PROTOCOL_MOTOR_ELECTRICAL{
        float dcAmps;
        float dcAmpsAvgAcc;
        float dcAmpsAvg;
        int r1;
        int r2;
        int q;

        int dcAmpsx100;

        int pwm_limiter;
        int pwm_requested;
        int pwm_actual;

        unsigned int limiter_count;
} PROTOCOL_MOTOR_ELECTRICAL;
#pragma pack(pop)

#pragma pack(push, 4) // all used types (float and int) are 4 bytes
typedef struct tag_PROTOCOL_ELECTRICAL_PARAMS{
    int bat_raw;
    float batteryVoltage;

    int board_temp_raw;
    float board_temp_filtered;
    float board_temp_deg_c;

    int charging;

    int dcCurLim; // amps*100
    int dc_adc_limit; // limit expressed in terms of ADC units.

    PROTOCOL_MOTOR_ELECTRICAL motors[2];

} PROTOCOL_ELECTRICAL_PARAMS;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct tag_PROTOCOL_sensor_frame{
  unsigned char header_00; // this byte gets 0x100 (9 bit serial)
  short Angle;
  short Angle_duplicate;
  unsigned char AA_55;
  unsigned char Accelleration;
  unsigned char Accelleration_duplicate;
  short Roll;
} PROTOCOL_SENSOR_FRAME;
#pragma pack(pop)

#pragma pack(push, 4)  // since on 'long' are used, alignment can be optimized for 4 bytes
typedef struct PROTOCOL_INTEGER_XYT_POSN_tag {
    long x;
    long y;
    long degrees;
} PROTOCOL_INTEGER_XYT_POSN;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint8_t buzzerFreq;
    uint8_t buzzerPattern;
    uint16_t buzzerLen;
} PROTOCOL_BUZZER_DATA;
#pragma pack(pop)


#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_PROTOCOL_POSN {
    long LeftAbsolute;
    long RightAbsolute;
    long LeftOffset;
    long RightOffset;
} PROTOCOL_POSN;
#pragma pack(pop)

#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_PROTOCOL_POSN_INCR {
    long Left;
    long Right;
} PROTOCOL_POSN_INCR;
#pragma pack(pop)


#define CONTROL_TYPE_NONE 0
#define CONTROL_TYPE_POSITION 1
#define CONTROL_TYPE_SPEED 2
#define CONTROL_TYPE_PWM 3
#define CONTROL_TYPE_MAX 4




#pragma pack(push, 1)
typedef struct tag_PROTOCOL_SUBSCRIBEDATA {
    char code;                       // code in protocol to refer to this
    unsigned int period;             // how often should the information be sent?
    int count;                       // how many messages shall be sent? -1 for infinity
    char som;                        // which SOM shall be used? with or without ACK
    unsigned long next_send_time;    // last time a message requiring an ACK was sent

} PROTOCOL_SUBSCRIBEDATA;
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct tag_PROTOCOL_MSG2 {
    unsigned char SOM; // 0x02
    unsigned char CI; // continuity counter
    unsigned char len; // len is len of bytes to follow, NOT including CS
    unsigned char bytes[255];  // variable number of data bytes, with a checksum on the end, cmd is first
    // checksum such that sum of bytes CI to CS is zero
} PROTOCOL_MSG2;
#pragma pack(pop)


//////////////////////////////////////////////////////////////////
// protocol_post uses this structure to store outgoing messages
// until they can be sent.
// messages are stored only as len|data
// SOM, CI, and CS are not included.
#define MACHINE_PROTOCOL_TX_BUFFER_SIZE 1024
typedef struct tag_MACHINE_PROTOCOL_TX_BUFFER {
    volatile unsigned char buff[MACHINE_PROTOCOL_TX_BUFFER_SIZE];
    volatile int head;
    volatile int tail;

    // count of buffer overflows
    volatile unsigned int overflow;

} MACHINE_PROTOCOL_TX_BUFFER;

//////////////////////////////////////////////////////////


#pragma pack(push, 4) // all used data types are 4 byte
typedef struct tag_PROTOCOLCOUNT {
    unsigned long rx;              // Count of received messages (valid CS)
    unsigned long rxMissing;       // If message IDs went missing..
    unsigned long tx;              // Count of sent messages (ACK, NACK and retries do not count)
    unsigned int  txRetries;       // how often were messages resend?
    unsigned int  txFailed;        // TX Messages which couldn't be deliveredr. No retries left.

    unsigned int  unwantedacks;              // count of unwated ACK messages
    unsigned int  unwantednacks;             // count of unwanted NACK messges
    unsigned int  unknowncommands;           // count of messages with unknown commands
    unsigned int  unplausibleresponse;       // count of unplausible replies
} PROTOCOLCOUNT;
#pragma pack(pop)


typedef struct tag_PROTOCOLSTATE {
    PROTOCOL_MSG2 curr_send_msg;             // transmit message storage
    char retries;                            // number of retries left to send message
    int lastTXCI;                            // CI of last sent message
    int lastRXCI;                            // CI of last received message in ACKed stream
    unsigned long last_send_time;            // last time a message requiring an ACK was sent

    PROTOCOLCOUNT counters;                  // Statistical data of the protocol performance
    MACHINE_PROTOCOL_TX_BUFFER TxBuffer;     // Buffer for Messages to be sent
} PROTOCOLSTATE;



typedef struct tag_PROTOCOL_STAT {
    char allow_ascii;                     // If set to 0, ascii protocol is not used
    unsigned long last_tick_time;         // last time the tick function was called
    char state;                           // state used in protocol_byte to receive messages
    unsigned long last_char_time;         // last time a character was received
    unsigned char CS;                     // temporary storage to calculate Checksum
    unsigned char count;                  // index pointing to last received byte
    PROTOCOL_MSG2 curr_msg;               // received message storage

    char send_state;                      // message transmission state (ACK_TX_WAITING or IDLE)

    int timeout1;                         // ACK has to be received in this time
    int timeout2;                         // when receiving a packet, longest time between characters

    int (*send_serial_data)( unsigned char *data, int len );       // Function Pointer to sending function
    int (*send_serial_data_wait)( unsigned char *data, int len );

    PROTOCOL_SUBSCRIBEDATA subscriptions[10];      // Subscriptions to periodic messages

    PROTOCOLSTATE ack;
    PROTOCOLSTATE noack;
} PROTOCOL_STAT;

struct tag_PARAMSTAT;
typedef struct tag_PARAMSTAT PARAMSTAT;

// NOTE: content can be NULL if len == 0
typedef void (*PARAMSTAT_FN)( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len );

struct tag_PARAMSTAT {
    unsigned char code;     // code in protocol to refer to this
    char *description;      // if non-null, description
    char *uistr;            // if non-null, used in ascii protocol to adjust with f<str>num<cr>
    char ui_type;           // only UI_NONE or UI_SHORT
    void *ptr;              // pointer to value
    int len;                // length of value
    char rw;                // PARAM_R or PARAM_RW

    PARAMSTAT_FN fn;        // function to handle events
};



/////////////////////////////////////////////////////////
// command definitions
// ack - no payload
#define PROTOCOL_CMD_ACK 'A'
// nack - no payload
#define PROTOCOL_CMD_NACK 'N'

// a test command - normal payload - 'Test'
#define PROTOCOL_CMD_TEST 'T'
#define PROTOCOL_CMD_TESTRESPONSE 't'

// cause unit to restart - no payload
#define PROTOCOL_CMD_REBOOT 'B'

// response to an unkonwn command - maybe payload
#define PROTOCOL_CMD_UNKNOWN '?'

#define PROTOCOL_SOM_ACK 2
#define PROTOCOL_SOM_NOACK 4
//
/////////////////////////////////////////////////////////////////

#define PROTOCOL_CMD_READVAL 'R'
#define PROTOCOL_CMD_READVALRESPONSE 'r'
#define PROTOCOL_CMD_WRITEVAL 'W'
#define PROTOCOL_CMD_WRITEVALRESPONSE 'w'


///////////////////////////////////////////////////
// structure used to gather variables we want to read/write.
#define PARAM_R     1
#define PARAM_RW    3
///////////////////////////////////////////////////
// defines for simple variable types.
// generally:
// if first nibble is 1, second nibble is bytecount for single variable.
// if second nibble is 2, second nibble is bytecount for each of two variables.
// etc.
// if 2nd nibble is NOT a power of 2, all bets off -> custom type (note 2^0 = 1)
// these are as yet not implemented....
#define UI_NONE 0
#define UI_CHAR 0x11
#define UI_SHORT 0x12
#define UI_LONG 0x14
#define UI_LONGLONG 0x18
#define UI_2CHAR 0x21
#define UI_2SHORT 0x22
#define UI_2LONG 0x24
#define UI_2LONGLONG 0x28
#define UI_3LONG 0x34
// e.g.
// #define UI_8CHAR 0x81
// #define UI_POSN 0x03 - custom structure type.

#define FN_TYPE_PRE_READ          1
#define FN_TYPE_POST_READ         2
#define FN_TYPE_PRE_WRITE         3
#define FN_TYPE_POST_WRITE        4
#define FN_TYPE_PRE_READRESPONSE  5
#define FN_TYPE_POST_READRESPONSE 6




//////////////////////////////////////////////////////
// PUBLIC functions
/////////////////////////////////////////////////////////
extern void ascii_add_immediate( unsigned char letter, int (*fn)(PROTOCOL_STAT *s, char byte,  char *ascii_out), char* description );
extern void ascii_add_line_fn( unsigned char letter, int (*fn)(PROTOCOL_STAT *s, char *line, char *ascii_out), char *description );
extern int ascii_init();
// Set entry in params
extern int setParam(PARAMSTAT *param);
/////////////////////////////////////////////////////////////////
// Change variable at runtime
extern int setParamVariable(unsigned char code, char ui_type, void *ptr, int len, char rw);
/////////////////////////////////////////////////////////////////
// Register new function handler at runtime
extern int setParamHandler(unsigned char code, PARAMSTAT_FN callback);
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
// call this with received bytes; normally from main loop
void protocol_byte( PROTOCOL_STAT *s, unsigned char byte );
/////////////////////////////////////////////////////////////////
// call this schedule a message. CI and Checksum are added
int protocol_post(PROTOCOL_STAT *s, PROTOCOL_MSG2 *msg);
/////////////////////////////////////////////////////////////////
// call this regularly from main.c
void protocol_tick(PROTOCOL_STAT *s);
/////////////////////////////////////////////////////////////////
// initialize protocol
int protocol_init(PROTOCOL_STAT *s);
/////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////
// PUBLIC variables
//////////////////////////////////////////////////////////

// used to enable immediate mode (action on keypress)
extern int enable_immediate;
// used to display help
extern PARAMSTAT *params[256];



///////////////////////////////////////////////////////
// Function Pointers to system functions
//////////////////////////////////////////////////////////

// Need to be assigned to functions "real" system fucntions
extern void (*protocol_Delay)(uint32_t Delay);
extern void (*protocol_SystemReset)(void);
extern uint32_t (*protocol_GetTick)(void);


#ifdef __cplusplus
}
#endif
