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
    int32_t wanted_posn_mm[2];

    // configurations/constants
    int32_t posn_max_speed; // max speed in this mode
    int32_t posn_min_speed; // minimum speed (to get wheels moving)

    // just so it can be read back
    int32_t posn_diff_mm[2];
    int32_t posn_speed_demand[2];
} PROTOCOL_POSN_DATA;
#pragma pack(pop)


#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_PROTOCOL_SPEED_DATA {
    // these get set
    int32_t wanted_speed_mm_per_sec[2];

    // configurations/constants
    int32_t speed_max_power; // max speed in this mode
    int32_t speed_min_power; // minimum speed (to get wheels moving)
    int32_t speed_minimum_speed; // below this, we don't ask it to do anything

    // just so it can be read back
    int32_t speed_diff_mm_per_sec[2];
    int32_t speed_power_demand[2];
} PROTOCOL_SPEED_DATA;
#pragma pack(pop)


#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_PROTOCOL_PWM_DATA {
    // these get set
    int32_t pwm[2];

    // configurations/constants
    int32_t speed_max_power; // max speed in this mode
    int32_t speed_min_power; // minimum speed (to get wheels moving)
    int32_t speed_minimum_pwm; // below this, we don't ask it to do anything
} PROTOCOL_PWM_DATA;
#pragma pack(pop)



#pragma pack(push, 4)  // long and float are 4 byte each
typedef struct tag_PROTOCOL_HALL_DATA_STRUCT{
    int32_t HallPosn; // 90 per revolution
    int32_t HallSpeed; // speed part calibrated to speed demand value

    float HallPosnMultiplier; // m per hall segment

    int32_t HallPosn_lastread; // posn offset set via protocol in raw value
    int32_t HallPosn_mm; // posn in mm
    int32_t HallPosn_mm_lastread; // posn offset set via protocol in mm
    int32_t HallSpeed_mm_per_s; // speed in m/s

    uint32_t HallTimeDiff;
    uint32_t HallSkipped;
} PROTOCOL_HALL_DATA_STRUCT;
#pragma pack(pop)

#pragma pack(push, 4) // all used types (float and int) are 4 bytes

typedef struct tag_PROTOCOL_MOTOR_ELECTRICAL{
        float dcAmps;
        float dcAmpsAvgAcc;
        float dcAmpsAvg;
        int32_t r1;
        int32_t r2;
        int32_t q;

        int32_t dcAmpsx100;

        int32_t pwm_limiter;
        int32_t pwm_requested;
        int32_t pwm_actual;

        uint32_t limiter_count;
} PROTOCOL_MOTOR_ELECTRICAL;
#pragma pack(pop)

#pragma pack(push, 4) // all used types (float and int) are 4 bytes
typedef struct tag_PROTOCOL_ELECTRICAL_PARAMS{
    int32_t bat_raw;
    float batteryVoltage;

    int32_t board_temp_raw;
    float board_temp_filtered;
    float board_temp_deg_c;

    int32_t charging;

    int32_t dcCurLim; // amps*100
    int32_t dc_adc_limit; // limit expressed in terms of ADC units.

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
    int32_t x;
    int32_t y;
    int32_t degrees;
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
    int32_t LeftAbsolute;
    int32_t RightAbsolute;
    int32_t LeftOffset;
    int32_t RightOffset;
} PROTOCOL_POSN;
#pragma pack(pop)

#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_PROTOCOL_POSN_INCR {
    int32_t Left;
    int32_t Right;
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
    uint32_t period;             // how often should the information be sent?
    int32_t count;                       // how many messages shall be sent? -1 for infinity
    char som;                        // which SOM shall be used? with or without ACK
    uint32_t next_send_time;    // last time a message requiring an ACK was sent

} PROTOCOL_SUBSCRIBEDATA;
#pragma pack(pop)

#pragma pack(push, 4)
typedef struct tag_PROTOCOL_ADC_SETTINGS{
    float adc1_mult_neg;
    float adc1_mult_pos;
    uint16_t adc1_min;
    uint16_t adc1_zero;
    uint16_t adc1_max;
    uint16_t adc2_min;
    uint16_t adc2_zero;
    uint16_t adc2_max;
    float adc2_mult_neg;
    float adc2_mult_pos;
    uint16_t adc_off_start;
    uint16_t adc_off_end;
    float adc_off_filter;
    float adc_relative_steer;
    unsigned char adc_switch_channels;
    unsigned char adc_reverse_steer;
    unsigned char adc_squared_steer;
    unsigned char adc_tankmode;
} PROTOCOL_ADC_SETTINGS;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct tag_PROTOCOL_MSG3 {
    unsigned char SOM;          // 0x00
    unsigned char cmd;          // first bit encodes if a ACK is necessary. Code must never be SOM.
    unsigned char CI;           // continuity counter (Value from 1 to 255)
    unsigned char len;          // len is length of COBS/R encoded message part including CS so it can never be 0x00. Does include optional COBS/R stuffing byte.
    unsigned char bytes[255];   // COBS/R encoded Part of the Message
                                // COBS/R Stuffing byte, only needed in some cases.
} PROTOCOL_MSG3;
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct tag_PROTOCOL_MSG3full {
    unsigned char SOM;          // 0x00
    unsigned char cmd;          // first bit encodes if a ACK is necessary. Code must never be SOM.
    unsigned char CI;           // continuity counter (Value from 1 to 255)
    unsigned char lenPayload;   // len is length of the payload. Changed during decoding, does not includ code, CS and COBS/R Stuffing byte
    unsigned char code;         // optional - code of value to write
    unsigned char content[sizeof( ((PROTOCOL_MSG3 *)0)->bytes ) - sizeof(unsigned char) - sizeof(unsigned char)];
                                // CI and the optional COBS/R stuffing byte are part of bytes and need to be substracted
                                // checksum such that sum of the complete decoded message is zero
} PROTOCOL_MSG3full;
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
    uint32_t rx;              // Count of received messages (valid CS)
    uint32_t rxMissing;       // If message IDs went missing..
    uint32_t tx;              // Count of sent messages (ACK, NACK and retries do not count)
    uint32_t  txRetries;       // how often were messages resend?
    uint32_t  txFailed;        // TX Messages which couldn't be deliveredr. No retries left.

    uint32_t  unwantedacks;              // count of unwated ACK messages
    uint32_t  unwantednacks;             // count of unwanted NACK messges
    uint32_t  unknowncommands;           // count of messages with unknown commands
    uint32_t  unplausibleresponse;       // count of unplausible replies
} PROTOCOLCOUNT;
#pragma pack(pop)


typedef struct tag_PROTOCOLSTATE {
    PROTOCOL_MSG3full curr_send_msg;         // transmit message storage
    char retries;                            // number of retries left to send message
    char lastTXCI;                           // CI of last sent message
    char lastRXCI;                           // CI of last received message in ACKed stream
    uint32_t last_send_time;                 // last time a message requiring an ACK was sent

    PROTOCOLCOUNT counters;                  // Statistical data of the protocol performance
    MACHINE_PROTOCOL_TX_BUFFER TxBuffer;     // Buffer for Messages to be sent
} PROTOCOLSTATE;

typedef struct tag_ASCII {
    int enable_immediate;
    int initialised;
    char ascii_cmd[20];
    char ascii_out[512];
    int ascii_posn;
    int8_t asciiProtocolUnlocked;
} ASCIISTATE;

struct tag_PARAMSTAT;
typedef struct tag_PARAMSTAT PARAMSTAT;

typedef struct tag_PROTOCOL_STAT {
    char allow_ascii;                     // If set to 0, ascii protocol is not used
    uint32_t last_tick_time;              // last time the tick function was called
    char state;                           // state used in protocol_byte to receive messages
    uint32_t last_char_time;              // last time a character was received
    unsigned char CS;                     // temporary storage to calculate Checksum
    unsigned char count;                  // index pointing to last received byte
    PROTOCOL_MSG3 curr_msg;               // received message storage

    char send_state;                      // message transmission state (ACK_TX_WAITING or IDLE)

    int timeout1;                         // ACK has to be received in this time
    int timeout2;                         // when receiving a packet, longest time between characters

    int (*send_serial_data)( unsigned char *data, int len );       // Function Pointer to sending function
    int (*send_serial_data_wait)( unsigned char *data, int len );

    PROTOCOL_SUBSCRIBEDATA subscriptions[10];      // Subscriptions to periodic messages

    PROTOCOLSTATE ack;
    PROTOCOLSTATE noack;
    PARAMSTAT *params[256];
    ASCIISTATE ascii;
    int initialised_functions;
} PROTOCOL_STAT;


// NOTE: content can be NULL if len == 0
typedef void (*PARAMSTAT_FN)( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg );

struct tag_PARAMSTAT {
    unsigned char code;     // code in protocol to refer to this
    char *description;      // if non-null, description
    char *uistr;            // if non-null, used in ascii protocol to adjust with f<str>num<cr>
    char ui_type;           // only UI_NONE or UI_SHORT
    void *ptr;              // pointer to value
    int len;                // length of value

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

#define PROTOCOL_SOM 0
#define PROTOCOL_SOM_ACK 2
#define PROTOCOL_SOM_NOACK 4
//
/////////////////////////////////////////////////////////////////

#define PROTOCOL_CMD_READVAL          'R'  // Request reading a value and request sending it back
#define PROTOCOL_CMD_READVALRESPONSE  'r'  // Sending back the read value. Similar to writing, but does not request confirmation
#define PROTOCOL_CMD_WRITEVAL         'W'  // Writing a value and requesting confirmaion
#define PROTOCOL_CMD_WRITEVALRESPONSE 'w'  // Confirmation for written value
#define PROTOCOL_CMD_SILENTREAD       's'  // Reads a value and triggers callback functions but does not actually send back anything


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


//////////////////////////////////////////////////////
// PUBLIC functions
/////////////////////////////////////////////////////////
extern void ascii_add_immediate( unsigned char letter, int (*fn)(PROTOCOL_STAT *s, char byte,  char *ascii_out), char* description );
extern void ascii_add_line_fn( unsigned char letter, int (*fn)(PROTOCOL_STAT *s, char *line, char *ascii_out), char *description );
extern int ascii_init(PROTOCOL_STAT *s);
// Set entry in params
extern int setParam( PROTOCOL_STAT *s, PARAMSTAT *param );
/////////////////////////////////////////////////////////////////
// Change variable at runtime
extern int setParamVariable( PROTOCOL_STAT *s, unsigned char code, char ui_type, void *ptr, int len);
/////////////////////////////////////////////////////////////////
// Register new function handler at runtime
extern int setParamHandler( PROTOCOL_STAT *s, unsigned char code, PARAMSTAT_FN callback );
/////////////////////////////////////////////////////////////////
// Default Param Handler, replies to Messages
void fn_defaultProcessing ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg );
void fn_defaultProcessingReadOnly ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg );
void fn_ping ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg );
/////////////////////////////////////////////////////////////////
// call this with received bytes; normally from main loop
void protocol_byte( PROTOCOL_STAT *s, unsigned char byte );
/////////////////////////////////////////////////////////////////
// call this schedule a message. CI and Checksum are added
int protocol_post(PROTOCOL_STAT *s, PROTOCOL_MSG3full *msg);
/////////////////////////////////////////////////////////////////
// call this regularly from main.c
void protocol_tick(PROTOCOL_STAT *s);
/////////////////////////////////////////////////////////////////
// initialize protocol
int protocol_init(PROTOCOL_STAT *s);
/////////////////////////////////////////////////////////////////
// Send Text over protocol
int protocol_send_text(PROTOCOL_STAT *s, char *message, unsigned char som);
/////////////////////////////////////////////////////////////////

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
