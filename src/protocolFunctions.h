#pragma once

#include <stdint.h>
#include <stdlib.h>


void ascii_byte( unsigned char byte );
void resetSystem();


//size_t send_serial_data( const uint8_t *data, size_t len );
extern int (*send_serial_data)( unsigned char *data, int len );
extern uint32_t (*getTick)(void);



extern uint8_t debug_out;
extern uint8_t disablepoweroff;
extern int powerofftimer;
extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
extern uint16_t buzzerLen;
extern uint8_t enablescope; // enable scope on values
extern int steer; // global variable for steering. -1000 to 1000
extern int speed; // global variable for speed. -1000 to 1000

extern uint8_t enable; // global variable for motor enable
extern volatile uint32_t timeout; // global variable for timeout

//////////////////////////////////////////////////////////////
// this is the Hall data we gather, and can be read elsewhere
// one for each wheel
typedef struct tag_HALL_DATA_STRUCT{
    long HallPosn; // 90 per revolution
    long HallSpeed; // speed part calibrated to speed demand value

    float HallPosnMultiplier; // m per hall segment

    long HallPosn_lastread; // posn offset set via protocol in raw value
    long HallPosn_mm; // posn in mm
    long HallPosn_mm_lastread; // posn offset set via protocol in mm
    long HallSpeed_mm_per_s; // speed in m/s

    unsigned long HallTimeDiff;
    unsigned long HallSkipped;
} HALL_DATA_STRUCT;
extern volatile HALL_DATA_STRUCT HallData[2];