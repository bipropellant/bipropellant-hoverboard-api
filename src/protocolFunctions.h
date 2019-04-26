#pragma once

#include <stdint.h>
#include <stdlib.h>


void resetSystem();


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

