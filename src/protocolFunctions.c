#include "hallinterrupts.h"
#include "protocol.h"
#include "stm32f1xx_hal.h"
#include "bldc.h"

// Called to reset the system from protocol. Not needed here.
void resetSystem() {}

uint32_t noTick(void) { return 0; };
uint32_t (*HAL_GetTick)(void) = noTick;

void noDelay(uint32_t Delay) {};
void (*HAL_Delay)(uint32_t Delay) = noDelay;

void noReset(void) {};
void (*HAL_NVIC_SystemReset)(void) = noReset;

//////////////////////////////////////////////
// Function pointer which can be set for "debugging"
void noprint(const char str[]) {};
void (*consoleLog)(const char str[]) = noprint;


uint8_t debug_out=0;
uint8_t disablepoweroff=0;
int powerofftimer=0;
uint8_t buzzerFreq=0;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
uint8_t buzzerPattern=0; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
uint16_t buzzerLen=0;
uint8_t enablescope=0; // enable scope on values
int steer=0; // global variable for steering. -1000 to 1000
int speed=0; // global variable for speed. -1000 to 1000

uint8_t enable=0; // global variable for motor enable
volatile uint32_t timeout=0; // global variable for timeout

volatile HALL_DATA_STRUCT HallData[2];
volatile HALL_PARAMS local_hall_params[2];

int pwms[2] = {0, 0};
int dspeeds[2] = {0,0};

volatile ELECTRICAL_PARAMS electrical_measurements;

TIME_STATS timeStats;