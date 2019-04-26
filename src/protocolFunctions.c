#include "protocolFunctions.h"

// Originally used to parse ASCII characters. Not needed here.
void ascii_byte( unsigned char byte ){}

// Called to reset the system from protocol. Not needed here.
void resetSystem() {}


uint32_t noTick(void) { return 0; };
uint32_t (*getTick)(void) = noTick;


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





#ifdef HOVERBOARDFW

/////////////////////////////////////////////////////////////
// specify where to send data out of with a function pointer.
#ifdef SOFTWARE_SERIAL
static int (*send_serial_data)( unsigned char *data, int len ) = softwareserial_Send;
//static int (*send_serial_data_wait)( unsigned char *data, int len ) = softwareserial_Send_Wait;
#endif

// TODO: Method to select which output is used for Protocol when both are active
#if defined(SERIAL_USART2_IT) && !defined(READ_SENSOR)
extern int USART2_IT_send(unsigned char *data, int len);

static int (*send_serial_data)( unsigned char *data, int len ) = USART2_IT_send;
//static int (*send_serial_data_wait)( unsigned char *data, int len ) = USART2_IT_send;
#elif defined(SERIAL_USART3_IT) && !defined(READ_SENSOR)
extern int USART3_IT_send(unsigned char *data, int len);

static int (*send_serial_data)( unsigned char *data, int len ) = USART3_IT_send;
//static int (*send_serial_data_wait)( unsigned char *data, int len ) = USART3_IT_send;
#endif



void resetSystem() {
    HAL_Delay(500);
    HAL_NVIC_SystemReset();
}

uint32_t getTick(void) {
    return HAL_GetTick();
}


#endif