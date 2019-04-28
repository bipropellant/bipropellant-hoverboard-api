/**********************************************************************************************
 * Arduino HoverboardAPI Library - Version 0.0.1
 * Arduino PID Library - Version 0.0.1
 * by Beat at fixme.ch and Philipp hacknology.de
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "HoverboardAPI.h"
extern "C" {
  #include "protocol.h"
  #include "hallinterrupts.h"
  #include "stm32f1xx_hal.h"
  extern int protocol_post(PROTOCOL_STAT *s, PROTOCOL_LEN_ONWARDS *len_bytes);
}


inline uint32_t tickWrapper(void) {
  return (uint32_t) millis();
}



/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
HoverboardAPI::HoverboardAPI(int (*send_serial_data)( unsigned char *data, int len ))
{
  protocol_init(&s);
  s.send_serial_data = send_serial_data;
  HAL_GetTick = tickWrapper;
  HAL_Delay = delay;
}


// Binary UART Hoverboard communication



void HoverboardAPI::sendSpeed(int16_t pwm, int16_t steer) {

    PROTOCOL_LEN_ONWARDS newMsg;
    memset((void*)&newMsg,0x00,sizeof(PROTOCOL_LEN_ONWARDS));
    PROTOCOL_LEN_ONWARDS *msg = &newMsg;

    /* Send pwm and steer via protocol */
    PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;
    PWM_STEER_CMD *writespeed = (PWM_STEER_CMD *) writevals->content;

    writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value
    writevals->code = 0x20; // speed data from params array

    writespeed->base_pwm = pwm;
    writespeed->steer = steer;

    msg->len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writespeed) + 1; // 1 for Checksum
    protocol_post(&s, msg);
}



void HoverboardAPI::protocolPush(unsigned char byte) {
  protocol_byte(&s, byte);
}

void HoverboardAPI::protocolTick() {
  protocol_tick(&s);
}


extern "C" PARAMSTAT params[];
extern "C" int paramcount;

void HoverboardAPI::setPreread(unsigned char code, void (*callback)(void)) {
  for (int i = 0; i < paramcount; i++) {
    if (params[i].code == code) {
        params[i].preread = callback;
    }
  }
}

void HoverboardAPI::setPrewrite(unsigned char code, void (*callback)(void)) {
  for (int i = 0; i < paramcount; i++) {
    if (params[i].code == code) {
        params[i].prewrite = callback;
    }
  }
}

void HoverboardAPI::setPostread(unsigned char code, void (*callback)(void)) {
  for (int i = 0; i < paramcount; i++) {
    if (params[i].code == code) {
        params[i].postread = callback;
    }
  }
}

void HoverboardAPI::setPostwrite(unsigned char code, void (*callback)(void)) {
  for (int i = 0; i < paramcount; i++) {
    if (params[i].code == code) {
        params[i].postwrite = callback;
    }
  }
}

void HoverboardAPI::requestHall() {

    PROTOCOL_LEN_ONWARDS newMsg;
    memset((void*)&newMsg,0x00,sizeof(PROTOCOL_LEN_ONWARDS));
    PROTOCOL_LEN_ONWARDS *msg = &newMsg;

    /* Request Hall data via protocol */
    PROTOCOL_BYTES_READVALS *readvals = (PROTOCOL_BYTES_READVALS *) msg->bytes;

    readvals->cmd  = PROTOCOL_CMD_READVAL;  // Read value
    readvals->code = 0x02; // hall data from params array

    msg->len = sizeof(readvals->cmd) + sizeof(readvals->code) + 1; // 1 for Checksum

    protocol_post(&s, msg);

}



void HoverboardAPI::sendBuzzer(uint8_t buzzerFreq, uint8_t buzzerPattern, uint8_t buzzerLen) {

    PROTOCOL_LEN_ONWARDS newMsg;
    memset((void*)&newMsg,0x00,sizeof(PROTOCOL_LEN_ONWARDS));
    PROTOCOL_LEN_ONWARDS *msg = &newMsg;


    PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;
    BUZZER *writebuzzer = (BUZZER *) writevals->content;


    writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value
    writevals->code = 0x21; // buzzer from params array

    writebuzzer->buzzerFreq = buzzerFreq;
    writebuzzer->buzzerPattern = buzzerPattern;
    writebuzzer->buzzerLen = buzzerLen;

    msg->len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writebuzzer) + 1; // 1 for Checksum
    protocol_post(&s, msg);
}



double HoverboardAPI::getSpeed_kmh() {
  return   (HallData[0].HallSpeed_mm_per_s + HallData[1].HallSpeed_mm_per_s) / 2.0 * 3600.0 / 1000000.0;
}

double HoverboardAPI::getSteer_kmh() {
  return   (HallData[0].HallSpeed_mm_per_s * 3600.0 / 1000000.0 )- getSpeed_kmh();
}