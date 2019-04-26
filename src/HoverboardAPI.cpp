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
}

Stream *serialport = NULL;

int serialWrapper(unsigned char *data, int len) {
      return (int) serialport->write(data,len);
}

//extern "C" int (*send_serial_data)( unsigned char *data, int len );
//int (*send_serial_data)( unsigned char *data, int len ) = serialWrapper;
extern "C" int (*send_serial_data)( unsigned char *data, int len );  // why do I need to do that? This is already done in protocol.h!?


uint32_t tickWrapper(void) {
  return (uint32_t) millis;
}

extern "C" uint32_t (*getTick)(void);




/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
HoverboardAPI::HoverboardAPI(Stream *port)
{
  serialport = port;
  send_serial_data = serialWrapper;
  getTick = tickWrapper;
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
    writevals->code = 0x07; // speed data from params array

    writespeed->base_pwm = pwm;
    writespeed->steer = steer;

    msg->len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writespeed) + 1; // 1 for Checksum
    protocol_post(msg);
}



void HoverboardAPI::protocolPush(unsigned char byte) {

  protocol_byte(byte);
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

    protocol_post(msg);

}



void HoverboardAPI::sendBuzzer(uint8_t buzzerFreq, uint8_t buzzerPattern, uint8_t buzzerLen) {

    PROTOCOL_LEN_ONWARDS newMsg;
    memset((void*)&newMsg,0x00,sizeof(PROTOCOL_LEN_ONWARDS));
    PROTOCOL_LEN_ONWARDS *msg = &newMsg;


    PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;
    BUZZER *writebuzzer = (BUZZER *) writevals->content;


    writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value
    writevals->code = 0x08; // buzzer from params array

    writebuzzer->buzzerFreq = buzzerFreq;
    writebuzzer->buzzerPattern = buzzerPattern;
    writebuzzer->buzzerLen = buzzerLen;

    msg->len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writebuzzer) + 1; // 1 for Checksum


double HoverboardAPI::getSpeed_kmh() {
  return   (HallData[0].HallSpeed_mm_per_s + HallData[1].HallSpeed_mm_per_s) / 2.0 * 3600.0 / 1000000.0;
}

double HoverboardAPI::getSteer_kmh() {
  return   (HallData[0].HallSpeed_mm_per_s * 3600.0 / 1000000.0 )- getSpeed_kmh();
}