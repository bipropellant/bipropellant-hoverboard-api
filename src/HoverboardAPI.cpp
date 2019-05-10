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
}


inline uint32_t tickWrapper(void) {
  return (uint32_t) millis();
}

void printWrapper(const char str[]) {
  Serial.print(str);
}



/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
HoverboardAPI::HoverboardAPI(int (*send_serial_data)( unsigned char *data, int len ))
{
  protocol_init(&s);
  s.send_serial_data = send_serial_data;
  s.send_serial_data_wait = send_serial_data;
  s.allow_ascii = 0;       // do not allow ASCII parsing.
  s.timeout1 = 50; //timeout for ACK
  s.timeout2 = 10; // timeout between characters
  HAL_GetTick = tickWrapper;
  HAL_Delay = delay;
  consoleLog = printWrapper;
  setParamHandler(0x21, NULL); // Disable callbacks for Hall
}


// Binary UART Hoverboard communication



void HoverboardAPI::sendPWM(int16_t pwm, int16_t steer) {

    PROTOCOL_MSG2 newMsg;
    memset((void*)&newMsg,0x00,sizeof(PROTOCOL_MSG2));
    PROTOCOL_MSG2 *msg = &newMsg;

    /* Send pwm and steer via protocol */
    PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;
    PWM_DATA *writespeed = (PWM_DATA *) writevals->content;

    writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value
    writevals->code = 0x0E; // pwm data (only PWM) from params array

    writespeed->pwm[0] = pwm + steer;
    writespeed->pwm[1] = pwm - steer;
    msg->SOM = PROTOCOL_SOM_NOACK;

    msg->len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writespeed->pwm);
    protocol_post(&s, msg);
}

void HoverboardAPI::sendPWMData(int16_t pwm, int16_t steer) {

    PROTOCOL_MSG2 newMsg;
    memset((void*)&newMsg,0x00,sizeof(PROTOCOL_MSG2));
    PROTOCOL_MSG2 *msg = &newMsg;

    /* Send pwm and steer via protocol */
    PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;
    PWM_DATA *writespeed = (PWM_DATA *) writevals->content;

    writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value
    writevals->code = 0x0D; // pwm data from params array

    writespeed->pwm[0] = pwm - steer;
    writespeed->pwm[1] = pwm + steer;
    writespeed->speed_max_power = 600;
    writespeed->speed_min_power = -600;
    writespeed->speed_minimum_pwm = 10;
    msg->SOM = PROTOCOL_SOM_NOACK;

    msg->len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writespeed);
    protocol_post(&s, msg);
}

void HoverboardAPI::protocolPush(unsigned char byte) {
  protocol_byte(&s, byte);
}

void HoverboardAPI::protocolTick() {
  protocol_tick(&s);
}


int HoverboardAPI::getTxBufferLevel() {
  return (mpTxQueued(&s.ack.TxBuffer) + mpTxQueued(&s.noack.TxBuffer));
}

extern "C" PARAMSTAT params[];
extern "C" int paramcount;

PARAMSTAT_FN HoverboardAPI::setParamHandler(unsigned char code, PARAMSTAT_FN callback) {
  PARAMSTAT_FN old = NULL;

  for (int i = 0; i < paramcount; i++) {
    if (params[i].code == code) {
        old = params[i].fn;
        params[i].fn = callback;
    }
  }
  return old;
}


void HoverboardAPI::printStats(Stream &port) {
  char buffer [100];
  int len;
  extern PROTOCOLCOUNT ProtocolcountData;


  len = snprintf ( buffer, 100, "ACK RX: %4li TX: %4li RXmissing: %4li TXretries: %4i    ", s.ack.counters.rx, s.ack.counters.tx, s.ack.counters.rxMissing, s.ack.counters.txRetries);
  port.print(buffer);
  len = snprintf ( buffer, 100, "NOACK RX: %4li TX: %4li RXmissing: %4li TXretries: %4i    ", s.noack.counters.rx, s.noack.counters.tx, s.noack.counters.rxMissing, s.noack.counters.txRetries);
  port.print(buffer);
  len = snprintf ( buffer, 100, "Received RX: %4li TX: %4li RXmissing: %4li TXretries: %4i    ",  ProtocolcountData.rx, ProtocolcountData.tx, ProtocolcountData.rxMissing, ProtocolcountData.txRetries);
  port.print(buffer);

  port.println();
}


void HoverboardAPI::requestCounters() {

    PROTOCOL_MSG2 msg;
    memset((void*)&msg,0x00,sizeof(PROTOCOL_MSG2));

    /* Request Counters data via protocol */
    PROTOCOL_BYTES_READVALS *readvals = (PROTOCOL_BYTES_READVALS *) &(msg.bytes);

    readvals->cmd  = PROTOCOL_CMD_READVAL;  // Read value
    readvals->code = 0x23; // fn_ProtocolcountDataSum

    msg.SOM = PROTOCOL_SOM_ACK;
    msg.len = sizeof(readvals->cmd) + sizeof(readvals->code) + 1; // 1 for Checksum

    protocol_post(&s, &msg);
}


void HoverboardAPI::requestHall() {

    PROTOCOL_MSG2 msg;
    memset((void*)&msg,0x00,sizeof(PROTOCOL_MSG2));

    /* Request Hall data via protocol */
    PROTOCOL_BYTES_READVALS *readvals = (PROTOCOL_BYTES_READVALS *) &(msg.bytes);

    readvals->cmd  = PROTOCOL_CMD_READVAL;  // Read value
    readvals->code = 0x02; // hall data from params array

    msg.SOM = PROTOCOL_SOM_NOACK;
    msg.len = sizeof(readvals->cmd) + sizeof(readvals->code);

    protocol_post(&s, &msg);
}

extern "C" {
  extern SUBSCRIBEDATA SubscribeData;
}

void HoverboardAPI::schedulePWM() {
  SubscribeData.code = 0x0E;               // PWM values only
  SubscribeData.count = -1;                // indefinetely
  SubscribeData.period = 30;               // all 30 ms
  SubscribeData.som = PROTOCOL_SOM_NOACK;

  // PostWrite_setSubscription(&s);
  for (int i = 0; i < paramcount; i++) {
    if (params[i].code == 0x22) {        // 0x22 for subscriptions
      if (params[i].fn) params[i].fn( &s, &params[i], FN_TYPE_POST_WRITE );
    }
  }
}

void HoverboardAPI::scheduleScheduling() {

  // local subscription
  SUBSCRIBEDATA localSubscribe;

  localSubscribe.code = 0x22;               // Subscribe to Subscriptions ;)
  localSubscribe.count = -1;                // indefinetely
  localSubscribe.period = 1000;             // ever second
  localSubscribe.som = PROTOCOL_SOM_ACK;    // verify delivery


  // remote subscription
  SubscribeData.code = 0x02;               // HallData
  SubscribeData.count = 100;               // 100 * 30 ms = 3s
  SubscribeData.period = 30;               // 100 * 30 ms = 3s
  SubscribeData.som = PROTOCOL_SOM_NOACK;  // no ack required



  int len = sizeof(s.subscriptions)/sizeof(s.subscriptions[0]);
  int index = 0;

  // look for vacant local subscription slot
  for (index = 0; index < len; index++) {
      if( s.subscriptions[index].code == 0 || s.subscriptions[index].count == 0 ) {
          break;
      }
  }

  // Fill in new subscription when possible; Plausibility check for period
  if(index < len) {
      s.subscriptions[index] = localSubscribe;
  }
}

void HoverboardAPI::requestScheduleHall() {


  PROTOCOL_MSG2 msg;
  memset((void*)&msg,0x00,sizeof(PROTOCOL_MSG2));

  /* Request Hall data via protocol */
  PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) &(msg.bytes);
  SUBSCRIBEDATA *writesubscribe = (SUBSCRIBEDATA *) writevals->content;


  writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Read value
  writevals->code = 0x22; // hall data from params array

  writesubscribe->code = 0x02;               // HallData
  writesubscribe->count = -1;                // indefinetely
  writesubscribe->period = 30;               // all 30 ms
  writesubscribe->som = PROTOCOL_SOM_NOACK;

  msg.SOM = PROTOCOL_SOM_ACK;
  msg.len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writesubscribe);

  protocol_post(&s, &msg);
}



void HoverboardAPI::sendBuzzer(uint8_t buzzerFreq, uint8_t buzzerPattern, uint16_t buzzerLen) {

    PROTOCOL_MSG2 newMsg;
    memset((void*)&newMsg,0x00,sizeof(PROTOCOL_MSG2));
    PROTOCOL_MSG2 *msg = &newMsg;


    PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;
    BUZZER_DATA *writebuzzer = (BUZZER_DATA *) writevals->content;


    writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value
    writevals->code = 0x21; // buzzer from params array

    writebuzzer->buzzerFreq = buzzerFreq;
    writebuzzer->buzzerPattern = buzzerPattern;
    writebuzzer->buzzerLen = buzzerLen;
    msg->SOM = PROTOCOL_SOM_NOACK;

    msg->len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writebuzzer);
    protocol_post(&s, msg);
}



double HoverboardAPI::getSpeed_kmh() {
  return   (HallData[0].HallSpeed_mm_per_s + HallData[1].HallSpeed_mm_per_s) / 2.0 * 3600.0 / 1000000.0;
}

double HoverboardAPI::getSteer_kmh() {
  return   (HallData[0].HallSpeed_mm_per_s * 3600.0 / 1000000.0 )- getSpeed_kmh();
}