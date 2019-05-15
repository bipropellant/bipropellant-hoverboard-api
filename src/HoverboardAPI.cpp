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
  #include "bldc.h"
}



/*Constructor (...)*********************************************************
 * Hand over function pointer where to send data.
 *
 * Arduino example:
 *
 *    int serialWrapper(unsigned char *data, int len) {
 *      return (int) Serial.write(data,len);
 *    }
 *    HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);
 *
 ***************************************************************************/
inline uint32_t tickWrapper(void) { return (uint32_t) millis(); }
void printWrapper(const char str[]) {  Serial.print(str); }

HoverboardAPI::HoverboardAPI(int (*send_serial_data)( unsigned char *data, int len )) {
  protocol_init(&s);
  s.send_serial_data = send_serial_data;
  s.send_serial_data_wait = send_serial_data;
  s.allow_ascii = 0;       // do not allow ASCII parsing.
  s.timeout1 = 50; //timeout for ACK
  s.timeout2 = 10; // timeout between characters
  HAL_GetTick = tickWrapper;
  HAL_Delay = delay;
  consoleLog = printWrapper;
  setParamHandler(hoverboardCodes::sensHall, NULL); // Disable callbacks for Hall
}


/***************************************************************************
 * Input function. Feed with Serial.read().
 ***************************************************************************/
void HoverboardAPI::protocolPush(unsigned char byte) {
  protocol_byte(&s, byte);
}

/***************************************************************************
 * Triggers message sending from Buffer and scheduled messages.
 ***************************************************************************/
void HoverboardAPI::protocolTick() {
  protocol_tick(&s);
}

/***************************************************************************
 * Returns local TX Buffer level
 ***************************************************************************/
int HoverboardAPI::getTxBufferLevel() {
  return (mpTxQueued(&s.ack.TxBuffer) + mpTxQueued(&s.noack.TxBuffer));
}

/***************************************************************************
 * Sets a callback Function to handle Protocol Read or Write events
 ***************************************************************************/
extern "C" PARAMSTAT params[];
extern "C" int paramcount;

PARAMSTAT_FN HoverboardAPI::setParamHandler(hoverboardCodes code, PARAMSTAT_FN callback) {
  PARAMSTAT_FN old = NULL;

  for (int i = 0; i < paramcount; i++) {
    if (params[i].code == code) {
        old = params[i].fn;
        params[i].fn = callback;
    }
  }
  return old;
}

/***************************************************************************
 * Print Protocol Statistics. Remote Data has to be requested firs.
 ***************************************************************************/
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


/***************************************************************************
 *    Triggers a readout of data from the hoverboard
 *    It is necessary to set a callback if something should happen when the
 *    data arrives. Otherwise the data can just be read from the variable.
 ***************************************************************************/
void HoverboardAPI::requestRead(hoverboardCodes code) {

    // Compose new Message, no ACK needed.
    PROTOCOL_MSG2 msg = {
      .SOM = PROTOCOL_SOM_NOACK,
    };

    // Message structure is for reading values.
    PROTOCOL_BYTES_READVALS *readvals = (PROTOCOL_BYTES_READVALS *) &(msg.bytes);
    readvals->cmd  = PROTOCOL_CMD_READVAL;  // Read value

    // Code indicating which variable should be read. See params[] in protocol.c
    readvals->code = code;

    msg.len = sizeof(readvals->cmd) + sizeof(readvals->code);

    protocol_post(&s, &msg);
}

/***************************************************************************
 * returns electrical Measurements. Readout has to be requested before with
 * requestRead or scheduling.
 ***************************************************************************/
float HoverboardAPI::getBatteryVoltage() {
  return electrical_measurements.batteryVoltage;
}

float HoverboardAPI::getMotorAmpsAvg(uint8_t motor) {
  if(motor > sizeof(electrical_measurements.motors)/sizeof(electrical_measurements.motors[0])) return -1.0;
  return electrical_measurements.motors[motor].dcAmpsAvg;
}

/***************************************************************************
 * Schedules periodic transmission of value from control to hoverboard
 * count -1 for indefinetely
 ***************************************************************************/
extern "C" {
  extern SUBSCRIBEDATA SubscribeData;
  extern void fn_SubscribeData ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type );
}

void HoverboardAPI::scheduleTransmission(hoverboardCodes code, int count, unsigned int period) {
  SubscribeData.code = code;
  SubscribeData.count = count;
  SubscribeData.period = period;
  SubscribeData.som = PROTOCOL_SOM_NOACK;

  for (int i = 0; i < paramcount; i++) {
    if (params[i].code == hoverboardCodes::protocolSubscriptions) {
      fn_SubscribeData( &s, &params[i], FN_TYPE_POST_WRITE );
    }
  }
}

/***************************************************************************
 * Create local schedule to request periodictransmission of messages.
 * can be used to reinit Message scheduling.
 * Otherwise when messages are scheduled and the other device reboots, the
 * schedule is lost.
 * Can only be done for one code, Subscribe Data is global and for each
 * local subscription there is only one slot.
 ***************************************************************************/
void HoverboardAPI::scheduleScheduling(hoverboardCodes remoteCode, int remoteCount, unsigned int remotePeriod, unsigned int localPeriod, int localCount) {
  // remote subscription
  SubscribeData.code = remoteCode;
  SubscribeData.count = remoteCount;
  SubscribeData.period = remotePeriod;
  SubscribeData.som = PROTOCOL_SOM_NOACK;  // no ack required


  // Subscribe to Subscriptions ;)
  scheduleTransmission(hoverboardCodes::protocolSubscriptions, localCount, localPeriod);
}

/***************************************************************************
 *    Triggers a periodic readout of data from the hoverboard
 *    It is necessary to set a callback if something should happen when the
 *    data arrives. Otherwise the data can just be read from the variable.
 ***************************************************************************/
void HoverboardAPI::scheduleRead(hoverboardCodes code, int count, unsigned int period) {

  // Compose new Message, with ACK.
  PROTOCOL_MSG2 msg = {
    .SOM = PROTOCOL_SOM_ACK,
  };

  // Prepare Message structure to write subscription values.
  PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) &(msg.bytes);
  SUBSCRIBEDATA *writesubscribe = (SUBSCRIBEDATA *) writevals->content;

  writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Read value

  // Write into Subscriptions
  writevals->code = hoverboardCodes::protocolSubscriptions;


  // Code indicating which variable should be read. See params[] in protocol.c
  writesubscribe->code = code;
  writesubscribe->count = count;
  writesubscribe->period = period;
  writesubscribe->som = PROTOCOL_SOM_NOACK;     // Readouts without ACK

  msg.len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writesubscribe);

  protocol_post(&s, &msg);
}


/***************************************************************************
 * Sends PWM values to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendPWM(int16_t pwm, int16_t steer) {

  // Compose new Message, no ACK needed.
  PROTOCOL_MSG2 msg = {
    .SOM = PROTOCOL_SOM_NOACK,
  };

  // Prepare Message structure to write PWM values.
  PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) &(msg.bytes);
  PWM_DATA *writespeed = (PWM_DATA *) writevals->content;


  writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value

  writevals->code = hoverboardCodes::setPointPWM;

  writespeed->pwm[0] = pwm + steer;
  writespeed->pwm[1] = pwm - steer;

  msg.len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writespeed->pwm);
  protocol_post(&s, &msg);
}

/***************************************************************************
 * Sends PWM values and Limits to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendPWMData(int16_t pwm, int16_t steer) {

  // Compose new Message, no ACK needed.
  PROTOCOL_MSG2 msg = {
    .SOM = PROTOCOL_SOM_NOACK,
  };

  // Prepare Message structure to write PWM values.
  PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) &(msg.bytes);
  PWM_DATA *writespeed = (PWM_DATA *) writevals->content;


  writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value

  writevals->code = hoverboardCodes::setPointPWMData;

  writespeed->pwm[0] = pwm + steer;
  writespeed->pwm[1] = pwm - steer;
  writespeed->speed_max_power = 600;
  writespeed->speed_min_power = -600;
  writespeed->speed_minimum_pwm = 10;


  msg.len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writespeed);
  protocol_post(&s, &msg);
}

/***************************************************************************
 * Sends Buzzer data to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendBuzzer(uint8_t buzzerFreq, uint8_t buzzerPattern, uint16_t buzzerLen) {

  // Compose new Message, no ACK needed.
  PROTOCOL_MSG2 msg = {
    .SOM = PROTOCOL_SOM_NOACK,
  };

  // Prepare Message structure to write buzzer values.
  PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) &(msg.bytes);
  BUZZER_DATA *writebuzzer = (BUZZER_DATA *) writevals->content;


  writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value

  writevals->code = hoverboardCodes::setBuzzer;

  writebuzzer->buzzerFreq = buzzerFreq;
  writebuzzer->buzzerPattern = buzzerPattern;
  writebuzzer->buzzerLen = buzzerLen;


  msg.len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writebuzzer);
  protocol_post(&s, &msg);
}



/***************************************************************************
 * returns hall data. Readout has to be requested before with
 * requestRead or scheduling.
 ***************************************************************************/
double HoverboardAPI::getSpeed_kmh() {
  return   (HallData[0].HallSpeed_mm_per_s + HallData[1].HallSpeed_mm_per_s) / 2.0 * 3600.0 / 1000000.0;
}

double HoverboardAPI::getSteer_kmh() {
  return   (HallData[0].HallSpeed_mm_per_s * 3600.0 / 1000000.0 )- getSpeed_kmh();
}