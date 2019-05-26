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

#include "protocol.h"
#include "hallinterrupts.h"
#include "stm32f1xx_hal.h"
#include "bldc.h"



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
  if(protocol_init(&s) != 0) while(1) {};
  s.send_serial_data = send_serial_data;
  s.send_serial_data_wait = send_serial_data;
  s.allow_ascii = 0;       // do not allow ASCII parsing.
//  s.timeout1 = 50; //timeout for ACK
//  s.timeout2 = 10; // timeout between characters
  HAL_GetTick = tickWrapper;
  HAL_Delay = delay;
  consoleLog = printWrapper;
  setParamHandler(Codes::sensHall, NULL); // Disable callbacks for Hall
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

PARAMSTAT_FN HoverboardAPI::setParamHandler(Codes code, PARAMSTAT_FN callback) {
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
  extern PROTOCOLCOUNT ProtocolcountData;

/*
  snprintf ( buffer, 100, "ACK RX: %4li TX: %4li RXmissing: %4li TXretries: %4i    ", s.ack.counters.rx, s.ack.counters.tx, s.ack.counters.rxMissing, s.ack.counters.txRetries);
  port.print(buffer);
  snprintf ( buffer, 100, "NOACK RX: %4li TX: %4li RXmissing: %4li TXretries: %4i    ", s.noack.counters.rx, s.noack.counters.tx, s.noack.counters.rxMissing, s.noack.counters.txRetries);
  port.print(buffer);
  snprintf ( buffer, 100, "Received RX: %4li TX: %4li RXmissing: %4li TXretries: %4i    ",  ProtocolcountData.rx, ProtocolcountData.tx, ProtocolcountData.rxMissing, ProtocolcountData.txRetries);
  port.print(buffer);
*/

  snprintf ( buffer, 100, "Local  RX: %4li TX: %4li RXmissing: %4li    ", s.ack.counters.rx + s.noack.counters.rx, s.ack.counters.tx + s.noack.counters.tx, s.ack.counters.rxMissing + s.noack.counters.rxMissing);
  port.print(buffer);
  snprintf ( buffer, 100, "Remote RX: %4li TX: %4li RXmissing: %4li    ",  ProtocolcountData.rx, ProtocolcountData.tx, ProtocolcountData.rxMissing);
  port.print(buffer);
  snprintf ( buffer, 100, "Missed Local->Remote %4li (%4li) Remote->Local %4li (%4li)", s.ack.counters.tx + s.noack.counters.tx - ProtocolcountData.rx, ProtocolcountData.rxMissing, ProtocolcountData.tx - s.ack.counters.rx - s.noack.counters.rx, s.ack.counters.rxMissing + s.noack.counters.rxMissing);
  port.print(buffer);
  port.println();
}


/***************************************************************************
 *    Triggers a readout of data from the hoverboard
 *    It is necessary to set a callback if something should happen when the
 *    data arrives. Otherwise the data can just be read from the variable.
 ***************************************************************************/
void HoverboardAPI::requestRead(Codes code, char som) {

    // Compose new Message, no ACK needed.
    PROTOCOL_MSG2 msg = {
      .SOM = som,
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

void HoverboardAPI::scheduleTransmission(Codes code, int count, unsigned int period, char som) {
  SUBSCRIBEDATA subscribeTemp = SubscribeData;

  SubscribeData.code = code;
  SubscribeData.count = count;
  SubscribeData.period = period;
  SubscribeData.som = som;

  for (int i = 0; i < paramcount; i++) {
    if (params[i].code == Codes::protocolSubscriptions) {
      fn_SubscribeData( &s, &params[i], FN_TYPE_POST_WRITE );
    }
  }
  SubscribeData = subscribeTemp;

}

/***************************************************************************
 *    Triggers a periodic readout of data from the hoverboard
 *    It is necessary to set a callback if something should happen when the
 *    data arrives. Otherwise the data can just be read from the variable.
 ***************************************************************************/
void HoverboardAPI::scheduleRead(Codes code, int count, unsigned int period, char som) {

  // Compose new Message, with ACK.
  PROTOCOL_MSG2 msg = {
    .SOM = PROTOCOL_SOM_ACK,
  };

  // Prepare Message structure to write subscription values.
  PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) &(msg.bytes);
  SUBSCRIBEDATA *writesubscribe = (SUBSCRIBEDATA *) writevals->content;

  writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Read value

  // Write into Subscriptions
  writevals->code = Codes::protocolSubscriptions;


  // Code indicating which variable should be read. See params[] in protocol.c
  writesubscribe->code = code;
  writesubscribe->count = count;
  writesubscribe->period = period;
  writesubscribe->som = som;

  msg.len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(*writesubscribe);

/*
  unsigned char* charPtr=(unsigned char*)writesubscribe;
  int i;
  printf("\n\rstructure size ESP: %zu bytes  ",sizeof(SUBSCRIBEDATA));
  for(i=0;i<sizeof(SUBSCRIBEDATA);i++)
      printf("%02x ",charPtr[i]);
*/

  protocol_post(&s, &msg);
}


/***************************************************************************
 * Sends PWM values to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendPWM(int16_t pwm, int16_t steer, char som) {

  // Compose new Message
  PROTOCOL_MSG2 msg = {
    .SOM = som,
  };

  // Prepare Message structure to write PWM values.
  PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) &(msg.bytes);
  PWM_DATA *writespeed = (PWM_DATA *) writevals->content;


  writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value

  writevals->code = Codes::setPointPWM;

  writespeed->pwm[0] = pwm + steer;
  writespeed->pwm[1] = pwm - steer;

  msg.len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writespeed->pwm);
  protocol_post(&s, &msg);
}

/***************************************************************************
 * Sends PWM values and Limits to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendPWMData(int16_t pwm, int16_t steer, int speed_max_power, int speed_min_power, int speed_minimum_pwm, char som) {

  // Compose new Message
  PROTOCOL_MSG2 msg = {
    .SOM = som,
  };

  // Prepare Message structure to write PWM values.
  PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) &(msg.bytes);
  PWM_DATA *writespeed = (PWM_DATA *) writevals->content;


  writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value

  writevals->code = Codes::setPointPWMData;

  writespeed->pwm[0] = pwm + steer;
  writespeed->pwm[1] = pwm - steer;
  writespeed->speed_max_power = speed_max_power;
  writespeed->speed_min_power = speed_min_power;
  writespeed->speed_minimum_pwm = speed_minimum_pwm;


  msg.len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(*writespeed);
  protocol_post(&s, &msg);
}

/***************************************************************************
 * Sends Buzzer data to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendBuzzer(uint8_t buzzerFreq, uint8_t buzzerPattern, uint16_t buzzerLen, char som) {

  // Compose new Message
  PROTOCOL_MSG2 msg = {
    .SOM = som,
  };

  // Prepare Message structure to write buzzer values.
  PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) &(msg.bytes);
  BUZZER_DATA *writebuzzer = (BUZZER_DATA *) writevals->content;


  writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value

  writevals->code = Codes::setBuzzer;

  writebuzzer->buzzerFreq = buzzerFreq;
  writebuzzer->buzzerPattern = buzzerPattern;
  writebuzzer->buzzerLen = buzzerLen;


  msg.len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(*writebuzzer);
  protocol_post(&s, &msg);
}

/***************************************************************************
 * Sends enable to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendEnable(uint8_t newEnable, char som) {

  // Compose new Message
  PROTOCOL_MSG2 msg = {
    .SOM = som,
  };

  // Prepare Message structure to write buzzer values.
  PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) &(msg.bytes);
  uint8_t *writeenable = (uint8_t *) writevals->content;


  writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value

  writevals->code = Codes::enableMotors;

  *writeenable = newEnable;


  msg.len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(*writeenable);
  protocol_post(&s, &msg);
}

/***************************************************************************
 * Reset statistic counters
 ***************************************************************************/
void HoverboardAPI::sendCounterReset(char som) {

  // Compose new Message
  PROTOCOL_MSG2 msg = {
    .SOM = som,
  };

  // Prepare Message structure to write buzzer values.
  PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) &(msg.bytes);
  PROTOCOLCOUNT *writeprotocolcount = (PROTOCOLCOUNT *) writevals->content;


  writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value

  writevals->code = Codes::protocolCountSum;

  writeprotocolcount->rx = 0;
  writeprotocolcount->rxMissing = 0;
  writeprotocolcount->tx = 0;
  writeprotocolcount->txRetries = 0;
  writeprotocolcount->txFailed = 0;
  writeprotocolcount->unwantedacks = 0;
  writeprotocolcount->unwantednacks = 0;
  writeprotocolcount->unknowncommands = 0;
  writeprotocolcount->unplausibleresponse = 0;

  msg.len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(*writeprotocolcount);
  protocol_post(&s, &msg);
}

/***************************************************************************
 * Reset statistic counters
 ***************************************************************************/
void HoverboardAPI::resetCounters() {

  s.ack.counters.rx = 0;
  s.ack.counters.rxMissing = 0;
  s.ack.counters.tx = 0;
  s.ack.counters.txRetries = 0;
  s.ack.counters.txFailed = 0;
  s.ack.counters.unwantedacks = 0;
  s.ack.counters.unwantednacks = 0;
  s.ack.counters.unknowncommands = 0;
  s.ack.counters.unplausibleresponse = 0;

  s.noack.counters.rx = 0;
  s.noack.counters.rxMissing = 0;
  s.noack.counters.tx = 0;
  s.noack.counters.txRetries = 0;
  s.noack.counters.txFailed = 0;
  s.noack.counters.unwantedacks = 0;
  s.noack.counters.unwantednacks = 0;
  s.noack.counters.unknowncommands = 0;
  s.noack.counters.unplausibleresponse = 0;
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