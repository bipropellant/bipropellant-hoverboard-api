/**********************************************************************************************
 * HoverboardAPI Library - Version 0.0.3
 **********************************************************************************************/

#include "HoverboardAPI.h"
#include "hbprotocol/protocol.h"
#include "hbprotocol/protocol_private.h"
#include "protocolFunctions.h"
#include <stdio.h>
#include <string.h>



/*Constructor (...)*********************************************************
 * Hand over function pointer where to send data.
 *
 * Arduino example:
 *
 *    int serialWrapper(unsigned char *data, int len)
 *    {
 *      return (int) Serial.write(data,len);
 *    }
 *    HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);
 *
 ***************************************************************************/

#ifdef ARDUINO
extern "C"
{
  extern void delay(uint32_t ms);
  extern unsigned long millis(void);
}
#else
#include <unistd.h>
#include <time.h>
extern "C" {

void delay(uint32_t ms) {
  usleep (ms*1000);
}

static uint64_t ts_start = 0;

unsigned long millis()
{
  struct timespec ts;

  if (ts_start == 0)
  {
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    ts_start = (uint64_t)ts.tv_sec * (uint64_t)1000 + (uint64_t)(ts.tv_nsec / 1000000L) ;
  }

  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  uint64_t ts_now = (uint64_t)ts.tv_sec * (uint64_t)1000 + (uint64_t)(ts.tv_nsec / 1000000L) ;

  return (uint32_t)(ts_now - ts_start);
}

}
#endif

uint32_t tickWrapper(void)
{
  return (uint32_t) millis();
}

HoverboardAPI::HoverboardAPI(int (*send_serial_data)( unsigned char *data, int len )) :
  HoverboardAPI()
{
  setSendSerialData(send_serial_data);
}

void HoverboardAPI::setSendSerialData(int (*send_serial_data)( unsigned char *data, int len ))
{
  s.send_serial_data = send_serial_data;
  s.send_serial_data_wait = send_serial_data;
}

HoverboardAPI::HoverboardAPI()
{
  if(protocol_init(&s) != 0) while(1) {};
  setup_protocol(&s);
  s.allow_ascii = 0;       // do not allow ASCII parsing.
  protocol_GetTick = tickWrapper;
  protocol_Delay = delay;
}

/***************************************************************************
 * Input function. Feed with Serial.read().
 ***************************************************************************/
void HoverboardAPI::protocolPush(unsigned char byte)
{
  protocol_byte(&s, byte);
}

/***************************************************************************
 * Triggers message sending from Buffer and scheduled messages.
 ***************************************************************************/
void HoverboardAPI::protocolTick()
{
  protocol_tick(&s);
}

/***************************************************************************
 * Returns local TX Buffer level
 ***************************************************************************/
int HoverboardAPI::getTxBufferLevel()
{
  return (mpTxQueued(&s.ack.TxBuffer) + mpTxQueued(&s.noack.TxBuffer));
}

/***************************************************************************
 * Sets a callback Function to handle Protocol Read or Write events
 ***************************************************************************/
PARAMSTAT_FN HoverboardAPI::updateParamHandler(Codes code, PARAMSTAT_FN callback)
{
  PARAMSTAT_FN old = getParamHandler(&s, code);
  setParamHandler(&s, code, callback);
  return old;
}

/***************************************************************************
 * Sets a Variable into which data is stored on receive and taken from for
 * scheduled sending.
 ***************************************************************************/
int HoverboardAPI::updateParamVariable(Codes code, void *ptr, int len)
{
  if(s.params[code] != NULL)
  {
    return setParamVariable(&s, code, s.params[code]->ui_type, ptr, len);
  }
  return 1;
}

/***************************************************************************
 * Print Protocol Statistics. Remote Data has to be requested first.
 ***************************************************************************/
void HoverboardAPI::printStats()
{
//   char buffer [100];
   extern PROTOCOLCOUNT ProtocolcountData;

// /*
//   snprintf ( buffer, 100, "ACK RX: %4li TX: %4li RXmissing: %4li TXretries: %4i    ", s.ack.counters.rx, s.ack.counters.tx, s.ack.counters.rxMissing, s.ack.counters.txRetries);
//   port.print(buffer);
//   snprintf ( buffer, 100, "NOACK RX: %4li TX: %4li RXmissing: %4li TXretries: %4i    ", s.noack.counters.rx, s.noack.counters.tx, s.noack.counters.rxMissing, s.noack.counters.txRetries);
//   port.print(buffer);
//   snprintf ( buffer, 100, "Received RX: %4li TX: %4li RXmissing: %4li TXretries: %4i    ",  ProtocolcountData.rx, ProtocolcountData.tx, ProtocolcountData.rxMissing, ProtocolcountData.txRetries);
//   port.print(buffer);
// */

  // printf ( "Local  RX: %4d TX: %4d RXmissing: %4d    ", s.ack.counters.rx + s.noack.counters.rx, s.ack.counters.tx + s.noack.counters.tx, s.ack.counters.rxMissing + s.noack.counters.rxMissing);
  // printf ( "Remote RX: %4d TX: %4d RXmissing: %4d    ",  ProtocolcountData.rx, ProtocolcountData.tx, ProtocolcountData.rxMissing);
  // printf ( "Missed Local->Remote %4d (%4d) Remote->Local %4d (%4d)", s.ack.counters.tx + s.noack.counters.tx - ProtocolcountData.rx, ProtocolcountData.rxMissing, ProtocolcountData.tx - s.ack.counters.rx - s.noack.counters.rx, s.ack.counters.rxMissing + s.noack.counters.rxMissing);
  // printf("\n");
}

/***************************************************************************
 *    Triggers a readout of data from the hoverboard
 *    It is necessary to set a callback if something should happen when the
 *    data arrives. Otherwise the data can just be read from the variable.
 ***************************************************************************/
void HoverboardAPI::requestRead(Codes code, char som)
{
  sendRawData(
    PROTOCOL_CMD_READVAL,
    code,
    NULL,
    0,
    som
  );
}

/***************************************************************************
 * returns electrical Measurements. Readout has to be requested before with
 * requestRead or scheduling.
 ***************************************************************************/
float HoverboardAPI::getBatteryVoltage()
{
  return electrical_measurements.batteryVoltage;
}

float HoverboardAPI::getMotorAmpsAvg(uint8_t motor)
{
  if(motor > sizeof(electrical_measurements.motors)/sizeof(electrical_measurements.motors[0])) return -1.0;
  return electrical_measurements.motors[motor].dcAmpsAvg;
}

/***************************************************************************
 * Schedules periodic transmission of value from control to hoverboard
 * count -1 for indefinetely
 ***************************************************************************/
void HoverboardAPI::scheduleTransmission(Codes code, int count, unsigned int period, char som)
{
  PROTOCOL_MSG3full newMsg;
  memset((void*)&newMsg,0x00,sizeof(PROTOCOL_MSG3full));

  PROTOCOL_SUBSCRIBEDATA *SubscribeData = (PROTOCOL_SUBSCRIBEDATA *) &newMsg.content;
  SubscribeData->code   = code;
  SubscribeData->count  = count;
  SubscribeData->period = period;
  SubscribeData->next_send_time = 0;
  SubscribeData->som = som;

  // Use native Subscription function to fill in array.
  newMsg.cmd  = PROTOCOL_CMD_READVALRESPONSE; // This should prevent further processing
  newMsg.code = Codes::protocolSubscriptions;
  newMsg.SOM = PROTOCOL_SOM_NOACK;
  newMsg.lenPayload = sizeof(PROTOCOL_SUBSCRIBEDATA);

  fn_SubscribeData( &s, s.params[Codes::protocolSubscriptions], PROTOCOL_CMD_READVALRESPONSE, &newMsg );
}

/***************************************************************************
 *    Triggers a periodic readout of data from the hoverboard
 *    It is necessary to set a callback if something should happen when the
 *    data arrives. Otherwise the data can just be read from the variable.
 ***************************************************************************/
void HoverboardAPI::scheduleRead(Codes code, int count, unsigned int period, char som)
{
  PROTOCOL_SUBSCRIBEDATA writesubscribe;
  writesubscribe.code = code;
  writesubscribe.count = count;
  writesubscribe.period = period;
  writesubscribe.som = som;

  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::protocolSubscriptions,
    (unsigned char *) &writesubscribe,
    sizeof(writesubscribe),
    som
  );
}

/***************************************************************************
 * Sends PWM values to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendPWM(int16_t pwm, int16_t steer, char som)
{
  PROTOCOL_PWM_DATA writespeed;
  writespeed.pwm[0] = pwm + steer;
  writespeed.pwm[1] = pwm - steer;

  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::setPointPWM,
    (unsigned char *) &writespeed,
    sizeof(writespeed.pwm),
    som
  );
}

void HoverboardAPI::sendDifferentialPWM(int16_t left_cmd, int16_t right_cmd, char som)
{
  PROTOCOL_PWM_DATA writespeed;
  writespeed.pwm[0] = left_cmd;
  writespeed.pwm[1] = right_cmd;

  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::setPointPWM,
    (unsigned char *) &writespeed,
    sizeof(writespeed.pwm),
    som
  );
}

/***************************************************************************
 * Sends PWM values and Limits to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendPWMData(int16_t pwm,
				int16_t steer,
				int speed_max_power,
				int speed_min_power,
				int speed_minimum_pwm,
				char som)
{
  PROTOCOL_PWM_DATA writespeed;
  writespeed.pwm[0] = pwm + steer;
  writespeed.pwm[1] = pwm - steer;
  writespeed.speed_max_power = speed_max_power;
  writespeed.speed_min_power = speed_min_power;
  writespeed.speed_minimum_pwm = speed_minimum_pwm;

  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::setPointPWMData,
    (unsigned char *) &writespeed,
    sizeof(writespeed),
    som
  );
}

/***************************************************************************
 * Sends speed control to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendSpeedData(double left_speed, double right_speed, int16_t max_power, int16_t min_speed, char som)
{
  PROTOCOL_SPEED_DATA writespeed;
  writespeed.wanted_speed_mm_per_sec[0] = left_speed * 1000;
  writespeed.wanted_speed_mm_per_sec[1] = right_speed * 1000;
  writespeed.speed_max_power = max_power;
  writespeed.speed_min_power = -max_power;
  writespeed.speed_minimum_speed = min_speed;

  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::setSpeed,
    (unsigned char *) &writespeed,
    sizeof(writespeed.wanted_speed_mm_per_sec) + sizeof(writespeed.speed_max_power) + sizeof(writespeed.speed_min_power) + sizeof(writespeed.speed_minimum_speed),
    som
  );
}

void HoverboardAPI::sendPIDControl(int16_t Kp, int16_t Ki, int16_t Kd, int16_t speed_increment, char som)
{
  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::setSpeedKp,
    (unsigned char *) &Kp,
    sizeof(Kp),
    som
  );

  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::setSpeedKi,
    (unsigned char *) &Ki,
    sizeof(Ki),
    som
  );

  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::setSpeedKd,
    (unsigned char *) &Kd,
    sizeof(Kd),
    som
  );

  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::setSpeedIncrLimit,
    (unsigned char *) &speed_increment,
    sizeof(speed_increment),
    som
  );
}

/***************************************************************************
 * Sends Buzzer data to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendBuzzer(uint8_t buzzerFreq, uint8_t buzzerPattern, uint16_t buzzerLen, char som)
{
  PROTOCOL_BUZZER_DATA writebuzzer;
  writebuzzer.buzzerFreq = buzzerFreq;
  writebuzzer.buzzerPattern = buzzerPattern;
  writebuzzer.buzzerLen = buzzerLen;

  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::setBuzzer,
    (unsigned char *) &writebuzzer,
    sizeof(writebuzzer),
    som
  );
}

/***************************************************************************
 * Sends enable to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendEnable(uint8_t newEnable, char som)
{
  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::enableMotors,
    (unsigned char *) &newEnable,
    sizeof(newEnable),
    som
  );
}

/***************************************************************************
 * Reset statistic counters
 ***************************************************************************/
void HoverboardAPI::sendCounterReset(char som)
{
  PROTOCOLCOUNT writeprotocolcount;
  writeprotocolcount.rx = 0;
  writeprotocolcount.rxMissing = 0;
  writeprotocolcount.tx = 0;
  writeprotocolcount.txRetries = 0;
  writeprotocolcount.txFailed = 0;
  writeprotocolcount.unwantedacks = 0;
  writeprotocolcount.unwantednacks = 0;
  writeprotocolcount.unknowncommands = 0;
  writeprotocolcount.unplausibleresponse = 0;

  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::protocolCountSum,
    (unsigned char *) &writeprotocolcount,
    sizeof(writeprotocolcount),
    som
  );
}

/***************************************************************************
 * Reset statistic counters
 ***************************************************************************/
void HoverboardAPI::resetCounters()
{
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
double HoverboardAPI::getSpeed_kmh()
{
  return   (HallData[0].HallSpeed_mm_per_s + HallData[1].HallSpeed_mm_per_s) / 2.0 * 3600.0 / 1000000.0;
}

double HoverboardAPI::getSteer_kmh()
{
  return   (HallData[0].HallSpeed_mm_per_s * 3600.0 / 1000000.0 )- getSpeed_kmh();
}

double HoverboardAPI::getSpeed_mms()
{
  return   (HallData[0].HallSpeed_mm_per_s + HallData[1].HallSpeed_mm_per_s) / 2.0;
}

double HoverboardAPI::getSteer_mms()
{
  return   HallData[0].HallSpeed_mm_per_s - getSpeed_mms();
}

double HoverboardAPI::getSpeed0_mms()
{
  return   HallData[0].HallSpeed_mm_per_s;
}

double HoverboardAPI::getSpeed1_mms()
{
  return   HallData[1].HallSpeed_mm_per_s;
}

double HoverboardAPI::getSpeed0_kmh()
{
  return   HallData[0].HallSpeed_mm_per_s * 3600.0 / 1000000.0;
}

double HoverboardAPI::getSpeed1_kmh()
{
  return   HallData[1].HallSpeed_mm_per_s * 3600.0 / 1000000.0;
}

double HoverboardAPI::getPosition0_mm()
{
  return   HallData[0].HallPosn_mm;
}

double HoverboardAPI::getPosition1_mm()
{
  return   HallData[1].HallPosn_mm;
}

/***************************************************************************
 * Sends Raw Data to hoverboard
 ***************************************************************************/
void HoverboardAPI::sendRawData(
        unsigned char msgCmd,
				unsigned char msgCode,
        unsigned char *content,
        unsigned char len,
				unsigned char msgSom)
{
  // Compose new Message
  PROTOCOL_MSG3full msg =
  {
    SOM        : msgSom,
    cmd        : msgCmd,
    CI         : 0,
    lenPayload : len,
    code       : msgCode
  };

  if(len <= sizeof( ((PROTOCOL_MSG3full *)0)->content ) ) memcpy(&msg.content, content, len);

  protocol_post(&s, &msg);
}

/***************************************************************************
 * Sends Raw msg to hoverboard
 ***************************************************************************/
void HoverboardAPI::protocolPost(PROTOCOL_MSG3full *msg)
{
  protocol_post(&s, msg);
}

/***************************************************************************
 * Sends Text msg to hoverboard
 ***************************************************************************/
int HoverboardAPI::sendText(char *message, unsigned char som)
{
  return protocol_send_text(&s, message, som);
}

/***************************************************************************
 * Fakes Receiving a Message from hoverboard (Useful for debugging)
 ***************************************************************************/
void HoverboardAPI::receiveText(char *message)
{
  if( (s.params[0x26]) && (strlen(message) <= s.params[0x26]->len ) )
  {
      PROTOCOL_MSG3full newMsg;
      memset((void*)&newMsg,0x00,sizeof(PROTOCOL_MSG3full));

      newMsg.SOM = PROTOCOL_SOM_NOACK;
      newMsg.lenPayload = strlen(message) + 1; // +1 for Null character \0

      newMsg.cmd  = PROTOCOL_CMD_READVALRESPONSE;
      newMsg.code = 0x26;                    // 0x26 for text
      strcpy( (char *) newMsg.content, message);

      protocol_process_message(&s, &newMsg);
  }
}

/***************************************************************************
 * Sends a ping to the hoverboard containing a timestamp
 ***************************************************************************/
void HoverboardAPI::sendPing(char som)
{
  unsigned long time = millis();

  sendRawData(
    PROTOCOL_CMD_WRITEVAL,
    Codes::ping,
    (unsigned char *) &time,
    sizeof(time),
    som
  );
}
