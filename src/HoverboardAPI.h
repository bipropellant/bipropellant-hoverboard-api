#pragma once

#include "hbprotocol/protocol.h"
#ifdef ARDUINO
#include "Stream.h"
#endif

class HoverboardAPI
{
  public:

    enum Codes {
      protocolVersion          = 0xFE,
      protocolSubscriptions    = 0x22,
      protocolCountSum         = 0x23,
      protocolCountACK         = 0x24,
      protocolCountnoACK       = 0x25,
      text                     = 0x26,
      ping                     = 0x27,
      sensHall                 = 0x02,
      setSpeed                 = 0x03,
      sensElectrical           = 0x08,
      enableMotors             = 0x09,
      disablePoweroff          = 0x0A,
      debugOut                 = 0x0B,
      setPointPWMData          = 0x0D,
      setPointPWM              = 0x0E,
      setBuzzer                = 0x21,
      setSpeedKp               = 0x85,
      setSpeedKi               = 0x86,
      setSpeedKd               = 0x87,
      setSpeedIncrLimit        = 0x88,
      adcSettings              = 0x90,
      flashMagic               = 0x80,
      paddleParameters         = 0x80
    };

  //commonly used functions **************************************************************************
    HoverboardAPI();                                                                 // * constructor.
    HoverboardAPI(int (*send_serial_data)( unsigned char *data, int len ));          // * constructor.
    void setSendSerialData(int (*send_serial_data)( unsigned char *data, int len ));


    void protocolPush(unsigned char byte);
    void protocolTick();
    PARAMSTAT_FN updateParamHandler(Codes code, PARAMSTAT_FN callback);
    int updateParamVariable(Codes code, void *ptr, int len);

    void scheduleTransmission(Codes code, int count, unsigned int period, char som = PROTOCOL_SOM_NOACK);

    void requestRead(Codes code, char som = PROTOCOL_SOM_NOACK);
    void scheduleRead(Codes code, int count, unsigned int period, char som = PROTOCOL_SOM_NOACK);


    void sendPWM(int16_t pwm, int16_t steer = 0, char som = PROTOCOL_SOM_NOACK);
    void sendPing(char som = PROTOCOL_SOM_ACK);
    int sendText(char *message, unsigned char som = PROTOCOL_SOM_ACK);
    void receiveText(char *message);

    void sendDifferentialPWM(int16_t left_cmd, int16_t right_cmd, char som = PROTOCOL_SOM_NOACK);
    void sendPWMData(int16_t pwm, int16_t steer = 0, int speed_max_power = 600, int speed_min_power = -600, int speed_minimum_pwm = 10, char som = PROTOCOL_SOM_ACK);
    void sendSpeedData(double left_speed, double right_speed, int16_t max_power, int16_t min_speed, char som = PROTOCOL_SOM_NOACK);
    void sendPIDControl(int16_t Kp, int16_t Ki, int16_t Kd, int16_t speed_increment, char som = PROTOCOL_SOM_NOACK);
    void sendRawData(unsigned char cmd, unsigned char code, unsigned char *content, unsigned char len, unsigned char som);
    void protocolPost(PROTOCOL_MSG3full *msg);


    void sendEnable(uint8_t newEnable, char som = PROTOCOL_SOM_ACK);
    void sendBuzzer(uint8_t buzzerFreq = 4, uint8_t buzzerPattern = 0, uint16_t buzzerLen = 100, char som = PROTOCOL_SOM_NOACK);
    void sendCounterReset(char som = PROTOCOL_SOM_ACK);

    float getBatteryVoltage();
    float getMotorAmpsAvg(uint8_t motor);
    int getTxBufferLevel();
    double getSpeed_kmh();
    double getSteer_kmh();
    double getSpeed_mms();
    double getSteer_mms();
    double getSpeed0_kmh();
    double getSpeed1_kmh();
    double getSpeed0_mms();
    double getSpeed1_mms();

    double getPosition0_mm();
    double getPosition1_mm();

    void resetCounters();
    void printStats();

    PROTOCOL_STAT s;

};

