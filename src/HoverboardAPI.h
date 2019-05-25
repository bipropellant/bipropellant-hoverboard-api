#ifndef HoverboardAPI_h
#define HoverboardAPI_h
#define LIBRARY_VERSION	0.0.1


#include "protocol.h"

class HoverboardAPI
{
  public:

    enum Codes {
      protocolVersion          = 0x00,
      protocolSubscriptions    = 0x22,
      protocolCountSum         = 0x23,
      protocolCountACK         = 0x23,
      protocolCountnoACK       = 0x23,
      sensHall                 = 0x02,
      sensElectrical           = 0x08,
      enableMotors             = 0x09,
      disablePoweroff          = 0x0A,
      debugOut                 = 0x0B,
      setPointPWMData          = 0x0D,
      setPointPWM              = 0x0E,
      setBuzzer                = 0x21,
    };

  //commonly used functions **************************************************************************
    HoverboardAPI(int (*send_serial_data)( unsigned char *data, int len ));          // * constructor.

    void protocolPush(unsigned char byte);
    void protocolTick();
    PARAMSTAT_FN setParamHandler(Codes code, PARAMSTAT_FN callback);

    void scheduleTransmission(Codes code, int count, unsigned int period, char som = PROTOCOL_SOM_NOACK);

    void requestRead(Codes code, char som = PROTOCOL_SOM_NOACK);
    void scheduleRead(Codes code, int count, unsigned int period, char som = PROTOCOL_SOM_NOACK);


    void sendPWM(int16_t pwm, int16_t steer = 0, char som = PROTOCOL_SOM_NOACK);
    void sendPWMData(int16_t pwm, int16_t steer = 0, int speed_max_power = 600, int speed_min_power = -600, int speed_minimum_pwm = 10, char som = PROTOCOL_SOM_ACK);
    void sendEnable(uint8_t newEnable, char som = PROTOCOL_SOM_ACK);
    void sendBuzzer(uint8_t buzzerFreq = 4, uint8_t buzzerPattern = 0, uint16_t buzzerLen = 100, char som = PROTOCOL_SOM_NOACK);
    void sendCounterReset(char som = PROTOCOL_SOM_ACK);

    float getBatteryVoltage();
    float getMotorAmpsAvg(uint8_t motor);
    int getTxBufferLevel();
    double getSpeed_kmh();
    double getSteer_kmh();

    void resetCounters();
    void printStats(Stream &Port);


    //available but not commonly used functions ********************************************************

  private:
    PROTOCOL_STAT s;


};
#endif

