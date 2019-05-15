#ifndef HoverboardAPI_h
#define HoverboardAPI_h
#define LIBRARY_VERSION	0.0.1


extern "C" {
  #include "protocol.h"
}

enum hoverboardCodes {
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


class HoverboardAPI
{


  public:

  //Constants used in some of the functions below
  //#define xxy 1

  //commonly used functions **************************************************************************
    HoverboardAPI(int (*send_serial_data)( unsigned char *data, int len ));          // * constructor.
    void sendPWM(int16_t pwm, int16_t steer);
    void sendPWMData(int16_t pwm, int16_t steer);

    void requestRead(hoverboardCodes code);
    float getBatteryVoltage();
    float getMotorAmpsAvg(uint8_t motor);

    void protocolPush(unsigned char byte);
    void protocolTick();

    void scheduleTransmission(hoverboardCodes code, int count, unsigned int period);

    void scheduleScheduling(hoverboardCodes remoteCode, int remoteCount, unsigned int remotePeriod, unsigned int localPeriod, int localCount);


    void scheduleRead(hoverboardCodes code, int count, unsigned int period);

    void printStats(Stream &Port);
    void sendBuzzer(uint8_t buzzerFreq, uint8_t buzzerPattern, uint16_t buzzerLen);

    PARAMSTAT_FN setParamHandler(hoverboardCodes code, PARAMSTAT_FN callback);

    int getTxBufferLevel();


    double getSpeed_kmh();
    double getSteer_kmh();


    //available but not commonly used functions ********************************************************

  private:
    PROTOCOL_STAT s;


};
#endif

