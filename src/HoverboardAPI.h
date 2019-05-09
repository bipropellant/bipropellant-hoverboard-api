#ifndef HoverboardAPI_h
#define HoverboardAPI_h
#define LIBRARY_VERSION	0.0.1


extern "C" {
  #include "protocol.h"
}

class HoverboardAPI
{


  public:

  //Constants used in some of the functions below
  //#define xxy 1

  //commonly used functions **************************************************************************
    HoverboardAPI(int (*send_serial_data)( unsigned char *data, int len ));          // * constructor.
    void sendPWM(int16_t pwm, int16_t steer);
    void sendPWMData(int16_t pwm, int16_t steer);
    void requestHall();
    void protocolPush(unsigned char byte);
    void protocolTick();
    void schedulePWM();
    void scheduleScheduling();
    void requestScheduleHall();
    void requestCounters();
    void printStats(Stream &Port);
    void sendBuzzer(uint8_t buzzerFreq, uint8_t buzzerPattern, uint16_t buzzerLen);

    PARAMSTAT_FN setParamHandler(unsigned char code, PARAMSTAT_FN callback);

    int getTxBufferLevel();


    double getSpeed_kmh();
    double getSteer_kmh();


    //available but not commonly used functions ********************************************************

  private:
    PROTOCOL_STAT s;


};
#endif

