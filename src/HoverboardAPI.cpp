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

#include <HoverboardAPI.h>

#define RX1_pin 16
#define TX1_pin 15

typedef struct MsgToHoverboard_t{
    unsigned char SOM;  // 0x02
    unsigned char len;  // len is len of ALL bytes to follow, including CS
    unsigned char cmd;  // 'W'
    unsigned char code; // code of value to write
    int16_t base_pwm;   // absolute value ranging from -1000 to 1000 .. base_pwm plus/minus steer is the raw PWM value 
    int16_t steer;      // absolute value ranging from -1000 to 1000 .. wether steer is added or substracted depends on the side R/L
    unsigned char CS;   // checksumm
  };

  typedef union UART_Packet_t{
    MsgToHoverboard_t msgToHover;
    byte UART_Packet[sizeof(MsgToHoverboard_t)];
  };

 
/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
HoverboardAPI::HoverboardAPI()
{
      Serial1.begin(115200,SERIAL_8N1,RX1_pin,TX1_pin);
}


// Binary UART Hoverboard communication



void HoverboardAPI::setHoverboardTraction( int16_t base_pwm, int16_t steer )   
{
  UART_Packet_t ups; 

  ups.msgToHover.SOM = 2 ;  // PROTOCOL_SOM; //Start of Message;  
  ups.msgToHover.len = 7;   // payload + SC only
  ups.msgToHover.cmd  = 'W'; // PROTOCOL_CMD_WRITEVAL;  // Write value
  ups.msgToHover.code = 0x07; // speed data from params array
  ups.msgToHover.base_pwm = base_pwm;
  ups.msgToHover.steer = steer;
  ups.msgToHover.CS = 0;
  
  for (int i = 0; i < ups.msgToHover.len; i++){
    ups.msgToHover.CS -= ups.UART_Packet[i+1];
  }
    
  Serial1.write(ups.UART_Packet,sizeof(UART_Packet_t));
}
