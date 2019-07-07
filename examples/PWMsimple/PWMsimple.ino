/*
  Minimalistic example to to get the wheels turning.
  Look at HoverboardAPI.c to learn which other functions are available.

  Further information on https://github.com/bipropellant
*/


#include <HoverboardAPI.h>


int serialWrapper(unsigned char *data, int len) {
 return (int) Serial1.write(data,len);
}
HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);



void setup() {
  Serial1.begin(115200);
}

void loop() {
  hoverboard.sendPWM(100, 30, PROTOCOL_SOM_NOACK);
  delay(30);
}