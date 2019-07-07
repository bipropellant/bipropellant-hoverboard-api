/*
  Read back speed.
  Look at HoverboardAPI.c to learn which other functions are available.

  Further information on https://github.com/bipropellant
*/


#include <HoverboardAPI.h>


// Setup HoverboardAPI to use Serial1
int serialWrapper(unsigned char *data, int len) {
 return (int) Serial1.write(data,len);
}
HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);



void setup() {
  Serial.begin(115200);   // Serial for debugging
  Serial1.begin(115200);  // Connection to hoverboard
}

void loop() {

  // Send PWM Value 100 with steer 30 to hoverboard. (7% duty cycle for wheel1, 13% duty cycle for wheel2)
  hoverboard.sendPWM(100, 30, PROTOCOL_SOM_NOACK);

  // Request read of Hall Data (=Motor Speed)
  hoverboard.requestRead(HoverboardAPI::Codes::sensHall, PROTOCOL_SOM_NOACK);

  // Print Speed on debug Serial. Since there was no delay between requesting and printing, this value is probably one cycle old.
  Serial.print("Motor Speed: ");
  Serial.print(hoverboard.getSpeed_kmh());
  Serial.print("km/h (average from ");
  Serial.print(hoverboard.getSpeed0_mms());
  Serial.print("mm/s and ");
  Serial.print(hoverboard.getSpeed1_mms());
  Serial.println("mm/s)");


  // Delay loop() for 30 ms use the time to send and receive UART protocol every 100 microseconds
  // This way messages which are received can be processed and scheduled messages can be sent.
  unsigned long start = millis();
  while (millis() < start + 30){

    // Read and Process Incoming data
    int i=0;
    while(Serial1.available() && i++ < 1024) { // read maximum 1024 bytes at once.
      hoverboard.protocolPush( Serial1.read() );
    }

    // Send Messages from queue, re-send failed messages and process scheduled transmissions
    hoverboard.protocolTick();
    delayMicroseconds(100);
  }

}
