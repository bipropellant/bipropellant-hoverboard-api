/*
  Scheduled transmission and reading of data.
  Look at HoverboardAPI.c to learn which other functions are available.

  Further information on https://github.com/bipropellant
*/


#include <HoverboardAPI.h>


// Setup HoverboardAPI to use Serial1
int serialWrapper(unsigned char *data, int len) {
 return (int) Serial1.write(data,len);
}
HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);

// Variable for PWM
PROTOCOL_PWM_DATA PWMData;
PROTOCOL_BUZZER_DATA buzzer = {8, 0, 50};

void setup() {
  Serial.begin(115200);   // Serial for debugging
  Serial1.begin(115200);  // Connection to hoverboard

  // Request Hall sensor data every 100ms
  hoverboard.scheduleRead(HoverboardAPI::Codes::sensHall, -1, 100);

  // Request Electrical Information every second
  hoverboard.scheduleRead(hoverboard.Codes::sensElectrical, -1, 1000);

  // Register Variable and send PWM values periodically
  hoverboard.updateParamVariable(HoverboardAPI::Codes::setPointPWM, &PWMData, sizeof(PWMData));
  hoverboard.scheduleTransmission(HoverboardAPI::Codes::setPointPWM, -1, 30);

  // Send PWM to 10% duty cycle for wheel1, 15% duty cycle for wheel2. Wheel2 is running backwards.
  PWMData.pwm[0] = 100.0;
  PWMData.pwm[1] = -150.0;

  // Register Variable and send Buzzer values periodically
  hoverboard.updateParamVariable(HoverboardAPI::Codes::setBuzzer, &buzzer, sizeof(buzzer));
  hoverboard.scheduleTransmission(HoverboardAPI::Codes::setBuzzer, -1, 200);

  // Set maxium PWM to 400, Minimum to -400 and threshold to 30. Require ACK (Message will be resend when not Acknowledged)
  hoverboard.sendPWMData(0, 0, 400, -400, 30, PROTOCOL_SOM_ACK);

}

void loop() {


  // Print Speed on debug Serial.
  Serial.print("Motor Speed: ");
  Serial.print(hoverboard.getSpeed_kmh());
  Serial.println("km/h (average wheel1 and wheel2");

  // Print battery Voltage on debug Serial.
  Serial.print("Battery Voltage: ");
  Serial.print(hoverboard.getBatteryVoltage());
  Serial.println("V");

  // Create annoying melody
  buzzer.buzzerFreq++;


  // Delay loop() for 300 ms use the time to send and receive UART protocol every 100 microseconds
  // This way messages which are received can be processed and scheduled messages can be sent.
  unsigned long start = millis();
  while (millis() < start + 300){

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