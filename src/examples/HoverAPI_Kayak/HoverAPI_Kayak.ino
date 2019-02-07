/********************************************************
 * HoverboardAPI Kayak
 ********************************************************/

#include <Arduino.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <driver/dac.h>
//#include <ArduinoJson.h>
#include <WebSocketsServer.h>

#include <HoverboardAPI.h>

#define SSID "hover"
#define PASSWORD "87654321"
#define SERVER_PORT 5000


typedef struct sensorData_t{
  int16_t x;
  int16_t y;
  int16_t z;
};

typedef union Websock_Packet_t{
sensorData_t sensor;
byte bytePacket[sizeof(sensorData_t)];
};

Websock_Packet_t wsd; 

WiFiServer server(SERVER_PORT);
WebSocketsServer webSocket = WebSocketsServer(81);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
				webSocket.sendTXT(num, "Connected");
            }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);
            break;
        case WStype_BIN:
            memcpy(wsd.bytePacket, payload, length);
            break;
		case WStype_ERROR:			
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
    }

}

HoverboardAPI hov;

void setup() 
{
    Serial.begin(115200);
    delay(1000);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, PASSWORD);
    server.begin();

    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        delay(1000);
    }
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

void loop() {
    static unsigned long last = 0;
    webSocket.loop();
    if(abs(millis() - last) > 1000) {
          last = millis();
          Serial.print("Analog X: ");
          Serial.print(wsd.sensor.x);
          Serial.print(" - Y: ");
          Serial.print(wsd.sensor.y);
          Serial.print(" - Z: ");
          Serial.println(wsd.sensor.z);
    }
    if(wsd.sensor.z < 0 ) wsd.sensor.z = 0;
    if(wsd.sensor.z == 0 ) wsd.sensor.x = 0;
    
    hov.setHoverboardTraction( wsd.sensor.z, wsd.sensor.x ) ;

}