#include "UbidotsEsp32Mqtt.h"
#include <time.h>
#include <stdlib.h>

/****************************************
 * Define Constants
 ****************************************/
const char *UBIDOTS_TOKEN = "BBFF-xnntRZDHI7xlR2aAk1DPCapV4tlbuP";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "AndroidAP";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "melk4534";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "test";   // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL = "618431ef192d6a481dfebd34"; // Put here your Variable label to which data  will be published
const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

unsigned long timer;
Ubidots ubidots(UBIDOTS_TOKEN);

/****************************************
 * Auxiliar Functions
 ****************************************/

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  srand(time(NULL));  
  //ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  timer = millis();
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
  if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {
    int x = rand()%1000; 
    float value = x;
    ubidots.add(VARIABLE_LABEL, value); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }
  ubidots.loop();
}