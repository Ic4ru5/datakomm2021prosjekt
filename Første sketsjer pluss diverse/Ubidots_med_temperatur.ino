#include "UbidotsEsp32Mqtt.h"
#include <time.h>
#include <stdlib.h>

//Alt med BM280
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
// BME280 I2C address is 0x76(108)
#define Addr 0x76
Adafruit_BME280 bme; // I2C
unsigned long delayTime;
//Slutt med BM280


/****************************************
 * Define Constants
 ****************************************/
const char *UBIDOTS_TOKEN = "BBFF-fj6fUms2Cn65pI6onX4N5oQZvtRbYe";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "8=========D";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "ECDeiendom";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "edvard";   // Put here your Device label to which data  will be published
const char *Temperatur = "temperatur"; // Put here your Variable label to which data  will be published
const char *Fuktighet = "fuktighet";
const char *Trykk = "trykk";
const char *Height = "height";
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

  bme.begin(Addr);  //BM280
  delayTime = 1000; //BM280
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
  if (millis() - timer > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {
    //Temperatur
    float value = bme.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");
    Serial.println();
    ubidots.add(Temperatur, value); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    //Fuktighet
    float value2 = bme.readHumidity();
    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
    Serial.println();
    ubidots.add(Fuktighet, value2); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    //Trykk
    float value3 = bme.readPressure();
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure());
    Serial.println(" hPa");
    Serial.println();
    ubidots.add(Trykk, value3); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    //Moh    
    float value4 = bme.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" moh");
    Serial.println();
    ubidots.add(Height, value4); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);   
    timer = millis();
  }

  
  ubidots.loop();
}
