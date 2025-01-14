#include <Arduino.h>
// #include <dht_nonblocking.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
// #include <DHT_U.h>
// #include <DHT11.h>

/*
NOTE:
The DHT11 is really unprecise
*/

#define TMP_SENSOR_EXT A0

#define DHTPIN 2 // Pin where the DHT sensor is connected
#define DHTTYPE DHT11 // DHT 11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  dht.begin();
  pinMode(TMP_SENSOR_EXT, INPUT);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  uint16_t val_ext = analogRead(TMP_SENSOR_EXT);
  float temp_ext = (((val_ext)*(5.0/1024.0)) - 0.5)*100;

  float humidity = dht.readHumidity();
  float temperature_int = dht.readTemperature();

  Serial.print("Temperature eau: ");
  Serial.println(temperature_int);
  Serial.print(" exterieur: ");
  Serial.println(temp_ext);
}