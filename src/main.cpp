#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define TMP_SENSOR_EXT A0

#define DHTPIN 2 // Pin where the DHT sensor is connected
#define DHTTYPE DHT11 // DHT 11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dht.begin();
  pinMode(TMP_SENSOR_EXT, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  uint8_t val_ext = analogRead(TMP_SENSOR_EXT);
  float_t temp_ext = (val_ext-0.5)*100;

  float_t humidity = dht.readHumidity();
  float_t temperature_int = dht.readTemperature();

  Serial.print("Temperature eau: ");
  Serial.print(temperature_int);
  Serial.print("exterieur: ");
  Serial.print(temp_ext);
}