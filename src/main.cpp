#include <Arduino.h>

#define TMP_SENSOR_PIN (A0)
#define RELAY_HEAT_PIN (7)
#define RELAY_VENT_PIN (4)

#define TEMP_DATA_BUFF_SIZE (10)

// #define START_HEAT_TEMP 35
#define STOP_HEAT_TEMP (36.0)
#define START_VENT_TEMP (38.0)

#define SSR_RELAY_MAX_TIME_MS 10

uint16_t temp_data_buff[TEMP_DATA_BUFF_SIZE] = {0};
uint8_t temp_buff_id = 0;

void controller_heat_if(float temp);
void controller_heat_pid(float temp);
uint16_t filter_raw_temp(uint16_t raw_temp);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(TMP_SENSOR_PIN, INPUT);
  pinMode(RELAY_HEAT_PIN, OUTPUT);
  pinMode(RELAY_VENT_PIN, OUTPUT);

  digitalWrite(RELAY_HEAT_PIN, LOW);
  digitalWrite(RELAY_VENT_PIN, LOW);

  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  // delay(1000);
  uint16_t val_temp = analogRead(TMP_SENSOR_PIN);
  uint16_t val_temp_filtered = filter_raw_temp(val_temp);
  float temp = (((val_temp_filtered)*(5.0/1024.0)) - 0.5)*100;

  Serial.print("Temperature: ");
  Serial.println(temp);

  delay(10);
}

void controller_heat_if(float temp){
  if(temp <= STOP_HEAT_TEMP){
    digitalWrite(RELAY_HEAT_PIN, HIGH);
  }
  else {
    digitalWrite(RELAY_HEAT_PIN, LOW);
  }
  if(temp >= START_VENT_TEMP){
    digitalWrite(RELAY_HEAT_PIN, HIGH);
  }
  else {
    digitalWrite(RELAY_HEAT_PIN, LOW);
  }

}

// void controller_heat_pid(float temp){
//   if(temp <= STOP_HEAT_TEMP){
//     digitalWrite(RELAY_HEAT_PIN, HIGH);
//   }
//   else {
//     digitalWrite(RELAY_HEAT_PIN, LOW);
//   }
//   if(temp >= START_VENT_TEMP){
//     digitalWrite(RELAY_HEAT_PIN, HIGH);
//   }
//   else {
//     digitalWrite(RELAY_HEAT_PIN, LOW);
//   }

// }

uint16_t filter_raw_temp(uint16_t raw_temp){
  temp_data_buff[temp_buff_id] = raw_temp;
  uint32_t sum = 0;
  for (uint8_t i = 0; i< TEMP_DATA_BUFF_SIZE; i++){
    sum += temp_data_buff[i];
  }

  Serial.println(sum);

  temp_buff_id++;
  if(temp_buff_id>=TEMP_DATA_BUFF_SIZE){
    temp_buff_id=0;
  }
  
  return sum/TEMP_DATA_BUFF_SIZE;
}