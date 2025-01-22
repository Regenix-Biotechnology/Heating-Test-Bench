#include <Arduino.h>

// doit spliiter le 5v du arduino pour sensor temp et relay board

#define TMP_SENSOR_PIN (A0)
#define RELAY_HEAT_PIN (2)
#define RELAY_VENT_PIN (4)

#define TEMP_DATA_BUFF_SIZE (10)

// #define START_HEAT_TEMP 35
#define STOP_HEAT_TEMP (36.0)
#define START_VENT_TEMP (38.0)
#define GOAL_TEMP (37.0)

#define SSR_RELAY_MAX_TIME_MS 10

#define KP (1)
#define KI (1)
#define KD (1)

#define RAW_TEMP_TO_CELSIUS(X) ( ( ( ( ((float)X)*(5.0/1024.0) ) - 0.5) * 100.0) )

uint16_t temp_data_buff[TEMP_DATA_BUFF_SIZE] = {0};
uint8_t temp_buff_id = 0;
bool temp_ready = false;

float last_temperature_pid = 0.0;
float last_error_pid = 0.0;
uint32_t last_pid_time = 0;
float integral_pid = 0.0;

void controller_heat_if(float temp);
void controller_heat_pid(float temp);

bool process_temp_sensor();
float get_temp_sensor_average();

float get_filtered_temp();

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
  
  // float temp = get_filtered_temp();

  // Serial.print("Temperature: ");
  // Serial.println(temp);

  // controller_heat_if(temp);

  // delay(1000);

  if(true == process_temp_sensor()){
    float temp = get_temp_sensor_average();
    Serial.println(temp);
    controller_heat_pid(temp);
  }
}

void controller_heat_if(float temp){
  if(temp <= STOP_HEAT_TEMP){
    digitalWrite(RELAY_HEAT_PIN, HIGH);
    // Serial.println("sup");
  }
  else {
    digitalWrite(RELAY_HEAT_PIN, LOW);
    Serial.println("bad heat");
  }
  if(temp >= START_VENT_TEMP){
    digitalWrite(RELAY_VENT_PIN, HIGH);
    Serial.println("bad vent");
  }
  else {
    digitalWrite(RELAY_VENT_PIN, LOW);
  }

}

void controller_heat_pid(float temp){

  float err = GOAL_TEMP-temp;

  uint32_t current_time = millis();
  uint32_t time_delta = current_time-last_pid_time;

  integral_pid += (err*time_delta);
  float derivative = (err-last_error_pid)/time_delta;

  float pid_res = KP*err + KI*integral_pid + KD*derivative;//essaie de garder entre -1 et 1

  if(pid_res > 0){//need to heat
    pid_res = pid_res > 1.0 ? 1.0 : pid_res;// set a 1 si depasse 1
    //basic pwm between 0 and 255
    uint8_t scaled_value = (uint8_t) pid_res*255;
    analogWrite(RELAY_HEAT_PIN, scaled_value);
    digitalWrite(RELAY_VENT_PIN, LOW);
  }
  else {
    pid_res = pid_res*-1; //set positive value
    pid_res = pid_res > 1.0 ? 1.0 : pid_res;// set a 1 si depasse 1
    //basic pwm between 0 and 255
    uint8_t scaled_value = (uint8_t) pid_res*255;
    analogWrite(RELAY_VENT_PIN, scaled_value);
    digitalWrite(RELAY_HEAT_PIN, LOW);
  }

}

bool process_temp_sensor(){
  uint16_t temp = analogRead(TMP_SENSOR_PIN);
  temp_data_buff[temp_buff_id] = temp;
  temp_buff_id++;
  if(temp_buff_id>=TEMP_DATA_BUFF_SIZE){
    temp_buff_id=0;
    temp_ready = true;
  }

  return temp_ready;
}

float get_temp_sensor_average(){
  if (temp_ready){
    uint32_t sum = 0;
    for (uint8_t i = 0; i< TEMP_DATA_BUFF_SIZE; i++){
      sum += temp_data_buff[i];
    }

    memset(temp_data_buff, 0, sizeof(temp_data_buff));
    temp_buff_id = 0;
    temp_ready = false;
    return RAW_TEMP_TO_CELSIUS(sum/TEMP_DATA_BUFF_SIZE);
  }

  return 0.0;
}

float get_filtered_temp(){
  //to make it faster just make sum global and substract last data and add new one each time
  uint16_t raw_temp = analogRead(TMP_SENSOR_PIN);
  temp_data_buff[temp_buff_id] = raw_temp;
  uint32_t sum = 0;
  for (uint8_t i = 0; i< TEMP_DATA_BUFF_SIZE; i++){
    sum += temp_data_buff[i];
  }

  temp_buff_id++;
  if(temp_buff_id>=TEMP_DATA_BUFF_SIZE){
    temp_buff_id=0;
  }
  
  return RAW_TEMP_TO_CELSIUS(sum/TEMP_DATA_BUFF_SIZE);
}