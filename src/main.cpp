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

#define SSR_RELAY_MAX_TIME_MS 10//good val
// #define SSR_RELAY_MAX_TIME_MS 500

#define KP (1)
#define KI (1)
#define KD (1)

#define RAW_TEMP_TO_CELSIUS(X) ( ( ( ( ((float)X)*(5.0/1024.0) ) - 0.5) * 100.0) )

uint16_t temp_data_buff[TEMP_DATA_BUFF_SIZE] = {0};
uint8_t temp_buff_id = 0;
bool temp_ready = false;

float last_error_pid = 0.0;
uint32_t last_pid_time = 0;
float integral_pid = 0.0;

uint16_t ms_interrupt_count = 0;

uint8_t pwm_period = 10;
uint8_t pwm_low_time_vent = 11;
uint8_t pwm_low_time_heat = 11;

void setup_timer();
void controller_heat_if(float temp);
void controller_heat_pid(float temp);

bool process_temp_sensor();
float get_temp_sensor_average();

float get_filtered_temp();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  setup_timer();

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

ISR(TIMER2_OVF_vect)// at each millisecond
{
  TCNT2 = 5; // Timer Preloading
  ms_interrupt_count++;

  if(ms_interrupt_count==pwm_low_time_heat){
    digitalWrite(RELAY_HEAT_PIN, HIGH);
  }
  if(ms_interrupt_count==pwm_low_time_vent){
    digitalWrite(RELAY_VENT_PIN, HIGH);
  }

  // reset count after period
  if(ms_interrupt_count >= pwm_period){
    ms_interrupt_count = 0;
    if (pwm_low_time_heat == 0){
      digitalWrite(RELAY_HEAT_PIN, HIGH);
      digitalWrite(RELAY_VENT_PIN, LOW);  
    }
    else if((pwm_low_time_vent == 0)){
      digitalWrite(RELAY_HEAT_PIN, LOW);
      digitalWrite(RELAY_VENT_PIN, HIGH);
    }
    else{
      digitalWrite(RELAY_HEAT_PIN, LOW);
      digitalWrite(RELAY_VENT_PIN, LOW);
    }

    // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));//debug
  }
}

void setup_timer(){
  // config generated from https://deepbluembedded.com/arduino-timer-calculator-code-generator/
  cli();

  //timer 2 est 8 bit
  TCCR2A = 0;           // Init Timer1
  TCCR2B = 0;           // Init Timer1
  TCCR2B |= B00000100;  // Prescalar = 64 donc incremente chaque 4 micro
  TCNT2 = 5;        // Timer Preloading to only count to 250 instead of 255 for approx 1 milli
  TIMSK2 |= B00000001;  // Enable Timer Overflow Interrupt

  sei();
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
    // uint8_t scaled_value = (uint8_t) pid_res*255;
    float inverse = 1.0-pid_res;
    if (pid_res == 1.0){
      pwm_low_time_heat = 0;
      pwm_period = SSR_RELAY_MAX_TIME_MS;
      pwm_low_time_vent = pwm_period+1;
    }
    else if (inverse<0.5){
      pwm_low_time_heat = SSR_RELAY_MAX_TIME_MS;
      pwm_period = SSR_RELAY_MAX_TIME_MS/inverse;
      pwm_low_time_vent = pwm_period+1;
    }
    else {
      pwm_period = SSR_RELAY_MAX_TIME_MS/pid_res;
      pwm_low_time_vent = pwm_period+1;
      pwm_low_time_heat = pwm_period-SSR_RELAY_MAX_TIME_MS;
    }

    // analogWrite(RELAY_HEAT_PIN, scaled_value);
    // digitalWrite(RELAY_VENT_PIN, LOW);
  }
  else {
    pid_res = pid_res*-1.0; //set positive value
    pid_res = pid_res > 1.0 ? 1.0 : pid_res;// set a 1 si depasse 1

    float inverse = 1.0-pid_res;
    if (pid_res == 1.0){
      pwm_low_time_vent = 0;
      pwm_period = SSR_RELAY_MAX_TIME_MS;
      pwm_low_time_heat = pwm_period+1;
    }
    else if (inverse<0.5){
      pwm_low_time_vent = SSR_RELAY_MAX_TIME_MS;
      pwm_period = SSR_RELAY_MAX_TIME_MS/inverse;
      pwm_low_time_heat = pwm_period+1;
    }
    else {
      pwm_period = SSR_RELAY_MAX_TIME_MS/pid_res;
      pwm_low_time_heat = pwm_period+1;
      pwm_low_time_vent = pwm_period-SSR_RELAY_MAX_TIME_MS;
    }
    //basic pwm between 0 and 255
    // uint8_t scaled_value = (uint8_t) pid_res*255;
    // analogWrite(RELAY_VENT_PIN, scaled_value);
    // digitalWrite(RELAY_HEAT_PIN, LOW);
  }

  Serial.print(">pid_val:");
  Serial.println(pid_res);
  Serial.print(">time:");
  Serial.println(millis());
  Serial.print(">err:");
  Serial.println(err);
  Serial.print(">temp:");
  Serial.println(temp);
  Serial.print(">temp:");
  Serial.println(temp);
  Serial.print(">pwm_period:");
  Serial.println(pwm_period);


  last_error_pid = err;
  last_pid_time = current_time;

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