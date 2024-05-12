#ifndef CONFIG_H
#define CONFIG_H
#include<Arduino.h>
#define RED 1
#define BLUE 0
#define uart_state 0
#define on 1
#define off 0
#define up 0
#define down 1
#define M0 0
#define M1 1
#define M2 2
#define M3 3
#define R 21.98 // do duoc < thuc te --> giam
#define ENCODER_TOTAL 1020.5 /// robot move > thuc te -> giam  //1004.5
#define WHEEL_DIAMETER 12 // 
#define min_cm_s 20
#define min_cm_y 25
#define switch_up 46
#define switch_down 48
#define a3 340.5
#define a3_2 41.5
#define offset_left_up -10.8
#define offset_left_down -10.8
#define offset_y 3.5
#define offset_angle 3.75
#define speed_silo 100
#define acc_silo 120
enum state_flow : uint8_t {
  uart_mode = 0,
  silo_mode = 1,
  start_mode = 2,
  retry_mode = 3,
  waiting=4,
};
const float STEERING_ADJUST_LIMIT=30;
uint16_t time_out=1000;
uint16_t count_stop_time_out=time_out/55;
float distance_to_silo=304.5;
float distance_to_next_silo=750.5; // 750 -201.5
float range_update=50;
String send_ok="AD:MOV;";
const float cm_per_count = (PI * WHEEL_DIAMETER) / ENCODER_TOTAL;  // cm/count banh
const float LOOP_CONTROL=0.01;
#define delta_cvt(speed) (speed * LOOP_CONTROL) / cm_per_count             // cm/s -> delta send
bool debug_speed=0;
const float LOOP_REC = 20.0;
// const float LOOP_SEND = 40.0;
const float LOOP_INTERVAL=(LOOP_REC/1000.0);
const float LOOP_CVT=LOOP_INTERVAL/LOOP_CONTROL;
// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
const int pwm=5; //{10,11}
const int dir=6;
const int servo_=9;

typedef void (*CallbackFunction)();
void callFunctionPeriodically(CallbackFunction functionToCall, unsigned long intervalTime, unsigned long &previousMillis) {
  unsigned long currentMillis = millis();  //
  if (currentMillis - previousMillis >= intervalTime) {
    functionToCall();
    previousMillis = currentMillis;
  }
}

float roundFloat(float number) {
  if (number > 0) {
    return floor(number + 0.5);
  } else {
    return ceil(number - 0.5);
  }
}
void sendByte(int value) {
  for (int k = 0; k < 2; k++) {
    byte out = (value >> 8 * (1 - k)) & 0xFF;
    Wire.write(out);
  }
}
long receiveLong() {
  long outValue;
  for (int k = 0; k < 4; k++) {
    byte nextByte = Wire.read();
    outValue = (outValue << 8) | nextByte;
  }
  return outValue;
}
#endif
