#include <Wire.h>
#include <Servo.h>
#include "profile.h"
#include "communicate.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Laser.h"
#include "variable.h"
#include "uart_process.h"
#include "silo_process.h"
#include "retry_and_start.h"

void position_compute(float des_coord[], float adjust) {
  float d_input[3] = { 0, 0, 0 };
  static float error_sum = 0;
  for (int i = 0; i < 2; i++) {
    error[i] += des_coord[i] - coord_now[i];
    d_input[i] = error[i] - last_e[i];
    last_e[i] = error[i];
    output_speed[i] = p[i] * error[i] + d_input[i] * d[i];
  }
  error[2] += des_coord[2] - coord_now[2];
  error[2] += adjust;
  // error_sum += error[2];
  d_input[2] = error[2] - last_e[2];
  // d_input[2] += error[2];
  // d_input[2]=constrain(d_input[2],-8,8);
  last_e[2] = error[2];
  output_speed[2] = p[2] * error[2] + d_input[2] * d[2];
}
// speed_ = (delta*cm_per_count)/delta_time // cm/s -> delta=(speed_*dt)/cm_per_count
void calculate_speed_robot(float x, float y, float theta, float angle_robot_now) {
  // x -> cm/s y-> cm/s
  float wheel_ang[] = { 45, 315, 135, 225 };
  float sin_wheel[4];
  float cos_wheel[4];
  float delta_now[4];
  float cm_s[4];
  for (int i = 0; i < 4; i++) {
    // wheel_ang[i] += theta_pid;
    wheel_ang[i] = wheel_ang[i] * PI / 180.0;
    sin_wheel[i] = -sin(wheel_ang[i]);
    cos_wheel[i] = cos(wheel_ang[i]);
    cm_s[i] = x * sin_wheel[i] + y * cos_wheel[i] + theta * R;
    speed_desired[i] = roundFloat(delta_cvt(cm_s[i]));
    if (i < 2) speed_desired[i] = -speed_desired[i];
  }
}
void update_pid() {
  forward_x.update();
  forward_y.update();
  rotate.update();
  // Serial.println(forward_x.speed());
  float speed_x = forward_x.speed();
  float speed_y = forward_y.speed();
  float turn = rotate.speed();
  float real_speed_X = 0;
  float real_speed_Y = 0;
  float real_speed_theta = 0;
  static float theta_des_old = d_theta_pid;
  static float setpoint_now = 0;
  setpoint_now = theta_pid;

  float des_coord[3] = { forward_x.increment(), forward_y.increment(), rotate.increment() };

  coord_now[0] = dx;
  coord_now[1] = dy;
  coord_now[2] = d_theta_pid;
  position_compute(des_coord, error_angle_sensor);
  real_speed_X += speed_x;
  real_speed_Y += speed_y;
  real_speed_theta += turn;
  real_speed_X += output_speed[0];
  real_speed_Y += output_speed[1];
  real_speed_theta += output_speed[2];
  calculate_speed_robot(real_speed_X, real_speed_Y, real_speed_theta * PI / 180.0, 0);
}
void send_motor() {
  // ;
  Wire.beginTransmission(0x01);
  sendByte(speed_desired[M0]);
  sendByte(speed_desired[M1]);
  Wire.endTransmission();
  Wire.beginTransmission(0x02);
  sendByte(speed_desired[M2]);
  sendByte(speed_desired[M3]);
  Wire.endTransmission();
}
void calculate_mpu() {
  static uint8_t count_ = 0;
  count_ += 1;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float imu_data_raw = orientationData.orientation.x;
  theta_imu_now = imu_data_raw - theta_imu_offset;
  if (!offset_) {
    theta_imu_offset = theta_imu_now;
    offset_ = 1;
  }
  if (count_ >= 15) {
    count_ = 0;
    if (state_main == uart_mode) Serial.println("YAW:" + String(imu_data_raw) + ";");
    // Serial.println("real:"+String(theta_pid));
  }
}

void print_debug() {
  if (start_send == 1) {
    if (!forward_x.is_finished() || !forward_y.is_finished() || !rotate.is_finished()) {
      if (debug_speed) {
        Serial.print(dx);
        Serial.print(",");
        Serial.print(forward_x.increment());
        Serial.print(",");
        Serial.print(dy);
        Serial.print(",");
        Serial.println(forward_y.increment());
      }
    } else {
      if (debug_speed) {
        Serial.println("done");
        Serial.print(x_pos);
        Serial.print(" ");
        Serial.print(y_pos);
        Serial.print(" ");
        Serial.println(theta_pid);
      }
      start_send = 0;
      x_pos = 0;
      y_pos = 0;
      theta_pos = 0;
      // theta_pid = 0;
      Serial.println(send_ok);
      forward_x.reset();
      forward_y.reset();
      rotate.reset();
      // reset_all();
    }
  }
}
void rec_motor() {
  float wheel_ang[] = { 45, 315, 135, 225 };
  static bool offset_encod = 0;
  Wire.requestFrom(1, 8);
  //   while (Wire.available() < 8);
  pos_delta[M0] = receiveLong();
  pos_delta[M1] = receiveLong();
  //   while (Wire.available() < 8);
  Wire.requestFrom(2, 8);
  pos_delta[M2] = receiveLong();
  pos_delta[M3] = receiveLong();
  if (!offset_encod) {

    offset_encod = 1;

  } else {
    float sin_wheel[4];
    float cos_wheel[4];
    float delta_now[4];
    float cm_s[4];
    static float last_theta_imu = 0;
    dx = 0;
    dy = 0;
    d_theta = 0;
    d_theta_imu = theta_imu_now - last_theta_imu;
    if (d_theta_imu > 300) {  /// last = 0 ; now = 358 0->358
      d_theta_imu = -(360 - theta_imu_now + last_theta_imu);
    } else if (d_theta_imu < -300)  // last = 358 now =0 ; 358 -> 0 = -358
    {
      d_theta_imu = 360 - last_theta_imu + theta_imu_now;
    }
    last_theta_imu = theta_imu_now;
    // theta_imu += d_theta_imu;
    for (int i = 0; i < 4; i++) {
      if (i < 2) pos_delta[i] = -pos_delta[i];
      pos[i] += pos_delta[i];
      // wheel_ang[i] += theta_pid;
      wheel_ang[i] = wheel_ang[i] * PI / 180.0;
      sin_wheel[i] = -sin(wheel_ang[i]);
      cos_wheel[i] = cos(wheel_ang[i]);
      cm_s[i] = (pos_delta[i] * cm_per_count);
      dx += 0.5 * cm_s[i] * sin_wheel[i];
      dy += 0.5 * cm_s[i] * cos_wheel[i];
      d_theta += (0.25 / R) * cm_s[i];
      // Serial.print(pos_delta[i]);
      // Serial.print(",");
      // speed_desired[i]=delta_cvt();
      // Serial.println(speed_desired[i]);
    }
    // Serial.println();
    d_theta = d_theta * 180 / PI;
    theta_pos += d_theta;
    d_theta_pid = d_theta_imu;
    //  d_theta_pid = d_theta_imu;

    // d_theta_pid = d_theta;
    theta_pid += d_theta_pid;
    x_pos += dx;
    y_pos += dy;
    update_pid();
    send_motor();
    static uint8_t count_print = 0;
    print_debug();
  }
}
void setup() {
  // put your setup code here, to run once:
  setID();
  Wire.begin();  // join i2c bus
  Wire.setClock(400000);
  Serial.begin(57600);
  myservo.attach(servo_);

  // myservo.write(179);

  pinMode(29, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);
  pinMode(switch_up, INPUT_PULLUP);
  pinMode(switch_down, INPUT_PULLUP);
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
  setup_state();

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  bno.setExtCrystalUse(true);
  Serial.setTimeout(50);
  move_gripper(0, 0);
  gripper_on(1);
  // myservo.write(170);
  delay(1500);  // wait i2c setup
}

void loop() {
  static unsigned long time_rec = millis();
  static unsigned long time_send = millis();
  static unsigned long time_read = millis();
  static unsigned long time_stop = millis();
  static unsigned long time_123 = millis();
  switch (state_main) {
    case waiting:
      static unsigned long time_waiting = millis();
      static bool done_init=0;
      if (Serial.available() && done_init==0) {
        String c = Serial.readStringUntil(';');
        if (c.indexOf("init")!=-1) {
          gripper_on(off);
          Serial.println("get_init");
          done_init=1;
        }
      }
      // Serial.print(digitalRead(start_pin));
      // Serial.print(",");
      // Serial.println(digitalRead(retry_pin));
      static uint8_t count_ = 0;
      reset_all();
      if (millis() - time_waiting > 100) {
        time_waiting = millis();
        count_ += 1;
        if (count_ >= 10) {
          count_ = 0;
          Serial.println("waiting");
        }
        send_motor();
      }

      check_start_state();
      // delay(200);
      break;
    case uart_mode:

      uart_process();
      callFunctionPeriodically(calculate_mpu, 10, time_send);
      // Serial.println(digitalRead(switch_down));
      callFunctionPeriodically(rec_motor, LOOP_REC, time_rec);
      // if (millis() - time_123 > 500) {
      //   time_123 = millis();
      //   uint16_t value_up;
      //   uint16_t value_down;
      //   uint16_t value_ll;
      //   // Serial.println("dsa");
      //   // read_sensor_1(&value_up);
      //   read_sensor_1(&value_down);
      //   read_sensor_2(&value_up);
      //   read_sensor_3(&value_ll);
      //   // float angle_diff = calculate_angle_sensor(value_up, value_down);
      //   // angle_diff = angle_diff;
      //   // Serial.println("sensor:" + String(angle_diff));
      //   Serial.println(value_up);
      //   Serial.println(value_down);
      //   Serial.println(value_ll);
      //   // read_all_sensors();
      // }
      break;
    case silo_mode:
      static unsigned long time_gripper = millis();
      if (millis() - time_gripper > 80) {
        time_gripper = millis();
        if (digitalRead(switch_up)) move_gripper(up, 50);
        else move_gripper(up, 0);
      }
      // static unsigned long time_send_2=millis();
      // static unsigned long time_rec_2=m
      // Serial.println("silo");
      callFunctionPeriodically(silo_process, 55, time_read);
      callFunctionPeriodically(calculate_mpu, 10, time_send);
      callFunctionPeriodically(rec_motor, LOOP_REC, time_rec);
      break;
    case retry_mode:
      retry_process(calculate_mpu, rec_motor);
      break;
    case start_mode:
      // delay(5000);

      // /* End retry_process */

      // if (mode_robot == RED) {
      //   Serial.println("RED;");
      //   delay(10);
      //   Serial.print("rb:");
      //   Serial.print(x_goal);
      //   Serial.print("/");
      //   Serial.print(y_goal);
      //   Serial.println(";");
      // } else {
      //   Serial.println("BLUE;");
      //   delay(10);
      //   Serial.print("rb:");
      //   Serial.print(x_goal_2);
      //   Serial.print("/");
      //   Serial.print(y_goal);
      //   Serial.println(";");
      // }
      // Serial.println("START;");
      // Serial.println();
      // // delay(2000);
      // p[2] = 0.14;
      // d[2] = 13.5;
      // state_main = uart_mode;
      start_process(calculate_mpu, rec_motor);
      // start_process();

      break;


      // Serial.println( digitalRead(retry_pin));
  }
  check_retry_state();
}
