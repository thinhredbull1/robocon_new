// #include "HardwareSerial.h"
#define start_pin 19
#define retry_pin 18

#define MODE 29
#define speed_start 120
#define acc_start 140

void retry_on() {
  // delay(500);
  state_main = retry_mode;
}
void start_on() {
  // delay(500);
  state_main = start_mode;
}
bool check_retry_state() {
  static unsigned long time_debounce = millis();
  if (millis() - time_debounce >= 65) {
    time_debounce = millis();
    static uint8_t count_delay = 0;
    if (digitalRead(retry_pin) != button_state_retry) {

      count_delay += 1;
      if (count_delay >= 4) {
        count_delay = 0;
        button_state_retry = digitalRead(retry_pin);
        state_main = retry_mode;
        return 1;
      }
    } else count_delay = 0;
  }
  return 0;
}
bool check_start_state() {
  static unsigned long time_debounce = millis();
  if (millis() - time_debounce >= 60) {
    time_debounce = millis();
    static uint8_t count_delay = 0;
    if (digitalRead(start_pin) != button_state_start) {

      count_delay += 1;
      if (count_delay >= 4) {
        count_delay = 0;
        button_state_start = digitalRead(start_pin);
        state_main = start_mode;
        return 1;
      }
    } else count_delay = 0;
  }
  return 0;
}

// void setup_retry_state() {
//   pinMode(retry_pin, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(retry_pin), retry_on, FALLING);
// }

void setup_state() {
  pinMode(retry_pin, INPUT_PULLUP);

  pinMode(start_pin, INPUT_PULLUP);

  delay(100);
  if (digitalRead(MODE) == RED) {
    Serial.println("red");
    mode_robot = RED;

  } else {
    Serial.println("green");
    mode_robot = BLUE;
  }

  button_state_retry = digitalRead(retry_pin);
  button_state_start = digitalRead(start_pin);
  // attachInterrupt(digitalPinToInterrupt(start_pin), start_on, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(retry_pin), retry_on, CHANGE);
}


//////////////////////////////////////////Retry Process///////////////////////////////////////////////////////////
void retry_process(CallbackFunction calculate_mpu, CallbackFunction rec_motor) {
  /* Write retry_process here*/
  //Reset position
  static unsigned long time_send_1 = millis();
  static unsigned long time_rec_1 = millis();
  reset_all();
  Serial.println("retrying;");
  static unsigned long time_wait = millis();
  reset_all();
  Wire.beginTransmission(0x01);
  sendByte(speed_desired[M0]);
  sendByte(speed_desired[M1]);
  Wire.endTransmission();
  Wire.beginTransmission(0x02);
  sendByte(speed_desired[M2]);
  sendByte(speed_desired[M3]);
  Wire.endTransmission();
  bool out_loop = 0;
  uint16_t delay_time = 4000;
  uint8_t count_ = 0;
  int count_delay = delay_time / 200;
  while (count_delay > 0) {
    if (millis() - time_wait > 200) {
      time_wait = millis();
      count_delay--;

      if (count_delay > 16) Serial.println("retrying;");
      Wire.beginTransmission(0x01);
      sendByte(speed_desired[M0]);
      sendByte(speed_desired[M1]);
      Wire.endTransmission();
      Wire.beginTransmission(0x02);
      sendByte(speed_desired[M2]);
      sendByte(speed_desired[M3]);
      theta_pid = 0;
      d_theta_pid = 0;
      Wire.endTransmission();
    }
  }
  Serial.println("done_delay");
  forward_x.reset();
  forward_y.reset();
  int offset = 2.1;


  //      358cm                                                                                                       90.5cm
  // <-------------^                                                                                          <-------------(<-- ROBOT DIRECTION)
  //               |                                                                                          |
  //               |   380 cm     Red(<--ROBOT DIRECTION)                           Blue                      |   380cm
  //               |                                                                                          |
  //               |   90.5 cm                                                                     358cm      |
  //           Run <---------  STAR                                                           <----------------
  //------------------------------------------------------Step 1--------------------------------------------------------------------------------------
  forward_y.start(125.5, speed_start, 0, acc_start);
  // forward_x.start(-20,speed_start,0,acc_start);
  // if (mode_robot == RED) {
  //   Serial.println("RED;");
  //   forward_x.start(415, speed_start, 0, acc_start);
  // } else {
  //   forward_x.start(-415, speed_start, 0, acc_start);
  //   // Serial.println("BLUE;");
  // }
  while (!forward_y.is_finished() && !out_loop) {
    // forward_y.update();
    out_loop = check_retry_state();
    callFunctionPeriodically(calculate_mpu, 10, time_send_1);
    callFunctionPeriodically(rec_motor, LOOP_REC, time_rec_1);
  }
  reset_all();
  forward_y.reset();
  float real_turn = -theta_pid;
  if (theta_pid < 2.5 && theta_pid > 0) real_turn = -theta_pid - offset;
  else if (theta_pid < 0 && theta_pid > -2.5) real_turn = -theta_pid + offset;

  rotate.start(real_turn, 90, 0, 100);
  Serial.println("turn:" + String(real_turn));
  while (!rotate.is_finished() && !out_loop) {
    // forward_y.update();
    out_loop = check_retry_state();
    callFunctionPeriodically(calculate_mpu, 10, time_send_1);
    callFunctionPeriodically(rec_motor, LOOP_REC, time_rec_1);
  }
  // Serial.println("step1");
  //-------------------------------------------------------Step 2 ------------------------------------------------------------------------------------
  /* Check for mode */
  if (digitalRead(MODE) == RED) {
    Serial.println("RED;");
    forward_x.start(400, speed_start, 0, acc_start);
  } else {
    forward_x.start(-405, speed_start, 0, acc_start);
    Serial.println("BLUE;");
  }

  while (!forward_x.is_finished() && !out_loop) {
    // forward_x.update();
    out_loop = check_retry_state();
    callFunctionPeriodically(calculate_mpu, 10, time_send_1);
    callFunctionPeriodically(rec_motor, LOOP_REC, time_rec_1);
  }
  forward_x.reset();
  real_turn = -theta_pid;
  if (theta_pid < 2.5 && theta_pid > 0) real_turn = -theta_pid - offset;
  else if (theta_pid < 0 && theta_pid > -2.5) real_turn = -theta_pid + offset;
  Serial.println("turn:" + String(real_turn));
  rotate.start(real_turn, 90, 0, 100);
  while (!rotate.is_finished() && !out_loop) {
    // forward_y.update();
    out_loop = check_retry_state();
    callFunctionPeriodically(calculate_mpu, 10, time_send_1);
    callFunctionPeriodically(rec_motor, LOOP_REC, time_rec_1);
  }
  // Serial.println("step2");
  //--------------------------------------------------------Step 3 -------------------------------------------------------------------------------
  forward_y.start(375, speed_start, 0, acc_start);
  if (mode_robot == RED) {
    Serial.println("RED;");
    // forward_x.start(380, speed_start, 0, acc_start);
  } else {
    // forward_x.start(-380, speed_start, 0, acc_start);
    Serial.println("BLUE;");
  }
  while (!forward_y.is_finished() && !out_loop) {
    // forward_y.update();
    out_loop = check_retry_state();
    callFunctionPeriodically(calculate_mpu, 10, time_send_1);
    callFunctionPeriodically(rec_motor, LOOP_REC, time_rec_1);
  }
  forward_y.reset();
  real_turn = -theta_pid;
  if (theta_pid < 2.5 && theta_pid > 0) real_turn = -theta_pid - offset;
  else if (theta_pid < 0 && theta_pid > -2.5) real_turn = -theta_pid + offset;
  Serial.println("turn:" + String(real_turn));
  rotate.start(real_turn, 90, 0, 100);
  while (!rotate.is_finished() && !out_loop) {
    // forward_y.update();
    out_loop = check_retry_state();
    callFunctionPeriodically(calculate_mpu, 10, time_send_1);
    callFunctionPeriodically(rec_motor, LOOP_REC, time_rec_1);
  }
  // Serial.println("step3");
  if (out_loop == 1) {
    state_main = waiting;
    button_state_retry = digitalRead(retry_pin);
  }
  if (state_main == retry_mode) {

    /* End retry_process */
    Serial.print("rb:");
    if (mode_robot == RED) {
      Serial.print(x_goal);
      Serial.print("/");
      Serial.print(y_goal);
      Serial.println(";");
    } else {
      Serial.print(x_goal_2);
      Serial.print("/");
      Serial.print(y_goal);
      Serial.println(";");
    }
    Serial.println("RETRY;");
    start_send = 0;
    reset_all();
    // delay(2000);
    // Serial.println("RETRY;");
    // delay(100);
    // Serial.println("RETRY;");
    state_main = uart_mode;  // Give control to PY
    // p[2] = 0.14;
    // d[2] = 13.5;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////(  START PROCESS   )///////////////////////////////////////////////////////////////

void start_process(CallbackFunction calculate_mpu, CallbackFunction rec_motor) {
  Serial.println("Starting mode");
  static unsigned long time_send_1 = millis();
  static unsigned long time_rec_1 = millis();
  static unsigned long time_wait = millis();
  int offset = 2.1;
  theta_pid = 0;
  int real_turn = -theta_pid;
  //Reset position
  reset_all();
  theta_pid = 0;
  Wire.beginTransmission(0x01);
  sendByte(speed_desired[M0]);
  sendByte(speed_desired[M1]);
  Wire.endTransmission();
  Wire.beginTransmission(0x02);
  sendByte(speed_desired[M2]);
  sendByte(speed_desired[M3]);
  Wire.endTransmission();
  uint16_t delay_time = 1000;
  uint8_t count_ = 0;
  int count_delay = delay_time / 200;

  while (count_delay > 0) {
    if (millis() - time_wait > 200) {
      theta_pid = 0;
      d_theta_pid = 0;
      time_wait = millis();
      count_delay--;
      // Serial.println("retrying;");
      Wire.beginTransmission(0x01);
      sendByte(speed_desired[M0]);
      sendByte(speed_desired[M1]);
      Wire.endTransmission();
      Wire.beginTransmission(0x02);
      sendByte(speed_desired[M2]);
      sendByte(speed_desired[M3]);
      Wire.endTransmission();
    }
  }
  forward_x.reset();
  forward_y.reset();
  // move_gripper(down, 60);
  // while (digitalRead(switch_down))
  //   ;
  bool out_loop = 0;
  move_gripper(down, 0);
  gripper_on(off);
  button_state_start = digitalRead(start_pin);


  //      358cm                                                                                                      610 cm
  // <-------------^                                                                                          <------------- START
  //               |                                                                                          |
  //               |   380 cm     Red(<--ROBOT DIRECTION)                           Blue                      |   380cm
  //               |                                                                                          |                  (<-- ROBOT DIRECTION)
  //               |    610cm                                                                      358cm      |
  //           Run <------------------- STAR                                          <-----------------------

  //---------------------------------------------------------Step 1--------------------------------------------------------------------------------
  forward_y.start(670, speed_start, 0, acc_start);

  forward_x.start(-25.5, speed_start, 0, acc_start);
  // Serial.println("BLUE;");

  bool move_now = 0;
  while (!forward_y.is_finished() && !out_loop) {

    // forward_y.update();
    out_loop = check_start_state();
    callFunctionPeriodically(calculate_mpu, 10, time_send_1);
    callFunctionPeriodically(rec_motor, LOOP_REC, time_rec_1);
    if (forward_y.position() >= 350 && !move_now) {
      move_gripper(up, 80);
      if (!digitalRead(switch_up)) {
        move_now = 1;
        move_gripper(up, 0);
        gripper_on(on);
      }
    }
  }
  reset_all();
  // Serial.println(theta_pid);
  real_turn = -theta_pid;
  if (theta_pid < 2.5 && theta_pid > 0) real_turn = -theta_pid - offset;
  else if (theta_pid < 0 && theta_pid > -2.5) real_turn = -theta_pid + offset;
  Serial.println("turn:" + String(real_turn));
  reset_pid();
  rotate.start(real_turn, 90, 0, 100);
  while (!rotate.is_finished() && !out_loop) {
    // forward_y.update();
    out_loop = check_start_state();
    callFunctionPeriodically(calculate_mpu, 10, time_send_1);
    callFunctionPeriodically(rec_motor, LOOP_REC, time_rec_1);
  }
  reset_pid();
  //---------------------------------------------------------Step 2 (Step 2 need to check for mode) --------------------------------------------------------
  /* Check for mode */
  if (digitalRead(MODE) == RED) {
    Serial.println("RED;");
    forward_x.start(405, speed_start, 0, acc_start);

  } else {
    forward_x.start(-400, speed_start, 0, acc_start);
    Serial.println("BLUE;");
  }
  forward_y.start(10, speed_start, 0, acc_start);
  // forward_y.start(30, speed_start, 0, acc_start);

  while (!forward_x.is_finished() && !out_loop) {
    // forward_x.update();
    out_loop = check_start_state();
    callFunctionPeriodically(calculate_mpu, 10, time_send_1);
    callFunctionPeriodically(rec_motor, LOOP_REC, time_rec_1);
  }
  reset_all();
  forward_x.reset();
  reset_pid();
  real_turn = -theta_pid;
  if (theta_pid < 2.5 && theta_pid > 0) real_turn = -theta_pid - offset;
  else if (theta_pid < 0 && theta_pid > -2.5) real_turn = -theta_pid + offset;
  // rotate.start(real_turn, 90, 0, 100);
  Serial.println("turn:" + String(real_turn));
  if (mode_robot == RED) {
    Serial.println("RED;");
    // forward_x.start(380, speed_start, 0, acc_start);
  } else {
    // forward_x.start(-380, speed_start, 0, acc_start);
    Serial.println("BLUE;");
  }
  rotate.start(real_turn, 90, 0, 100);
  //  Serial.println(theta_pid);
  while (!rotate.is_finished() && !out_loop) {
    // forward_y.update();
    out_loop = check_start_state();
    callFunctionPeriodically(calculate_mpu, 10, time_send_1);
    callFunctionPeriodically(rec_motor, LOOP_REC, time_rec_1);
  }
  reset_pid();
  //-----------------------------------------------------------Step 3 -----------------------------------------------------------------------------------------------------------
  forward_y.start(380, speed_start, 0, acc_start);

  while (!forward_y.is_finished() && !out_loop) {
    //forward_y.update();
    out_loop = check_start_state();
    callFunctionPeriodically(calculate_mpu, 10, time_send_1);
    callFunctionPeriodically(rec_motor, LOOP_REC, time_rec_1);
  }
  forward_y.reset();
  reset_pid();
  real_turn = -theta_pid;
  if (theta_pid < 2.5 && theta_pid > 0) real_turn = -theta_pid - offset;
  else if (theta_pid < 0 && theta_pid > -2.5) real_turn = -theta_pid + offset;
  // rotate.start(real_turn, 90, 0, 100);
  Serial.println("turn:" + String(real_turn));
  rotate.start(real_turn, 90, 0, 100);
  //  Serial.println(theta_pid);
  while (!rotate.is_finished() && !out_loop) {
    // forward_y.update();
    out_loop = check_start_state();
    callFunctionPeriodically(calculate_mpu, 10, time_send_1);
    callFunctionPeriodically(rec_motor, LOOP_REC, time_rec_1);
  }
  reset_pid();
  if (out_loop) {
    state_main = waiting;
    button_state_start = digitalRead(start_pin);
  }
  if (state_main == start_mode) {
    /* End retry_process */
    Serial.print("rb:");
    if (mode_robot == RED) {
      Serial.print(x_goal);
      Serial.print("/");
      Serial.print(y_goal);
      Serial.println(";");
    } else {
      Serial.print(x_goal_2);
      Serial.print("/");
      Serial.print(y_goal);
      Serial.println(";");
    }
    Serial.println("START;");
    reset_all();
    state_main = uart_mode;  // Give control to PY
    // p[2] = 0.14;
    // d[2] = 13.5;
  }
}