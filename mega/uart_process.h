void uart_process() {
  static bool start_move = 0;
  static bool grip_send = 0;
  static char dir_now;
  static char grip_now;
  static int speed_down;
  static int speed_max = 160;
  static uint8_t speed_min = 40;
  static unsigned long time_speed_des = millis();
  static uint16_t time_delay_down = 350;
  static int count_total = time_delay_down / 50;
  static int acc = (speed_max - speed_min) / (count_total);
  static int count_time_delay = 0;
  if (Serial.available()) {
    String c = Serial.readStringUntil(';');
    int index_test = c.indexOf(",");
    int index_kp_desired = c.indexOf(":");
    int grip = c.indexOf("f");
    int move_ = c.indexOf("m");
    int silo_ = c.indexOf("silo");
    if (silo_ != -1) {
      state_main = silo_mode;
      state_silo = 0;
      silo_offset_index = c.substring(0, silo_).toInt();
      if (!init_sensor) {
        digitalWrite(LEFT_X, HIGH);
        digitalWrite(LEFT_Y, HIGH);
        digitalWrite(RIGHT_X, HIGH);
        digitalWrite(SHT_LOX4, HIGH);
        delay(10);

        // activating LOX1 and resetting LOX2,LOX3,LOX4
        digitalWrite(LEFT_X, HIGH);
        digitalWrite(LEFT_Y, LOW);
        digitalWrite(RIGHT_X, LOW);
        digitalWrite(SHT_LOX4, LOW);
        // initing LOX1
        if (!lox1.begin(LOX1_ADDRESS)) {
          Serial.println(F("Failed to boot first VL53L0X"));
        }
        delay(10);

        // activating LOX2
        digitalWrite(LEFT_Y, HIGH);
        delay(10);

        //initing LOX2
        if (!lox2.begin(LOX2_ADDRESS)) {
          Serial.println(F("Failed to boot second VL53L0X"));
        }

        // activating LOX3
        digitalWrite(RIGHT_X, HIGH);
        delay(10);

        // initing LOX2
        if (!lox3.begin(LOX3_ADDRESS)) {
          Serial.println(F("Failed to boot third VL53L0X"));
        }
        init_sensor=1;
      }
      reset_all();
      forward_y.start(40,speed_silo,speed_silo-10,acc_silo);
   
    }
    if (move_ != -1) {
      start_move = 1;
      dir_now = c.substring(0, move_)[0];

      count_time_delay = count_total;
      speed_down = speed_max;
    }
    if (grip != -1) {
      grip_now = c.substring(0, grip)[0];
      grip_send = 1;
    }  if (index_test != -1 && start_send == 0) {
      int index_3 = c.indexOf("~");
      int index_4 = c.indexOf("&");
      int index_2 = c.indexOf("#");
      float x_start = c.substring(0, index_test).toFloat();
      float y_start = c.substring(index_test + 1, index_3).toFloat();
      float rotate_start = c.substring(index_3 + 1, index_2).toFloat();
      float a_max = c.substring(index_2 + 1, index_4).toFloat();
      float v_max = c.substring(index_4 + 1).toFloat();
      forward_x.start(x_start, v_max, 0, a_max);
      forward_y.start(y_start, v_max, 0, a_max);
      rotate.start(rotate_start, 140, 0, 200);
      angle_des = rotate_start;
      start_send = 1;
    }
    // else if (index_kp_desired != -1) {
    //   int index_cal = c.indexOf("#");
    //   int index_cal_cal = c.indexOf("*");
    //   if (index_cal != -1) {
    //     p[0] = c.substring(0, index_kp_desired).toFloat();
    //     d[0] = c.substring(index_kp_desired + 1, index_cal).toFloat();
    //     p[2] = c.substring(index_cal + 1, index_cal_cal).toFloat();
    //     d[2] = c.substring(index_cal_cal + 1).toFloat();
    //     Serial.print(p[0]);
    //     Serial.print(" ");
    //     Serial.print(d[0]);
    //     Serial.print(" ");
    //     Serial.print(p[2]);
    //     Serial.print(" ");
    //     Serial.println(d[2]);
    //     reset_all();
    //   }
    // }
  }
  if (grip_send) {
    if (grip_now == 'o') gripper_on(on);
    else gripper_on(off);
    grip_send = 0;
    Serial.println("SV;");
  }
  if (start_move) {
    // static bool stop_ok =0;
    if (dir_now == 'u') {  // up
      move_gripper(up, 85);
      if (!digitalRead(switch_up)) {
        // move_gripper(down, 50);
        Serial.println("MU;");
        // delay(10);
        move_gripper(0, 0);
        start_move = 0;
      }
    } else {

      if (millis() - time_speed_des >= 50) {
        time_speed_des = millis();
        if (count_time_delay <= 0) {
          if (speed_down > speed_min) {
            speed_down = speed_down - acc;
            if (speed_down <= speed_min) speed_down = speed_min;
          }
        } else count_time_delay--;
        move_gripper(down, speed_down);
      }
      if (!digitalRead(switch_down)) {
        // move_gripper(up, 70);
        Serial.println("MD;");
        // delay(10);
        move_gripper(down, 0);

        start_move = 0;
      }
    }
  }
}