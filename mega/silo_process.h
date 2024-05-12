enum silo_follow : uint8_t {
  dap_tuong = 0,
  wait_laser_to_wall = 1,
  go_to_silo = 2,
  wait_to_silo = 3,
  delay_robot = 4,
  rotate_align = 5,
  wait_rotate = 6,
  delay_robot_y = 7,
  go_to_wall_now = 8,
  wait_to_wall = 9,
  delay_alig = 10,


};
void silo_process() {
  static uint16_t count_ = 0;
  static uint16_t value_sensor_sum_u = 0;
  static uint16_t value_sensor_sum_d = 0;
  static uint8_t count_2 = 0;
  uint16_t value_left_up;
  uint16_t value_left_down;
  count_ += 1;
  switch (state_silo) {
    // case align_first:
    // // float offset=1.8;
    // //   float real_turn = -theta_pid;
    // //   Serial.println("alig");
    // //   if (theta_pid < 2.5 && theta_pid > 0) real_turn = -theta_pid - offset;
    // //   else if (theta_pid < 0 && theta_pid > -2.5) real_turn = -theta_pid + offset;
    // //   rotate.start(real_turn, 90, 0, 100);
    // //   state_silo = delay_alig;
    //   count_ = 0;
    //   break;
    case delay_alig:
      Serial.println("delay");
      if (rotate.is_finished()) {
        count_ = 0;
        reset_all();
        move_gripper(up, 85);
        Serial.println("dede");
        while (digitalRead(switch_up))
          ;
        // move_gripper(down, 50);
        // Serial.println("MO;");
        // delay(10);
        //  Serial.println("delay_alig");
        move_gripper(0, 0);
        // start_move = 0;

        state_silo = dap_tuong;
        break;
      }

    case dap_tuong:
      static uint8_t count_3 = 0;

      // forward_x.start(-20, 60, 0, 60);
      read_sensor_y(&value_left_up);
      // Serial.println(value_left_up);
      value_left_up += offset_left_up;
      if (value_left_up <= 320) {
        Serial.println("see wall");
        count_ = 0;
        state_silo = delay_robot_y;
        reset_all();
      }
      break;
    case wait_laser_to_wall:
      // Serial.println("HERE");
      // lox1.rangingTest(&measure1, false); // 1 la y 2 la x
      if (mode_robot == RED) read_sensor_left_x(&value_left_down);
      else read_sensor_right_x(&value_left_down);
      value_left_down += offset_left_down;
      // Serial.println(value_left_down);
      if (value_left_down <= 290) {
        count_2 += 1;
        if (count_2 >= 2) {
          count_2 = 0;
          state_silo = delay_robot;
          reset_all();
          count_ = 0;
          // send_motor();
        }
      } else {
        count_2 = 0;
      }
      count_ = 0;
      break;
    case go_to_silo:
      if (mode_robot == RED) read_sensor_left_x(&value_left_up);
      else {
        read_sensor_right_x(&value_left_up);
        value_left_up -= 4.4;
      }
      //  Serial.println(value_left_up);
      value_left_up += offset_left_down;
      // read_sensor_2(&value_left_down);
      value_sensor_sum_u += value_left_up;
      // Serial.println(value_left_up);
      // value_sensor_sum_d += value_left_down;
      if (count_ >= 3) {
        float sum_up = (float)(value_sensor_sum_u / count_);
        // float sum_down = (float)(value_sensor_sum_d / count_);
        value_sensor_sum_u = 0;
        value_sensor_sum_d = 0;
        // float total_dis = (sum_up + sum_down) / 2.0;
        if (mode_robot != RED) distance_to_next_silo = 714;
        else distance_to_next_silo = 750.5;
        if(mode_robot==RED)distance_to_silo=315.5;
        else distance_to_silo=304.5;
        float x_move_to_silo = (distance_to_silo + silo_offset_index * distance_to_next_silo) - sum_up;

        x_move_to_silo = x_move_to_silo / 10.0;
        // Serial.println(x_move_to_silo);
        if (mode_robot == RED) forward_x.start(x_move_to_silo, speed_silo, 0, acc_silo);
        else forward_x.start(-x_move_to_silo, speed_silo, 0, acc_silo);
         Serial.println("wall_x:"+String(x_move_to_silo));
        state_silo = wait_to_silo;
        count_ = 0;
        // silo_now = 0;
      }
      break;
    case wait_to_silo:
      if (forward_x.is_finished()) {
        state_silo = 0;
        // Serial.println("DONEALL");
        silo_now = 0;
        state_main = uart_mode;
        reset_all();
        myservo.write(5);
        Serial.println("GET2;");
        digitalWrite(LEFT_X, LOW);
        digitalWrite(LEFT_Y, LOW);
        digitalWrite(RIGHT_X, LOW);
        init_sensor = 0;
        count_ = 0;
      }
      break;
    case delay_robot:
      if (count_ >= 3) {
        state_silo = go_to_silo;
        count_ = 0;
      }
      break;
    case rotate_align:
      read_sensor_left_x(&value_left_up);
      value_left_up += offset_left_up;
      read_sensor_y(&value_left_down);
      value_sensor_sum_u += value_left_up;
      value_sensor_sum_d += value_left_down;
      if (count_ >= 4) {
        float sum_up = (float)(value_sensor_sum_u / count_);
        float sum_down = (float)(value_sensor_sum_d / count_);
        value_sensor_sum_u = 0;
        value_sensor_sum_d = 0;
        float angle_diff = calculate_angle_sensor(sum_up, sum_down);
        // angle_diff = angle_diff * 57.29;silo
        Serial.println("anf" + String(angle_diff));
        rotate.start(angle_diff + offset_angle, 80, 0, 80);
        count_ = 0;
        state_silo = wait_rotate;
        // state_main = uart_mode;
        // silo_now = 0;
      }


      break;
    case wait_rotate:
      if (rotate.is_finished()) {

        // Serial.println("DONEALL");
        // count_ +=1;
        if (count_ >= 4) {
          count_ = 0;
          state_silo = go_to_silo;
        }
      } else count_ = 0;
      break;
    case delay_robot_y:
      if (count_ >= 3) {
        state_silo = go_to_wall_now;
        count_ = 0;
      }
      break;
    case go_to_wall_now:
      read_sensor_y(&value_left_down);
      value_left_down += offset_left_up;
      // read_sensor_2(&value_left_down);
      value_sensor_sum_u += value_left_down;

      if (count_ >= 2) {
        float sum_now = (float)(value_sensor_sum_u / count_);
        // float sum_down = (float)(value_sensor_sum_d / count_);
        value_sensor_sum_u = 0;
        value_sensor_sum_d = 0;
        sum_now = sum_now / 10.0;
        // Serial.println(sum_now);
        forward_y.start(sum_now - offset_y, 80, 0, 80);
        Serial.println("sattuong:" + String(sum_now - offset_y));
        state_silo = wait_to_wall;
        count_ = 0;
        // silo_now = 0;
      }
      break;
    case wait_to_wall:
      if (forward_y.is_finished()) {

        if (count_ >= 3) {
          count_ = 0;
          reset_all();
          if (mode_robot == RED) forward_x.start(-40, speed_silo, speed_silo - 20, acc_silo);
          else forward_x.start(40, speed_silo, speed_silo - 20, acc_silo);
          state_silo = wait_laser_to_wall;
          Serial.println("go_to_wall_x");
        }

      } else {
        count_ = 0;
      }
      break;
  }
}