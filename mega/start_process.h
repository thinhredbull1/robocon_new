void calculate_error_angle(float error) {
  float pTerm = p_sensor * error;
  float dTerm = d_sensor * (error - last_steering_error);
  float adjustment = (pTerm + dTerm);
  adjustment = constrain(adjustment, -STEERING_ADJUST_LIMIT, STEERING_ADJUST_LIMIT);
  last_steering_error = error;
  error_angle_sensor = adjustment;
}
void start_process() {
  static unsigned long time_process = millis();
  static uint16_t value_now_up;
  static uint16_t value_now_down;
  if (!forward_y.is_finished()) {
    if (millis() - time_process >= 50) {
      time_process = millis();

      // Serial.println("dsa");
      read_sensor_1(&value_now_up);
      value_now_up += offset_left_up;
      read_sensor_2(&value_now_down);
      // Serial.println("run");
      // float angle_diff = calculate_angle_sensor(value_up, value_down);
      // float error = calculate_angle_sensor(value_now_up, value_now_down);
      // calculate_error_angle(error);
    }
  }
  else{
    reset_all();
  }
}