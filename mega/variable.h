Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Servo myservo;  // create servo object to control a servo
Profile forward_x;
Profile forward_y;
Profile rotate;
float silo_offset_index;
float x_pos = 0;
float y_pos = 0;
float theta_pos = 0;
float theta_pid = 0;
float d_theta_pid = 0;
float theta_imu_now = 0;
float coord_now[3] = { 0, 0, 0 };
int posPrev[] = { 0, 0 };
long pos[] = { 0, 0, 0, 0 };
long pos_delta[] = { 0, 0, 0, 0 };
int speed_desired[4];
bool start_run = 0;
float dx = 0;
float dy = 0;
bool start_send = 0;
float d_theta = 0;
float p[3] = { 10.1, 10.1, 2.8 };  // 0.11 0.125
float d[3] = { 41.4, 41.4, 24.5 };  // 12.1 13.5
float p_sensor = 0.0;
float d_sensor = 0;
float y_desired;
float i_input = 0;
// float d[3] = { 0, 0, 0 };
float output_speed[] = { 0, 0, 0 };
float last_e[3] = { 0, 0, 0 };
float error[3] = { 0, 0, 0 };
float theta_imu = 0;
float d_theta_imu = 0;
float last_steering_error = 0;
float theta_imu_offset;
bool offset_ = 0;
sensors_event_t orientationData;
double x_goal = 430.5;
double x_goal_2 = 770.1;
double y_goal = 200;
float angle_des = 0;
bool silo_now = 0;
bool wait_for_done = 0;
uint8_t state_silo = 0;
bool button_state_start = 0;
bool button_state_retry = 0;
volatile uint8_t state_main = waiting;
bool init_sensor = 0;
float error_angle_sensor = 0;
bool mode_robot = 0;
void reset_pid() {
  for (int i = 0; i < 3; i++) {
    error[i] = 0;
    last_e[i] = 0;
    output_speed[i] = 0;
  }
}
void reset_all() {
  forward_x.reset();
  forward_y.reset();
  rotate.reset();
  x_pos = 0;
  last_steering_error = 0;
  for (int i = 0; i < 4; i++) {
    speed_desired[i] = 0;
  }
  y_pos = 0;
  theta_pos = 0;
  // theta_imu = 0;
  error_angle_sensor = 0;
  // theta_pid = 0;
  for (int i = 0; i < 3; i++) {
    error[i] = 0;
    last_e[i] = 0;
    output_speed[i] = 0;
  }
}
float calculate_angle_sensor(uint16_t dis_a1, uint16_t dis_a2) {
  float angle;
  int diff = dis_a2 - dis_a1;
  if (abs(diff) <= 3) return 0;
  float a4;
  if (diff > 0) {
    a4 = (float)(dis_a1 * a3) / (diff);
    angle = atan(dis_a1 / a4) * 57.29;
  } else {
    a4 = (dis_a2 * a3) / (diff);
    angle = atan(dis_a2 / a4) * 57.29;
  }
  // Serial.println(a4);
  return angle;  // rad
}
void move_gripper(bool direct, int speed) {
  if (!direct) {
    analogWrite(dir, abs(speed));
    analogWrite(pwm, 0);

  } else {
    analogWrite(dir, 0);
    analogWrite(pwm, abs(speed));
  }
}
void gripper_on(bool dir) {
  if (dir == on) myservo.write(0);  // mo coap ra
  else myservo.write(120);
}