
#include "config.h"
#include <util/atomic.h>
#include "digitalWriteFast.h"
#include <Wire.h>
#include "YetAnotherPcInt.h"

int speed_desired_left = 0;
int speed_desired_right = 0;
volatile int speed_desired[] = { 0, 0 };
bool ff = 0;
volatile int encoder_count[NMOTORS];

volatile int pos[2] = { 0, 0 };
SimplePID pid[NMOTORS];
int encod_state[4];
void readEncoderM0() {
  static bool old_a = false;
  bool newA = digitalReadFast(enca[M1]);
  bool newB = digitalReadFast(encb[M1]);
  int delta = 0;
  if (newA && !old_a) {
    delta = newB > 0 ? dir_encod_M1 : -dir_encod_M1;
  }  // rising
  else {
    delta = newB > 0 ? -dir_encod_M1 : dir_encod_M1;
  }
  encoder_count[M1] += delta;
  old_a = newA;
}
void readEncoderM1() {
  static bool old_a = false;
  bool newA = digitalReadFast(encb[M1]);
  bool newB = digitalReadFast(enca[M1]);
  int delta = 0;
  if (newA && !old_a) {
    delta = newB > 0 ? dir_encod_M1 : -dir_encod_M1;
  }  // rising
  else {
    delta = newB > 0 ? -dir_encod_M1 : dir_encod_M1;
  }
  encoder_count[M1] -= delta;
  old_a = newA;
}


void sendLong(long value) {
  for (int k = 0; k < 4; k++) {
    byte out = (value >> 8 * (3 - k)) & 0xFF;
    Wire.write(out);
  }
}
int receiveLong() {
  int outValue = 0;
  for (int k = 0; k < 2; k++) {
    byte nextByte = Wire.read();
    outValue = (outValue << 8) | nextByte;
  }
  return outValue;
}
void requestEvent() {
  sendLong(pos[M0]);
  sendLong(pos[M1]);
  pos[M0] = 0;
  pos[M1] = 0;
}

void receiveEvent(int howMany) {
  speed_desired[M0] = receiveLong();
  speed_desired[M1] = receiveLong();
}
bool receive_uart() {
  if (Serial.available()) {
    String c = Serial.readString();
    int index_now = c.indexOf("/");
    int index_kp_desired = c.indexOf(":");
    int index_ff = c.indexOf("f");

    if (index_now != -1) {
      // speed_linear = c.substring(0, index_now).toFloat();
      // angular_speed = (c.substring(index_now + 1).toFloat());
      speed_desired[M0] = c.substring(0, index_now).toInt();
      speed_desired[M1] = c.substring(index_now + 1).toInt();
      // Serial.print("speed:");

      // Serial.print(speed_desired[0]);
      // Serial.print(",");
      // Serial.println(speed_desired[1]);
      return 1;
    } else if (index_ff != -1) {
      ff = c.substring(0, index_ff).toInt();
      Serial.println("ff " + String(ff));
    } else if (index_kp_desired != -1) {
      int index_cal = c.indexOf("#");
      if (index_cal != -1) {
        float new_kp = c.substring(0, index_kp_desired).toFloat();
        float new_ki = c.substring(index_kp_desired + 1, index_cal).toFloat();
        float new_kd = c.substring(index_cal + 1).toFloat();

        pid[M0].setParams(new_kp, new_ki, new_kd, 255);
        pid[M1].setParams(new_kp, new_ki, new_kd, 255);

        Serial.print(pid[M1].GetKp());
        Serial.print(" ");
        Serial.print(pid[M1].GetKi());
        Serial.print(" ");
        Serial.println(pid[M1].GetKd());
      }
    }
  }
  return 0;
}
void control_motor(int motor, int speed) {
  bool direct = speed > 0 ? 1 : 0;
  // digitalWrite(dir[motor],direct);
  if (direct) {
    analogWrite(pwm[motor], 255 - abs(speed));
    digitalWrite(dir[motor], 1);
  } else {
    analogWrite(pwm[motor], abs(speed));
    digitalWrite(dir[motor], 0);
  }
}

void setup() {
  Wire.begin(1);  // join I2C
  Wire.setClock(400000);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  if (serial_tune == 1) Serial.begin(9600);
  for (int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT_PULLUP);
    pinMode(encb[k], INPUT_PULLUP);
    pinMode(pwm[k], OUTPUT);
    pinMode(dir[k], OUTPUT);
  }
  pid[M0].setParams(6.85, 43.2, 0, 255);  //39.2 34.6
  pid[M1].setParams(6.88, 42.2, 0, 255);   //39.2 34.6
  // PcInt::attachInterrupt(enca[M0], readEncoderM0, "", CHANGE);
  attachInterrupt(digitalPinToInterrupt(encb[M1]), readEncoderM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enca[M1]), readEncoderM0, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encb[M1]), readEncoderM3, CHANGE);
}
void loop() {
  static unsigned long time_now = micros();
  static bool start_run_motor = 0;
  static unsigned long time_start = millis();
  bool enca_now = digitalReadFast(enca[M0]);
  bool enb_now = digitalReadFast(encb[M0]);
  static int buttonRisingEdgeA = 0;
  static int lastButtonStateA = 0;
  static int buttonRisingEdgeB = 0;
  static int lastButtonStateB = 0;
  int delta_now = 0;
  if (PinStateChanged(enca_now, &lastButtonStateA, &buttonRisingEdgeA)) {
    //Pin 11 state changed.
    //buttonRisingEdgeA says how: 0 == falling, 1 == Rising
    if (buttonRisingEdgeA == 0) {
      delta_now = enb_now > 0 ? dir_encod_M0 : -dir_encod_M0;
    } else {
      delta_now = enb_now > 0 ? -dir_encod_M0 : dir_encod_M0;
    }
    encoder_count[M0] += delta_now;
  }
  if (PinStateChanged(enb_now, &lastButtonStateB, &buttonRisingEdgeB)) {
    //Pin 11 state changed.
    //buttonRisingEdgeA says how: 0 == falling, 1 == Rising
    if (buttonRisingEdgeB == 0) {
      delta_now = enca_now > 0 ? dir_encod_M0 : -dir_encod_M0;
    } else {
      delta_now = enca_now > 0 ? -dir_encod_M0 : dir_encod_M0;
    }
    encoder_count[M0] -= delta_now;
  }
  if (micros() - time_now >= micros_interval) {

    time_now = micros();
    int delta[2] = { 0, 0 };
    int m_pwm[2] = { 0, 0 };
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      for (int i = 0; i < 2; i++) {
        delta[i] = encoder_count[i];
        encoder_count[i] = 0;
      }
    }
    static float new_delta[2] = { 0, 0 };
    static int last_delta[2] = { 0, 0 };
    for (int i = 0; i < 2; i++) {

      pos[i] += delta[i];
      if (ff) m_pwm[i] = speed_ff * speed_desired[i];
      m_pwm[i] += pid[i].compute(delta[i], speed_desired[i], (1.0 / LOOP_FREQUENCY));  // speed >0 ->  delta must >0
      new_delta[i] = new_delta[i] * 0.22826091 + 0.38586955 * (float)delta[i] + 0.38586955 * (float)last_delta[i];
      last_delta[i] = delta[i];
    }

    if (!serial_tune) {
      for (int i = 0; i < 2; i++) {
        control_motor(i, m_pwm[i]);
      }
    }
    //  Serial.print(pos[1]);
    else {

      // delay(200);
      if (receive_uart() && start_run_motor == 0) {

        start_run_motor = 1;
        for (int i = 0; i < 2; i++) pid[i].reset_all();
        time_start = millis();
      }
      if (start_run_motor == 1) {
        static uint16_t count_print = 0;
        count_print += 1;
        if (count_print >= 5) {
          // Serial.println("M0:" + String(pos[M0]));
          // Serial.println(pos[M1]);
          Serial.print(new_delta[M0]);
          Serial.print(",");
          Serial.println(new_delta[M1]);
          count_print = 0;
        }
        if (test_ff) {
          control_motor(M0, speed_desired[M0]);  //
          control_motor(M1, speed_desired[M1]);
        } else {
          control_motor(M0, m_pwm[0]);
          control_motor(M1, m_pwm[1]);
        }
        static int period = time_run_test / 2;
        uint32_t time_now = millis() - time_start;
        // Serial.println(millis());
        // Serial.println(time_start);
        float sinus = sin(2 * PI * time_now / period);  // base pattern
        //  speed_desired_left = (2 * 0.16 / PI) * asin(sinus); // triangle

        if (time_now >= time_run_test) {
          speed_desired_left = 0;
          speed_desired_right = 0;
          start_run_motor = 0;
          control_motor(M0, 0);
          control_motor(M1, 0);
          Serial.println("done");
        }
      }
    }
  }
}