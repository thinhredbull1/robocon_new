#include "Adafruit_VL53L0X.h"
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33
// set the pins to shutdown
#define LEFT_X 40 // 1
#define LEFT_Y 44 // 2 
#define RIGHT_X 42 // 3 
#define SHT_LOX4 38
// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;
/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void init_id()
{

}
void setID() {

  pinMode(LEFT_X, OUTPUT);
  pinMode(LEFT_Y, OUTPUT);
  pinMode(RIGHT_X, OUTPUT);
  // pinMode(SHT_LOX4, OUTPUT);
  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(LEFT_X, LOW);
  digitalWrite(LEFT_Y, LOW);
  digitalWrite(RIGHT_X, LOW);
  // digitalWrite(SHT_LOX4, LOW);
  Serial.println(F("4 laser in reset mode...(pins are low)"));


  // Serial.println(F("Starting..."));
  // all reset
  digitalWrite(LEFT_X, LOW);
  digitalWrite(LEFT_Y, LOW);
  digitalWrite(RIGHT_X, LOW);
  // digitalWrite(SHT_LOX4, LOW);
  delay(10);
  // all unreset
  // digitalWrite(LEFT_X, HIGH);
  // digitalWrite(LEFT_Y, HIGH);
  // digitalWrite(RIGHT_X, HIGH);
  // digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2,LOX3,LOX4
  // digitalWrite(LEFT_X, HIGH);
  // digitalWrite(LEFT_Y, LOW);
  // digitalWrite(RIGHT_X, LOW);
  // digitalWrite(SHT_LOX4, LOW);
  // initing LOX1
  // if (!lox1.begin(LOX1_ADDRESS)) {
  //   Serial.println(F("Failed to boot first VL53L0X"));
  // }
  // delay(10);

  // // activating LOX2
  // digitalWrite(LEFT_Y, HIGH);
  // delay(10);

  // //initing LOX2
  // if (!lox2.begin(LOX2_ADDRESS)) {
  //   Serial.println(F("Failed to boot second VL53L0X"));
  // }

  // activating LOX3
  //   digitalWrite(RIGHT_X, HIGH);
  // delay(10);

  // // initing LOX2
  // if (!lox3.begin(LOX3_ADDRESS)) {
  //   Serial.println(F("Failed to boot third VL53L0X"));
  // }

  // // activating LOX4
  // digitalWrite(SHT_LOX4, HIGH);
  // delay(10);

  // //initing LOX4
  // if (!lox4.begin(LOX4_ADDRESS)) {
  //   Serial.println(F("Failed to boot fourth VL53L0X"));
  // }
}
// Read all value of sensor and print to screen (Used to debug)
void read_all_sensors() {

  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);  // pass in 'true' to get debug data printout!
  // lox3.rangingTest(&measure3, false);
  // lox4.rangingTest(&measure4, false);
  // print sensor one reading
  Serial.print(F("1: "));
  if (measure1.RangeStatus != 4) {  // if not out of range
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if (measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  Serial.print(F(" "));
  // print sensor three reading
  Serial.print(F("3: "));
  if (measure3.RangeStatus != 4) {
    Serial.print(measure3.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  Serial.print(F(" "));
  // print sensor fourth reading
  Serial.print(F("4: "));
  if (measure4.RangeStatus != 4) {
    Serial.print(measure4.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.println();
}

// This function read sensor1 value
void read_sensor_left_x(uint16_t *value) {
  lox1.rangingTest(&measure1, false);
  if (measure1.RangeStatus != 4) {  // if not out of range
    *value = measure1.RangeMilliMeter;
  } else {
    *value = 2200;  //Range from 0.05m to 2.2m
  }
}

void read_sensor_y(uint16_t *value) {
  lox2.rangingTest(&measure2, false);
  if (measure2.RangeStatus != 4) {  // if not out of range
    *value = measure2.RangeMilliMeter;
  } else {
    *value = 2200;  //Range from 0.05m to 2.2m
  }
}

void read_sensor_right_x(uint16_t *value) {
  lox3.rangingTest(&measure3, false);
  if (measure3.RangeStatus != 4) {  // if not out of range
    *value = measure3.RangeMilliMeter;
  } else {
    *value = 2200;  //Range from 0.05m to 2.2m
  }
}

void read_sensor_4(uint16_t *value) {
  lox4.rangingTest(&measure4, false);
  if (measure4.RangeStatus != 4) {  // if not out of range
    *value = measure4.RangeMilliMeter;
  } else {
    *value = 2200;  //Range from 0.05m to 2.2m
  }
}
