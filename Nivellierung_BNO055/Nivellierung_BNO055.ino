#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "DynamixelMotor.h"

const float pitch_offset = -170;
const float roll_offset = 40;
const uint8_t id_pitch = 5;
const uint8_t id_roll = 1;
const int32_t baudrate = 57142;
const uint16_t initial_value = 2048;

Adafruit_BNO055 bno = Adafruit_BNO055();
DynamixelInterface &interface=*createSoftSerialInterface(9,10,2);

DynamixelMotor motor_pitch(interface, id_pitch);
DynamixelMotor motor_roll(interface, id_roll);

void setup() {
  bno.begin();
  interface.begin(baudrate);
  motor_pitch.init();
  motor_roll.init();
  motor_pitch.enableTorque();
  motor_roll.enableTorque(); 
  motor_pitch.jointMode();
  motor_roll.jointMode();
  motor_pitch.goalPosition(initial_value + pitch_offset);
  motor_roll.goalPosition(initial_value);
}

void loop() {
 
uint16_t pitch = 0, roll = 0;

imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  roll = (uint16_t)(((float)-euler.z() + 180.0) * 11.377778 + roll_offset);
  pitch = (uint16_t)(((float)-euler.y() + 180.0) * 11.377778 + pitch_offset);
  motor_pitch.goalPosition(pitch);
  motor_roll.goalPosition(roll);
}
