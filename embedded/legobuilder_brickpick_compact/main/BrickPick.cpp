#include "Arduino.h"
#include "BrickPick.h"

#include <Adafruit_MotorShield.h>
#include <Encoder.h>

const int ENCODER_RESOLUTION = 12; // clicks / enc rev
const int GEAR_RATIO = 10; // enc rev / screw rev
const int SCREW_PITCH = 3; //Note this needs to be measured more accurately (mm/screw rev)
const double ENCODER2LENGTH = (double) SCREW_PITCH / (ENCODER_RESOLUTION * GEAR_RATIO); // (mm/click)
const double LENGTH2ENCODER = (double) (ENCODER_RESOLUTION * GEAR_RATIO) / SCREW_PITCH;

BrickPick::BrickPick(Adafruit_DCMotor *short_motor, Adafruit_DCMotor *long_motor, Encoder *short_encoder, Encoder *long_encoder) {
  // Actuators
  _short_motor = short_motor;
  _long_motor = long_motor;

  // Sensors
  _short_encoder = short_encoder;
  _long_encoder = long_encoder;

  _short_plate_pos = 0;
  _short_plate_pos_mm = 0;
  _long_plate_pos = 0;
  _long_plate_pos_mm = 0;

  _short_plate_control = 0;
  _long_plate_control = 0;
}

void BrickPick::set_command(const char* request) {
  int x;
  (void) x;
}

void BrickPick::update() {
  // Motors reversed
  _short_plate_pos = -_short_encoder->read();
  _short_plate_pos_mm = _short_plate_pos * ENCODER2LENGTH;
  _long_plate_pos = -_long_encoder->read();
  _long_plate_pos_mm = _long_plate_pos * ENCODER2LENGTH;
}

void BrickPick::set_short_ctrl(double u) {
  u = min(max(u, -100.0), 100.0); // Clip to -100, 100
  int speed = int(abs(u/100.0) * 255);
  _short_motor->setSpeed(speed);

  if (u > _release_tolerance) {
    _short_motor->run(BACKWARD);
  } else if (u < -_release_tolerance) {
    _short_motor->run(FORWARD);
  } else {
    _short_motor->run(RELEASE);
  }

}


long BrickPick::get_short_plate_pos() {
  return _short_plate_pos;
}

long BrickPick::get_long_plate_pos() {
  return _long_plate_pos;
}

double BrickPick::get_short_plate_pos_mm() {
  return _short_plate_pos_mm;
}

double BrickPick::get_long_plate_pos_mm() {
  return _long_plate_pos_mm;
}