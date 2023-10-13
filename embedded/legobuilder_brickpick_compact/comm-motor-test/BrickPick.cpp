#include "Arduino.h"
#include "BrickPick.h"

#include <Adafruit_MotorShield.h>

BrickPick::BrickPick(Adafruit_DCMotor *short_motor, Adafruit_DCMotor *long_motor) {
  // Actuators
  _short_motor = short_motor;
  _long_motor = long_motor;

  _short_plate_pos = 0;
  _long_plate_pos = 0;

  _short_plate_control = 0;
  _long_plate_control = 0;
}


void BrickPick::set_short_ctrl(double u) {
  u = min(max(u, -100.0), 100.0); // Clip to -100, 100
  int speed = int(abs(u/100.0) * 255);
  _short_motor->setSpeed(speed);

  if (u > _release_tolerance) {
    _short_motor->run(FORWARD);
  } else if (u < -_release_tolerance) {
    _short_motor->run(BACKWARD);
  } else {
    _short_motor->run(RELEASE);
  }

}


double BrickPick::get_short_plate_pos() {
  return _short_plate_pos;
}

double BrickPick::get_long_plate_pos() {
  return _long_plate_pos;
}