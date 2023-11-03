#include "Arduino.h"
#include "BrickPick.h"
#include "Pins.h"

#include <Adafruit_MotorShield.h>
#include <Encoder.h>

const int ENCODER_RESOLUTION = 6;                                                      // clicks / enc rev
const int GEAR_RATIO = 10;                                                              // enc rev / screw rev
const int SCREW_PITCH = 3;                                                              //Note this needs to be measured more accurately (mm/screw rev)
const double ENCODER2LENGTH = (double)SCREW_PITCH / (ENCODER_RESOLUTION * GEAR_RATIO);  // (mm/click)
const double LENGTH2ENCODER = (double)(ENCODER_RESOLUTION * GEAR_RATIO) / SCREW_PITCH;

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

void BrickPick::set_command(const char *request, size_t n) {
  if (!is_valid_request(request, n)) {
    Serial.println("Invalid request");
    return;
  }
  _request_status = 1;

  // Request parsing
  // this is kept very simple for the moment and is not super robust/scalable
  // find a maintained HTTP parser for better
  String s = request;
  if (s.startsWith("/set_long_ctrl")) {
    size_t i = s.indexOf("=");
    long u = s.substring(i + 1, n).toInt();
    set_long_control_mode(0);  // velocity
    set_long_ctrl(u);
    _request_status = 0;
  }
  if (s.startsWith("/set_short_ctrl")) {
    size_t i = s.indexOf("=");
    long u = s.substring(i + 1, n).toInt();
    set_short_ctrl(u);
    _request_status = 0;
  }
  if (s.startsWith("/set_long_target_brick")) {
    size_t i = s.indexOf("=");
    size_t brick = s.substring(i + 1, n).toInt();
    set_long_control_mode(1);  // PID
    set_long_target_brick(brick);
    _request_status = 0;
  }
  if (s.startsWith("/set_short_target_brick")) {
    size_t i = s.indexOf("=");
    size_t brick = s.substring(i + 1, n).toInt();
    set_short_target_brick(brick);
    _request_status = 0;
  }
  if (s.startsWith("/set_long_target_mm")) {
    size_t i = s.indexOf("=");
    size_t target_mm = s.substring(i + 1, n).toInt();
    set_long_control_mode(1);  // PID
    set_long_target_mm(target_mm);
    _request_status = 0;
  }
  if (s.startsWith("/set_short_target_mm")) {
    size_t i = s.indexOf("=");
    size_t target_mm = s.substring(i + 1, n).toInt();
    set_short_target_mm(target_mm);
    _request_status = 0;
  }
  if (s.startsWith("/stop")) {
    // What to do about PID here?
    // Disable for now
    set_long_control_mode(0);  // velocity
    set_short_ctrl(0);
    set_long_ctrl(0);
    // Wait until no velocity?
    _request_status = 0;
  }
}

void BrickPick::set_long_control_mode(size_t control_mode) {
  if (_long_control_mode == 0) {
    // Currently in velocity
    if (control_mode == 0) {
      return;
    } else if (control_mode == 1) {
      // Changing to PID
      reset_long_PID();
      _long_control_mode = control_mode;
    } else {
      return;
    }
  } else if (_long_control_mode == 1) {
    // Currently in PID
    if (control_mode == 0) {
      _long_control_mode = control_mode;
    } else if (control_mode == 1) {
      // Do we want to reset anything here?
      return;
    } else {
      return;
    }
  }
}

void BrickPick::reset_long_PID() {
  _e_long = 0;
  _e_long_prev = 0;
  _de_long = 0;
  _e_sum_long = 0;
}

bool BrickPick::is_valid_request(const char *request, size_t n) {
  // TODO - make a command registry to validate requests
  String s = request;
  if (s.startsWith("/set_long_ctrl")) { return true; }
  if (s.startsWith("/set_short_ctrl")) { return true; }
  if (s.startsWith("/set_long_target_brick")) { return true; }
  if (s.startsWith("/set_short_target_brick")) { return true; }
  if (s.startsWith("/set_long_target_mm")) { return true; }
  if (s.startsWith("/set_short_target_mm")) { return true; }
  if (s.startsWith("/stop")) { return true; }

  if (s.startsWith("/reset")) { return true; }
  return false;
}

bool BrickPick::request_complete() {
  // 0 is complete
  // 1 is pending
  // 2 is failed
  return (_request_status == 0);
}

void BrickPick::update() {
  // Motors reversed
  //_short_plate_pos = -_short_encoder->getCount();
  _short_plate_pos_mm = _short_plate_pos * ENCODER2LENGTH;
  //_long_plate_pos = -_long_encoder->getCount();
  _long_plate_pos_mm = _long_plate_pos * ENCODER2LENGTH;

  // === PID ===
  // long plate
  if (_long_control_mode == 0) {
    // Velocity
    // Nothing
  } else if (_long_control_mode == 1) {
    // Position
    _e_long = _long_plate_target - _long_plate_pos;
    _de_long = _e_long - _e_long_prev;
    _e_sum_long = min(_e_sum_long_max, _e_sum_long + _e_long);

    long u_l = _Kp_l * _e_long + _Kd_l * _de_long + _Ki_l * _e_sum_long;
    set_long_ctrl(u_l);

    _e_long_prev = _e_long;
  }
}

void BrickPick::set_short_ctrl(long u) {
  // Motor directions constrained HERE with positive control in the same direction as extension
  long u_max = 65535;                       // Max of a uint16_t
  u = constrain(u, -u_max, u_max);          // Clip
  u = abs(u) > _release_tolerance ? u : 0;  //Flatten deadzone
  uint16_t speed = abs(u);
  _short_motor->setSpeedFine(speed);

  if (u > 0) {
    _short_motor->run(BACKWARD);
  } else if (u < 0) {
    _short_motor->run(FORWARD);
  } else {  // u == 0
    _short_motor->run(RELEASE);
  }
  // Update object parameter
  _short_plate_control = u;
}

void BrickPick::set_long_ctrl(long u) {
  // Motor directions constrained HERE with positive control in the same direction as extension
  long u_max = 65535;                       // Max of a uint16_t
  u = constrain(u, -u_max, u_max);          // Clip
  u = abs(u) > _release_tolerance ? u : 0;  //Flatten deadzone
  uint16_t speed = abs(u);
  _long_motor->setSpeedFine(speed);

  if (u > 0) {
    _long_motor->run(FORWARD);
  } else if (u < 0) {
    _long_motor->run(BACKWARD);
  } else {  // u == 0
    _long_motor->run(RELEASE);
  }
  // Update object parameter
  _long_plate_control = u;
}


long BrickPick::get_short_plate_pos() {
  return _short_plate_pos;
}

long BrickPick::get_long_plate_pos() {
  return _long_plate_pos;
}

long BrickPick::get_short_plate_target() {
  return _short_plate_target;
}

long BrickPick::get_long_plate_target() {
  return _long_plate_target;
}

double BrickPick::get_short_plate_pos_mm() {
  return _short_plate_pos_mm;
}

double BrickPick::get_long_plate_pos_mm() {
  return _long_plate_pos_mm;
}

double BrickPick::get_short_plate_target_mm() {
  return get_short_plate_target() * ENCODER2LENGTH;
}

double BrickPick::get_long_plate_target_mm() {
  return get_long_plate_target() * ENCODER2LENGTH;
}

long BrickPick::get_short_plate_ctrl() {
  return _short_plate_control;
}

long BrickPick::get_long_plate_ctrl() {
  return _long_plate_control;
}

void BrickPick::set_long_target_brick(size_t brick) {
  double BRICK_HEIGHT_MM = 9.6;
  size_t BRICK_MAX = 5;
  double HEIGHT_OFFSET_MM = -2.0;
  brick = min(brick, BRICK_MAX);
  set_long_target_mm(BRICK_HEIGHT_MM * brick + HEIGHT_OFFSET_MM);
}

void BrickPick::set_long_target_mm(double target_mm) {
  // add limiting here later
  long target = LENGTH2ENCODER * target_mm;
  _long_plate_target = target;
}

void BrickPick::set_short_target_brick(size_t brick) {
  double BRICK_HEIGHT_MM = 9.6;
  size_t BRICK_MAX = 5;
  double HEIGHT_OFFSET_MM = -2.0;
  brick = min(brick, BRICK_MAX);
  set_short_target_mm(BRICK_HEIGHT_MM * brick + HEIGHT_OFFSET_MM);
}

void BrickPick::set_short_target_mm(double target_mm) {
  // add limiting here later
  long target = LENGTH2ENCODER * target_mm;
  _short_plate_target = target;
}

void BrickPick::long_plate_pos_incr() {
  _long_plate_pos ++;
}

void BrickPick::long_plate_pos_decr() {
  _long_plate_pos --;
}