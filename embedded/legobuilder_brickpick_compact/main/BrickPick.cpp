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

BrickPick::BrickPick(Adafruit_DCMotor *short_motor, Adafruit_DCMotor *long_motor, Adafruit_DCMotor *plunger_motor) {
  // Actuators
  _short_motor = short_motor;
  _long_motor = long_motor;
  _plunger_motor = plunger_motor;

  // Sensors
  

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

  // Request parsing
  // this is kept very simple for the moment and is not super robust/scalable
  // find a maintained HTTP parser for better
  String s = request;
  if (s.startsWith("/set_long_ctrl")) {
    size_t i = s.indexOf("=");
    long u = s.substring(i + 1, n).toInt();
    set_long_control_mode(0);  // velocity
    set_long_ctrl(u);
  }
  if (s.startsWith("/set_short_ctrl")) {
    size_t i = s.indexOf("=");
    long u = s.substring(i + 1, n).toInt();
    set_short_control_mode(0); // velocity
    set_short_ctrl(u);
  }
  if (s.startsWith("/set_long_target_brick") || s.startsWith("/goto_long_brick")) {
    size_t i = s.indexOf("=");
    size_t brick = s.substring(i + 1, n).toInt();
    set_long_control_mode(1);  // PID
    set_long_target_brick(brick);
  }
  if (s.startsWith("/set_short_target_brick") || s.startsWith("/goto_short_brick")) {
    size_t i = s.indexOf("=");
    size_t brick = s.substring(i + 1, n).toInt();
    set_short_control_mode(1); // PID
    set_short_target_brick(brick);
  }
  if (s.startsWith("/set_long_target_mm")) {
    size_t i = s.indexOf("=");
    double target_mm = s.substring(i + 1, n).toDouble();
    set_long_control_mode(1);  // PID
    set_long_target_mm(target_mm);
  }
  if (s.startsWith("/set_short_target_mm")) {
    size_t i = s.indexOf("=");
    double target_mm = s.substring(i + 1, n).toDouble();
    set_short_control_mode(1); // PID
    set_short_target_mm(target_mm);
  }
  if (s.startsWith("/stop")) {
    // What to do about PID here?
    // Disable for now
    set_long_control_mode(0);  // velocity
    set_short_control_mode(0); // velocity
    set_short_ctrl(0);
    set_long_ctrl(0);
    // Wait until no velocity?
  }
  if (s.startsWith("/goto_plunger")) {
    // Set plunger PID target to down or up
    size_t i = s.indexOf("=");
    size_t plunger_target = s.substring(i + 1,n).toInt();
    // 0 is up
    // 1 is down
    set_plunger_target(plunger_target);
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

void BrickPick::set_short_control_mode(size_t control_mode) {
  if (_short_control_mode == 0) {
    // Currently in velocity
    if (control_mode == 0) {
      return;
    } else if (control_mode == 1) {
      // Changing to PID
      reset_short_PID();
      _short_control_mode = control_mode;
    } else {
      return;
    }
  } else if (_short_control_mode == 1) {
    // Currently in PID
    if (control_mode == 0) {
      _short_control_mode = control_mode;
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

void BrickPick::reset_short_PID() {
  _e_short = 0;
  _e_short_prev = 0;
  _de_short = 0;
  _e_sum_short = 0;
}

bool BrickPick::is_valid_request(const char *request, size_t n) {
  String s = request;
  if (s.startsWith("/set_long_ctrl")) { return true; }
  if (s.startsWith("/set_short_ctrl")) { return true; }
  if (s.startsWith("/set_long_target_brick") || s.startsWith("/goto_long_brick")) { return true; }
  if (s.startsWith("/set_short_target_brick") || s.startsWith("/goto_short_brick")) { return true; }
  if (s.startsWith("/set_long_target_mm")) { return true; }
  if (s.startsWith("/set_short_target_mm")) { return true; }
  if (s.startsWith("/stop")) { return true; }
  if (s.startsWith("/goto_plunger")) {return true; }

  if (s.startsWith("/reset")) { return true; }
  return false;
}

int BrickPick::check_command_status(String active_command) {
  // active_command: /goto_long_brick?..., /goto_plunger?...
  int r;
  // 0 is complete
  // 1 is pending
  // 2 is failed (not implemented)
  if (active_command.startsWith("/goto_plunger")) { // Goto plunger waits for plunger to stall
    r = _plunger_stalled ? 0 : 1;
  } else if (active_command.startsWith("/set")) { // Any set command is an immediate complete!
    r = 0;
  } else if (active_command.startsWith("/goto_long")) { // Goto plates waits for low ctrl and error
    r = (abs(_long_plate_control) < 65535 / 50 && _e_long  < LENGTH2ENCODER * 0.5) ? 0 : 1; // 2% drive and 0.5mm tolerance
  } else if (active_command.startsWith("/goto_short")) { // Goto plates waits for low ctrl and error
    r = (abs(_short_plate_control) < 65535 / 50 && _e_short  < LENGTH2ENCODER * 0.5) ? 0 : 1; // 2% drive and 0.5mm tolerance
  } else { // Something weird
    r = 2;
  }

  return r;
}

void BrickPick::update() {
  // Motors reversed
  _short_plate_pos_mm = _short_plate_pos * ENCODER2LENGTH;
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

  // short plate
  if (_short_control_mode == 0) {
    // Velocity
    // Nothing
  } else if (_short_control_mode == 1) {
    // Position
    _e_short = _short_plate_target - _short_plate_pos;
    _de_short = _e_short - _e_short_prev;
    _e_sum_short = min(_e_sum_short_max, _e_sum_short + _e_short);

    long u_s = _Kp_s * _e_short + _Kd_s * _de_short + _Ki_s * _e_sum_short;
    set_short_ctrl(u_s);

    _e_short_prev = _e_short;
  }

  if (!_plunger_stalled) {
    update_plunger_ctrls();
  }
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

void BrickPick::short_plate_pos_incr() {
  _short_plate_pos ++;
}

void BrickPick::short_plate_pos_decr() {
  _short_plate_pos --;
}


void BrickPick::set_plunger_target(size_t i) {
  _plunger_stalled = false;
  _plunger_target = i;
}

void BrickPick::update_plunger_ctrls() {
  int MAX_POS = 3300;
  int MIN_POS = 2500;
  int plunger_pos = analogRead(PLUNGER_POT);
  int plunger_target = (MAX_POS - MIN_POS) * _plunger_target + MIN_POS;
  int error = plunger_target - plunger_pos;
  int plunger_ctrl = _Kp_plunger * error;
  if (abs(error) > 50) {
    plunger_ctrl = 255 * (error > 0 ? 1 : -1);
  }

  // Check if plunger is stalled by checking if the last 5 controls and positions are all equal
  bool stall_check = true;
  for (int i = 0; i < _plunger_buf_n - 1; i++) {
    _plunger_ctrls[i] = _plunger_ctrls[i+1];
    _plunger_positions[i] = _plunger_positions[i+1];

    if ((abs(_plunger_ctrls[i]) > 30) ||
        (abs(_plunger_positions[i] - plunger_pos) > 15)) {
          stall_check = false;
        }
  }
  if (abs(plunger_ctrl) > 30) {stall_check = false; }
  _plunger_ctrls[_plunger_buf_n - 1] = plunger_ctrl;
  _plunger_positions[_plunger_buf_n - 1] = plunger_pos;
  if (stall_check) {
    for (int i = 0; i < _plunger_buf_n; i++) {
      _plunger_ctrls[i] = 0;
    }
    _plunger_stalled = true;
    _plunger_motor->setSpeed(0);
    _plunger_motor->run(RELEASE);

    Serial.println("Plunger Stalled");
    return;
  }
  _plunger_motor->run((plunger_ctrl < 0) ? FORWARD : BACKWARD);
  _plunger_motor->setSpeed(abs(plunger_ctrl));
}