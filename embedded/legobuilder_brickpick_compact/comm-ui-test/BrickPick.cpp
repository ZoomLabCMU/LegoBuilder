#include "Arduino.h"
#include "BrickPick.h"

BrickPick::BrickPick() {
  _left_plate_pos = 0;
  _right_plate_pos = 0;
}


void BrickPick::move_left_plate_up() {
  _left_plate_pos = max(_left_plate_pos_min, _left_plate_pos - 1);
}

void BrickPick::move_left_plate_down() {
  _left_plate_pos = min(_left_plate_pos_max, _left_plate_pos + 1);
}


double BrickPick::get_left_plate_pos() {
  return _left_plate_pos;
}

double BrickPick::get_right_plate_pos() {
  return _right_plate_pos;
}