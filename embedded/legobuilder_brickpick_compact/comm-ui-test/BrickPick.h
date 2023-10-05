#ifndef BrickPick_h
#define BrickPick_h

#include "Arduino.h"


class BrickPick {
  public:
    double _left_plate_pos;
    double _right_plate_pos;

    BrickPick();

    void move_left_plate_up();
    void move_left_plate_down();

    double get_left_plate_pos();
    double get_right_plate_pos();

  private:
    const double _left_plate_pos_min = 0;
    const double _left_plate_pos_max = 5;
};

#endif
