#ifndef BrickPick_h
#define BrickPick_h

#include "Arduino.h"
#include <Adafruit_MotorShield.h>


class BrickPick {
  public:
    double _short_plate_pos;
    double _long_plate_pos;

    double _short_plate_control;
    double _long_plate_control;

    BrickPick(Adafruit_DCMotor* short_motor, Adafruit_DCMotor* long_motor);

    void set_short_ctrl(double u);

    double get_short_plate_pos();
    double get_long_plate_pos();

  private:
    Adafruit_DCMotor* _short_motor;
    Adafruit_DCMotor* _long_motor;

    const int _release_tolerance = 20;

    const double _short_plate_pos_min = 0;
    const double _short_plate_pos_max = 5;
};

#endif
