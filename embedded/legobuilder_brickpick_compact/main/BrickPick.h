#ifndef BrickPick_h
#define BrickPick_h

#include "Arduino.h"
#include <Adafruit_MotorShield.h>
#include <Encoder.h>

class BrickPick {
  // All external objects should be passed in as pointers
  // == Main functions ==
  // set_command(): Adjust control policy and targets
  // update(): state estimation, observe state, update command status (In Progress/Complete), safety checks, update_PID 
  // == Read functions ==
  // get_long_plate_pos(): long plate position in mm
  // get_short_plate_pos(): short plate position in mm
  public:
    long _short_plate_pos;
    long _long_plate_pos;
    double _short_plate_pos_mm;
    double _long_plate_pos_mm;

    double _short_plate_control;
    double _long_plate_control;

    BrickPick(Adafruit_DCMotor* short_motor, Adafruit_DCMotor* long_motor, Encoder* short_encoder, Encoder* long_encoder);

    void set_command(const char* request);
    void update();

    void set_short_ctrl(double u);

    long get_short_plate_pos();
    long get_long_plate_pos();
    double get_short_plate_pos_mm();
    double get_long_plate_pos_mm();

  private:
    Adafruit_DCMotor* _short_motor;
    Adafruit_DCMotor* _long_motor;
    Encoder* _short_encoder;
    Encoder* _long_encoder;

    const int _release_tolerance = 20;

    const double _short_plate_pos_min = 0;
    const double _short_plate_pos_max = 5;
};

#endif
