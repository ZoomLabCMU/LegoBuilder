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
    // Unitless variables will be in their native units (e.g _pos and encoder ticks)
    // Encoders: encoder ticks (signed long)
    // Motors: control value -100->100 (double)
    BrickPick(Adafruit_DCMotor* short_motor, Adafruit_DCMotor* long_motor, Encoder* short_encoder, Encoder* long_encoder);

    void set_command(const char* request, size_t n);
    bool is_valid_request(const char* request, size_t n);
    bool request_complete();    
    void update();

    void set_short_ctrl(long u);
    void set_long_ctrl(long u);

    long get_short_plate_pos();
    long get_long_plate_pos();
    double get_short_plate_pos_mm();
    double get_long_plate_pos_mm();

  private:
    // Objects
    Adafruit_DCMotor* _short_motor;
    Adafruit_DCMotor* _long_motor;
    Encoder* _short_encoder;
    Encoder* _long_encoder;

    // Safety/Boundaries
    const uint16_t _release_tolerance = 65535 / 20; // 5% of maximum value
    //const double _long_pos_max_mm = 0;
    //const double _short_pos_max_mm = 0;

    // Comms
    int _request_status = 0; // 0 for complete, 1 for pending, 2 for failure

    // State Estimation
    long _short_plate_pos;
    long _long_plate_pos;
    double _short_plate_pos_mm;
    double _long_plate_pos_mm;

    // Control
    long _short_plate_control;
    long _long_plate_control;
    long _short_plate_target;
    long _long_plate_target;

    double _Kp_l = 0;
    double _Ki_l = 0;
    double _Kd_l = 0;
    
    double _Kp_s = 0;
    double _Ki_s = 0;
    double _Kd_s = 0;
};

#endif
