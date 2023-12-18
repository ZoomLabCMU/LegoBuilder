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
    BrickPick(Adafruit_DCMotor* short_motor, Adafruit_DCMotor* long_motor, Adafruit_DCMotor* plunger_motor);

    void set_command(const char* request, size_t n);
    bool is_valid_request(const char* request, size_t n);
    int check_command_status(String active_command);
    void update();

    void long_plate_pos_incr();
    void long_plate_pos_decr();
    void short_plate_pos_incr();
    void short_plate_pos_decr();

    long get_short_plate_pos();
    long get_long_plate_pos();
    double get_short_plate_pos_mm();
    double get_long_plate_pos_mm();

    long get_short_plate_target();
    long get_long_plate_target();
    double get_short_plate_target_mm();
    double get_long_plate_target_mm();

    long get_short_plate_ctrl();
    long get_long_plate_ctrl();

  private:
    // Objects
    Adafruit_DCMotor* _short_motor;
    Adafruit_DCMotor* _long_motor;
    Adafruit_DCMotor* _plunger_motor;

    // Safety/Boundaries
    const uint16_t _release_tolerance = 65535 / 100; // 1% of maximum value
    //const double _long_pos_max_mm = 0;
    //const double _short_pos_max_mm = 0;


    // State Estimation
    volatile long _short_plate_pos;
    volatile long _long_plate_pos;
    double _short_plate_pos_mm;
    double _long_plate_pos_mm;
    // Control
    size_t _long_control_mode = 0; // 0 for velocity, 1 for position
    size_t _short_control_mode = 0; // 0 for velocity, 1 for position

    void set_long_control_mode(size_t control_mode);
    void set_short_control_mode(size_t control_mode);

    long _short_plate_control;
    long _long_plate_control;

    void set_short_ctrl(long u);
    void set_long_ctrl(long u);

    long _short_plate_target;
    long _long_plate_target;

    void set_short_target_brick(size_t brick);
    void set_long_target_brick(size_t brick);
    void set_short_target_mm(double target_mm);
    void set_long_target_mm(double target_mm);

    double _Kp_l = 500;
    double _Ki_l = 0.0;
    double _Kd_l = 1200;
    long _e_long = 0;
    long _e_long_prev = 0;
    long _de_long = 0;
    long _e_sum_long = 0;
    long _e_sum_long_max = 0xFFFFFFFF;
    
    double _Kp_s = 500;
    double _Ki_s = 0.0;
    double _Kd_s = 600;
    long _e_short = 0;
    long _e_short_prev = 0;
    long _de_short = 0;
    long _e_sum_short = 0;
    long _e_sum_short_max = 0xFFFFFFFF;
    
    void reset_long_PID();
    void reset_short_PID();

    // Plunger
    size_t _plunger_target = 0; // 0 is up, 1 is down
    const static size_t _plunger_buf_n = 15;
    long _plunger_ctrls[_plunger_buf_n] = {0};
    int _plunger_positions[_plunger_buf_n] = {0};
    bool _plunger_stalled;
    double _Kp_plunger = 5.0;

    void set_plunger_target(size_t i);
    void update_plunger_ctrls();
};

#endif
