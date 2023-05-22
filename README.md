# BrickPick
Lego Manipulator

Serial Interface:
-----Command Bank------
H: Help
G q1 q2 q3: GOTO <q1:long_plate, q2:short_plate, q3:plunger> in mm
B plate brick: BRICK GOTO <0:long_plate, 1:short_plate>, base of brick <0,1,2,3>
P: Toggle ON/OFF Serial Plotting
D: DEPOSIT Brick
R: RAISE Plunger


joints: < long_plate, short_plate, plunger, NONE >
joint_limits: < <1.5, 49.0>, <1.5, 49.0>, <1.0, 8.5>, <0, 0> >



--------------myServoController-------------------
PID servo controller for Actuonix linear actuators
tuned for L12-50 and L12-10
NUM_MOTORS = 4

API:

myServoController(Adafruit_DCMotor* motors[NUM_MOTORS], 
                  Adafruit_ADS1015* adcs[NUM_MOTORS], 
                  int channels[NUM_MOTORS],
                  float joint_lengths[NUM_MOTORS],
                  float joint_limits[NUM_MOTORS][2]);

Class constructor
    motors: Adafruit_DCMotor objects
    adcs: Adafruit_ADS1015 objects
    channels: adc channel mapping
    joint_lengths: maximum stroke length in mm
    joint_limits: commanded joint limits


void init_controller();
    Initializiation function for setup()
    calls reset_joints()
    Resets commands to release motors


void reset_joints();
    Drive motors to minimum extension


void cmdPos(float qCmd[]);
    Make the internal PID setpoint qCmd[] in mm
    <q1, q2, q3, q4>


void cmdPosSingle(float qCmd, int i);
    Make the internal PID setpoint qCmd[i] = qCmd in mm
    <q1, q2, q3, q4>


void cmdVel(float q_dotCmd[]);
    Set the velocity reference command value in mm/s


void driveController();
    Perform state estimation
    Compute PID values
    Set Motor values


void print_state();
    Print controller state to Serial Monitor in a readable format


void plot_state();
    Print controller state for Serial Plotter


void ramp2pos(float xf[NUM_MOTORS], float vmax, float amax);
    Drive servos along a ramp trajectory to xf in mm
    xf: <q1, q2, q3, q4>
