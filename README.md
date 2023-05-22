# BrickPick
Lego Manipulator

Serial Interface:<br />
-----Command Bank------<br />
H: Help<br />
G q1 q2 q3: GOTO <q1:long_plate, q2:short_plate, q3:plunger> in mm<br />
B plate brick: BRICK GOTO <0:long_plate, 1:short_plate>, base of brick <0,1,2,3><br />
P: Toggle ON/OFF Serial Plotting<br />
D: DEPOSIT Brick<br />
R: RAISE Plunger<br />


joints: < long_plate, short_plate, plunger, NONE ><br />
joint_limits: < <1.5, 49.0>, <1.5, 49.0>, <1.0, 8.5>, <0, 0> ><br />



--------------myServoController-------------------<br />
PID servo controller for Actuonix linear actuators<br />
tuned for L12-50 and L12-10<br />
NUM_MOTORS = 4<br />

API:<br />

myServoController(Adafruit_DCMotor* motors[NUM_MOTORS], <br />
                  Adafruit_ADS1015* adcs[NUM_MOTORS], <br />
                  int channels[NUM_MOTORS],<br />
                  float joint_lengths[NUM_MOTORS],<br />
                  float joint_limits[NUM_MOTORS][2]);<br />

Class constructor<br />
    motors: Adafruit_DCMotor objects<br />
    adcs: Adafruit_ADS1015 objects<br />
    channels: adc channel mapping<br />
    joint_lengths: maximum stroke length in mm<br />
    joint_limits: commanded joint limits<br />


void init_controller();<br />
    Initializiation function for setup()<br />
    calls reset_joints()<br />
    Resets commands to release motors<br />


void reset_joints();<br />
    Drive motors to minimum extension<br />


void cmdPos(float qCmd[]);<br />
    Make the internal PID setpoint qCmd[] in mm<br />
    <q1, q2, q3, q4><br />


void cmdPosSingle(float qCmd, int i);<br />
    Make the internal PID setpoint qCmd[i] = qCmd in mm<br />
    <q1, q2, q3, q4><br />


void cmdVel(float q_dotCmd[]);<br />
    Set the velocity reference command value in mm/s<br />


void driveController();<br />
    Perform state estimation<br />
    Compute PID values<br />
    Set Motor values<br />


void print_state();<br />
    Print controller state to Serial Monitor in a readable format<br />


void plot_state();<br />
    Print controller state for Serial Plotter<br />


void ramp2pos(float xf[NUM_MOTORS], float vmax, float amax);<br />
    Drive servos along a ramp trajectory to xf in mm<br />
    xf: <q1, q2, q3, q4><br />
