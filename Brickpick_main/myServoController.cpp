#include "myServoController.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

// 1m per 1650 bits (3.3V/4.096V)*2^11 (range is +- 4.096V with 1 gain)
// Multiply ADC_COEFF by stroke (for linear pot) to determine position in mm
#define ADC_COEFF 0.000606060606 //ADC reading to pos in mm/lg.
#define CMD2VEL 0.0920 // Motor command val to vel in [mm/s]
#define VEL2CMD 10.8695652 // vel [mm/s] to motor command val [x/+-255]

myServoController::myServoController(Adafruit_DCMotor* motors[NUM_MOTORS], 
                                     Adafruit_ADS1015* adcs[NUM_MOTORS], 
                                     int channels[NUM_MOTORS],
                                     float joint_lengths[NUM_MOTORS],
                                     float joint_limits[NUM_MOTORS][2]) {
  for (int i=0; i<NUM_MOTORS; i++){
    _motors[i] = motors[i];
    _adcs[i] = adcs[i];
    _channels[i] = channels[i];
    _joint_lengths[i] = joint_lengths[i];
    _joint_limits[i][0] = joint_limits[i][0];
    _joint_limits[i][1] = joint_limits[i][1];
  }
}


void myServoController::init_controller() {
  // Initialize actuator and encoder
  _millis = millis();
  for (int i=0; i<NUM_MOTORS; i++) {
    _motors[i]->setSpeed(0);
    _motors[i]->run(RELEASE);
    _joint_velocities[i] = 0;
  }
  reset_joints();
  read_joint_positions(); 
}

void myServoController::reset_joints() {
  read_joint_positions();
  float xf[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    xf[i] = _joint_limits[i][0]; // _joint_limits = {{min, max}, ...}
  }
  ramp2pos(xf, 25.0, 12.5);
  for (int i=0; i<NUM_MOTORS; i++) {
    _motors[i]->run(RELEASE);
  }
}

void myServoController::read_joint_positions() {
  for (int i=0; i<NUM_MOTORS; i++) {
    int ADC_val = _adcs[i]->readADC_SingleEnded(_channels[i]);
    _joint_positions[i] = ADC_COEFF*_joint_lengths[i]*ADC_val; //ADC_COEFF is bits for 1mm stroke
    //multiply by max joint extension in m for ADC reading to meter conversion
  }
}

void myServoController::update_state() {
  for (int i=0; i<NUM_MOTORS; i++) {
    _prev_joint_positions[i] = _joint_positions[i];
  }
  long curr_millis = millis();
  _delta_millis = curr_millis - _millis;
  _millis = curr_millis;
  read_joint_positions();
  for (int i=0; i<NUM_MOTORS; i++) {
    _joint_velocities[i] = (_joint_positions[i] - _prev_joint_positions[i])/(_delta_millis * 0.001);
  }
  
}

void myServoController::print_state() {
  Serial.println("Joint Positions");
  for (int i=0; i<NUM_MOTORS; i++) {
    Serial.print(_joint_positions[i]); Serial.print('\t');
  }
  Serial.println();
  /*
  Serial.println("Joint Velocities");
  for (int i=0; i<NUM_MOTORS; i++) {
    Serial.print(1000*_joint_velocities[i]); Serial.print('\t');
  }
  Serial.println();
  */
  Serial.println("Joint Commands");
  for (int i=0; i<NUM_MOTORS; i++) {
    Serial.print(min(max(_motorCmd[i],-255),255)); Serial.print('\t');
  }
  Serial.println();
  Serial.println();
}

void myServoController::plot_state() {
  // Joint Positions
  char out[3];
  for (int i=0; i<NUM_MOTORS; i++) {
    sprintf(out, "q%d:", i);
    Serial.print(out);
    Serial.print(_joint_positions[i]); Serial.print('\t');
  }
  /*
  // Joint Velocities
  for (int i=0; i<NUM_MOTORS; i++) {
    sprintf(out, "v%d:", i);
    Serial.print(out);
    Serial.print(1000*_joint_velocities[i]); Serial.print('\t');
  }
  */
  // Joint Commands
  for (int i=0; i<NUM_MOTORS; i++) {
    sprintf(out, "u%d:", i);;
    Serial.print(out);
    Serial.print(min(max(_motorCmd[i],-255),255)); Serial.print('\t');
  }
  Serial.println();
}

void myServoController::clip_cmds(float qCmd[]) {
  for (int i=0; i<NUM_MOTORS; i++) {
    qCmd[i] = min(max(qCmd[i], _joint_limits[i][0]), _joint_limits[i][1]); 
  }
}

void myServoController::cmdPos(float qCmd[]) {
  for(int i=0; i<NUM_MOTORS; i++) {
    _joint_positionsCmd[i] = qCmd[i];
  }
  clip_cmds(_joint_positionsCmd);
}

void myServoController::cmdPosSingle(float qCmd, int i) {
  _joint_positionsCmd[i] = qCmd;
  clip_cmds(_joint_positionsCmd);
}

void myServoController::cmdVel(float q_dotCmd[]) {
  for (int i=0; i<NUM_MOTORS; i++){
    _joint_velocitiesCmd[i] = q_dotCmd[i];
    _refCmd[i] = q_dotCmd[i] * VEL2CMD;
  }
}

void myServoController::positional_PID(float PID_Cmd[NUM_MOTORS]) {
  int integral_error_max = 1.0;
  for (int i=0; i<NUM_MOTORS; i++) {
    float error = _joint_positionsCmd[i] - _joint_positions[i];
    _cmdPos_integral_error[i] = min(max(_cmdPos_integral_error[i] + error * (0.001*_delta_millis),-integral_error_max),integral_error_max);
    _cmdPos_derivative_error[i] = (error - _cmdPos_error[i]) / (0.001*_delta_millis);
    _cmdPos_error[i] = error;
  
    PID_Cmd[i] = (_cmdPos_Kp[i]*_cmdPos_error[i] + 
                  _cmdPos_Ki[i]*_cmdPos_integral_error[i] + 
                  _cmdPos_Kd[i]*_cmdPos_derivative_error[i]);
  }
}

void myServoController::send_motorCmd() {
  int min_cmd = 30;
  for (int i=0; i<NUM_MOTORS; i++) {
    int motor_cmd_val = _motorCmd[i];
    motor_cmd_val = min(max(motor_cmd_val, -255), 255); // Clip to safe bounds
    
    if (motor_cmd_val > min_cmd) {
      _motors[i]->run(FORWARD);
    } else if (motor_cmd_val < -min_cmd){
      _motors[i]->run(BACKWARD);
    } else {
      _motors[i]->run(RELEASE);
    }
    _motors[i]->setSpeed((uint8_t)(abs(motor_cmd_val)));
  }
}

void myServoController::driveController() {
  update_state();
  
  float PID_Cmd[NUM_MOTORS];
  positional_PID(PID_Cmd);

  for (int i=0; i<NUM_MOTORS; i++) {
    _motorCmd[i] = (int)(_refCmd[i] + _lambda_cmdPos*PID_Cmd[i]);
  }

  send_motorCmd();
}

//********************* Trajectories ******************************//
void myServoController::ramp_pos(float t, float qCmd[NUM_MOTORS], float x0[NUM_MOTORS], float xf[NUM_MOTORS], float v[NUM_MOTORS], float amax_ref) {
  // Modify qCmd with the x(t) values of a ramp traj
  float vmax;
  float amax;
  
  for (int i=0; i<NUM_MOTORS; i++) {
    float delta_x = xf[i] - x0[i];
    if (delta_x < 0) {
      vmax = -v[i];
      amax = -amax_ref;
    } else {
      vmax = v[i];
      amax = amax_ref;
    }
    float tr = min(delta_x/vmax, vmax/amax);
    float tm = max(0, delta_x/vmax - vmax/amax);
    // Piece wise integration
    if (t < tr) {
      qCmd[i] = x0[i] + 0.5*amax*pow(t,2);
    } else if (tr <= t & t < tr + tm) {
      qCmd[i] = x0[i] + (0.5*amax*pow(tr,2)) + vmax*(t-tr);
    } else if (tr + tm <= t & t < 2*tr + tm) {
      qCmd[i] = x0[i] + (0.5*amax*pow(tr,2)) + (vmax*tm) + (vmax*(t-tm-tr) - 0.5*amax*pow(t-tm-tr,2));
    } else {
      qCmd[i] = xf[i];
    }
  }
}

void myServoController::ramp_vel(float t, float q_dotCmd[NUM_MOTORS], float x0[NUM_MOTORS], float xf[NUM_MOTORS], float v[NUM_MOTORS], float amax_ref) {
  // Modify q_dotCmd with the v(t) values of a ramp traj
  float vmax;
  float amax;
  
  for (int i=0; i<NUM_MOTORS; i++) {
    float delta_x = xf[i] - x0[i];
    float pure_PID_tol = 0.001; // If dx is small, ref commands may cause overshoot
    if (delta_x < 0) {
      vmax = -v[i];
      amax = -amax_ref;
    } else {
      vmax = v[i];
      amax = amax_ref;
    }
    float tr = min(delta_x/vmax, vmax/amax);
    float tm = max(0, delta_x/vmax - vmax/amax);
    // Piece wise integration
    if (t < tr) {
      q_dotCmd[i] = amax*t;
    } else if (tr <= t & t < tr + tm) {
      q_dotCmd[i] = vmax;
    } else if (tr + tm <= t & t < 2*tr + tm) {
      q_dotCmd[i] = vmax - amax*(t-tr-tm);
    } else {
      q_dotCmd[i] = 0.0;
    }
    if (abs(delta_x) < pure_PID_tol) {
      q_dotCmd[i] = 0.0;
    }
  }
}

float myServoController::ramp_tf(float tf[NUM_MOTORS], float x0[NUM_MOTORS], float xf[NUM_MOTORS], float vmax, float amax) {
  float tf_max = 0;
  float tr;
  float tm;
  float delta_x;
  for (int i=0; i<NUM_MOTORS; i++) {
    delta_x = abs(xf[i] - x0[i]);
    tr = min(delta_x/vmax, vmax/amax);
    tm = max(0, delta_x/vmax - vmax/amax);
    tf[i] = 2*tr + tm;
    tf_max = max(tf_max, tf[i]);
  }
  return tf_max;
}


void myServoController::ramp2pos(float xf[NUM_MOTORS], float vmax, float amax) {
  // Blocking function to continuously drive the robot accross a ramp trajectory
  update_state();
  float x0[NUM_MOTORS];
  float tf[NUM_MOTORS];
  float v[NUM_MOTORS];
  for (int i=0; i<NUM_MOTORS; i++) {
    x0[i] = _joint_positions[i];
  }
  float tf_max = ramp_tf(tf, x0, xf, vmax, amax);
  for (int i=0; i<NUM_MOTORS; i++) {
    v[i] = vmax;
  }

  float qCmd[NUM_MOTORS];
  float q_dotCmd[NUM_MOTORS];

  _traj_t0 = millis()/1000.0;
  _traj_tf = tf_max;
  _traj_t = 0;


  while (_traj_t < _traj_tf) {
    _traj_t = (millis()/1000.0) - _traj_t0;
    
    ramp_pos(_traj_t, qCmd, x0, xf, v, amax);
    ramp_vel(_traj_t, q_dotCmd, x0, xf, v, amax);

    
    cmdPos(qCmd);
    cmdVel(q_dotCmd);
    driveController();

    //Serial.print(_joint_positions[0]); Serial.print("\t");
    //Serial.print(_joint_positions[1]); Serial.print("\t");
    //Serial.println(_joint_positions[2]);

    
    delay(50);
  }
}
