#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include "myServoController.h"

//################################## Feather MC and ADC Libraries INIT #####################3
Adafruit_MotorShield MC0 = Adafruit_MotorShield();
//
Adafruit_DCMotor *M1 = MC0.getMotor(1);
Adafruit_DCMotor *M2 = MC0.getMotor(2);
Adafruit_DCMotor *M3 = MC0.getMotor(3);
Adafruit_DCMotor *M4 = MC0.getMotor(4);
//
Adafruit_DCMotor* motors[NUM_MOTORS] = {M1, M2, M3, M4};

Adafruit_ADS1015 ADC0;

Adafruit_ADS1015* adcs[NUM_MOTORS] = {&ADC0, &ADC0, &ADC0, &ADC0};
int channels[NUM_MOTORS] = {0, 1, 3, 2};


// Motor Config: {Long Plate, Short Plate, Plunger, NONE}
float plunger_min = 1.0;
float plunger_max = 8.5;
float plate_min = 1.5;
float plate_max = 29.5;
float joint_lengths[NUM_MOTORS] = {50, 50, 10, 0};
float joint_limits[NUM_MOTORS][2] = {{plate_min, plate_max}, {plate_min, plate_max}, {plunger_min, plunger_max}, {0, 0}};
myServoController controller(motors, adcs, channels, joint_lengths, joint_limits);

// Commands
int brick_height = 9.6; //mm
int brick_loc0 = 3.0; // mm
int brick_locs[] = {brick_loc0, 
                    brick_loc0 + brick_height,
                    brick_loc0 + 2*brick_height,
                    brick_loc0 + 3*brick_height};

// Communication
char TX_BUF[100] = {'\0'};
char RX_BUF[100] = {'\0'};

void setup() {
  Serial.begin(115200);
  while (!Serial) {delay(10);} //
  Serial1.begin(115200);
  while (!Serial1) {delay(10);} //Hang until ESP01 connected

  //wifi_init();

  // set all the base dc motor control pins to outputs
  MC0.begin();
  // start all the ADCs
  ADC0.begin(0x48);
  ADC0.setGain(GAIN_ONE);
  
  controller.init_controller();
  //startTimer(20); //20 Hz -> 50ms update freq  
}

float vmax = 25.0; //mm/s
float amax = 25.0; //mm/s2

long m_prev = millis();
long tic = millis();
bool state = 0;
bool plot = false;

void loop() {
  if (Serial.available()) {
    Serial.readBytesUntil('\n', RX_BUF, 100);
    int r = parse_cmd(RX_BUF);
    if (r == -1) {
      Serial.println("Error Parsing CMD");
    }
  }
  controller.driveController();
  //controller.print_state();
  if (plot) {
    controller.plot_state();
  }
  delay(10);
}



int parse_cmd(char buf[]) {
  char cmd_word = buf[0];
  switch(cmd_word) {
    case 'H': {// HELP
      Serial.println("-----Command Bank------");
      Serial.println("H: Help");
      Serial.println("G q1 q2 q3: GOTO <long_plate, short_plate, plunger> in mm");
      Serial.println("B plate brick: BRICK GOTO <0:long_plate, 1:short_plate>, base of brick");
      Serial.println("P: Toggle ON/OFF Serial Plotting");
      Serial.println("D: DEPOSIT Brick");
      Serial.println("R: RAISE Plunger");
      break;
    }
    case 'G': {//GOTO
      //"G q1 q2 q3"
      int q1; //Long Plate
      int q2; //Short Plate
      int q3; //Plunger
      int q4 = 0; //None
      sscanf(buf, "%c %d %d %d", &cmd_word, &q1, &q2, &q3);
      float qCmd[4] = {(float)q1, (float)q2, (float)q3, (float)q4};
      float q_dotCmd[4] = {0, 0, 0, 0};
      controller.cmdPos(qCmd);
      controller.cmdVel(q_dotCmd);
      break;
    }
    case 'P': { // PLOT
      plot = !plot;
      break;
    }
    case 'B': {// BRICK GOTO
      //"B plate brick" 
      // Where plate: {0: long plate, 1: short plate} direct mapping to motor indeces
      // Brick: {0, .., 3} brick_locs[brick] is location of bottom of brick in mm
      int plate;
      int brick;
      sscanf(buf, "%c %d %d", &cmd_word, &plate, &brick);
      if (brick  < 0 || brick > 3 || plate < 0 || plate > 1) {
        Serial.println("Brick: <0,1,2,3>");
        Serial.println("Plate: <0:long, 1:short>");
      }
      brick = min(max(brick, 0), 3); // clip to safe bounds
      plate = min(max(plate, 0), 1); // clip to safe bounds
      float qCmd = brick_locs[brick];
      controller.cmdPosSingle(qCmd, plate);
      break;
    }
    case 'D': {// DEPOSIT
      controller.cmdPosSingle(8.0, 2);
      break;
    }
    case 'R': {// RAISE
      controller.cmdPosSingle(2.0, 2);
      break;
    }
    default: {//Unspecified Command Word
      return -1;
    }
  }
  return 0;
}
