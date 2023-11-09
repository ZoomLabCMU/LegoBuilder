/*
 BrickPick main firmware loop

 BrickPick.cpp/.h handles onboard controls
 Pins.h is a common file for the pinout
 UI.ino handles the updating of the display
 comms.ino handles the WiFi server and client requests

 This is written for a network using WPA2 encryption. For insecure
 WEP or WPA, change the Wifi.begin() call and use Wifi.setMinSecurity() accordingly.

 ST77XX_COLOR specifies TFT color macros

 */
#include <Adafruit_MotorShield.h>
#include <arduino-timer.h>

#include "BrickPick.h"
#include "Pins.h"
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *short_motor = AFMS.getMotor(4);
Adafruit_DCMotor *long_motor = AFMS.getMotor(3);
Adafruit_DCMotor *plunger_motor = AFMS.getMotor(2);

// Create brickpick class object
BrickPick brickpick = BrickPick(short_motor, long_motor, plunger_motor);

// Instantiate timer for main loop callbacks
Timer<3, millis, BrickPick*> update_timer;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== BrickPick Initializing ===");

  // Initialize the UI (TFT display)
  init_UI();

  // We start by connecting to a WiFi network
  const char* ssid     = "CMU-DEVICE"; //"128NCraig617";
  const char* password = NULL; //"617617617";

  // Display login info on TFT to verify
  show_WiFi_info(String(ssid), "NULL");
  delay(5000);

  // Create WiFi server
  init_server(ssid, password);

  // Now that WiFi is established switch to main display
  show_main_display();

  // Setup encoders
  attachInterrupt(digitalPinToInterrupt(LONG_ENC_A), update_long_count, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SHORT_ENC_A), update_short_count, CHANGE);
  pinMode(LONG_ENC_A, INPUT);
  pinMode(LONG_ENC_B, INPUT);
  pinMode(SHORT_ENC_A, INPUT);
  pinMode(SHORT_ENC_B, INPUT);

  // Add callbacks to timer
  // millis
  update_timer.every(100, update_UI_callback, &brickpick);
  update_timer.every(10, plotting_callback, &brickpick);
  update_timer.every(1, update_brickpick_callback, &brickpick);

  // Init Motors
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while(1);
  }
  Serial.println(F("=== Initialized ==="));
}

void loop(){
  // Set new commands and reply when command complete
  // This is a non-blocking function that uses persistent variables to track
  //  client requests and will not accept new or close the current client before
  //  command termination (complete or fail)
  int return_code = handle_clients(brickpick);

  // Progress timer
  update_timer.tick();
}

// Call the brickpick class update function
//  this handles the PID updates
bool update_brickpick_callback(BrickPick *brickpick) {
  brickpick->update();
  return true;
}


// Update the UI (TFT display) according to the estimated state in the brickpick object
bool update_UI_callback(BrickPick *brickpick) {
  // Timer wants pointers to be passed as input
  update_UI(*brickpick);
  //tmp_brick = (tmp_brick + 1) % 5;
  //brickpick->set_long_target_brick(tmp_brick);
  return true;
}

// Function to enable serial plotting for PID tuning and debugging
bool plotting_callback(BrickPick *brickpick) {
  return true;
  double x = brickpick->get_long_plate_pos_mm();
  double target = brickpick->get_long_plate_target_mm();
  long u_l = brickpick->get_long_plate_ctrl();

  Serial.print("x:");
  Serial.print(x);
  Serial.print(",");
  Serial.print("target:");
  Serial.print(target);
  Serial.println();
  return true;
}

void update_long_count() {
  if (digitalRead(LONG_ENC_A) == HIGH) {
    if (digitalRead(LONG_ENC_B) == LOW)
      brickpick.long_plate_pos_incr();
    else
      brickpick.long_plate_pos_decr();
  } else {
    if (digitalRead(LONG_ENC_B) == LOW)
      brickpick.long_plate_pos_decr();
    else
      brickpick.long_plate_pos_incr();
  }
}

// Encoder interrupts are attached in the main loop and call increment and decrement functions
//  to update the measured state of the worm motors in the brickpick object. 
// TODO - Figure out how to implement this internally to the brickpick object
// NOTE - I put this outside because it is complicated to attach interrupts within class initializers.
//        This will require some sort of static implementation and is not really necessary

void update_short_count() {
  if (digitalRead(SHORT_ENC_A) == HIGH) {
    if (digitalRead(SHORT_ENC_B) == LOW)
      brickpick.short_plate_pos_incr();
    else
      brickpick.short_plate_pos_decr();
  } else {
    if (digitalRead(SHORT_ENC_B) == LOW)
      brickpick.short_plate_pos_decr();
    else
      brickpick.short_plate_pos_incr();
  }
}
