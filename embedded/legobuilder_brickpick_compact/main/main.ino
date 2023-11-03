/*
 WiFi Web Server + BrickPick UI

 If the IP address of your shield is yourAddress:
 http://yourAddress/Up  moves the left plate indicator up
 http://yourAddress/Down moves the left plate indicator down

 This example is written for a network using WPA2 encryption. For insecure
 WEP or WPA, change the Wifi.begin() call and use Wifi.setMinSecurity() accordingly.

 ST77XX_COLOR specifies TFT color macros

 */
#include "BrickPick.h"
#include <Adafruit_MotorShield.h>
#include <arduino-timer.h>

#include <Encoder.h>

#include "Pins.h"
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *short_motor = AFMS.getMotor(4);
Adafruit_DCMotor *long_motor = AFMS.getMotor(3);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);
Encoder short_encoder(SHORT_ENC_A, SHORT_ENC_B);
Encoder long_encoder(LONG_ENC_A, LONG_ENC_B);

BrickPick brickpick = BrickPick(short_motor, long_motor, &short_encoder, &long_encoder);

Timer<3, millis, BrickPick*> update_timer;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Basic Comms and UI test ===");

  // We start by connecting to a WiFi network
  const char* ssid     = "128NCraig617";
  const char* password = "617617617";
  init_server(ssid, password);
  
  // Initialize the UI (TFT display)
  init_UI();

  // Long state est
  attachInterrupt(digitalPinToInterrupt(LONG_ENC_A), update_long_count, CHANGE);
  // encoder reading pins
  pinMode(LONG_ENC_A, INPUT);
  pinMode(LONG_ENC_B, INPUT);

  // millis
  update_timer.every(100, update_UI_callback, &brickpick);
  update_timer.every(100, plotting_callback, &brickpick);
  update_timer.every(1, update_brickpick_callback, &brickpick);
  // Init Motors
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while(1);
  }
  Serial.println(F("Initialized"));
}

size_t tmp_brick = 0;
void loop(){
  // View client requests for commands
  int return_code = handle_clients(brickpick);
  /*
  request = parse_client_requests();
  if (request != NULL) {
    return_code = brickpick->set_command(request);
  }
  */
  update_timer.tick();
}

bool update_brickpick_callback(BrickPick *brickpick) {
  brickpick->update();
  return true;
}

bool update_UI_callback(BrickPick *brickpick) {
  // Timer wants pointers to be passed as input
  update_UI(*brickpick);
  //tmp_brick = (tmp_brick + 1) % 5;
  //brickpick->set_long_target_brick(tmp_brick);
  return true;
}

bool plotting_callback(BrickPick *brickpick) {
  double x = brickpick->get_long_plate_pos_mm();
  double target = brickpick->get_long_plate_target_mm();
  long u_l = brickpick->get_long_plate_ctrl();

  Serial.print("x:");
  Serial.print(x);
  Serial.print(",");
  Serial.print("target:");
  Serial.print(target);
  Serial.print(",");
  Serial.print("u_l:");
  Serial.print(u_l);
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

