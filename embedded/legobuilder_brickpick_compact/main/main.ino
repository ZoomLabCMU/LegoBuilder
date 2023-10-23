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
Adafruit_DCMotor *short_motor = AFMS.getMotor(3);
Adafruit_DCMotor *long_motor = AFMS.getMotor(4);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);
Encoder short_encoder(SHORT_ENC_A, SHORT_ENC_B);
Encoder long_encoder(LONG_ENC_A, LONG_ENC_B);


BrickPick brickpick = BrickPick(short_motor, long_motor, &short_encoder, &long_encoder);

Timer<1, millis, BrickPick*> UI_timer;

void setup()
{
  Serial.begin(115200);
  delay(10);
  Serial.println("=== Basic Comms and UI test ===");

  // We start by connecting to a WiFi network
  const char* ssid     = "128NCraig617";
  const char* password = "617617617";
  init_server(ssid, password);
  
  // Initialize the UI (TFT display)
  init_UI();
  // millis
  UI_timer.every(50, UI_timer_callback, &brickpick);

  // Init Motors
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while(1);
  }

  Serial.println(F("Initialized"));
}

double enc = 0;
double enc_prev = 0;
void loop(){
  // View client requests for commands
  int return_code = handle_clients(brickpick);
  /*
  request = parse_client_requests();
  if (request != NULL) {
    return_code = brickpick->set_command(request);
  }
  */
  brickpick.update();
  enc_prev = enc;
  enc = brickpick.get_long_plate_pos_mm();
  if (enc != enc_prev) {
    Serial.println(enc);
  }

  UI_timer.tick();
}

bool UI_timer_callback(BrickPick *brickpick) {
  // Timer wants pointers to be passed as input
  update_UI(*brickpick);
  return true;
}

