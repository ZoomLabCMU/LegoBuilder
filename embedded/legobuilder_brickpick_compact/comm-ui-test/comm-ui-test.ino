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

BrickPick brickpick;

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

  Serial.println(F("Initialized"));
}

void loop(){
  // View client requests for commands
  int return_code = handle_clients(brickpick);
  // Execute commands
  if (return_code == 0) {
    update_UI(brickpick);
  }
}

