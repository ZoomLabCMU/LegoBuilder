/*
User Interface (on built-in TFT screen) for BrickPick
*/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include "BrickPick.h"

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void show_WiFi_info(String ssid, String password) {
  // display the SSID and password on the TFT display
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0,0);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.println("Connecting...");
  tft.println();
  tft.println("   " + ssid);
  tft.println();
  tft.print("   " + String(password[0]));
  for (int i = 1; i < password.length() - 1; i++) {
    tft.print("*");
  }
  tft.println(password[password.length() - 1]);
}

void init_UI() {
  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // initialize TFT
  tft.init(135, 240); // Init ST7789 240x135
}

void show_main_display() {
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);
  draw_payload();
  // Left is short, Right is long (L I know...)
  tft.drawChar(15, 160, 'S', ST77XX_CYAN, ST77XX_BLACK, 3);
  tft.drawChar(110, 160, 'L', ST77XX_CYAN, ST77XX_BLACK, 3);
  tft.setCursor(10, 230);
  tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.print(get_ip_string());
}

void update_UI(BrickPick &brickpick) {
  double MM2PIXEL = 30 / 9.6;
  // Safe casting?
  int left_plate_pos = brickpick.get_short_plate_pos_mm() * MM2PIXEL;
  int right_plate_pos = brickpick.get_long_plate_pos_mm() * MM2PIXEL;
  int left_target_pos = brickpick.get_short_plate_target_mm() * MM2PIXEL;
  int right_target_pos = brickpick.get_long_plate_target_mm() * MM2PIXEL;
  //draw_payload(); Not necessary (yet)
  draw_left_plate(left_plate_pos);
  draw_right_plate(right_plate_pos);
  draw_left_target(left_target_pos);
  draw_right_target(right_target_pos);
}

void draw_payload() {
  int WIDTH = 135;
  int HEIGHT = 240;
  int BRICK_HEIGHT = 30;
  int BRICK_WIDTH = 50;
  int x0 = WIDTH/2 - BRICK_WIDTH/2;
  int y0 = 0;
  tft.fillRect(x0, y0, BRICK_WIDTH, BRICK_HEIGHT, ST77XX_RED);
  tft.fillRect(x0, y0 + BRICK_HEIGHT, BRICK_WIDTH, BRICK_HEIGHT, ST77XX_MAGENTA);
  tft.fillRect(x0, y0 + 2*BRICK_HEIGHT, BRICK_WIDTH, BRICK_HEIGHT, ST77XX_YELLOW);
  tft.fillRect(x0, y0 + 3*BRICK_HEIGHT, BRICK_WIDTH, BRICK_HEIGHT, ST77XX_GREEN);
  tft.fillRect(x0, y0 + 4*BRICK_HEIGHT, BRICK_WIDTH, BRICK_HEIGHT, ST77XX_ORANGE);
}

void draw_left_plate(int left_plate_pos) {
  // left_plate_pos is finger tip center
  static int left_plate_pos_prev = 0;

  int BRICK_HEIGHT = 30;
  int PLATE_WIDTH = 15;
  int FINGER_WIDTH = 15;
  int FINGER_RAD = 9;
  int Y_LIM = BRICK_HEIGHT * 5 + FINGER_RAD;

  left_plate_pos += FINGER_RAD; // FOR EASE OF DRAWING
  left_plate_pos = min(max(left_plate_pos, 0), Y_LIM);
  int x0 = 5;
  int y0 = 0;

  if (left_plate_pos_prev == left_plate_pos) {
    return;
  } else if (left_plate_pos_prev < left_plate_pos) {
    // Extend plate
    // Erase previous finger zone
    int buffer = 15;
    tft.fillRect(x0 + PLATE_WIDTH, left_plate_pos_prev - (2*FINGER_RAD-1) - 1 - buffer, FINGER_WIDTH, (2*FINGER_RAD-1) + buffer, ST77XX_BLACK);
    // Draw new plate to depth
    tft.fillRect(x0, y0 + left_plate_pos_prev, PLATE_WIDTH, left_plate_pos - left_plate_pos_prev, ST77XX_RED);
    // Draw new finger 
    tft.fillRect(x0 + PLATE_WIDTH + FINGER_WIDTH - FINGER_RAD, left_plate_pos - (2*FINGER_RAD-1) - 2, FINGER_WIDTH-FINGER_RAD, 2*FINGER_RAD + 1, ST77XX_RED);
    tft.fillCircle(x0 + PLATE_WIDTH + FINGER_WIDTH - FINGER_RAD, left_plate_pos - FINGER_RAD - 1, FINGER_RAD, ST77XX_RED);
  } else {
    // Retract plate
    // Erase previous finger zone
    int buffer = 15;
    tft.fillRect(x0 + PLATE_WIDTH, left_plate_pos_prev - (2*FINGER_RAD-1) + 1, FINGER_WIDTH, (2*FINGER_RAD-1) + buffer, ST77XX_BLACK);
    // Erase previous plate to depth
    tft.fillRect(x0, y0 + left_plate_pos, PLATE_WIDTH, left_plate_pos_prev - left_plate_pos, ST77XX_BLACK);
    // Draw new finger
    tft.fillRect(x0 + PLATE_WIDTH + FINGER_WIDTH - FINGER_RAD, left_plate_pos - (2*FINGER_RAD-1) - 2, FINGER_WIDTH-FINGER_RAD, 2*FINGER_RAD + 1, ST77XX_RED);
    tft.fillCircle(x0 + PLATE_WIDTH + FINGER_WIDTH - FINGER_RAD, left_plate_pos - FINGER_RAD - 1, FINGER_RAD, ST77XX_RED);
  }
  left_plate_pos_prev = left_plate_pos;
}

void draw_right_plate(int right_plate_pos) {
  // right_plate_pos is finger tip center
  static int right_plate_pos_prev = 0;

  int BRICK_HEIGHT = 30;
  int PLATE_WIDTH = 15;
  int FINGER_WIDTH = 15;
  int FINGER_RAD = 9;
  int Y_LIM = BRICK_HEIGHT * 5 + FINGER_RAD;

  right_plate_pos += FINGER_RAD; // FOR EASE OF DRAWING
  right_plate_pos = min(max(right_plate_pos, 0), Y_LIM);
  // Top-Right of bounding box
  // (x0, y0)----|
  // |       |   |
  // |       |   |
  // | [=====|___| __right_plate_pos
  int x0 = 100;
  int y0 = 0;

  if (right_plate_pos_prev == right_plate_pos) {
    return;
  } else if (right_plate_pos_prev < right_plate_pos) {
    // Extend plate
    // Erase previous finger zone
    int buffer = 15;
    tft.fillRect(x0, right_plate_pos_prev - (2*FINGER_RAD-1) - 1 - buffer, FINGER_WIDTH, (2*FINGER_RAD-1) + buffer, ST77XX_BLACK);
    // Draw new plate to depth
    tft.fillRect(x0 + FINGER_WIDTH, y0 + right_plate_pos_prev, PLATE_WIDTH, right_plate_pos - right_plate_pos_prev, ST77XX_RED);
    // Draw new finger 
    tft.fillRect(x0 + FINGER_RAD, right_plate_pos - (2*FINGER_RAD-1) - 2, FINGER_WIDTH-FINGER_RAD, 2*FINGER_RAD + 1, ST77XX_RED);
    tft.fillCircle(x0 + FINGER_RAD, right_plate_pos - FINGER_RAD - 1, FINGER_RAD, ST77XX_RED);
  } else {
    // Retract plate
    // Erase previous finger zone
    int buffer = 15;
    tft.fillRect(x0, right_plate_pos_prev - (2*FINGER_RAD-1) + 1, FINGER_WIDTH, (2*FINGER_RAD-1) + buffer, ST77XX_BLACK);
    // Erase previous plate to depth
    tft.fillRect(x0 + FINGER_WIDTH, y0 + right_plate_pos, PLATE_WIDTH, right_plate_pos_prev - right_plate_pos, ST77XX_BLACK);
    // Draw new finger
    tft.fillRect(x0 + FINGER_RAD, right_plate_pos - (2*FINGER_RAD-1) - 2, FINGER_WIDTH-FINGER_RAD, 2*FINGER_RAD + 1, ST77XX_RED);
    tft.fillCircle(x0 + FINGER_RAD, right_plate_pos - FINGER_RAD - 1, FINGER_RAD, ST77XX_RED);
  }
  right_plate_pos_prev = right_plate_pos;
}

void draw_left_target(int left_target_pos) {
  static int left_target_pos_prev = 0;
  int BRICK_HEIGHT = 30;
  int TARGET_HEIGHT = 1;
  int TARGET_WIDTH = 4;
  int Y_LIM = BRICK_HEIGHT * 5;
  int x0 = 35;
  int y0 = 0;

  if (left_target_pos_prev == left_target_pos) {
    return;
  } else {
    // Erase previous target zone
    int buffer = 1;
    tft.fillRect(x0, left_target_pos_prev - TARGET_HEIGHT - buffer, TARGET_WIDTH, TARGET_HEIGHT + 2*buffer, ST77XX_BLACK);
    // Draw new target to depth
    tft.fillRect(x0 , left_target_pos - TARGET_HEIGHT, TARGET_WIDTH, TARGET_HEIGHT, ST77XX_GREEN);
  }
  left_target_pos_prev = left_target_pos;
}

void draw_right_target(int right_target_pos) {
  static int right_target_pos_prev = 0;
  int BRICK_HEIGHT = 30;
  int TARGET_HEIGHT = 1;
  int TARGET_WIDTH = 4;
  int Y_LIM = BRICK_HEIGHT * 5;
  int x0 = 95;
  int y0 = 0;

  if (right_target_pos_prev == right_target_pos) {
    return;
  } else {
    // Erase previous target zone
    int buffer = 1;
    tft.fillRect(x0, right_target_pos_prev - TARGET_HEIGHT - buffer, TARGET_WIDTH, TARGET_HEIGHT + 2*buffer, ST77XX_BLACK);
    // Draw new target to depth
    tft.fillRect(x0 , right_target_pos - TARGET_HEIGHT, TARGET_WIDTH, TARGET_HEIGHT, ST77XX_GREEN);
  }
  right_target_pos_prev = right_target_pos;
}