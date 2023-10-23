/*
User Interface (on built-in TFT screen) for BrickPick
*/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include "BrickPick.h"

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

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
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);
  draw_payload();
  // Left is short, Right is long (L I know...)
  tft.drawChar(10, 160, 'S', ST77XX_CYAN, ST77XX_BLACK, 5);
  tft.drawChar(95, 160, 'L', ST77XX_CYAN, ST77XX_BLACK, 5);
}

void update_UI(BrickPick &brickpick) {
  double MM2PIXEL = 30 / 9.6;
  // Safe casting?
  int left_plate_pos = brickpick.get_short_plate_pos_mm() * MM2PIXEL;
  int right_plate_pos = brickpick.get_long_plate_pos_mm() * MM2PIXEL;
  //draw_payload(); Not necessary (yet)
  draw_left_plate(left_plate_pos);
  draw_right_plate(right_plate_pos);
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
  int BRICK_HEIGHT = 30;
  left_plate_pos = min(max(left_plate_pos, 0), 5 * BRICK_HEIGHT); // Do not fall below brick stack
  int x0 = 10;
  int PLATE_WIDTH = 30;
  int y0 = 0;
  tft.fillRect(x0, y0, PLATE_WIDTH, left_plate_pos, ST77XX_RED);
  tft.fillRect(x0 + PLATE_WIDTH/2, y0, PLATE_WIDTH/2, max(0,left_plate_pos - 10), ST77XX_BLACK);
  tft.fillRect(x0, y0 + BRICK_HEIGHT * left_plate_pos, PLATE_WIDTH, BRICK_HEIGHT * 5 - left_plate_pos, ST77XX_BLACK);
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
  int x0 = 95;
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