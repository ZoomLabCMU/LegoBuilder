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
}

void update_UI(BrickPick &brickpick) {
  double left_plate_pos = brickpick.get_short_plate_pos();
  double right_plate_pos = brickpick.get_long_plate_pos();
  //draw_payload();
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

void draw_left_plate(double left_plate_pos) {
  int x0 = 10;
  int PLATE_WIDTH = 30;
  int y0 = 0;
  int BRICK_HEIGHT = 30;
  Serial.println();
  Serial.println(left_plate_pos);
  tft.fillRect(x0, y0, PLATE_WIDTH, BRICK_HEIGHT * left_plate_pos, ST77XX_RED);
  tft.fillRect(x0 + PLATE_WIDTH/2, y0, PLATE_WIDTH/2, max(0.0, BRICK_HEIGHT * left_plate_pos - 10), ST77XX_BLACK);
  tft.fillRect(x0, y0 + BRICK_HEIGHT * left_plate_pos, PLATE_WIDTH, BRICK_HEIGHT * (5 - left_plate_pos), ST77XX_BLACK);
}

void draw_right_plate(double right_plate_pos) {
  int x0 = 200;
  int PLATE_WIDTH = 30;
  int y0 = 0;
  int BRICK_HEIGHT = 30;
  tft.fillRect(x0, y0, PLATE_WIDTH, BRICK_HEIGHT * right_plate_pos, ST77XX_RED);
  tft.fillRect(x0, y0 + BRICK_HEIGHT * right_plate_pos, PLATE_WIDTH, BRICK_HEIGHT * (5 - right_plate_pos), ST77XX_BLACK);
}