#ifndef UI_LCD_H
#define UI_LCD_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#include "config_build.h"

class UILCD {
public:
  void init();
  void update(); // Call at >= 5 Hz

  void showBootScreen();
  void showMainMenu(uint8_t selection);

  void clear();
  void setCursor(uint8_t col, uint8_t row);
  void print(const char* text);
  void print(const __FlashStringHelper* text);

private:
  LiquidCrystal_I2C lcd_{LCD_I2C_ADDR, 20, 4};
  char line_buffer_[21];

  struct {
    uint8_t screen;
    uint8_t menu_selection;
    uint8_t dirty : 1;
  } state_;

  void renderBoot_();
  void renderMainMenu_();
};

#endif
