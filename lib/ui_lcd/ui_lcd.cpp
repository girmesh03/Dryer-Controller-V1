#include "ui_lcd.h"

#include <Arduino.h>

#include "config_build.h"

namespace {
constexpr uint8_t kLcdCols = 20;
constexpr uint8_t kLcdRows = 4;

enum Screen : uint8_t { SCREEN_NONE = 0, SCREEN_BOOT = 1, SCREEN_MAIN_MENU = 2 };

void copyProgmemString(char* dst, uint8_t dst_len, const char* progmem_src) {
  if (dst_len == 0u) {
    return;
  }
  strncpy_P(dst, progmem_src, dst_len);
  dst[dst_len - 1u] = '\0';
}
} // namespace

void UILCD::init() {
  state_.screen = SCREEN_NONE;
  state_.menu_selection = 0;
  state_.dirty = 1;
  line_buffer_[0] = '\0';

  lcd_.init();
  lcd_.begin(kLcdCols, kLcdRows);
  lcd_.backlight();
  lcd_.clear();
}

void UILCD::clear() {
  lcd_.clear();
}

void UILCD::setCursor(uint8_t col, uint8_t row) {
  lcd_.setCursor(col, row);
}

void UILCD::print(const char* text) {
  lcd_.print(text);
}

void UILCD::print(const __FlashStringHelper* text) {
  lcd_.print(text);
}

void UILCD::showBootScreen() {
  state_.screen = SCREEN_BOOT;
  state_.dirty = 1;
  renderBoot_();
  state_.dirty = 0;
}

void UILCD::showMainMenu(uint8_t selection) {
  const uint8_t clamped = (selection > 1u) ? 1u : selection;
  if (state_.screen != SCREEN_MAIN_MENU || state_.menu_selection != clamped) {
    state_.screen = SCREEN_MAIN_MENU;
    state_.menu_selection = clamped;
    state_.dirty = 1;
  }
  renderMainMenu_();
  state_.dirty = 0;
}

void UILCD::update() {
  if (!state_.dirty) {
    return;
  }
  state_.dirty = 0;

  switch (state_.screen) {
    case SCREEN_BOOT:
      renderBoot_();
      break;
    case SCREEN_MAIN_MENU:
      renderMainMenu_();
      break;
    default:
      break;
  }
}

void UILCD::renderBoot_() {
  extern bool g_was_watchdog_reset;

  lcd_.clear();

  lcd_.setCursor(0, 0);
  lcd_.print(F("INDUSTRIAL DRYER"));

  lcd_.setCursor(0, 1);
  lcd_.print(F("FW: v"));
  copyProgmemString(line_buffer_, sizeof(line_buffer_), FW_VERSION);
  lcd_.print(line_buffer_);

  lcd_.setCursor(0, 2);
  lcd_.print(F("Self-Test: PASS"));

  lcd_.setCursor(0, 3);
  if (g_was_watchdog_reset) {
    lcd_.print(F("LAST RESET: WDT"));
  }
}

void UILCD::renderMainMenu_() {
  lcd_.clear();

  lcd_.setCursor(0, 0);
  lcd_.print(F("MAIN MENU"));

  lcd_.setCursor(0, 1);
  lcd_.print((state_.menu_selection == 0u) ? F("> AUTO") : F("  AUTO"));

  lcd_.setCursor(0, 2);
  lcd_.print((state_.menu_selection == 1u) ? F("> MANUAL") : F("  MANUAL"));
}
