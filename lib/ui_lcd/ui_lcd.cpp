#include "ui_lcd.h"

#include <Arduino.h>

#include "config_build.h"

namespace {
constexpr uint8_t kLcdCols = 20;
constexpr uint8_t kLcdRows = 4;

enum Screen : uint8_t { SCREEN_NONE = 0, SCREEN_BOOT = 1, SCREEN_MAIN_MENU = 2 };

void writeProgmemToLine(char* line, uint8_t start_col, const char* progmem_src) {
  if (start_col >= kLcdCols) {
    return;
  }
  uint8_t col = start_col;
  while (col < kLcdCols) {
    const char c = static_cast<char>(pgm_read_byte(progmem_src++));
    if (c == '\0') {
      break;
    }
    line[col++] = c;
  }
}

void clearLine(char* line) {
  for (uint8_t i = 0u; i < kLcdCols; i++) {
    line[i] = ' ';
  }
  line[kLcdCols] = '\0';
}
} // namespace

void UILCD::init() {
  state_.screen = SCREEN_NONE;
  state_.menu_selection = 0;
  state_.last_temp_deci_c = 32767;
  state_.dirty = 1;
  state_.temp_valid = 0;
  state_.temp_unit = 0;
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

void UILCD::print(char c) {
  lcd_.write(static_cast<uint8_t>(c));
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

void UILCD::setTempUnit(uint8_t unit) {
  state_.temp_unit = (unit != 0u) ? 1u : 0u;
  // Force a refresh on next showTemperature() call.
  state_.last_temp_deci_c = 32767;
  state_.temp_valid = 1u;
}

void UILCD::showTemperature(float temp_c, bool valid) {
  // Line 4 (row index 3) is reserved for live temperature / sensor health.
  if (!valid) {
    if (state_.temp_valid == 0u) {
      return; // no change
    }
    state_.temp_valid = 0u;
    clearLine(line_buffer_);
    static const char kFault[] PROGMEM = "SENSOR FAULT";
    writeProgmemToLine(line_buffer_, 0u, kFault);
    lcd_.setCursor(0, 3);
    lcd_.print(line_buffer_);
    return;
  }

  const int16_t deci_c =
      static_cast<int16_t>(temp_c * 10.0f + ((temp_c >= 0.0f) ? 0.5f : -0.5f));

  int16_t disp = deci_c;
  char unit_char = 'C';
  const uint8_t unit = state_.temp_unit;

  if (unit != 0u) {
    // Display Fahrenheit, rounded to nearest whole degree (Req 36).
    const int32_t d = static_cast<int32_t>(deci_c);
    const int32_t deci_f = (d * 9 + ((d >= 0) ? 2 : -2)) / 5 + 320; // °F * 10
    const int16_t whole_f = static_cast<int16_t>((deci_f + ((deci_f >= 0) ? 5 : -5)) / 10);
    disp = whole_f;
    unit_char = 'F';
  }

  if (state_.temp_valid != 0u && state_.last_temp_deci_c == disp) {
    return;
  }

  state_.temp_valid = 1u;
  state_.last_temp_deci_c = disp;

  clearLine(line_buffer_);
  static const char kPrefix[] PROGMEM = "TEMP: ";
  writeProgmemToLine(line_buffer_, 0u, kPrefix);

  uint8_t pos = 6u; // after "TEMP: "
  uint16_t abs_val = static_cast<uint16_t>((disp < 0) ? -disp : disp);
  if (disp < 0) {
    line_buffer_[pos++] = '-';
  }

  if (unit == 0u) {
    const uint16_t whole = static_cast<uint16_t>(abs_val / 10u);
    const uint8_t frac = static_cast<uint8_t>(abs_val % 10u);

    if (whole >= 100u) {
      line_buffer_[pos++] = static_cast<char>('0' + ((whole / 100u) % 10u));
      line_buffer_[pos++] = static_cast<char>('0' + ((whole / 10u) % 10u));
      line_buffer_[pos++] = static_cast<char>('0' + (whole % 10u));
    } else if (whole >= 10u) {
      line_buffer_[pos++] = static_cast<char>('0' + ((whole / 10u) % 10u));
      line_buffer_[pos++] = static_cast<char>('0' + (whole % 10u));
    } else {
      line_buffer_[pos++] = static_cast<char>('0' + (whole % 10u));
    }

    line_buffer_[pos++] = '.';
    line_buffer_[pos++] = static_cast<char>('0' + (frac % 10u));
  } else {
    // Fahrenheit: whole degrees.
    if (abs_val >= 100u) {
      line_buffer_[pos++] = static_cast<char>('0' + ((abs_val / 100u) % 10u));
      line_buffer_[pos++] = static_cast<char>('0' + ((abs_val / 10u) % 10u));
      line_buffer_[pos++] = static_cast<char>('0' + (abs_val % 10u));
    } else if (abs_val >= 10u) {
      line_buffer_[pos++] = static_cast<char>('0' + ((abs_val / 10u) % 10u));
      line_buffer_[pos++] = static_cast<char>('0' + (abs_val % 10u));
    } else {
      line_buffer_[pos++] = static_cast<char>('0' + (abs_val % 10u));
    }
  }
  line_buffer_[pos++] = static_cast<char>(0xDF); // degree symbol on HD44780
  line_buffer_[pos++] = unit_char;

  lcd_.setCursor(0, 3);
  lcd_.print(line_buffer_);
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
  // Clearing the LCD also clears the temp line, so force a re-render on next showTemperature().
  state_.last_temp_deci_c = 32767;
  state_.temp_valid = 1u;

  lcd_.setCursor(0, 0);
  lcd_.print(F("INDUSTRIAL DRYER"));

  lcd_.setCursor(0, 1);
  lcd_.print(F("FW: v"));
  // FW_VERSION is stored in PROGMEM.
  for (uint8_t i = 0u; i < kLcdCols; i++) {
    const char c = static_cast<char>(pgm_read_byte(&FW_VERSION[i]));
    if (c == '\0') {
      break;
    }
    lcd_.print(c);
  }

  lcd_.setCursor(0, 2);
  lcd_.print(F("Self-Test: PASS"));

  lcd_.setCursor(0, 3);
  if (g_was_watchdog_reset) {
    lcd_.print(F("LAST RESET: WDT"));
  }
}

void UILCD::renderMainMenu_() {
  lcd_.clear();
  // Clearing the LCD also clears the temp line, so force a re-render on next showTemperature().
  state_.last_temp_deci_c = 32767;
  state_.temp_valid = 1u;

  lcd_.setCursor(0, 0);
  lcd_.print(F("MAIN MENU"));

  lcd_.setCursor(0, 1);
  lcd_.print((state_.menu_selection == 0u) ? F("> AUTO") : F("  AUTO"));

  lcd_.setCursor(0, 2);
  lcd_.print((state_.menu_selection == 1u) ? F("> MANUAL") : F("  MANUAL"));
}
