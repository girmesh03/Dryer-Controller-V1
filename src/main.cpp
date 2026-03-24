#include <Arduino.h>

#include <avr/io.h>
#include <avr/wdt.h>

#include "config_build.h"
#include "config_pins.h"
#include "eeprom_store.h"
#include "io_abstraction.h"
#include "ui_lcd.h"
#include "keypad_input.h"
#include "app.h"

#ifndef ENABLE_SERIAL_DEBUG
#define ENABLE_SERIAL_DEBUG 0
#endif

static IOAbstraction io;
static EEPROMStore eepromStore;

uint8_t g_reset_flags_mcusr = 0;
bool g_was_watchdog_reset = false;

UILCD lcd;
KeypadInput keypad;
AppStateMachine app;

namespace {
#if ENABLE_SERIAL_DEBUG
void printResetFlags(uint8_t flags) {
  Serial.print(F("RESET FLAGS (MCUSR)=0x"));
  Serial.println(flags, HEX);

  if (flags == 0) {
    Serial.println(F("Reset cause: POWER-ON (assumed)"));
    return;
  }

  if (flags & _BV(PORF)) {
    Serial.println(F("Reset cause: POWER-ON"));
  }
  if (flags & _BV(EXTRF)) {
    Serial.println(F("Reset cause: EXTERNAL"));
  }
  if (flags & _BV(BORF)) {
    Serial.println(F("Reset cause: BROWN-OUT"));
  }
  if (flags & _BV(WDRF)) {
    Serial.println(F("Reset cause: WATCHDOG"));
  }
}
#endif
} // namespace

void setup() {
  // Capture reset cause before clearing, and disable watchdog to avoid reset loops.
  g_reset_flags_mcusr = MCUSR;
  g_was_watchdog_reset = (g_reset_flags_mcusr & _BV(WDRF)) != 0;
  MCUSR = 0;
  wdt_disable();

  eepromStore.init();

  io.init();

  lcd.init();
  keypad.init();
  app.init();

#if ENABLE_SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("Industrial Dryer Controller - Phase 2"));
  printResetFlags(g_reset_flags_mcusr);

  Serial.print(F("EEPROM CRC valid: "));
  Serial.println(eepromStore.isValid() ? F("YES") : F("NO"));
  Serial.println(F("IO initialized: outputs forced OFF, door input pull-up enabled"));
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Enable watchdog after initialization completes.
  wdt_enable(WDTO_2S);
}

void loop() {
  // Reset watchdog - must be called every loop iteration
  wdt_reset();
  const uint32_t now = millis();

  static uint32_t last_keypad_ms = 0;
  static uint32_t last_io_ms = 0;
  static uint32_t last_app_ms = 0;
  static uint32_t last_lcd_ms = 0;
  static uint32_t last_slow_ms = 0;
  static bool last_door_closed = true;

  // Keypad scan loop: 20 Hz (50ms)
  if (now - last_keypad_ms >= 50u) {
    last_keypad_ms = now;
    keypad.update();

    const auto key = keypad.getKey();
    if (key != KeypadInput::Key::NONE) {
      app.handleKeyPress(key);
    }
  }

  // IO loop: 10 Hz (100ms)
  if (now - last_io_ms >= FAST_LOOP_PERIOD) {
    last_io_ms = now;
    io.update();

    // Door state change reporting (useful for Phase 1 bench verification).
    const bool door_closed = io.isDoorClosed();
    if (door_closed != last_door_closed) {
      last_door_closed = door_closed;
#if ENABLE_SERIAL_DEBUG
      Serial.println(door_closed ? F("DOOR: CLOSED") : F("DOOR: OPEN"));
#endif
    }
  }

  // App state machine: 10 Hz (100ms)
  if (now - last_app_ms >= FAST_LOOP_PERIOD) {
    last_app_ms = now;
    app.update();
  }

  // LCD refresh: >= 5 Hz (200ms)
  if (now - last_lcd_ms >= 200u) {
    last_lcd_ms = now;
    lcd.update();
  }

  // Slow tick: EEPROM deferred writes, diagnostics, heartbeat (1 Hz).
  if (now - last_slow_ms >= SLOW_LOOP_PERIOD) {
    last_slow_ms = now;
    eepromStore.update(now);

    // Heartbeat LED (visible progress on bench).
    digitalWrite(LED_BUILTIN, (digitalRead(LED_BUILTIN) == LOW) ? HIGH : LOW);
  }
}
