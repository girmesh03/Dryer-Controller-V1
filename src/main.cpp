#include <Arduino.h>

#include <avr/io.h>
#include <avr/wdt.h>

#include "config_build.h"
#include "config_pins.h"
#include "eeprom_store.h"
#include "io_abstraction.h"

static IOAbstraction io;
static EEPROMStore eepromStore;

static uint8_t reset_flags_mcusr = 0;

namespace {
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
} // namespace

void setup() {
  // Capture reset cause before clearing, and disable watchdog to avoid reset loops.
  reset_flags_mcusr = MCUSR;
  MCUSR = 0;
  wdt_disable();

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("Industrial Dryer Controller - Phase 1"));
  printResetFlags(reset_flags_mcusr);

  eepromStore.init();
  Serial.print(F("EEPROM CRC valid: "));
  Serial.println(eepromStore.isValid() ? F("YES") : F("NO"));

  io.init();
  Serial.println(F("IO initialized: outputs forced OFF, door input pull-up enabled"));

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Enable watchdog after initialization completes.
  wdt_enable(WDTO_2S);
}

void loop() {
  // Reset watchdog - must be called every loop iteration
  wdt_reset();
  const uint32_t now = millis();

  static uint32_t last_fast_ms = 0;
  static uint32_t last_medium_ms = 0;
  static uint32_t last_slow_ms = 0;
  static bool last_door_closed = true;

  // Fast loop: IO update (Phase 1 uses door debounce and safety stop).
  if (now - last_fast_ms >= FAST_LOOP_PERIOD) {
    last_fast_ms = now;
    io.update();

    // Door state change reporting (useful for Phase 1 bench verification).
    const bool door_closed = io.isDoorClosed();
    if (door_closed != last_door_closed) {
      last_door_closed = door_closed;
      Serial.println(door_closed ? F("DOOR: CLOSED") : F("DOOR: OPEN"));
    }
  }

  // Medium tick: sensors + UI (4 Hz). Phase 1 reserved for future modules.
  if (now - last_medium_ms >= MEDIUM_LOOP_PERIOD) {
    last_medium_ms = now;
  }

  // Slow tick: EEPROM deferred writes, diagnostics, heartbeat (1 Hz).
  if (now - last_slow_ms >= SLOW_LOOP_PERIOD) {
    last_slow_ms = now;
    eepromStore.update(now);

    // Heartbeat LED (visible progress on bench).
    digitalWrite(LED_BUILTIN, (digitalRead(LED_BUILTIN) == LOW) ? HIGH : LOW);
  }
}
