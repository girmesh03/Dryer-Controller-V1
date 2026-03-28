#include "io_abstraction.h"

#include <Arduino.h>

#include "config_build.h"
#include "config_pins.h"

namespace {
constexpr uint8_t kDoorDebounceCounts =
    (DEBOUNCE_TIME_MS + (FAST_LOOP_PERIOD - 1u)) / FAST_LOOP_PERIOD;

// Relay inputs are active-LOW: LOW=energized, HIGH=de-energized.
constexpr uint8_t kRelayOnLevel = LOW;
constexpr uint8_t kRelayOffLevel = HIGH;
}

void IOAbstraction::init() {
  state_.last_direction_change_ms = 0;
  state_.door_stable_count = 0;
  state_.output_mismatch_count = 0;
  state_.door_state = 0;
  state_.door_raw_state = 0;
  state_.heater_state = 0;
  state_.motor_fwd_state = 0;
  state_.motor_rev_state = 0;
  state_.aux_state = 0;

  // Ensure outputs are forced to safe OFF state before enabling as outputs.
  digitalWrite(PIN_HEATER_RELAY, kRelayOffLevel);
  digitalWrite(PIN_MOTOR_FWD_RELAY, kRelayOffLevel);
  digitalWrite(PIN_MOTOR_REV_RELAY, kRelayOffLevel);
  digitalWrite(PIN_AUX_RELAY, kRelayOffLevel);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(PIN_HEATER_RELAY, OUTPUT);
  pinMode(PIN_MOTOR_FWD_RELAY, OUTPUT);
  pinMode(PIN_MOTOR_REV_RELAY, OUTPUT);
  pinMode(PIN_AUX_RELAY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

#if defined(ESP32)
  // ESP32 GPIO34-39 are input-only and do not support internal pull-ups.
  if (PIN_DOOR_SENSOR >= 34u && PIN_DOOR_SENSOR <= 39u) {
    pinMode(PIN_DOOR_SENSOR, INPUT);
  } else {
    pinMode(PIN_DOOR_SENSOR, INPUT_PULLUP);
  }
#else
  pinMode(PIN_DOOR_SENSOR, INPUT_PULLUP);
#endif

  const bool raw_closed = (digitalRead(PIN_DOOR_SENSOR) == DOOR_CLOSED);
  state_.door_raw_state = raw_closed ? 1u : 0u;
  state_.door_state = raw_closed ? 1u : 0u;

  // Allow immediate direction activation on first command.
  state_.last_direction_change_ms = millis() - MOTOR_DIRECTION_DELAY_MS;
}

void IOAbstraction::update() {
  const bool raw_closed = (digitalRead(PIN_DOOR_SENSOR) == DOOR_CLOSED);
  const bool last_raw_closed = (state_.door_raw_state != 0u);

  if (raw_closed == last_raw_closed) {
    if (state_.door_stable_count < 255u) {
      state_.door_stable_count++;
    }
  } else {
    state_.door_raw_state = raw_closed ? 1u : 0u;
    state_.door_stable_count = 0;
  }

  if (state_.door_stable_count >= kDoorDebounceCounts) {
    state_.door_state = raw_closed ? 1u : 0u;
  }

  // Safety-first: door open forces all outputs OFF at IO layer.
  if (!isDoorClosed()) {
    emergencyStop();
  }
}

bool IOAbstraction::isDoorClosed() const {
  return state_.door_state != 0u;
}

uint8_t IOAbstraction::getOutputMismatchCount() const {
  return state_.output_mismatch_count;
}

bool IOAbstraction::verifyOutputState(uint8_t pin, bool expected) {
  const bool actual = (digitalRead(pin) == kRelayOnLevel);
  if (actual != expected) {
    if (state_.output_mismatch_count < 255u) {
      state_.output_mismatch_count++;
    }
    return false;
  }
  state_.output_mismatch_count = 0;
  return true;
}

void IOAbstraction::emergencyStop() {
  digitalWrite(PIN_HEATER_RELAY, kRelayOffLevel);
  digitalWrite(PIN_MOTOR_FWD_RELAY, kRelayOffLevel);
  digitalWrite(PIN_MOTOR_REV_RELAY, kRelayOffLevel);
  digitalWrite(PIN_AUX_RELAY, kRelayOffLevel);
  noTone(PIN_BUZZER);

  state_.heater_state = 0;
  state_.motor_fwd_state = 0;
  state_.motor_rev_state = 0;
  state_.aux_state = 0;
}

void IOAbstraction::setHeaterRelay(bool state) {
  if (!state || !isDoorClosed()) {
    digitalWrite(PIN_HEATER_RELAY, kRelayOffLevel);
    state_.heater_state = 0;
    (void)verifyOutputState(PIN_HEATER_RELAY, false);
    return;
  }

  digitalWrite(PIN_HEATER_RELAY, kRelayOnLevel);
  state_.heater_state = 1;
  (void)verifyOutputState(PIN_HEATER_RELAY, true);
}

void IOAbstraction::setMotorForward(bool state) {
  if (!state || !isDoorClosed()) {
    digitalWrite(PIN_MOTOR_FWD_RELAY, kRelayOffLevel);
    state_.motor_fwd_state = 0;
    (void)verifyOutputState(PIN_MOTOR_FWD_RELAY, false);
    return;
  }

  // Never allow both directions energized simultaneously.
  if (state_.motor_rev_state != 0u) {
    digitalWrite(PIN_MOTOR_REV_RELAY, kRelayOffLevel);
    state_.motor_rev_state = 0;
    (void)verifyOutputState(PIN_MOTOR_REV_RELAY, false);
    state_.last_direction_change_ms = millis();
  }

  // Enforce dead-time between direction changes (break-before-make).
  if (millis() - state_.last_direction_change_ms < MOTOR_DIRECTION_DELAY_MS) {
    digitalWrite(PIN_MOTOR_FWD_RELAY, kRelayOffLevel);
    state_.motor_fwd_state = 0;
    (void)verifyOutputState(PIN_MOTOR_FWD_RELAY, false);
    return;
  }

  digitalWrite(PIN_MOTOR_FWD_RELAY, kRelayOnLevel);
  state_.motor_fwd_state = 1;
  (void)verifyOutputState(PIN_MOTOR_FWD_RELAY, true);
}

void IOAbstraction::setMotorReverse(bool state) {
  if (!state || !isDoorClosed()) {
    digitalWrite(PIN_MOTOR_REV_RELAY, kRelayOffLevel);
    state_.motor_rev_state = 0;
    (void)verifyOutputState(PIN_MOTOR_REV_RELAY, false);
    return;
  }

  // Never allow both directions energized simultaneously.
  if (state_.motor_fwd_state != 0u) {
    digitalWrite(PIN_MOTOR_FWD_RELAY, kRelayOffLevel);
    state_.motor_fwd_state = 0;
    (void)verifyOutputState(PIN_MOTOR_FWD_RELAY, false);
    state_.last_direction_change_ms = millis();
  }

  // Enforce dead-time between direction changes (break-before-make).
  if (millis() - state_.last_direction_change_ms < MOTOR_DIRECTION_DELAY_MS) {
    digitalWrite(PIN_MOTOR_REV_RELAY, kRelayOffLevel);
    state_.motor_rev_state = 0;
    (void)verifyOutputState(PIN_MOTOR_REV_RELAY, false);
    return;
  }

  digitalWrite(PIN_MOTOR_REV_RELAY, kRelayOnLevel);
  state_.motor_rev_state = 1;
  (void)verifyOutputState(PIN_MOTOR_REV_RELAY, true);
}

void IOAbstraction::setAuxRelay(bool state) {
  if (!state || !isDoorClosed()) {
    digitalWrite(PIN_AUX_RELAY, kRelayOffLevel);
    state_.aux_state = 0;
    (void)verifyOutputState(PIN_AUX_RELAY, false);
    return;
  }

  digitalWrite(PIN_AUX_RELAY, kRelayOnLevel);
  state_.aux_state = 1;
  (void)verifyOutputState(PIN_AUX_RELAY, true);
}

void IOAbstraction::beep(uint16_t freq_hz, uint16_t duration_ms) {
  if (duration_ms == 0u || freq_hz == 0u) {
    return;
  }
  tone(PIN_BUZZER, freq_hz, duration_ms);
}

void IOAbstraction::beepKey() {
  beep(4000u, 30u);
}
