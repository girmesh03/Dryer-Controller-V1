#include "heater_control.h"

#include <Arduino.h>

#include "app.h"
#include "config_build.h"
#include "ds18b20_sensor.h"
#include "io_abstraction.h"

extern IOAbstraction io;
extern DS18B20Sensor tempSensor;
extern AppStateMachine app;

// Phase 10 will replace these placeholders with the real fault system.
extern uint8_t g_has_fault;

// Set by service screens (Phase 5+).
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_HEATER_TEST
extern uint8_t g_heater_test_active;
#endif
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_FOPDT_ID
extern uint8_t g_fopdt_active;
#endif

// Phase 6+ will set this when RUNNING_HEAT is implemented.
extern float g_setpoint_c;

namespace {
constexpr uint16_t kDefaultWindowMs = 10000u;
constexpr uint16_t kMinWindowMs = 5000u;
constexpr uint16_t kMaxWindowMs = 15000u;
constexpr uint8_t kSlewLimitPercentPerWindow = 10u;

float absf(float x) {
  return (x < 0.0f) ? -x : x;
}
} // namespace

void HeaterControl::init() {
  const uint32_t now = millis();

  state_.duty_percent = 0u;
  state_.last_duty = 0u;

  state_.window_period_ms = kDefaultWindowMs;
  state_.window_start_ms = now;

  // Allow immediate ON/OFF transitions on first command.
  state_.heater_on_start_ms = now - (static_cast<uint32_t>(HEATER_MIN_ON_TIME_S) * 1000u);
  state_.heater_off_start_ms = now - (static_cast<uint32_t>(HEATER_MIN_OFF_TIME_S) * 1000u);

  state_.heater_state = 0u;
  state_.enabled = 0u;
}

void HeaterControl::setWindowPeriod(uint16_t period_ms) {
  if (period_ms < kMinWindowMs) {
    period_ms = kMinWindowMs;
  } else if (period_ms > kMaxWindowMs) {
    period_ms = kMaxWindowMs;
  }
  state_.window_period_ms = period_ms;
}

void HeaterControl::enable() {
  state_.enabled = 1u;
}

void HeaterControl::disable() {
  state_.enabled = 0u;
  state_.duty_percent = 0u;
  if (state_.heater_state != 0u) {
    state_.heater_state = 0u;
    state_.heater_off_start_ms = millis();
  }
}

bool HeaterControl::isHeaterOn() const {
  return state_.heater_state != 0u;
}

uint8_t HeaterControl::getCurrentDuty() const {
  return state_.duty_percent;
}

void HeaterControl::setDutyCycle(float duty_percent) {
  // Clamp input (no heavy math; keep small).
  uint8_t duty = 0u;
  if (duty_percent >= 100.0f) {
    duty = 100u;
  } else if (duty_percent > 0.0f) {
    duty = static_cast<uint8_t>(duty_percent + 0.5f);
  }

  // Slew limiting applies only to closed-loop RUNNING_HEAT operation. Service tools
  // (HEATER TEST / FOPDT / AUTOTUNE) require true step changes.
  const SystemState st = app.getCurrentState();
  const bool apply_slew = (st == SystemState::RUNNING_HEAT);

  if (apply_slew) {
    // Slew limit relative to the last applied duty at the start of the current window.
    const int16_t delta = static_cast<int16_t>(duty) - static_cast<int16_t>(state_.last_duty);
    if (delta > static_cast<int16_t>(kSlewLimitPercentPerWindow)) {
      duty = static_cast<uint8_t>(state_.last_duty + kSlewLimitPercentPerWindow);
    } else if (delta < -static_cast<int16_t>(kSlewLimitPercentPerWindow)) {
      duty = static_cast<uint8_t>(state_.last_duty - kSlewLimitPercentPerWindow);
    }
  }

  state_.duty_percent = duty;
}

bool HeaterControl::canTurnOn_(uint32_t now_ms) const {
  return (now_ms - state_.heater_off_start_ms) >= (static_cast<uint32_t>(HEATER_MIN_OFF_TIME_S) * 1000u);
}

bool HeaterControl::canTurnOff_(uint32_t now_ms) const {
  return (now_ms - state_.heater_on_start_ms) >= (static_cast<uint32_t>(HEATER_MIN_ON_TIME_S) * 1000u);
}

bool HeaterControl::isWithinDeadband_(float temp_c, float setpoint_c) const {
  return absf(temp_c - setpoint_c) <= 1.0f;
}

bool HeaterControl::canEnergizeHeater_() const {
  if (state_.enabled == 0u) {
    return false;
  }
  if (g_has_fault != 0u) {
    return false;
  }
  if (!io.isDoorClosed()) {
    return false;
  }
  if (!tempSensor.isValid()) {
    return false;
  }

  const float pv_c = tempSensor.getTemperature();
  if (pv_c >= static_cast<float>(OVER_TEMP_THRESHOLD)) {
    return false;
  }

  const SystemState st = app.getCurrentState();
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_HEATER_TEST
  const bool in_heater_test = (st == SystemState::SERVICE) && (g_heater_test_active != 0u);
#else
  const bool in_heater_test = false;
#endif
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_FOPDT_ID
  const bool in_fopdt = (st == SystemState::SERVICE) && (g_fopdt_active != 0u);
#else
  const bool in_fopdt = false;
#endif
  const bool in_running_heat = (st == SystemState::RUNNING_HEAT);
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE
  const bool in_autotune = (st == SystemState::AUTOTUNE);
#else
  const bool in_autotune = false;
#endif

  if (!in_heater_test && !in_fopdt && !in_running_heat && !in_autotune) {
    return false;
  }

  // Deadband applies to closed-loop heating; service HEATER TEST is duty-driven.
  if (in_running_heat && isWithinDeadband_(pv_c, g_setpoint_c)) {
    return false;
  }

  return true;
}

void HeaterControl::update() {
  const uint32_t now = millis();

  if ((now - state_.window_start_ms) >= state_.window_period_ms) {
    state_.window_start_ms = now;
    state_.last_duty = state_.duty_percent;
  }

  const bool safe_to_run = canEnergizeHeater_();

  // Safety gating: force OFF immediately (do not honor min-on time).
  if (!safe_to_run) {
    if (state_.heater_state != 0u) {
      state_.heater_state = 0u;
      state_.heater_off_start_ms = now;
    }
    return;
  }

  // Commanded OFF: honor minimum ON time (anti-chatter) unless safety gating fails.
  if (state_.duty_percent == 0u) {
    if (state_.heater_state != 0u && canTurnOff_(now)) {
      state_.heater_state = 0u;
      state_.heater_off_start_ms = now;
    }
    return;
  }

  const uint32_t elapsed_ms = now - state_.window_start_ms;
  const uint32_t on_time_ms =
      (static_cast<uint32_t>(state_.window_period_ms) * static_cast<uint32_t>(state_.duty_percent)) / 100u;
  const bool want_on = elapsed_ms < on_time_ms;

  if (want_on) {
    if (state_.heater_state == 0u && canTurnOn_(now)) {
      state_.heater_state = 1u;
      state_.heater_on_start_ms = now;
    }
    return;
  }

  if (state_.heater_state != 0u && canTurnOff_(now)) {
    state_.heater_state = 0u;
    state_.heater_off_start_ms = now;
  }
}
