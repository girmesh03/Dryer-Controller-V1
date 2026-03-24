#include "app.h"

#include <Arduino.h>

#include "config_build.h"
#include "ds18b20_sensor.h"
#include "ui_lcd.h"

extern UILCD lcd;
extern DS18B20Sensor tempSensor;

#ifndef ENABLE_SERIAL_DEBUG
#define ENABLE_SERIAL_DEBUG 0
#endif

namespace {
constexpr uint32_t kBootScreenDurationMs = 3000;
constexpr uint32_t kServiceSeqTimeoutMs = 2000;
constexpr uint32_t kInvalidKeyMsgMs = 800;
constexpr uint32_t kIdleTempRefreshMs = 1000;
} // namespace

void AppStateMachine::init() {
  state_.current_state = SystemState::BOOT;
  state_.previous_state = SystemState::BOOT;
  state_.state_entry_time_ms = millis();

  state_.menu_selection = 0;
  state_.service_seq = 0;
  state_.service_seq_start_ms = 0;
  state_.invalid_key_until_ms = 0;
  state_.last_temp_display_ms = 0;
  state_.last_temp_valid = 0;

  onEnter_(SystemState::BOOT);
}

SystemState AppStateMachine::getCurrentState() const {
  return state_.current_state;
}

bool AppStateMachine::canTransition_(SystemState from, SystemState to) const {
  if (from == to) {
    return false;
  }

  switch (from) {
    case SystemState::BOOT:
      return (to == SystemState::IDLE) || (to == SystemState::FAULT);
    case SystemState::IDLE:
      return (to == SystemState::PROGRAM_SELECT) || (to == SystemState::PARAM_EDIT) ||
             (to == SystemState::SERVICE) || (to == SystemState::FAULT);
    case SystemState::PROGRAM_SELECT:
    case SystemState::PARAM_EDIT:
    case SystemState::SERVICE:
      return (to == SystemState::IDLE) || (to == SystemState::FAULT);
    default:
      // Other states are implemented in later phases.
      return false;
  }
}

void AppStateMachine::onEnter_(SystemState new_state) {
  switch (new_state) {
    case SystemState::BOOT:
      lcd.showBootScreen();
      break;
    case SystemState::IDLE:
      lcd.showMainMenu(state_.menu_selection);
      state_.last_temp_valid = tempSensor.isValid() ? 1u : 0u;
      lcd.showTemperature(tempSensor.getTemperature(), state_.last_temp_valid != 0u);
      state_.last_temp_display_ms = millis();
      break;
    case SystemState::SERVICE:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("SERVICE MENU"));
      lcd.setCursor(0, 3);
      lcd.print(F("B:EXIT"));
      break;
    case SystemState::PROGRAM_SELECT:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("AUTO MODE"));
      lcd.setCursor(0, 2);
      lcd.print(F("OK: TBD"));
      lcd.setCursor(0, 3);
      lcd.print(F("B:BACK"));
      break;
    case SystemState::PARAM_EDIT:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("MANUAL MODE"));
      lcd.setCursor(0, 2);
      lcd.print(F("OK: TBD"));
      lcd.setCursor(0, 3);
      lcd.print(F("B:BACK"));
      break;
    default:
      break;
  }
}

void AppStateMachine::onExit_(SystemState old_state) {
  (void)old_state;
}

void AppStateMachine::transitionTo(SystemState new_state) {
  const SystemState from = state_.current_state;
  if (!canTransition_(from, new_state)) {
    return;
  }

  onExit_(from);
  state_.previous_state = from;
  state_.current_state = new_state;
  state_.state_entry_time_ms = millis();

#if ENABLE_SERIAL_DEBUG
  Serial.print(F("STATE "));
  Serial.print(static_cast<uint8_t>(from));
  Serial.print(F(" -> "));
  Serial.println(static_cast<uint8_t>(new_state));
#endif

  onEnter_(new_state);
}

void AppStateMachine::restoreScreen_() {
  onEnter_(state_.current_state);
}

void AppStateMachine::showInvalidKey_() {
  state_.invalid_key_until_ms = millis() + kInvalidKeyMsgMs;
  lcd.setCursor(0, 3);
  lcd.print(F("INVALID KEY"));
}

void AppStateMachine::update() {
  const uint32_t now = millis();

  if (state_.invalid_key_until_ms != 0u && now >= state_.invalid_key_until_ms) {
    state_.invalid_key_until_ms = 0u;
    restoreScreen_();
  }

  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::IDLE) {
    const uint8_t valid_now = tempSensor.isValid() ? 1u : 0u;
    const bool validity_changed = (valid_now != state_.last_temp_valid);

    if (validity_changed || (now - state_.last_temp_display_ms) >= kIdleTempRefreshMs) {
      state_.last_temp_valid = valid_now;
      state_.last_temp_display_ms = now;
      lcd.showTemperature(tempSensor.getTemperature(), valid_now != 0u);
    }
  }

  if (state_.current_state == SystemState::BOOT) {
    if (now - state_.state_entry_time_ms >= kBootScreenDurationMs) {
      transitionTo(SystemState::IDLE);
    }
  }

  if (state_.service_seq != 0u && (now - state_.service_seq_start_ms) > kServiceSeqTimeoutMs) {
    state_.service_seq = 0u;
  }
}

void AppStateMachine::handleKeyPress(KeypadInput::Key key) {
  const uint32_t now = millis();

  // Stop/Cancel always has highest priority.
  if (key == KeypadInput::Key::STOP) {
    state_.service_seq = 0u;

    if (state_.current_state == SystemState::SERVICE || state_.current_state == SystemState::PROGRAM_SELECT ||
        state_.current_state == SystemState::PARAM_EDIT) {
      transitionTo(SystemState::IDLE);
      return;
    }

    // In IDLE, STOP has no action.
    return;
  }

  switch (state_.current_state) {
    case SystemState::IDLE: {
      // Service menu sequence: **5 from idle (Req 19).
      if (key == KeypadInput::Key::KEY_STAR) {
        if (state_.service_seq == 0u) {
          state_.service_seq = 1u;
          state_.service_seq_start_ms = now;
          return;
        }
        if (state_.service_seq == 1u) {
          state_.service_seq = 2u;
          return;
        }
        state_.service_seq = 1u;
        state_.service_seq_start_ms = now;
        return;
      }

      if (key == KeypadInput::Key::OK && state_.service_seq == 2u &&
          (now - state_.service_seq_start_ms) <= kServiceSeqTimeoutMs) {
        state_.service_seq = 0u;
        transitionTo(SystemState::SERVICE);
        return;
      }

      state_.service_seq = 0u;

      if (key == KeypadInput::Key::UP) {
        state_.menu_selection = (state_.menu_selection == 0u) ? 1u : 0u;
        lcd.showMainMenu(state_.menu_selection);
        state_.last_temp_valid = tempSensor.isValid() ? 1u : 0u;
        lcd.showTemperature(tempSensor.getTemperature(), state_.last_temp_valid != 0u);
        return;
      }

      if (key == KeypadInput::Key::DOWN) {
        state_.menu_selection = (state_.menu_selection == 0u) ? 1u : 0u;
        lcd.showMainMenu(state_.menu_selection);
        state_.last_temp_valid = tempSensor.isValid() ? 1u : 0u;
        lcd.showTemperature(tempSensor.getTemperature(), state_.last_temp_valid != 0u);
        return;
      }

      if (key == KeypadInput::Key::OK) {
        if (state_.menu_selection == 0u) {
          transitionTo(SystemState::PROGRAM_SELECT);
        } else {
          transitionTo(SystemState::PARAM_EDIT);
        }
        return;
      }

      showInvalidKey_();
      return;
    }

    case SystemState::SERVICE:
    case SystemState::PROGRAM_SELECT:
    case SystemState::PARAM_EDIT:
      showInvalidKey_();
      return;

    default:
      showInvalidKey_();
      return;
  }
}
