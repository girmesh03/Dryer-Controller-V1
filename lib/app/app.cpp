#include "app.h"

#include <Arduino.h>

#include "config_build.h"
#include "ds18b20_sensor.h"
#include "drum_control.h"
#include "heater_control.h"
#include "ui_lcd.h"

extern UILCD lcd;
extern DS18B20Sensor tempSensor;
extern DrumControl drumControl;
extern HeaterControl heaterControl;

extern uint8_t g_heater_test_active;

#ifndef ENABLE_SERIAL_DEBUG
#define ENABLE_SERIAL_DEBUG 0
#endif

namespace {
constexpr uint32_t kBootScreenDurationMs = 3000;
constexpr uint32_t kServiceSeqTimeoutMs = 5000;
constexpr uint32_t kInvalidKeyMsgMs = 800;
constexpr uint32_t kIdleTempRefreshMs = 1000;

constexpr uint8_t kServiceViewMenu = 0u;
constexpr uint8_t kServiceViewDrumTest = 1u;
constexpr uint8_t kServiceViewHeaterTest = 2u;
constexpr uint8_t kServiceViewPidView = 3u;
constexpr uint8_t kServiceViewIoTest = 4u;
constexpr uint8_t kServiceMenuItems = 4u;

void formatDuty3Chars(uint8_t duty, char out[4]) {
  if (duty > 100u) {
    duty = 100u;
  }
  if (duty == 100u) {
    out[0] = '1';
    out[1] = '0';
    out[2] = '0';
    out[3] = '\0';
    return;
  }

  out[0] = ' ';
  if (duty >= 10u) {
    out[1] = static_cast<char>('0' + (duty / 10u));
    out[2] = static_cast<char>('0' + (duty % 10u));
  } else {
    out[1] = ' ';
    out[2] = static_cast<char>('0' + duty);
  }
  out[3] = '\0';
}
} // namespace

void AppStateMachine::init() {
  state_.current_state = SystemState::BOOT;
  state_.previous_state = SystemState::BOOT;
  state_.state_entry_time_ms = millis();

  state_.menu_selection = 0;
  state_.service_seq = 0;
  state_.service_seq_start_ms = 0;
  state_.service_menu_selection = 0;
  state_.service_view = kServiceViewMenu;
  state_.service_last_dir = 255u;
  state_.heater_test_duty = 0u;
  state_.service_last_heater_duty = 255u;
  state_.service_last_heater_on = 255u;
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
      renderService_();
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
  if (old_state == SystemState::SERVICE) {
    // Safety: never leave service tools with the drum running.
    drumControl.stop();
    g_heater_test_active = 0u;
    heaterControl.disable();
  }
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

void AppStateMachine::renderService_() {
  switch (state_.service_view) {
    case kServiceViewMenu:
      renderServiceMenu_();
      break;
    case kServiceViewDrumTest:
      renderDrumTest_();
      break;
    case kServiceViewHeaterTest:
      renderHeaterTest_();
      break;
    case kServiceViewPidView:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("PID VIEW"));
      lcd.setCursor(0, 2);
      lcd.print(F("TBD"));
      lcd.setCursor(0, 3);
      lcd.print(F("B:BACK"));
      break;
    case kServiceViewIoTest:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("I/O TEST"));
      lcd.setCursor(0, 2);
      lcd.print(F("TBD"));
      lcd.setCursor(0, 3);
      lcd.print(F("B:BACK"));
      break;
    default:
      state_.service_view = kServiceViewMenu;
      renderServiceMenu_();
      break;
  }
}

void AppStateMachine::renderServiceMenu_() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SERVICE MENU"));

  const uint8_t start_index = (state_.service_menu_selection <= 1u) ? 0u : 1u;
  for (uint8_t row = 0u; row < 3u; row++) {
    const uint8_t item = static_cast<uint8_t>(start_index + row);
    lcd.setCursor(0, static_cast<uint8_t>(row + 1u));
    lcd.print((item == state_.service_menu_selection) ? F("> ") : F("  "));

    switch (item) {
      case 0u:
        lcd.print(F("DRUM TEST         "));
        break;
      case 1u:
        lcd.print(F("HEATER TEST       "));
        break;
      case 2u:
        lcd.print(F("PID VIEW          "));
        break;
      case 3u:
        lcd.print(F("I/O TEST          "));
        break;
      default:
        lcd.print(F("                  "));
        break;
    }
  }
}

void AppStateMachine::updateDrumTestDirection_() {
  const DrumControl::Direction dir = drumControl.getCurrentDirection();
  const uint8_t dir_u = static_cast<uint8_t>(dir);
  if (dir_u == state_.service_last_dir) {
    return;
  }
  state_.service_last_dir = dir_u;

  lcd.setCursor(0, 1);
  lcd.print(F("DIR: ")); // 5 chars

  switch (dir) {
    case DrumControl::Direction::FORWARD:
      lcd.print(F("FORWARD"));
      break;
    case DrumControl::Direction::REVERSE:
      lcd.print(F("REVERSE"));
      break;
    default:
      lcd.print(F("STOPPED"));
      break;
  }

  // Clear remainder of the line (20 chars total).
  lcd.print(F("        "));
}

void AppStateMachine::renderDrumTest_() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("DRUM TEST"));

  state_.service_last_dir = 255u;
  updateDrumTestDirection_();

  lcd.setCursor(0, 3);
  lcd.print(F("B:STOP"));
}

void AppStateMachine::updateHeaterTestStatus_() {
  const uint8_t duty = state_.heater_test_duty;
  const uint8_t on = heaterControl.isHeaterOn() ? 1u : 0u;

  if (duty != state_.service_last_heater_duty) {
    state_.service_last_heater_duty = duty;
    lcd.setCursor(0, 1);
    lcd.print(F("DUTY: "));
    char duty_str[4];
    formatDuty3Chars(duty, duty_str);
    lcd.print(duty_str);
    lcd.print(F("%"));
    lcd.print(F("          "));
  }

  if (on != state_.service_last_heater_on) {
    state_.service_last_heater_on = on;
    lcd.setCursor(0, 2);
    lcd.print(F("STATE: "));
    lcd.print(on ? F("ON ") : F("OFF"));
    lcd.print(F("          "));
  }
}

void AppStateMachine::renderHeaterTest_() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("HEATER TEST"));

  state_.service_last_heater_duty = 255u;
  state_.service_last_heater_on = 255u;
  updateHeaterTestStatus_();

  lcd.setCursor(0, 3);
  lcd.print(F("UP/DN:ADJUST"));
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

  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewDrumTest) {
    updateDrumTestDirection_();
  }

  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewHeaterTest) {
    updateHeaterTestStatus_();
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

    if (state_.current_state == SystemState::SERVICE) {
      if (state_.service_view != kServiceViewMenu) {
        // STOP acts as "back" inside service tools (and stops the drum test).
        if (state_.service_view == kServiceViewDrumTest) {
          drumControl.stop();
        }
        if (state_.service_view == kServiceViewHeaterTest) {
          g_heater_test_active = 0u;
          heaterControl.disable();
          state_.heater_test_duty = 0u;
        }
        state_.service_view = kServiceViewMenu;
        renderService_();
        return;
      }

      transitionTo(SystemState::IDLE);
      return;
    }

    if (state_.current_state == SystemState::PROGRAM_SELECT || state_.current_state == SystemState::PARAM_EDIT) {
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
          state_.service_seq_start_ms = now;
          return;
        }
        state_.service_seq = 1u;
        state_.service_seq_start_ms = now;
        return;
      }

      if (key == KeypadInput::Key::OK && state_.service_seq == 2u &&
          (now - state_.service_seq_start_ms) <= kServiceSeqTimeoutMs) {
        state_.service_seq = 0u;
        state_.service_menu_selection = 0u;
        state_.service_view = kServiceViewMenu;
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

    case SystemState::SERVICE: {
      if (state_.service_view == kServiceViewHeaterTest) {
        if (key == KeypadInput::Key::UP || key == KeypadInput::Key::DOWN) {
          uint8_t duty = state_.heater_test_duty;
          if (key == KeypadInput::Key::UP) {
            duty = (duty >= 95u) ? 100u : static_cast<uint8_t>(duty + 5u);
          } else {
            duty = (duty <= 5u) ? 0u : static_cast<uint8_t>(duty - 5u);
          }
          heaterControl.setDutyCycle(static_cast<float>(duty));
          state_.heater_test_duty = heaterControl.getCurrentDuty();
          updateHeaterTestStatus_();
          return;
        }

        showInvalidKey_();
        return;
      }

      if (state_.service_view != kServiceViewMenu) {
        showInvalidKey_();
        return;
      }

      if (key == KeypadInput::Key::UP) {
        state_.service_menu_selection =
            (state_.service_menu_selection == 0u) ? (kServiceMenuItems - 1u) : (state_.service_menu_selection - 1u);
        renderServiceMenu_();
        return;
      }

      if (key == KeypadInput::Key::DOWN) {
        state_.service_menu_selection =
            static_cast<uint8_t>((state_.service_menu_selection + 1u) % kServiceMenuItems);
        renderServiceMenu_();
        return;
      }

      if (key == KeypadInput::Key::OK) {
        if (state_.service_menu_selection == 0u) {
          drumControl.setPattern(50u, 50u, 5u);
          drumControl.start();
          state_.service_view = kServiceViewDrumTest;
          renderService_();
          return;
        }

        if (state_.service_menu_selection == 1u) {
          state_.heater_test_duty = 0u;
          heaterControl.setDutyCycle(0.0f);
          heaterControl.enable();
          g_heater_test_active = 1u;
          state_.service_view = kServiceViewHeaterTest;
          renderService_();
          return;
        }

        state_.service_view = (state_.service_menu_selection == 2u) ? kServiceViewPidView : kServiceViewIoTest;
        renderService_();
        return;
      }

      showInvalidKey_();
      return;
    }

    case SystemState::PROGRAM_SELECT:
    case SystemState::PARAM_EDIT:
      showInvalidKey_();
      return;

    default:
      showInvalidKey_();
      return;
  }
}
