#include "app.h"

#include <Arduino.h>

#include "config_build.h"
#include "ds18b20_sensor.h"
#include "drum_control.h"
#include "heater_control.h"
#include "pid_control.h"
#include "ui_lcd.h"

extern UILCD lcd;
extern DS18B20Sensor tempSensor;
extern DrumControl drumControl;
extern HeaterControl heaterControl;
extern PIDControl pidController;

#if ENABLE_SERVICE_MENU
extern uint8_t g_heater_test_active;
#endif

#ifndef ENABLE_SERIAL_DEBUG
#define ENABLE_SERIAL_DEBUG 0
#endif

namespace {
constexpr uint32_t kBootScreenDurationMs = 3000;
constexpr uint32_t kInvalidKeyMsgMs = 800;
constexpr uint32_t kIdleTempRefreshMs = 1000;

#if ENABLE_SERVICE_MENU
constexpr uint32_t kServiceSeqTimeoutMs = 5000;
constexpr uint32_t kPidViewRefreshMs = 1000;

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

uint8_t clampFloatToU8(float value, uint8_t max_v) {
  if (value <= 0.0f) {
    return 0u;
  }
  const float vmax = static_cast<float>(max_v);
  if (value >= vmax) {
    return max_v;
  }
  return static_cast<uint8_t>(value + 0.5f);
}

void clearLine20(char line[21]) {
  for (uint8_t i = 0u; i < 20u; i++) {
    line[i] = ' ';
  }
  line[20] = '\0';
}

uint16_t scaleClampU16(float value, float scale, uint16_t max_scaled) {
  if (value <= 0.0f) {
    return 0u;
  }
  const float scaled = (value * scale) + 0.5f;
  const float max_f = static_cast<float>(max_scaled);
  if (scaled >= max_f) {
    return max_scaled;
  }
  return static_cast<uint16_t>(scaled);
}

void writeU8_3(char* dest, uint8_t value) {
  dest[0] = static_cast<char>('0' + (value / 100u));
  dest[1] = static_cast<char>('0' + ((value / 10u) % 10u));
  dest[2] = static_cast<char>('0' + (value % 10u));
}

void writeU16Whole3(char* dest, uint16_t whole) {
  if (whole >= 100u) {
    dest[0] = static_cast<char>('0' + ((whole / 100u) % 10u));
    dest[1] = static_cast<char>('0' + ((whole / 10u) % 10u));
    dest[2] = static_cast<char>('0' + (whole % 10u));
    return;
  }
  if (whole >= 10u) {
    dest[0] = ' ';
    dest[1] = static_cast<char>('0' + ((whole / 10u) % 10u));
    dest[2] = static_cast<char>('0' + (whole % 10u));
    return;
  }
  dest[0] = ' ';
  dest[1] = ' ';
  dest[2] = static_cast<char>('0' + (whole % 10u));
}

void writeU16_1dp5(char* dest, uint16_t scaled10) {
  const uint16_t whole = static_cast<uint16_t>(scaled10 / 10u);
  const uint8_t frac = static_cast<uint8_t>(scaled10 % 10u);
  writeU16Whole3(dest, whole);
  dest[3] = '.';
  dest[4] = static_cast<char>('0' + (frac % 10u));
}

void writeU16_2dp6(char* dest, uint16_t scaled100) {
  const uint16_t whole = static_cast<uint16_t>(scaled100 / 100u);
  const uint8_t frac = static_cast<uint8_t>(scaled100 % 100u);
  writeU16Whole3(dest, whole);
  dest[3] = '.';
  dest[4] = static_cast<char>('0' + (frac / 10u));
  dest[5] = static_cast<char>('0' + (frac % 10u));
}
#endif // ENABLE_SERVICE_MENU
} // namespace

void AppStateMachine::init() {
  state_.current_state = SystemState::BOOT;
  state_.previous_state = SystemState::BOOT;
  state_.state_entry_time_ms = millis();

  state_.menu_selection = 0;

#if ENABLE_SERVICE_MENU
  state_.service_seq = 0;
  state_.service_seq_start_ms = 0;
  state_.service_menu_selection = 0;
  state_.service_view = kServiceViewMenu;
  state_.service_last_dir = 255u;
  state_.heater_test_duty = 0u;
  state_.service_last_heater_duty = 255u;
  state_.service_last_heater_on = 255u;
  state_.service_pid_last_update_ms = 0u;
#endif

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
#if ENABLE_SERVICE_MENU
      return (to == SystemState::PROGRAM_SELECT) || (to == SystemState::PARAM_EDIT) ||
             (to == SystemState::SERVICE) || (to == SystemState::FAULT);
#else
      return (to == SystemState::PROGRAM_SELECT) || (to == SystemState::PARAM_EDIT) ||
             (to == SystemState::FAULT);
#endif
    case SystemState::PROGRAM_SELECT:
    case SystemState::PARAM_EDIT:
#if ENABLE_SERVICE_MENU
    case SystemState::SERVICE:
#endif
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
#if ENABLE_SERVICE_MENU
    case SystemState::SERVICE:
      renderService_();
      break;
#endif
    case SystemState::RUNNING_HEAT:
      // Phase 6: PID bumpless transfer occurs on RUNNING_HEAT entry.
      pidController.reset();
      heaterControl.enable();
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
#if ENABLE_SERVICE_MENU
  if (old_state == SystemState::SERVICE) {
    // Safety: never leave service tools with the drum running.
    drumControl.stop();
    g_heater_test_active = 0u;
    heaterControl.disable();
  }
#else
  (void)old_state;
#endif
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

#if ENABLE_SERVICE_MENU
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
      renderPidView_();
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

void AppStateMachine::updatePidView_() {
  const uint32_t now = millis();
  if (state_.service_pid_last_update_ms != 0u &&
      (now - state_.service_pid_last_update_ms) < kPidViewRefreshMs) {
    return;
  }
  state_.service_pid_last_update_ms = now;

  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  pidController.getTunings(kp, ki, kd);

  const float sp = pidController.getTargetSetpoint();
  const float pv = tempSensor.getTemperature();
  const float out_f = pidController.getLastOutput();

  const uint8_t sp_u8 = clampFloatToU8(sp, 250u);
  const uint8_t pv_u8 = clampFloatToU8(pv, 250u);
  const uint8_t out_u8 = clampFloatToU8(out_f, 100u);

  const uint16_t kp10 = scaleClampU16(kp, 10.0f, 9999u);     // 999.9 max displayed
  const uint16_t ki100 = scaleClampU16(ki, 100.0f, 65535u);  // 655.35 max displayed
  const uint16_t kd100 = scaleClampU16(kd, 100.0f, 65535u);  // 655.35 max displayed

  char line1[21];
  char line2[21];
  char line3[21];
  clearLine20(line1);
  clearLine20(line2);
  clearLine20(line3);

  // Line 2: "Kp: 10.0 Ki:  0.50"
  line1[0] = 'K';
  line1[1] = 'p';
  line1[2] = ':';
  writeU16_1dp5(&line1[3], kp10);
  line1[8] = ' ';
  line1[9] = 'K';
  line1[10] = 'i';
  line1[11] = ':';
  writeU16_2dp6(&line1[12], ki100);

  // Line 3: "Kd:  2.00 OUT:050%"
  line2[0] = 'K';
  line2[1] = 'd';
  line2[2] = ':';
  writeU16_2dp6(&line2[3], kd100);
  line2[9] = ' ';
  line2[10] = 'O';
  line2[11] = 'U';
  line2[12] = 'T';
  line2[13] = ':';
  line2[14] = static_cast<char>('0' + (out_u8 / 100u));
  line2[15] = static_cast<char>('0' + ((out_u8 / 10u) % 10u));
  line2[16] = static_cast<char>('0' + (out_u8 % 10u));
  line2[17] = '%';

  // Line 4: "SP:060 PV:055 B:BACK"
  line3[0] = 'S';
  line3[1] = 'P';
  line3[2] = ':';
  writeU8_3(&line3[3], sp_u8);
  line3[6] = ' ';
  line3[7] = 'P';
  line3[8] = 'V';
  line3[9] = ':';
  writeU8_3(&line3[10], pv_u8);
  line3[13] = ' ';
  line3[14] = 'B';
  line3[15] = ':';
  line3[16] = 'B';
  line3[17] = 'A';
  line3[18] = 'C';
  line3[19] = 'K';

  lcd.setCursor(0, 1);
  lcd.print(line1);
  lcd.setCursor(0, 2);
  lcd.print(line2);
  lcd.setCursor(0, 3);
  lcd.print(line3);
}

void AppStateMachine::renderPidView_() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("PID PARAMETERS"));
  state_.service_pid_last_update_ms = 0u;
  updatePidView_();
}
#endif // ENABLE_SERVICE_MENU

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

#if ENABLE_SERVICE_MENU
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewDrumTest) {
    updateDrumTestDirection_();
  }

  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewHeaterTest) {
    updateHeaterTestStatus_();
  }

  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewPidView) {
    updatePidView_();
  }
#endif

  if (state_.current_state == SystemState::BOOT) {
    if (now - state_.state_entry_time_ms >= kBootScreenDurationMs) {
      transitionTo(SystemState::IDLE);
    }
  }

#if ENABLE_SERVICE_MENU
  if (state_.service_seq != 0u && (now - state_.service_seq_start_ms) > kServiceSeqTimeoutMs) {
    state_.service_seq = 0u;
  }
#endif
}

void AppStateMachine::handleKeyPress(KeypadInput::Key key) {
  const uint32_t now = millis();

  // Stop/Cancel always has highest priority.
  if (key == KeypadInput::Key::STOP) {
#if ENABLE_SERVICE_MENU
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
#endif

    if (state_.current_state == SystemState::PROGRAM_SELECT || state_.current_state == SystemState::PARAM_EDIT) {
      transitionTo(SystemState::IDLE);
      return;
    }

    // In IDLE, STOP has no action.
    return;
  }

  switch (state_.current_state) {
    case SystemState::IDLE: {
#if ENABLE_SERVICE_MENU
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
#endif

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

#if ENABLE_SERVICE_MENU
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
#endif

    case SystemState::PROGRAM_SELECT:
    case SystemState::PARAM_EDIT:
      showInvalidKey_();
      return;

    default:
      showInvalidKey_();
      return;
  }
}
