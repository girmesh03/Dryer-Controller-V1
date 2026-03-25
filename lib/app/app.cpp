#include "app.h"

#include <Arduino.h>

#include "config_build.h"
#include "ds18b20_sensor.h"
#include "drum_control.h"
#include "heater_control.h"
#include "pid_control.h"
#include "ui_lcd.h"

#if ENABLE_SERVICE_MENU && (ENABLE_SERVICE_FOPDT_ID || ENABLE_SERVICE_AUTOTUNE)
#include "eeprom_store.h"
#include "io_abstraction.h"
#endif
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_FOPDT_ID
#include "fopdt_model.h"
#endif

extern UILCD lcd;
extern DS18B20Sensor tempSensor;
extern DrumControl drumControl;
extern HeaterControl heaterControl;
extern PIDControl pidController;

#if ENABLE_SERVICE_MENU
#if ENABLE_SERVICE_FOPDT_ID || ENABLE_SERVICE_AUTOTUNE
extern IOAbstraction io;
extern EEPROMStore eepromStore;
#endif
#if ENABLE_SERVICE_FOPDT_ID
extern FOPDTModel fopdt;
extern uint8_t g_fopdt_active;
#endif
#if ENABLE_SERVICE_HEATER_TEST
extern uint8_t g_heater_test_active;
#endif
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
constexpr uint8_t kServiceViewMenu = 0u;
#if ENABLE_SERVICE_DRUM_TEST
constexpr uint8_t kServiceViewDrumTest = 1u;
#endif
#if ENABLE_SERVICE_HEATER_TEST
constexpr uint8_t kServiceViewHeaterTest = 2u;
#endif
#if ENABLE_SERVICE_PID_VIEW
constexpr uint8_t kServiceViewPidView = 3u;
constexpr uint32_t kPidViewRefreshMs = 1000;
#endif
#if ENABLE_SERVICE_IO_TEST
constexpr uint8_t kServiceViewIoTest = 4u;
#endif
#if ENABLE_SERVICE_FOPDT_ID
constexpr uint8_t kServiceViewFopdt = 5u;
constexpr uint32_t kFopdtRefreshMs = 1000;

constexpr float kFopdtStepSize = 0.3f; // 30% (fraction)
constexpr float kFopdtAmbientMinC = 15.0f;
constexpr float kFopdtAmbientMaxC = 30.0f;
#endif

#if ENABLE_SERVICE_AUTOTUNE
constexpr uint8_t kServiceViewAutoTunePre = 6u;
constexpr uint32_t kAutoTuneUiRefreshMs = 1000;
#endif

enum class ServiceItem : uint8_t {
#if ENABLE_SERVICE_DRUM_TEST
  DRUM_TEST,
#endif
#if ENABLE_SERVICE_HEATER_TEST
  HEATER_TEST,
#endif
#if ENABLE_SERVICE_PID_VIEW
  PID_VIEW,
#endif
#if ENABLE_SERVICE_IO_TEST
  IO_TEST,
#endif
#if ENABLE_SERVICE_FOPDT_ID
  FOPDT_ID,
#endif
#if ENABLE_SERVICE_AUTOTUNE
  AUTOTUNE,
#endif
};

// NOTE: The AVR toolchain defaults to C++11, where constexpr functions are
// restricted to a single return statement. Keep this as a pure constant
// expression so it works under C++11 without extra init code.
constexpr uint8_t kServiceMenuItems =
    static_cast<uint8_t>(ENABLE_SERVICE_DRUM_TEST + ENABLE_SERVICE_HEATER_TEST + ENABLE_SERVICE_PID_VIEW +
                         ENABLE_SERVICE_IO_TEST + ENABLE_SERVICE_FOPDT_ID + ENABLE_SERVICE_AUTOTUNE);
static_assert(kServiceMenuItems > 0u, "Service menu enabled but no service tools enabled");

ServiceItem serviceMenuItemByIndex_(uint8_t idx) {
#if ENABLE_SERVICE_DRUM_TEST
  if (idx-- == 0u) return ServiceItem::DRUM_TEST;
#endif
#if ENABLE_SERVICE_HEATER_TEST
  if (idx-- == 0u) return ServiceItem::HEATER_TEST;
#endif
#if ENABLE_SERVICE_PID_VIEW
  if (idx-- == 0u) return ServiceItem::PID_VIEW;
#endif
#if ENABLE_SERVICE_IO_TEST
  if (idx-- == 0u) return ServiceItem::IO_TEST;
#endif
#if ENABLE_SERVICE_FOPDT_ID
  if (idx-- == 0u) return ServiceItem::FOPDT_ID;
#endif
#if ENABLE_SERVICE_AUTOTUNE
  if (idx-- == 0u) return ServiceItem::AUTOTUNE;
#endif

  // Fallback (should be unreachable due to bounds checks).
#if ENABLE_SERVICE_DRUM_TEST
  return ServiceItem::DRUM_TEST;
#elif ENABLE_SERVICE_HEATER_TEST
  return ServiceItem::HEATER_TEST;
#elif ENABLE_SERVICE_PID_VIEW
  return ServiceItem::PID_VIEW;
#elif ENABLE_SERVICE_IO_TEST
  return ServiceItem::IO_TEST;
#else
#if ENABLE_SERVICE_FOPDT_ID
  return ServiceItem::FOPDT_ID;
#else
  return ServiceItem::AUTOTUNE;
#endif
#endif
}

#if ENABLE_SERVICE_HEATER_TEST || ENABLE_SERVICE_FOPDT_ID || ENABLE_SERVICE_AUTOTUNE
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
#endif

#if ENABLE_SERVICE_PID_VIEW
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
#endif

#if ENABLE_SERVICE_PID_VIEW || ENABLE_SERVICE_FOPDT_ID || ENABLE_SERVICE_AUTOTUNE
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
#endif

#if ENABLE_SERVICE_PID_VIEW
void writeU8_3(char* dest, uint8_t value) {
  dest[0] = static_cast<char>('0' + (value / 100u));
  dest[1] = static_cast<char>('0' + ((value / 10u) % 10u));
  dest[2] = static_cast<char>('0' + (value % 10u));
}
#endif

#if ENABLE_SERVICE_PID_VIEW || ENABLE_SERVICE_FOPDT_ID || ENABLE_SERVICE_AUTOTUNE
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
#endif
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
#if ENABLE_SERVICE_DRUM_TEST
  state_.service_last_dir = 255u;
#endif
#if ENABLE_SERVICE_HEATER_TEST
  state_.heater_test_duty = 0u;
  state_.service_last_heater_duty = 255u;
  state_.service_last_heater_on = 255u;
#endif
#if ENABLE_SERVICE_PID_VIEW
  state_.service_pid_last_update_ms = 0u;
#endif
#if ENABLE_SERVICE_FOPDT_ID
  state_.service_fopdt_page = 0u;
  state_.service_fopdt_last_update_ms = 0u;
#endif
#if ENABLE_SERVICE_AUTOTUNE
  state_.service_autotune_page = 0u;
  state_.service_autotune_last_update_ms = 0u;
  state_.service_autotune_start_ms = 0u;
#endif
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
    case SystemState::FAULT:
      // Fault handling is fully implemented in Phase 10; for now allow returning
      // to IDLE after the operator acknowledges.
      return (to == SystemState::IDLE);
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
      return (to == SystemState::IDLE) || (to == SystemState::FAULT);
#if ENABLE_SERVICE_MENU
    case SystemState::SERVICE:
#if ENABLE_SERVICE_AUTOTUNE
      return (to == SystemState::IDLE) || (to == SystemState::FAULT) || (to == SystemState::AUTOTUNE);
#else
      return (to == SystemState::IDLE) || (to == SystemState::FAULT);
#endif
#endif
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE
    case SystemState::AUTOTUNE:
      return (to == SystemState::SERVICE) || (to == SystemState::FAULT) || (to == SystemState::IDLE);
#endif
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
    case SystemState::FAULT:
      // Phase 10 introduces the full fault system; keep this minimal and safe.
      drumControl.stop();
      heaterControl.disable();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("FAULT"));
#if ENABLE_SERVICE_MENU && (ENABLE_SERVICE_FOPDT_ID || ENABLE_SERVICE_AUTOTUNE)
      lcd.setCursor(0, 1);
      lcd.print(!io.isDoorClosed() ? F("DOOR OPEN") : F("CHECK SYSTEM"));
#endif
      lcd.setCursor(0, 3);
      lcd.print(F("B:ACK"));
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
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE
    case SystemState::AUTOTUNE:
      state_.service_autotune_page = 0u;
      state_.service_autotune_last_update_ms = 0u;
      state_.service_autotune_start_ms = millis();
      pidController.startAutoTune();
      renderAutoTune_();
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
#if ENABLE_SERVICE_HEATER_TEST
    g_heater_test_active = 0u;
#endif
#if ENABLE_SERVICE_FOPDT_ID
    g_fopdt_active = 0u;
    fopdt.abort();
#endif
    heaterControl.disable();
  }

#if ENABLE_SERVICE_AUTOTUNE
  if (old_state == SystemState::AUTOTUNE) {
    // Safety: never leave AUTOTUNE with heater control active.
    if (pidController.isAutoTuning() || pidController.isAutoTuneComplete()) {
      pidController.abortAutoTune();
    }
  }
#endif
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
#if ENABLE_SERVICE_DRUM_TEST
    case kServiceViewDrumTest:
      renderDrumTest_();
      break;
#endif
#if ENABLE_SERVICE_HEATER_TEST
    case kServiceViewHeaterTest:
      renderHeaterTest_();
      break;
#endif
#if ENABLE_SERVICE_PID_VIEW
    case kServiceViewPidView:
      renderPidView_();
      break;
#endif
#if ENABLE_SERVICE_IO_TEST
    case kServiceViewIoTest:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("I/O TEST"));
      lcd.setCursor(0, 2);
      lcd.print(F("TBD"));
      lcd.setCursor(0, 3);
      lcd.print(F("B:BACK"));
      break;
#endif
#if ENABLE_SERVICE_FOPDT_ID
    case kServiceViewFopdt:
      renderFopdt_();
      break;
#endif
#if ENABLE_SERVICE_AUTOTUNE
    case kServiceViewAutoTunePre:
      renderAutoTunePre_();
      break;
#endif
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

  uint8_t start_index = 0u;
  if (kServiceMenuItems > 3u) {
    if (state_.service_menu_selection > 1u) {
      start_index = static_cast<uint8_t>(state_.service_menu_selection - 1u);
      const uint8_t max_start = static_cast<uint8_t>(kServiceMenuItems - 3u);
      if (start_index > max_start) {
        start_index = max_start;
      }
    }
  }
  for (uint8_t row = 0u; row < 3u; row++) {
    const uint8_t item = static_cast<uint8_t>(start_index + row);
    lcd.setCursor(0, static_cast<uint8_t>(row + 1u));
    if (item >= kServiceMenuItems) {
      lcd.print(F("                    "));
      continue;
    }

    lcd.print((item == state_.service_menu_selection) ? F("> ") : F("  "));

    switch (serviceMenuItemByIndex_(item)) {
#if ENABLE_SERVICE_DRUM_TEST
      case ServiceItem::DRUM_TEST:
        lcd.print(F("DRUM TEST         "));
        break;
#endif
#if ENABLE_SERVICE_HEATER_TEST
      case ServiceItem::HEATER_TEST:
        lcd.print(F("HEATER TEST       "));
        break;
#endif
#if ENABLE_SERVICE_PID_VIEW
      case ServiceItem::PID_VIEW:
        lcd.print(F("PID VIEW          "));
        break;
#endif
#if ENABLE_SERVICE_IO_TEST
      case ServiceItem::IO_TEST:
        lcd.print(F("I/O TEST          "));
        break;
#endif
#if ENABLE_SERVICE_FOPDT_ID
      case ServiceItem::FOPDT_ID:
        lcd.print(F("FOPDT ID          "));
        break;
#endif
#if ENABLE_SERVICE_AUTOTUNE
      case ServiceItem::AUTOTUNE:
        lcd.print(F("AUTO TUNE         "));
        break;
#endif
      default:
        lcd.print(F("                  "));
        break;
    }
  }
}

#if ENABLE_SERVICE_DRUM_TEST
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
#endif // ENABLE_SERVICE_DRUM_TEST

#if ENABLE_SERVICE_HEATER_TEST
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
#endif // ENABLE_SERVICE_HEATER_TEST

#if ENABLE_SERVICE_PID_VIEW
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
#endif // ENABLE_SERVICE_PID_VIEW

#if ENABLE_SERVICE_FOPDT_ID
void AppStateMachine::renderFopdt_() {
  const FOPDTModel::State st = fopdt.getState();

  if (st == FOPDTModel::State::IDLE) {
    const bool door_ok = io.isDoorClosed();
    const bool sens_ok = tempSensor.isValid();

    lcd.setCursor(0, 0);
    lcd.print(F("FOPDT ID            "));

    lcd.setCursor(0, 1);
    lcd.print(F("EMPTY DRUM          "));

    lcd.setCursor(0, 2);
    lcd.print(F("D:"));
    lcd.print(door_ok ? F("OK") : F("OP"));
    lcd.print(F(" S:"));
    lcd.print(sens_ok ? F("OK") : F("ER"));
    lcd.print(F(" AMB15-30C"));
    lcd.print(F(" "));

    lcd.setCursor(0, 3);
    lcd.print(F("A:START B:CANCEL    "));
    return;
  }

  if (st == FOPDTModel::State::COMPLETE) {
    float k = 0.0f;
    float tau_s = 0.0f;
    float l_s = 0.0f;
    const bool ok = fopdt.getResults(k, tau_s, l_s);

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (ok) {
      fopdt.computePIDGains(k, tau_s, l_s, kp, ki, kd);
    }

    if (state_.service_fopdt_page == 0u) {
      lcd.setCursor(0, 0);
      lcd.print(F("FOPDT RESULT        "));

      const uint16_t k100 = scaleClampU16(k, 100.0f, 65535u);
      const uint16_t tau_u16 = scaleClampU16(tau_s, 1.0f, 999u);
      const uint16_t l_u16 = scaleClampU16(l_s, 1.0f, 999u);

      char k_str[7];
      writeU16_2dp6(k_str, k100);
      k_str[6] = '\0';

      lcd.setCursor(0, 1);
      lcd.print(F("K:"));
      lcd.print(k_str);
      lcd.print(F(" tau:"));
      char tau_str[4];
      writeU16Whole3(tau_str, tau_u16);
      tau_str[3] = '\0';
      lcd.print(tau_str);
      lcd.print(F("s"));
      lcd.print(F("   "));

      lcd.setCursor(0, 2);
      lcd.print(F("L:"));
      char l_str[4];
      writeU16Whole3(l_str, l_u16);
      l_str[3] = '\0';
      lcd.print(l_str);
      lcd.print(F("s"));
      lcd.print(F("              "));

      lcd.setCursor(0, 3);
      lcd.print(F("A:ACPT B:REJ OK:>   "));
      return;
    }

    lcd.setCursor(0, 0);
    lcd.print(F("IMC GAINS           "));

    const uint16_t kp10 = scaleClampU16(kp, 10.0f, 9999u);
    const uint16_t ki100 = scaleClampU16(ki, 100.0f, 65535u);
    const uint16_t kd100 = scaleClampU16(kd, 100.0f, 65535u);

    char kp_str[6];
    writeU16_1dp5(kp_str, kp10);
    kp_str[5] = '\0';

    char ki_str[7];
    writeU16_2dp6(ki_str, ki100);
    ki_str[6] = '\0';

    char kd_str[7];
    writeU16_2dp6(kd_str, kd100);
    kd_str[6] = '\0';

    lcd.setCursor(0, 1);
    lcd.print(F("Kp:"));
    lcd.print(kp_str);
    lcd.print(F(" Ki:"));
    lcd.print(ki_str);
    lcd.print(F("  "));

    lcd.setCursor(0, 2);
    lcd.print(F("Kd:"));
    lcd.print(kd_str);
    lcd.print(F("           ")); // pad to end of 20-char line

    lcd.setCursor(0, 3);
    lcd.print(F("A:ACPT B:REJ OK:<   "));
    return;
  }

  // BASELINE / STEP / MEASURE progress screens.
  const uint16_t rem_s = fopdt.getBaselineSecondsRemaining();
  const uint16_t elap_s = fopdt.getMeasureSecondsElapsed();
  const uint8_t duty = fopdt.getStepDutyPercent();

  const uint16_t temp10 = scaleClampU16(tempSensor.getTemperature(), 10.0f, 9999u);
  char temp_str[6];
  writeU16_1dp5(temp_str, temp10);
  temp_str[5] = '\0';

  lcd.setCursor(0, 0);
  if (st == FOPDTModel::State::BASELINE) {
    lcd.print(F("FOPDT BASELINE      "));
  } else if (st == FOPDTModel::State::STEP) {
    lcd.print(F("FOPDT STEP          "));
  } else {
    lcd.print(F("FOPDT MEASURE       "));
  }

  lcd.setCursor(0, 1);
  if (st == FOPDTModel::State::BASELINE) {
    lcd.print(F("T-LEFT:"));
    char rem_str[4];
    writeU16Whole3(rem_str, (rem_s > 999u) ? 999u : rem_s);
    rem_str[3] = '\0';
    lcd.print(rem_str);
    lcd.print(F("s"));
    lcd.print(F("         "));
  } else {
    lcd.print(F("DUTY:"));
    char duty_str[4];
    formatDuty3Chars(duty, duty_str);
    lcd.print(duty_str);
    lcd.print(F("% ELAP:"));
    char elap_str[4];
    writeU16Whole3(elap_str, (elap_s > 999u) ? 999u : elap_s);
    elap_str[3] = '\0';
    lcd.print(elap_str);
    lcd.print(F("s"));
    lcd.print(F(" "));
  }

  lcd.setCursor(0, 2);
  lcd.print(F("TEMP:"));
  lcd.print(temp_str);
  lcd.print(F("C"));
  lcd.print(F("         "));

  lcd.setCursor(0, 3);
  lcd.print(F("B:ABORT"));
  lcd.print(F("             "));
}

void AppStateMachine::updateFopdt_() {
  const uint32_t now = millis();
  if (state_.service_fopdt_last_update_ms != 0u &&
      (now - state_.service_fopdt_last_update_ms) < kFopdtRefreshMs) {
    return;
  }
  state_.service_fopdt_last_update_ms = now;

  if (fopdt.isIdentifying()) {
    if (!io.isDoorClosed() || !tempSensor.isValid()) {
      fopdt.abort();
      g_fopdt_active = 0u;
      heaterControl.disable();
      state_.service_fopdt_page = 0u;
      renderFopdt_();
      return;
    }

    fopdt.updateIdentification(tempSensor.getTemperature(), static_cast<float>(heaterControl.getCurrentDuty()));

    // Drive the heater step via HeaterControl; safety gating remains enforced there.
    const FOPDTModel::State st = fopdt.getState();
    if (st == FOPDTModel::State::BASELINE) {
      heaterControl.setDutyCycle(0.0f);
    } else if (st == FOPDTModel::State::STEP || st == FOPDTModel::State::MEASURE) {
      heaterControl.setDutyCycle(static_cast<float>(fopdt.getStepDutyPercent()));
    } else if (st == FOPDTModel::State::COMPLETE) {
      g_fopdt_active = 0u;
      heaterControl.disable();
    }
  }

  renderFopdt_();
}
#endif // ENABLE_SERVICE_FOPDT_ID

#if ENABLE_SERVICE_AUTOTUNE
void AppStateMachine::renderAutoTunePre_() {
  const bool door_ok = io.isDoorClosed();
  const bool sens_ok = tempSensor.isValid();

  const uint16_t pv10 = scaleClampU16(tempSensor.getTemperature(), 10.0f, 9999u);
  char pv_str[6];
  writeU16_1dp5(pv_str, pv10);
  pv_str[5] = '\0';

  lcd.setCursor(0, 0);
  lcd.print(F("AUTO TUNE RUN~20MIN "));

  lcd.setCursor(0, 1);
  lcd.print(F("EMPTY DRUM AMB15-30C"));

  lcd.setCursor(0, 2);
  lcd.print(F("D:"));
  lcd.print(door_ok ? F("OK") : F("OP"));
  lcd.print(F(" S:"));
  lcd.print(sens_ok ? F("OK") : F("ER"));
  lcd.print(F(" PV:"));
  lcd.print(pv_str);
  lcd.print(F("C"));
  lcd.print(F(" "));

  lcd.setCursor(0, 3);
  lcd.print(F("A:START B:CANCEL    "));
}

void AppStateMachine::renderAutoTune_() {
  if (pidController.isAutoTuneComplete()) {
    float ku = 0.0f;
    float tu_s = 0.0f;
    (void)pidController.getAutoTuneKuTu(ku, tu_s);

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    (void)pidController.getAutoTuneResults(kp, ki, kd);

    lcd.setCursor(0, 0);
    lcd.print(F("AUTOTUNE DONE       "));

    if (state_.service_autotune_page == 0u) {
      const uint16_t ku10 = scaleClampU16(ku, 10.0f, 9999u);
      char ku_str[6];
      writeU16_1dp5(ku_str, ku10);
      ku_str[5] = '\0';

      const uint16_t tu_u16 = scaleClampU16(tu_s, 1.0f, 999u);
      char tu_str[4];
      writeU16Whole3(tu_str, tu_u16);
      tu_str[3] = '\0';

      lcd.setCursor(0, 1);
      lcd.print(F("Ku:"));
      lcd.print(ku_str);
      lcd.print(F(" Tu:"));
      lcd.print(tu_str);
      lcd.print(F("s"));
      lcd.print(F("    "));

      const uint16_t kp10 = scaleClampU16(kp, 10.0f, 9999u);
      char kp_str[6];
      writeU16_1dp5(kp_str, kp10);
      kp_str[5] = '\0';

      const uint16_t ki100 = scaleClampU16(ki, 100.0f, 65535u);
      char ki_str[7];
      writeU16_2dp6(ki_str, ki100);
      ki_str[6] = '\0';

      lcd.setCursor(0, 2);
      lcd.print(F("Kp:"));
      lcd.print(kp_str);
      lcd.print(F(" Ki:"));
      lcd.print(ki_str);
      lcd.print(F("  "));

      lcd.setCursor(0, 3);
      lcd.print(F("A:SAVE B:REJ OK:>   "));
      return;
    }

    const uint16_t kd100 = scaleClampU16(kd, 100.0f, 65535u);
    char kd_str[7];
    writeU16_2dp6(kd_str, kd100);
    kd_str[6] = '\0';

    lcd.setCursor(0, 1);
    lcd.print(F("Kd:"));
    lcd.print(kd_str);
    lcd.print(F("           "));

    lcd.setCursor(0, 2);
    lcd.print(F("OK:PAGE A:SAVE"));
    lcd.print(F("      "));

    lcd.setCursor(0, 3);
    lcd.print(F("B:REJ OK:<"));
    lcd.print(F("          "));
    return;
  }

  // In-progress screen.
  const uint32_t elapsed_s = (millis() - state_.service_autotune_start_ms) / 1000u;
  const uint8_t cyc = pidController.getAutoTuneCycleCount();

  const uint8_t rem_cycles = (cyc >= 3u) ? 0u : static_cast<uint8_t>(3u - cyc);
  uint8_t eta_min = 0u;
  if (cyc != 0u) {
    const uint32_t avg_s = elapsed_s / cyc;
    const uint32_t rem_s = avg_s * rem_cycles;
    eta_min = static_cast<uint8_t>((rem_s > 5999u) ? 99u : (rem_s / 60u));
  }

  const uint16_t pv10 = scaleClampU16(tempSensor.getTemperature(), 10.0f, 9999u);
  char pv_str[6];
  writeU16_1dp5(pv_str, pv10);
  pv_str[5] = '\0';

  const uint8_t duty_cmd = pidController.getAutoTuneCommandDuty();
  char duty_str[4];
  formatDuty3Chars(duty_cmd, duty_str);

  const uint8_t mm = static_cast<uint8_t>((elapsed_s / 60u) % 100u);
  const uint8_t ss = static_cast<uint8_t>(elapsed_s % 60u);
  const char mm1 = static_cast<char>('0' + (mm / 10u));
  const char mm2 = static_cast<char>('0' + (mm % 10u));
  const char ss1 = static_cast<char>('0' + (ss / 10u));
  const char ss2 = static_cast<char>('0' + (ss % 10u));

  lcd.setCursor(0, 0);
  lcd.print(F("AUTOTUNE            "));

  lcd.setCursor(0, 1);
  lcd.print(F("CYCLE "));
  lcd.print(static_cast<char>('0' + ((cyc > 9u) ? 9u : cyc)));
  lcd.print(F("/3 ETA:"));
  if (cyc == 0u) {
    lcd.print(F("--"));
  } else {
    lcd.print(static_cast<char>('0' + (eta_min / 10u)));
    lcd.print(static_cast<char>('0' + (eta_min % 10u)));
  }
  lcd.print(F("m   "));

  lcd.setCursor(0, 2);
  lcd.print(F("TEMP:"));
  lcd.print(pv_str);
  lcd.print(F("C DUT:"));
  lcd.print(duty_str);
  lcd.print(F(" "));

  lcd.setCursor(0, 3);
  lcd.print(F("TIME:"));
  lcd.print(mm1);
  lcd.print(mm2);
  lcd.print(F(":"));
  lcd.print(ss1);
  lcd.print(ss2);
  lcd.print(F(" B:ABRT"));
  lcd.print(F("   "));
}

void AppStateMachine::updateAutoTuneUi_() {
  const uint32_t now = millis();
  if (state_.service_autotune_last_update_ms != 0u &&
      (now - state_.service_autotune_last_update_ms) < kAutoTuneUiRefreshMs) {
    return;
  }
  state_.service_autotune_last_update_ms = now;

  if (pidController.isAutoTuning()) {
    pidController.updateAutoTune(tempSensor.getTemperature());
  }

  // If AutoTune aborted (but not complete), exit appropriately.
  if (!pidController.isAutoTuning() && !pidController.isAutoTuneComplete()) {
    const uint8_t reason = pidController.getAutoTuneAbortReason();
    if (reason == 1u) { // door open -> FAULT (Req 6)
      transitionTo(SystemState::FAULT);
      return;
    }

    state_.service_view = kServiceViewMenu;
    transitionTo(SystemState::SERVICE);
    state_.invalid_key_until_ms = now + kInvalidKeyMsgMs;
    lcd.setCursor(0, 3);
    lcd.print(F("AT ABORT            "));
    return;
  }

  renderAutoTune_();
}
#endif // ENABLE_SERVICE_AUTOTUNE
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
#if ENABLE_SERVICE_DRUM_TEST
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewDrumTest) {
    updateDrumTestDirection_();
  }
#endif

#if ENABLE_SERVICE_HEATER_TEST
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewHeaterTest) {
    updateHeaterTestStatus_();
  }
#endif

#if ENABLE_SERVICE_PID_VIEW
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewPidView) {
    updatePidView_();
  }
#endif

#if ENABLE_SERVICE_FOPDT_ID
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewFopdt) {
    updateFopdt_();
  }
#endif

#if ENABLE_SERVICE_AUTOTUNE
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewAutoTunePre) {
    const uint32_t now_ui = millis();
    if (state_.service_autotune_last_update_ms == 0u ||
        (now_ui - state_.service_autotune_last_update_ms) >= kAutoTuneUiRefreshMs) {
      state_.service_autotune_last_update_ms = now_ui;
      renderAutoTunePre_();
    }
  }

  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::AUTOTUNE) {
    updateAutoTuneUi_();
  }
#endif
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
#if ENABLE_SERVICE_DRUM_TEST
        if (state_.service_view == kServiceViewDrumTest) {
          drumControl.stop();
        }
#endif
#if ENABLE_SERVICE_HEATER_TEST
        if (state_.service_view == kServiceViewHeaterTest) {
          g_heater_test_active = 0u;
          heaterControl.disable();
          state_.heater_test_duty = 0u;
        }
#endif
#if ENABLE_SERVICE_FOPDT_ID
        if (state_.service_view == kServiceViewFopdt) {
          fopdt.abort();
          g_fopdt_active = 0u;
          heaterControl.disable();
          state_.service_fopdt_page = 0u;
        }
#endif
        state_.service_view = kServiceViewMenu;
        renderService_();
        return;
      }

      transitionTo(SystemState::IDLE);
      return;
    }
#endif

#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE
    if (state_.current_state == SystemState::AUTOTUNE) {
      pidController.abortAutoTune();
      state_.service_view = kServiceViewMenu;
      transitionTo(SystemState::SERVICE);
      state_.invalid_key_until_ms = now + kInvalidKeyMsgMs;
      lcd.setCursor(0, 3);
      lcd.print(F("ABORTED             "));
      return;
    }
#endif

    if (state_.current_state == SystemState::FAULT) {
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
#if ENABLE_SERVICE_HEATER_TEST
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
#endif

#if ENABLE_SERVICE_FOPDT_ID
      if (state_.service_view == kServiceViewFopdt) {
        const FOPDTModel::State fst = fopdt.getState();

        if (key == KeypadInput::Key::OK && fst == FOPDTModel::State::COMPLETE) {
          state_.service_fopdt_page ^= 1u;
          renderFopdt_();
          return;
        }

        if (key == KeypadInput::Key::START) {
          if (fst == FOPDTModel::State::IDLE) {
            const bool door_ok = io.isDoorClosed();
            const bool sens_ok = tempSensor.isValid();
            const float pv = tempSensor.getTemperature();
            const bool ambient_ok = (pv >= kFopdtAmbientMinC) && (pv <= kFopdtAmbientMaxC);

            if (!door_ok || !sens_ok || !ambient_ok) {
              state_.invalid_key_until_ms = now + kInvalidKeyMsgMs;
              lcd.setCursor(0, 3);
              lcd.print(F("CHECK DOOR/TEMP"));
              return;
            }

            // Ensure other service tools are inactive.
            drumControl.stop();
#if ENABLE_SERVICE_HEATER_TEST
            g_heater_test_active = 0u;
#endif
            heaterControl.disable();

            heaterControl.setDutyCycle(0.0f);
            heaterControl.enable();
            g_fopdt_active = 1u;

            state_.service_fopdt_page = 0u;
            state_.service_fopdt_last_update_ms = 0u;
            fopdt.startIdentification(kFopdtStepSize);
            renderFopdt_();
            return;
          }

          if (fst == FOPDTModel::State::COMPLETE) {
            float k = 0.0f;
            float tau_s = 0.0f;
            float l_s = 0.0f;
            if (fopdt.getResults(k, tau_s, l_s)) {
              float kp = 0.0f;
              float ki = 0.0f;
              float kd = 0.0f;
              fopdt.computePIDGains(k, tau_s, l_s, kp, ki, kd);

              pidController.setTunings(kp, ki, kd);
              eepromStore.requestSaveFOPDT(k, tau_s, l_s);
              eepromStore.requestSavePIDGains(kp, ki, kd);
              eepromStore.update(now);
            }

            g_fopdt_active = 0u;
            heaterControl.disable();

            state_.service_view = kServiceViewMenu;
            renderService_();
            state_.invalid_key_until_ms = now + kInvalidKeyMsgMs;
            lcd.setCursor(0, 3);
            lcd.print(F("SAVED"));
            return;
          }
        }

        showInvalidKey_();
        return;
      }
#endif

#if ENABLE_SERVICE_AUTOTUNE
      if (state_.service_view == kServiceViewAutoTunePre) {
#if ENABLE_SERVICE_FOPDT_ID
        (void)fopdt; // suppress unused warnings when combined builds toggle flags
#endif
        if (key == KeypadInput::Key::START) {
          const bool door_ok = io.isDoorClosed();
          const bool sens_ok = tempSensor.isValid();
          const float pv = tempSensor.getTemperature();
          const bool ambient_ok = (pv >= 15.0f) && (pv <= 30.0f);

          if (!door_ok || !sens_ok || !ambient_ok) {
            state_.invalid_key_until_ms = now + kInvalidKeyMsgMs;
            lcd.setCursor(0, 3);
            lcd.print(F("CHECK DOOR/TEMP"));
            return;
          }

          // Ensure other service tools are inactive.
          drumControl.stop();
#if ENABLE_SERVICE_HEATER_TEST
          g_heater_test_active = 0u;
#endif
#if ENABLE_SERVICE_FOPDT_ID
          g_fopdt_active = 0u;
          fopdt.abort();
#endif
          heaterControl.disable();

          state_.service_autotune_page = 0u;
          state_.service_autotune_last_update_ms = 0u;
          state_.service_autotune_start_ms = now;
          transitionTo(SystemState::AUTOTUNE);
          return;
        }

        showInvalidKey_();
        return;
      }
#endif

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
        const ServiceItem item = serviceMenuItemByIndex_(state_.service_menu_selection);

        switch (item) {
#if ENABLE_SERVICE_DRUM_TEST
          case ServiceItem::DRUM_TEST:
            drumControl.setPattern(50u, 50u, 5u);
            drumControl.start();
            state_.service_view = kServiceViewDrumTest;
            renderService_();
            return;
#endif
#if ENABLE_SERVICE_HEATER_TEST
          case ServiceItem::HEATER_TEST:
            state_.heater_test_duty = 0u;
            heaterControl.setDutyCycle(0.0f);
            heaterControl.enable();
            g_heater_test_active = 1u;
            state_.service_view = kServiceViewHeaterTest;
            renderService_();
            return;
#endif
#if ENABLE_SERVICE_PID_VIEW
          case ServiceItem::PID_VIEW:
            state_.service_view = kServiceViewPidView;
            renderService_();
            return;
#endif
#if ENABLE_SERVICE_IO_TEST
          case ServiceItem::IO_TEST:
            state_.service_view = kServiceViewIoTest;
            renderService_();
            return;
#endif
#if ENABLE_SERVICE_FOPDT_ID
          case ServiceItem::FOPDT_ID:
            fopdt.abort();
            g_fopdt_active = 0u;
            heaterControl.disable();
            state_.service_fopdt_page = 0u;
            state_.service_fopdt_last_update_ms = 0u;
            state_.service_view = kServiceViewFopdt;
            renderService_();
            return;
#endif
#if ENABLE_SERVICE_AUTOTUNE
          case ServiceItem::AUTOTUNE:
            state_.service_autotune_last_update_ms = 0u;
            state_.service_view = kServiceViewAutoTunePre;
            renderService_();
            return;
#endif
          default:
            break;
        }

        showInvalidKey_();
        return;
      }

      showInvalidKey_();
      return;
    }
#endif

// AUTOTUNE state (Phase 8).
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE
    case SystemState::AUTOTUNE: {
      if (pidController.isAutoTuneComplete()) {
        if (key == KeypadInput::Key::OK) {
          state_.service_autotune_page ^= 1u;
          renderAutoTune_();
          return;
        }

        if (key == KeypadInput::Key::START) {
          float kp = 0.0f;
          float ki = 0.0f;
          float kd = 0.0f;
          if (pidController.getAutoTuneResults(kp, ki, kd)) {
            pidController.setTunings(kp, ki, kd);
            eepromStore.requestSavePIDGains(kp, ki, kd);
            eepromStore.update(now);
          }

          pidController.endAutoTuneSession();
          state_.service_view = kServiceViewMenu;
          transitionTo(SystemState::SERVICE);
          state_.invalid_key_until_ms = now + kInvalidKeyMsgMs;
          lcd.setCursor(0, 3);
          lcd.print(F("SAVED               "));
          return;
        }

        return;
      }

      // In progress: only STOP aborts (handled above).
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
