#include "app.h"

#include <Arduino.h>
#include <pgmspace.h>

#include "config_build.h"
#if defined(ESP32)
#include <esp_ota_ops.h>
#endif
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_FACTORY_RESET
#if defined(__AVR__)
#include <avr/wdt.h>
#endif
#endif
#include "ds18b20_sensor.h"
#include "drum_control.h"
#include "faults.h"
#include "heater_control.h"
#include "pid_control.h"
#include "ui_lcd.h"

#if ENABLE_CYCLE_EXECUTION ||                                                                                       \
    (ENABLE_SERVICE_MENU &&                                                                                          \
     (ENABLE_SERVICE_FOPDT_ID || ENABLE_SERVICE_AUTOTUNE || ENABLE_SERVICE_FACTORY_RESET || ENABLE_SERVICE_IO_TEST))
#include "io_abstraction.h"
#endif
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_FOPDT_ID
#include "fopdt_model.h"
#endif

extern UILCD lcd;
extern EEPROMStore eepromStore;
extern DS18B20Sensor tempSensor;
extern DrumControl drumControl;
extern HeaterControl heaterControl;
extern PIDControl pidController;
extern FaultManager faultMgr;

#if ENABLE_SETTINGS_MENU
extern EEPROMStore::Settings g_settings;
#endif

#if ENABLE_CYCLE_EXECUTION ||                                                                                       \
    (ENABLE_SERVICE_MENU &&                                                                                          \
     (ENABLE_SERVICE_FOPDT_ID || ENABLE_SERVICE_AUTOTUNE || ENABLE_SERVICE_FACTORY_RESET || ENABLE_SERVICE_IO_TEST))
extern IOAbstraction io;
#endif
#if ENABLE_SERVICE_MENU
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

namespace
{
  constexpr uint32_t kBootScreenDurationMs = 3000;
  constexpr uint32_t kInvalidKeyMsgMs = 800;
  constexpr uint32_t kIdleTempRefreshMs = 1000;

  constexpr uint32_t kEntrySeqTimeoutMs = 5000;

  constexpr uint8_t kParamModeManual = 0u;
  constexpr uint8_t kParamModeAuto = 1u;

  constexpr uint8_t kAutoFlagTempModified = 1u << 0;
  constexpr uint8_t kAutoFlagTimeModified = 1u << 1;
  constexpr uint8_t kAutoFlagEditing = 1u << 2;
  constexpr uint8_t kAutoFlagFieldTime = 1u << 3; // 0=temp, 1=duration

#if ENABLE_CYCLE_EXECUTION
  constexpr uint8_t kManualViewTemp = 0u;
  constexpr uint8_t kManualViewDuration = 1u;

  constexpr uint32_t kStartDelayMs = 1000u;
  constexpr uint32_t kRunUiRefreshMs = 1000u;
  constexpr uint32_t kPauseTimeoutMs = 30ul * 60ul * 1000ul;
  constexpr uint32_t kPauseTimeoutMsgMs = 1500u;

  constexpr uint32_t kCooldownMaxMs = 15ul * 60ul * 1000ul;

  constexpr uint32_t kCycleCompleteSummaryMs = 10ul * 1000ul;
  constexpr uint32_t kAntiCreaseMaxMs = 2ul * 60ul * 60ul * 1000ul;
  constexpr uint32_t kAntiCreaseTumbleEveryMs = 5ul * 60ul * 1000ul;
  constexpr uint32_t kAntiCreaseTumbleDurationMs = 10ul * 1000ul;
#endif

#if ENABLE_SETTINGS_MENU
  constexpr uint8_t kTempUnitC = 0u;
  constexpr uint8_t kTempUnitF = 1u;
#endif

#if ENABLE_PROGRAM_EDITOR
  constexpr uint8_t kProgFieldName = 0u;
  constexpr uint8_t kProgFieldTemp = 1u;
  constexpr uint8_t kProgFieldTime = 2u;
  constexpr uint8_t kProgFieldFwd = 3u;
  constexpr uint8_t kProgFieldRev = 4u;
  constexpr uint8_t kProgFieldStop = 5u;
  constexpr uint8_t kProgFieldDuty = 6u;
  constexpr uint8_t kProgFieldCount = 7u;
#endif

  // Small LCD formatting helpers (defined later in this file).
  void lcdPrintU8_2_(UILCD &out, uint8_t value);
  void lcdPrintU16_NoLeading_(UILCD &out, uint16_t value);
  void lcdPadSpacesToEol_(UILCD &out, uint8_t printed_cols);

#if ENABLE_SETTINGS_MENU
  constexpr uint8_t kSettingsViewMenu = 0u;
  constexpr uint8_t kSettingsViewTempUnits = 1u;
  constexpr uint8_t kSettingsViewSound = 2u;
#if ENABLE_PROGRAM_EDITOR
  constexpr uint8_t kSettingsViewProgramList = 3u;
  constexpr uint8_t kSettingsViewProgramEdit = 4u;
  constexpr uint8_t kSettingsViewProgramEditConfirm = 5u;
#endif
#endif

#if ENABLE_SETTINGS_MENU
  uint8_t getTempUnit_()
  {
    return (g_settings.temp_unit == 0u) ? kTempUnitC : kTempUnitF;
  }

  char getTempUnitChar_()
  {
    return (getTempUnit_() == kTempUnitF) ? 'F' : 'C';
  }

  uint8_t cToF_u8(uint8_t c)
  {
    // °F = (°C × 9/5) + 32, rounded to nearest integer.
    return static_cast<uint8_t>(((static_cast<uint16_t>(c) * 9u) + 2u) / 5u + 32u);
  }

  uint8_t fToC_u8(uint8_t f)
  {
    // °C = (°F - 32) × 5/9, rounded to nearest integer.
    if (f <= 32u)
    {
      return 0u;
    }
    const uint16_t x = static_cast<uint16_t>(f - 32u);
    return static_cast<uint8_t>(((x * 5u) + 4u) / 9u);
  }
#endif

  uint8_t wrapStepU8(uint8_t value, uint8_t step, uint8_t min_v, uint8_t max_v, bool inc)
  {
    if (min_v > max_v || step == 0u)
    {
      return value;
    }
    if (inc)
    {
      const uint16_t next = static_cast<uint16_t>(value) + step;
      return (next > max_v) ? min_v : static_cast<uint8_t>(next);
    }
    if (value <= min_v)
    {
      return max_v;
    }
    const uint16_t prev = static_cast<uint16_t>(value) - step;
    return (prev < min_v) ? max_v : static_cast<uint8_t>(prev);
  }

  uint8_t digitsU16_(uint16_t value)
  {
    if (value >= 10000u)
      return 5u;
    if (value >= 1000u)
      return 4u;
    if (value >= 100u)
      return 3u;
    if (value >= 10u)
      return 2u;
    return 1u;
  }

#if ENABLE_SERVICE_MENU
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
#if ENABLE_SERVICE_MEMORY_INFO
  constexpr uint8_t kServiceViewMemoryInfo = 9u;
  constexpr uint32_t kMemoryInfoRefreshMs = 1000u;
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

#if ENABLE_SERVICE_FAULT_HISTORY
  constexpr uint8_t kServiceViewFaultHistory = 8u;
#endif

#if ENABLE_SERVICE_FACTORY_RESET
  constexpr uint8_t kServiceViewFactoryReset = 7u;
#endif

  enum class ServiceItem : uint8_t
  {
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
#if ENABLE_SERVICE_MEMORY_INFO
    MEMORY_INFO,
#endif
#if ENABLE_SERVICE_FAULT_HISTORY
    FAULT_HISTORY,
#endif
#if ENABLE_SERVICE_FOPDT_ID
    FOPDT_ID,
#endif
#if ENABLE_SERVICE_AUTOTUNE
    AUTOTUNE,
#endif
#if ENABLE_SERVICE_FACTORY_RESET
    FACTORY_RESET,
#endif
  };

  // NOTE: The AVR toolchain defaults to C++11, where constexpr functions are
  // restricted to a single return statement. Keep this as a pure constant
  // expression so it works under C++11 without extra init code.
  constexpr uint8_t kServiceMenuItems =
      static_cast<uint8_t>(ENABLE_SERVICE_DRUM_TEST + ENABLE_SERVICE_HEATER_TEST + ENABLE_SERVICE_PID_VIEW +
                           ENABLE_SERVICE_IO_TEST + ENABLE_SERVICE_MEMORY_INFO + ENABLE_SERVICE_FAULT_HISTORY +
                           ENABLE_SERVICE_FOPDT_ID + ENABLE_SERVICE_AUTOTUNE + ENABLE_SERVICE_FACTORY_RESET);
  static_assert(kServiceMenuItems > 0u, "Service menu enabled but no service tools enabled");

  ServiceItem serviceMenuItemByIndex_(uint8_t idx)
  {
#if ENABLE_SERVICE_DRUM_TEST
    if (idx-- == 0u)
      return ServiceItem::DRUM_TEST;
#endif
#if ENABLE_SERVICE_HEATER_TEST
    if (idx-- == 0u)
      return ServiceItem::HEATER_TEST;
#endif
#if ENABLE_SERVICE_PID_VIEW
    if (idx-- == 0u)
      return ServiceItem::PID_VIEW;
#endif
#if ENABLE_SERVICE_IO_TEST
    if (idx-- == 0u)
      return ServiceItem::IO_TEST;
#endif
#if ENABLE_SERVICE_MEMORY_INFO
    if (idx-- == 0u)
      return ServiceItem::MEMORY_INFO;
#endif
#if ENABLE_SERVICE_FAULT_HISTORY
    if (idx-- == 0u)
      return ServiceItem::FAULT_HISTORY;
#endif
#if ENABLE_SERVICE_FOPDT_ID
    if (idx-- == 0u)
      return ServiceItem::FOPDT_ID;
#endif
#if ENABLE_SERVICE_AUTOTUNE
    if (idx-- == 0u)
      return ServiceItem::AUTOTUNE;
#endif
#if ENABLE_SERVICE_FACTORY_RESET
    if (idx-- == 0u)
      return ServiceItem::FACTORY_RESET;
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
#elif ENABLE_SERVICE_MEMORY_INFO
    return ServiceItem::MEMORY_INFO;
#else
#if ENABLE_SERVICE_FAULT_HISTORY
    return ServiceItem::FAULT_HISTORY;
#else
#if ENABLE_SERVICE_FOPDT_ID
    return ServiceItem::FOPDT_ID;
#else
#if ENABLE_SERVICE_AUTOTUNE
    return ServiceItem::AUTOTUNE;
#else
    return ServiceItem::FACTORY_RESET;
#endif
#endif
#endif
#endif
  }

#if ENABLE_SERVICE_HEATER_TEST || ENABLE_SERVICE_FOPDT_ID || ENABLE_SERVICE_AUTOTUNE
  void formatDuty3Chars(uint8_t duty, char out[4])
  {
    if (duty > 100u)
    {
      duty = 100u;
    }
    if (duty == 100u)
    {
      out[0] = '1';
      out[1] = '0';
      out[2] = '0';
      out[3] = '\0';
      return;
    }

    out[0] = ' ';
    if (duty >= 10u)
    {
      out[1] = static_cast<char>('0' + (duty / 10u));
      out[2] = static_cast<char>('0' + (duty % 10u));
    }
    else
    {
      out[1] = ' ';
      out[2] = static_cast<char>('0' + duty);
    }
    out[3] = '\0';
  }
#endif

#if ENABLE_SERVICE_PID_VIEW
  uint8_t clampFloatToU8(float value, uint8_t max_v)
  {
    if (value <= 0.0f)
    {
      return 0u;
    }
    const float vmax = static_cast<float>(max_v);
    if (value >= vmax)
    {
      return max_v;
    }
    return static_cast<uint8_t>(value + 0.5f);
  }

  void clearLine20(char line[21])
  {
    for (uint8_t i = 0u; i < 20u; i++)
    {
      line[i] = ' ';
    }
    line[20] = '\0';
  }
#endif

#if ENABLE_SERVICE_PID_VIEW || ENABLE_SERVICE_FOPDT_ID || ENABLE_SERVICE_AUTOTUNE
  uint16_t scaleClampU16(float value, float scale, uint16_t max_scaled)
  {
    if (value <= 0.0f)
    {
      return 0u;
    }
    const float scaled = (value * scale) + 0.5f;
    const float max_f = static_cast<float>(max_scaled);
    if (scaled >= max_f)
    {
      return max_scaled;
    }
    return static_cast<uint16_t>(scaled);
  }
#endif

#if ENABLE_SERVICE_PID_VIEW
  void writeU8_3(char *dest, uint8_t value)
  {
    dest[0] = static_cast<char>('0' + (value / 100u));
    dest[1] = static_cast<char>('0' + ((value / 10u) % 10u));
    dest[2] = static_cast<char>('0' + (value % 10u));
  }
#endif

#if ENABLE_SERVICE_PID_VIEW || ENABLE_SERVICE_FOPDT_ID || ENABLE_SERVICE_AUTOTUNE
  void writeU16Whole3(char *dest, uint16_t whole)
  {
    if (whole >= 100u)
    {
      dest[0] = static_cast<char>('0' + ((whole / 100u) % 10u));
      dest[1] = static_cast<char>('0' + ((whole / 10u) % 10u));
      dest[2] = static_cast<char>('0' + (whole % 10u));
      return;
    }
    if (whole >= 10u)
    {
      dest[0] = ' ';
      dest[1] = static_cast<char>('0' + ((whole / 10u) % 10u));
      dest[2] = static_cast<char>('0' + (whole % 10u));
      return;
    }
    dest[0] = ' ';
    dest[1] = ' ';
    dest[2] = static_cast<char>('0' + (whole % 10u));
  }

  void writeU16_1dp5(char *dest, uint16_t scaled10)
  {
    const uint16_t whole = static_cast<uint16_t>(scaled10 / 10u);
    const uint8_t frac = static_cast<uint8_t>(scaled10 % 10u);
    writeU16Whole3(dest, whole);
    dest[3] = '.';
    dest[4] = static_cast<char>('0' + (frac % 10u));
  }

  void writeU16_2dp6(char *dest, uint16_t scaled100)
  {
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

void AppStateMachine::init()
{
  state_.current_state = SystemState::BOOT;
  state_.previous_state = SystemState::BOOT;
  state_.state_entry_time_ms = millis();

  state_.menu_selection = 0;

  state_.entry_seq = 0u;
  state_.entry_seq_start_ms = 0u;

  state_.auto_program_index = 0u;
  EEPROMStore::getDefaultProgram(0u, state_.auto_program);
  state_.auto_temp_c = state_.auto_program.temp_setpoint;
  state_.auto_duration_min = state_.auto_program.duration_min;
  state_.auto_flags = 0u;
  state_.param_mode = kParamModeManual;

#if ENABLE_CYCLE_EXECUTION
  state_.manual_temp_c = 60u;
  state_.manual_duration_min = 30u;
  state_.manual_view = kManualViewTemp;

  state_.cycle_setpoint_c = 0u;
  state_.cycle_duration_min = 0u;
  state_.cycle_duty_limit = 100u;
  state_.cycle_last_ui_ms = 0u;
  state_.cycle_orig_setpoint_c = 0u;
  state_.cycle_orig_duration_min = 0u;
  state_.heat_start_ms = 0u;
  state_.heat_pause_total_ms = 0u;
  state_.cooldown_start_ms = 0u;
  state_.cooldown_pause_total_ms = 0u;
  state_.pause_start_ms = 0u;
  state_.paused_from_state = SystemState::IDLE;
  state_.pause_timeout_msg_start_ms = 0u;
  state_.pause_timeout_shown = 0u;
  state_.cycle_edit_active = 0u;
  state_.cycle_edit_field = 0u;
  state_.cycle_temp_modified = 0u;
  state_.cycle_time_modified = 0u;
  state_.cycle_edit_temp_c = 0u;
  state_.cycle_edit_duration_min = 0u;
  state_.cycle_total_time_ms = 0u;
  state_.anticrease_start_ms = 0u;
  state_.tumble_cycle_start_ms = 0u;
  state_.tumble_start_ms = 0u;
  state_.anticrease_page = 0u;
  state_.tumble_state = 0u;
#endif

#if ENABLE_SETTINGS_MENU
  state_.settings_menu_selection = 0u;
  state_.settings_view = kSettingsViewMenu;
#if ENABLE_PROGRAM_EDITOR
  state_.program_list_selection = 0u;
  state_.program_edit_index = 0u;
  EEPROMStore::getDefaultProgram(0u, state_.program_edit);
  state_.program_edit_field = 0u;
  state_.program_name_cursor = 0u;
#endif
#endif

#if ENABLE_SERVICE_MENU
  state_.service_menu_selection = 0u;
  state_.service_view = kServiceViewMenu;
#if ENABLE_SERVICE_FAULT_HISTORY
  state_.fault_history_index = 0u;
  state_.fault_history_page = 0u;
#endif
#if ENABLE_SERVICE_IO_TEST
  state_.io_test_page = 0u;
  state_.io_test_selection = 0u;
  state_.io_test_out_mask = 0u;
  state_.io_test_last_update_ms = 0u;
  state_.io_test_last_sig = 0u;
#endif
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
#if ENABLE_SERVICE_MEMORY_INFO
  state_.service_mem_page = 0u;
  state_.service_mem_last_update_ms = 0u;
#endif
#endif

  state_.invalid_key_until_ms = 0;
  state_.last_temp_display_ms = 0;
  state_.last_temp_valid = 0;

  onEnter_(SystemState::BOOT);
}

SystemState AppStateMachine::getCurrentState() const
{
  return state_.current_state;
}

#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_IO_TEST
bool AppStateMachine::isIoTestActive() const
{
  return (state_.current_state == SystemState::SERVICE) && (state_.service_view == kServiceViewIoTest);
}

void AppStateMachine::getIoTestOutputs(bool& heater_on, bool& motor_fwd_on, bool& motor_rev_on, bool& aux_on) const
{
  heater_on = false;
  motor_fwd_on = false;
  motor_rev_on = false;
  aux_on = false;

  if (!isIoTestActive())
  {
    return;
  }

  const uint8_t mask = state_.io_test_out_mask;
  heater_on = (mask & 0x01u) != 0u;
  motor_fwd_on = (mask & 0x02u) != 0u;
  motor_rev_on = (mask & 0x04u) != 0u;
  aux_on = (mask & 0x08u) != 0u;
}
#endif

bool AppStateMachine::canTransition_(SystemState from, SystemState to) const
{
  if (from == to)
  {
    return false;
  }

  // Safety: any state may transition to FAULT (Req 21.9).
  if (to == SystemState::FAULT)
  {
    return true;
  }

  switch (from)
  {
  case SystemState::BOOT:
    return (to == SystemState::IDLE) || (to == SystemState::FAULT);
  case SystemState::FAULT:
    // Latched fault: only allow returning to IDLE after operator acknowledgement.
    return (to == SystemState::IDLE);
  case SystemState::IDLE:
#if ENABLE_SERVICE_MENU
    return (to == SystemState::PROGRAM_SELECT) || (to == SystemState::PARAM_EDIT) ||
#if ENABLE_SETTINGS_MENU
           (to == SystemState::SETTINGS) ||
#endif
           (to == SystemState::SERVICE) || (to == SystemState::FAULT);
#else
    return (to == SystemState::PROGRAM_SELECT) || (to == SystemState::PARAM_EDIT) ||
#if ENABLE_SETTINGS_MENU
           (to == SystemState::SETTINGS) ||
#endif
           (to == SystemState::FAULT);
#endif
  case SystemState::PROGRAM_SELECT:
    return (to == SystemState::IDLE) || (to == SystemState::PARAM_EDIT) || (to == SystemState::FAULT);
  case SystemState::PARAM_EDIT:
    return (to == SystemState::IDLE) || (to == SystemState::PROGRAM_SELECT) || (to == SystemState::READY) ||
#if ENABLE_CYCLE_EXECUTION
           (to == SystemState::START_DELAY) ||
#endif
           (to == SystemState::FAULT);
#if ENABLE_SETTINGS_MENU
  case SystemState::SETTINGS:
    return (to == SystemState::IDLE) || (to == SystemState::FAULT);
#endif
  case SystemState::READY:
    return (to == SystemState::IDLE) ||
#if ENABLE_CYCLE_EXECUTION
           (to == SystemState::START_DELAY) ||
#endif
           (to == SystemState::FAULT);
#if ENABLE_CYCLE_EXECUTION
  case SystemState::START_DELAY:
    return (to == SystemState::RUNNING_HEAT) || (to == SystemState::IDLE) || (to == SystemState::FAULT);
  case SystemState::RUNNING_HEAT:
    return (to == SystemState::RUNNING_COOLDOWN) || (to == SystemState::PAUSED) || (to == SystemState::IDLE) ||
           (to == SystemState::FAULT);
  case SystemState::RUNNING_COOLDOWN:
    return (to == SystemState::ANTI_CREASE) || (to == SystemState::PAUSED) || (to == SystemState::IDLE) ||
           (to == SystemState::FAULT);
  case SystemState::PAUSED:
    return (to == SystemState::RUNNING_HEAT) || (to == SystemState::RUNNING_COOLDOWN) || (to == SystemState::IDLE) ||
           (to == SystemState::FAULT);
  case SystemState::ANTI_CREASE:
    return (to == SystemState::IDLE) || (to == SystemState::FAULT);
#endif
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

void AppStateMachine::onEnter_(SystemState new_state)
{
  switch (new_state)
  {
  case SystemState::BOOT:
    lcd.showBootScreen();
    break;
  case SystemState::FAULT:
    drumControl.stop();
    heaterControl.disable();

#if ENABLE_CYCLE_EXECUTION || (ENABLE_SERVICE_MENU && (ENABLE_SERVICE_FOPDT_ID || ENABLE_SERVICE_AUTOTUNE || ENABLE_SERVICE_FACTORY_RESET))
    // Safety redundancy: force outputs OFF on FAULT entry.
    io.emergencyStop();
#endif

    renderFault_();
    break;
  case SystemState::IDLE:
    // Safety: IDLE must always be outputs-OFF.
    drumControl.stop();
    heaterControl.disable();
    lcd.showMainMenu(state_.menu_selection);
    state_.last_temp_valid = tempSensor.isValid() ? 1u : 0u;
    lcd.showTemperature(tempSensor.getTemperature(), state_.last_temp_valid != 0u);
    state_.last_temp_display_ms = millis();
    break;
#if ENABLE_SETTINGS_MENU
  case SystemState::SETTINGS:
    state_.settings_view = kSettingsViewMenu;
    renderSettings_();
    break;
#endif
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
#if ENABLE_CYCLE_EXECUTION
  case SystemState::START_DELAY:
    renderStartDelay_();
    break;

  case SystemState::RUNNING_HEAT:
  {
    const uint32_t now_ms = millis();

    if (state_.previous_state == SystemState::START_DELAY)
    {
      // Fresh cycle start: capture original parameters (Req 13 limits).
      state_.cycle_orig_setpoint_c = state_.cycle_setpoint_c;
      state_.cycle_orig_duration_min = state_.cycle_duration_min;
      state_.cycle_temp_modified = 0u;
      state_.cycle_time_modified = 0u;
      state_.cycle_edit_active = 0u;
      state_.cycle_edit_field = 0u;

      // Phase timing baselines.
      state_.heat_start_ms = now_ms;
      state_.heat_pause_total_ms = 0u;
      state_.cooldown_start_ms = 0u;
      state_.cooldown_pause_total_ms = 0u;
      state_.cycle_total_time_ms = 0u;
    }

    // Phase 6: PID bumpless transfer occurs on RUNNING_HEAT entry.
    // Prepare PID and heater gating for (re)entry.
    pidController.setOutputLimits(0.0f, 0.0f); // force output/integral to 0
    pidController.setOutputLimits(0.0f, static_cast<float>((state_.cycle_duty_limit > 100u) ? 100u
                                                                                            : state_.cycle_duty_limit));
    pidController.setSetpoint(static_cast<float>(state_.cycle_setpoint_c));
    pidController.reset();

    heaterControl.disable();
    heaterControl.setDutyCycle(0.0f);
    heaterControl.enable();

    // Drum pattern:
    //  - AUTO: from selected program
    //  - MANUAL: fixed Mixed_Load defaults (Req 12B)
    if (state_.param_mode == kParamModeAuto)
    {
      drumControl.setPattern(state_.auto_program.fwd_time_s, state_.auto_program.rev_time_s,
                             state_.auto_program.stop_time_s);
    }
    else
    {
      drumControl.setPattern(50u, 50u, 5u);
    }
    drumControl.start();

    state_.cycle_last_ui_ms = 0u;
    renderRunningHeat_();
    break;
  }

  case SystemState::RUNNING_COOLDOWN:
  {
    const uint32_t now_ms = millis();

    heaterControl.setDutyCycle(0.0f);
    heaterControl.disable();

    // Start cooldown timing on first entry.
    if (state_.previous_state == SystemState::RUNNING_HEAT)
    {
      state_.cooldown_start_ms = now_ms;
      state_.cooldown_pause_total_ms = 0u;
    }

    // Keep drum turning during cooldown (Req 10).
    if (!drumControl.isRunning())
    {
      drumControl.start();
    }

    state_.cycle_last_ui_ms = 0u;
    state_.cycle_edit_active = 0u;
    renderRunningCooldown_();
    break;
  }

  case SystemState::PAUSED:
    // Pause is only reachable from RUNNING_HEAT/COOLDOWN (Req 26).
    state_.paused_from_state = state_.previous_state;
    state_.pause_start_ms = millis();
    state_.pause_timeout_shown = 0u;
    state_.pause_timeout_msg_start_ms = 0u;

    // Force outputs OFF.
    heaterControl.setDutyCycle(0.0f);
    heaterControl.disable();
    drumControl.stop();

    // Prevent output jump on resume: clamp PID output to 0 while paused.
    pidController.setOutputLimits(0.0f, 0.0f);
    pidController.reset();

    renderPaused_();
    break;

  case SystemState::ANTI_CREASE:
  {
    const uint32_t now_ms = millis();

    // Heater must remain OFF (Req 11).
    heaterControl.setDutyCycle(0.0f);
    heaterControl.disable();

    // Anti-crease starts from a stopped drum, then tumbles periodically.
    drumControl.stop();

    // Compute total time (heat total + cooldown elapsed), pause-aware.
    const uint32_t heat_total_ms = static_cast<uint32_t>(state_.cycle_duration_min) * 60ul * 1000ul;
    uint32_t cooldown_elapsed_ms = 0u;
    if (state_.cooldown_start_ms != 0u)
    {
      cooldown_elapsed_ms = now_ms - state_.cooldown_start_ms - state_.cooldown_pause_total_ms;
    }
    state_.cycle_total_time_ms = heat_total_ms + cooldown_elapsed_ms;

    state_.anticrease_start_ms = now_ms;
    state_.anticrease_page = 0u; // summary first
    state_.tumble_state = 0u;
    state_.tumble_cycle_start_ms = now_ms;
    state_.tumble_start_ms = 0u;

    state_.cycle_last_ui_ms = 0u;
    renderCycleCompleteSummary_();
    break;
  }
#else
  case SystemState::RUNNING_HEAT:
    // Cycle execution compiled out.
    pidController.reset();
    heaterControl.enable();
    break;
#endif
  case SystemState::PROGRAM_SELECT:
    state_.auto_program_index = 0u;
    (void)eepromStore.loadProgram(state_.auto_program_index, state_.auto_program);
    state_.auto_temp_c = state_.auto_program.temp_setpoint;
    state_.auto_duration_min = state_.auto_program.duration_min;
    state_.auto_flags = 0u;
    renderProgramSelect_();
    break;
  case SystemState::PARAM_EDIT:
    if (state_.param_mode == kParamModeAuto)
    {
      state_.auto_flags &= static_cast<uint8_t>(~kAutoFlagEditing);
      renderAutoParamReview_();
    }
    else
    {
#if ENABLE_CYCLE_EXECUTION
      // Req 12B: Manual mode starts at temperature input with defaults each entry.
      if (state_.previous_state == SystemState::IDLE)
      {
        state_.manual_temp_c = 60u;
        state_.manual_duration_min = 30u;
      }
      state_.manual_view = kManualViewTemp;
      renderManualTemp_();
#else
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("MANUAL MODE"));
      lcd.setCursor(0, 2);
      lcd.print(F("DISABLED"));
      lcd.setCursor(0, 3);
      lcd.print(F("B:BACK"));
#endif
    }
    break;
  case SystemState::READY:
    if (state_.param_mode == kParamModeAuto)
    {
      // Should not be entered in current AUTO flow; keep a minimal safe screen.
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("READY"));
      lcd.setCursor(0, 3);
      lcd.print(F("B:BACK"));
    }
    else
    {
#if ENABLE_CYCLE_EXECUTION
      renderManualConfirm_();
#else
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("MANUAL MODE"));
      lcd.setCursor(0, 2);
      lcd.print(F("DISABLED"));
      lcd.setCursor(0, 3);
      lcd.print(F("B:BACK"));
#endif
    }
    break;
  default:
    break;
  }
}

void AppStateMachine::onExit_(SystemState old_state)
{
#if ENABLE_SERVICE_MENU
  if (old_state == SystemState::SERVICE)
  {
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
  if (old_state == SystemState::AUTOTUNE)
  {
    // Safety: never leave AUTOTUNE with heater control active.
    if (pidController.isAutoTuning() || pidController.isAutoTuneComplete())
    {
      pidController.abortAutoTune();
    }
  }
#endif
#else
  (void)old_state;
#endif
}

void AppStateMachine::transitionTo(SystemState new_state)
{
  const SystemState from = state_.current_state;
  if (!canTransition_(from, new_state))
  {
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

void AppStateMachine::renderFault_()
{
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(F("** FAULT **"));
  lcdPadSpacesToEol_(lcd, 11u);

  lcd.setCursor(0, 1);
  const __FlashStringHelper *msg = faultMgr.getFaultMessage();
  lcd.print(msg);

  // Pad the remainder of the line (Flash string length is unknown at compile time).
  const char *p = reinterpret_cast<const char *>(msg);
  uint8_t len = 0u;
  while (len < 20u)
  {
    const char c = static_cast<char>(pgm_read_byte(p + len));
    if (c == '\0')
    {
      break;
    }
    len++;
  }
  lcdPadSpacesToEol_(lcd, len);

  lcd.setCursor(0, 2);
  lcd.print(F("CLOSE TO CONTINUE"));
  lcdPadSpacesToEol_(lcd, 17u);

  lcd.setCursor(0, 3);
  lcd.print(F("PRESS B TO CLEAR"));
  lcdPadSpacesToEol_(lcd, 16u);
}

void AppStateMachine::restoreScreen_()
{
  // Re-render the current screen without re-running entry actions that may
  // have side effects (e.g., restarting control loops).
  switch (state_.current_state)
  {
  case SystemState::BOOT:
    lcd.showBootScreen();
    break;
  case SystemState::IDLE:
    lcd.showMainMenu(state_.menu_selection);
    state_.last_temp_valid = tempSensor.isValid() ? 1u : 0u;
    lcd.showTemperature(tempSensor.getTemperature(), state_.last_temp_valid != 0u);
    break;
  case SystemState::PROGRAM_SELECT:
    renderProgramSelect_();
    break;
  case SystemState::PARAM_EDIT:
    if (state_.param_mode == kParamModeAuto)
    {
      const bool editing = (state_.auto_flags & kAutoFlagEditing) != 0u;
      editing ? renderAutoParamEdit_() : renderAutoParamReview_();
    }
    else
    {
#if ENABLE_CYCLE_EXECUTION
      (state_.manual_view == kManualViewDuration) ? renderManualDuration_() : renderManualTemp_();
#endif
    }
    break;
  case SystemState::READY:
#if ENABLE_CYCLE_EXECUTION
    if (state_.param_mode == kParamModeManual)
    {
      renderManualConfirm_();
    }
#endif
    break;
#if ENABLE_SETTINGS_MENU
  case SystemState::SETTINGS:
    renderSettings_();
    break;
#endif
#if ENABLE_SERVICE_MENU
  case SystemState::SERVICE:
    renderService_();
    break;
#if ENABLE_SERVICE_AUTOTUNE
  case SystemState::AUTOTUNE:
    renderAutoTune_();
    break;
#endif
#endif
#if ENABLE_CYCLE_EXECUTION
  case SystemState::START_DELAY:
    renderStartDelay_();
    break;
  case SystemState::RUNNING_HEAT:
    if (state_.cycle_edit_active != 0u)
    {
      renderCycleEdit_();
    }
    else
    {
      renderRunningHeat_();
      state_.cycle_last_ui_ms = 0u;
      updateRunningHeatUi_();
    }
    break;
  case SystemState::RUNNING_COOLDOWN:
    renderRunningCooldown_();
    state_.cycle_last_ui_ms = 0u;
    updateRunningCooldownUi_();
    break;
  case SystemState::PAUSED:
    renderPaused_();
    break;
  case SystemState::ANTI_CREASE:
    if (state_.anticrease_page == 0u)
    {
      renderCycleCompleteSummary_();
    }
    else
    {
      renderAntiCreaseActive_();
    }
    break;
#endif
  case SystemState::FAULT:
    renderFault_();
    break;
  default:
    break;
  }
}

void AppStateMachine::showInvalidKey_()
{
  state_.invalid_key_until_ms = millis() + kInvalidKeyMsgMs;
  lcd.setCursor(0, 3);
  lcd.print(F("INVALID KEY         "));
}

namespace
{
  void lcdPrintU8_2_(UILCD &out, uint8_t value)
  {
    out.print(static_cast<char>('0' + ((value / 10u) % 10u)));
    out.print(static_cast<char>('0' + (value % 10u)));
  }

  void lcdPrintU16_NoLeading_(UILCD &out, uint16_t value)
  {
    if (value >= 10000u)
    {
      out.print(static_cast<char>('0' + (value / 10000u)));
      out.print(static_cast<char>('0' + ((value / 1000u) % 10u)));
      out.print(static_cast<char>('0' + ((value / 100u) % 10u)));
      out.print(static_cast<char>('0' + ((value / 10u) % 10u)));
      out.print(static_cast<char>('0' + (value % 10u)));
      return;
    }
    if (value >= 1000u)
    {
      out.print(static_cast<char>('0' + (value / 1000u)));
      out.print(static_cast<char>('0' + ((value / 100u) % 10u)));
      out.print(static_cast<char>('0' + ((value / 10u) % 10u)));
      out.print(static_cast<char>('0' + (value % 10u)));
      return;
    }
    if (value >= 100u)
    {
      out.print(static_cast<char>('0' + ((value / 100u) % 10u)));
      out.print(static_cast<char>('0' + ((value / 10u) % 10u)));
      out.print(static_cast<char>('0' + (value % 10u)));
      return;
    }
    if (value >= 10u)
    {
      out.print(static_cast<char>('0' + ((value / 10u) % 10u)));
      out.print(static_cast<char>('0' + (value % 10u)));
      return;
    }
    out.print(static_cast<char>('0' + (value % 10u)));
  }

  void lcdPrintU16_3_(UILCD &out, uint16_t value)
  {
    if (value >= 100u)
    {
      out.print(static_cast<char>('0' + ((value / 100u) % 10u)));
      out.print(static_cast<char>('0' + ((value / 10u) % 10u)));
      out.print(static_cast<char>('0' + (value % 10u)));
      return;
    }
    if (value >= 10u)
    {
      out.print(' ');
      out.print(static_cast<char>('0' + ((value / 10u) % 10u)));
      out.print(static_cast<char>('0' + (value % 10u)));
      return;
    }
    out.print(' ');
    out.print(' ');
    out.print(static_cast<char>('0' + (value % 10u)));
  }

  void lcdPadSpacesToEol_(UILCD &out, uint8_t printed_cols)
  {
    constexpr uint8_t kCols = 20u;
    if (printed_cols >= kCols)
    {
      return;
    }
    for (uint8_t i = printed_cols; i < kCols; i++)
    {
      out.print(' ');
    }
  }

#if ENABLE_PROGRAM_EDITOR
  char cycleNameChar_(char c, bool inc)
  {
    // Allowed: space, A-Z, 0-9. Defaults unknown chars to space.
    if (c == '\0')
    {
      c = ' ';
    }
    if (c == ' ')
    {
      return inc ? 'A' : '9';
    }
    if (c >= 'A' && c <= 'Z')
    {
      if (inc)
      {
        return (c == 'Z') ? '0' : static_cast<char>(c + 1);
      }
      return (c == 'A') ? ' ' : static_cast<char>(c - 1);
    }
    if (c >= '0' && c <= '9')
    {
      if (inc)
      {
        return (c == '9') ? ' ' : static_cast<char>(c + 1);
      }
      return (c == '0') ? 'Z' : static_cast<char>(c - 1);
    }
    return ' ';
  }
#endif
} // namespace

void AppStateMachine::renderProgramSelect_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("AUTO MODE           "));

  lcd.setCursor(0, 1);
  lcd.print(F("SELECT PROGRAM:     "));

  lcd.setCursor(0, 2);
  char line[21];
  for (uint8_t i = 0u; i < 20u; i++)
  {
    line[i] = ' ';
  }
  line[20] = '\0';

  line[0] = '>';
  line[1] = ' ';
  line[2] = static_cast<char>('1' + (state_.auto_program_index % EEPROMStore::PROGRAM_COUNT));
  line[3] = '.';
  line[4] = ' ';
  for (uint8_t i = 0u; i < 12u; i++)
  {
    const char c = state_.auto_program.name[i];
    if (c == '\0')
      break;
    line[5u + i] = c;
  }
  lcd.print(line);

  lcd.setCursor(0, 3);
  lcd.print(F("UP/DN OK:SELECT     "));
}

void AppStateMachine::renderAutoParamReview_()
{
  lcd.clear();

  // Line 1: Program name
  lcd.setCursor(0, 0);
  lcd.print(state_.auto_program.name);
  uint8_t name_len = 0u;
  for (uint8_t i = 0u; i < 12u; i++)
  {
    if (state_.auto_program.name[i] == '\0')
      break;
    name_len++;
  }
  lcdPadSpacesToEol_(lcd, name_len);

  // Line 2: TEMP
  lcd.setCursor(0, 1);
  lcd.print(F("TEMP: "));

  uint16_t t_disp = state_.auto_temp_c;
  char unit_char = 'C';
#if ENABLE_SETTINGS_MENU
  if (getTempUnit_() == kTempUnitF)
  {
    t_disp = cToF_u8(state_.auto_temp_c);
    unit_char = 'F';
  }
#endif
  lcdPrintU16_NoLeading_(lcd, t_disp);
  lcd.print(static_cast<char>(0xDF));
  lcd.print(unit_char);

  uint8_t col = static_cast<uint8_t>(6u + ((t_disp >= 100u) ? 3u : ((t_disp >= 10u) ? 2u : 1u)) + 2u);
  if ((state_.auto_flags & kAutoFlagTempModified) != 0u)
  {
    lcd.print(' ');
    lcd.print('*');
    col = static_cast<uint8_t>(col + 2u);
  }
  lcdPadSpacesToEol_(lcd, col);

  // Line 3: TIME
  lcd.setCursor(0, 2);
  lcd.print(F("TIME: "));
  lcdPrintU16_NoLeading_(lcd, state_.auto_duration_min);
  lcd.print(F(" MIN"));
  col = static_cast<uint8_t>(6u + ((state_.auto_duration_min >= 100u) ? 3u : 2u) + 4u);
  if ((state_.auto_flags & kAutoFlagTimeModified) != 0u)
  {
    lcd.print(' ');
    lcd.print('*');
    col = static_cast<uint8_t>(col + 2u);
  }
  lcdPadSpacesToEol_(lcd, col);

  // Line 4: Actions
  lcd.setCursor(0, 3);
  lcd.print(F("A:START  *:EDIT"));
  lcdPadSpacesToEol_(lcd, 15u);
}

void AppStateMachine::renderAutoParamEdit_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("EDIT PARAMETERS"));
  lcdPadSpacesToEol_(lcd, 15u);

  const bool field_time = (state_.auto_flags & kAutoFlagFieldTime) != 0u;

  // Line 2: TEMP
  lcd.setCursor(0, 1);
  lcd.print(F("TEMP: "));

  uint16_t t_disp = state_.auto_temp_c;
  char unit_char = 'C';
#if ENABLE_SETTINGS_MENU
  if (getTempUnit_() == kTempUnitF)
  {
    t_disp = cToF_u8(state_.auto_temp_c);
    unit_char = 'F';
  }
#endif

  if (!field_time)
    lcd.print('[');
  lcdPrintU16_NoLeading_(lcd, t_disp);
  if (!field_time)
    lcd.print(']');
  lcd.print(static_cast<char>(0xDF));
  lcd.print(unit_char);

  uint8_t digits = (t_disp >= 100u) ? 3u : ((t_disp >= 10u) ? 2u : 1u);
  uint8_t used = static_cast<uint8_t>(6u + digits + 2u + (field_time ? 0u : 2u));
  lcdPadSpacesToEol_(lcd, used);

  // Line 3: TIME
  lcd.setCursor(0, 2);
  lcd.print(F("TIME: "));
  if (field_time)
    lcd.print('[');
  lcdPrintU16_NoLeading_(lcd, state_.auto_duration_min);
  if (field_time)
    lcd.print(']');
  lcd.print(F(" MIN"));

  digits = (state_.auto_duration_min >= 100u) ? 3u : 2u;
  used = static_cast<uint8_t>(6u + digits + 4u + (field_time ? 2u : 0u));
  lcdPadSpacesToEol_(lcd, used);

  // Line 4: Hint
  lcd.setCursor(0, 3);
  lcd.print(F("LT/RT OK:SAVE"));
  lcdPadSpacesToEol_(lcd, 13u);
}

#if ENABLE_CYCLE_EXECUTION
void AppStateMachine::renderManualTemp_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("MANUAL MODE"));
  lcdPadSpacesToEol_(lcd, 11u);

  lcd.setCursor(0, 1);
  lcd.print(F("SET TEMPERATURE:"));
  lcdPadSpacesToEol_(lcd, 16u);

  lcd.setCursor(0, 2);
  lcd.print('[');
  uint16_t t_disp = state_.manual_temp_c;
  char unit_char = 'C';
#if ENABLE_SETTINGS_MENU
  if (getTempUnit_() == kTempUnitF)
  {
    t_disp = cToF_u8(state_.manual_temp_c);
    unit_char = 'F';
  }
#endif
  lcdPrintU16_NoLeading_(lcd, t_disp);
  lcd.print(']');
  lcd.print(static_cast<char>(0xDF));
  lcd.print(unit_char);
  lcdPadSpacesToEol_(lcd, static_cast<uint8_t>(4u + ((t_disp >= 100u) ? 3u : 2u)));

  lcd.setCursor(0, 3);
  lcd.print(F("UP/DN:CHG OK:NEXT"));
  lcdPadSpacesToEol_(lcd, 17u);
}

void AppStateMachine::renderManualDuration_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("MANUAL MODE"));
  lcdPadSpacesToEol_(lcd, 11u);

  lcd.setCursor(0, 1);
  lcd.print(F("SET DURATION:"));
  lcdPadSpacesToEol_(lcd, 13u);

  lcd.setCursor(0, 2);
  lcd.print('[');
  lcdPrintU16_NoLeading_(lcd, state_.manual_duration_min);
  lcd.print(F("] MIN"));
  const uint8_t d = (state_.manual_duration_min >= 100u) ? 3u : 2u;
  lcdPadSpacesToEol_(lcd, static_cast<uint8_t>(d + 6u));

  lcd.setCursor(0, 3);
  lcd.print(F("UP/DN:CHG OK:NEXT"));
  lcdPadSpacesToEol_(lcd, 17u);
}

void AppStateMachine::renderManualConfirm_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("MANUAL MODE"));
  lcdPadSpacesToEol_(lcd, 11u);

  lcd.setCursor(0, 1);
  lcd.print(F("TEMP: "));
  uint16_t t_disp = state_.manual_temp_c;
  char unit_char = 'C';
#if ENABLE_SETTINGS_MENU
  if (getTempUnit_() == kTempUnitF)
  {
    t_disp = cToF_u8(state_.manual_temp_c);
    unit_char = 'F';
  }
#endif
  lcdPrintU16_NoLeading_(lcd, t_disp);
  lcd.print(static_cast<char>(0xDF));
  lcd.print(unit_char);
  lcdPadSpacesToEol_(lcd, static_cast<uint8_t>(8u + ((t_disp >= 100u) ? 3u : 2u)));

  lcd.setCursor(0, 2);
  lcd.print(F("TIME: "));
  lcdPrintU16_NoLeading_(lcd, state_.manual_duration_min);
  lcd.print(F(" MIN"));
  lcdPadSpacesToEol_(lcd, static_cast<uint8_t>(10u + ((state_.manual_duration_min >= 100u) ? 3u : 2u)));

  // Requirement 12B.35: exact hint.
  lcd.setCursor(0, 3);
  lcd.print(F("A:START B:CANCEL"));
  lcdPadSpacesToEol_(lcd, 16u);
}

void AppStateMachine::renderStartDelay_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  if (state_.param_mode == kParamModeAuto)
  {
    lcd.print(F("AUTO: "));
    lcd.print(state_.auto_program.name);
  }
  else
  {
    lcd.print(F("MANUAL MODE"));
  }

  lcd.setCursor(0, 1);
  lcd.print(F("STARTING..."));

  lcd.setCursor(0, 3);
  lcd.print(F("B:STOP"));
}

void AppStateMachine::renderRunningHeat_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  if (state_.param_mode == kParamModeAuto)
  {
    lcd.print(F("AUTO: "));
    lcd.print(state_.auto_program.name);
  }
  else
  {
    lcd.print(F("MANUAL MODE"));
  }
}

void AppStateMachine::updateRunningHeatUi_()
{
  const uint32_t now = millis();

  // Fault detection is centralized in FaultManager (Phase 10).
  if (!tempSensor.isValid())
  {
    return;
  }
  const float pv_c = tempSensor.getTemperature();

  const uint32_t total_ms = static_cast<uint32_t>(state_.cycle_duration_min) * 60ul * 1000ul;
  const uint32_t elapsed_ms = (state_.heat_start_ms == 0u)
                                  ? 0u
                                  : (now - state_.heat_start_ms - state_.heat_pause_total_ms);

  if (elapsed_ms >= total_ms)
  {
    transitionTo(SystemState::RUNNING_COOLDOWN);
    return;
  }

  // Do not overwrite the in-cycle edit screen (but still enforce timing above).
  if (state_.cycle_edit_active != 0u)
  {
    return;
  }

  if (state_.cycle_last_ui_ms != 0u && (now - state_.cycle_last_ui_ms) < kRunUiRefreshMs)
  {
    return;
  }
  state_.cycle_last_ui_ms = now;

  // Remaining time (MM:SS).
  const uint32_t rem_ms = total_ms - elapsed_ms;
  const uint16_t rem_s = static_cast<uint16_t>(rem_ms / 1000ul);
  const uint16_t rem_min = static_cast<uint16_t>(rem_s / 60u);
  const uint8_t rem_sec = static_cast<uint8_t>(rem_s % 60u);

  // SP/PV display unit conversion (whole degrees).
  uint16_t sp_disp = state_.cycle_setpoint_c;
  int16_t pv_whole = static_cast<int16_t>(pv_c + ((pv_c >= 0.0f) ? 0.5f : -0.5f));
  if (pv_whole < 0)
    pv_whole = 0;
  if (pv_whole > 999)
    pv_whole = 999;
  uint16_t pv_disp = static_cast<uint16_t>(pv_whole);
  char unit_char = 'C';

#if ENABLE_SETTINGS_MENU
  if (getTempUnit_() == kTempUnitF)
  {
    sp_disp = cToF_u8(state_.cycle_setpoint_c);
    const float pv_f = (pv_c * 1.8f) + 32.0f;
    pv_whole = static_cast<int16_t>(pv_f + ((pv_f >= 0.0f) ? 0.5f : -0.5f));
    if (pv_whole < 0)
      pv_whole = 0;
    if (pv_whole > 999)
      pv_whole = 999;
    pv_disp = static_cast<uint16_t>(pv_whole);
    unit_char = 'F';
  }
#endif

  // Line 2: SP/PV
  lcd.setCursor(0, 1);
  lcd.print(F("SP:"));
  lcdPrintU16_3_(lcd, sp_disp);
  if (state_.cycle_temp_modified != 0u)
  {
    lcd.print('*');
  }
  lcd.print(F(" PV:"));
  lcdPrintU16_3_(lcd, pv_disp);
  lcd.print(' ');
  lcd.print(static_cast<char>(0xDF));
  lcd.print(unit_char);
  lcdPadSpacesToEol_(lcd, (state_.cycle_temp_modified != 0u) ? 17u : 16u);

  // Line 3: TIME
  lcd.setCursor(0, 2);
  lcd.print(F("TIME: "));
  lcdPrintU16_3_(lcd, rem_min);
  lcd.print(':');
  lcdPrintU8_2_(lcd, rem_sec);
  lcd.print(state_.cycle_time_modified ? '*' : ' ');
  lcdPadSpacesToEol_(lcd, 13u);

  // Line 4: status flags (Req 12A/12B.41/.50)
  lcd.setCursor(0, 3);
  lcd.print(heaterControl.isHeaterOn() ? 'H' : ' ');
  const DrumControl::Direction dir = drumControl.getCurrentDirection();
  lcd.print((dir == DrumControl::Direction::FORWARD) ? 'F' : ' ');
  lcd.print((dir == DrumControl::Direction::REVERSE) ? 'R' : ' ');
  lcd.print(F(" B:PAUSE"));
  lcdPadSpacesToEol_(lcd, 11u);
}

void AppStateMachine::renderCycleEdit_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("EDIT PARAMETERS"));
  lcdPadSpacesToEol_(lcd, 15u);

  // Line 2: TEMP
  lcd.setCursor(0, 1);
  lcd.print(F("TEMP: "));
  uint16_t t_disp = state_.cycle_edit_temp_c;
  char unit_char = 'C';
#if ENABLE_SETTINGS_MENU
  if (getTempUnit_() == kTempUnitF)
  {
    t_disp = cToF_u8(state_.cycle_edit_temp_c);
    unit_char = 'F';
  }
#endif
  if (state_.cycle_edit_field == 0u)
    lcd.print('[');
  lcdPrintU16_NoLeading_(lcd, t_disp);
  if (state_.cycle_edit_field == 0u)
    lcd.print(']');
  lcd.print(static_cast<char>(0xDF));
  lcd.print(unit_char);

  const uint8_t td = (t_disp >= 100u) ? 3u : 2u;
  uint8_t used = static_cast<uint8_t>(6u + td + 2u + ((state_.cycle_edit_field == 0u) ? 2u : 0u));
  lcdPadSpacesToEol_(lcd, used);

  // Line 3: TIME
  lcd.setCursor(0, 2);
  lcd.print(F("TIME: "));
  if (state_.cycle_edit_field != 0u)
    lcd.print('[');
  lcdPrintU16_NoLeading_(lcd, state_.cycle_edit_duration_min);
  if (state_.cycle_edit_field != 0u)
    lcd.print(']');
  lcd.print(F(" MIN"));

  const uint8_t md = (state_.cycle_edit_duration_min >= 100u) ? 3u : 2u;
  used = static_cast<uint8_t>(6u + md + 4u + ((state_.cycle_edit_field != 0u) ? 2u : 0u));
  lcdPadSpacesToEol_(lcd, used);

  // Line 4: hint (Req 13 / Phase 11.4)
  lcd.setCursor(0, 3);
  lcd.print(F("UP/DN OK:SAVE"));
  lcdPadSpacesToEol_(lcd, 13u);
}

void AppStateMachine::renderRunningCooldown_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  if (state_.param_mode == kParamModeAuto)
  {
    lcd.print(F("AUTO: "));
    lcd.print(state_.auto_program.name);
  }
  else
  {
    lcd.print(F("MANUAL MODE"));
  }

  lcd.setCursor(0, 1);
  lcd.print(F("COOLING DOWN"));
  lcdPadSpacesToEol_(lcd, 12u);
}

void AppStateMachine::updateRunningCooldownUi_()
{
  const uint32_t now = millis();

  if (!tempSensor.isValid())
  {
    return;
  }

  if (state_.cooldown_start_ms == 0u)
  {
    state_.cooldown_start_ms = now;
    state_.cooldown_pause_total_ms = 0u;
  }

  const float pv_c = tempSensor.getTemperature();

  const uint32_t elapsed_ms = now - state_.cooldown_start_ms - state_.cooldown_pause_total_ms;

  // Transition conditions (Req 10 / Phase 11.5).
  if (pv_c < static_cast<float>(COOLDOWN_THRESHOLD) || elapsed_ms >= kCooldownMaxMs)
  {
    transitionTo(SystemState::ANTI_CREASE);
    return;
  }

  if (state_.cycle_last_ui_ms != 0u && (now - state_.cycle_last_ui_ms) < kRunUiRefreshMs)
  {
    return;
  }
  state_.cycle_last_ui_ms = now;

  // PV display unit conversion (whole degrees).
  int16_t pv_whole = static_cast<int16_t>(pv_c + ((pv_c >= 0.0f) ? 0.5f : -0.5f));
  if (pv_whole < 0)
    pv_whole = 0;
  if (pv_whole > 999)
    pv_whole = 999;
  uint16_t pv_disp = static_cast<uint16_t>(pv_whole);
  char unit_char = 'C';

#if ENABLE_SETTINGS_MENU
  if (getTempUnit_() == kTempUnitF)
  {
    const float pv_f = (pv_c * 1.8f) + 32.0f;
    pv_whole = static_cast<int16_t>(pv_f + ((pv_f >= 0.0f) ? 0.5f : -0.5f));
    if (pv_whole < 0)
      pv_whole = 0;
    if (pv_whole > 999)
      pv_whole = 999;
    pv_disp = static_cast<uint16_t>(pv_whole);
    unit_char = 'F';
  }
#endif

  const uint16_t elapsed_s = static_cast<uint16_t>(elapsed_ms / 1000ul);
  const uint8_t e_min = static_cast<uint8_t>((elapsed_s / 60u) % 100u);
  const uint8_t e_sec = static_cast<uint8_t>(elapsed_s % 60u);

  // Line 3: PV + TIME
  lcd.setCursor(0, 2);
  lcd.print(F("PV:"));
  lcdPrintU16_3_(lcd, pv_disp);
  lcd.print(static_cast<char>(0xDF));
  lcd.print(unit_char);
  lcd.print(F(" TIME:"));
  lcdPrintU8_2_(lcd, e_min);
  lcd.print(':');
  lcdPrintU8_2_(lcd, e_sec);
  lcdPadSpacesToEol_(lcd, 19u);

  // Line 4: motor status only (no heater indicator).
  lcd.setCursor(0, 3);
  const DrumControl::Direction dir = drumControl.getCurrentDirection();
  const char d = (dir == DrumControl::Direction::FORWARD) ? 'F' : (dir == DrumControl::Direction::REVERSE) ? 'R'
                                                                                                           : ' ';
  lcd.print(d);
  lcd.print(F(" B:PAUSE"));
  lcdPadSpacesToEol_(lcd, 9u);
}

void AppStateMachine::renderPaused_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  if (state_.param_mode == kParamModeAuto)
  {
    lcd.print(F("AUTO: "));
    lcd.print(state_.auto_program.name);
  }
  else
  {
    lcd.print(F("MANUAL MODE"));
  }

  lcd.setCursor(0, 1);
  lcd.print(F("** PAUSED **"));
  lcdPadSpacesToEol_(lcd, 12u);

  lcd.setCursor(0, 2);
  lcd.print(F("A:RESUME B:ABORT"));
  lcdPadSpacesToEol_(lcd, 16u);

  // Line 4: elapsed time (cycle progress).
  uint32_t elapsed_ms = 0u;
  if (state_.paused_from_state == SystemState::RUNNING_HEAT)
  {
    const uint32_t now_ms = state_.pause_start_ms;
    elapsed_ms = (state_.heat_start_ms == 0u) ? 0u : (now_ms - state_.heat_start_ms - state_.heat_pause_total_ms);
  }
  else if (state_.paused_from_state == SystemState::RUNNING_COOLDOWN)
  {
    const uint32_t heat_total_ms = static_cast<uint32_t>(state_.cycle_duration_min) * 60ul * 1000ul;
    const uint32_t now_ms = state_.pause_start_ms;
    const uint32_t cd_elapsed =
        (state_.cooldown_start_ms == 0u) ? 0u : (now_ms - state_.cooldown_start_ms - state_.cooldown_pause_total_ms);
    elapsed_ms = heat_total_ms + cd_elapsed;
  }

  const uint16_t elapsed_s = static_cast<uint16_t>(elapsed_ms / 1000ul);
  const uint16_t e_min = static_cast<uint16_t>(elapsed_s / 60u);
  const uint8_t e_sec = static_cast<uint8_t>(elapsed_s % 60u);

  lcd.setCursor(0, 3);
  lcd.print(F("ELAPSED: "));
  lcdPrintU16_3_(lcd, e_min);
  lcd.print(':');
  lcdPrintU8_2_(lcd, e_sec);
  lcdPadSpacesToEol_(lcd, 15u);
}

void AppStateMachine::updatePausedUi_()
{
  const uint32_t now = millis();

  if (state_.pause_timeout_shown != 0u)
  {
    if (now - state_.pause_timeout_msg_start_ms >= kPauseTimeoutMsgMs)
    {
      transitionTo(SystemState::IDLE);
    }
    return;
  }

  if (state_.pause_start_ms != 0u && (now - state_.pause_start_ms) >= kPauseTimeoutMs)
  {
    // Abort after 30 minutes paused (Req 26).
    lcd.clear();
    lcd.setCursor(0, 0);
    if (state_.param_mode == kParamModeAuto)
    {
      lcd.print(F("AUTO: "));
      lcd.print(state_.auto_program.name);
    }
    else
    {
      lcd.print(F("MANUAL MODE"));
    }
    lcd.setCursor(0, 1);
    lcd.print(F("PAUSE TIMEOUT"));
    lcdPadSpacesToEol_(lcd, 12u);
    lcd.setCursor(0, 2);
    lcd.print(F("ABORTING..."));
    lcdPadSpacesToEol_(lcd, 11u);
    lcd.setCursor(0, 3);
    lcd.print(F("RETURNING TO MENU"));
    lcdPadSpacesToEol_(lcd, 17u);

    state_.pause_timeout_shown = 1u;
    state_.pause_timeout_msg_start_ms = now;
  }
}

void AppStateMachine::renderCycleCompleteSummary_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("CYCLE COMPLETE"));
  lcdPadSpacesToEol_(lcd, 14u);

  lcd.setCursor(0, 1);
  if (state_.param_mode == kParamModeAuto)
  {
    lcd.print(state_.auto_program.name);
    uint8_t used = 0u;
    for (uint8_t i = 0u; i < 12u; i++)
    {
      if (state_.auto_program.name[i] == '\0')
        break;
      used++;
    }
    lcdPadSpacesToEol_(lcd, used);
  }
  else
  {
    lcd.print(F("MANUAL MODE"));
    lcdPadSpacesToEol_(lcd, 11u);
  }

  const uint32_t total_s = state_.cycle_total_time_ms / 1000ul;
  const uint16_t t_min = static_cast<uint16_t>(total_s / 60u);
  const uint8_t t_sec = static_cast<uint8_t>(total_s % 60u);

  lcd.setCursor(0, 2);
  lcd.print(F("TOTAL TIME: "));
  lcdPrintU16_3_(lcd, t_min);
  lcd.print(':');
  lcdPrintU8_2_(lcd, t_sec);
  lcdPadSpacesToEol_(lcd, 18u);

  lcd.setCursor(0, 3);
  lcd.print(F("PRESS ANY KEY"));
  lcdPadSpacesToEol_(lcd, 13u);
}

void AppStateMachine::renderAntiCreaseActive_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("CYCLE COMPLETE"));
  lcdPadSpacesToEol_(lcd, 14u);

  lcd.setCursor(0, 1);
  lcd.print(F("ANTI-CREASE ACTIVE"));
  lcdPadSpacesToEol_(lcd, 18u);

  lcd.setCursor(0, 2);
  lcd.print(F("ELAPSED: 00:00"));
  lcdPadSpacesToEol_(lcd, 14u);

  lcd.setCursor(0, 3);
  lcd.print(F("PRESS STOP TO EXIT"));
  lcdPadSpacesToEol_(lcd, 18u);
}

void AppStateMachine::updateAntiCreaseUi_()
{
  const uint32_t now = millis();

  // Door open ends anti-crease immediately (Req 11.6).
  if (!io.isDoorClosed())
  {
    drumControl.stop();
    transitionTo(SystemState::IDLE);
    return;
  }

  if (now - state_.anticrease_start_ms >= kAntiCreaseMaxMs)
  {
    drumControl.stop();
    transitionTo(SystemState::IDLE);
    return;
  }

  if (state_.anticrease_page == 0u)
  {
    if (now - state_.state_entry_time_ms >= kCycleCompleteSummaryMs)
    {
      state_.anticrease_page = 1u;
      state_.tumble_state = 0u;
      state_.tumble_cycle_start_ms = now;
      state_.tumble_start_ms = 0u;
      renderAntiCreaseActive_();
    }
    return;
  }

  // Active anti-crease periodic tumbling.
  if (state_.tumble_state == 0u)
  {
    if (now - state_.tumble_cycle_start_ms >= kAntiCreaseTumbleEveryMs)
    {
      drumControl.start();
      state_.tumble_state = 1u;
      state_.tumble_start_ms = now;
    }
  }
  else
  {
    if (now - state_.tumble_start_ms >= kAntiCreaseTumbleDurationMs)
    {
      drumControl.stop();
      state_.tumble_state = 0u;
      state_.tumble_cycle_start_ms = now;
    }
  }

  // Update elapsed HH:MM display every second.
  if (state_.cycle_last_ui_ms != 0u && (now - state_.cycle_last_ui_ms) < kRunUiRefreshMs)
  {
    return;
  }
  state_.cycle_last_ui_ms = now;

  const uint32_t elapsed_ms = now - state_.anticrease_start_ms;
  const uint32_t elapsed_min_total = elapsed_ms / 60000ul;
  const uint16_t hh = static_cast<uint16_t>((elapsed_min_total / 60ul) % 100ul);
  const uint8_t mm = static_cast<uint8_t>(elapsed_min_total % 60ul);

  lcd.setCursor(0, 2);
  lcd.print(F("ELAPSED: "));
  lcdPrintU8_2_(lcd, static_cast<uint8_t>(hh));
  lcd.print(':');
  lcdPrintU8_2_(lcd, mm);
  lcdPadSpacesToEol_(lcd, 14u);
}
#endif // ENABLE_CYCLE_EXECUTION

#if ENABLE_SETTINGS_MENU
void AppStateMachine::renderSettings_()
{
  switch (state_.settings_view)
  {
  case kSettingsViewMenu:
    renderSettingsMenu_();
    break;
  case kSettingsViewTempUnits:
    renderSettingsTempUnits_();
    break;
  case kSettingsViewSound:
    renderSettingsSound_();
    break;
#if ENABLE_PROGRAM_EDITOR
  case kSettingsViewProgramList:
    renderProgramList_();
    break;
  case kSettingsViewProgramEdit:
  case kSettingsViewProgramEditConfirm:
    renderProgramEdit_();
    break;
#endif
  default:
    state_.settings_view = kSettingsViewMenu;
    renderSettingsMenu_();
    break;
  }
}

void AppStateMachine::renderSettingsMenu_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SETTINGS"));
  lcdPadSpacesToEol_(lcd, 8u);

  lcd.setCursor(0, 1);
  lcd.print((state_.settings_menu_selection == 0u) ? F("> TEMP UNITS") : F("  TEMP UNITS"));
  lcdPadSpacesToEol_(lcd, 12u);

  lcd.setCursor(0, 2);
  lcd.print((state_.settings_menu_selection == 1u) ? F("> SOUND") : F("  SOUND"));
  lcdPadSpacesToEol_(lcd, 7u);

#if ENABLE_PROGRAM_EDITOR
  lcd.setCursor(0, 3);
  lcd.print((state_.settings_menu_selection == 2u) ? F("> PROGRAM EDITOR") : F("  PROGRAM EDITOR"));
  lcdPadSpacesToEol_(lcd, 16u);
#else
  lcd.setCursor(0, 3);
  lcd.print(F("                    "));
#endif
}

void AppStateMachine::renderSettingsTempUnits_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("TEMP UNITS"));
  lcdPadSpacesToEol_(lcd, 10u);

  const uint8_t unit = getTempUnit_();
  lcd.setCursor(0, 1);
  lcd.print((unit == kTempUnitC) ? F("> CELSIUS") : F("  CELSIUS"));
  lcdPadSpacesToEol_(lcd, 9u);

  lcd.setCursor(0, 2);
  lcd.print((unit == kTempUnitF) ? F("> FAHRENHEIT") : F("  FAHRENHEIT"));
  lcdPadSpacesToEol_(lcd, 12u);

  lcd.setCursor(0, 3);
  lcd.print(F("UP/DN OK:SAVE"));
  lcdPadSpacesToEol_(lcd, 13u);
}

void AppStateMachine::renderSettingsSound_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SOUND"));
  lcdPadSpacesToEol_(lcd, 5u);

  const uint8_t on = (g_settings.sound_enabled != 0u) ? 1u : 0u;
  lcd.setCursor(0, 1);
  lcd.print(on ? F("> ON") : F("  ON"));
  lcdPadSpacesToEol_(lcd, 4u);

  lcd.setCursor(0, 2);
  lcd.print(on ? F("  OFF") : F("> OFF"));
  lcdPadSpacesToEol_(lcd, 5u);

  lcd.setCursor(0, 3);
  lcd.print(F("UP/DN OK:SAVE"));
  lcdPadSpacesToEol_(lcd, 13u);
}

#if ENABLE_PROGRAM_EDITOR
void AppStateMachine::renderProgramList_()
{
  lcd.clear();
  // Print a full line to avoid any wrap/padding overwriting (LCD-safe).
  lcd.setCursor(0, 0);
  lcd.print(F("PROGRAM EDITOR      "));

  const uint8_t sel =
      (state_.program_list_selection >= EEPROMStore::PROGRAM_COUNT) ? 0u : state_.program_list_selection;
  state_.program_list_selection = sel;

  // List UI matches operator menus: show selection arrow + next item + hints.
  EEPROMStore::Program p_sel;
  (void)eepromStore.loadProgram(sel, p_sel);

  const uint8_t next = static_cast<uint8_t>((sel + 1u) % EEPROMStore::PROGRAM_COUNT);
  EEPROMStore::Program p_next;
  (void)eepromStore.loadProgram(next, p_next);

  // Line 2: selected program
  lcd.setCursor(0, 1);
  lcd.print(F(" > "));
  lcd.print(p_sel.name);
  uint8_t used = 3u;
  for (uint8_t i = 0u; i < 12u; i++)
  {
    if (p_sel.name[i] == '\0')
      break;
    used++;
  }
  lcdPadSpacesToEol_(lcd, used);

  // Line 3: next program (context)
  lcd.setCursor(0, 2);
  lcd.print(F("   "));
  lcd.print(p_next.name);
  used = 3u;
  for (uint8_t i = 0u; i < 12u; i++)
  {
    if (p_next.name[i] == '\0')
      break;
    used++;
  }
  lcdPadSpacesToEol_(lcd, used);

  // Line 4: hints
  lcd.setCursor(0, 3);
  lcd.print(F("UP/DN OK:SELECT"));
  lcdPadSpacesToEol_(lcd, 15u);
}

void AppStateMachine::renderProgramEdit_()
{
  if (state_.settings_view == kSettingsViewProgramEditConfirm)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("SAVE CHANGES?"));
    lcdPadSpacesToEol_(lcd, 13u);

    lcd.setCursor(0, 1);
    lcd.print(state_.program_edit.name);
    uint8_t len = 0u;
    for (uint8_t i = 0u; i < 12u; i++)
    {
      if (state_.program_edit.name[i] == '\0')
        break;
      len++;
    }
    lcdPadSpacesToEol_(lcd, len);

    lcd.setCursor(0, 2);
    lcd.print(F("A:YES    B:NO"));
    lcdPadSpacesToEol_(lcd, 13u);

    lcd.setCursor(0, 3);
    lcd.print(F("                    "));
    return;
  }

  const uint8_t field = state_.program_edit_field;

  // Header line: "EDIT: <name>"
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("EDIT: "));
  lcd.print(state_.program_edit.name);
  {
    uint8_t used = 6u;
    for (uint8_t i = 0u; i < 12u; i++)
    {
      if (state_.program_edit.name[i] == '\0')
        break;
      used++;
    }
    lcdPadSpacesToEol_(lcd, used);
  }

  if (field == kProgFieldName)
  {
    // Line 2: show name
    lcd.setCursor(0, 1);
    lcd.print(state_.program_edit.name);
    uint8_t used = 0u;
    for (uint8_t i = 0u; i < 12u; i++)
    {
      if (state_.program_edit.name[i] == '\0')
        break;
      used++;
    }
    lcdPadSpacesToEol_(lcd, used);

    // Line 3: caret under active character (wrap at 12 chars)
    lcd.setCursor(0, 2);
    for (uint8_t i = 0u; i < 20u; i++)
    {
      lcd.print(' ');
    }
    const uint8_t cpos = (state_.program_name_cursor >= 12u) ? 0u : state_.program_name_cursor;
    lcd.setCursor(cpos, 2);
    lcd.print('^');

    lcd.setCursor(0, 3);
    lcd.print(F("UP/DN CH OK:NXT"));
    lcdPadSpacesToEol_(lcd, 15u);
    return;
  }

  // Page: TEMP/TIME
  if (field == kProgFieldTemp || field == kProgFieldTime)
  {
    lcd.setCursor(0, 1);
    lcd.print(F("TEMP: "));

    uint16_t t_disp = state_.program_edit.temp_setpoint;
    char unit_char = 'C';
#if ENABLE_SETTINGS_MENU
    if (getTempUnit_() == kTempUnitF)
    {
      t_disp = cToF_u8(state_.program_edit.temp_setpoint);
      unit_char = 'F';
    }
#endif
    if (field == kProgFieldTemp)
      lcd.print('[');
    lcdPrintU16_NoLeading_(lcd, t_disp);
    if (field == kProgFieldTemp)
      lcd.print(']');
    lcd.print(static_cast<char>(0xDF));
    lcd.print(unit_char);

    uint8_t digits = (t_disp >= 100u) ? 3u : 2u;
    uint8_t used = static_cast<uint8_t>(6u + digits + 2u + ((field == kProgFieldTemp) ? 2u : 0u));
    lcdPadSpacesToEol_(lcd, used);

    lcd.setCursor(0, 2);
    lcd.print(F("TIME: "));
    if (field == kProgFieldTime)
      lcd.print('[');
    lcdPrintU16_NoLeading_(lcd, state_.program_edit.duration_min);
    if (field == kProgFieldTime)
      lcd.print(']');
    lcd.print(F(" MIN"));

    digits = (state_.program_edit.duration_min >= 100u) ? 3u : 2u;
    used = static_cast<uint8_t>(6u + digits + 4u + ((field == kProgFieldTime) ? 2u : 0u));
    lcdPadSpacesToEol_(lcd, used);

    lcd.setCursor(0, 3);
    lcd.print(F("A:SAVE B:CANCEL"));
    lcdPadSpacesToEol_(lcd, 15u);
    return;
  }

  // Page: drum pattern + duty limit
  lcd.setCursor(0, 1);
  lcd.print(F("FWD:"));
  if (field == kProgFieldFwd)
    lcd.print('[');
  lcdPrintU8_2_(lcd, state_.program_edit.fwd_time_s);
  if (field == kProgFieldFwd)
    lcd.print(']');
  lcd.print(F("s "));

  lcd.print(F("REV:"));
  if (field == kProgFieldRev)
    lcd.print('[');
  lcdPrintU8_2_(lcd, state_.program_edit.rev_time_s);
  if (field == kProgFieldRev)
    lcd.print(']');
  lcd.print(F("s"));

  lcdPadSpacesToEol_(lcd, 18u);

  lcd.setCursor(0, 2);
  lcd.print(F("STOP:"));
  if (field == kProgFieldStop)
    lcd.print('[');
  lcdPrintU8_2_(lcd, state_.program_edit.stop_time_s);
  if (field == kProgFieldStop)
    lcd.print(']');
  lcd.print(F("s "));

  lcd.print(F("DUTY:"));
  if (field == kProgFieldDuty)
    lcd.print('[');
  const uint8_t duty = (state_.program_edit.duty_limit > 100u) ? 100u : state_.program_edit.duty_limit;
  if (duty >= 100u)
  {
    lcd.print('1');
    lcd.print('0');
    lcd.print('0');
  }
  else
  {
    lcd.print(static_cast<char>('0' + ((duty / 10u) % 10u)));
    lcd.print(static_cast<char>('0' + (duty % 10u)));
  }
  if (field == kProgFieldDuty)
    lcd.print(']');
  lcd.print('%');

  lcdPadSpacesToEol_(lcd, (field == kProgFieldDuty) ? 20u : 18u);

  lcd.setCursor(0, 3);
  lcd.print(F("A:SAVE B:CANCEL"));
  lcdPadSpacesToEol_(lcd, 15u);
}
#endif // ENABLE_PROGRAM_EDITOR
#endif // ENABLE_SETTINGS_MENU

#if ENABLE_SERVICE_MENU
void AppStateMachine::renderService_()
{
  switch (state_.service_view)
  {
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
    renderIoTest_();
    break;
#endif
#if ENABLE_SERVICE_MEMORY_INFO
  case kServiceViewMemoryInfo:
    renderMemoryInfo_();
    break;
#endif
#if ENABLE_SERVICE_FAULT_HISTORY
  case kServiceViewFaultHistory:
    renderFaultHistory_();
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
#if ENABLE_SERVICE_FACTORY_RESET
  case kServiceViewFactoryReset:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("FACTORY RESET"));
    lcdPadSpacesToEol_(lcd, 13u);
    lcd.setCursor(0, 1);
    lcd.print(F("RESET TO FACTORY"));
    lcdPadSpacesToEol_(lcd, 16u);
    lcd.setCursor(0, 2);
    lcd.print(F("DEFAULTS?"));
    lcdPadSpacesToEol_(lcd, 9u);
    lcd.setCursor(0, 3);
    lcd.print(F("A:YES  B:NO"));
    lcdPadSpacesToEol_(lcd, 12u);
    break;
#endif
  default:
    state_.service_view = kServiceViewMenu;
    renderServiceMenu_();
    break;
  }
}

void AppStateMachine::renderServiceMenu_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SERVICE MENU"));

  uint8_t start_index = 0u;
  if (kServiceMenuItems > 3u)
  {
    if (state_.service_menu_selection > 1u)
    {
      start_index = static_cast<uint8_t>(state_.service_menu_selection - 1u);
      const uint8_t max_start = static_cast<uint8_t>(kServiceMenuItems - 3u);
      if (start_index > max_start)
      {
        start_index = max_start;
      }
    }
  }
  for (uint8_t row = 0u; row < 3u; row++)
  {
    const uint8_t item = static_cast<uint8_t>(start_index + row);
    lcd.setCursor(0, static_cast<uint8_t>(row + 1u));
    if (item >= kServiceMenuItems)
    {
      lcd.print(F("                    "));
      continue;
    }

    lcd.print((item == state_.service_menu_selection) ? F("> ") : F("  "));

    switch (serviceMenuItemByIndex_(item))
    {
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
#if ENABLE_SERVICE_MEMORY_INFO
    case ServiceItem::MEMORY_INFO:
      lcd.print(F("MEMORY INFO       "));
      break;
#endif
#if ENABLE_SERVICE_FAULT_HISTORY
    case ServiceItem::FAULT_HISTORY:
      lcd.print(F("FAULT HISTORY     "));
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
#if ENABLE_SERVICE_FACTORY_RESET
    case ServiceItem::FACTORY_RESET:
      lcd.print(F("FACTORY RESET     "));
      break;
#endif
    default:
      lcd.print(F("                  "));
      break;
    }
  }
}

#if ENABLE_SERVICE_FAULT_HISTORY
namespace
{
  const char FAULT_HIST_DOOR[] PROGMEM = "DOOR OPEN";
  const char FAULT_HIST_TEMP[] PROGMEM = "TEMP SENSOR";
  const char FAULT_HIST_OVERT[] PROGMEM = "OVER-TEMP";
  const char FAULT_HIST_RUNAWAY[] PROGMEM = "THERM RUNAWAY";
  const char FAULT_HIST_TIMEOUT[] PROGMEM = "HEAT TIMEOUT";
  const char FAULT_HIST_OUTPUT[] PROGMEM = "OUTPUT FAULT";
  const char FAULT_HIST_WDT[] PROGMEM = "WDT RESET";
  const char FAULT_HIST_BROWNOUT[] PROGMEM = "BROWNOUT";
  const char FAULT_HIST_SELFTEST[] PROGMEM = "SELF-TEST";
  const char FAULT_HIST_UNKNOWN[] PROGMEM = "UNKNOWN";

  const __FlashStringHelper *faultHistoryLabel_(uint8_t code)
  {
    switch (static_cast<FaultCode>(code))
    {
    case FaultCode::DOOR_OPEN:
      return reinterpret_cast<const __FlashStringHelper *>(FAULT_HIST_DOOR);
    case FaultCode::TEMP_SENSOR_FAULT:
      return reinterpret_cast<const __FlashStringHelper *>(FAULT_HIST_TEMP);
    case FaultCode::OVER_TEMP:
      return reinterpret_cast<const __FlashStringHelper *>(FAULT_HIST_OVERT);
    case FaultCode::THERMAL_RUNAWAY:
      return reinterpret_cast<const __FlashStringHelper *>(FAULT_HIST_RUNAWAY);
    case FaultCode::HEATING_TIMEOUT:
      return reinterpret_cast<const __FlashStringHelper *>(FAULT_HIST_TIMEOUT);
    case FaultCode::OUTPUT_FAULT:
      return reinterpret_cast<const __FlashStringHelper *>(FAULT_HIST_OUTPUT);
    case FaultCode::WATCHDOG_RESET:
      return reinterpret_cast<const __FlashStringHelper *>(FAULT_HIST_WDT);
    case FaultCode::BROWNOUT:
      return reinterpret_cast<const __FlashStringHelper *>(FAULT_HIST_BROWNOUT);
    case FaultCode::SELF_TEST_FAIL:
      return reinterpret_cast<const __FlashStringHelper *>(FAULT_HIST_SELFTEST);
    case FaultCode::NONE:
    default:
      return reinterpret_cast<const __FlashStringHelper *>(FAULT_HIST_UNKNOWN);
    }
  }

  uint8_t u16Digits_(uint16_t value)
  {
    if (value >= 100u)
      return 3u;
    if (value >= 10u)
      return 2u;
    return 1u;
  }
} // namespace

void AppStateMachine::renderFaultHistory_()
{
  if (state_.fault_history_page != 0u)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("CLEAR HISTORY?"));
    lcdPadSpacesToEol_(lcd, 14u);
    lcd.setCursor(0, 1);
    lcd.print(F("A:YES  B:NO"));
    lcdPadSpacesToEol_(lcd, 11u);
    lcd.setCursor(0, 2);
    lcd.print(F("                "));
    lcd.setCursor(0, 3);
    lcd.print(F("                "));
    return;
  }

  uint8_t count = 0u;
  EEPROMStore::FaultEntry tmp;
  for (uint8_t i = 0u; i < EEPROMStore::FAULT_HISTORY_COUNT; i++)
  {
    if (!eepromStore.getFaultHistory(i, tmp))
    {
      break;
    }
    count++;
  }

  lcd.clear();
  // Print a full line to avoid any wrap/padding overwriting (LCD-safe).
  lcd.setCursor(0, 0);
  lcd.print(F("FAULT HISTORY       "));

  if (count == 0u)
  {
    lcd.setCursor(0, 1);
    lcd.print(F("NO FAULTS LOGGED"));
    lcdPadSpacesToEol_(lcd, 16u);
    lcd.setCursor(0, 2);
    lcd.print(F("                "));
    lcd.setCursor(0, 3);
    lcd.print(F("B:BACK"));
    lcdPadSpacesToEol_(lcd, 6u);
    return;
  }

  if (state_.fault_history_index >= count)
  {
    state_.fault_history_index = static_cast<uint8_t>(count - 1u);
  }

  EEPROMStore::FaultEntry entry;
  (void)eepromStore.getFaultHistory(state_.fault_history_index, entry);

  // Line 2: index/total + short fault label.
  {
    const uint8_t pos = static_cast<uint8_t>(state_.fault_history_index + 1u);
    const uint8_t pos_digits = (pos >= 10u) ? 2u : 1u;
    const uint8_t cnt_digits = (count >= 10u) ? 2u : 1u;
    uint8_t cols = static_cast<uint8_t>(pos_digits + 1u + cnt_digits + 1u); // "p/c "

    lcd.setCursor(0, 1);
    lcdPrintU16_NoLeading_(lcd, pos);
    lcd.print('/');
    lcdPrintU16_NoLeading_(lcd, count);
    lcd.print(' ');

    const __FlashStringHelper *label = faultHistoryLabel_(entry.code);
    lcd.print(label);

    // Pad the remainder of the line (Flash string length is unknown at compile time).
    const char *p = reinterpret_cast<const char *>(label);
    const uint8_t max_len = (cols < 20u) ? static_cast<uint8_t>(20u - cols) : 0u;
    uint8_t len = 0u;
    while (len < max_len)
    {
      const char c = static_cast<char>(pgm_read_byte(p + len));
      if (c == '\0')
      {
        break;
      }
      len++;
    }
    cols = static_cast<uint8_t>(cols + len);
    lcdPadSpacesToEol_(lcd, cols);
  }

  // Line 3: temperature + timestamp.
  {
    char unit_char = 'C';
    int16_t temp_deci = static_cast<int16_t>(entry.temperature_c * 10.0f +
                                             ((entry.temperature_c >= 0.0f) ? 0.5f : -0.5f));

#if ENABLE_SETTINGS_MENU
    if (getTempUnit_() == kTempUnitF)
    {
      const int32_t d = static_cast<int32_t>(temp_deci);
      const int32_t deci_f = (d * 9 + ((d >= 0) ? 2 : -2)) / 5 + 320; // °F * 10
      temp_deci = static_cast<int16_t>(deci_f);
      unit_char = 'F';
    }
#endif

    bool neg = (temp_deci < 0);
    uint16_t abs_val = static_cast<uint16_t>(neg ? -temp_deci : temp_deci);
    const uint16_t whole = static_cast<uint16_t>(abs_val / 10u);
    const uint8_t frac = static_cast<uint8_t>(abs_val % 10u);

    uint8_t cols = 0u;
    lcd.setCursor(0, 2);
    lcd.print(F("T:"));
    cols = 2u;
    if (neg)
    {
      lcd.print('-');
      cols++;
    }
    lcdPrintU16_NoLeading_(lcd, whole);
    cols = static_cast<uint8_t>(cols + u16Digits_(whole));
    lcd.print('.');
    lcd.print(static_cast<char>('0' + (frac % 10u)));
    cols = static_cast<uint8_t>(cols + 2u);
    lcd.print(unit_char);
    cols++;
    lcd.print(' ');
    cols++;

    uint32_t t = entry.timestamp_s;
    uint16_t hh = static_cast<uint16_t>(t / 3600u);
    if (hh > 99u)
      hh = 99u;
    const uint8_t mm = static_cast<uint8_t>((t / 60u) % 60u);
    const uint8_t ss = static_cast<uint8_t>(t % 60u);
    lcdPrintU8_2_(lcd, static_cast<uint8_t>(hh));
    cols = static_cast<uint8_t>(cols + 2u);
    lcd.print(':');
    cols++;
    lcdPrintU8_2_(lcd, mm);
    cols = static_cast<uint8_t>(cols + 2u);
    lcd.print(':');
    cols++;
    lcdPrintU8_2_(lcd, ss);
    cols = static_cast<uint8_t>(cols + 2u);

    lcdPadSpacesToEol_(lcd, cols);
  }

  // Line 4: hints.
  lcd.setCursor(0, 3);
  lcd.print(F("UP/DN C:CLEAR B:BACK"));
  lcdPadSpacesToEol_(lcd, 20u);
}
#endif // ENABLE_SERVICE_FAULT_HISTORY

#if ENABLE_SERVICE_DRUM_TEST
void AppStateMachine::updateDrumTestDirection_()
{
  const DrumControl::Direction dir = drumControl.getCurrentDirection();
  const uint8_t dir_u = static_cast<uint8_t>(dir);
  if (dir_u == state_.service_last_dir)
  {
    return;
  }
  state_.service_last_dir = dir_u;

  lcd.setCursor(0, 1);
  lcd.print(F("DIR: ")); // 5 chars

  switch (dir)
  {
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

void AppStateMachine::renderDrumTest_()
{
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
void AppStateMachine::updateHeaterTestStatus_()
{
  const uint8_t duty = state_.heater_test_duty;
  const uint8_t on = heaterControl.isHeaterOn() ? 1u : 0u;

  if (duty != state_.service_last_heater_duty)
  {
    state_.service_last_heater_duty = duty;
    lcd.setCursor(0, 1);
    lcd.print(F("DUTY: "));
    char duty_str[4];
    formatDuty3Chars(duty, duty_str);
    lcd.print(duty_str);
    lcd.print(F("%"));
    lcd.print(F("          "));
  }

  if (on != state_.service_last_heater_on)
  {
    state_.service_last_heater_on = on;
    lcd.setCursor(0, 2);
    lcd.print(F("STATE: "));
    lcd.print(on ? F("ON ") : F("OFF"));
    lcd.print(F("          "));
  }
}

void AppStateMachine::renderHeaterTest_()
{
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
void AppStateMachine::updatePidView_()
{
  const uint32_t now = millis();
  if (state_.service_pid_last_update_ms != 0u &&
      (now - state_.service_pid_last_update_ms) < kPidViewRefreshMs)
  {
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

  const uint16_t kp10 = scaleClampU16(kp, 10.0f, 9999u);    // 999.9 max displayed
  const uint16_t ki100 = scaleClampU16(ki, 100.0f, 65535u); // 655.35 max displayed
  const uint16_t kd100 = scaleClampU16(kd, 100.0f, 65535u); // 655.35 max displayed

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

void AppStateMachine::renderPidView_()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("PID PARAMETERS"));
  state_.service_pid_last_update_ms = 0u;
  updatePidView_();
}
#endif // ENABLE_SERVICE_PID_VIEW

#if ENABLE_SERVICE_IO_TEST
namespace
{
  constexpr uint8_t kIoMaskHeater = 0x01u;
  constexpr uint8_t kIoMaskMotorFwd = 0x02u;
  constexpr uint8_t kIoMaskMotorRev = 0x04u;
  constexpr uint8_t kIoMaskAux = 0x08u;
  constexpr uint8_t kIoOutputCount = 4u;

  uint16_t toScaled10_(float v)
  {
    if (v <= 0.0f)
    {
      return 0u;
    }
    const float x = (v * 10.0f) + 0.5f;
    if (x >= 9999.0f)
    {
      return 9999u;
    }
    return static_cast<uint16_t>(x);
  }

  void printTemp10_(UILCD& out, uint16_t scaled10)
  {
    const uint16_t whole = static_cast<uint16_t>(scaled10 / 10u);
    const uint8_t frac = static_cast<uint8_t>(scaled10 % 10u);
    lcdPrintU16_NoLeading_(out, whole);
    out.print('.');
    out.print(static_cast<char>('0' + frac));
  }

  void renderIoOutputLine_(UILCD& out, uint8_t row, uint8_t idx, bool selected, uint8_t mask)
  {
    out.setCursor(0, row);
    out.print(selected ? F("> ") : F("  "));

    uint8_t label_len = 0u;
    bool on = false;

    switch (idx)
    {
    case 0u:
      out.print(F("HEATER"));
      label_len = 6u;
      on = (mask & kIoMaskHeater) != 0u;
      break;
    case 1u:
      out.print(F("MOTOR FWD"));
      label_len = 9u;
      on = (mask & kIoMaskMotorFwd) != 0u;
      break;
    case 2u:
      out.print(F("MOTOR REV"));
      label_len = 9u;
      on = (mask & kIoMaskMotorRev) != 0u;
      break;
    default:
      out.print(F("AUX RELAY"));
      label_len = 9u;
      on = (mask & kIoMaskAux) != 0u;
      break;
    }

    out.print(F(": "));
    out.print(on ? F("ON ") : F("OFF"));

    const uint8_t used = static_cast<uint8_t>(2u + label_len + 2u + 3u);
    lcdPadSpacesToEol_(out, used);
  }
} // namespace

void AppStateMachine::renderIoTest_()
{
  if (state_.io_test_page == 0u)
  {
    // Header: include a compact temperature readout.
    lcd.setCursor(0, 0);
    lcd.print(F("I/O TEST "));

    uint8_t used_cols = 8u;
    if (tempSensor.isValid())
    {
      float pv = tempSensor.getTemperature();
      char unit_char = 'C';
#if ENABLE_SETTINGS_MENU
      if (getTempUnit_() == kTempUnitF)
      {
        pv = (pv * 1.8f) + 32.0f;
        unit_char = 'F';
      }
#endif
      const uint16_t pv10 = toScaled10_(pv);
      lcd.print(F("T:"));
      printTemp10_(lcd, pv10);
      lcd.print(static_cast<char>(0xDF));
      lcd.print(unit_char);

      used_cols = static_cast<uint8_t>(used_cols + 2u + digitsU16_(static_cast<uint16_t>(pv10 / 10u)) + 2u + 2u);
    }
    else
    {
      lcd.print(F("T:---.-"));
      used_cols = static_cast<uint8_t>(used_cols + 6u);
    }
    lcdPadSpacesToEol_(lcd, used_cols);

    // Normalize selection.
    if (state_.io_test_selection >= kIoOutputCount)
    {
      state_.io_test_selection = 0u;
    }
    const uint8_t sel = state_.io_test_selection;
    const uint8_t next = static_cast<uint8_t>((sel + 1u) % kIoOutputCount);

    renderIoOutputLine_(lcd, 1u, sel, true, state_.io_test_out_mask);
    renderIoOutputLine_(lcd, 2u, next, false, state_.io_test_out_mask);

    lcd.setCursor(0, 3);
    lcd.print(F("UP/DN OK:TOG D:"));
    lcd.print(io.isDoorClosed() ? F("CLSD") : F("OPEN"));
    lcdPadSpacesToEol_(lcd, 19u);
    return;
  }

  // Page: sensors
  lcd.setCursor(0, 0);
  lcd.print(F("I/O TEST SENSORS"));
  lcdPadSpacesToEol_(lcd, 16u);

  lcd.setCursor(0, 1);
  lcd.print(F("TEMP: "));
  if (tempSensor.isValid())
  {
    float pv = tempSensor.getTemperature();
    char unit_char = 'C';
#if ENABLE_SETTINGS_MENU
    if (getTempUnit_() == kTempUnitF)
    {
      pv = (pv * 1.8f) + 32.0f;
      unit_char = 'F';
    }
#endif
    const uint16_t pv10 = toScaled10_(pv);
    const uint8_t d = digitsU16_(static_cast<uint16_t>(pv10 / 10u));
    printTemp10_(lcd, pv10);
    lcd.print(static_cast<char>(0xDF));
    lcd.print(unit_char);
    lcdPadSpacesToEol_(lcd, static_cast<uint8_t>(10u + d));
  }
  else
  {
    lcd.print(F("---.-"));
    lcdPadSpacesToEol_(lcd, 11u);
  }

  lcd.setCursor(0, 2);
  lcd.print(F("RAW:  "));
  if (tempSensor.isValid())
  {
    float raw = tempSensor.getRawTemperature();
    char unit_char = 'C';
#if ENABLE_SETTINGS_MENU
    if (getTempUnit_() == kTempUnitF)
    {
      raw = (raw * 1.8f) + 32.0f;
      unit_char = 'F';
    }
#endif
    const uint16_t raw10 = toScaled10_(raw);
    const uint8_t d = digitsU16_(static_cast<uint16_t>(raw10 / 10u));
    printTemp10_(lcd, raw10);
    lcd.print(static_cast<char>(0xDF));
    lcd.print(unit_char);
    lcdPadSpacesToEol_(lcd, static_cast<uint8_t>(10u + d));
  }
  else
  {
    lcd.print(F("---.-"));
    lcdPadSpacesToEol_(lcd, 11u);
  }

  lcd.setCursor(0, 3);
  lcd.print(F("DOOR:"));
  lcd.print(io.isDoorClosed() ? F("CLOSED") : F("OPEN  "));
  lcd.print(F(" LT/RT"));
  lcdPadSpacesToEol_(lcd, 17u);
}

void AppStateMachine::updateIoTestUi_()
{
  const uint32_t now = millis();

  // Safety: door open forces outputs OFF and clears latched IO test selections
  // so they do not restart automatically when the door closes.
  if (!io.isDoorClosed())
  {
    if (state_.io_test_out_mask != 0u)
    {
      state_.io_test_out_mask = 0u;
      heaterControl.setDutyCycle(0.0f);
      heaterControl.disable();
      drumControl.stop();
      io.emergencyStop();
      state_.io_test_last_update_ms = 0u;
    }
  }

  if (state_.io_test_last_update_ms != 0u && (now - state_.io_test_last_update_ms) < 250u)
  {
    return;
  }

  // Avoid LCD flicker: only re-render if the displayed values changed.
  // (Other service views use the same "update on change" strategy.)
  uint32_t sig = 2166136261u; // FNV-1a
  auto mix = [&sig](uint32_t v)
  {
    sig ^= v;
    sig *= 16777619u;
  };

  mix(static_cast<uint32_t>(state_.io_test_page));
  mix(static_cast<uint32_t>(state_.io_test_selection));
  mix(static_cast<uint32_t>(state_.io_test_out_mask));
  mix(io.isDoorClosed() ? 1u : 0u);

  uint8_t unit = 0u;
#if ENABLE_SETTINGS_MENU
  if (getTempUnit_() == kTempUnitF)
  {
    unit = 1u;
  }
#endif
  mix(static_cast<uint32_t>(unit));

  const bool valid = tempSensor.isValid();
  mix(valid ? 1u : 0u);
  if (valid)
  {
    float pv = tempSensor.getTemperature();
    float raw = tempSensor.getRawTemperature();
    if (unit != 0u)
    {
      pv = (pv * 1.8f) + 32.0f;
      raw = (raw * 1.8f) + 32.0f;
    }
    mix(static_cast<uint32_t>(toScaled10_(pv)));
    if (state_.io_test_page != 0u)
    {
      mix(static_cast<uint32_t>(toScaled10_(raw)));
    }
  }

  state_.io_test_last_update_ms = now;
  if (sig == state_.io_test_last_sig)
  {
    return;
  }
  state_.io_test_last_sig = sig;
  renderIoTest_();
}
#endif // ENABLE_SERVICE_IO_TEST

#if ENABLE_SERVICE_MEMORY_INFO
void AppStateMachine::renderMemoryInfo_()
{
  const uint32_t now = millis();
  if (state_.service_mem_last_update_ms != 0u &&
      (now - state_.service_mem_last_update_ms) < kMemoryInfoRefreshMs)
  {
    return;
  }
  state_.service_mem_last_update_ms = now;

  if (state_.service_mem_page == 0u)
  {
    lcd.setCursor(0, 0);
    lcd.print(F("MEMORY INFO"));
    lcdPadSpacesToEol_(lcd, 11u);

#if defined(ESP32)
    const uint32_t sketch_used_b = ESP.getSketchSize();
    uint32_t sketch_total_b = 0u;
    const esp_partition_t* const running = esp_ota_get_running_partition();
    if (running != nullptr)
    {
      sketch_total_b = running->size;
    }
    if (sketch_total_b == 0u)
    {
      // Fallback: use Arduino helper. This may not match the app partition size on all partition schemes.
      sketch_total_b = sketch_used_b + ESP.getFreeSketchSpace();
    }

    const uint16_t flash_used_k = static_cast<uint16_t>((sketch_used_b + 512u) / 1024u);
    const uint16_t flash_total_k = static_cast<uint16_t>((sketch_total_b + 512u) / 1024u);

    const uint32_t heap_total_b = ESP.getHeapSize();
    const uint32_t heap_free_b = ESP.getFreeHeap();
    const uint32_t heap_used_b = heap_total_b - heap_free_b;
    const uint16_t heap_used_k = static_cast<uint16_t>((heap_used_b + 512u) / 1024u);
    const uint16_t heap_total_k = static_cast<uint16_t>((heap_total_b + 512u) / 1024u);

    const uint16_t eep_used_b = EEPROMStore::reservedBytes();
    const uint16_t eep_total_b = EEPROMStore::totalBytes();

    constexpr uint8_t kWarnPct = 85u;

    uint8_t flash_pct = 0u;
    if (sketch_total_b != 0u)
    {
      const uint64_t num = static_cast<uint64_t>(sketch_used_b) * 100ull + (static_cast<uint64_t>(sketch_total_b) / 2ull);
      flash_pct = static_cast<uint8_t>(num / static_cast<uint64_t>(sketch_total_b));
    }
    if (flash_pct > 99u)
      flash_pct = 99u;

    uint8_t heap_pct = 0u;
    if (heap_total_b != 0u)
    {
      const uint64_t num = static_cast<uint64_t>(heap_used_b) * 100ull + (static_cast<uint64_t>(heap_total_b) / 2ull);
      heap_pct = static_cast<uint8_t>(num / static_cast<uint64_t>(heap_total_b));
    }
    if (heap_pct > 99u)
      heap_pct = 99u;

    uint8_t eep_pct = 0u;
    if (eep_total_b != 0u)
    {
      const uint32_t num = static_cast<uint32_t>(eep_used_b) * 100u + (eep_total_b / 2u);
      eep_pct = static_cast<uint8_t>(num / eep_total_b);
    }
    if (eep_pct > 99u)
      eep_pct = 99u;

    const bool warn_flash = flash_pct > kWarnPct;
    const bool warn_heap = heap_pct > kWarnPct;
    const bool warn_eep = eep_pct > kWarnPct;

    if (warn_flash || warn_heap || warn_eep)
    {
      lcd.setCursor(19, 0);
      lcd.print('!');
    }
#else
    const uint16_t flash_used_k = 0u;
    const uint16_t flash_total_k = 0u;
    const uint16_t heap_used_k = 0u;
    const uint16_t heap_total_k = 0u;
    const uint16_t eep_used_b = EEPROMStore::reservedBytes();
    const uint16_t eep_total_b = EEPROMStore::totalBytes();
    const uint8_t flash_pct = 0u;
    const uint8_t heap_pct = 0u;
    const uint8_t eep_pct = 0u;
    const bool warn_flash = false;
    const bool warn_heap = false;
    const bool warn_eep = false;
#endif

    lcd.setCursor(0, 1);
    lcd.print(F("FLASH:"));
    lcdPrintU16_NoLeading_(lcd, flash_used_k);
    lcd.print('/');
    lcdPrintU16_NoLeading_(lcd, flash_total_k);
    lcd.print('K');
    lcd.print(warn_flash ? '!' : ' ');
    lcdPrintU16_NoLeading_(lcd, flash_pct);
    lcd.print('%');
    lcdPadSpacesToEol_(
        lcd, static_cast<uint8_t>(10u + digitsU16_(flash_used_k) + digitsU16_(flash_total_k) + digitsU16_(flash_pct)));

    lcd.setCursor(0, 2);
    lcd.print(F("HEAP:"));
    lcdPrintU16_NoLeading_(lcd, heap_used_k);
    lcd.print('/');
    lcdPrintU16_NoLeading_(lcd, heap_total_k);
    lcd.print('K');
    lcd.print(warn_heap ? '!' : ' ');
    lcdPrintU16_NoLeading_(lcd, heap_pct);
    lcd.print('%');
    lcdPadSpacesToEol_(
        lcd, static_cast<uint8_t>(9u + digitsU16_(heap_used_k) + digitsU16_(heap_total_k) + digitsU16_(heap_pct)));

    lcd.setCursor(0, 3);
    lcd.print(F("EEP:"));
    lcdPrintU16_NoLeading_(lcd, eep_used_b);
    lcd.print('/');
    lcdPrintU16_NoLeading_(lcd, eep_total_b);
    lcd.print('B');
    lcd.print(warn_eep ? '!' : ' ');
    lcdPrintU16_NoLeading_(lcd, eep_pct);
    lcd.print('%');
    lcdPadSpacesToEol_(lcd,
                       static_cast<uint8_t>(8u + digitsU16_(eep_used_b) + digitsU16_(eep_total_b) + digitsU16_(eep_pct)));
    return;
  }

  // Page 1: firmware/build identifiers.
  lcd.setCursor(0, 0);
  lcd.print(F("FIRMWARE INFO"));
  lcdPadSpacesToEol_(lcd, 13u);

  lcd.setCursor(0, 1);
  lcd.print(F("FW:"));
  lcd.print(reinterpret_cast<const __FlashStringHelper*>(FW_VERSION));
  lcdPadSpacesToEol_(lcd, static_cast<uint8_t>(3u + strlen_P(FW_VERSION)));

  lcd.setCursor(0, 2);
  lcd.print(F("BUILD:"));
  lcd.print(reinterpret_cast<const __FlashStringHelper*>(BUILD_DATE));
  lcdPadSpacesToEol_(lcd, static_cast<uint8_t>(6u + strlen_P(BUILD_DATE)));

  lcd.setCursor(0, 3);
  lcd.print(F("TIME:"));
  lcd.print(reinterpret_cast<const __FlashStringHelper*>(BUILD_TIME));
  lcdPadSpacesToEol_(lcd, static_cast<uint8_t>(5u + strlen_P(BUILD_TIME)));
}
#endif // ENABLE_SERVICE_MEMORY_INFO

#if ENABLE_SERVICE_FOPDT_ID
void AppStateMachine::renderFopdt_()
{
  const FOPDTModel::State st = fopdt.getState();

  if (st == FOPDTModel::State::IDLE)
  {
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

  if (st == FOPDTModel::State::COMPLETE)
  {
    float k = 0.0f;
    float tau_s = 0.0f;
    float l_s = 0.0f;
    const bool ok = fopdt.getResults(k, tau_s, l_s);

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (ok)
    {
      fopdt.computePIDGains(k, tau_s, l_s, kp, ki, kd);
    }

    if (state_.service_fopdt_page == 0u)
    {
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
  if (st == FOPDTModel::State::BASELINE)
  {
    lcd.print(F("FOPDT BASELINE      "));
  }
  else if (st == FOPDTModel::State::STEP)
  {
    lcd.print(F("FOPDT STEP          "));
  }
  else
  {
    lcd.print(F("FOPDT MEASURE       "));
  }

  lcd.setCursor(0, 1);
  if (st == FOPDTModel::State::BASELINE)
  {
    lcd.print(F("T-LEFT:"));
    char rem_str[4];
    writeU16Whole3(rem_str, (rem_s > 999u) ? 999u : rem_s);
    rem_str[3] = '\0';
    lcd.print(rem_str);
    lcd.print(F("s"));
    lcd.print(F("         "));
  }
  else
  {
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

void AppStateMachine::updateFopdt_()
{
  const uint32_t now = millis();
  if (state_.service_fopdt_last_update_ms != 0u &&
      (now - state_.service_fopdt_last_update_ms) < kFopdtRefreshMs)
  {
    return;
  }
  state_.service_fopdt_last_update_ms = now;

  if (fopdt.isIdentifying())
  {
    if (!io.isDoorClosed() || !tempSensor.isValid())
    {
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
    if (st == FOPDTModel::State::BASELINE)
    {
      heaterControl.setDutyCycle(0.0f);
    }
    else if (st == FOPDTModel::State::STEP || st == FOPDTModel::State::MEASURE)
    {
      heaterControl.setDutyCycle(static_cast<float>(fopdt.getStepDutyPercent()));
    }
    else if (st == FOPDTModel::State::COMPLETE)
    {
      g_fopdt_active = 0u;
      heaterControl.disable();
    }
  }

  renderFopdt_();
}
#endif // ENABLE_SERVICE_FOPDT_ID

#if ENABLE_SERVICE_AUTOTUNE
void AppStateMachine::renderAutoTunePre_()
{
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

void AppStateMachine::renderAutoTune_()
{
  if (pidController.isAutoTuneComplete())
  {
    float ku = 0.0f;
    float tu_s = 0.0f;
    (void)pidController.getAutoTuneKuTu(ku, tu_s);

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    (void)pidController.getAutoTuneResults(kp, ki, kd);

    lcd.setCursor(0, 0);
    lcd.print(F("AUTOTUNE DONE       "));

    if (state_.service_autotune_page == 0u)
    {
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
  if (cyc != 0u)
  {
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
  if (cyc == 0u)
  {
    lcd.print(F("--"));
  }
  else
  {
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

void AppStateMachine::updateAutoTuneUi_()
{
  const uint32_t now = millis();
  if (state_.service_autotune_last_update_ms != 0u &&
      (now - state_.service_autotune_last_update_ms) < kAutoTuneUiRefreshMs)
  {
    return;
  }
  state_.service_autotune_last_update_ms = now;

  if (pidController.isAutoTuning())
  {
    pidController.updateAutoTune(tempSensor.getTemperature());
  }

  // If AutoTune aborted (but not complete), exit appropriately.
  if (!pidController.isAutoTuning() && !pidController.isAutoTuneComplete())
  {
    const uint8_t reason = pidController.getAutoTuneAbortReason();
    if (reason == 1u)
    { // door open -> FAULT (Req 6)
      faultMgr.setFault(FaultCode::DOOR_OPEN);
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

void AppStateMachine::update()
{
  const uint32_t now = millis();

  if (state_.invalid_key_until_ms != 0u && now >= state_.invalid_key_until_ms)
  {
    state_.invalid_key_until_ms = 0u;
    restoreScreen_();
  }

  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::IDLE)
  {
    const uint8_t valid_now = tempSensor.isValid() ? 1u : 0u;
    const bool validity_changed = (valid_now != state_.last_temp_valid);

    if (validity_changed || (now - state_.last_temp_display_ms) >= kIdleTempRefreshMs)
    {
      state_.last_temp_valid = valid_now;
      state_.last_temp_display_ms = now;
      lcd.showTemperature(tempSensor.getTemperature(), valid_now != 0u);
    }
  }

#if ENABLE_CYCLE_EXECUTION
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::START_DELAY)
  {
    if (now - state_.state_entry_time_ms >= kStartDelayMs)
    {
      transitionTo(SystemState::RUNNING_HEAT);
      return;
    }
  }

  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::RUNNING_HEAT)
  {
    updateRunningHeatUi_();
  }

  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::RUNNING_COOLDOWN)
  {
    updateRunningCooldownUi_();
  }

  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::PAUSED)
  {
    updatePausedUi_();
  }

  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::ANTI_CREASE)
  {
    updateAntiCreaseUi_();
  }
#endif

#if ENABLE_SERVICE_MENU
#if ENABLE_SERVICE_DRUM_TEST
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewDrumTest)
  {
    updateDrumTestDirection_();
  }
#endif

#if ENABLE_SERVICE_HEATER_TEST
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewHeaterTest)
  {
    updateHeaterTestStatus_();
  }
#endif

#if ENABLE_SERVICE_PID_VIEW
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewPidView)
  {
    updatePidView_();
  }
#endif

#if ENABLE_SERVICE_IO_TEST
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewIoTest)
  {
    updateIoTestUi_();
  }
#endif

#if ENABLE_SERVICE_MEMORY_INFO
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewMemoryInfo)
  {
    renderMemoryInfo_();
  }
#endif

#if ENABLE_SERVICE_FOPDT_ID
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewFopdt)
  {
    updateFopdt_();
  }
#endif

#if ENABLE_SERVICE_AUTOTUNE
  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::SERVICE &&
      state_.service_view == kServiceViewAutoTunePre)
  {
    const uint32_t now_ui = millis();
    if (state_.service_autotune_last_update_ms == 0u ||
        (now_ui - state_.service_autotune_last_update_ms) >= kAutoTuneUiRefreshMs)
    {
      state_.service_autotune_last_update_ms = now_ui;
      renderAutoTunePre_();
    }
  }

  if (state_.invalid_key_until_ms == 0u && state_.current_state == SystemState::AUTOTUNE)
  {
    updateAutoTuneUi_();
  }
#endif
#endif

  if (state_.current_state == SystemState::BOOT)
  {
    if (now - state_.state_entry_time_ms >= kBootScreenDurationMs)
    {
      transitionTo(SystemState::IDLE);
    }
  }

  if (state_.entry_seq != 0u && (now - state_.entry_seq_start_ms) > kEntrySeqTimeoutMs)
  {
    state_.entry_seq = 0u;
  }
}

void AppStateMachine::handleKeyPress(KeypadInput::Key key)
{
  const uint32_t now = millis();

  // Stop/Cancel always has highest priority.
  if (key == KeypadInput::Key::STOP)
  {
    state_.entry_seq = 0u;

#if ENABLE_SERVICE_MENU
    if (state_.current_state == SystemState::SERVICE)
    {
      if (state_.service_view != kServiceViewMenu)
      {
        // STOP acts as "back" inside service tools (and stops the drum test).
#if ENABLE_SERVICE_FAULT_HISTORY
        if (state_.service_view == kServiceViewFaultHistory && state_.fault_history_page != 0u)
        {
          state_.fault_history_page = 0u;
          renderService_();
          return;
        }
#endif
#if ENABLE_SERVICE_DRUM_TEST
        if (state_.service_view == kServiceViewDrumTest)
        {
          drumControl.stop();
        }
#endif
#if ENABLE_SERVICE_HEATER_TEST
        if (state_.service_view == kServiceViewHeaterTest)
        {
          g_heater_test_active = 0u;
          heaterControl.disable();
          state_.heater_test_duty = 0u;
        }
#endif
#if ENABLE_SERVICE_FOPDT_ID
        if (state_.service_view == kServiceViewFopdt)
        {
          fopdt.abort();
          g_fopdt_active = 0u;
          heaterControl.disable();
          state_.service_fopdt_page = 0u;
        }
#endif
#if ENABLE_SERVICE_IO_TEST
        if (state_.service_view == kServiceViewIoTest)
        {
          state_.io_test_out_mask = 0u;
          heaterControl.setDutyCycle(0.0f);
          heaterControl.disable();
          drumControl.stop();
          io.emergencyStop();
          state_.io_test_last_update_ms = 0u;
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

#if ENABLE_SETTINGS_MENU
    if (state_.current_state == SystemState::SETTINGS)
    {
      if (state_.settings_view != kSettingsViewMenu)
      {
#if ENABLE_PROGRAM_EDITOR
        if (state_.settings_view == kSettingsViewProgramEditConfirm)
        {
          state_.settings_view = kSettingsViewProgramEdit;
          renderSettings_();
          return;
        }
        if (state_.settings_view == kSettingsViewProgramEdit)
        {
          state_.settings_view = kSettingsViewProgramList;
          renderSettings_();
          return;
        }
        if (state_.settings_view == kSettingsViewProgramList)
        {
          state_.settings_view = kSettingsViewMenu;
          renderSettings_();
          return;
        }
#endif
        state_.settings_view = kSettingsViewMenu;
        renderSettings_();
        return;
      }

      transitionTo(SystemState::IDLE);
      return;
    }
#endif

#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE
    if (state_.current_state == SystemState::AUTOTUNE)
    {
      pidController.abortAutoTune();
      state_.service_view = kServiceViewMenu;
      transitionTo(SystemState::SERVICE);
      state_.invalid_key_until_ms = now + kInvalidKeyMsgMs;
      lcd.setCursor(0, 3);
      lcd.print(F("ABORTED             "));
      return;
    }
#endif

    if (state_.current_state == SystemState::FAULT)
    {
      // Fault clearing requires operator acknowledgment AND cleared conditions.
      if (faultMgr.canClearFault())
      {
        faultMgr.clearFault();
        transitionTo(SystemState::IDLE);
        return;
      }

      // Briefly show why it can't be cleared, then re-render FAULT screen.
      state_.invalid_key_until_ms = now + kInvalidKeyMsgMs;
      lcd.setCursor(0, 2);
      lcd.print(F("CONDITIONS NOT OK"));
      lcdPadSpacesToEol_(lcd, 17u);
      return;
    }

    if (state_.current_state == SystemState::READY)
    {
      transitionTo(SystemState::IDLE);
      return;
    }

    if (state_.current_state == SystemState::PARAM_EDIT && state_.param_mode == kParamModeAuto)
    {
      if ((state_.auto_flags & kAutoFlagEditing) != 0u)
      {
        // Discard edits and return to review.
        state_.auto_temp_c = state_.auto_program.temp_setpoint;
        state_.auto_duration_min = state_.auto_program.duration_min;
        state_.auto_flags = 0u;
        renderAutoParamReview_();
        return;
      }

      transitionTo(SystemState::PROGRAM_SELECT);
      return;
    }

#if ENABLE_CYCLE_EXECUTION
    if (state_.current_state == SystemState::PARAM_EDIT && state_.param_mode == kParamModeManual)
    {
      if (state_.manual_view == kManualViewDuration)
      {
        // Manual duration screen: STOP returns to temp screen (Req 12B.30).
        state_.manual_view = kManualViewTemp;
        renderManualTemp_();
        return;
      }
    }

    if (state_.current_state == SystemState::PAUSED)
    {
      // Req 12A/12B: B in PAUSED aborts to main menu.
      transitionTo(SystemState::IDLE);
      return;
    }

    if (state_.current_state == SystemState::ANTI_CREASE)
    {
      // Anti-crease exit (Req 11.7).
      drumControl.stop();
      transitionTo(SystemState::IDLE);
      return;
    }

    if (state_.current_state == SystemState::START_DELAY)
    {
      // Before cycle starts, STOP aborts directly.
      transitionTo(SystemState::IDLE);
      return;
    }

    if (state_.current_state == SystemState::RUNNING_HEAT)
    {
      if (state_.cycle_edit_active != 0u)
      {
        // Cancel in-cycle edit and return to running screen.
        state_.cycle_edit_active = 0u;
        renderRunningHeat_();
        state_.cycle_last_ui_ms = 0u;
        updateRunningHeatUi_();
        return;
      }

      // Req 12A/12B: B during cycle pauses.
      transitionTo(SystemState::PAUSED);
      return;
    }

    if (state_.current_state == SystemState::RUNNING_COOLDOWN)
    {
      // Req 26: B during cooldown pauses.
      transitionTo(SystemState::PAUSED);
      return;
    }
#endif

    if (state_.current_state == SystemState::PROGRAM_SELECT || state_.current_state == SystemState::PARAM_EDIT)
    {
      transitionTo(SystemState::IDLE);
      return;
    }

    // In IDLE, STOP has no action.
    return;
  }

  switch (state_.current_state)
  {
  case SystemState::IDLE:
  {
    if (key == KeypadInput::Key::KEY_STAR)
    {
      // Entry sequences from IDLE:
      //  - **5 => Service menu (Req 19)
      //  - **8 => Settings menu (Phase 9 task)
      if (state_.entry_seq == 0u)
      {
        state_.entry_seq = 1u;
        state_.entry_seq_start_ms = now;
        return;
      }
      if (state_.entry_seq == 1u)
      {
        state_.entry_seq = 2u;
        state_.entry_seq_start_ms = now;
        return;
      }
      state_.entry_seq = 1u;
      state_.entry_seq_start_ms = now;
      return;
    }

    if (state_.entry_seq == 2u && (now - state_.entry_seq_start_ms) <= kEntrySeqTimeoutMs)
    {
#if ENABLE_SERVICE_MENU
      if (key == KeypadInput::Key::OK)
      {
        state_.entry_seq = 0u;
        state_.service_menu_selection = 0u;
        state_.service_view = kServiceViewMenu;
        transitionTo(SystemState::SERVICE);
        return;
      }
#endif
#if ENABLE_SETTINGS_MENU
      if (key == KeypadInput::Key::LEFT)
      {
        state_.entry_seq = 0u;
        state_.settings_menu_selection = 0u;
        state_.settings_view = kSettingsViewMenu;
        transitionTo(SystemState::SETTINGS);
        return;
      }
#endif
    }

    state_.entry_seq = 0u;

    if (key == KeypadInput::Key::UP)
    {
      state_.menu_selection = (state_.menu_selection == 0u) ? 1u : 0u;
      lcd.showMainMenu(state_.menu_selection);
      state_.last_temp_valid = tempSensor.isValid() ? 1u : 0u;
      lcd.showTemperature(tempSensor.getTemperature(), state_.last_temp_valid != 0u);
      return;
    }

    if (key == KeypadInput::Key::DOWN)
    {
      state_.menu_selection = (state_.menu_selection == 0u) ? 1u : 0u;
      lcd.showMainMenu(state_.menu_selection);
      state_.last_temp_valid = tempSensor.isValid() ? 1u : 0u;
      lcd.showTemperature(tempSensor.getTemperature(), state_.last_temp_valid != 0u);
      return;
    }

    if (key == KeypadInput::Key::OK)
    {
      if (state_.menu_selection == 0u)
      {
        state_.param_mode = kParamModeAuto;
        transitionTo(SystemState::PROGRAM_SELECT);
      }
      else
      {
        state_.param_mode = kParamModeManual;
        transitionTo(SystemState::PARAM_EDIT);
      }
      return;
    }

    showInvalidKey_();
    return;
  }

  case SystemState::PROGRAM_SELECT:
  {
    if (key == KeypadInput::Key::UP)
    {
      state_.auto_program_index =
          (state_.auto_program_index == 0u) ? static_cast<uint8_t>(EEPROMStore::PROGRAM_COUNT - 1u)
                                            : static_cast<uint8_t>(state_.auto_program_index - 1u);
      (void)eepromStore.loadProgram(state_.auto_program_index, state_.auto_program);
      state_.auto_temp_c = state_.auto_program.temp_setpoint;
      state_.auto_duration_min = state_.auto_program.duration_min;
      state_.auto_flags = 0u;
      renderProgramSelect_();
      return;
    }

    if (key == KeypadInput::Key::DOWN)
    {
      state_.auto_program_index =
          static_cast<uint8_t>((state_.auto_program_index + 1u) % EEPROMStore::PROGRAM_COUNT);
      (void)eepromStore.loadProgram(state_.auto_program_index, state_.auto_program);
      state_.auto_temp_c = state_.auto_program.temp_setpoint;
      state_.auto_duration_min = state_.auto_program.duration_min;
      state_.auto_flags = 0u;
      renderProgramSelect_();
      return;
    }

    if (key == KeypadInput::Key::OK)
    {
      state_.param_mode = kParamModeAuto;
      transitionTo(SystemState::PARAM_EDIT);
      return;
    }

    showInvalidKey_();
    return;
  }

  case SystemState::PARAM_EDIT:
  {
    if (state_.param_mode != kParamModeAuto)
    {
#if ENABLE_CYCLE_EXECUTION
      // Manual mode parameter entry (Req 12B).
      if (state_.manual_view == kManualViewTemp)
      {
        if (key == KeypadInput::Key::UP || key == KeypadInput::Key::DOWN)
        {
          const bool inc = (key == KeypadInput::Key::UP);
          uint8_t t_c = state_.manual_temp_c;
#if ENABLE_SETTINGS_MENU
          if (getTempUnit_() == kTempUnitF)
          {
            const uint8_t min_f = cToF_u8(MIN_TEMP);
            const uint8_t max_f = cToF_u8(MAX_TEMP);
            uint8_t t_f = cToF_u8(t_c);
            t_f = wrapStepU8(t_f, 5u, min_f, max_f, inc);
            t_c = fToC_u8(t_f);
            if (t_c < MIN_TEMP)
              t_c = MIN_TEMP;
            if (t_c > MAX_TEMP)
              t_c = MAX_TEMP;
          }
          else
#endif
          {
            t_c = wrapStepU8(t_c, 5u, MIN_TEMP, MAX_TEMP, inc);
          }
          state_.manual_temp_c = t_c;
          renderManualTemp_();
          return;
        }
        if (key == KeypadInput::Key::OK)
        {
          state_.manual_view = kManualViewDuration;
          renderManualDuration_();
          return;
        }

        showInvalidKey_();
        return;
      }

      // Duration input view.
      if (key == KeypadInput::Key::UP || key == KeypadInput::Key::DOWN)
      {
        const bool inc = (key == KeypadInput::Key::UP);
        uint8_t m = state_.manual_duration_min;
        m = wrapStepU8(m, 5u, 10u, 120u, inc);
        state_.manual_duration_min = m;
        renderManualDuration_();
        return;
      }
      if (key == KeypadInput::Key::OK)
      {
        transitionTo(SystemState::READY);
        return;
      }

      showInvalidKey_();
      return;
#else
      showInvalidKey_();
      return;
#endif
    }

    const bool editing = (state_.auto_flags & kAutoFlagEditing) != 0u;

    if (!editing)
    {
      if (key == KeypadInput::Key::START)
      {
#if ENABLE_CYCLE_EXECUTION
        state_.cycle_setpoint_c = state_.auto_temp_c;
        state_.cycle_duration_min = state_.auto_duration_min;
        state_.cycle_duty_limit = state_.auto_program.duty_limit;
        transitionTo(SystemState::START_DELAY);
#else
        showInvalidKey_();
#endif
        return;
      }
      if (key == KeypadInput::Key::KEY_STAR)
      {
        state_.auto_flags |= kAutoFlagEditing; // start editing on TEMP
        state_.auto_flags &= static_cast<uint8_t>(~kAutoFlagFieldTime);
        renderAutoParamEdit_();
        return;
      }

      showInvalidKey_();
      return;
    }

    if (key == KeypadInput::Key::LEFT || key == KeypadInput::Key::RIGHT)
    {
      state_.auto_flags ^= kAutoFlagFieldTime;
      renderAutoParamEdit_();
      return;
    }

    if (key == KeypadInput::Key::UP || key == KeypadInput::Key::DOWN)
    {
      const bool field_time = (state_.auto_flags & kAutoFlagFieldTime) != 0u;
      if (!field_time)
      {
        const bool inc = (key == KeypadInput::Key::UP);
        uint8_t t_c = state_.auto_temp_c;
#if ENABLE_SETTINGS_MENU
        if (getTempUnit_() == kTempUnitF)
        {
          const uint8_t min_f = cToF_u8(MIN_TEMP);
          const uint8_t max_f = cToF_u8(MAX_TEMP);
          uint8_t t_f = cToF_u8(t_c);
          t_f = wrapStepU8(t_f, 5u, min_f, max_f, inc);
          t_c = fToC_u8(t_f);
          if (t_c < MIN_TEMP)
            t_c = MIN_TEMP;
          if (t_c > MAX_TEMP)
            t_c = MAX_TEMP;
        }
        else
#endif
        {
          t_c = wrapStepU8(t_c, 5u, MIN_TEMP, MAX_TEMP, inc);
        }
        state_.auto_temp_c = t_c;
      }
      else
      {
        uint8_t m = state_.auto_duration_min;
        if (key == KeypadInput::Key::UP)
        {
          m = static_cast<uint8_t>(m + 5u);
          if (m > 120u)
            m = 10u;
        }
        else
        {
          if (m <= 10u)
          {
            m = 120u;
          }
          else
          {
            m = static_cast<uint8_t>(m - 5u);
          }
        }
        state_.auto_duration_min = m;
      }
      renderAutoParamEdit_();
      return;
    }

    if (key == KeypadInput::Key::OK)
    {
      // Save edits (temporary, not persisted to EEPROM).
      state_.auto_flags &= static_cast<uint8_t>(~kAutoFlagEditing);

      if (state_.auto_temp_c != state_.auto_program.temp_setpoint)
      {
        state_.auto_flags |= kAutoFlagTempModified;
      }
      else
      {
        state_.auto_flags &= static_cast<uint8_t>(~kAutoFlagTempModified);
      }
      if (state_.auto_duration_min != state_.auto_program.duration_min)
      {
        state_.auto_flags |= kAutoFlagTimeModified;
      }
      else
      {
        state_.auto_flags &= static_cast<uint8_t>(~kAutoFlagTimeModified);
      }

      renderAutoParamReview_();
      return;
    }

    showInvalidKey_();
    return;
  }

#if ENABLE_CYCLE_EXECUTION
  case SystemState::READY:
  {
    if (state_.param_mode != kParamModeManual)
    {
      showInvalidKey_();
      return;
    }

    if (key == KeypadInput::Key::START)
    {
      state_.cycle_setpoint_c = state_.manual_temp_c;
      state_.cycle_duration_min = state_.manual_duration_min;
      state_.cycle_duty_limit = 100u;
      transitionTo(SystemState::START_DELAY);
      return;
    }
    if (key == KeypadInput::Key::KEY_STAR)
    {
      state_.manual_view = kManualViewTemp;
      transitionTo(SystemState::PARAM_EDIT);
      return;
    }

    showInvalidKey_();
    return;
  }

  case SystemState::PAUSED:
  {
    if (state_.pause_timeout_shown != 0u)
    {
      // Ignored while showing timeout message.
      return;
    }

    if (key == KeypadInput::Key::START)
    {
      const uint32_t pause_ms = now - state_.pause_start_ms;
      if (state_.paused_from_state == SystemState::RUNNING_HEAT)
      {
        state_.heat_pause_total_ms += pause_ms;
      }
      else if (state_.paused_from_state == SystemState::RUNNING_COOLDOWN)
      {
        state_.cooldown_pause_total_ms += pause_ms;
      }

      transitionTo(state_.paused_from_state);
      return;
    }

    showInvalidKey_();
    return;
  }

  case SystemState::RUNNING_HEAT:
  {
    // In-cycle edit mode (Req 13 / Phase 11.4).
    if (state_.cycle_edit_active != 0u)
    {
      if (key == KeypadInput::Key::KEY_STAR)
      {
        // Cancel edit without applying.
        state_.cycle_edit_active = 0u;
        renderRunningHeat_();
        state_.cycle_last_ui_ms = 0u;
        updateRunningHeatUi_();
        return;
      }

      if (key == KeypadInput::Key::LEFT || key == KeypadInput::Key::RIGHT)
      {
        state_.cycle_edit_field ^= 1u;
        renderCycleEdit_();
        return;
      }

      if (key == KeypadInput::Key::UP || key == KeypadInput::Key::DOWN)
      {
        const bool inc = (key == KeypadInput::Key::UP);

        if (state_.cycle_edit_field == 0u)
        {
          // TEMP limits: ±10C around original setpoint, clamped to global MIN/MAX.
          uint8_t min_c = (state_.cycle_orig_setpoint_c > 10u)
                              ? static_cast<uint8_t>(state_.cycle_orig_setpoint_c - 10u)
                              : 0u;
          uint8_t max_c = static_cast<uint8_t>(state_.cycle_orig_setpoint_c + 10u);
          if (min_c < MIN_TEMP)
            min_c = MIN_TEMP;
          if (max_c > MAX_TEMP)
            max_c = MAX_TEMP;

          uint8_t t_c = state_.cycle_edit_temp_c;
#if ENABLE_SETTINGS_MENU
          if (getTempUnit_() == kTempUnitF)
          {
            const uint8_t min_f = cToF_u8(min_c);
            const uint8_t max_f = cToF_u8(max_c);
            uint8_t t_f = cToF_u8(t_c);
            if (inc)
            {
              t_f = (t_f >= max_f) ? max_f : static_cast<uint8_t>(t_f + 5u);
            }
            else
            {
              t_f = (t_f <= min_f) ? min_f : static_cast<uint8_t>(t_f - 5u);
            }
            t_c = fToC_u8(t_f);
            if (t_c < min_c)
              t_c = min_c;
            if (t_c > max_c)
              t_c = max_c;
          }
          else
#endif
          {
            if (inc)
            {
              t_c = (t_c >= max_c) ? max_c : static_cast<uint8_t>(t_c + 5u);
            }
            else
            {
              t_c = (t_c <= min_c) ? min_c : static_cast<uint8_t>(t_c - 5u);
            }
          }
          state_.cycle_edit_temp_c = t_c;
        }
        else
        {
          // TIME limits: ±20 min around original duration.
          uint8_t min_m = (state_.cycle_orig_duration_min > 20u)
                              ? static_cast<uint8_t>(state_.cycle_orig_duration_min - 20u)
                              : 0u;
          uint8_t max_m = static_cast<uint8_t>(state_.cycle_orig_duration_min + 20u);
          if (min_m < 10u)
            min_m = 10u;
          if (max_m > 120u)
            max_m = 120u;

          uint8_t m = state_.cycle_edit_duration_min;
          if (inc)
          {
            m = (m >= max_m) ? max_m : static_cast<uint8_t>(m + 5u);
          }
          else
          {
            m = (m <= min_m) ? min_m : static_cast<uint8_t>(m - 5u);
          }
          state_.cycle_edit_duration_min = m;
        }

        renderCycleEdit_();
        return;
      }

      if (key == KeypadInput::Key::OK)
      {
        // Apply edits (temporary, not persisted).
        state_.cycle_setpoint_c = state_.cycle_edit_temp_c;
        state_.cycle_duration_min = state_.cycle_edit_duration_min;

        state_.cycle_temp_modified = (state_.cycle_setpoint_c != state_.cycle_orig_setpoint_c) ? 1u : 0u;
        state_.cycle_time_modified = (state_.cycle_duration_min != state_.cycle_orig_duration_min) ? 1u : 0u;

        pidController.setSetpoint(static_cast<float>(state_.cycle_setpoint_c));

        state_.cycle_edit_active = 0u;
        renderRunningHeat_();
        state_.cycle_last_ui_ms = 0u;
        updateRunningHeatUi_();
        return;
      }

      showInvalidKey_();
      return;
    }

    if (key == KeypadInput::Key::KEY_STAR)
    {
      state_.cycle_edit_active = 1u;
      state_.cycle_edit_field = 0u;
      state_.cycle_edit_temp_c = state_.cycle_setpoint_c;
      state_.cycle_edit_duration_min = state_.cycle_duration_min;
      renderCycleEdit_();
      return;
    }

    showInvalidKey_();
    return;
  }

  case SystemState::RUNNING_COOLDOWN:
    showInvalidKey_();
    return;

  case SystemState::ANTI_CREASE:
    if (state_.anticrease_page == 0u)
    {
      // Cycle complete summary: any key returns to IDLE (Req 12A/12B).
      transitionTo(SystemState::IDLE);
      return;
    }
    showInvalidKey_();
    return;
#endif

#if ENABLE_SETTINGS_MENU
  case SystemState::SETTINGS:
  {
    // Views are handled inside SETTINGS without introducing more global states.
    if (state_.settings_view == kSettingsViewMenu)
    {
      const uint8_t max_sel =
#if ENABLE_PROGRAM_EDITOR
          2u;
#else
          1u;
#endif
      if (key == KeypadInput::Key::UP)
      {
        state_.settings_menu_selection =
            (state_.settings_menu_selection == 0u) ? max_sel
                                                   : static_cast<uint8_t>(state_.settings_menu_selection - 1u);
        renderSettingsMenu_();
        return;
      }
      if (key == KeypadInput::Key::DOWN)
      {
        state_.settings_menu_selection =
            (state_.settings_menu_selection >= max_sel) ? 0u
                                                        : static_cast<uint8_t>(state_.settings_menu_selection + 1u);
        renderSettingsMenu_();
        return;
      }
      if (key == KeypadInput::Key::OK)
      {
        if (state_.settings_menu_selection == 0u)
        {
          state_.settings_view = kSettingsViewTempUnits;
          renderSettings_();
          return;
        }
        if (state_.settings_menu_selection == 1u)
        {
          state_.settings_view = kSettingsViewSound;
          renderSettings_();
          return;
        }
#if ENABLE_PROGRAM_EDITOR
        if (state_.settings_menu_selection == 2u)
        {
          state_.settings_view = kSettingsViewProgramList;
          state_.program_list_selection = 0u;
          renderSettings_();
          return;
        }
#endif
        showInvalidKey_();
        return;
      }
      showInvalidKey_();
      return;
    }

    if (state_.settings_view == kSettingsViewTempUnits)
    {
      if (key == KeypadInput::Key::UP || key == KeypadInput::Key::DOWN)
      {
        g_settings.temp_unit ^= 1u;
        renderSettingsTempUnits_();
        return;
      }
      if (key == KeypadInput::Key::OK)
      {
        eepromStore.requestSaveSettings(g_settings);
        eepromStore.update(now);
        lcd.setTempUnit(g_settings.temp_unit);
        state_.settings_view = kSettingsViewMenu;
        renderSettings_();
        return;
      }
      showInvalidKey_();
      return;
    }

    if (state_.settings_view == kSettingsViewSound)
    {
      if (key == KeypadInput::Key::UP || key == KeypadInput::Key::DOWN)
      {
        g_settings.sound_enabled ^= 1u;
        renderSettingsSound_();
        return;
      }
      if (key == KeypadInput::Key::OK)
      {
        eepromStore.requestSaveSettings(g_settings);
        eepromStore.update(now);
        state_.settings_view = kSettingsViewMenu;
        renderSettings_();
        return;
      }
      showInvalidKey_();
      return;
    }

#if ENABLE_PROGRAM_EDITOR
    if (state_.settings_view == kSettingsViewProgramList)
    {
      if (key == KeypadInput::Key::UP)
      {
        state_.program_list_selection =
            (state_.program_list_selection == 0u) ? static_cast<uint8_t>(EEPROMStore::PROGRAM_COUNT - 1u)
                                                  : static_cast<uint8_t>(state_.program_list_selection - 1u);
        renderProgramList_();
        return;
      }
      if (key == KeypadInput::Key::DOWN)
      {
        state_.program_list_selection =
            static_cast<uint8_t>((state_.program_list_selection + 1u) % EEPROMStore::PROGRAM_COUNT);
        renderProgramList_();
        return;
      }
      if (key == KeypadInput::Key::OK)
      {
        state_.program_edit_index = state_.program_list_selection;
        (void)eepromStore.loadProgram(state_.program_edit_index, state_.program_edit);
        // Ensure name editing never sees embedded NULs (keep NUL only at [12]).
        for (uint8_t i = 0u; i < 12u; i++)
        {
          if (state_.program_edit.name[i] == '\0')
          {
            state_.program_edit.name[i] = ' ';
          }
        }
        state_.program_edit.name[12] = '\0';

        state_.program_edit_field = kProgFieldTemp;
        state_.program_name_cursor = 0u;
        state_.settings_view = kSettingsViewProgramEdit;
        renderProgramEdit_();
        return;
      }
      showInvalidKey_();
      return;
    }

    if (state_.settings_view == kSettingsViewProgramEditConfirm)
    {
      if (key == KeypadInput::Key::START)
      {
        eepromStore.requestSaveProgram(state_.program_edit_index, state_.program_edit);
        eepromStore.update(now);
        state_.settings_view = kSettingsViewProgramList;
        renderProgramList_();
        state_.invalid_key_until_ms = now + kInvalidKeyMsgMs;
        lcd.setCursor(0, 3);
        lcd.print(F("SAVED               "));
        return;
      }
      showInvalidKey_();
      return;
    }

    if (state_.settings_view == kSettingsViewProgramEdit)
    {
      if (key == KeypadInput::Key::LEFT)
      {
        if (state_.program_edit_field == kProgFieldName)
        {
          state_.program_name_cursor =
              (state_.program_name_cursor == 0u) ? 11u : static_cast<uint8_t>(state_.program_name_cursor - 1u);
        }
        else
        {
          state_.program_edit_field = (state_.program_edit_field == 0u)
                                          ? static_cast<uint8_t>(kProgFieldCount - 1u)
                                          : static_cast<uint8_t>(state_.program_edit_field - 1u);
        }
        renderProgramEdit_();
        return;
      }
      if (key == KeypadInput::Key::RIGHT)
      {
        if (state_.program_edit_field == kProgFieldName)
        {
          state_.program_name_cursor =
              (state_.program_name_cursor >= 11u) ? 0u : static_cast<uint8_t>(state_.program_name_cursor + 1u);
        }
        else
        {
          state_.program_edit_field = (state_.program_edit_field >= static_cast<uint8_t>(kProgFieldCount - 1u))
                                          ? 0u
                                          : static_cast<uint8_t>(state_.program_edit_field + 1u);
        }
        renderProgramEdit_();
        return;
      }

      if (key == KeypadInput::Key::OK)
      {
        if (state_.program_edit_field == kProgFieldName)
        {
          state_.program_edit_field = kProgFieldTemp;
        }
        else
        {
          state_.program_edit_field = (state_.program_edit_field >= static_cast<uint8_t>(kProgFieldCount - 1u))
                                          ? 0u
                                          : static_cast<uint8_t>(state_.program_edit_field + 1u);
        }
        renderProgramEdit_();
        return;
      }

      if (key == KeypadInput::Key::UP || key == KeypadInput::Key::DOWN)
      {
        const bool inc = (key == KeypadInput::Key::UP);
        switch (state_.program_edit_field)
        {
        case kProgFieldName:
        {
          uint8_t pos = (state_.program_name_cursor >= 12u) ? 0u : state_.program_name_cursor;
          state_.program_edit.name[pos] = cycleNameChar_(state_.program_edit.name[pos], inc);
          state_.program_edit.name[12] = '\0';
          break;
        }
        case kProgFieldTemp:
        {
          uint8_t t_c = state_.program_edit.temp_setpoint;
#if ENABLE_SETTINGS_MENU
          if (getTempUnit_() == kTempUnitF)
          {
            const uint8_t min_f = cToF_u8(MIN_TEMP);
            const uint8_t max_f = cToF_u8(MAX_TEMP);
            uint8_t t_f = cToF_u8(t_c);
            t_f = wrapStepU8(t_f, 5u, min_f, max_f, inc);
            t_c = fToC_u8(t_f);
            if (t_c < MIN_TEMP)
              t_c = MIN_TEMP;
            if (t_c > MAX_TEMP)
              t_c = MAX_TEMP;
          }
          else
#endif
          {
            t_c = wrapStepU8(t_c, 5u, MIN_TEMP, MAX_TEMP, inc);
          }
          state_.program_edit.temp_setpoint = t_c;
          break;
        }
        case kProgFieldTime:
        {
          state_.program_edit.duration_min =
              wrapStepU8(state_.program_edit.duration_min, 5u, 10u, 120u, inc);
          break;
        }
        case kProgFieldFwd:
        {
          state_.program_edit.fwd_time_s = wrapStepU8(state_.program_edit.fwd_time_s, 5u, 30u, 90u, inc);
          break;
        }
        case kProgFieldRev:
        {
          state_.program_edit.rev_time_s = wrapStepU8(state_.program_edit.rev_time_s, 5u, 30u, 90u, inc);
          break;
        }
        case kProgFieldStop:
        {
          state_.program_edit.stop_time_s = wrapStepU8(state_.program_edit.stop_time_s, 1u, 3u, 15u, inc);
          break;
        }
        default:
        { // kProgFieldDuty
          state_.program_edit.duty_limit = wrapStepU8(state_.program_edit.duty_limit, 5u, 0u, 100u, inc);
          break;
        }
        }
        renderProgramEdit_();
        return;
      }

      if (key == KeypadInput::Key::START)
      {
        state_.settings_view = kSettingsViewProgramEditConfirm;
        renderProgramEdit_();
        return;
      }

      showInvalidKey_();
      return;
    }
#endif // ENABLE_PROGRAM_EDITOR

    showInvalidKey_();
    return;
  }
#endif // ENABLE_SETTINGS_MENU

#if ENABLE_SERVICE_MENU
  case SystemState::SERVICE:
  {
#if ENABLE_SERVICE_HEATER_TEST
    if (state_.service_view == kServiceViewHeaterTest)
    {
      if (key == KeypadInput::Key::UP || key == KeypadInput::Key::DOWN)
      {
        uint8_t duty = state_.heater_test_duty;
        if (key == KeypadInput::Key::UP)
        {
          duty = (duty >= 95u) ? 100u : static_cast<uint8_t>(duty + 5u);
        }
        else
        {
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

#if ENABLE_SERVICE_IO_TEST
    if (state_.service_view == kServiceViewIoTest)
    {
      // Page toggle.
      if (key == KeypadInput::Key::LEFT || key == KeypadInput::Key::RIGHT)
      {
        state_.io_test_page ^= 1u;
        state_.io_test_last_update_ms = 0u;
        updateIoTestUi_();
        return;
      }

      if (state_.io_test_page != 0u)
      {
        showInvalidKey_();
        return;
      }

      if (key == KeypadInput::Key::UP)
      {
        state_.io_test_selection =
            (state_.io_test_selection == 0u) ? static_cast<uint8_t>(kIoOutputCount - 1u)
                                             : static_cast<uint8_t>(state_.io_test_selection - 1u);
        state_.io_test_last_update_ms = 0u;
        updateIoTestUi_();
        return;
      }

      if (key == KeypadInput::Key::DOWN)
      {
        state_.io_test_selection = static_cast<uint8_t>((state_.io_test_selection + 1u) % kIoOutputCount);
        state_.io_test_last_update_ms = 0u;
        updateIoTestUi_();
        return;
      }

      if (key == KeypadInput::Key::OK)
      {
        if (!io.isDoorClosed())
        {
          state_.invalid_key_until_ms = now + kInvalidKeyMsgMs;
          lcd.setCursor(0, 3);
          lcd.print(F("CLOSE DOOR          "));
          return;
        }

        uint8_t mask = state_.io_test_out_mask;
        const uint8_t sel = (state_.io_test_selection >= kIoOutputCount) ? 0u : state_.io_test_selection;

        if (sel == 0u)
        {
          // Heater: ON=100% duty, OFF=disabled. HeaterControl enforces interlocks.
          mask ^= kIoMaskHeater;
          state_.io_test_out_mask = mask;

          if ((mask & kIoMaskHeater) != 0u)
          {
            heaterControl.setDutyCycle(100.0f);
            heaterControl.enable();
          }
          else
          {
            heaterControl.setDutyCycle(0.0f);
            heaterControl.disable();
          }

          state_.io_test_last_update_ms = 0u;
          updateIoTestUi_();
          return;
        }

        if (sel == 1u)
        {
          // Motor forward: enforce mutual exclusion with reverse.
          const bool on = (mask & kIoMaskMotorFwd) != 0u;
          if (on)
          {
            mask &= static_cast<uint8_t>(~kIoMaskMotorFwd);
          }
          else
          {
            mask |= kIoMaskMotorFwd;
            mask &= static_cast<uint8_t>(~kIoMaskMotorRev);
          }
          state_.io_test_out_mask = mask;
          state_.io_test_last_update_ms = 0u;
          updateIoTestUi_();
          return;
        }

        if (sel == 2u)
        {
          // Motor reverse: enforce mutual exclusion with forward.
          const bool on = (mask & kIoMaskMotorRev) != 0u;
          if (on)
          {
            mask &= static_cast<uint8_t>(~kIoMaskMotorRev);
          }
          else
          {
            mask |= kIoMaskMotorRev;
            mask &= static_cast<uint8_t>(~kIoMaskMotorFwd);
          }
          state_.io_test_out_mask = mask;
          state_.io_test_last_update_ms = 0u;
          updateIoTestUi_();
          return;
        }

        // AUX relay toggle (reserved).
        mask ^= kIoMaskAux;
        state_.io_test_out_mask = mask;
        state_.io_test_last_update_ms = 0u;
        updateIoTestUi_();
        return;
      }

      showInvalidKey_();
      return;
    }
#endif

#if ENABLE_SERVICE_MEMORY_INFO
    if (state_.service_view == kServiceViewMemoryInfo)
    {
      if (key == KeypadInput::Key::OK || key == KeypadInput::Key::LEFT || key == KeypadInput::Key::RIGHT)
      {
        state_.service_mem_page ^= 1u;
        state_.service_mem_last_update_ms = 0u;
        renderMemoryInfo_();
        return;
      }

      showInvalidKey_();
      return;
    }
#endif

#if ENABLE_SERVICE_FAULT_HISTORY
    if (state_.service_view == kServiceViewFaultHistory)
    {
      if (state_.fault_history_page == 0u)
      {
        // Scroll entries (0 = most recent).
        if (key == KeypadInput::Key::UP || key == KeypadInput::Key::DOWN)
        {
          uint8_t count = 0u;
          EEPROMStore::FaultEntry tmp;
          for (uint8_t i = 0u; i < EEPROMStore::FAULT_HISTORY_COUNT; i++)
          {
            if (!eepromStore.getFaultHistory(i, tmp))
            {
              break;
            }
            count++;
          }

          if (count != 0u)
          {
            uint8_t idx = state_.fault_history_index;
            if (key == KeypadInput::Key::UP)
            {
              idx = (idx == 0u) ? static_cast<uint8_t>(count - 1u) : static_cast<uint8_t>(idx - 1u);
            }
            else
            {
              idx = static_cast<uint8_t>((idx + 1u) % count);
            }
            state_.fault_history_index = idx;
            renderFaultHistory_();
          }
          return;
        }

        if (key == KeypadInput::Key::KEY_C)
        {
          // Confirmation prompt (Req 18.6).
          state_.fault_history_page = 1u;
          renderFaultHistory_();
          return;
        }

        showInvalidKey_();
        return;
      }

      // Page: confirm clear.
      if (key == KeypadInput::Key::START)
      {
        eepromStore.clearFaultHistory();
        state_.fault_history_page = 0u;
        state_.fault_history_index = 0u;
        renderFaultHistory_();
        state_.invalid_key_until_ms = now + kInvalidKeyMsgMs;
        lcd.setCursor(0, 3);
        lcd.print(F("HISTORY CLEARED"));
        lcdPadSpacesToEol_(lcd, 15u);
        return;
      }

      showInvalidKey_();
      return;
    }
#endif

#if ENABLE_SERVICE_FOPDT_ID
    if (state_.service_view == kServiceViewFopdt)
    {
      const FOPDTModel::State fst = fopdt.getState();

      if (key == KeypadInput::Key::OK && fst == FOPDTModel::State::COMPLETE)
      {
        state_.service_fopdt_page ^= 1u;
        renderFopdt_();
        return;
      }

      if (key == KeypadInput::Key::START)
      {
        if (fst == FOPDTModel::State::IDLE)
        {
          const bool door_ok = io.isDoorClosed();
          const bool sens_ok = tempSensor.isValid();
          const float pv = tempSensor.getTemperature();
          const bool ambient_ok = (pv >= kFopdtAmbientMinC) && (pv <= kFopdtAmbientMaxC);

          if (!door_ok || !sens_ok || !ambient_ok)
          {
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

        if (fst == FOPDTModel::State::COMPLETE)
        {
          float k = 0.0f;
          float tau_s = 0.0f;
          float l_s = 0.0f;
          if (fopdt.getResults(k, tau_s, l_s))
          {
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
    if (state_.service_view == kServiceViewAutoTunePre)
    {
#if ENABLE_SERVICE_FOPDT_ID
      (void)fopdt; // suppress unused warnings when combined builds toggle flags
#endif
      if (key == KeypadInput::Key::START)
      {
        const bool door_ok = io.isDoorClosed();
        const bool sens_ok = tempSensor.isValid();
        const float pv = tempSensor.getTemperature();
        const bool ambient_ok = (pv >= 15.0f) && (pv <= 30.0f);

        if (!door_ok || !sens_ok || !ambient_ok)
        {
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

#if ENABLE_SERVICE_FACTORY_RESET
    if (state_.service_view == kServiceViewFactoryReset)
    {
      if (key == KeypadInput::Key::START)
      {
        // Safety: ensure outputs are de-energized before the intentional reboot.
        drumControl.stop();
        heaterControl.disable();
        io.emergencyStop();

        eepromStore.factoryReset();

#if ENABLE_SETTINGS_MENU
        // Reload settings so subsequent displays (if any) match defaults.
        (void)eepromStore.loadSettings(g_settings);
        lcd.setTempUnit(g_settings.temp_unit);
#endif

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("RESET COMPLETE"));
        lcdPadSpacesToEol_(lcd, 14u);
        lcd.setCursor(0, 1);
        lcd.print(F("RESTARTING"));
        lcdPadSpacesToEol_(lcd, 10u);
        lcd.setCursor(0, 2);
        lcd.print(F("                "));

        // Restart the controller to apply defaults.
#if defined(ESP32)
        delay(250);
        ESP.restart();
#elif defined(__AVR__)
        // Hold here and let watchdog reset the MCU (Req 30).
        wdt_enable(WDTO_2S);
        while (true)
        {
          // intentional: no wdt_reset()
        }
#else
        while (true)
        {
          // Fallback: stop execution.
        }
#endif
      }

      showInvalidKey_();
      return;
    }
#endif

    if (state_.service_view != kServiceViewMenu)
    {
      showInvalidKey_();
      return;
    }

    if (key == KeypadInput::Key::UP)
    {
      state_.service_menu_selection =
          (state_.service_menu_selection == 0u) ? (kServiceMenuItems - 1u) : (state_.service_menu_selection - 1u);
      renderServiceMenu_();
      return;
    }

    if (key == KeypadInput::Key::DOWN)
    {
      state_.service_menu_selection =
          static_cast<uint8_t>((state_.service_menu_selection + 1u) % kServiceMenuItems);
      renderServiceMenu_();
      return;
    }

    if (key == KeypadInput::Key::OK)
    {
      const ServiceItem item = serviceMenuItemByIndex_(state_.service_menu_selection);

      switch (item)
      {
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
        // Ensure other service tools are inactive.
        drumControl.stop();
#if ENABLE_SERVICE_HEATER_TEST
        g_heater_test_active = 0u;
        state_.heater_test_duty = 0u;
#endif
#if ENABLE_SERVICE_FOPDT_ID
        g_fopdt_active = 0u;
        fopdt.abort();
#endif
        heaterControl.setDutyCycle(0.0f);
        heaterControl.disable();
        io.emergencyStop();

        state_.io_test_page = 0u;
        state_.io_test_selection = 0u;
        state_.io_test_out_mask = 0u;
        state_.io_test_last_update_ms = 0u;
        state_.service_view = kServiceViewIoTest;
        renderService_();
        return;
#endif
#if ENABLE_SERVICE_MEMORY_INFO
      case ServiceItem::MEMORY_INFO:
        state_.service_mem_page = 0u;
        state_.service_mem_last_update_ms = 0u;
        state_.service_view = kServiceViewMemoryInfo;
        renderService_();
        return;
#endif
#if ENABLE_SERVICE_FAULT_HISTORY
      case ServiceItem::FAULT_HISTORY:
        state_.fault_history_index = 0u;
        state_.fault_history_page = 0u;
        state_.service_view = kServiceViewFaultHistory;
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
#if ENABLE_SERVICE_FACTORY_RESET
      case ServiceItem::FACTORY_RESET:
        state_.service_view = kServiceViewFactoryReset;
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
  case SystemState::AUTOTUNE:
  {
    if (pidController.isAutoTuneComplete())
    {
      if (key == KeypadInput::Key::OK)
      {
        state_.service_autotune_page ^= 1u;
        renderAutoTune_();
        return;
      }

      if (key == KeypadInput::Key::START)
      {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        if (pidController.getAutoTuneResults(kp, ki, kd))
        {
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

  default:
    showInvalidKey_();
    return;
  }
}
