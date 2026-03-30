#include "faults.h"

#include <Arduino.h>
#include <pgmspace.h>

#include "app.h"
#include "config_build.h"
#include "ds18b20_sensor.h"
#include "eeprom_store.h"
#include "drum_control.h"
#include "heater_control.h"
#include "io_abstraction.h"
#include "pid_control.h"

extern IOAbstraction io;
extern DS18B20Sensor tempSensor;
extern DrumControl drumControl;
extern HeaterControl heaterControl;
extern PIDControl pidController;
extern EEPROMStore eepromStore;
extern AppStateMachine app;

namespace {
constexpr uint32_t kSensorFaultDelayMs = 2000u;
constexpr uint32_t kHeatTimeoutMs = 90ul * 60ul * 1000ul;
constexpr uint32_t kRunawaySampleMs = 10000u;

// Requirement 17.9 / Appendix C: require <50C before clearing thermal faults.
constexpr float kThermalClearC = 50.0f;

// Requirement 100: trip output fault after 3 consecutive mismatches.
constexpr uint8_t kOutputMismatchTrip = 3u;

int16_t toDeciC(float temp_c) {
  const float x = temp_c * 10.0f;
  return static_cast<int16_t>(x + ((x >= 0.0f) ? 0.5f : -0.5f));
}

// Fault messages in PROGMEM (Flash). Keep each string <= 20 chars to fit LCD rows.
const char FAULT_MSG_NONE[] PROGMEM = "NO FAULT";
const char FAULT_MSG_DOOR[] PROGMEM = "DOOR OPEN";
const char FAULT_MSG_TEMP[] PROGMEM = "TEMP SENSOR FAULT";
const char FAULT_MSG_OVER_TEMP[] PROGMEM = "OVER-TEMP FAULT";
const char FAULT_MSG_RUNAWAY[] PROGMEM = "THERMAL RUNAWAY";
const char FAULT_MSG_TIMEOUT[] PROGMEM = "HEATING TIMEOUT";
const char FAULT_MSG_OUTPUT[] PROGMEM = "OUTPUT FAULT";
const char FAULT_MSG_WDT[] PROGMEM = "WATCHDOG RESET";
const char FAULT_MSG_BROWNOUT[] PROGMEM = "POWER FAULT RESET";
const char FAULT_MSG_SELFTEST[] PROGMEM = "SELF-TEST FAIL";
} // namespace

void FaultManager::resetDetectionState_() {
  state_.sensor_invalid_start_ms = 0u;
  state_.heating_start_ms = 0u;
  state_.last_temp_sample_ms = 0u;
  state_.temp_hist_count = 0u;
  for (uint8_t i = 0u; i < 3u; i++) {
    state_.temp_hist_deci_c[i] = 0;
  }
}

void FaultManager::init() {
  state_.current_fault = FaultCode::NONE;
  state_.fault_start_ms = 0u;
  state_.temp_at_fault_c = 0.0f;
  state_.latched = 0u;
  resetDetectionState_();
}

bool FaultManager::hasFault() const {
  return state_.latched != 0u;
}

FaultCode FaultManager::getCurrentFault() const {
  return state_.current_fault;
}

uint32_t FaultManager::getFaultStartMs() const {
  return state_.fault_start_ms;
}

float FaultManager::getTempAtFault() const {
  return state_.temp_at_fault_c;
}

const __FlashStringHelper* FaultManager::getFaultMessage() const {
  const char* msg = FAULT_MSG_NONE;
  switch (state_.current_fault) {
    case FaultCode::DOOR_OPEN:
      msg = FAULT_MSG_DOOR;
      break;
    case FaultCode::TEMP_SENSOR_FAULT:
      msg = FAULT_MSG_TEMP;
      break;
    case FaultCode::OVER_TEMP:
      msg = FAULT_MSG_OVER_TEMP;
      break;
    case FaultCode::THERMAL_RUNAWAY:
      msg = FAULT_MSG_RUNAWAY;
      break;
    case FaultCode::HEATING_TIMEOUT:
      msg = FAULT_MSG_TIMEOUT;
      break;
    case FaultCode::OUTPUT_FAULT:
      msg = FAULT_MSG_OUTPUT;
      break;
    case FaultCode::WATCHDOG_RESET:
      msg = FAULT_MSG_WDT;
      break;
    case FaultCode::BROWNOUT:
      msg = FAULT_MSG_BROWNOUT;
      break;
    case FaultCode::SELF_TEST_FAIL:
      msg = FAULT_MSG_SELFTEST;
      break;
    case FaultCode::NONE:
    default:
      msg = FAULT_MSG_NONE;
      break;
  }
  return reinterpret_cast<const __FlashStringHelper*>(msg);
}

bool FaultManager::canClearFault() const {
  // Latching + operator acknowledgement are enforced by the App. This predicate
  // checks only the recovery conditions (Appendix C / Requirements).
  switch (state_.current_fault) {
    case FaultCode::NONE:
      return true;

    case FaultCode::DOOR_OPEN:
      return io.isDoorClosed();

    case FaultCode::TEMP_SENSOR_FAULT:
      return tempSensor.isValid();

    case FaultCode::OVER_TEMP:
    case FaultCode::THERMAL_RUNAWAY:
      return tempSensor.isValid() && (tempSensor.getTemperature() < kThermalClearC);

    case FaultCode::OUTPUT_FAULT:
      // Require mismatch counter to be back to 0 before allowing clear.
      return io.isDoorClosed() && (io.getOutputMismatchCount() < kOutputMismatchTrip);

    case FaultCode::HEATING_TIMEOUT:
    case FaultCode::WATCHDOG_RESET:
    case FaultCode::BROWNOUT:
      return true; // acknowledge-only

    case FaultCode::SELF_TEST_FAIL:
    default:
      return false;
  }
}

void FaultManager::setFault(FaultCode code) {
  if (code == FaultCode::NONE) {
    return;
  }
  if (state_.latched != 0u) {
    return;
  }

  state_.current_fault = code;
  state_.latched = 1u;
  state_.fault_start_ms = millis();
  state_.temp_at_fault_c = tempSensor.isValid() ? tempSensor.getTemperature() : 0.0f;

  // Requirement 18 / 10.7: persist fault history entry.
  eepromStore.logFault(static_cast<uint8_t>(code), state_.fault_start_ms / 1000u, state_.temp_at_fault_c);
}

void FaultManager::clearFault() {
  state_.current_fault = FaultCode::NONE;
  state_.latched = 0u;
  state_.fault_start_ms = 0u;
  state_.temp_at_fault_c = 0.0f;
  resetDetectionState_();
}

void FaultManager::sampleRunaway_(uint32_t now_ms, int16_t temp_deci_c, bool heater_on) {
  if (!heater_on) {
    // Only evaluate runaway while heating is actually energized.
    state_.temp_hist_count = 0u;
    return;
  }

  if (state_.last_temp_sample_ms != 0u && (now_ms - state_.last_temp_sample_ms) < kRunawaySampleMs) {
    return;
  }
  state_.last_temp_sample_ms = now_ms;

  // Shift: [0]=newest, [2]=oldest (30s window after 3 samples).
  state_.temp_hist_deci_c[2] = state_.temp_hist_deci_c[1];
  state_.temp_hist_deci_c[1] = state_.temp_hist_deci_c[0];
  state_.temp_hist_deci_c[0] = temp_deci_c;
  if (state_.temp_hist_count < 3u) {
    state_.temp_hist_count++;
    return;
  }

  const int16_t delta = static_cast<int16_t>(state_.temp_hist_deci_c[0] - state_.temp_hist_deci_c[2]);
  // Requirement 10.4: >15.0C rise over 30s while heater on.
  if (delta > 150) {
    setFault(FaultCode::THERMAL_RUNAWAY);
  }
}

void FaultManager::update() {
  const uint32_t now = millis();

  if (state_.latched != 0u) {
    return;
  }

  const SystemState st = app.getCurrentState();

  // Door open -> FAULT only during operation (Design: Safety State Prioritization).
  // Anti-crease exit is handled by the App (ANTI_CREASE -> IDLE).
  bool door_fault_active = false;
  switch (st) {
    case SystemState::START_DELAY:
    case SystemState::RUNNING_HEAT:
    case SystemState::RUNNING_COOLDOWN:
    case SystemState::PAUSED:
    case SystemState::AUTOTUNE:
      door_fault_active = true;
      break;
#if ENABLE_SERVICE_MENU
    case SystemState::SERVICE:
      // Only treat SERVICE as operational when a tool is actively driving outputs.
      door_fault_active = heaterControl.isHeaterOn() || drumControl.isRunning();
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_IO_TEST
      if (app.isIoTestActive()) {
        bool h = false;
        bool f = false;
        bool r = false;
        bool a = false;
        app.getIoTestOutputs(h, f, r, a);
        door_fault_active = door_fault_active || h || f || r || a;
      }
#endif
      break;
#endif
    default:
      door_fault_active = false;
      break;
  }

  if (door_fault_active && !io.isDoorClosed()) {
    setFault(FaultCode::DOOR_OPEN);
    return;
  }

  // Output readback mismatch (hardware integrity).
  if (io.getOutputMismatchCount() >= kOutputMismatchTrip) {
    setFault(FaultCode::OUTPUT_FAULT);
    return;
  }

  // Temperature-dependent faults require a valid sensor.
  const bool sensor_ok = tempSensor.isValid();
  if (!sensor_ok) {
    if (state_.sensor_invalid_start_ms == 0u) {
      state_.sensor_invalid_start_ms = now;
    }
    if ((now - state_.sensor_invalid_start_ms) >= kSensorFaultDelayMs) {
      setFault(FaultCode::TEMP_SENSOR_FAULT);
      return;
    }
  } else {
    state_.sensor_invalid_start_ms = 0u;
  }

  if (!sensor_ok) {
    // Cannot evaluate over-temp/runaway/timeout reliably.
    state_.heating_start_ms = 0u;
    state_.temp_hist_count = 0u;
    return;
  }

  const float pv_c = tempSensor.getTemperature();
  if (pv_c >= static_cast<float>(OVER_TEMP_THRESHOLD)) {
    setFault(FaultCode::OVER_TEMP);
    return;
  }

  if (st == SystemState::RUNNING_HEAT) {
    // Heating timeout: 90 minutes without reaching setpoint.
    if (state_.heating_start_ms == 0u) {
      state_.heating_start_ms = now;
    }

    const float sp_c = pidController.getTargetSetpoint();
    if (sp_c > 0.1f && pv_c >= (sp_c - 1.0f)) {
      // Consider setpoint reached (±1C deadband).
      state_.heating_start_ms = 0u;
    } else if (state_.heating_start_ms != 0u && (now - state_.heating_start_ms) >= kHeatTimeoutMs) {
      setFault(FaultCode::HEATING_TIMEOUT);
      return;
    }

    // Thermal runaway (rate-of-rise).
    sampleRunaway_(now, toDeciC(pv_c), heaterControl.isHeaterOn());
    return;
  }

  // Reset heating-related timers when not in RUNNING_HEAT.
  state_.heating_start_ms = 0u;
  state_.temp_hist_count = 0u;
  state_.last_temp_sample_ms = 0u;
}
