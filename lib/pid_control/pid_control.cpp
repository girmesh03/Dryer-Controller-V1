#include "pid_control.h"

#include <Arduino.h>
#include <pgmspace.h>

#include "config_build.h"

namespace {
constexpr uint16_t kComputePeriodMs = 100u; // Requirement 5: 10 Hz
constexpr float kRampRateCPerSec = 2.0f;   // Requirement 5: max 2°C/s

const float kDefaultPidGains[3] PROGMEM = {10.0f, 0.5f, 2.0f}; // Kp, Ki, Kd

float absf(float x) {
  return (x < 0.0f) ? -x : x;
}

#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE
constexpr uint8_t kAutoTuneRelayHighPercent = 50u;
constexpr float kAutoTuneBaselineOffsetC = 5.0f;
constexpr float kAutoTuneBandC = 2.0f;
constexpr uint8_t kAutoTuneRequiredCycles = 3u;
constexpr uint32_t kAutoTuneMaxDurationMs = 30ul * 60ul * 1000ul; // Design: 30 minutes

// Avoid pulling in libm for M_PI.
constexpr float kFourOverPi = 1.2732396f; // 4/pi

enum AutoTuneAbort : uint8_t {
  AUTOTUNE_ABORT_NONE = 0u,
  AUTOTUNE_ABORT_DOOR = 1u,
  AUTOTUNE_ABORT_SENSOR = 2u,
  AUTOTUNE_ABORT_OVERTEMP = 3u,
  AUTOTUNE_ABORT_STOP = 4u,
  AUTOTUNE_ABORT_TIMEOUT = 5u,
  AUTOTUNE_ABORT_FAULT = 6u,
  AUTOTUNE_ABORT_AMBIENT = 7u,
};

enum AutoTunePhase : uint8_t {
  AUTOTUNE_PHASE_HEAT = 0u,
  AUTOTUNE_PHASE_COOL_REQ = 1u,
  AUTOTUNE_PHASE_COOL = 2u,
  AUTOTUNE_PHASE_HEAT_REQ = 3u,
};

int16_t toDeciC(float temp_c) {
  const float scaled = (temp_c * 10.0f) + ((temp_c >= 0.0f) ? 0.5f : -0.5f);
  if (scaled > 32767.0f) {
    return 32767;
  }
  if (scaled < -32768.0f) {
    return static_cast<int16_t>(-32768);
  }
  return static_cast<int16_t>(scaled);
}
#endif
} // namespace

void PIDControl::init() {
  state_.target_setpoint = 0.0f;
  state_.current_setpoint = 0.0f;

  state_.integral = 0.0f;
  state_.last_input = 0.0f;

  state_.output_min = 0.0f;
  state_.output_max = 100.0f;
  state_.last_output = 0.0f;

  state_.last_compute_ms = 0u;
  state_.pending_reset = 1u;
  state_.have_last_input = 0u;

  const float kp = pgm_read_float(&kDefaultPidGains[0]);
  const float ki = pgm_read_float(&kDefaultPidGains[1]);
  const float kd = pgm_read_float(&kDefaultPidGains[2]);
  setTunings(kp, ki, kd);

#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE
  state_.autotune.active = 0u;
  state_.autotune.complete = 0u;
  state_.autotune.phase = AUTOTUNE_PHASE_HEAT;
  state_.autotune.cmd_duty = 0u;
  state_.autotune.relay_high = kAutoTuneRelayHighPercent;
  state_.autotune.cycles = 0u;
  state_.autotune.abort_reason = AUTOTUNE_ABORT_NONE;
  state_.autotune.last_heater_on = 0u;
  state_.autotune.relay_edge_count = 0u;

  state_.autotune.baseline_deci_c = 0;
  state_.autotune.peak_high_deci_c = 0;
  state_.autotune.peak_low_deci_c = 0;

  state_.autotune.sum_high_deci_c = 0;
  state_.autotune.sum_low_deci_c = 0;
  state_.autotune.high_count = 0u;
  state_.autotune.low_count = 0u;

  state_.autotune.start_ms = 0u;
  state_.autotune.last_high_ms = 0u;
  state_.autotune.sum_tu_ms = 0u;
  state_.autotune.tu_count = 0u;

  state_.autotune.ku = 0.0f;
  state_.autotune.tu_s = 0.0f;
  state_.autotune.tuned_kp = 0.0f;
  state_.autotune.tuned_ki = 0.0f;
  state_.autotune.tuned_kd = 0.0f;

  state_.autotune.prev_kp = state_.kp;
  state_.autotune.prev_ki = state_.ki;
  state_.autotune.prev_kd = state_.kd;
#endif
}

void PIDControl::setTunings(float kp, float ki, float kd) {
  // Clamp to sane non-negative values (safety; avoids inverted control).
  state_.kp = (kp < 0.0f) ? 0.0f : kp;
  state_.ki = (ki < 0.0f) ? 0.0f : ki;
  state_.kd = (kd < 0.0f) ? 0.0f : kd;
}

void PIDControl::setSetpoint(float sp_c) {
  state_.target_setpoint = sp_c;
  // Do not snap current_setpoint here; ramping is enforced in compute().
}

void PIDControl::setOutputLimits(float min_out, float max_out) {
  if (max_out < min_out) {
    const float tmp = min_out;
    min_out = max_out;
    max_out = tmp;
  }

  state_.output_min = min_out;
  state_.output_max = max_out;

  state_.integral = constrainOutput_(state_.integral);
  state_.last_output = constrainOutput_(state_.last_output);
}

float PIDControl::constrainOutput_(float out) const {
  if (out > state_.output_max) {
    return state_.output_max;
  }
  if (out < state_.output_min) {
    return state_.output_min;
  }
  return out;
}

void PIDControl::reset() {
  // Bumpless transfer is applied at the next compute() call so the current
  // input is available without coupling PIDControl to the sensor module.
  state_.pending_reset = 1u;
}

void PIDControl::rampSetpoint_(float dt_s) {
  const float target = state_.target_setpoint;
  float current = state_.current_setpoint;

  const float max_step = kRampRateCPerSec * dt_s;
  const float delta = target - current;

  if (absf(delta) <= max_step) {
    state_.current_setpoint = target;
    return;
  }

  if (delta > 0.0f) {
    current += max_step;
  } else {
    current -= max_step;
  }
  state_.current_setpoint = current;
}

float PIDControl::compute(float input_c) {
  const uint32_t now = millis();

  if (state_.last_compute_ms != 0u && (now - state_.last_compute_ms) < kComputePeriodMs) {
    return state_.last_output;
  }

  uint32_t dt_ms = (state_.last_compute_ms == 0u) ? kComputePeriodMs : (now - state_.last_compute_ms);
  if (dt_ms == 0u) {
    dt_ms = kComputePeriodMs;
  }
  // Guard against very large dt (e.g., state changes) to keep integral stable.
  if (dt_ms > 1000u) {
    dt_ms = 1000u;
  }

  const float dt_s = static_cast<float>(dt_ms) * 0.001f;
  state_.last_compute_ms = now;

  if (state_.pending_reset != 0u) {
    state_.pending_reset = 0u;
    state_.have_last_input = 1u;
    state_.last_input = input_c;

    // Start ramping from current PV for a smooth heat-up.
    state_.current_setpoint = input_c;

    // Keep output continuous: with error ~0 and dInput ~0, integral holds output.
    state_.integral = constrainOutput_(state_.last_output);
    return state_.last_output;
  }

  rampSetpoint_(dt_s);

  const float sp = state_.current_setpoint;
  const float error = sp - input_c;

  const float p = state_.kp * error;

  // Derivative on measurement (prevents setpoint kick): -Kd * dInput/dt
  float d = 0.0f;
  if (state_.have_last_input != 0u) {
    const float d_input = input_c - state_.last_input;
    const float inv_dt = 1000.0f / static_cast<float>(dt_ms);
    d = -state_.kd * d_input * inv_dt;
  }

  // Integrate.
  const float i_prev = state_.integral;
  float i = i_prev + (state_.ki * error * dt_s);

  // Compute unclamped output with updated I.
  float out = p + i + d;
  const float out_clamped = constrainOutput_(out);

  // Anti-windup: if saturated and error pushes further into saturation, undo integration.
  if (out_clamped != out) {
    if ((out_clamped >= state_.output_max && error > 0.0f) ||
        (out_clamped <= state_.output_min && error < 0.0f)) {
      i = i_prev;
      out = p + i + d;
    }
  }

  // Clamp integral to output range to prevent accumulation beyond actuator capability.
  state_.integral = constrainOutput_(i);

  state_.last_input = input_c;
  state_.have_last_input = 1u;

  state_.last_output = constrainOutput_(out);
  return state_.last_output;
}

void PIDControl::getTunings(float& kp, float& ki, float& kd) const {
  kp = state_.kp;
  ki = state_.ki;
  kd = state_.kd;
}

float PIDControl::getTargetSetpoint() const {
  return state_.target_setpoint;
}

float PIDControl::getCurrentSetpoint() const {
  return state_.current_setpoint;
}

float PIDControl::getLastOutput() const {
  return state_.last_output;
}

#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE
#include "ds18b20_sensor.h"
#include "faults.h"
#include "heater_control.h"
#include "io_abstraction.h"

extern IOAbstraction io;
extern DS18B20Sensor tempSensor;
extern HeaterControl heaterControl;
extern FaultManager faultMgr;

void PIDControl::startAutoTune() {
  if (state_.autotune.active != 0u) {
    return;
  }

  // Preconditions (Requirement 6).
  if (!io.isDoorClosed()) {
    state_.autotune.abort_reason = AUTOTUNE_ABORT_DOOR;
    return;
  }
  if (!tempSensor.isValid()) {
    state_.autotune.abort_reason = AUTOTUNE_ABORT_SENSOR;
    return;
  }
  if (faultMgr.hasFault()) {
    state_.autotune.abort_reason = AUTOTUNE_ABORT_FAULT;
    return;
  }

  const float temp_c = tempSensor.getTemperature();
  if (temp_c < 15.0f || temp_c > 30.0f) {
    state_.autotune.abort_reason = AUTOTUNE_ABORT_AMBIENT;
    return;
  }

  // Save previous gains for restoration on abort/reject.
  state_.autotune.prev_kp = state_.kp;
  state_.autotune.prev_ki = state_.ki;
  state_.autotune.prev_kd = state_.kd;

  const uint32_t now = millis();

  // Target center for relay oscillation: a few degrees above ambient so cooling
  // can cross the low threshold without requiring sub-ambient temperatures.
  const float baseline_c = temp_c + kAutoTuneBaselineOffsetC;
  state_.autotune.baseline_deci_c = toDeciC(baseline_c);

  state_.autotune.active = 1u;
  state_.autotune.complete = 0u;
  state_.autotune.phase = AUTOTUNE_PHASE_HEAT;
  state_.autotune.cmd_duty = state_.autotune.relay_high;
  state_.autotune.cycles = 0u;
  state_.autotune.abort_reason = AUTOTUNE_ABORT_NONE;
  state_.autotune.last_heater_on = heaterControl.isHeaterOn() ? 1u : 0u;
  state_.autotune.relay_edge_count = 0u;

  // Peak tracking.
  const int16_t temp_deci = toDeciC(temp_c);
  state_.autotune.peak_high_deci_c = temp_deci;
  state_.autotune.peak_low_deci_c = temp_deci;
  state_.autotune.sum_high_deci_c = 0;
  state_.autotune.sum_low_deci_c = 0;
  state_.autotune.high_count = 0u;
  state_.autotune.low_count = 0u;

  // Period measurement (Tu) from successive high peaks.
  state_.autotune.start_ms = now;
  state_.autotune.last_high_ms = 0u;
  state_.autotune.sum_tu_ms = 0u;
  state_.autotune.tu_count = 0u;

  state_.autotune.ku = 0.0f;
  state_.autotune.tu_s = 0.0f;
  state_.autotune.tuned_kp = 0.0f;
  state_.autotune.tuned_ki = 0.0f;
  state_.autotune.tuned_kd = 0.0f;

  // Begin with the relay "high" command. HeaterControl enforces min on/off.
  heaterControl.setDutyCycle(static_cast<float>(state_.autotune.cmd_duty));
  heaterControl.enable();
}

void PIDControl::abortAutoTune() {
  // Restore previous gains (Requirement 6).
  setTunings(state_.autotune.prev_kp, state_.autotune.prev_ki, state_.autotune.prev_kd);

  state_.autotune.active = 0u;
  state_.autotune.complete = 0u;
  state_.autotune.cmd_duty = 0u;

  heaterControl.setDutyCycle(0.0f);
  heaterControl.disable();
}

void PIDControl::endAutoTuneSession() {
  state_.autotune.active = 0u;
  state_.autotune.complete = 0u;
  state_.autotune.cmd_duty = 0u;
  state_.autotune.abort_reason = AUTOTUNE_ABORT_NONE;
  heaterControl.setDutyCycle(0.0f);
  heaterControl.disable();
}

bool PIDControl::isAutoTuning() const {
  return (state_.autotune.active != 0u) && (state_.autotune.complete == 0u);
}

bool PIDControl::isAutoTuneComplete() const {
  return state_.autotune.complete != 0u;
}

uint8_t PIDControl::getAutoTuneCycleCount() const {
  return state_.autotune.cycles;
}

uint8_t PIDControl::getAutoTuneCommandDuty() const {
  return state_.autotune.cmd_duty;
}

uint8_t PIDControl::getAutoTuneAbortReason() const {
  return state_.autotune.abort_reason;
}

bool PIDControl::getAutoTuneKuTu(float& ku, float& tu_s) const {
  if (state_.autotune.complete == 0u) {
    return false;
  }
  ku = state_.autotune.ku;
  tu_s = state_.autotune.tu_s;
  return true;
}

bool PIDControl::getAutoTuneResults(float& kp, float& ki, float& kd) const {
  if (state_.autotune.complete == 0u) {
    return false;
  }
  kp = state_.autotune.tuned_kp;
  ki = state_.autotune.tuned_ki;
  kd = state_.autotune.tuned_kd;
  return true;
}

void PIDControl::updateAutoTune(float input_c) {
  if (state_.autotune.active == 0u || state_.autotune.complete != 0u) {
    return;
  }

  const uint32_t now = millis();
  if ((now - state_.autotune.start_ms) > kAutoTuneMaxDurationMs) {
    state_.autotune.abort_reason = AUTOTUNE_ABORT_TIMEOUT;
    abortAutoTune();
    return;
  }

  // Continuous safety checks.
  if (!io.isDoorClosed()) {
    state_.autotune.abort_reason = AUTOTUNE_ABORT_DOOR;
    abortAutoTune();
    return;
  }
  if (!tempSensor.isValid()) {
    state_.autotune.abort_reason = AUTOTUNE_ABORT_SENSOR;
    abortAutoTune();
    return;
  }
  if (faultMgr.hasFault()) {
    state_.autotune.abort_reason = AUTOTUNE_ABORT_FAULT;
    abortAutoTune();
    return;
  }
  if (input_c >= static_cast<float>(OVER_TEMP_THRESHOLD)) {
    state_.autotune.abort_reason = AUTOTUNE_ABORT_OVERTEMP;
    abortAutoTune();
    return;
  }

  const int16_t input_deci = toDeciC(input_c);
  const int16_t baseline = state_.autotune.baseline_deci_c;
  const int16_t band = static_cast<int16_t>(kAutoTuneBandC * 10.0f + 0.5f); // 2.0C -> 20

  // Track actual relay transitions (Requirement 6/Design: respect anti-chatter).
  const uint8_t heater_on = heaterControl.isHeaterOn() ? 1u : 0u;
  if (heater_on != state_.autotune.last_heater_on) {
    state_.autotune.last_heater_on = heater_on;
    if (state_.autotune.relay_edge_count != 255u) {
      state_.autotune.relay_edge_count++;
    }
  }

  switch (state_.autotune.phase) {
    case AUTOTUNE_PHASE_HEAT: {
      heaterControl.setDutyCycle(static_cast<float>(state_.autotune.relay_high));
      state_.autotune.cmd_duty = state_.autotune.relay_high;

      if (input_deci > state_.autotune.peak_high_deci_c) {
        state_.autotune.peak_high_deci_c = input_deci;
      }

      if (input_deci >= static_cast<int16_t>(baseline + band)) {
        state_.autotune.phase = AUTOTUNE_PHASE_COOL_REQ;
        heaterControl.setDutyCycle(0.0f);
        state_.autotune.cmd_duty = 0u;
      }
      break;
    }

    case AUTOTUNE_PHASE_COOL_REQ: {
      // Command cooling and wait for the relay to actually turn OFF.
      heaterControl.setDutyCycle(0.0f);
      state_.autotune.cmd_duty = 0u;

      if (input_deci > state_.autotune.peak_high_deci_c) {
        state_.autotune.peak_high_deci_c = input_deci;
      }

      if (heater_on == 0u) {
        // High peak event at the moment the relay is confirmed OFF.
        state_.autotune.sum_high_deci_c += state_.autotune.peak_high_deci_c;
        if (state_.autotune.high_count != 255u) {
          state_.autotune.high_count++;
        }

        if (state_.autotune.last_high_ms != 0u) {
          const uint32_t tu_ms = now - state_.autotune.last_high_ms;
          state_.autotune.sum_tu_ms += tu_ms;
          if (state_.autotune.tu_count != 255u) {
            state_.autotune.tu_count++;
          }
          state_.autotune.cycles = state_.autotune.tu_count;
        }
        state_.autotune.last_high_ms = now;

        state_.autotune.phase = AUTOTUNE_PHASE_COOL;
        state_.autotune.peak_low_deci_c = input_deci;
      }
      break;
    }

    case AUTOTUNE_PHASE_COOL: {
      heaterControl.setDutyCycle(0.0f);
      state_.autotune.cmd_duty = 0u;

      if (input_deci < state_.autotune.peak_low_deci_c) {
        state_.autotune.peak_low_deci_c = input_deci;
      }

      if (input_deci <= static_cast<int16_t>(baseline - band)) {
        state_.autotune.phase = AUTOTUNE_PHASE_HEAT_REQ;
        heaterControl.setDutyCycle(static_cast<float>(state_.autotune.relay_high));
        state_.autotune.cmd_duty = state_.autotune.relay_high;
      }
      break;
    }

    case AUTOTUNE_PHASE_HEAT_REQ: {
      // Command heating and wait for the relay to actually turn ON.
      heaterControl.setDutyCycle(static_cast<float>(state_.autotune.relay_high));
      state_.autotune.cmd_duty = state_.autotune.relay_high;

      if (input_deci < state_.autotune.peak_low_deci_c) {
        state_.autotune.peak_low_deci_c = input_deci;
      }

      if (heater_on != 0u) {
        // Low peak event at the moment the relay is confirmed ON.
        state_.autotune.sum_low_deci_c += state_.autotune.peak_low_deci_c;
        if (state_.autotune.low_count != 255u) {
          state_.autotune.low_count++;
        }

        state_.autotune.phase = AUTOTUNE_PHASE_HEAT;
        state_.autotune.peak_high_deci_c = input_deci;
      }
      break;
    }

    default:
      state_.autotune.phase = AUTOTUNE_PHASE_HEAT;
      break;
  }

  // Compute results once we have enough complete cycles and both peak types.
  if (state_.autotune.tu_count < kAutoTuneRequiredCycles) {
    return;
  }
  if (state_.autotune.high_count == 0u || state_.autotune.low_count == 0u) {
    return;
  }

  const int16_t avg_high = static_cast<int16_t>(state_.autotune.sum_high_deci_c / state_.autotune.high_count);
  const int16_t avg_low = static_cast<int16_t>(state_.autotune.sum_low_deci_c / state_.autotune.low_count);
  const int16_t diff = static_cast<int16_t>(avg_high - avg_low);
  if (diff <= 0) {
    return;
  }

  // Oscillation amplitude (°C): (high - low)/2.
  const float amp_c = static_cast<float>(diff) * 0.05f;
  if (amp_c <= 0.01f) {
    return;
  }

  // Ultimate period Tu (s): average of measured peak-to-peak intervals.
  const float tu_s = (static_cast<float>(state_.autotune.sum_tu_ms) / static_cast<float>(state_.autotune.tu_count)) *
                     0.001f;
  if (tu_s <= 0.1f) {
    return;
  }

  // Ultimate gain Ku using relay amplitude (half of output step).
  const float relay_amp = static_cast<float>(state_.autotune.relay_high) * 0.5f;
  const float ku = kFourOverPi * (relay_amp / amp_c);

  // Ziegler–Nichols PID.
  float kp = 0.6f * ku;
  float ki = (1.2f * ku) / tu_s;
  float kd = 0.075f * ku * tu_s;

  // Sanity clamps (avoid extreme values).
  if (kp < 0.0f) kp = 0.0f;
  if (ki < 0.0f) ki = 0.0f;
  if (kd < 0.0f) kd = 0.0f;
  if (kp > 500.0f) kp = 500.0f;
  if (ki > 50.0f) ki = 50.0f;
  if (kd > 5000.0f) kd = 5000.0f;

  state_.autotune.ku = ku;
  state_.autotune.tu_s = tu_s;
  state_.autotune.tuned_kp = kp;
  state_.autotune.tuned_ki = ki;
  state_.autotune.tuned_kd = kd;
  state_.autotune.complete = 1u;

  // Stop driving the heater; wait for operator accept/reject.
  heaterControl.setDutyCycle(0.0f);
  heaterControl.disable();
  state_.autotune.cmd_duty = 0u;
}
#endif // ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE
