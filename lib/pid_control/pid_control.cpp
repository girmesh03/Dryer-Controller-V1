#include "pid_control.h"

#include <Arduino.h>
#include <avr/pgmspace.h>

namespace {
constexpr uint16_t kComputePeriodMs = 100u; // Requirement 5: 10 Hz
constexpr float kRampRateCPerSec = 2.0f;   // Requirement 5: max 2°C/s

const float kDefaultPidGains[3] PROGMEM = {10.0f, 0.5f, 2.0f}; // Kp, Ki, Kd

float absf(float x) {
  return (x < 0.0f) ? -x : x;
}
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

