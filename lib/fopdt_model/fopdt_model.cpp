#include "fopdt_model.h"

#include <Arduino.h>
#include <pgmspace.h>

namespace {
constexpr uint8_t kBaselineDurationS = 30u;
constexpr uint16_t kMeasureDurationS = 600u; // 10 minutes for near steady-state (design guidance)

constexpr int16_t kNoSample = static_cast<int16_t>(-32768);
constexpr int16_t kFirstResponseThresholdDeciC = 5; // 0.5°C

constexpr uint16_t kUnsetU16 = 0xFFFFu;

// Sparse sampling schedule (seconds since step start). Stored in Flash.
constexpr uint8_t kScheduleCount = 16u;
const uint16_t kSampleScheduleS[kScheduleCount] PROGMEM = {
    5u,  10u,  15u,  20u,  30u,  45u,  60u,  90u,
    120u, 150u, 180u, 240u, 300u, 360u, 420u, 600u,
};

int16_t toDeciC(float temp_c) {
  // Convert float °C to deci-°C with rounding. Avoid libm.
  const float scaled = (temp_c * 10.0f) + ((temp_c >= 0.0f) ? 0.5f : -0.5f);
  if (scaled > 32767.0f) {
    return 32767;
  }
  if (scaled < -32768.0f) {
    return static_cast<int16_t>(-32768);
  }
  return static_cast<int16_t>(scaled);
}

} // namespace

void FOPDTModel::init() {
  state_.state = State::IDLE;
  state_.identifying = 0u;
  state_.result_valid = 0u;

  state_.step_size = 0.0f;

  state_.baseline_sum_deci = 0;
  state_.baseline_samples = 0u;
  state_.baseline_temp_deci = 0;

  state_.state_start_ms = 0u;
  state_.step_start_ms = 0u;

  state_.peak_temp_deci = 0;
  state_.delay_s = kUnsetU16;
  state_.tau_s = kUnsetU16;

  state_.next_sample_index = 0u;
  for (uint8_t i = 0u; i < kSampleCount; i++) {
    state_.sample_temp_deci[i] = kNoSample;
  }

  state_.result_k = 0.0f;
  state_.result_tau_s = 0.0f;
  state_.result_l_s = 0.0f;
}

void FOPDTModel::startIdentification(float step_size) {
  // Clamp to sane bounds to avoid divide-by-zero and unsafe full-scale.
  if (step_size < 0.01f) {
    step_size = 0.01f;
  } else if (step_size > 1.0f) {
    step_size = 1.0f;
  }

  const uint32_t now = millis();

  state_.state = State::BASELINE;
  state_.identifying = 1u;
  state_.result_valid = 0u;

  state_.step_size = step_size;

  state_.baseline_sum_deci = 0;
  state_.baseline_samples = 0u;
  state_.baseline_temp_deci = 0;

  state_.state_start_ms = now;
  state_.step_start_ms = 0u;

  state_.peak_temp_deci = 0;
  state_.delay_s = kUnsetU16;
  state_.tau_s = kUnsetU16;

  state_.next_sample_index = 0u;
  for (uint8_t i = 0u; i < kSampleCount; i++) {
    state_.sample_temp_deci[i] = kNoSample;
  }

  state_.result_k = 0.0f;
  state_.result_tau_s = 0.0f;
  state_.result_l_s = 0.0f;
}

void FOPDTModel::abort() {
  init();
}

bool FOPDTModel::isIdentifying() const {
  return state_.identifying != 0u;
}

FOPDTModel::State FOPDTModel::getState() const {
  return state_.state;
}

uint8_t FOPDTModel::getStepDutyPercent() const {
  float duty_f = state_.step_size * 100.0f;
  if (duty_f <= 0.0f) {
    return 0u;
  }
  if (duty_f >= 100.0f) {
    return 100u;
  }
  return static_cast<uint8_t>(duty_f + 0.5f);
}

uint16_t FOPDTModel::getBaselineSecondsRemaining() const {
  if (state_.state != State::BASELINE) {
    return 0u;
  }
  const uint32_t elapsed_s = (millis() - state_.state_start_ms) / 1000u;
  if (elapsed_s >= kBaselineDurationS) {
    return 0u;
  }
  return static_cast<uint16_t>(kBaselineDurationS - elapsed_s);
}

uint16_t FOPDTModel::getMeasureSecondsElapsed() const {
  if (state_.step_start_ms == 0u) {
    return 0u;
  }
  return static_cast<uint16_t>((millis() - state_.step_start_ms) / 1000u);
}

void FOPDTModel::updateIdentification(float temp_c, float output_percent) {
  (void)output_percent; // reserved for future use (e.g., use measured duty for K)

  if (state_.identifying == 0u) {
    return;
  }

  const uint32_t now = millis();
  const int16_t temp_deci = toDeciC(temp_c);

  switch (state_.state) {
    case State::BASELINE: {
      state_.baseline_sum_deci += static_cast<int32_t>(temp_deci);
      if (state_.baseline_samples != 255u) {
        state_.baseline_samples++;
      }

      if ((now - state_.state_start_ms) < (static_cast<uint32_t>(kBaselineDurationS) * 1000u)) {
        return;
      }

      if (state_.baseline_samples == 0u) {
        // Unexpected; keep identifying but restart baseline window.
        state_.state_start_ms = now;
        return;
      }

      state_.baseline_temp_deci = static_cast<int16_t>(state_.baseline_sum_deci / state_.baseline_samples);

      // Step begins now (STATE::STEP exists for UI clarity).
      state_.state = State::STEP;
      state_.state_start_ms = now;
      state_.step_start_ms = now;

      state_.peak_temp_deci = state_.baseline_temp_deci;
      state_.delay_s = kUnsetU16;
      state_.tau_s = kUnsetU16;

      state_.next_sample_index = 0u;
      for (uint8_t i = 0u; i < kSampleCount; i++) {
        state_.sample_temp_deci[i] = kNoSample;
      }
      return;
    }

    case State::STEP: {
      // After ~1s, transition to measurement.
      if ((now - state_.state_start_ms) >= 1000u) {
        state_.state = State::MEASURE;
        state_.state_start_ms = now;
      }
      // Continue tracking peak/delay even during this brief state.
      const uint16_t elapsed_s = static_cast<uint16_t>((now - state_.step_start_ms) / 1000u);
      if (temp_deci > state_.peak_temp_deci) {
        state_.peak_temp_deci = temp_deci;
      }
      if (state_.delay_s == kUnsetU16 && elapsed_s > 0u &&
          temp_deci >= static_cast<int16_t>(state_.baseline_temp_deci + kFirstResponseThresholdDeciC)) {
        state_.delay_s = elapsed_s;
      }
      return;
    }

    case State::MEASURE: {
      const uint16_t elapsed_s = static_cast<uint16_t>((now - state_.step_start_ms) / 1000u);

      if (temp_deci > state_.peak_temp_deci) {
        state_.peak_temp_deci = temp_deci;
      }

      if (state_.delay_s == kUnsetU16 && elapsed_s > 0u &&
          temp_deci >= static_cast<int16_t>(state_.baseline_temp_deci + kFirstResponseThresholdDeciC)) {
        state_.delay_s = elapsed_s;
      }

      while (state_.next_sample_index < kSampleCount) {
        const uint16_t t_s = pgm_read_word(&kSampleScheduleS[state_.next_sample_index]);
        if (elapsed_s < t_s) {
          break;
        }
        state_.sample_temp_deci[state_.next_sample_index] = temp_deci;
        state_.next_sample_index++;
      }

      const bool done_by_samples = (state_.next_sample_index >= kSampleCount);
      const bool done_by_time = (elapsed_s >= kMeasureDurationS);

      if (!done_by_samples && !done_by_time) {
        return;
      }

      computeResults_();
      state_.state = State::COMPLETE;
      state_.identifying = 0u;
      return;
    }

    case State::COMPLETE:
    case State::IDLE:
    default:
      return;
  }
}

bool FOPDTModel::getResults(float& k, float& tau_s, float& l_s) const {
  if (state_.result_valid == 0u) {
    return false;
  }
  k = state_.result_k;
  tau_s = state_.result_tau_s;
  l_s = state_.result_l_s;
  return true;
}

void FOPDTModel::computePIDGains(float k, float tau_s, float l_s, float& kp, float& ki, float& kd) const {
  kp = 0.0f;
  ki = 0.0f;
  kd = 0.0f;

  if (k <= 0.0f || tau_s <= 0.0f) {
    return;
  }
  if (l_s < 0.0f) {
    l_s = 0.0f;
  }

  // IMC PI tuning (Design: lambda = tau). K is °C / (fraction of full output).
  // Convert to percent output by multiplying Kp and Ki by 100.
  const float lambda = tau_s;
  const float denom = k * (lambda + l_s);
  if (denom <= 0.0f) {
    return;
  }

  float kp_frac = tau_s / denom; // output fraction per °C
  float kp_out = kp_frac * 100.0f;

  // Spec lists Ki = 1/tau (i.e. 1/Ti). Our PID implementation expects Ki = Kp/Ti.
  float ki_out = (tau_s > 0.0f) ? (kp_out / tau_s) : 0.0f;

  // Sanity clamp and fallback to Cohen-Coon PI if IMC yields extreme results.
  const bool imc_ok = (kp_out == kp_out) && (ki_out == ki_out) && (kp_out >= 0.0f) && (ki_out >= 0.0f) &&
                      (kp_out <= 500.0f) && (ki_out <= 50.0f);

  if (!imc_ok && l_s > 1.0f) {
    const float r = l_s / tau_s; // L/tau
    if (r > 0.0f) {
      const float kc_frac = ((tau_s / l_s) * (0.9f + (r / 12.0f))) / k; // fraction per °C
      const float ti_s = l_s * ((30.0f + (3.0f * r)) / (9.0f + (20.0f * r)));
      if (ti_s > 0.0f && kc_frac > 0.0f) {
        kp_out = kc_frac * 100.0f;
        ki_out = kp_out / ti_s;
      }
    }
  }

  // Final clamps (never allow negative).
  if (kp_out < 0.0f) {
    kp_out = 0.0f;
  }
  if (ki_out < 0.0f) {
    ki_out = 0.0f;
  }

  kp = kp_out;
  ki = ki_out;
  kd = 0.0f;
}

void FOPDTModel::computeResults_() {
  state_.result_valid = 0u;

  const int16_t baseline = state_.baseline_temp_deci;
  const int16_t peak = state_.peak_temp_deci;
  const int16_t delta_deci = static_cast<int16_t>(peak - baseline);
  if (delta_deci <= 0) {
    return;
  }

  // Process gain: K = ΔT / Δu, where Δu is the commanded step (fraction).
  const float delta_c = static_cast<float>(delta_deci) * 0.1f;
  const float step = (state_.step_size <= 0.0f) ? 0.01f : state_.step_size;
  const float k = delta_c / step;

  // Dead time L: seconds from step to first detectable response.
  const uint16_t l_s_u16 = (state_.delay_s == kUnsetU16) ? 0u : state_.delay_s;

  // Find t63 (seconds since step start) from sparse samples.
  const int16_t threshold_deci =
      static_cast<int16_t>(baseline + static_cast<int16_t>((static_cast<int32_t>(delta_deci) * 632 + 500) / 1000));

  uint16_t t63_s = kMeasureDurationS;
  int16_t y_prev = baseline;
  uint16_t t_prev = 0u;

  for (uint8_t i = 0u; i < kSampleCount; i++) {
    const int16_t y = state_.sample_temp_deci[i];
    if (y == kNoSample) {
      break;
    }
    const uint16_t t = pgm_read_word(&kSampleScheduleS[i]);

    if (y >= threshold_deci) {
      if (i == 0u) {
        t63_s = t;
      } else if (y == y_prev) {
        t63_s = t;
      } else {
        // Linear interpolation between (t_prev, y_prev) and (t, y).
        const int16_t dy = static_cast<int16_t>(y - y_prev);
        const int16_t dth = static_cast<int16_t>(threshold_deci - y_prev);
        const uint16_t dt_s = static_cast<uint16_t>(t - t_prev);

        // Guard against unexpected sign issues.
        if (dy > 0 && dth >= 0) {
          const uint32_t num = static_cast<uint32_t>(static_cast<uint16_t>(dth)) * static_cast<uint32_t>(dt_s);
          const uint16_t frac_dt = static_cast<uint16_t>(num / static_cast<uint16_t>(dy));
          t63_s = static_cast<uint16_t>(t_prev + frac_dt);
        } else {
          t63_s = t;
        }
      }
      break;
    }

    y_prev = y;
    t_prev = t;
  }

  // Time constant tau (seconds): tau = t63 - L.
  uint16_t tau_s_u16 = 1u;
  if (t63_s > l_s_u16) {
    tau_s_u16 = static_cast<uint16_t>(t63_s - l_s_u16);
    if (tau_s_u16 == 0u) {
      tau_s_u16 = 1u;
    }
  }

  state_.tau_s = tau_s_u16;
  state_.result_k = k;
  state_.result_tau_s = static_cast<float>(tau_s_u16);
  state_.result_l_s = static_cast<float>(l_s_u16);
  state_.result_valid = 1u;
}
