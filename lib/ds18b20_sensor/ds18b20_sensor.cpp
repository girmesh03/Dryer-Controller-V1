#include "ds18b20_sensor.h"

#include <Arduino.h>

#include "config_pins.h"

namespace {
constexpr uint8_t kWindowSize = 5u;
constexpr float kEmaAlpha = 0.2f;
constexpr float kMaxRateCPerSec = 5.0f; // Requirement 2
constexpr uint32_t kDisconnectTimeoutMs = 2000u;
constexpr uint8_t kFaultTripCount = 3u;
constexpr uint16_t kMaxConversionWaitMs = 750u; // DS18B20 @12-bit

float absf(float x) {
  return (x < 0.0f) ? -x : x;
}
} // namespace

void DS18B20Sensor::init() {
  const uint32_t now = millis();
  one_wire_.begin(PIN_TEMP_SENSOR);
  sensors_.setOneWire(&one_wire_);
  sensors_.begin();

  // Non-blocking conversions (Requirement 33).
  sensors_.setWaitForConversion(false);
  sensors_.setCheckForConversion(true);

  // Meet Requirement 2 (>= 2 Hz sampling) by avoiding 12-bit (750ms) conversions.
  // 11-bit conversion time is ~375ms, which yields >=2 Hz effective updates with a 250ms scheduler tick.
  sensors_.setResolution(11);

  conversion_wait_ms_ = sensors_.millisToWaitForConversion();
  if (conversion_wait_ms_ == 0u || conversion_wait_ms_ > kMaxConversionWaitMs) {
    conversion_wait_ms_ = kMaxConversionWaitMs;
  }

  state_.state = State::IDLE;
  state_.state_start_ms = now;
  state_.reading_index = 0u;
  state_.sample_count = 0u;
  state_.filtered_temp = 0.0f;
  state_.ema_temp = 0.0f;
  state_.last_valid_temp = 0.0f;
  state_.last_valid_ms = now;
  state_.fault_counter = 0u;
  // Start with "no fault" but invalid until we obtain at least one plausible sample.
  // This avoids false positives during the first few seconds while the filter window fills.
  state_.fault_code = 0u;
  state_.valid = 0u;

  for (uint8_t i = 0u; i < kWindowSize; i++) {
    state_.raw_readings[i] = 0.0f;
  }
}

float DS18B20Sensor::getTemperature() const {
  return state_.filtered_temp;
}

float DS18B20Sensor::getRawTemperature() const {
  if (state_.sample_count == 0u) {
    return 0.0f;
  }
  const uint8_t last_index = (state_.reading_index == 0u) ? (kWindowSize - 1u) : (state_.reading_index - 1u);
  return state_.raw_readings[last_index];
}

bool DS18B20Sensor::isValid() const {
  return state_.valid != 0u;
}

uint8_t DS18B20Sensor::getFaultCode() const {
  return state_.fault_code;
}

bool DS18B20Sensor::isPlausible_(float temp_c) const {
  return (temp_c >= -10.0f) && (temp_c <= 150.0f);
}

bool DS18B20Sensor::checkRateOfChange_(float new_temp_c, uint32_t now_ms) const {
  if (!isValid()) {
    return true;
  }
  if (state_.last_valid_temp == 0.0f) {
    // No baseline yet (matches phase-3 spec behavior).
    return true;
  }

  const uint32_t dt_ms = now_ms - state_.last_valid_ms;
  if (dt_ms == 0u) {
    return true;
  }

  const float dt_s = static_cast<float>(dt_ms) / 1000.0f;
  const float allowed_delta = kMaxRateCPerSec * dt_s;
  return absf(new_temp_c - state_.last_valid_temp) <= allowed_delta;
}

float DS18B20Sensor::applyEMA_(float new_value) {
  if (!isValid()) {
    state_.ema_temp = new_value;
    return new_value;
  }
  state_.ema_temp = (kEmaAlpha * new_value) + ((1.0f - kEmaAlpha) * state_.ema_temp);
  return state_.ema_temp;
}

float DS18B20Sensor::computeMedian_() const {
  float v[kWindowSize];
  for (uint8_t i = 0u; i < kWindowSize; i++) {
    v[i] = state_.raw_readings[i];
  }

  // Insertion sort for 5 values (small, predictable).
  for (uint8_t i = 1u; i < kWindowSize; i++) {
    const float key = v[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = key;
  }

  return v[2];
}

void DS18B20Sensor::recordFault_(uint8_t code, uint32_t now_ms) {
  if (state_.fault_counter < 255u) {
    state_.fault_counter++;
  }

  // Requirement 2: detect disconnect within 2 seconds.
  if ((now_ms - state_.last_valid_ms) >= kDisconnectTimeoutMs) {
    state_.valid = 0u;
    state_.fault_code = 2u;
    return;
  }

  // Requirement 2: enter fault only after 3 consecutive failures.
  if (state_.fault_counter >= kFaultTripCount) {
    state_.valid = 0u;
    state_.fault_code = code;
    return;
  }

  // Transient failures do not immediately invalidate the sensor (design.md).
  // Keep the last valid filtered value; fault counter continues to accumulate.
}

void DS18B20Sensor::update() {
  const uint32_t now = millis();

  // Allow multiple instantaneous state transitions per scheduler tick so we can
  // (1) start conversions ASAP and (2) start the next conversion immediately
  // after reading, achieving >=2 Hz effective sampling with a 250ms update rate.
  for (uint8_t steps = 0u; steps < 4u; steps++) {
    switch (state_.state) {
      case State::IDLE:
        state_.state = State::REQUEST_CONVERSION;
        continue;

      case State::REQUEST_CONVERSION:
        (void)sensors_.requestTemperatures();
        state_.state_start_ms = now;
        state_.state = State::WAIT_CONVERSION;
        return;

      case State::WAIT_CONVERSION:
        if (sensors_.isConversionComplete() || (now - state_.state_start_ms) >= conversion_wait_ms_) {
          state_.state = State::READ_DATA;
          continue;
        }
        return;

      case State::READ_DATA: {
        const float raw = sensors_.getTempCByIndex(0);

        state_.raw_readings[state_.reading_index] = raw;
        state_.reading_index = static_cast<uint8_t>((state_.reading_index + 1u) % kWindowSize);
        if (state_.sample_count < kWindowSize) {
          state_.sample_count++;
        }

        if (raw == DEVICE_DISCONNECTED_C) {
          // Library reports disconnected/CRC/read failure via sentinel.
          recordFault_(1u, now);
          state_.state = State::IDLE;
          continue;
        }

        // Before the median/EMA pipeline has enough samples, treat any plausible reading
        // as "sensor present" to prevent spurious TEMP_SENSOR_FAULT at boot.
        if (state_.sample_count < kWindowSize) {
          if (!isPlausible_(raw)) {
            recordFault_(3u, now);
            state_.state = State::IDLE;
            continue;
          }

          state_.filtered_temp = raw;
          state_.ema_temp = raw;
          state_.last_valid_ms = now;
          state_.fault_counter = 0u;
          state_.fault_code = 0u;
          state_.valid = 1u;
          state_.state = State::IDLE;
          continue;
        }

        if (state_.sample_count >= kWindowSize) {
          const float median = computeMedian_();
          if (!isPlausible_(median)) {
            recordFault_(3u, now);
            state_.state = State::IDLE;
            continue;
          }

          if (!checkRateOfChange_(median, now)) {
            recordFault_(3u, now);
            state_.state = State::IDLE;
            continue;
          }

          // Valid reading path.
          const float filtered = applyEMA_(median);
          state_.filtered_temp = filtered;
          state_.last_valid_temp = median;
          state_.last_valid_ms = now;
          state_.fault_counter = 0u;
          state_.fault_code = 0u;
          state_.valid = 1u;
        }

        state_.state = State::IDLE;
        continue;
      }

      default:
        state_.state = State::IDLE;
        return;
    }
  }
}
