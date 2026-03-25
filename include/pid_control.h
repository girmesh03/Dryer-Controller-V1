#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <Arduino.h>

#include "config_build.h"

class PIDControl {
public:
  void init();

  void setTunings(float kp, float ki, float kd);
  void setSetpoint(float sp_c);
  void setOutputLimits(float min_out, float max_out);

  float compute(float input_c); // Returns output in configured units (0-100% for heater duty)

  void reset(); // Bumpless transfer (applies on next compute)

  // Lightweight accessors for service UI / integration.
  void getTunings(float& kp, float& ki, float& kd) const;
  float getTargetSetpoint() const;
  float getCurrentSetpoint() const;
  float getLastOutput() const;

#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE
  // AutoTune (relay feedback, Åström–Hägglund).
  void startAutoTune();
  void abortAutoTune();
  void endAutoTuneSession(); // clears autotune state without restoring gains
  bool isAutoTuning() const;
  bool isAutoTuneComplete() const;
  void updateAutoTune(float input_c);
  bool getAutoTuneResults(float& kp, float& ki, float& kd) const;
  bool getAutoTuneKuTu(float& ku, float& tu_s) const;
  uint8_t getAutoTuneCycleCount() const;
  uint8_t getAutoTuneCommandDuty() const; // 0 or relay high (%)
  uint8_t getAutoTuneAbortReason() const; // 0=none/OK
#endif

private:
  struct {
    float target_setpoint;
    float current_setpoint;
    float kp;
    float ki;
    float kd;
    float integral;
    float last_input;
    float output_min;
    float output_max;
    float last_output;
    uint32_t last_compute_ms;
    uint8_t pending_reset : 1;
    uint8_t have_last_input : 1;

#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE
    struct {
      uint8_t active : 1;
      uint8_t complete : 1;
      uint8_t phase; // internal autotune phase (see pid_control.cpp)

      uint8_t cmd_duty; // 0 or relay_high
      uint8_t relay_high; // percent
      uint8_t cycles; // completed cycles (Tu samples)
      uint8_t abort_reason;
      uint8_t last_heater_on : 1;
      uint8_t relay_edge_count;

      int16_t baseline_deci_c;
      int16_t peak_high_deci_c;
      int16_t peak_low_deci_c;

      int32_t sum_high_deci_c;
      int32_t sum_low_deci_c;
      uint8_t high_count;
      uint8_t low_count;

      uint32_t start_ms;
      uint32_t last_high_ms;
      uint32_t sum_tu_ms;
      uint8_t tu_count;

      float ku;
      float tu_s;
      float tuned_kp;
      float tuned_ki;
      float tuned_kd;

      float prev_kp;
      float prev_ki;
      float prev_kd;
    } autotune;
#endif
  } state_;

  float constrainOutput_(float out) const;
  void rampSetpoint_(float dt_s);
};

#endif
