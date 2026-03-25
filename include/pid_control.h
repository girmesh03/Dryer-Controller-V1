#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <Arduino.h>

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
  } state_;

  float constrainOutput_(float out) const;
  void rampSetpoint_(float dt_s);
};

#endif

