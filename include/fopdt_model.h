#ifndef FOPDT_MODEL_H
#define FOPDT_MODEL_H

#include <Arduino.h>

class FOPDTModel {
public:
  enum class State : uint8_t { IDLE, BASELINE, STEP, MEASURE, COMPLETE };

  void init();

  void startIdentification(float step_size); // 0.0-1.0 (fraction of full-scale output)
  void abort();
  bool isIdentifying() const;
  State getState() const;

  void updateIdentification(float temp_c, float output_percent);
  bool getResults(float& k, float& tau_s, float& l_s) const;

  void computePIDGains(float k, float tau_s, float l_s, float& kp, float& ki, float& kd) const;

  uint8_t getStepDutyPercent() const;

  uint16_t getBaselineSecondsRemaining() const;
  uint16_t getMeasureSecondsElapsed() const;

private:
  static constexpr uint8_t kSampleCount = 16u;

  struct {
    State state;
    uint8_t identifying : 1;
    uint8_t result_valid : 1;

    float step_size; // fraction (0.0-1.0)

    int32_t baseline_sum_deci;
    uint8_t baseline_samples;
    int16_t baseline_temp_deci;

    uint32_t state_start_ms;
    uint32_t step_start_ms;

    int16_t peak_temp_deci;
    uint16_t delay_s;
    uint16_t tau_s;

    uint8_t next_sample_index;
    int16_t sample_temp_deci[kSampleCount];

    float result_k;
    float result_tau_s;
    float result_l_s;
  } state_;

  void computeResults_();
};

#endif

