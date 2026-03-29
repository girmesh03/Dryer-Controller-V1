#ifndef FAULTS_H
#define FAULTS_H

#include <Arduino.h>

// Phase 10: Fault System, History, and Diagnostics
// - Latching fault manager with safety-first detection.
// - Fault messages stored in Flash (PROGMEM).

enum class FaultCode : uint8_t {
  NONE = 0,
  DOOR_OPEN = 1,
  TEMP_SENSOR_FAULT = 2,
  OVER_TEMP = 3,
  THERMAL_RUNAWAY = 4,
  HEATING_TIMEOUT = 5,
  OUTPUT_FAULT = 6,
  WATCHDOG_RESET = 7,
  BROWNOUT = 8,
  SELF_TEST_FAIL = 9,
};

class FaultManager {
public:
  void init();
  void update(); // Call at 10 Hz (100ms)

  void setFault(FaultCode code);
  void clearFault();

  bool hasFault() const;
  FaultCode getCurrentFault() const;
  const __FlashStringHelper* getFaultMessage() const;
  bool canClearFault() const;

  uint32_t getFaultStartMs() const;
  float getTempAtFault() const;

private:
  struct {
    FaultCode current_fault;
    uint32_t fault_start_ms;
    float temp_at_fault_c;

    // Detection bookkeeping.
    uint32_t sensor_invalid_start_ms;
    uint32_t heating_start_ms;

    int16_t temp_hist_deci_c[3];
    uint32_t last_temp_sample_ms;
    uint8_t temp_hist_count;

    uint8_t latched : 1;
  } state_;

  void resetDetectionState_();
  void sampleRunaway_(uint32_t now_ms, int16_t temp_deci_c, bool heater_on);
};

#endif
