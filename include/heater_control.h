#ifndef HEATER_CONTROL_H
#define HEATER_CONTROL_H

#include <Arduino.h>

class HeaterControl {
public:
  void init();
  void update(); // Call at 10 Hz (100ms)

  void setDutyCycle(float duty_percent);      // 0-100%
  void setWindowPeriod(uint16_t period_ms);  // 5000-15000ms
  void enable();
  void disable();

  bool isHeaterOn() const;
  uint8_t getCurrentDuty() const;

private:
  struct {
    uint8_t duty_percent; // slew-limited
    uint16_t window_period_ms;
    uint32_t window_start_ms;
    uint32_t heater_on_start_ms;
    uint32_t heater_off_start_ms;
    uint8_t last_duty;
    uint8_t heater_state : 1;
    uint8_t enabled : 1;
  } state_;

  bool canTurnOn_(uint32_t now_ms) const;
  bool canTurnOff_(uint32_t now_ms) const;
  bool isWithinDeadband_(float temp_c, float setpoint_c) const;
  bool canEnergizeHeater_() const;
};

#endif

