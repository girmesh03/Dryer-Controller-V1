#ifndef IO_ABSTRACTION_H
#define IO_ABSTRACTION_H

#include <Arduino.h>

class IOAbstraction {
public:
  void init();
  void update(); // Call at 20 Hz (50ms)

  // Inputs (debounced)
  bool isDoorClosed() const;

  // Outputs (active-HIGH, interlocks enforced)
  void setHeaterRelay(bool state);
  void setMotorForward(bool state);
  void setMotorReverse(bool state);

  // Safety override
  void emergencyStop();

  // Output verification
  bool verifyOutputState(uint8_t pin, bool expected);
  uint8_t getOutputMismatchCount() const;

  // Audible feedback
  void beep(uint16_t freq_hz, uint16_t duration_ms);
  void beepKey();

private:
  struct {
    uint32_t last_direction_change_ms;
    uint8_t door_stable_count;
    uint8_t output_mismatch_count;
    uint8_t flags;
  } state_;

  static constexpr uint8_t FLAG_DOOR_CLOSED = 1u << 0;
  static constexpr uint8_t FLAG_DOOR_RAW_CLOSED = 1u << 1;
  static constexpr uint8_t FLAG_HEATER_ON = 1u << 2;
  static constexpr uint8_t FLAG_MOTOR_FWD_ON = 1u << 3;
  static constexpr uint8_t FLAG_MOTOR_REV_ON = 1u << 4;

  void setFlag(uint8_t flag, bool value);
  bool getFlag(uint8_t flag) const;
};

#endif

