#ifndef IO_ABSTRACTION_H
#define IO_ABSTRACTION_H

#include <Arduino.h>

class IOAbstraction {
public:
  void init();
  void update(); // Call at 10 Hz (100ms)

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
    uint8_t door_stable_count;
    uint8_t output_mismatch_count;
    uint32_t last_direction_change_ms;

    // Packed boolean flags (memory-optimized).
    uint8_t door_state : 1;      // 1=closed, 0=open (debounced)
    uint8_t door_raw_state : 1;  // last raw sampled state
    uint8_t heater_state : 1;
    uint8_t motor_fwd_state : 1;
    uint8_t motor_rev_state : 1;
  } state_;
};

#endif
