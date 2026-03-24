#ifndef DRUM_CONTROL_H
#define DRUM_CONTROL_H

#include <Arduino.h>

class DrumControl {
public:
  enum class Direction : uint8_t { STOPPED, FORWARD, REVERSE };

  struct DrumPattern {
    uint8_t fwd_s;
    uint8_t rev_s;
    uint8_t stop_s;
  };

  void init();
  void update(); // Call at 10 Hz (100ms)

  void setPattern(uint8_t fwd_time_s, uint8_t rev_time_s, uint8_t stop_time_s);
  void start();
  void stop();
  bool isRunning() const;

  Direction getCurrentDirection() const;

  static DrumPattern getDefaultPattern(uint8_t program_index);

private:
  enum class State : uint8_t { STOPPED, FORWARD, STOP_BEFORE_REV, REVERSE, STOP_BEFORE_FWD };

  struct {
    State state;
    uint32_t state_start_ms;
    uint8_t fwd_time_s;
    uint8_t rev_time_s;
    uint8_t stop_time_s;
    uint8_t running : 1;
  } state_;

  void transitionTo_(State new_state, uint32_t now_ms);
};

#endif

