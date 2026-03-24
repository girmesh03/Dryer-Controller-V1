#include "drum_control.h"

#include <Arduino.h>

#include "config_build.h"
#include "io_abstraction.h"

extern IOAbstraction io;

namespace {
constexpr uint8_t kProgramCount = 6u;

const DrumControl::DrumPattern kDefaultPatterns[kProgramCount] PROGMEM = {
    {45u, 45u, 5u}, // Towels
    {60u, 60u, 6u}, // Bed Sheets
    {40u, 40u, 8u}, // Delicates
    {50u, 50u, 5u}, // Heavy Cotton
    {50u, 50u, 5u}, // Mixed Load
    {45u, 45u, 6u}, // Synthetic
};

uint32_t secondsToMs(uint8_t seconds) {
  return static_cast<uint32_t>(seconds) * 1000u;
}
} // namespace

void DrumControl::init() {
  state_.state = State::STOPPED;
  state_.state_start_ms = millis();
  const DrumPattern p = getDefaultPattern(0u);
  state_.fwd_time_s = p.fwd_s;
  state_.rev_time_s = p.rev_s;
  state_.stop_time_s = p.stop_s;
  state_.running = 0u;
}

void DrumControl::setPattern(uint8_t fwd_time_s, uint8_t rev_time_s, uint8_t stop_time_s) {
  state_.fwd_time_s = fwd_time_s;
  state_.rev_time_s = rev_time_s;
  state_.stop_time_s = stop_time_s;
}

void DrumControl::transitionTo_(State new_state, uint32_t now_ms) {
  state_.state = new_state;
  state_.state_start_ms = now_ms;
}

void DrumControl::start() {
  if (!io.isDoorClosed()) {
    stop();
    return;
  }

  if (state_.running != 0u) {
    return;
  }

  state_.running = 1u;
  transitionTo_(State::FORWARD, millis());
}

void DrumControl::stop() {
  state_.running = 0u;
  transitionTo_(State::STOPPED, millis());
}

bool DrumControl::isRunning() const {
  return state_.running != 0u;
}

DrumControl::Direction DrumControl::getCurrentDirection() const {
  if (!isRunning()) {
    return Direction::STOPPED;
  }

  switch (state_.state) {
    case State::FORWARD:
      return Direction::FORWARD;
    case State::REVERSE:
      return Direction::REVERSE;
    default:
      return Direction::STOPPED;
  }
}

DrumControl::DrumPattern DrumControl::getDefaultPattern(uint8_t program_index) {
  const uint8_t idx = (program_index < kProgramCount) ? program_index : 0u;

  DrumPattern p{};
  p.fwd_s = static_cast<uint8_t>(pgm_read_byte(&kDefaultPatterns[idx].fwd_s));
  p.rev_s = static_cast<uint8_t>(pgm_read_byte(&kDefaultPatterns[idx].rev_s));
  p.stop_s = static_cast<uint8_t>(pgm_read_byte(&kDefaultPatterns[idx].stop_s));
  return p;
}

void DrumControl::update() {
  const uint32_t now = millis();

  if (!io.isDoorClosed()) {
    if (isRunning()) {
      stop();
    }
    return;
  }

  if (!isRunning()) {
    if (state_.state != State::STOPPED) {
      transitionTo_(State::STOPPED, now);
    }
    return;
  }

  const uint32_t elapsed_ms = now - state_.state_start_ms;

  switch (state_.state) {
    case State::STOPPED:
      transitionTo_(State::FORWARD, now);
      break;

    case State::FORWARD:
      if (elapsed_ms >= secondsToMs(state_.fwd_time_s)) {
        transitionTo_(State::STOP_BEFORE_REV, now);
      }
      break;

    case State::STOP_BEFORE_REV: {
      uint32_t wait_ms = secondsToMs(state_.stop_time_s);
      if (wait_ms < MOTOR_DIRECTION_DELAY_MS) {
        wait_ms = MOTOR_DIRECTION_DELAY_MS;
      }
      if (elapsed_ms >= wait_ms) {
        transitionTo_(State::REVERSE, now);
      }
      break;
    }

    case State::REVERSE:
      if (elapsed_ms >= secondsToMs(state_.rev_time_s)) {
        transitionTo_(State::STOP_BEFORE_FWD, now);
      }
      break;

    case State::STOP_BEFORE_FWD: {
      uint32_t wait_ms = secondsToMs(state_.stop_time_s);
      if (wait_ms < MOTOR_DIRECTION_DELAY_MS) {
        wait_ms = MOTOR_DIRECTION_DELAY_MS;
      }
      if (elapsed_ms >= wait_ms) {
        transitionTo_(State::FORWARD, now);
      }
      break;
    }

    default:
      transitionTo_(State::STOPPED, now);
      break;
  }
}
