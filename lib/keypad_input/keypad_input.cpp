#include "keypad_input.h"

#include <Arduino.h>

#include "config_build.h"
#include "config_pins.h"

namespace {
constexpr uint8_t kRows = 4;
constexpr uint8_t kCols = 4;

char kKeymap[kRows][kCols] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'},
};

byte kRowPins[kRows] = {PIN_KEYPAD_ROW[0], PIN_KEYPAD_ROW[1], PIN_KEYPAD_ROW[2], PIN_KEYPAD_ROW[3]};
byte kColPins[kCols] = {PIN_KEYPAD_COL[0], PIN_KEYPAD_COL[1], PIN_KEYPAD_COL[2], PIN_KEYPAD_COL[3]};
} // namespace

KeypadInput::KeypadInput()
    : keypad_(makeKeymap(kKeymap), kRowPins, kColPins, kRows, kCols) {
  state_.last_key = Key::NONE;
  state_.last_key_time_ms = 0;
  state_.current_key = Key::NONE;
  state_.key_reported = 0;
}

void KeypadInput::init() {
  // Scan on every call to update(); implement the 50ms stable-time requirement
  // in this module (per spec) instead of relying on library debounce.
  keypad_.setDebounceTime(1);
  keypad_.setHoldTime(500);

  state_.last_key = Key::NONE;
  state_.last_key_time_ms = millis();
  state_.current_key = Key::NONE;
  state_.key_reported = 0;
}

KeypadInput::Key KeypadInput::mapKey_(char raw_key) {
  switch (raw_key) {
    case '4':
      return Key::UP;
    case '6':
      return Key::DOWN;
    case '8':
      return Key::LEFT;
    case '2':
      return Key::RIGHT;
    case '5':
      return Key::OK;
    case 'A':
      return Key::START;
    case 'B':
      return Key::STOP;
    case '*':
      return Key::KEY_STAR;
    case '#':
      return Key::KEY_HASH;
    case '0':
      return Key::KEY_0;
    case '1':
      return Key::KEY_1;
    case '3':
      return Key::KEY_3;
    case '7':
      return Key::KEY_7;
    case '9':
      return Key::KEY_9;
    case 'C':
      return Key::KEY_C;
    case 'D':
      return Key::KEY_D;
    default:
      return Key::NONE;
  }
}

void KeypadInput::update() {
  const uint32_t now = millis();
  (void)keypad_.getKeys();

  char pressed = NO_KEY;
  for (uint8_t i = 0; i < LIST_MAX; i++) {
    if (keypad_.key[i].kchar == NO_KEY) {
      continue;
    }
    if (keypad_.key[i].kstate == PRESSED || keypad_.key[i].kstate == HOLD) {
      pressed = keypad_.key[i].kchar;
      break;
    }
  }

  const Key mapped = (pressed == NO_KEY) ? Key::NONE : mapKey_(pressed);

  if (mapped != state_.last_key) {
    state_.last_key = mapped;
    state_.last_key_time_ms = now;
    state_.current_key = Key::NONE;
    state_.key_reported = 0;
    return;
  }

  // Require key to be stable for at least DEBOUNCE_TIME_MS before reporting.
  if (mapped != Key::NONE && !state_.key_reported && state_.current_key == Key::NONE &&
      (now - state_.last_key_time_ms) >= DEBOUNCE_TIME_MS) {
    state_.current_key = mapped;
    state_.key_reported = 1;
  }
}

KeypadInput::Key KeypadInput::getKey() {
  const Key k = state_.current_key;
  state_.current_key = Key::NONE;
  return k;
}

bool KeypadInput::isKeyPressed(Key key) const {
  return state_.last_key == key;
}
