#ifndef KEYPAD_INPUT_H
#define KEYPAD_INPUT_H

#include <Arduino.h>
#include <Keypad.h>

class KeypadInput {
public:
  enum class Key : uint8_t {
    NONE,
    UP,
    DOWN,
    LEFT,
    RIGHT,
    OK,
    START,
    STOP,
    KEY_0,
    KEY_1,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_5,
    KEY_6,
    KEY_7,
    KEY_8,
    KEY_9,
    KEY_STAR,
    KEY_HASH,
    KEY_C,
    KEY_D,
  };

  KeypadInput();

  void init();
  void update(); // Call at 20 Hz (50ms)

  Key getKey();
  bool isKeyPressed(Key key) const;

private:
  Keypad keypad_;

  struct {
    Key last_key;
    uint32_t last_key_time_ms;
    Key current_key;
    uint8_t key_reported : 1;
  } state_;

  static Key mapKey_(char raw_key);
};

#endif
