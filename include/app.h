#ifndef APP_H
#define APP_H

#include <Arduino.h>

#include "keypad_input.h"

enum class SystemState : uint8_t {
  BOOT,
  IDLE,
  MENU,
  PROGRAM_SELECT,
  PARAM_EDIT,
  READY,
  START_DELAY,
  RUNNING_HEAT,
  RUNNING_COOLDOWN,
  PAUSED,
  STOPPING,
  FAULT,
  AUTOTUNE,
  SERVICE,
  ANTI_CREASE,
};

class AppStateMachine {
public:
  void init();
  void update(); // Call at 10 Hz (100ms)

  SystemState getCurrentState() const;
  void transitionTo(SystemState new_state);
  void handleKeyPress(KeypadInput::Key key);

private:
  struct {
    SystemState current_state;
    uint32_t state_entry_time_ms;
    SystemState previous_state;

    uint8_t menu_selection;

    uint8_t service_seq;
    uint32_t service_seq_start_ms;
    uint8_t service_menu_selection;
    uint8_t service_view;
    uint8_t service_last_dir;

    uint32_t invalid_key_until_ms;
    uint32_t last_temp_display_ms;
    uint8_t last_temp_valid : 1;
  } state_;

  bool canTransition_(SystemState from, SystemState to) const;

  void onEnter_(SystemState new_state);
  void onExit_(SystemState old_state);

  void showInvalidKey_();
  void restoreScreen_();

  void renderService_();
  void renderServiceMenu_();
  void renderDrumTest_();
  void updateDrumTestDirection_();
};

#endif
