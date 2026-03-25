#ifndef APP_H
#define APP_H

#include <Arduino.h>

#include "config_build.h"
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

#if ENABLE_SERVICE_MENU
    uint8_t service_seq;
    uint32_t service_seq_start_ms;
    uint8_t service_menu_selection;
    uint8_t service_view;
#if ENABLE_SERVICE_DRUM_TEST
    uint8_t service_last_dir;
#endif
#if ENABLE_SERVICE_HEATER_TEST
    uint8_t heater_test_duty;
    uint8_t service_last_heater_duty;
    uint8_t service_last_heater_on;
#endif
#if ENABLE_SERVICE_PID_VIEW
    uint32_t service_pid_last_update_ms;
#endif
#if ENABLE_SERVICE_FOPDT_ID
    uint8_t service_fopdt_page;
    uint32_t service_fopdt_last_update_ms;
#endif
#if ENABLE_SERVICE_AUTOTUNE
    uint8_t service_autotune_page;
    uint32_t service_autotune_last_update_ms;
    uint32_t service_autotune_start_ms;
#endif
#endif

    uint32_t invalid_key_until_ms;
    uint32_t last_temp_display_ms;
    uint8_t last_temp_valid : 1;
  } state_;

  bool canTransition_(SystemState from, SystemState to) const;

  void onEnter_(SystemState new_state);
  void onExit_(SystemState old_state);

  void showInvalidKey_();
  void restoreScreen_();

#if ENABLE_SERVICE_MENU
  void renderService_();
  void renderServiceMenu_();
#if ENABLE_SERVICE_DRUM_TEST
  void renderDrumTest_();
  void updateDrumTestDirection_();
#endif
#if ENABLE_SERVICE_HEATER_TEST
  void renderHeaterTest_();
  void updateHeaterTestStatus_();
#endif
#if ENABLE_SERVICE_PID_VIEW
  void renderPidView_();
  void updatePidView_();
#endif
#if ENABLE_SERVICE_FOPDT_ID
  void renderFopdt_();
  void updateFopdt_();
#endif
#if ENABLE_SERVICE_AUTOTUNE
  void renderAutoTunePre_();
  void renderAutoTune_();
  void updateAutoTuneUi_();
#endif
#endif
};

#endif
