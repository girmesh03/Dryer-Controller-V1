#ifndef APP_H
#define APP_H

#include <Arduino.h>

#include "config_build.h"
#include "eeprom_store.h"
#include "keypad_input.h"

enum class SystemState : uint8_t {
  BOOT,
  IDLE,
  MENU,
  PROGRAM_SELECT,
  PARAM_EDIT,
  SETTINGS,
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

#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_IO_TEST
  bool isIoTestActive() const;
  void getIoTestOutputs(bool& heater_on, bool& motor_fwd_on, bool& motor_rev_on, bool& aux_on) const;
#endif

private:
  struct {
    SystemState current_state;
    uint32_t state_entry_time_ms;
    SystemState previous_state;

    uint8_t menu_selection;

    // Entry sequences from IDLE (e.g. **5 for Service, **8 for Settings).
    uint8_t entry_seq;
    uint32_t entry_seq_start_ms;

    // AUTO mode program selection / review / temporary edit (Phase 9).
    uint8_t auto_program_index;
    EEPROMStore::Program auto_program;
    uint8_t auto_temp_c;
    uint8_t auto_duration_min;
    uint8_t auto_flags;
    uint8_t param_mode; // 0=manual placeholder, 1=auto review/edit

#if ENABLE_CYCLE_EXECUTION
    // Manual mode entry screens (Req 12B).
    uint8_t manual_temp_c;
    uint8_t manual_duration_min;
    uint8_t manual_view; // 0=temp, 1=duration

    // Active cycle parameters (copied from AUTO/Manual inputs on start).
    uint8_t cycle_setpoint_c;
    uint8_t cycle_duration_min;
    uint8_t cycle_duty_limit;
    uint32_t cycle_last_ui_ms;

    // Original parameters at cycle start (for in-cycle edit limits + asterisk indicator).
    uint8_t cycle_orig_setpoint_c;
    uint8_t cycle_orig_duration_min;

    // Heating phase timing (pause-aware).
    uint32_t heat_start_ms;
    uint32_t heat_pause_total_ms;

    // Cooldown phase timing (pause-aware).
    uint32_t cooldown_start_ms;
    uint32_t cooldown_pause_total_ms;

    // Pause/resume state.
    uint32_t pause_start_ms;
    SystemState paused_from_state;
    uint32_t pause_timeout_msg_start_ms;
    uint8_t pause_timeout_shown : 1;

    // In-cycle parameter modification (Req 13).
    uint8_t cycle_edit_active : 1;
    uint8_t cycle_edit_field : 1; // 0=temp, 1=time
    uint8_t cycle_temp_modified : 1;
    uint8_t cycle_time_modified : 1;
    uint8_t cycle_edit_temp_c;
    uint8_t cycle_edit_duration_min;

    // Cycle completion / anti-crease.
    uint32_t cycle_total_time_ms; // heat_total + cooldown_elapsed (pause-aware)
    uint32_t anticrease_start_ms;
    uint32_t tumble_cycle_start_ms;
    uint32_t tumble_start_ms;
    uint8_t anticrease_page; // 0=summary, 1=active
    uint8_t tumble_state;    // 0=idle, 1=tumbling
#endif

#if ENABLE_SETTINGS_MENU
    uint8_t settings_menu_selection;
    uint8_t settings_view;
#if ENABLE_PROGRAM_EDITOR
    uint8_t program_list_selection;
    uint8_t program_edit_index;
    EEPROMStore::Program program_edit;
    uint8_t program_edit_field;
    uint8_t program_name_cursor;
#endif
#endif

#if ENABLE_SERVICE_MENU
    uint8_t service_menu_selection;
    uint8_t service_view;
#if ENABLE_SERVICE_FAULT_HISTORY
    uint8_t fault_history_index;
    uint8_t fault_history_page; // 0=view, 1=confirm clear
#endif
#if ENABLE_SERVICE_IO_TEST
    uint8_t io_test_page;      // 0=outputs, 1=sensors
    uint8_t io_test_selection; // 0=heater,1=fwd,2=rev,3=aux (optional)
    uint8_t io_test_out_mask;  // bit0=heater, bit1=fwd, bit2=rev, bit3=aux
    uint32_t io_test_last_update_ms;
    uint32_t io_test_last_sig;
#endif
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
#if ENABLE_SERVICE_MEMORY_INFO
    uint8_t service_mem_page;
    uint32_t service_mem_last_update_ms;
#endif
#endif

    uint32_t invalid_key_until_ms;
    uint32_t last_temp_display_ms;
    uint8_t last_temp_valid : 1;
  } state_;

  bool canTransition_(SystemState from, SystemState to) const;

  void onEnter_(SystemState new_state);
  void onExit_(SystemState old_state);
  void renderFault_();
  void renderFaultHistory_();

  void showInvalidKey_();
  void restoreScreen_();

  void renderProgramSelect_();
  void renderAutoParamReview_();
  void renderAutoParamEdit_();

#if ENABLE_CYCLE_EXECUTION
  void renderManualTemp_();
  void renderManualDuration_();
  void renderManualConfirm_();

  void renderStartDelay_();
  void renderRunningHeat_();
  void updateRunningHeatUi_();
  void renderRunningCooldown_();
  void updateRunningCooldownUi_();
  void renderPaused_();
  void updatePausedUi_();
  void renderCycleEdit_();
  void renderCycleCompleteSummary_();
  void renderAntiCreaseActive_();
  void updateAntiCreaseUi_();
#endif

#if ENABLE_SETTINGS_MENU
  void renderSettings_();
  void renderSettingsMenu_();
  void renderSettingsTempUnits_();
  void renderSettingsSound_();
#if ENABLE_PROGRAM_EDITOR
  void renderProgramList_();
  void renderProgramEdit_();
#endif
#endif

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
#if ENABLE_SERVICE_IO_TEST
  void renderIoTest_();
  void updateIoTestUi_();
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
#if ENABLE_SERVICE_MEMORY_INFO
  void renderMemoryInfo_();
#endif
#endif
};

#endif
