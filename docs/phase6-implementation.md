# Phase 6 Implementation Summary (PID Integration + Bumpless Transfer + Anti-Windup)

This document summarizes what was implemented in **Phase 6** and how it integrates with the existing Phase 1–5 codebase.

## What Was Added

### `pid_control` module

- Files:
  - `include/pid_control.h`
  - `lib/pid_control/pid_control.cpp`
- Implements a custom, size-focused PID controller (Req 5):
  - 10 Hz compute cadence enforced internally (100 ms minimum interval)
  - Derivative on measurement (reduces setpoint kick)
  - Anti-windup: integral accumulation is prevented when output is saturated and the error would push further into saturation
  - Setpoint ramping at **2°C/s** (target → current setpoint)
  - Output limits default to **0–100%** (heater duty)
- Default PID gains are stored in **PROGMEM** and loaded during `init()`:
  - Kp = **10.0**, Ki = **0.5**, Kd = **2.0**

### EEPROM gains override (consistency with Phase 1)

- In `src/main.cpp`, after `eepromStore.init()` and `pidController.init()`, stored PID gains are loaded from EEPROM when valid and applied via `pidController.setTunings()`.

## Integration Updates

### `src/main.cpp`

- Added global `PIDControl pidController;`
- Initializes PID in `setup()` and loads tunings from EEPROM when available.
- In the 10 Hz heater tick, when `SystemState::RUNNING_HEAT` and temperature is valid:
  - Computes PID output and feeds it into `heaterControl.setDutyCycle(...)`
  - Updates `g_setpoint_c` from PID target setpoint for HeaterControl deadband gating

### `lib/app/app.cpp`

- **Bumpless transfer hook:** On entry to `SystemState::RUNNING_HEAT`, `pidController.reset()` is called (applies on next compute), and `heaterControl.enable()` is asserted.
- **Service menu PID VIEW implemented (Req 19):**
  - Screen header: `PID PARAMETERS`
  - Displays: `Kp`, `Ki`, `Kd`, `SP`, `PV`, and `OUT`
  - Refreshes once per second (minimum LCD traffic; fixed-width line updates)

## Hardware Checkpoint (User Action)

Run and report:
- `pio run -e esp32`
- `pio run -e esp32 -t upload`

Verify on bench:
- Enter Service menu: `* * 5`
- Select `PID VIEW` and confirm default gains are shown (`10.0 / 0.5 / 2.0`)
- Confirm PV updates and OUT displays (OUT will remain at last computed value unless RUNNING_HEAT is entered in later phases)

## Build-Time Flags (Memory Optimization)

On ESP32, feature-splitting flags are no longer required for Flash/SRAM reasons; the firmware is intended to ship with all features enabled.
