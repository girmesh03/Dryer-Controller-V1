# Phase 4 Implementation Summary (Drum Control + Safe Reversing + Door Interlock)

This document summarizes what was implemented in **Phase 4** and the patterns Phase 5+ builds on.

## What Was Added

### `drum_control` module

- Files:
  - `include/drum_control.h`
  - `lib/drum_control/drum_control.cpp`
- Implements a reversing FSM:
  - `STOPPED → FORWARD → STOP_BEFORE_REV → REVERSE → STOP_BEFORE_FWD → FORWARD ...`
  - Enforces **minimum 2s stop** before any direction change using `MOTOR_DIRECTION_DELAY_MS`.
- Public API:
  - `setPattern(fwd_s, rev_s, stop_s)`, `start()`, `stop()`, `isRunning()`, `getCurrentDirection()`.
- Default patterns (Req 9) stored in **PROGMEM**:
  - 6 built-in patterns (Towels/Bed Sheets/Delicates/Heavy Cotton/Mixed/Synthetic)
  - Accessible via `DrumControl::getDefaultPattern(program_index)`

### Door interlock behavior

- `DrumControl` refuses to start if the door is open (debounced door state from `IOAbstraction`).
- If the door opens while running, `DrumControl` stops and remains stopped until manually restarted.
- `IOAbstraction` continues to enforce immediate output shutdown on door open at the I/O layer.

## Integration Updates

### `src/main.cpp`

- Added global `DrumControl drumControl;`
- Scheduler updates:
  - `io.update()` now runs at **50ms** to meet the door-response requirement while keeping 50ms debounce.
  - `drumControl.update()` runs at **100ms**.
- Motor relay outputs are driven from the drum direction every 100ms:
  - `FORWARD` → `io.setMotorForward(true)`
  - `REVERSE` → `io.setMotorReverse(true)`
  - stop states → both OFF

### Service menu: Drum Test mode

- `AppStateMachine` service menu now includes:
  - `DRUM TEST`, `PID VIEW (stub)`, `I/O TEST (stub)`
- Selecting **DRUM TEST**:
  - applies pattern `50/50/5`
  - starts the drum FSM
  - displays current direction (`DIR: FORWARD/REVERSE/STOPPED`)
  - `STOP (B)` stops the drum test and returns to the service menu

## Hardware Checkpoint (User Action)

Run and report:
- `pio run -e esp32`
- `pio run -e esp32 -t upload`

Verify on bench:
- Enter Service menu: `* * 5`
- Start Drum Test and observe D10/D11 LEDs:
  - ESP32 relay outputs: `PIN_MOTOR_FWD_RELAY` (GPIO27) / `PIN_MOTOR_REV_RELAY` (GPIO26)
  - Forward ON for 50s, then both OFF for 5s, then Reverse ON for 50s, repeat
  - Forward and Reverse never ON at the same time
- Open door during motion: both outputs OFF within ~100ms
- Close door: drum does not restart automatically (manual restart required)
