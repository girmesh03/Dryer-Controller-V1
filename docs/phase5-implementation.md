# Phase 5 Implementation Summary (Heater Control + Time-Proportioning + Relay Gating)

This document summarizes what was implemented in **Phase 5** and the patterns Phase 6+ builds on.

## What Was Added

### `heater_control` module

- Files:
  - `include/heater_control.h`
  - `lib/heater_control/heater_control.cpp`
- Implements time-proportioning heater control:
  - Default window period: **10s**
  - Duty: 0–100%
  - ON-time per window computed as `window_ms * duty / 100`
- Anti-chatter protections (Req 3):
  - Minimum ON time: `HEATER_MIN_ON_TIME_S` (10s)
  - Minimum OFF time: `HEATER_MIN_OFF_TIME_S` (10s)
  - Safety gating forces OFF immediately and does **not** honor minimum ON time.
- Slew limiting (Req 3):
  - Limits duty change to **±10% per window** (tracked via `last_duty` at window boundaries).
- Deadband helper (Req 3):
  - `±1.0°C` deadband check is implemented and applied during `RUNNING_HEAT` (setpoint is provided later in Phase 6+).

### Safety gating logic

Heater energization is permitted only when all conditions are met:
- `enable()` called
- door closed (`IOAbstraction`)
- DS18B20 sensor valid
- no fault latched (placeholder flag until Phase 10 fault system)
- system state allows heating:
  - `RUNNING_HEAT`, or
  - `SERVICE` + `HEATER TEST` active
- temperature below `OVER_TEMP_THRESHOLD`

## Integration Updates

### `src/main.cpp`

- Added global `HeaterControl heaterControl;`
- Runs `heaterControl.update()` at 10 Hz (100ms)
- Applies output to relay each tick:
  - `io.setHeaterRelay(heaterControl.isHeaterOn())`

## Service Menu: HEATER TEST

- Added `HEATER TEST` option under Service menu (`**5` from IDLE).
- Screen shows:
  - `HEATER TEST`
  - `DUTY: XX%`
  - `STATE: ON/OFF`
  - `UP/DN:ADJUST`
- Adjust duty in **5% steps** with UP/DOWN.
- All safety gating remains enforced (door closed + sensor valid + over-temp protection).

## Hardware Checkpoint (User Action)

Run and report:
- `pio run -e nanoatmega328`
- `pio run -e nanoatmega328 -t upload`

Verify on bench:
- Enter Service menu: `* * 5`
- Select `HEATER TEST`, set duty 0/50/100, observe D9 output behavior
- Note: with 10s minimum ON/OFF times, 50% duty will be enforced as a longer-period pattern (average 50%), not 5s ON / 5s OFF.
- Open door: heater output drops immediately
- Sensor disconnect: heater output forced OFF
