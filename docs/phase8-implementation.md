# Phase 8 Implementation Summary (AutoTune Integration + Guided UI Flow)

This document summarizes what was implemented in **Phase 8** and how it integrates with the existing Phase 1–7 codebase.

## What Was Added

### PID AutoTune (Åström–Hägglund relay method)

- Updated `pid_control` (guarded by `ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE`):
  - Adds relay-feedback AutoTune that toggles heater command between **0%** and **50%** duty.
  - Detects oscillation peaks and measures the ultimate period **Tu** (seconds).
  - Computes ultimate gain **Ku** and derives PID gains using Ziegler–Nichols:
    - `Kp = 0.6 * Ku`
    - `Ki = 1.2 * Ku / Tu`
    - `Kd = 0.075 * Ku * Tu`
  - Requires **3 complete cycles** before reporting results.
  - Safety checks (Req 6 / design):
    - Door open → abort (and transitions to FAULT in UI)
    - Sensor invalid → abort
    - Over-temp (≥95°C) → abort
    - Timeout (~30 min) → abort
    - STOP key abort handled in the App state machine

### Service UI: “AUTO TUNE”

- Updated `app` (guarded by `ENABLE_SERVICE_MENU && ENABLE_SERVICE_AUTOTUNE`):
  - Adds service menu item **AUTO TUNE**
  - Guided preconditions screen (door/sensor/ambient check)
  - `A/START` begins tuning, `B/STOP` aborts at any time
  - Progress screen updates at 1 Hz (cycle count, PV, elapsed time, duty command)
  - Completion screen shows `Ku`, `Tu`, and computed `Kp/Ki/Kd`, with `A` to save and `B` to reject

### HeaterControl service-step correctness

- Updated `heater_control`:
  - Slew-rate limiting (10%/window) now applies only in `SystemState::RUNNING_HEAT`.
  - Service tools (FOPDT / AutoTune / Heater Test) can apply true step changes without being “slew-ramped”, improving identification accuracy.

## Integration Updates

### EEPROM persistence (PID gains)

- AutoTune accept path calls `EEPROMStore::requestSavePIDGains(...)`.
- `src/main.cpp` loads PID gains at boot via `EEPROMStore::loadPIDGains(...)` and applies them with `pidController.setTunings(...)`.
- EEPROM integrity is protected by the existing **reserved-map CRC8** scheme (global CRC in header).

### Build workflow

On ESP32, Flash/SRAM headroom allows building a **single firmware image** with both operator flows and commissioning tools enabled. PID gains saved by AutoTune are stored in EEPROM and loaded at startup.

## Hardware Checkpoint (User Action)

Build and upload:
- `pio run -e esp32`
- `pio run -e esp32 -t upload`

Bench verify:
- Enter Service menu: `* * 5`
- Select `AUTO TUNE`
- Confirm ambient PV 15–30°C, door closed, sensor valid
- Press `A` to start
- Observe heater cycling (0% / 50%), and PV oscillation
- When complete, press `A` to save
- Wait **≥5 seconds** before power-cycling (EEPROM deferred write rate-limit)

To verify saved gains after reboot:
- Use `PID VIEW` from the Service menu, or enable Serial debug output.
