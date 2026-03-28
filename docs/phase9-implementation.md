# Phase 9 Implementation Summary (Programs + EEPROM Persistence + Editor)

This document summarizes what was implemented in **Phase 9** and how it integrates with the existing Phase 1–8 codebase.

## What Was Added

### EEPROMStore: full reserved-map layout + CRC8 + deferred writes

- Expanded `eeprom_store` to match the `design.md` 512-byte reserved map and **global CRC8** integrity check:
  - Magic (`'D''R'` = `0x4452`) @ `0x000`, Version (`0x01`) @ `0x002`, CRC8 @ `0x003`
  - Settings (20B) @ `0x004`
  - Programs (6 × 40B = 240B) @ `0x018`
  - PID gains (3 × float) @ `0x108`
  - FOPDT params (3 × float) @ `0x114`
  - Fault history reserved (200B) @ `0x120`
  - Filter clean counter (4B) @ `0x1E8`
- CRC8:
  - Polynomial `0x07`, init `0x00`
  - Computed over the entire 512-byte reserved region **excluding** `0x003` (CRC byte itself).
- Deferred write strategy (Req 22):
  - `requestSave*()` marks pending blocks.
  - `EEPROMStore::update(now_ms)` flushes pending writes **at most once per 5 seconds**.
  - Header + CRC are updated after any flush.
- Factory reset:
  - `EEPROMStore::factoryReset()` writes default settings/programs/PID/FOPDT, clears reserved region, then writes header + CRC.
  - Default `filter_interval` is **50 cycles** (Req 43 / `design.md`).

### Program table (6 programs) + defaults in PROGMEM

- `EEPROMStore::Program` is a packed 40-byte entry:
  - `name[13]` (12 chars + NUL), `temp_setpoint`, `duration_min`, `fwd_time_s`, `rev_time_s`, `stop_time_s`, `duty_limit`, reserved padding.
- Default programs stored in PROGMEM and written during factory reset.

### Settings menu + Program Editor (EEPROM-backed)

- New `SystemState::SETTINGS` with entry sequence from IDLE:
  - `* * 8` → Settings menu (key `8` maps to LEFT)
- Settings menu screens:
  - Temperature units (C/F) stored in EEPROM
  - Sound enable/disable stored in EEPROM
  - Program editor entry
- Program editor:
  - List view shows 3 programs at a time (scrolls through all 6).
  - Edit view supports modifying:
    - **Name** (12 chars), **Temp setpoint**, **Duration**, **FWD/REV/STOP** pattern, **Duty limit**
  - Save requires confirmation prompt (Req 28).
  - Save uses EEPROM deferred write + CRC update (`requestSaveProgram()` → `update()`).

### Auto mode: program selection + temporary parameter edit (non-persistent)

- AUTO flow implemented (Req 12A / `design.md`):
  - `IDLE → PROGRAM_SELECT → PARAM_EDIT → START_DELAY → RUNNING_HEAT`
- Parameter review:
  - Shows program setpoint/duration loaded from EEPROM.
  - `A/START` starts the cycle (enters `START_DELAY`, then `RUNNING_HEAT`).
  - `*` enters temporary edit mode.
- Temporary edit mode:
  - Temp and duration edits are **NOT persisted** to EEPROM program definitions (Req 34).
  - Review screen marks modified fields with `*`.

### Operator cycle execution: RUNNING_HEAT (PID-driven)

- `RUNNING_HEAT` is now functional in the **operator build**:
  - PID setpoint uses the selected program temperature (or the temporary edited value).
  - PID output is limited by the selected program `duty_limit`.
  - PID gains are loaded from EEPROM at boot (if valid) and applied automatically.
  - Drum pattern uses the selected program’s `fwd/rev/stop` timings.
- Safety behavior (current):
  - Door open / temp sensor invalid / over-temp transitions to **FAULT** and latches.
  - Clearing requires conditions OK + operator acknowledgment (`B/STOP`), then returns to IDLE.
  - The cycle does **not** resume automatically after fault clear (operator must restart).
- Cycle screen shows:
  - `AUTO: <PROGRAM>` (or `MANUAL MODE`) on line 1
  - `SP:xxx PV:xxx°<unit>` on line 2
  - `TIME: mmm:ss` remaining on line 3
  - Status flags on line 4: `H` (heater), `F` (forward), `R` (reverse) plus `B:STOP`

### Manual mode: parameter input → confirmation → RUNNING_HEAT

- Manual mode is implemented per Req 12B (operator build):
  - Entry from main menu opens temperature input screen (defaults **60°C**, wraps 40–85).
  - OK advances to duration input (defaults **30 min**, wraps 10–120).
  - OK advances to confirmation screen (`A:START B:CANCEL`).
  - `*` on confirmation returns to temperature input for re-entry.
- Manual cycle behavior:
  - Uses operator-entered setpoint/duration (not persisted to EEPROM).
  - Uses fixed drum pattern **50/50/5** (Mixed_Load default).
  - Uses PID gains loaded from EEPROM at boot.

## Integration Updates

### Settings load + temperature unit application

- `src/main.cpp` loads settings at boot via `EEPROMStore::loadSettings(...)`.
- `UILCD::setTempUnit(unit)` sets the display unit used by `UILCD::showTemperature()`:
  - Celsius: shows `xx.x°C`
  - Fahrenheit: shows **rounded whole degrees** `xxx°F` (Req 36)

### Key beep gated by stored sound setting

- Keypress beep is now gated by `Settings.sound_enabled` (Req 37) in `src/main.cpp`.

### Service menu: Factory Reset entry (Req 30)

- New service tool (guarded by `ENABLE_SERVICE_FACTORY_RESET`):
  - Confirm prompt shown in Service menu
  - On confirm: outputs forced OFF, EEPROM factory reset performed, then restart (ESP32: `ESP.restart()`)

## Build-Time Flags

On ESP32, feature-splitting flags are no longer required for Flash/SRAM reasons. The main remaining build-time toggle is serial logging (`ENABLE_SERIAL_DEBUG`).

## Hardware Checkpoint (User Action)

Build and upload:
- `pio run -e esp32`
- `pio run -e esp32 -t upload`

Verify:
- AUTO → select program → review screen → `*` edit temp/duration → OK to save temporary edit → confirm `*` markers.
- AUTO → press `A/START` → observe `START_DELAY` then `RUNNING_HEAT` screen; verify relay/drum status flags.
- MANUAL → set TEMP and TIME → confirm → `A/START` → observe `RUNNING_HEAT` screen.
- Power-cycle and confirm program table edits persist (program editor), while AUTO temporary edits do not.

Verify Factory Reset:
- Enter Service menu: `* * 5`
- Select `FACTORY RESET`, confirm prompt appears
- Press `A` to confirm → observe “RESET COMPLETE / RESTARTING” then reboot
- After reboot, programs/settings revert to defaults
