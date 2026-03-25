# Phase 7 Implementation Summary (FOPDT Identification + IMC Tuning)

This document summarizes what was implemented in **Phase 7** and how it integrates with the existing Phase 1–6 codebase.

## What Was Added

### `fopdt_model` module

- Files:
  - `include/fopdt_model.h`
  - `lib/fopdt_model/fopdt_model.cpp`
- Implements a non-blocking FOPDT step-test identification utility (Req 7):
  - State machine: `IDLE → BASELINE → STEP → MEASURE → COMPLETE`
  - Baseline averaging over **30 seconds**
  - Step response measurement window of **10 minutes** (design guidance for near steady-state)
  - Dead time **L** detected at first response **> 0.5°C** above baseline
  - Time constant **τ** estimated at the time the response reaches **63.2%** of final rise
  - Process gain **K** computed as `ΔT / step_size` (°C per output fraction)
  - Uses a sparse, PROGMEM-stored sample schedule (16 points) and deci-°C integers to keep SRAM low

### IMC-based PI tuning computation

- Implemented in `FOPDTModel::computePIDGains(...)`:
  - Conservative IMC rule: `λ = τ`
  - PI form (thermal): `Kd = 0`
  - Converts IMC output-fraction gains into **percent-output** gains compatible with `PIDControl` (0–100%)
  - Includes a lightweight **Cohen–Coon PI fallback** when IMC results are extreme/invalid

## Integration Updates

### Heater gating for FOPDT

- Added a SERVICE-mode flag `g_fopdt_active` (guarded by `ENABLE_SERVICE_MENU`).
- Updated `lib/heater_control/heater_control.cpp` so heating is permitted in:
  - `SystemState::RUNNING_HEAT`, or
  - `SystemState::SERVICE` when `g_heater_test_active` **or** `g_fopdt_active` is set.

### Service menu: “FOPDT ID”

- Updated `lib/app/app.cpp` (guarded by `ENABLE_SERVICE_MENU`):
  - Adds a new service menu item: **FOPDT ID**
  - Preconditions enforced at START:
    - Door closed
    - Sensor valid
    - Ambient PV in **15–30°C** range
  - During identification:
    - Calls `fopdt.updateIdentification(...)` at **1 Hz**
    - Commands the heater step via `HeaterControl` (safety gating stays enforced in HeaterControl)
    - Aborts immediately if door opens or the sensor becomes invalid
  - Results display:
    - Page 1: `K`, `τ`, `L`
    - Page 2: computed `Kp`, `Ki`, `Kd` (PI → `Kd=0`)
    - `OK` toggles pages, `B` rejects/backs out, `A` saves
  - On save:
    - Stores **FOPDT parameters** to EEPROM via `EEPROMStore::requestSaveFOPDT(...)`
    - Applies the derived gains to `PIDControl` and stores them via `EEPROMStore::requestSavePIDGains(...)`

### EEPROM persistence

- Reuses the existing Phase 1 EEPROM architecture:
  - FOPDT stored at `0x114` as 3 floats (`K`, `τ`, `L`)
  - CRC8 validation uses the **global EEPROM reserved-map CRC** (stored at `0x003`) per `design.md`
  - No per-block CRC is added at `0x120` because `design.md` reserves `0x120` for fault history in later phases

## Hardware Checkpoint (User Action)

Run and report:
- Service build (Phase 7 tools): `pio run -e nanoatmega328`
- Upload: `pio run -e nanoatmega328 -t upload`
- Operator build (service compiled out): `pio run -e nano_operator`

Verify on bench:
- Enter Service menu: `* * 5`
- Select **FOPDT ID**
- Confirm preconditions (door closed, sensor valid, PV between 15–30°C)
- Press `A` to start:
  - Baseline runs for ~30s, then step begins and measurement continues
- On COMPLETE:
  - Confirm `K`, `τ`, `L` and computed gains are displayed
  - Press `A` to save
- Reboot and verify persisted gains/values (PID VIEW should show the new gains)

## Practical Workflow (Two Builds)

To stay within Arduino Nano Flash limits while still supporting commissioning tools:

- **Day-to-day operation:** upload `nano_operator` (service menu/tools compiled out).
  - Auto Mode: program select → optional temporary parameter edit → run PID using PID gains stored in EEPROM.
  - Manual Mode: parameter edit → run PID using PID gains stored in EEPROM against the setpoint.
- **Commissioning / tuning / diagnostics:** temporarily upload `nanoatmega328` (service build) with only the needed service tool(s) enabled (e.g. FOPDT).
  - Run the tool and **save** results (writes to EEPROM).
  - Then upload `nano_operator` again for normal operation.

Both builds share the same EEPROM layout/magic/version, so PID/FOPDT values saved by a service build are loaded and used by the operator build at startup.

## Build-Time Flags (Memory Optimization)

- `ENABLE_SERVICE_MENU` (default `1`)
  - Set to `0` to compile out the service menu/tools (saves Flash/SRAM).
  - When disabled: `FOPDT ID` is removed from the build and `g_fopdt_active` is not used.

### Granular Service tool flags

When `ENABLE_SERVICE_MENU=1`, each tool can be compiled in/out independently:
- `ENABLE_SERVICE_DRUM_TEST`
- `ENABLE_SERVICE_HEATER_TEST`
- `ENABLE_SERVICE_PID_VIEW`
- `ENABLE_SERVICE_IO_TEST`
- `ENABLE_SERVICE_FOPDT_ID`
- `ENABLE_SERVICE_AUTOTUNE`

The default `platformio.ini` service environments enable only one tool at a time to stay under Flash limits:
- `nanoatmega328`: `FOPDT ID`
- `nano_autotune`: `AUTO TUNE` (Phase 8)

Enable other tools only when you need them for bench work.
