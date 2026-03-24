# Phase 2 Implementation Summary (LCD + Keypad + Base Menu Framework)

This document summarizes what was implemented in **Phase 2** and the patterns that Phase 3+ builds on.

## What Was Added

### `ui_lcd` module

- Files:
  - `include/ui_lcd.h`
  - `lib/ui_lcd/ui_lcd.cpp`
- LCD: 20x4 I2C, address `0x27`.
- Screens implemented:
  - Boot screen:
    - `INDUSTRIAL DRYER`
    - `FW: v<FW_VERSION>`
    - `Self-Test: PASS`
    - Optional line-4 watchdog reset hint (`LAST RESET: WDT`)
  - Main menu:
    - Line 1: `MAIN MENU`
    - Line 2: `> AUTO` / `  AUTO`
    - Line 3: `> MANUAL` / `  MANUAL`
- Optimization:
  - Reusable `line_buffer_[21]` for formatting and SRAM control.
  - All fixed strings printed from Flash using `F()`.

### `keypad_input` module

- Files:
  - `include/keypad_input.h`
  - `lib/keypad_input/keypad_input.cpp`
- Key mapping per Requirement 14:
  - `4`=UP, `6`=DOWN, `8`=LEFT, `2`=RIGHT, `5`=OK, `A`=START, `B`=STOP, `*`=`KEY_STAR`, `#`=`KEY_HASH`
- Scan timing:
  - Update called every 50ms (20 Hz).
  - Enforces 50ms “stable key” requirement before reporting a press.
- Optimization:
  - No dynamic allocation, no `String`.
  - Minimal state stored (last key, stable time, one-shot report flag).

### `app` module (state machine foundation)

- Files:
  - `include/app.h`
  - `lib/app/app.cpp`
- Implements a finite state machine (subset of full design):
  - `BOOT` → after 3 seconds → `IDLE`
  - `IDLE` menu navigation and OK transitions:
    - AUTO → `PROGRAM_SELECT` (stub screen)
    - MANUAL → `PARAM_EDIT` (stub screen)
  - Service entry (Req 19):
    - From `IDLE`, key sequence `* * 5` enters `SERVICE`
    - `B` exits back to `IDLE`
  - Invalid keys:
    - Brief `INVALID KEY` message on line 4, then screen restores.
- Optimization:
  - Serial transition logging compiled out by default (see Build Flags).

## `main.cpp` Integration / Scheduler

- Globals created in `src/main.cpp`:
  - `UILCD lcd;`
  - `KeypadInput keypad;`
  - `AppStateMachine app;`
- Cooperative scheduler (millis-based, non-blocking):
  - Keypad scan: 50ms
  - IO update: 100ms
  - App update: 100ms
  - LCD refresh tick: 200ms (≥5 Hz requirement)
  - EEPROM deferred writes: 1000ms

## Build / Debug Optimizations

- Serial debug is disabled by default:
  - `-DENABLE_SERIAL_DEBUG=0`
  - Serial buffers reduced:
    - `-DSERIAL_RX_BUFFER_SIZE=32`
    - `-DSERIAL_TX_BUFFER_SIZE=32`

## Hardware Checkpoint Result (User Reported)

- Flash: 8166 bytes (26.6% of 30720)
- SRAM:  558 bytes (27.2% of 2048)
- EEPROM reserved map: 512 bytes (50.0% of 1024; limit 870 bytes)

