# Phase 11 Implementation Summary (Polishing - UI + Cycle Execution + Service Tools)

This document summarizes what was implemented in **Phase 11** and how it integrates with the existing Phase 1–10 codebase on **ESP32 (esp32doit-devkit-v1)**.

## What Was Added / Completed

### Manual Mode complete operator flow (Req 12B)

- Manual mode now provides the full guided UI flow:
  - Temperature input (`40–85°C`, step `5°C`, wrap)
  - Duration input (`10–120 min`, step `5 min`, wrap)
  - Confirmation screen (`A:START B:CANCEL`, `*:EDIT` back to temp input)
- Manual parameters are **temporary only** (not saved to EEPROM).
- Manual drum pattern defaults to **50s FWD / 50s REV / 5s STOP** (Mixed Load baseline).

### Cycle execution screens + logic (Req 12A/12B/10/11/15)

- `START_DELAY` (1s): shows “STARTING…” then enters `RUNNING_HEAT`.
- `RUNNING_HEAT` running screen (1 Hz refresh):
  - Line 1: `AUTO: <PROGRAM>` or `MANUAL MODE`
  - Line 2: `SP:xxx[*] PV:xxx °C/°F` (`*` if SP was modified in-cycle)
  - Line 3: `TIME: mmm:ss[*]` (`*` if duration modified in-cycle)
  - Line 4: `HFR` indicators + `B:PAUSE`
- `RUNNING_COOLDOWN` screen:
  - Heater is forced OFF, drum continues with the active pattern.
  - Exits to anti-crease when `PV < 45°C` **or** after 15 minutes cooldown timeout.
- Cycle completion + anti-crease:
  - On cooldown completion, controller enters `ANTI_CREASE`:
    - First shows a short **cycle complete summary** screen (`PRESS ANY KEY` to exit to IDLE).
    - Then shows **ANTI-CREASE ACTIVE** and periodically tumbles the drum.

### Pause / resume (Req 26)

- `B/STOP` during `RUNNING_HEAT` or `RUNNING_COOLDOWN` pauses the cycle (`PAUSED` state):
  - Heater OFF, drum OFF
  - `A/START` resumes to the previous running state
  - `B/STOP` aborts to main menu (IDLE)
  - Automatic abort after **30 minutes paused** with a short “PAUSE TIMEOUT” message

### In-cycle parameter modification (Req 13)

- `*` during `RUNNING_HEAT` opens in-cycle edit:
  - Temperature change limited to `±10°C` from the original cycle setpoint (clamped to global min/max).
  - Duration change limited to `±20 min` from the original cycle duration (clamped to 10–120).
  - `OK` applies changes:
    - Setpoint is applied via `pidController.setSetpoint()` (PID module ramps at `2°C/s`).
    - Duration updates immediately (remaining time recalculates from new duration).
  - Changes are **temporary** (not saved to EEPROM program table).

### Anti-crease mode (Req 11)

- Anti-crease behavior:
  - Heater stays OFF.
  - Periodic tumbling: **10s every 5 minutes**.
  - Exit conditions:
    - Door opens → stop and return to IDLE
    - `B/STOP` → stop and return to IDLE
    - Automatic return to IDLE after **2 hours**

### Service tools additions (Req 19)

- **I/O TEST** (Service menu):
  - Output page: manually toggle `HEATER`, `MOTOR FWD`, `MOTOR REV`, `AUX RELAY (reserved)`
  - Sensor page: shows `TEMP`, `RAW`, and `DOOR` state
  - Safety:
    - Door must be closed to enable any output
    - Motor forward/reverse are mutually exclusive
    - Door opening forces outputs OFF and clears the latched IO test output mask (no auto-restart)
  - Integration:
    - Motor/AUX relays are overridden in `src/main.cpp` while IO test is active.
    - Heater is still driven through `heaterControl` (anti-chatter + safety gating).
- **MEMORY INFO** (Service menu):
  - Page 0: displays:
    - Flash usage against the **running app partition size** (matches PlatformIO “Project Inspect” totals)
    - Heap usage (`ESP.getHeapSize()` / `ESP.getFreeHeap()`)
    - EEPROM reserved-map usage (512B map vs 1KB device total)
    - Each line shows a `%` and a `!` warning indicator if `>85%`
  - Page 1: firmware version + build date/time (`FW_VERSION`, `__DATE__`, `__TIME__`).
  - Rendering avoids repeated `lcd.clear()` calls (prevents visible flicker).

## Key Integration Notes

- `src/main.cpp` remains the “scheduler”:
  - FAST loop (100ms): `faultMgr.update()` + `app.update()`
  - Drum outputs: normal `drumControl` direction OR IO test override outputs
  - Heater: PID computes only in `RUNNING_HEAT`, drives `heaterControl`, then `io.setHeaterRelay(heaterControl.isHeaterOn())`
- Door-open fault gating was refined so `ANTI_CREASE` exits to IDLE without latching a fault (the App handles it), while operational states still latch door faults.

## LCD Rendering Reliability Fix

- Multiple LCD “missing first character” issues were traced to **line padding off-by-one** that caused a wrap-around write into column `(0,0)`.
- Phase 11 corrected those padding counts on affected screens (e.g., Program Editor list and Fault History view) to prevent unintended wrapping.

## Hardware Checkpoint (User Action)

Build and upload:
- `pio run -e esp32`
- `pio run -e esp32 -t upload`

Verify:
- Manual mode entry screens → start cycle → running screen updates every second
- Pause/resume (`B` to pause, `A` to resume, `B` to abort) + 30-min timeout
- Cooldown begins after heat duration ends and transitions to anti-crease at `PV < 45°C` or after 15 minutes
- Service menu (`**5`) → I/O TEST toggles relays (door interlock enforced) and MEMORY INFO pages display
