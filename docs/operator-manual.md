# Operator Manual + Quick Reference

This manual describes day-to-day operation of the Industrial Dryer Controller firmware on **ESP32**.

## Keypad Quick Reference

Physical keypad mapping (4x4):

| Key | Meaning | Used for |
|---|---|---|
| `4` | UP | Navigate lists / increase values |
| `6` | DOWN | Navigate lists / decrease values |
| `8` | LEFT | Move field selection |
| `2` | RIGHT | Move field selection |
| `5` | OK | Select / confirm |
| `A` | START | Start cycle / resume |
| `B` | STOP | Pause / back / abort / clear faults |
| `*` | EDIT | Enter edit screens (auto review/edit, in-cycle edit) |
| `C` | CANCEL | Tool-specific cancel/confirm |
| `D` | MENU | Reserved (future) |

Service and settings access:
- Service menu: `* * 5`
- Settings menu: `* * 8`

## Main Menu

Screen:
- `MAIN MENU`
  - `> AUTO`
  - `  MANUAL`

Use `UP/DOWN` to choose, `OK` to enter.

## Auto Mode (Program-Based)

1) Enter AUTO from main menu.
2) **Program select**:
   - Use `UP/DOWN` to choose one of 6 programs (loaded from EEPROM).
   - `OK` to select.
3) **Parameter review**:
   - Shows program temperature and duration.
   - `A/START` begins the cycle.
   - `*/EDIT` enters temporary parameter edit (does **not** overwrite the program in EEPROM).
   - `B/STOP` goes back.
4) **Temporary parameter edit** (optional):
   - Edit TEMP and TIME.
   - `OK` saves the temporary edits for this run.
   - `B/STOP` cancels.

## Manual Mode (Operator-Entered)

1) Enter MANUAL from main menu.
2) Set temperature:
   - Range `40–85°C` in `5°C` steps (wraps).
   - `UP/DOWN` changes, `OK` next.
3) Set duration:
   - Range `10–120 min` in `5 min` steps (wraps).
   - `UP/DOWN` changes, `OK` next.
4) Confirm screen:
   - `A/START` begins.
   - `B/STOP` cancels back to main menu.
   - `*/EDIT` returns to temperature input.

Manual parameters are **temporary** and are not saved to EEPROM.

## During Cycle

### RUNNING_HEAT (Heating phase)

The running screen shows:
- Line 1: `AUTO: <PROGRAM>` or `MANUAL MODE`
- Line 2: `SP:xxx PV:xxx °C/°F` (`SP` = setpoint, `PV` = measured)
- Line 3: `TIME: mmm:ss` remaining
- Line 4: status flags (heater/motor) and pause hint

Pause:
- Press `B/STOP` to pause.

In-cycle edit (heating only):
- Press `*` to edit TEMP/TIME while running.
- TEMP is limited to `±10°C` of the original setpoint.
- TIME is limited to `±20 min` of the original duration.
- `OK` applies changes (temporary).
- `*` cancels edit without applying.

### RUNNING_COOLDOWN (Cooldown phase)

- Heater is OFF.
- Drum continues with the active pattern.
- Cooldown exits when `PV < 45°C` or after a 15-minute cooldown timeout.

### Cycle Complete + Anti-Crease

After the cycle finishes:
- A short **cycle complete summary** screen is shown.
  - Press any key during this summary to return to main menu (IDLE).
- If no key is pressed, the controller enters **ANTI-CREASE**:
  - Drum tumbles **10s every 5 minutes**, up to 2 hours.
  - Exit anti-crease by opening the door or pressing `B/STOP`.

## Fault Handling (Safety)

If a fault is detected:
- Outputs are forced OFF.
- LCD shows the FAULT screen.

To clear:
- Fix the condition (close door, reconnect sensor, allow cooling, etc.).
- Press `B/STOP` to clear when allowed.

See `docs/fault-code-reference.md` for meanings and recovery steps.

## Service Menu (Commissioning / Diagnostics)

Enter: `* * 5`

Service tools include:
- DRUM TEST
- HEATER TEST
- PID VIEW
- AUTO TUNE
- FOPDT ID
- I/O TEST
- FAULT HISTORY
- MEMORY INFO
- FACTORY RESET

## Settings Menu

Enter: `* * 8`

Settings:
- TEMP UNITS (°C/°F)
- SOUND (enable/disable key beep)
- PROGRAM EDITOR (edit program table stored in EEPROM)

