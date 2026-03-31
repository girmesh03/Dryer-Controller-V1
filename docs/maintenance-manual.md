# Maintenance + Service Manual

This manual is intended for technicians performing commissioning, diagnostics, tuning, and maintenance.

## Preconditions for Service Operations

Before running service tools that drive outputs:
- Drum is empty.
- Door is closed (door interlock OK).
- Ambient temperature is within **15–30°C** (for tuning/identification tools).
- Ensure ventilation and fire safety precautions.

## PID Tuning (AutoTune)

AutoTune uses a relay-feedback method (Åström–Hägglund) to produce Ziegler–Nichols gains.

Procedure:
1) Enter Service Menu: `* * 5`
2) Select `AUTO TUNE`
3) Confirm preconditions screen.
4) Press `A/START` to begin.
5) Wait until at least **3 oscillation cycles** are completed.
6) Review results and press `A/ACCEPT` to store gains to EEPROM.
7) Reboot and verify gains persist (PID VIEW).

Abort conditions:
- Door opens → AutoTune aborts and outputs are forced OFF.
- Sensor fault → abort.
- Over-temperature (>95°C) → abort.
- `B/STOP` → abort.

## Process Identification (FOPDT ID)

FOPDT identification measures **K**, **τ**, and **L** from a step test.

Procedure:
1) Enter Service Menu: `* * 5`
2) Select `FOPDT ID`
3) Confirm preconditions.
4) Press `A/START` to begin.
5) Baseline is recorded for ~30 seconds, then the heater step is applied.
6) Wait for completion, review K/τ/L + derived gains.
7) Press `A/ACCEPT` to store results to EEPROM.

## Program Editor (EEPROM Program Table)

Enter Settings Menu: `* * 8` → `PROGRAM EDITOR`

Editable parameters (per program):
- Temperature: `40–85°C`
- Duration: `10–120 min`
- Forward / Reverse: `30–90s`
- Stop: `3–15s`

Changes are persisted to EEPROM using CRC8 validation.

## I/O Test Mode (Outputs + Sensors)

Enter Service Menu: `* * 5` → `I/O TEST`

Capabilities:
- Toggle relays: HEATER, MOTOR FWD, MOTOR REV, AUX (reserved)
- View door state and temperature values

Safety rules enforced by firmware:
- Door must be closed to energize any output.
- Motor forward and reverse cannot be ON simultaneously.
- Direction change enforces a **2-second dead-time** (break-before-make).

Exit: `B/STOP` (outputs forced OFF).

## Fault History

Enter Service Menu: `* * 5` → `FAULT HISTORY`

- Scroll: `UP/DOWN`
- Clear: `C` → confirm (`A` yes / `B` no)
- Back: `B/STOP`

See `docs/fault-code-reference.md` for codes and meanings.

## Factory Reset

Factory reset restores defaults:
- Settings (temp unit, sound, etc.)
- Programs (6 defaults)
- PID gains defaults
- FOPDT defaults
- Fault history cleared

Run from Service Menu: `* * 5` → `FACTORY RESET`

## EEPROM Layout (Reserved Map)

The CRC-protected reserved EEPROM map is 512 bytes:
- `0x000–0x003`: header (magic, version, CRC8)
- `0x004`: settings (20 bytes)
- `0x018`: programs (6 × 40 bytes)
- `0x108`: PID gains (3 floats)
- `0x114`: FOPDT params (3 floats)
- `0x120`: fault history (10 × 20 bytes)

ESP32 note:
- EEPROM is flash-emulated; initialized with `EEPROM.begin(1024)` and flushed with `EEPROM.commit()`.

### EEPROM Corruption Recovery Test (CRC8)

To validate corruption recovery (Phase 12):
1) Temporarily add to `setup()` in `src/main.cpp` (one-time), upload once, then remove:
   - `EEPROM.write(0x003, EEPROM.read(0x003) ^ 0xFF);`
   - `EEPROM.commit();`
2) Reboot. The controller should detect invalid CRC and auto factory-reset:
   - Boot screen shows `EEPROM RESET`
   - Programs / PID gains restored to defaults

## Troubleshooting

- Temperature does not rise:
  - Verify heater relay wiring and relay board logic (active‑LOW).
  - Verify door is closed and no fault is active.
- Temperature oscillates / overshoots:
  - Re-run AutoTune at ambient conditions.
  - Verify sensor placement and airflow.
- Drum does not rotate:
  - Check motor relays and wiring.
  - Verify door sensor state.
- LCD issues:
  - Verify I2C wiring (GPIO21/22) and address `0x27`.
  - Ensure I2C pull-ups are not to 5V (ESP32 is 3.3V logic).

## Firmware Update (PlatformIO)

1) Connect ESP32 via USB.
2) Build: `pio run -e esp32`
3) Upload: `pio run -e esp32 -t upload`
4) Verify boot screen shows FW version and no unexpected faults.

## Reset Cause Tests (Watchdog / Brownout)

- Watchdog reset (WDT):
  - Temporarily add an infinite loop early in `loop()` (before the next watchdog kick) and upload.
  - Confirm the unit resets and the next boot shows `LAST RESET: WDT` and logs a `WATCHDOG RESET` entry.
- Brownout:
  - Briefly interrupt power during operation (controlled test) and confirm `LAST RESET: BROWNOUT` on next boot and a fault history entry.
