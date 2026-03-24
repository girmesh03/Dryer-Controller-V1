# Phase 1 Implementation Summary (Scaffolding + Safe Initialization)

This document summarizes what was implemented in **Phase 1** and the resulting architectural patterns that Phase 2+ builds on.

## Build System

- **PlatformIO environment:** `nanoatmega328` (Arduino Nano ATmega328P).
- **Pinned libraries:** LCD, Keypad, OneWire/DallasTemperature, PID, plus `Wire` + `EEPROM`.
- **Size optimization flags:** `-Os`, `-flto`, `-ffunction-sections`, `-fdata-sections`, link `--gc-sections`, `--relax`, `-mcall-prologues`.
- **Memory enforcement:** `check_memory.py` runs post-link and hard-fails if:
  - Flash > 25500 bytes
  - SRAM > 1740 bytes
  - EEPROM reserved map > 870 bytes (map is currently reserved at 512 bytes)
- **Generated build header (ignored):** `include/generated/build_memory.h` written by `check_memory.py`.

## Configuration Headers

- `include/config_pins.h`
  - Door input: `D6` (`INPUT_PULLUP`, `LOW=open`, `HIGH=closed`)
  - Temperature sensor: `D12` (OneWire/DS18B20 later)
  - Relays: `D9` heater, `D10` motor fwd, `D11` motor rev (**active-HIGH**)
  - Keypad: rows `A0..A3`, cols `D2..D5`
  - Buzzer: `D8` (tone output)

- `include/config_build.h`
  - Firmware version/date/time in **PROGMEM**
  - Scheduler periods (Phase 1): `FAST_LOOP_PERIOD=100ms`, `MEDIUM_LOOP_PERIOD=250ms`, `SLOW_LOOP_PERIOD=1000ms`
  - Safety thresholds and timing constants (debounce, motor deadtime, heater min on/off, watchdog timeout)
  - LCD I2C address constant: `0x27`

## IO Abstraction (Safety Layer)

- `include/io_abstraction.h` + `lib/io_abstraction/io_abstraction.cpp`
- Safe defaults:
  - All relay outputs forced **LOW** before pinMode OUTPUT.
  - Door input configured as `INPUT_PULLUP`.
- Door input debounced at IO layer with minimum stable time.
- Direction interlock + dead-time:
  - Never allows forward+reverse simultaneously.
  - Enforces `MOTOR_DIRECTION_DELAY_MS` break-before-make.
- Door-open safety:
  - Door open forces immediate `emergencyStop()` at IO layer (heater + motor off).
- Output readback:
  - `verifyOutputState()` compares expected vs actual pin readback.
  - Mismatch counter stored for later fault integration.
- Buzzer helpers:
  - `beep()` and `beepKey()` (no UI setting gating yet; added later).

## EEPROM Store (Early Skeleton)

- `include/eeprom_store.h` + `lib/eeprom_store/eeprom_store.cpp`
- Reserved EEPROM map: **512 bytes** (Appendix D placeholder map).
- Header fields: magic/version + CRC8 over reserved region.
- If EEPROM invalid at boot → `factoryReset()` to deterministic defaults then writes header/CRC.
- Deferred write strategy:
  - Minimum write interval: 5 seconds.
  - Supports request/save for PID gains and FOPDT parameters (used in later phases).

## Watchdog + Scheduler (main.cpp)

- Watchdog reset cause captured from `MCUSR` at startup and watchdog disabled before re-enabling to prevent reset loops.
- Watchdog enabled with a **2s** timeout after initialization.
- Cooperative scheduler uses `millis()` and runs:
  - Fast loop: IO update (100ms in Phase 1; keypad/UI will add faster ticks in Phase 2)
  - Medium loop: reserved for future (Phase 2 LCD refresh)
  - Slow loop: EEPROM deferred writes + heartbeat LED (1 Hz)

## Bench Expectations (Phase 1)

- On power-up, D9/D10/D11 remain **LOW**.
- Door changes print to Serial (`DOOR: OPEN/CLOSED`) for basic verification.
- Watchdog reset can be tested by intentionally hanging `loop()` (temporary) and verifying reset.

