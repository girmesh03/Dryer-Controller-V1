# Phase 1 Implementation Summary (Scaffolding + Safe Initialization)

This document summarizes what was implemented in **Phase 1** and the resulting architectural patterns that Phase 2+ builds on.

> Note: The hardware target is now **ESP32-WROOM-32** (`esp32doit-devkit-v1`). See `docs/esp32-migration.md` for the updated pin map, PlatformIO environment, and ESP32-specific notes. Any Arduino Nano memory-limit discussion below is historical.

## Build System

- **PlatformIO environment:** `esp32` (ESP32-WROOM-32 / DOIT DEVKIT V1).
- **Pinned libraries:** LCD, Keypad, OneWire/DallasTemperature, PID, plus `Wire` + `EEPROM`.
- **Size optimization flags:** `-Os`, `-ffunction-sections`, `-fdata-sections`, link `--gc-sections`.
- **Memory enforcement:** Arduino Nano `check_memory.py` enforcement is **not used** on ESP32.

## Configuration Headers

- `include/config_pins.h`
  - Door input: `GPIO33` (`INPUT_PULLUP`, `LOW=closed`, `HIGH=open`)
  - Temperature sensor: `GPIO32` (OneWire/DS18B20)
  - Relays (**active-LOW**):
    - `GPIO14` heater, `GPIO27` motor fwd, `GPIO26` motor rev
    - `GPIO25` reserved aux relay (future use)
  - Keypad:
    - rows `GPIO16/GPIO17/GPIO18/GPIO19` (internal pull-ups)
    - cols `GPIO13/GPIO4/GPIO5/GPIO15` (includes strapping pins; do not hold keys at reset)
  - Buzzer: `GPIO23` (tone output)
  - LCD I2C: `SDA=GPIO21`, `SCL=GPIO22`

- `include/config_build.h`
  - Firmware version/date/time in **PROGMEM**
  - Scheduler periods (Phase 1): `FAST_LOOP_PERIOD=100ms`, `MEDIUM_LOOP_PERIOD=250ms`, `SLOW_LOOP_PERIOD=1000ms`
  - Safety thresholds and timing constants (debounce, motor deadtime, heater min on/off, watchdog timeout)
  - LCD I2C address constant: `0x27`

## IO Abstraction (Safety Layer)

- `include/io_abstraction.h` + `lib/io_abstraction/io_abstraction.cpp`
- Safe defaults:
  - All relay outputs forced **HIGH** before pinMode OUTPUT (active-LOW OFF state).
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

- Reset cause captured at startup:
  - ESP32 uses `esp_reset_reason()` and maps watchdog resets to the boot screen indicator.
- Watchdog enabled with a **2s** nominal timeout after initialization and kicked in `loop()`.
- Cooperative scheduler uses `millis()` and runs:
  - Fast loop: IO update (100ms in Phase 1; keypad/UI will add faster ticks in Phase 2)
  - Medium loop: reserved for future (Phase 2 LCD refresh)
  - Slow loop: EEPROM deferred writes + heartbeat LED (1 Hz)

## Bench Expectations (Phase 1)

- On power-up, relay outputs remain **OFF** (active-LOW → outputs driven **HIGH** on `PIN_HEATER_RELAY`, `PIN_MOTOR_FWD_RELAY`, `PIN_MOTOR_REV_RELAY`, `PIN_AUX_RELAY`).
- Door changes print to Serial (`DOOR: OPEN/CLOSED`) for basic verification.
- Watchdog reset can be tested by intentionally hanging `loop()` (temporary) and verifying reset.
