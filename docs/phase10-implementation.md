# Phase 10 Implementation Summary (Fault System + History + Diagnostics)

This document summarizes what was implemented in **Phase 10** and how it integrates with the existing Phase 1–9 codebase on **ESP32**.

## What Was Added

### FaultManager module (latched faults + detection)

- Added `faults` module:
  - `include/faults.h`
  - `lib/faults/faults.cpp`
- Fault codes (`FaultCode`):
  - `DOOR_OPEN`, `TEMP_SENSOR_FAULT`, `OVER_TEMP`, `THERMAL_RUNAWAY`, `HEATING_TIMEOUT`, `OUTPUT_FAULT`,
    `WATCHDOG_RESET`, `BROWNOUT`, `SELF_TEST_FAIL`
- Fault behavior:
  - **Latched** on first fault (`setFault()` ignores subsequent faults until cleared)
  - `getFaultMessage()` returns a Flash/PROGMEM-stored string (LCD-friendly, ≤20 chars)
  - `canClearFault()` enforces recovery conditions:
    - Door fault: door must be closed
    - Sensor fault: sensor must be valid
    - Thermal faults: sensor valid AND temperature < **50°C** (Req 17.9)
    - WDT/Brownout/Timeout: acknowledge-only (clear allowed)
- Fault detection in `FaultManager::update()` (10 Hz):
  - **Door open**: immediate `DOOR_OPEN` (Req 1 / Phase 10.2)
  - **Temp sensor invalid**: after **2s** continuous invalid → `TEMP_SENSOR_FAULT`
  - **Over-temp**: `>= 95°C` → `OVER_TEMP`
  - **Thermal runaway**: heater ON + `>15°C rise over 30s` → `THERMAL_RUNAWAY`
  - **Heating timeout**: `RUNNING_HEAT` active `>90 min` without reaching setpoint → `HEATING_TIMEOUT`
  - **Output fault**: output mismatch trip at **3** consecutive mismatches (`io.getOutputMismatchCount()`)
- Fault history logging:
  - `FaultManager::setFault()` calls `eepromStore.logFault(code, timestamp_s, temp_c)` (Req 18)
  - Startup note: the DS18B20 driver marks the sensor **valid after the first plausible sample** (even before the 5-sample median/EMA window is full) to avoid a false `TEMP_SENSOR_FAULT` during the first seconds after boot.

### EEPROM fault history ring buffer (10 entries)

- Extended `EEPROMStore` with Phase 10 fault history support:
  - `EEPROMStore::FaultEntry` is 20 bytes (packed)
  - 10-entry ring buffer stored at:
    - Entries: `0x120` (200 bytes)
    - Head index: `0x1EC` (1 byte)
- New APIs:
  - `EEPROMStore::logFault(...)`
  - `EEPROMStore::getFaultHistory(index, entry)` (`index=0` is most recent)
  - `EEPROMStore::clearFaultHistory()`
- ESP32 note:
  - Fault logs are written **immediately** and followed by CRC update + `EEPROM.commit()`

## Integration Updates

### main.cpp: safety shutdown + FAULT transition

- Added global `FaultManager faultMgr` and integrated it into the **FAST loop (100ms)**:
  - `faultMgr.update()`
  - If `faultMgr.hasFault()`:
    - `io.emergencyStop()` forces all outputs OFF immediately
    - `app.transitionTo(SystemState::FAULT)` enters fault UI/state handling

### App FAULT screen + operator acknowledgment

- FAULT UI is now driven by the real `faultMgr`:
  - Line 1: `** FAULT **`
  - Line 2: `faultMgr.getFaultMessage()`
  - Line 3: `CLOSE TO CONTINUE`
  - Line 4: `PRESS B TO CLEAR`
- Clearing:
  - Press `B/STOP` to clear **only if** `faultMgr.canClearFault()` is true
  - Otherwise, a brief `CONDITIONS NOT OK` message is shown and the FAULT screen is restored
- On FAULT entry:
  - `drumControl.stop()`, `heaterControl.disable()`, and `io.emergencyStop()` are asserted for safety redundancy

### Service menu: Fault History view

- Added a new service tool: **FAULT HISTORY**
  - Scroll with `UP/DOWN`
  - Clear history with `C` → confirmation (`A:YES / B:NO`)
  - Exit/back with `B/STOP`

### Reset cause logging + boot indication (ESP32-aware)

- Reset cause is captured on boot and mapped into flags:
  - ESP32 uses `esp_reset_reason()`
  - AVR uses `MCUSR` (kept for legacy compatibility)
- If last reset was Watchdog or Brownout:
  - Acknowledge-only fault is logged (`WATCHDOG_RESET` or `BROWNOUT`)
  - Boot screen shows `LAST RESET: WDT` or `LAST RESET: BROWNOUT`

## Hardware Checkpoint (User Action)

Build and upload:
- `pio run -e esp32`
- `pio run -e esp32 -t upload`

Verify:
- Open door → outputs OFF and FAULT screen shown, then close door + press `B` to clear
- Disconnect DS18B20 → `TEMP SENSOR FAULT` after ~2s, heater forced OFF
- Force temperature >95°C → `OVER-TEMP FAULT` (use external heat source carefully)
- Service menu → `FAULT HISTORY` shows logged events; `C` clears history with confirmation
