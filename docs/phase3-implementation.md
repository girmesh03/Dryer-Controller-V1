# Phase 3 Implementation Summary (DS18B20 + Filtering + Fault Detection)

This document summarizes what was implemented in **Phase 3** and the patterns Phase 4+ builds on.

## What Was Added

### `ds18b20_sensor` module

- Files:
  - `include/ds18b20_sensor.h`
  - `lib/ds18b20_sensor/ds18b20_sensor.cpp`
- Library usage:
  - `OneWire` on `PIN_TEMP_SENSOR` (D12)
  - `DallasTemperature` for DS18B20 access
- Non-blocking read architecture:
  - Internal FSM states: `IDLE → REQUEST_CONVERSION → WAIT_CONVERSION → READ_DATA`
  - Conversions are requested with `setWaitForConversion(false)` (no blocking wait).
  - The WAIT state exits when conversion completes or when the computed wait time elapses.
- Sampling-rate decision (Requirement 2):
  - DS18B20 resolution is set to **11-bit** so conversion time supports **≥2 Hz** effective sampling when called from the 250ms medium tick.
  - A **750ms** upper bound is retained as a safety fallback.

### Filtering pipeline (Requirement 2)

Applied after the raw buffer has 5 samples:
1. Median filter (window size 5)
2. Plausibility check: -10°C to 150°C
3. Rate-of-change rejection: >5°C/s (uses time delta between valid samples)
4. EMA smoothing: α = 0.2

### Sensor fault detection (Requirement 2)

- `DEVICE_DISCONNECTED_C` read sentinel counts as a read/CRC failure event.
- Sensor becomes invalid after **3 consecutive failures** (`fault_code=1`).
- Sensor disconnect is detected within **2 seconds** without valid readings (`fault_code=2`).
- Repeated implausible/ROC rejections can trip invalid (`fault_code=3`).
- On valid readings, the fault counter resets and `fault_code` returns to 0.

## UI Integration (Temperature in IDLE)

- `UILCD` gained `showTemperature(float temp_c, bool valid)` and renders on **line 4** (row 3):
  - Valid: `TEMP: XX.X°C`
  - Invalid: `SENSOR FAULT`
- `AppStateMachine` refreshes the temperature line every **1 second** while in `IDLE`.

### Display formatting optimization

Temperature formatting is done with a small fixed-point style render (deci-degrees) to avoid pulling in large `printf` float formatting.

## `main.cpp` Integration / Scheduler

- Globals:
  - `DS18B20Sensor tempSensor;`
- Medium tick (250ms):
  - `tempSensor.update()` runs the non-blocking FSM.
- IDLE UI refresh (1s):
  - `lcd.showTemperature(tempSensor.getTemperature(), tempSensor.isValid())`

## Hardware Checkpoint (User Action)

Run and report:
- `pio run -e nanoatmega328`
- `pio run -e nanoatmega328 -t upload`

Verify on bench:
- Temperature appears on the main menu line 4 and updates every second.
- Unplug DS18B20: `SENSOR FAULT` appears within ~2 seconds.
- Reconnect DS18B20: temperature recovers and resumes updating.

