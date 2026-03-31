# Fault Code Reference

Fault handling is implemented by `FaultManager` (`include/faults.h`, `lib/faults/faults.cpp`) and displayed by the App’s `FAULT` state.

## Fault Screen (UI)

When a fault is latched, the LCD shows:

- Line 1: `** FAULT **`
- Line 2: fault message (see table below)
- Line 3: `CLOSE TO CONTINUE`
- Line 4: `PRESS B TO CLEAR`

Clear behavior:
- Press `B` (STOP key) to clear only if the recovery conditions are satisfied (`faultMgr.canClearFault()`).

## Fault Codes (EEPROM + UI)

| Code | Name | LCD Message (Line 2) | Meaning | Recovery |
|---:|---|---|---|---|
| 0 | `NONE` | `NO FAULT` | No fault latched | — |
| 1 | `DOOR_OPEN` | `DOOR OPEN` | Door opened while in an operational state | Close door, press `B` |
| 2 | `TEMP_SENSOR_FAULT` | `TEMP SENSOR FAULT` | DS18B20 invalid for ≥2s (disconnect/CRC/read faults) | Reconnect sensor, verify reading returns, press `B` |
| 3 | `OVER_TEMP` | `OVER-TEMP FAULT` | Temperature ≥ 95°C | Cool below 50°C, press `B` |
| 4 | `THERMAL_RUNAWAY` | `THERMAL RUNAWAY` | Heater ON and temperature rises >15°C in 30s | Cool below 50°C, inspect heater control/sensor placement, press `B` |
| 5 | `HEATING_TIMEOUT` | `HEATING TIMEOUT` | Heating for 90 minutes without reaching setpoint | Inspect heater, sensor, airflow, insulation; press `B` |
| 6 | `OUTPUT_FAULT` | `OUTPUT FAULT` | Output mismatch counter tripped | Check wiring/relay board; close door; press `B` |
| 7 | `WATCHDOG_RESET` | `WATCHDOG RESET` | Previous reboot was caused by watchdog | Acknowledge: press `B` (also logged) |
| 8 | `BROWNOUT` | `POWER FAULT RESET` | Previous reboot caused by brownout/power fault | Acknowledge: press `B` (also logged) |
| 9 | `SELF_TEST_FAIL` | `SELF-TEST FAIL` | Reserved for future self-test failures | Service required |

## Fault History Entry Format

Fault history is stored by `EEPROMStore` as a ring buffer (10 entries), each entry is 20 bytes:

- `code` (1 byte)
- `timestamp_s` (4 bytes, seconds since boot)
- `temperature_c` (4 bytes, float °C)
- `reserved` (11 bytes)

The UI shows entries most-recent-first.

