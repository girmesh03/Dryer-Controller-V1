# Product Overview

This is an Arduino Nano-based industrial tumble dryer controller that reverse-engineers the IMESA ES Series Industrial Tumble Dryer. The system is designed for commercial laundry environments and implements safety-critical machine control with <100ms emergency response time.

## Core Functionality

- Menu-driven HMI with 20x4 I2C LCD and 4x4 matrix keypad
- Closed-loop PID temperature regulation using DS18B20 OneWire sensor
- Drum motor control with configurable reversing patterns (forward/reverse/pause)
- 6 predefined drying programs: Towels, Bed Sheets, Mixed Load, Heavy Cotton, Delicate, Work Wear
- Manual mode for testing and custom cycles (limited to 90°C)
- Industrial-grade fault handling with latching and logging
- Service mode (code-protected) for diagnostics and configuration
- PID auto-tune using relay feedback test (Ziegler-Nichols)

## Cycle Phases

1. STARTING - Pre-start safety validation
2. HEATING - Reach target temperature (PID control active)
3. DRYING - Maintain temperature for configured duration
4. COOLDOWN - Heater off, drum continues rotating
5. COMPLETE - All outputs off, ready for unload

## Safety Features

- Door safety interlock with <100ms response (immediate shutdown when door opens)
- Over-temperature protection (100°C limit, fault at 100°C, safe below 80°C)
- Temperature sensor fault detection (3 consecutive invalid readings)
- Motor relay mutual exclusion (forward and reverse never both active)
- Minimum 2-second pause enforced between motor direction changes
- Heater anti-chatter (5-second minimum interval between state changes)
- Motor anti-chatter (2-second minimum interval between state changes)
- Watchdog timer protection (2-second timeout)
- Comprehensive fault logging to EEPROM with CRC validation

## Fault Codes

- DOOR_OPEN - Door opened during operation (auto-recovers when closed)
- TEMP_SENSOR_FAIL - DS18B20 communication failure (3 consecutive failures)
- OVER_TEMPERATURE - Temperature exceeds 100°C (requires cooldown to 80°C)
- LCD_FAIL - I2C LCD initialization failure (requires service)
- RELAY_CONFLICT - Both motor relays active simultaneously (requires service)
- WATCHDOG_RESET - System watchdog timeout (logged, auto-recovers)

## Target Environment

Commercial/industrial laundry facilities requiring reliable, safe, and user-friendly dryer control systems with industrial-grade reliability and deterministic real-time behavior.
