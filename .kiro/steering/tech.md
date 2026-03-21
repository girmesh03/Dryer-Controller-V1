# Technology Stack

## Platform

- **Hardware**: Arduino Nano (ATmega328P microcontroller)
- **Clock**: 16MHz
- **Memory**: 32KB Flash, 2KB SRAM, 1KB EEPROM
- **Logic Level**: 5V
- **Framework**: Arduino
- **Memory Budget**: Target <28KB flash, <1.5KB SRAM (leaving ~7% flash margin, ~24% SRAM margin)

## Build System

**PlatformIO** is used for building, uploading, and managing dependencies.

### Configuration

- Platform: `atmelavr`
- Board: `nanoatmega328`
- Framework: `arduino`

### Common Commands

```bash
# Build the project
pio run

# Upload to device
pio run --target upload

# Clean build files
pio run --target clean

# Open serial monitor
pio device monitor

# Build and upload
pio run --target upload && pio device monitor

# Run unit tests (native)
pio test -e native

# Run property tests
pio test -e native --filter "test_properties"

# Check memory usage
pio run -e nano --target size
```

## Key Libraries & Dependencies

### Required Libraries (platformio.ini)

- **OneWire** (^2.3.7): DS18B20 temperature sensor communication (1-Wire protocol)
- **DallasTemperature** (^3.11.0): DS18B20 sensor library with non-blocking reads
- **LiquidCrystal_I2C** (^1.1.2): 20x4 LCD display via I2C (PCF8574 backpack)
- **Keypad** (^3.1.1): 4x4 matrix keypad scanning and debouncing
- **EEPROM** (built-in): Non-volatile storage for programs, PID params, fault log, statistics

### Optional Testing Libraries

- **RapidCheck**: Property-based testing framework (for native tests only)

## Hardware Interfaces

### Digital I/O

- Pin 2-5: Keypad columns (OUTPUT)
- Pin 6: Door sensor (INPUT_PULLUP, active LOW when open)
- Pin 9: Heater relay (OUTPUT, HIGH = heater on)
- Pin 10: Motor forward relay (OUTPUT, HIGH = forward rotation)
- Pin 11: Motor reverse relay (OUTPUT, HIGH = reverse rotation)
- Pin 12: DS18B20 temperature sensor (OneWire bidirectional)

### Analog I/O

- A0-A3: Keypad rows (INPUT)
- A4: I2C SDA (LCD communication)
- A5: I2C SCL (LCD communication)

### I2C Devices

- LCD: Address 0x27 (configurable, typical for PCF8574)

### Keypad Layout

```
Row 1: '7'(↑↑) '8'(UP)  '9'(↑↑) '/'(START)
Row 2: '4'(LT) '5'(OK)  '6'(RT) '*'(STOP)
Row 3: '1'(↓↓) '2'(DN)  '3'(↓↓) '-'
Row 4: 'C'(CANCEL) '0' '='      '+'
```

## Timing Constraints

### Main Control Loop

- **Loop Period**: 100ms (enforced, non-blocking)
- **Critical Path**: ~15ms per loop (15% CPU utilization)
- **Watchdog Timeout**: 2 seconds (must reset every loop)

### Task Scheduling

| Task                     | Frequency | Max Duration | Priority |
| ------------------------ | --------- | ------------ | -------- |
| Door sensor read         | 100ms     | 1ms          | Critical |
| Safety interlock check   | 100ms     | 2ms          | Critical |
| Relay output update      | 100ms     | 1ms          | Critical |
| Temperature read         | 1000ms    | 50ms         | High     |
| PID calculation          | 1000ms    | 5ms          | High     |
| Reversing pattern update | 100ms     | 2ms          | High     |
| State machine update     | 100ms     | 5ms          | Medium   |
| Keypad scan              | 100ms     | 5ms          | Medium   |
| Display update           | 500ms     | 20ms         | Low      |
| EEPROM write             | On-demand | 100ms        | Low      |

### Response Time Requirements

- Door open → outputs off: <100ms
- Stop key → pause state: <200ms
- Temperature sensor read: <1s (non-blocking, 750ms conversion)
- Display update: <500ms

## EEPROM Memory Map

```
Address Range | Size  | Content
--------------|-------|----------------------------------
0x0000-0x0003 | 4     | Magic number (0xDEADBEEF)
0x0004-0x0005 | 2     | Format version (0x0001)
0x0006-0x0007 | 2     | Header CRC16
0x0008-0x000B | 4     | Service code (default: 1234)
0x000C-0x001F | 20    | PID parameters + CRC8
0x0020-0x00FF | 224   | Program overrides (6 × 37 bytes)
0x0100-0x01FF | 256   | Fault log (circular buffer, 20 entries)
0x0200-0x023F | 64    | Statistics + CRC16
0x0240-0x03FF | 448   | Reserved for future use
```

## Code Conventions

### General

- Use Arduino framework functions (`digitalWrite`, `digitalRead`, `millis`, etc.)
- Implement non-blocking timing using `millis()` instead of `delay()`
- Use state machines for cycle control (explicit FSM pattern)
- Apply debouncing to all digital inputs (minimum 50ms stable time)
- Enforce safety interlocks in all operational modes
- Reset watchdog timer every loop iteration

### Memory Management

- Store all constant strings in PROGMEM using `F()` macro
- Store default programs in PROGMEM
- Use bit fields for boolean flags to save SRAM
- Avoid dynamic memory allocation (no `new`, `malloc`, or `String` class)
- Use static memory allocation only
- Monitor free SRAM (target: keep >200 bytes free)

### Safety-Critical Code

- All safety checks must complete within timing budget
- Emergency stop must execute within 100ms
- Use hardware abstraction layer for all I/O
- Validate all EEPROM data with CRC checksums
- Implement graceful degradation for non-critical faults

### Testing

- Write unit tests for all modules
- Write property-based tests for safety-critical properties
- Tag property tests with: `// Feature: imesa-dryer-controller, Property N: [description]`
- Minimum 100 iterations per property test
- Test with mock HAL implementations for portability

### Code Organization

- Use header guards in all `.h` files
- Group related functionality into modules (hal/, control/, app/, storage/, util/)
- Separate hardware-specific code from business logic
- Use dependency injection for testability
- Keep `main.cpp` minimal (setup, loop, module coordination only)
