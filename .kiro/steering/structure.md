# Project Structure

## Root Directory Layout

```
/
├── .pio/                    # PlatformIO build artifacts (generated)
├── .kiro/                   # Kiro IDE configuration and specs
│   ├── specs/              # Feature specifications
│   │   └── imesa-dryer-controller/
│   │       ├── requirements.md  # Detailed requirements (21 requirements)
│   │       ├── design.md        # Architecture and design decisions
│   │       └── tasks.md         # Implementation task breakdown
│   └── steering/           # Project steering documents
│       ├── product.md      # Product overview and features
│       ├── tech.md         # Technology stack and conventions
│       └── structure.md    # This file
├── include/                 # Header files for the project
│   └── config.h            # Pin definitions, constants, version info
├── lib/                     # Project-specific libraries
├── src/                     # Main source code
│   ├── main.cpp            # Application entry point
│   ├── hal/                # Hardware Abstraction Layer
│   │   ├── sensor_hal.h/cpp       # Temperature & door sensor
│   │   ├── output_hal.h/cpp       # Relay control
│   │   ├── display_hal.h/cpp      # LCD interface
│   │   └── keypad_hal.h/cpp       # Keypad scanning
│   ├── control/            # Control Layer
│   │   ├── pid_controller.h/cpp   # PID implementation
│   │   ├── reversing_pattern.h/cpp # Drum direction control
│   │   └── safety_interlock.h/cpp # Safety monitoring
│   ├── app/                # Application Layer
│   │   ├── state_machine.h/cpp    # Cycle state machine
│   │   ├── menu_system.h/cpp      # Menu navigation
│   │   ├── service_mode.h/cpp     # Service functions
│   │   └── program_manager.h/cpp  # Program storage
│   ├── storage/            # Persistence Layer
│   │   ├── eeprom_manager.h/cpp   # EEPROM operations
│   │   ├── fault_logger.h/cpp     # Fault logging
│   │   └── statistics.h/cpp       # Usage tracking
│   └── util/               # Utilities
│       ├── timing.h/cpp           # Non-blocking timers
│       ├── filter.h/cpp           # Signal filtering
│       └── types.h                # Common types and enums
├── test/                    # Unit tests
│   ├── test_sensor_hal/
│   ├── test_output_hal/
│   ├── test_pid_controller/
│   ├── test_state_machine/
│   ├── test_eeprom/
│   └── test_integration/
├── platformio.ini          # PlatformIO configuration
└── .gitignore              # Git ignore rules
```

## Layered Architecture

The system follows a strict layered architecture:

```
┌─────────────────────────────────────────────────────────┐
│                   Application Layer                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ Cycle State  │  │ Menu System  │  │ Service Mode │  │
│  │   Machine    │  │              │  │              │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│                   Control Layer                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ PID          │  │ Reversing    │  │ Safety       │  │
│  │ Controller   │  │ Pattern      │  │ Interlock    │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│              Hardware Abstraction Layer                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ Sensor HAL   │  │ Output HAL   │  │ HMI HAL      │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│                   Hardware Layer                         │
│  DS18B20 │ Door Sensor │ Relays │ LCD │ Keypad │ EEPROM │
└─────────────────────────────────────────────────────────┘
```

## Source Organization

### src/main.cpp

Entry point with `setup()` and `loop()` functions. Responsibilities:

- System initialization (watchdog, HAL, control, application layers)
- Main control loop execution (100ms cycle time)
- Module coordination
- Watchdog management

Keep this file minimal - delegate all logic to appropriate modules.

### include/config.h

Central configuration file containing:

- Pin definitions (all hardware pins)
- Timing constants (loop period, update intervals)
- Safety limits (temperature ranges, timeouts)
- Default PID parameters
- EEPROM memory map addresses
- Firmware version and build date

### src/hal/ - Hardware Abstraction Layer

Isolates all hardware-specific code. Each module provides an interface for testing with mocks.

- **sensor_hal**: DS18B20 temperature sensor (OneWire, non-blocking reads, moving average filter), door sensor (debounced digital input)
- **output_hal**: Relay control with anti-chatter logic, mutual exclusion enforcement, emergency stop
- **display_hal**: LCD I2C interface with dirty line tracking to minimize I2C traffic
- **keypad_hal**: 4x4 matrix keypad scanning with debouncing and key mapping

### src/control/ - Control Layer

Implements control algorithms and safety logic.

- **pid_controller**: PID temperature control with anti-windup, derivative-on-measurement, relay output conversion with hysteresis
- **reversing_pattern**: Drum direction control state machine (forward/pause/reverse/pause), enforces minimum 2-second pause
- **safety_interlock**: Continuous safety monitoring, fault detection and latching, emergency stop triggering

### src/app/ - Application Layer

High-level application logic and user interaction.

- **state_machine**: Cycle execution FSM (IDLE, STARTING, HEATING, DRYING, COOLDOWN, COMPLETE, PAUSED, FAULT)
- **menu_system**: Menu navigation, display layouts, user input handling, timeout management
- **service_mode**: Code-protected access, diagnostic tests, parameter editing, fault log viewing
- **program_manager**: 6 default programs (PROGMEM), EEPROM override support, parameter validation

### src/storage/ - Persistence Layer

EEPROM data management with integrity checking.

- **eeprom_manager**: CRC-validated read/write, memory map management, factory reset
- **fault_logger**: Circular buffer fault log (20 entries), wear leveling
- **statistics**: Usage tracking (cycles, hours, relay activations)

### src/util/ - Utilities

Common utility functions and data structures.

- **timing**: Non-blocking timer utilities using `millis()`
- **filter**: Moving average filter template for sensor data
- **types**: Common enumerations (CycleState, FaultCode, Key, MenuContext)

## Test Organization

### test/

Unit tests organized by module, plus integration tests.

```
test/
├── test_sensor_hal/
│   ├── test_temperature_filtering.cpp
│   ├── test_door_debouncing.cpp
│   └── test_sensor_validation.cpp
├── test_output_hal/
│   ├── test_relay_control.cpp
│   ├── test_anti_chatter.cpp
│   └── test_mutual_exclusion.cpp
├── test_pid_controller/
│   ├── test_pid_computation.cpp
│   ├── test_anti_windup.cpp
│   └── test_auto_tune.cpp
├── test_state_machine/
│   ├── test_state_transitions.cpp
│   ├── test_cycle_execution.cpp
│   └── test_fault_handling.cpp
├── test_eeprom/
│   ├── test_checksum.cpp
│   ├── test_program_storage.cpp
│   └── test_fault_log.cpp
└── test_integration/
    ├── test_complete_cycle.cpp
    ├── test_safety_interlocks.cpp
    └── test_menu_navigation.cpp
```

### Testing Strategy

- **Unit Tests**: Specific examples, edge cases, error conditions
- **Property Tests**: Universal properties with randomized inputs (RapidCheck)
- **Integration Tests**: End-to-end scenarios with multiple modules
- **Hardware-in-Loop**: Validation with actual sensors and relays

## Build Artifacts

### .pio/

Generated by PlatformIO during build. Contains:

- Compiled object files
- Linked firmware binaries (.elf, .hex)
- Dependency downloads
- Build cache

**Do not commit** `.pio/` to version control.

## Configuration Files

### platformio.ini

Build configuration with two environments:

- **nano**: Arduino Nano target for deployment
- **native**: Native x86 target for unit testing

Includes library dependencies with version pinning.

### .gitignore

Excludes:

- `.pio/` build artifacts
- `.vscode/` IDE settings (optional)
- Compiled binaries

## Specs Directory

`.kiro/specs/imesa-dryer-controller/` contains the complete specification:

- **requirements.md**: 21 detailed requirements with acceptance criteria
- **design.md**: Architecture, data models, algorithms, testing strategy
- **tasks.md**: 24 implementation tasks with checkpoints

## Code Organization Guidelines

### Module Design

- Each module has a clear, single responsibility
- Use interfaces (abstract base classes) for testability
- Implement dependency injection for loose coupling
- Keep modules independent and reusable

### File Naming

- Use snake_case for filenames: `sensor_hal.h`, `pid_controller.cpp`
- Match header and implementation files: `module.h` + `module.cpp`
- Use descriptive names that reflect purpose

### Header Files

- Always use header guards: `#ifndef MODULE_H` / `#define MODULE_H` / `#endif`
- Include only what's necessary (minimize dependencies)
- Declare interfaces before implementations
- Document public APIs with comments

### Implementation Files

- Include corresponding header first
- Group includes: system, libraries, project
- Keep functions focused and small
- Use helper functions for complex logic

### Memory Considerations

- Prefer stack allocation over heap
- Use PROGMEM for constant data (strings, default programs)
- Use bit fields for boolean flags
- Monitor SRAM usage (target: <1.5KB)

### Safety-Critical Code

- Validate all inputs
- Check all return values
- Use defensive programming
- Document safety assumptions
- Test exhaustively

### Testability

- Separate hardware access from logic
- Use dependency injection
- Avoid global state where possible
- Write tests alongside implementation
