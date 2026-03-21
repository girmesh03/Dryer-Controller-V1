# Implementation Plan: IMESA ES Series Industrial Tumble Dryer Controller

## Overview

This implementation plan breaks down the Arduino Nano-based industrial tumble dryer controller into discrete, actionable coding tasks. The controller implements safety-critical machine control with closed-loop temperature regulation, drum motor control with reversing patterns, menu-driven HMI, and industrial-grade fault handling.

The implementation follows a layered architecture approach, building from hardware abstraction through control algorithms to application logic. Each task builds incrementally on previous work, with checkpoints to validate functionality before proceeding.

## Tasks

- [ ] 1. Project setup and core infrastructure

  - Initialize PlatformIO project for Arduino Nano (ATmega328P)
  - Configure platformio.ini with required libraries (OneWire, DallasTemperature, LiquidCrystal_I2C, Keypad)
  - Create directory structure (hal/, control/, app/, storage/, util/)
  - Create config.h with pin definitions and constants
  - Create types.h with common enumerations (CycleState, FaultCode, Key, etc.)
  - Set up basic main.cpp with setup() and loop() skeleton
  - _Requirements: 1.1-1.5, 24.1-24.3, 37.1_

- [ ] 2. Hardware Abstraction Layer - Sensor interfaces

  - [ ] 2.1 Implement door sensor HAL

    - Create sensor_hal.h with ISensorHAL interface
    - Implement door sensor reading with internal pull-up (pin 6)
    - Implement 50ms debouncing for door sensor
    - _Requirements: 2.1-2.4, 2.10, 27.2_

  - [ ] 2.2 Implement temperature sensor HAL

    - Implement DS18B20 OneWire communication (pin 12)
    - Implement non-blocking temperature read with 750ms conversion time
    - Implement moving average filter with minimum 3 samples
    - Implement temperature range validation (-10°C to 150°C)
    - Implement sensor validity checking with 3-strike fault detection
    - _Requirements: 3.1-3.10, 27.5_

  - [ ]\* 2.3 Write unit tests for sensor HAL
    - Test door sensor debouncing with rapid state changes
    - Test temperature filtering with noisy input
    - Test temperature range validation at boundaries
    - Test sensor fault detection after 3 invalid readings
    - _Requirements: 2.10, 3.4, 3.5, 3.7_

- [ ] 3. Hardware Abstraction Layer - Output interfaces

  - [ ] 3.1 Implement relay output HAL

    - Create output_hal.h with IOutputHAL interface
    - Implement heater relay control (pin 9)
    - Implement motor forward relay control (pin 10)
    - Implement motor reverse relay control (pin 11)
    - Implement emergencyStop() function to deactivate all outputs
    - Initialize all outputs to LOW during construction
    - _Requirements: 4.1-4.5, 5.1-5.5, 6.1-6.5_

  - [ ] 3.2 Implement relay anti-chatter logic

    - Enforce 5-second minimum interval for heater relay changes
    - Enforce 2-second minimum interval for motor relay changes
    - Track last change time for each relay
    - _Requirements: 4.8, 29.2, 29.3_

  - [ ] 3.3 Implement motor relay mutual exclusion

    - Prevent simultaneous activation of forward and reverse relays
    - Enforce minimum 2-second pause between direction changes
    - _Requirements: 5.7, 5.8, 6.7, 6.8, 16.10_

  - [ ]\* 3.4 Write property test for motor relay mutual exclusion

    - **Property 6: Motor Relay Mutual Exclusion**
    - **Validates: Requirements 5.7, 6.7**
    - Test that forward and reverse relays are never both active simultaneously
    - Use randomized command sequences to verify mutual exclusion

  - [ ]\* 3.5 Write property test for motor direction change pause
    - **Property 7: Motor Direction Change Pause**
    - **Validates: Requirements 5.8, 6.8, 16.10, 29.3, 36.4**
    - Test that direction changes enforce minimum 2-second pause
    - Use various configured pause times including values < 2 seconds

- [ ] 4. Hardware Abstraction Layer - HMI interfaces

  - [ ] 4.1 Implement LCD display HAL

    - Create display_hal.h with IDisplayHAL interface
    - Initialize 20x4 I2C LCD (address 0x27, pins A4/A5)
    - Implement display buffer with dirty line tracking
    - Implement updateLine() to minimize I2C traffic
    - Implement flush() to write only changed lines
    - _Requirements: 7.1-7.6, 27.3_

  - [ ] 4.2 Implement keypad HAL

    - Create keypad_hal.h with IKeypadHAL interface
    - Configure 4x4 matrix keypad (rows A0-A3, cols D2-D5)
    - Implement key mapping (8=UP, 2=DOWN, 4=LEFT, 6=RIGHT, 5=OK, /=START, \*=STOP, C=CANCEL)
    - Implement 50ms debouncing for keypad
    - Implement non-blocking keypad scanning
    - _Requirements: 8.1-8.16, 27.4_

  - [ ]\* 4.3 Write unit tests for HMI HAL
    - Test LCD dirty line tracking reduces I2C writes
    - Test keypad debouncing filters rapid presses
    - Test keypad mapping for all defined keys
    - _Requirements: 8.13, 27.3, 27.4_

- [ ] 5. Checkpoint - HAL validation

  - Ensure all HAL tests pass
  - Verify sensor readings on actual hardware
  - Verify relay outputs on actual hardware
  - Verify LCD display and keypad input
  - Ask the user if questions arise

- [ ] 6. Control Layer - PID temperature controller

  - [ ] 6.1 Implement PID controller core

    - Create pid_controller.h with IPIDController interface
    - Implement PID computation with proportional, integral, and derivative terms
    - Implement anti-windup protection for integral term
    - Implement derivative-on-measurement to avoid setpoint kick
    - Use default parameters (Kp=2.0, Ki=0.5, Kd=1.0)
    - _Requirements: 14.1-14.9, 35.1_

  - [ ] 6.2 Implement PID relay control

    - Convert PID output (0-100) to relay on/off with hysteresis
    - Use upper threshold (50%) and lower threshold (45%) to prevent chatter
    - Integrate with OutputHAL for heater control
    - _Requirements: 14.6, 14.7, 29.4_

  - [ ]\* 6.3 Write property test for PID anti-windup

    - **Property 10: PID Anti-Windup**
    - **Validates: Requirements 14.8**
    - Test that integral term is clamped with sustained error
    - Verify integral contribution stays within bounds

  - [ ]\* 6.4 Write unit tests for PID controller
    - Test PID computation with known inputs
    - Test anti-windup clamping behavior
    - Test derivative-on-measurement
    - Test relay control hysteresis
    - _Requirements: 14.1-14.9_

- [ ] 7. Control Layer - Reversing pattern controller

  - [ ] 7.1 Implement reversing pattern state machine

    - Create reversing_pattern.h with pattern state enum (FORWARD, PAUSE_AFTER_FORWARD, REVERSE, PAUSE_AFTER_REVERSE)
    - Implement pattern execution with configurable forward/reverse/pause times
    - Enforce minimum 2-second pause regardless of configuration
    - Integrate with OutputHAL for motor control
    - _Requirements: 16.1-16.12_

  - [ ]\* 7.2 Write property test for reversing pattern timing

    - **Property 19: Reversing Pattern Timing**
    - **Validates: Requirements 16.1, 16.2, 16.3**
    - Test that pattern executes with correct timing for forward, reverse, and pause
    - Verify minimum 2-second pause constraint

  - [ ]\* 7.3 Write unit tests for reversing pattern
    - Test pattern state transitions
    - Test timing for each phase
    - Test minimum pause enforcement
    - _Requirements: 16.1-16.12_

- [ ] 8. Control Layer - Safety interlock system

  - [ ] 8.1 Implement safety interlock monitoring

    - Create safety_interlock.h with ISafetyInterlock interface
    - Implement continuous monitoring of door sensor, temperature sensor, temperature limits
    - Implement fault detection for door open, sensor fail, over-temperature, relay conflict
    - Implement isSafe(), canActivateHeater(), canActivateMotor() checks
    - _Requirements: 2.5-2.9, 19.1-19.11, 36.1-36.10_

  - [ ] 8.2 Implement fault handling and logging

    - Implement triggerFault() to call emergencyStop() and log fault
    - Implement fault latching until acknowledged
    - Implement fault auto-recovery conditions for specific faults
    - _Requirements: 19.3-19.11, 20.1-20.10_

  - [ ]\* 8.3 Write property test for emergency stop response time

    - **Property 1: Emergency Stop Response Time**
    - **Validates: Requirements 2.5, 2.6, 19.3, 19.4, 27.8, 39.2**
    - Test that all outputs deactivate within 100ms of fault detection
    - Use various fault conditions (door open, over-temp, sensor fail)

  - [ ]\* 8.4 Write property test for heater interlock when door open

    - **Property 15: Heater Interlock When Door Open**
    - **Validates: Requirements 36.2**
    - Test that heater cannot activate when door is open
    - Test that heater deactivates immediately when door opens

  - [ ]\* 8.5 Write property test for motor interlock when door open

    - **Property 16: Motor Interlock When Door Open**
    - **Validates: Requirements 36.3**
    - Test that motor relays cannot activate when door is open
    - Test that motor relays deactivate immediately when door opens

  - [ ]\* 8.6 Write unit tests for safety interlocks
    - Test fault detection for each fault type
    - Test fault latching behavior
    - Test auto-recovery conditions
    - Test safety check functions
    - _Requirements: 19.1-19.11, 36.1-36.10_

- [ ] 9. Checkpoint - Control layer validation

  - Ensure all control layer tests pass
  - Verify PID control with simulated temperature input
  - Verify reversing pattern timing
  - Verify safety interlocks trigger correctly
  - Ask the user if questions arise

- [ ] 10. Storage Layer - EEPROM management

  - [ ] 10.1 Implement EEPROM manager core

    - Create eeprom_manager.h with memory map definitions
    - Implement CRC8 and CRC16 checksum functions
    - Implement init() to verify magic number and version
    - Implement factoryReset() to initialize all EEPROM data
    - _Requirements: 26.1-26.10_

  - [ ] 10.2 Implement program storage

    - Implement loadProgram() with checksum verification
    - Implement saveProgram() with checksum generation
    - Implement restoreDefaultProgram() for factory defaults
    - Define 6 default programs in PROGMEM (Towels, Bed Sheets, Mixed Load, Heavy Cotton, Delicate, Work Wear)
    - _Requirements: 12.1-12.10, 34.1-34.2_

  - [ ] 10.3 Implement PID parameter storage

    - Implement loadPIDParams() with checksum verification
    - Implement savePIDParams() with checksum generation
    - Use default PID parameters if EEPROM is corrupted
    - _Requirements: 14.10-14.11, 35.1-35.10_

  - [ ]\* 10.4 Write property test for EEPROM checksum validation

    - **Property 8: EEPROM Checksum Validation**
    - **Validates: Requirements 12.5, 26.4**
    - Test that data blocks with invalid checksums are rejected
    - Test that corrupted data falls back to defaults

  - [ ]\* 10.5 Write property test for EEPROM corruption recovery

    - **Property 12: EEPROM Corruption Recovery**
    - **Validates: Requirements 26.5**
    - Test that system continues operation with factory defaults when EEPROM is corrupted
    - Verify no fault state is entered

  - [ ]\* 10.6 Write unit tests for EEPROM manager
    - Test CRC8 and CRC16 computation
    - Test program save/load with valid data
    - Test program load with corrupted checksum
    - Test PID parameter save/load
    - _Requirements: 26.1-26.10_

- [ ] 11. Storage Layer - Fault logging and statistics

  - [ ] 11.1 Implement fault logger

    - Create fault_logger.h with FaultEntry and FaultLog structures
    - Implement appendFaultEntry() with circular buffer and wear leveling
    - Implement readFaultLog() with checksum verification
    - Implement clearFaultLog()
    - Store up to 20 fault entries
    - _Requirements: 23.1-23.10_

  - [ ] 11.2 Implement statistics tracking

    - Create statistics.h with Statistics structure
    - Implement tracking for total cycles, operating hours, heater hours, motor hours
    - Implement cycles per program tracking
    - Implement relay activation counting
    - Implement saveStatistics() and loadStatistics() with checksums
    - _Requirements: 31.1-31.10_

  - [ ]\* 11.3 Write unit tests for fault logger and statistics
    - Test circular buffer wrapping
    - Test fault log checksum validation
    - Test statistics accumulation
    - Test wear leveling for fault log
    - _Requirements: 23.1-23.10, 31.1-31.10_

- [ ] 12. Checkpoint - Storage layer validation

  - Ensure all storage layer tests pass
  - Verify EEPROM read/write operations
  - Verify checksum validation and corruption recovery
  - Verify fault logging and statistics tracking
  - Ask the user if questions arise

- [ ] 13. Application Layer - Cycle state machine

  - [ ] 13.1 Implement state machine core

    - Create state_machine.h with IStateMachine interface
    - Define CycleState enum (IDLE, STARTING, HEATING, DRYING, COOLDOWN, COMPLETE, PAUSED, FAULT)
    - Implement state transition logic with guards
    - Implement start(), stop(), pause(), resume() functions
    - Implement update() to execute current state behavior
    - _Requirements: 17.1-17.13_

  - [ ] 13.2 Implement state behaviors

    - Implement IDLE state: display ready screen, monitor for start
    - Implement STARTING state: validate safety, transition to HEATING
    - Implement HEATING state: activate PID and reversing, wait for target temp
    - Implement DRYING state: maintain temp, run timer, display progress
    - Implement COOLDOWN state: heater off, motor continues, run cooldown timer
    - Implement COMPLETE state: all off, display complete, wait for acknowledgment
    - Implement PAUSED state: save state, wait for resume or stop
    - Implement FAULT state: emergency stop, display fault, wait for clear
    - _Requirements: 17.3-17.13, 18.1-18.10_

  - [ ] 13.3 Integrate state machine with control layer

    - Call PID controller during HEATING and DRYING states
    - Call reversing pattern during HEATING, DRYING, and COOLDOWN states
    - Call safety interlock every update
    - Transition to FAULT state on safety violations
    - _Requirements: 17.1-17.13, 19.12_

  - [ ]\* 13.4 Write property test for state transition logging

    - **Property 11: State Transition Logging**
    - **Validates: Requirements 17.13**
    - Test that all state transitions create log entries with timestamp
    - Verify source and destination states are recorded

  - [ ]\* 13.5 Write unit tests for state machine
    - Test each state transition with guards
    - Test state behaviors
    - Test fault handling during each state
    - Test pause/resume functionality
    - _Requirements: 17.1-17.13_

- [ ] 14. Application Layer - Program manager

  - [ ] 14.1 Implement program manager

    - Create program_manager.h with Program structure
    - Implement loadProgram() to get program from EEPROM or defaults
    - Implement saveProgram() for service mode editing
    - Implement parameter validation (temp 30-85°C, duration 5-180 min, etc.)
    - Provide access to 6 default programs
    - _Requirements: 12.1-12.10, 13.1-13.10_

  - [ ]\* 14.2 Write property test for parameter range validation

    - **Property 9: Parameter Range Validation**
    - **Validates: Requirements 12.8**
    - Test that invalid parameter values are rejected before saving
    - Test boundary values for all parameters

  - [ ]\* 14.3 Write unit tests for program manager
    - Test loading default programs
    - Test parameter validation for each parameter type
    - Test program save/load cycle
    - _Requirements: 12.1-12.10_

- [ ] 15. Application Layer - Menu system

  - [ ] 15.1 Implement menu navigation core

    - Create menu_system.h with MenuContext enum and MenuState structure
    - Implement menu hierarchy (Main Menu, Auto Mode, Manual Mode, Service Mode, etc.)
    - Implement handleKey() for navigation (UP, DOWN, OK, CANCEL)
    - Implement menu timeout (60 seconds of inactivity)
    - _Requirements: 9.1-9.10_

  - [ ] 15.2 Implement Auto Mode menu

    - Display program selection list
    - Allow navigation through programs
    - Display program parameters when selected
    - Start cycle on START key press
    - _Requirements: 10.1-10.10_

  - [ ] 15.3 Implement Manual Mode menu

    - Provide manual control for heater, motor forward, motor reverse
    - Provide target temperature and timer setting
    - Enforce all safety interlocks in manual mode
    - Limit manual mode to 90°C maximum
    - _Requirements: 11.1-11.10_

  - [ ] 15.4 Implement display layouts

    - Implement idle screen layout
    - Implement cycle running screen with progress bar
    - Implement fault screen with flashing
    - Implement menu navigation screen
    - Implement parameter edit screen
    - _Requirements: 30.1-30.10_

  - [ ]\* 15.5 Write unit tests for menu system
    - Test menu navigation with key sequences
    - Test menu timeout behavior
    - Test program selection flow
    - Test manual mode controls
    - _Requirements: 9.1-9.10, 10.1-10.10, 11.1-11.10_

- [ ] 16. Application Layer - Service mode

  - [ ] 16.1 Implement service mode access

    - Implement service code entry (default "1234")
    - Implement lockout after 3 incorrect attempts
    - Implement service mode timeout (10 minutes)
    - _Requirements: 21.1-21.10_

  - [ ] 16.2 Implement diagnostic tests

    - Implement relay test functions (5-second activation)
    - Implement sensor test functions (real-time display)
    - Implement LCD test pattern
    - Implement keypad echo test
    - _Requirements: 22.1-22.10_

  - [ ] 16.3 Implement service mode functions

    - Implement program parameter editing
    - Implement PID parameter viewing and editing
    - Implement fault log viewing
    - Implement statistics viewing
    - Implement factory reset with confirmation
    - _Requirements: 13.1-13.10, 21.1-21.10, 23.1-23.10, 44.1-44.11_

  - [ ]\* 16.4 Write unit tests for service mode
    - Test service code validation
    - Test lockout after failed attempts
    - Test diagnostic test execution
    - Test parameter editing workflow
    - _Requirements: 21.1-21.10, 22.1-22.10_

- [ ] 17. Checkpoint - Application layer validation

  - Ensure all application layer tests pass
  - Verify state machine transitions on actual hardware
  - Verify menu navigation on LCD/keypad
  - Verify program execution end-to-end
  - Ask the user if questions arise

- [ ] 18. Main control loop integration

  - [ ] 18.1 Implement main loop timing

    - Implement 100ms control loop period
    - Implement non-blocking timing using millis()
    - Implement loop counter for periodic tasks
    - Ensure critical path completes within timing budget
    - _Requirements: 27.1-27.12, 39.1_

  - [ ] 18.2 Integrate all modules in main loop

    - Call sensorHAL.update() every loop
    - Call safetyInterlock.update() every loop
    - Call stateMachine.update() every loop
    - Call reversingPattern.update() every loop
    - Update PID every 1 second (10 loops)
    - Call keypadHAL.update() every loop
    - Update display every 500ms (5 loops)
    - _Requirements: 27.1-27.12_

  - [ ] 18.3 Implement watchdog timer

    - Enable ATmega328P watchdog with 2-second timeout
    - Reset watchdog every loop iteration
    - Detect watchdog reset on startup
    - Increment reset counter in EEPROM
    - Enter fault state if reset counter exceeds 3
    - _Requirements: 25.1-25.10_

  - [ ]\* 18.4 Write property test for stop key response time

    - **Property 13: Stop Key Response Time**
    - **Validates: Requirements 27.9, 39.3**
    - Test that system transitions to PAUSED within 200ms of stop key press
    - Verify response time across different cycle states

  - [ ]\* 18.5 Write integration tests for main loop
    - Test loop timing consistency
    - Test watchdog reset behavior
    - Test module coordination
    - _Requirements: 27.1-27.12, 25.1-25.10_

- [ ] 19. System initialization and startup

  - [ ] 19.1 Implement startup sequence

    - Initialize all outputs to LOW before any other initialization
    - Initialize I2C for LCD
    - Initialize OneWire for temperature sensor
    - Initialize keypad interface
    - Display startup message with firmware version
    - _Requirements: 24.1-24.14_

  - [ ] 19.2 Implement self-test

    - Test LCD communication
    - Test temperature sensor communication
    - Test door sensor reading
    - Enter FAULT state if critical component fails
    - Load EEPROM data or use defaults if corrupted
    - _Requirements: 24.8-24.14_

  - [ ]\* 19.3 Write unit tests for initialization
    - Test startup sequence order
    - Test self-test fault detection
    - Test EEPROM initialization
    - _Requirements: 24.1-24.14_

- [ ] 20. PID Auto-Tune implementation

  - [ ] 20.1 Implement relay feedback test

    - Create auto-tune state machine (IDLE, WAITING, RELAY_TEST, CALCULATING, COMPLETE, FAILED)
    - Implement relay feedback oscillation test
    - Detect peaks and valleys in temperature response
    - Calculate ultimate gain and period from oscillations
    - _Requirements: 15.1-15.12_

  - [ ] 20.2 Implement Ziegler-Nichols tuning

    - Calculate PID parameters from ultimate gain and period
    - Apply Ziegler-Nichols tuning rules (Kp=0.6*Ku, Ki=1.2*Ku/Tu, Kd=0.075*Ku*Tu)
    - Save tuned parameters to EEPROM
    - Provide auto-tune progress display
    - _Requirements: 15.1-15.12_

  - [ ]\* 20.3 Write unit tests for auto-tune
    - Test peak/valley detection
    - Test parameter calculation
    - Test auto-tune timeout and failure handling
    - _Requirements: 15.1-15.12_

- [ ] 21. Checkpoint - System integration validation

  - Ensure all integration tests pass
  - Verify complete cycle execution for all 6 programs
  - Verify all safety interlocks on actual hardware
  - Verify fault handling for all fault types
  - Verify menu system on actual LCD/keypad
  - Ask the user if questions arise

- [ ] 22. Memory optimization and final polish

  - [ ] 22.1 Optimize memory usage

    - Move all constant strings to PROGMEM using F() macro
    - Store default programs in PROGMEM
    - Use bit fields for boolean flags
    - Verify flash usage < 28KB and SRAM usage < 1.5KB
    - _Requirements: 40.1-40.11_

  - [ ] 22.2 Add version information and build date

    - Define FIRMWARE_VERSION and FIRMWARE_DATE in config.h
    - Display version on startup screen
    - Display version in service mode
    - _Requirements: 37.11, 44.8_

  - [ ] 22.3 Final code cleanup
    - Add module header comments
    - Add function documentation
    - Ensure consistent code style
    - Remove debug code and unused functions
    - Verify compilation without warnings
    - _Requirements: 37.2-37.12_

- [ ] 23. Hardware validation and commissioning

  - [ ] 23.1 Perform hardware-in-the-loop testing

    - Test with actual DS18B20 sensor and heating element
    - Test with actual door switch
    - Test with actual motor and relays
    - Measure door open response time (must be < 100ms)
    - Measure stop key response time (must be < 200ms)
    - _Requirements: 38.1-38.10, 39.1-39.12_

  - [ ] 23.2 Validate safety interlocks on hardware

    - Test door open during each cycle state
    - Test temperature sensor failure during operation
    - Test over-temperature condition
    - Test relay conflict detection
    - Verify emergency stop response
    - _Requirements: 36.1-36.10, 38.9_

  - [ ] 23.3 Perform thermal characterization

    - Run PID auto-tune with actual dryer thermal mass
    - Measure temperature control accuracy (must be ±3°C)
    - Measure time to reach setpoint (should be < 10 minutes)
    - Validate cooldown behavior
    - _Requirements: 15.1-15.12, 39.10, 39.11_

  - [ ] 23.4 Stress testing
    - Run continuous operation for 8 hours
    - Perform rapid start/stop cycles
    - Test repeated door open/close
    - Verify EEPROM wear leveling
    - _Requirements: 38.5_

- [ ] 24. Final checkpoint - System validation complete
  - All unit tests pass
  - All property tests pass
  - All integration tests pass
  - All hardware tests pass
  - Memory usage within constraints
  - Timing requirements met
  - Safety interlocks validated
  - Ready for deployment

## Notes

- Tasks marked with `*` are optional testing tasks and can be skipped for faster MVP delivery
- Each task references specific requirements for traceability
- Checkpoints ensure incremental validation before proceeding
- Property tests validate universal correctness properties from the design document
- Unit tests validate specific examples and edge cases
- Hardware validation must be performed before deployment to ensure safety-critical functionality
- The implementation uses C++ for Arduino Nano (ATmega328P) platform
- All code must fit within 28KB flash and 1.5KB SRAM constraints
