# Requirements Document: IMESA ES Series Industrial Tumble Dryer Controller

## Introduction

This document specifies the requirements for an Arduino Nano-based industrial tumble dryer controller that reverse-engineers the behavior of the IMESA ES Series Industrial Tumble Dryer. The controller implements safety-critical machine control, menu-driven HMI, closed-loop temperature regulation, drum motor control with reversing patterns, and industrial-grade fault handling.

The system is designed for commercial laundry environments and must meet industrial reliability, safety, and usability standards. This specification provides sufficient detail for an embedded development team to design, implement, test, and validate the complete controller firmware.

## Glossary

- **Controller**: The Arduino Nano-based embedded system implementing the dryer control logic
- **Drum**: The rotating cylinder that holds and tumbles the laundry items
- **Heater**: The electric heating element controlled by the Heater_Relay
- **Heater_Relay**: Digital output controlling power to the heating element
- **Motor_Forward_Relay**: Digital output activating forward drum rotation
- **Motor_Reverse_Relay**: Digital output activating reverse drum rotation
- **Door_Sensor**: Digital input detecting door open/closed state
- **Temperature_Sensor**: DS18B20 one-wire digital temperature sensor measuring drum air temperature
- **LCD**: 20x4 character liquid crystal display with I2C interface for user interface
- **Keypad**: 4x4 matrix keypad for user input
- **Program**: A predefined drying cycle with specific temperature, duration, and drum motion parameters
- **Cycle**: A single execution of a Program from start to completion
- **PID_Controller**: Proportional-Integral-Derivative closed-loop temperature control algorithm
- **Auto_Tune**: Automatic PID parameter calibration procedure
- **HMI**: Human-Machine Interface comprising LCD and Keypad
- **Fault_State**: A latched error condition requiring user acknowledgment or system reset
- **Safe_State**: Controller state with all outputs deactivated (heater off, motor off)
- **Cooldown**: Final cycle phase where drum continues rotating with heater off to reduce temperature
- **Reversing_Pattern**: Timed sequence of forward and reverse drum rotation
- **IMESA_ES_Series**: Target industrial tumble dryer model being reverse-engineered
- **Service_Mode**: Diagnostic and configuration interface for technicians
- **Interlock**: Safety mechanism preventing operation when conditions are unsafe

## Requirements

### Requirement 1: Hardware Platform

**User Story:** As a system integrator, I want the controller to run on Arduino Nano hardware, so that I can use readily available, cost-effective embedded platforms.

#### Acceptance Criteria

1. THE Controller SHALL execute on Arduino Nano hardware
2. THE Controller SHALL use the ATmega328P microcontroller capabilities within Arduino Nano specifications
3. THE Controller SHALL operate within Arduino Nano memory constraints (32KB flash, 2KB SRAM)
4. THE Controller SHALL use Arduino Nano 5V logic levels for all digital I/O
5. THE Controller SHALL use Arduino Nano 16MHz clock frequency for all timing calculations

### Requirement 2: Door Safety Interlock

**User Story:** As a safety engineer, I want the door sensor to immediately stop all hazardous operations when the door opens, so that operators cannot access moving or hot components during operation.

#### Acceptance Criteria

1. THE Controller SHALL read the Door_Sensor digital input on pin 6
2. THE Door_Sensor SHALL be configured with internal pull-up resistor enabled
3. WHEN the Door_Sensor reads LOW, THE Controller SHALL interpret this as door open
4. WHEN the Door_Sensor reads HIGH, THE Controller SHALL interpret this as door closed
5. WHEN the door opens during any operation, THE Controller SHALL immediately deactivate the Heater_Relay within 100 milliseconds
6. WHEN the door opens during any operation, THE Controller SHALL immediately deactivate both Motor_Forward_Relay and Motor_Reverse_Relay within 100 milliseconds
7. WHEN the door opens during any operation, THE Controller SHALL enter Fault_State with fault code "DOOR OPEN"
8. WHEN the door is open, THE Controller SHALL NOT activate the Heater_Relay
9. WHEN the door is open, THE Controller SHALL NOT activate either motor relay
10. THE Controller SHALL debounce the Door_Sensor input with minimum 50 millisecond stable time before state change recognition

### Requirement 3: Temperature Sensing

**User Story:** As a control engineer, I want accurate drum temperature measurement using DS18B20 sensor, so that the controller can regulate drying temperature precisely.

#### Acceptance Criteria

1. THE Controller SHALL read temperature from a DS18B20 sensor connected to pin 12
2. THE Controller SHALL use the OneWire protocol to communicate with the Temperature_Sensor
3. THE Controller SHALL request temperature readings at intervals not exceeding 1 second
4. THE Controller SHALL apply a moving average filter with minimum 3 samples to Temperature_Sensor readings
5. WHEN the Temperature_Sensor returns a reading outside the range -10°C to 150°C, THE Controller SHALL reject the reading as invalid
6. WHEN the Temperature_Sensor fails to respond within 1000 milliseconds, THE Controller SHALL flag a sensor communication fault
7. WHEN three consecutive Temperature_Sensor readings are invalid, THE Controller SHALL enter Fault_State with fault code "TEMP SENSOR FAIL"
8. WHEN in Fault_State due to sensor failure, THE Controller SHALL deactivate the Heater_Relay
9. THE Controller SHALL display current temperature on the LCD with 1°C resolution
10. THE Controller SHALL store the last valid temperature reading for display purposes during temporary communication errors

### Requirement 4: Heater Control Output

**User Story:** As a control engineer, I want precise heater relay control, so that the system can regulate drum temperature safely and effectively.

#### Acceptance Criteria

1. THE Controller SHALL control the Heater_Relay via digital output on pin 9
2. WHEN the Heater_Relay output is HIGH, THE Controller SHALL energize the heater
3. WHEN the Heater_Relay output is LOW, THE Controller SHALL de-energize the heater
4. THE Controller SHALL initialize the Heater_Relay to LOW (off) during startup
5. THE Controller SHALL set the Heater_Relay to LOW (off) during any Fault_State
6. THE Controller SHALL set the Heater_Relay to LOW (off) when the door is open
7. THE Controller SHALL set the Heater_Relay to LOW (off) when Temperature_Sensor readings are invalid
8. THE Controller SHALL implement anti-chatter logic preventing Heater_Relay state changes more frequent than once per 5 seconds
9. WHEN the measured temperature exceeds the target temperature by 5°C or more, THE Controller SHALL set the Heater_Relay to LOW
10. THE Controller SHALL control the Heater_Relay using PID_Controller output during normal cycle operation

### Requirement 5: Motor Forward Control Output

**User Story:** As a control engineer, I want to control forward drum rotation, so that the system can tumble laundry in the forward direction.

#### Acceptance Criteria

1. THE Controller SHALL control the Motor_Forward_Relay via digital output on pin 10
2. WHEN the Motor_Forward_Relay output is HIGH, THE Controller SHALL activate forward drum rotation
3. WHEN the Motor_Forward_Relay output is LOW, THE Controller SHALL deactivate forward drum rotation
4. THE Controller SHALL initialize the Motor_Forward_Relay to LOW (off) during startup
5. THE Controller SHALL set the Motor_Forward_Relay to LOW (off) during any Fault_State
6. THE Controller SHALL set the Motor_Forward_Relay to LOW (off) when the door is open
7. THE Controller SHALL NOT activate Motor_Forward_Relay simultaneously with Motor_Reverse_Relay
8. WHEN transitioning from Motor_Reverse_Relay active to Motor_Forward_Relay active, THE Controller SHALL enforce a minimum 2 second pause with both relays LOW
9. THE Controller SHALL control Motor_Forward_Relay according to the active Reversing_Pattern during cycle operation
10. THE Controller SHALL deactivate Motor_Forward_Relay within 100 milliseconds of stop command or fault detection

### Requirement 6: Motor Reverse Control Output

**User Story:** As a control engineer, I want to control reverse drum rotation, so that the system can tumble laundry in the reverse direction to prevent tangling.

#### Acceptance Criteria

1. THE Controller SHALL control the Motor_Reverse_Relay via digital output on pin 11
2. WHEN the Motor_Reverse_Relay output is HIGH, THE Controller SHALL activate reverse drum rotation
3. WHEN the Motor_Reverse_Relay output is LOW, THE Controller SHALL deactivate reverse drum rotation
4. THE Controller SHALL initialize the Motor_Reverse_Relay to LOW (off) during startup
5. THE Controller SHALL set the Motor_Reverse_Relay to LOW (off) during any Fault_State
6. THE Controller SHALL set the Motor_Reverse_Relay to LOW (off) when the door is open
7. THE Controller SHALL NOT activate Motor_Reverse_Relay simultaneously with Motor_Forward_Relay
8. WHEN transitioning from Motor_Forward_Relay active to Motor_Reverse_Relay active, THE Controller SHALL enforce a minimum 2 second pause with both relays LOW
9. THE Controller SHALL control Motor_Reverse_Relay according to the active Reversing_Pattern during cycle operation
10. THE Controller SHALL deactivate Motor_Reverse_Relay within 100 milliseconds of stop command or fault detection

### Requirement 7: LCD Display Interface

**User Story:** As an operator, I want a clear 20x4 character display, so that I can view system status, menus, and operational information.

#### Acceptance Criteria

1. THE Controller SHALL interface with a 20x4 character LCD via I2C protocol
2. THE Controller SHALL use I2C address 0x27 for the LCD (or support configurable address)
3. THE Controller SHALL connect LCD SDA to Arduino Nano pin A4
4. THE Controller SHALL connect LCD SCL to Arduino Nano pin A5
5. THE Controller SHALL initialize the LCD during startup with backlight enabled
6. THE Controller SHALL display system status on the LCD during idle state
7. THE Controller SHALL display menu navigation on the LCD during menu operation
8. THE Controller SHALL display cycle progress information during active drying cycles
9. THE Controller SHALL display fault messages on the LCD when in Fault_State
10. THE Controller SHALL update the LCD display at intervals not exceeding 500 milliseconds during active operation
11. THE Controller SHALL use clear, abbreviated text suitable for 20-character line width
12. WHEN the LCD fails to initialize, THE Controller SHALL enter Fault_State with fault code "LCD FAIL"

### Requirement 8: Keypad Input Interface

**User Story:** As an operator, I want a responsive 4x4 keypad, so that I can navigate menus, select programs, and control the dryer.

#### Acceptance Criteria

1. THE Controller SHALL interface with a 4x4 matrix keypad using the Keypad library
2. THE Controller SHALL connect keypad rows to Arduino Nano pins A0, A1, A2, A3
3. THE Controller SHALL connect keypad columns to Arduino Nano pins D2, D3, D4, D5
4. THE Controller SHALL map keypad buttons according to the following layout:
   - Row 1: '7', '8', '9', '/'
   - Row 2: '4', '5', '6', '\*'
   - Row 3: '1', '2', '3', '-'
   - Row 4: 'C', '0', '=', '+'
5. THE Controller SHALL interpret key '8' as Up navigation
6. THE Controller SHALL interpret key '2' as Down navigation
7. THE Controller SHALL interpret key '4' as Left navigation
8. THE Controller SHALL interpret key '6' as Right navigation
9. THE Controller SHALL interpret key '5' as OK/Enter/Confirm
10. THE Controller SHALL interpret key '/' as Start command
11. THE Controller SHALL interpret key '\*' as Stop command
12. THE Controller SHALL interpret key 'C' as Cancel/Back command
13. THE Controller SHALL debounce keypad inputs with minimum 50 millisecond stable time
14. THE Controller SHALL scan the keypad at intervals not exceeding 100 milliseconds
15. THE Controller SHALL provide visual or audible feedback for valid key presses (if hardware supports)
16. WHEN an invalid key is pressed in a given context, THE Controller SHALL ignore the key press without state change

### Requirement 9: Main Menu System

**User Story:** As an operator, I want an intuitive menu system, so that I can easily select operating modes and access system functions.

#### Acceptance Criteria

1. THE Controller SHALL implement a hierarchical menu system accessible from idle state
2. THE Controller SHALL provide a Main Menu with the following options:
   - Auto Mode
   - Manual Mode
   - Program Selection
   - Service Mode
   - View Status
3. WHEN the user presses Up or Down keys in the Main Menu, THE Controller SHALL navigate between menu items
4. WHEN the user presses OK in the Main Menu, THE Controller SHALL enter the selected menu item
5. WHEN the user presses Cancel in any submenu, THE Controller SHALL return to the parent menu
6. THE Controller SHALL highlight the currently selected menu item on the LCD
7. THE Controller SHALL display menu items using clear, abbreviated text within 20-character line width
8. WHEN no key is pressed for 60 seconds in any menu, THE Controller SHALL return to idle status display
9. THE Controller SHALL NOT allow menu navigation during active cycle operation
10. WHEN in Fault_State, THE Controller SHALL display fault information and limit menu access to fault acknowledgment and Service Mode

### Requirement 10: Auto Mode Operation

**User Story:** As an operator, I want to select and run predefined drying programs, so that I can dry different fabric types with appropriate settings.

#### Acceptance Criteria

1. WHEN Auto Mode is selected, THE Controller SHALL display a list of available Programs
2. THE Controller SHALL support navigation through the Program list using Up and Down keys
3. WHEN a Program is selected with OK key, THE Controller SHALL display Program parameters (temperature, duration)
4. WHEN the Start key is pressed with a Program selected and door closed, THE Controller SHALL begin cycle execution
5. THE Controller SHALL execute the selected Program according to its defined parameters
6. WHEN the Stop key is pressed during Auto Mode cycle, THE Controller SHALL pause the cycle and prompt for confirmation
7. WHEN stop is confirmed, THE Controller SHALL terminate the cycle and enter cooldown phase
8. WHEN a cycle completes normally, THE Controller SHALL enter cooldown phase then return to idle state
9. THE Controller SHALL display cycle progress (remaining time, current temperature) during Auto Mode operation
10. THE Controller SHALL log cycle start time, end time, and completion status (if persistent storage is available)

### Requirement 11: Manual Mode Operation

**User Story:** As an operator or technician, I want manual control of heater and motor, so that I can test components or run custom drying cycles.

#### Acceptance Criteria

1. WHEN Manual Mode is selected, THE Controller SHALL display manual control options
2. THE Controller SHALL provide manual control for:
   - Heater On/Off
   - Motor Forward
   - Motor Reverse
   - Motor Stop
   - Target Temperature
   - Timer Duration
3. WHEN manual heater control is activated, THE Controller SHALL activate the Heater_Relay subject to safety interlocks
4. WHEN manual motor control is activated, THE Controller SHALL activate the selected motor relay subject to safety interlocks
5. THE Controller SHALL enforce all safety requirements in Manual Mode (door interlock, temperature limits, relay conflicts)
6. WHEN the Stop key is pressed in Manual Mode, THE Controller SHALL immediately deactivate all outputs
7. THE Controller SHALL display current temperature and output states during Manual Mode operation
8. WHEN Manual Mode timer expires, THE Controller SHALL deactivate outputs and return to idle state
9. THE Controller SHALL require door closed before allowing Manual Mode heater or motor activation
10. THE Controller SHALL limit Manual Mode heater operation to maximum temperature of 90°C

### Requirement 12: Program Definition and Storage

**User Story:** As a system designer, I want predefined drying programs for common industrial laundry items, so that operators can select appropriate cycles without manual parameter entry.

#### Acceptance Criteria

1. THE Controller SHALL support a minimum of 6 predefined Programs
2. THE Controller SHALL include Programs for the following fabric types:
   - Towels
   - Bed Sheets
   - Mixed Load
   - Heavy Cotton
   - Delicate Load
   - Work Wear
3. FOR EACH Program, THE Controller SHALL store the following parameters:
   - Program name (maximum 16 characters)
   - Target temperature (°C)
   - Cycle duration (minutes)
   - Forward rotation time (seconds)
   - Reverse rotation time (seconds)
   - Pause time between direction changes (seconds)
   - Cooldown duration (minutes)
4. THE Controller SHALL store Program definitions in program memory (PROGMEM) or EEPROM
5. WHERE EEPROM storage is used, THE Controller SHALL validate Program data integrity using checksums
6. THE Controller SHALL provide default Program parameters if stored data is corrupted
7. THE Controller SHALL allow Program parameter editing in Service Mode
8. WHEN Program parameters are edited, THE Controller SHALL validate parameter ranges before saving
9. THE Controller SHALL limit target temperature to range 30°C to 85°C for all Programs
10. THE Controller SHALL limit cycle duration to range 5 to 180 minutes for all Programs

### Requirement 13: Program Parameter Editing

**User Story:** As a service technician, I want to edit program parameters, so that I can customize drying cycles for specific customer requirements.

#### Acceptance Criteria

1. WHERE Service Mode is active, THE Controller SHALL allow editing of Program parameters
2. WHEN a Program is selected for editing, THE Controller SHALL display current parameter values
3. THE Controller SHALL allow navigation between parameters using Up and Down keys
4. WHEN a parameter is selected with OK key, THE Controller SHALL enter edit mode for that parameter
5. THE Controller SHALL allow numeric value entry using keypad number keys
6. WHEN editing is complete, THE Controller SHALL validate the entered value against parameter limits
7. WHEN an invalid value is entered, THE Controller SHALL display an error message and reject the change
8. WHEN a valid value is entered and confirmed, THE Controller SHALL save the new parameter value
9. THE Controller SHALL provide a "Restore Defaults" option for each Program
10. WHEN parameters are modified, THE Controller SHALL mark the Program as "Custom" on the display

### Requirement 14: PID Temperature Control

**User Story:** As a control engineer, I want closed-loop PID temperature control, so that the system maintains stable target temperature during drying cycles.

#### Acceptance Criteria

1. THE Controller SHALL implement a PID_Controller for temperature regulation
2. THE PID_Controller SHALL calculate control output based on temperature error (setpoint minus measured temperature)
3. THE PID_Controller SHALL use proportional, integral, and derivative terms in control calculation
4. THE PID_Controller SHALL update control output at intervals not exceeding 1 second
5. THE PID_Controller SHALL output a value in range 0 to 100 representing heater duty cycle percentage
6. WHEN PID output exceeds 50 percent, THE Controller SHALL activate the Heater_Relay
7. WHEN PID output is below 50 percent, THE Controller SHALL deactivate the Heater_Relay
8. THE Controller SHALL implement anti-windup protection for the integral term
9. THE Controller SHALL limit integral term accumulation to prevent excessive overshoot
10. THE Controller SHALL store PID tuning parameters (Kp, Ki, Kd) in EEPROM
11. THE Controller SHALL provide default PID parameters suitable for typical industrial dryer thermal characteristics
12. WHERE temperature sensor readings are invalid, THE Controller SHALL freeze PID calculation and deactivate the Heater_Relay

### Requirement 15: PID Auto-Tune Function

**User Story:** As a service technician, I want automatic PID tuning, so that the controller can optimize temperature control for different dryer installations.

#### Acceptance Criteria

1. WHERE Service Mode is active, THE Controller SHALL provide a PID Auto-Tune function
2. WHEN Auto-Tune is initiated, THE Controller SHALL display a warning that the process will take 15-30 minutes
3. WHEN Auto-Tune is confirmed, THE Controller SHALL require door closed and no faults present
4. THE Controller SHALL execute a relay-feedback test to determine system dynamics
5. THE Controller SHALL apply step changes to the Heater_Relay and measure temperature response
6. THE Controller SHALL calculate ultimate gain and ultimate period from oscillation characteristics
7. THE Controller SHALL compute PID parameters using Ziegler-Nichols or similar tuning rules
8. WHEN Auto-Tune completes successfully, THE Controller SHALL save new PID parameters to EEPROM
9. WHEN Auto-Tune fails (no oscillation, sensor error, door opened), THE Controller SHALL abort and restore previous parameters
10. THE Controller SHALL display Auto-Tune progress and estimated time remaining during the procedure
11. THE Controller SHALL allow Auto-Tune cancellation via Stop key
12. WHEN Auto-Tune is cancelled, THE Controller SHALL restore previous PID parameters

### Requirement 16: Drum Reversing Pattern Control

**User Story:** As a control engineer, I want configurable drum reversing patterns, so that the system can prevent fabric tangling and ensure even drying.

#### Acceptance Criteria

1. THE Controller SHALL implement Reversing_Pattern control for drum motor direction
2. THE Reversing_Pattern SHALL define forward rotation duration, reverse rotation duration, and pause duration
3. THE Controller SHALL execute the Reversing_Pattern continuously during cycle operation
4. WHEN forward rotation time expires, THE Controller SHALL deactivate Motor_Forward_Relay
5. WHEN forward rotation time expires, THE Controller SHALL wait for pause duration with both motor relays off
6. WHEN pause duration expires after forward rotation, THE Controller SHALL activate Motor_Reverse_Relay
7. WHEN reverse rotation time expires, THE Controller SHALL deactivate Motor_Reverse_Relay
8. WHEN reverse rotation time expires, THE Controller SHALL wait for pause duration with both motor relays off
9. WHEN pause duration expires after reverse rotation, THE Controller SHALL activate Motor_Forward_Relay
10. THE Controller SHALL enforce minimum 2 second pause between direction changes regardless of configured pause duration
11. THE Controller SHALL allow Reversing_Pattern parameters to be configured per Program
12. THE Controller SHALL validate that forward time, reverse time, and pause time are all greater than 2 seconds

### Requirement 17: Cycle State Machine

**User Story:** As a software engineer, I want a well-defined state machine for cycle execution, so that the system behavior is predictable and maintainable.

#### Acceptance Criteria

1. THE Controller SHALL implement cycle execution as a finite state machine
2. THE Controller SHALL define the following cycle states:
   - IDLE
   - STARTING
   - HEATING
   - DRYING
   - COOLDOWN
   - COMPLETE
   - PAUSED
   - FAULT
3. WHEN in IDLE state and Start is pressed with valid Program selected, THE Controller SHALL transition to STARTING state
4. WHEN in STARTING state with door closed and no faults, THE Controller SHALL transition to HEATING state
5. WHEN in HEATING state and temperature reaches target minus 5°C, THE Controller SHALL transition to DRYING state
6. WHEN in DRYING state and cycle timer expires, THE Controller SHALL transition to COOLDOWN state
7. WHEN in COOLDOWN state and cooldown timer expires, THE Controller SHALL transition to COMPLETE state
8. WHEN in COMPLETE state and OK is pressed, THE Controller SHALL transition to IDLE state
9. WHEN Stop is pressed in HEATING or DRYING state, THE Controller SHALL transition to PAUSED state
10. WHEN in PAUSED state and Start is pressed, THE Controller SHALL resume previous state
11. WHEN in PAUSED state and stop is confirmed, THE Controller SHALL transition to COOLDOWN state
12. WHEN door opens or fault occurs in any active state, THE Controller SHALL transition to FAULT state
13. THE Controller SHALL log all state transitions with timestamp (if persistent storage is available)

### Requirement 18: Cooldown Phase Operation

**User Story:** As a safety engineer, I want automatic cooldown at cycle end, so that operators do not open the door to excessively hot laundry.

#### Acceptance Criteria

1. WHEN a cycle enters COOLDOWN state, THE Controller SHALL deactivate the Heater_Relay
2. WHEN in COOLDOWN state, THE Controller SHALL continue drum rotation using the active Reversing_Pattern
3. THE Controller SHALL execute cooldown for the duration specified in the active Program
4. THE Controller SHALL display "COOLDOWN" and remaining time on the LCD during cooldown phase
5. WHEN cooldown completes, THE Controller SHALL deactivate all motor relays
6. WHEN cooldown completes, THE Controller SHALL display "CYCLE COMPLETE" on the LCD
7. THE Controller SHALL allow door opening during cooldown without entering Fault_State
8. WHEN door opens during cooldown, THE Controller SHALL immediately stop motor rotation
9. THE Controller SHALL provide an audible alert when cooldown completes (if hardware supports buzzer)
10. THE Controller SHALL allow cooldown to be skipped via Service Mode setting for testing purposes

### Requirement 19: Fault Detection and Handling

**User Story:** As a safety engineer, I want comprehensive fault detection, so that the system responds safely to abnormal conditions.

#### Acceptance Criteria

1. THE Controller SHALL detect and respond to the following fault conditions:
   - Door opened during operation
   - Temperature sensor failure
   - Temperature sensor invalid reading
   - Over-temperature (exceeds target by 15°C)
   - LCD communication failure
   - Relay conflict (both motor relays active simultaneously)
   - Watchdog timeout (if implemented)
2. WHEN a fault is detected, THE Controller SHALL immediately transition to FAULT state
3. WHEN entering FAULT state, THE Controller SHALL deactivate Heater_Relay within 100 milliseconds
4. WHEN entering FAULT state, THE Controller SHALL deactivate both motor relays within 100 milliseconds
5. WHEN in FAULT state, THE Controller SHALL display fault code and description on the LCD
6. WHEN in FAULT state, THE Controller SHALL latch the fault condition until acknowledged
7. THE Controller SHALL require user acknowledgment (OK key press) to clear non-critical faults
8. THE Controller SHALL require Service Mode access to clear critical faults
9. THE Controller SHALL log fault occurrence with timestamp and fault code (if persistent storage is available)
10. WHEN a fault is cleared, THE Controller SHALL return to IDLE state
11. THE Controller SHALL NOT allow cycle restart until fault is cleared and door is closed

### Requirement 20: Over-Temperature Protection

**User Story:** As a safety engineer, I want over-temperature protection, so that the system prevents fire hazards and fabric damage.

#### Acceptance Criteria

1. THE Controller SHALL monitor drum temperature continuously during all operations
2. WHEN measured temperature exceeds 100°C, THE Controller SHALL immediately deactivate the Heater_Relay
3. WHEN measured temperature exceeds 100°C, THE Controller SHALL enter FAULT state with fault code "OVER TEMP"
4. WHEN over-temperature fault occurs, THE Controller SHALL continue motor rotation to dissipate heat
5. WHEN over-temperature fault occurs, THE Controller SHALL display warning message on LCD
6. THE Controller SHALL NOT allow heater reactivation until temperature drops below 80°C
7. THE Controller SHALL require Service Mode access to clear over-temperature fault
8. THE Controller SHALL log over-temperature events with timestamp and peak temperature (if persistent storage is available)
9. WHERE over-temperature occurs three times in a single cycle, THE Controller SHALL abort the cycle and require service
10. THE Controller SHALL implement a software-based over-temperature threshold independent of PID control

### Requirement 21: Service Mode Access and Functions

**User Story:** As a service technician, I want a protected service mode, so that I can access diagnostic functions and configuration settings.

#### Acceptance Criteria

1. THE Controller SHALL provide Service Mode accessible from Main Menu
2. THE Controller SHALL require a service code entry to access Service Mode
3. THE Controller SHALL use service code "1234" as default (configurable in firmware)
4. WHEN correct service code is entered, THE Controller SHALL grant Service Mode access for current session
5. WHEN incorrect service code is entered three times, THE Controller SHALL lock Service Mode access for 5 minutes
6. WHERE Service Mode is active, THE Controller SHALL provide access to:
   - Program parameter editing
   - PID parameter viewing and editing
   - PID Auto-Tune function
   - Diagnostic tests
   - Fault log viewing
   - Factory reset function
   - Sensor calibration (if applicable)
7. THE Controller SHALL display "SERVICE MODE" indicator on LCD when Service Mode is active
8. THE Controller SHALL exit Service Mode when Cancel key is pressed or after 10 minutes of inactivity
9. THE Controller SHALL log Service Mode access with timestamp (if persistent storage is available)
10. THE Controller SHALL NOT allow Service Mode access during active cycle operation

### Requirement 22: Diagnostic Test Functions

**User Story:** As a service technician, I want diagnostic tests for each output and input, so that I can verify hardware functionality during installation and troubleshooting.

#### Acceptance Criteria

1. WHERE Service Mode is active, THE Controller SHALL provide diagnostic test functions
2. THE Controller SHALL provide individual tests for:
   - Heater relay activation
   - Motor forward relay activation
   - Motor reverse relay activation
   - Door sensor reading
   - Temperature sensor reading
   - LCD display test pattern
   - Keypad test (echo key presses)
3. WHEN a relay test is selected, THE Controller SHALL activate the relay for 5 seconds then deactivate
4. WHEN a relay test is active, THE Controller SHALL display warning message and require door closed
5. WHEN door sensor test is selected, THE Controller SHALL display real-time door state
6. WHEN temperature sensor test is selected, THE Controller SHALL display real-time temperature with 0.1°C resolution
7. WHEN LCD test is selected, THE Controller SHALL display test pattern cycling through all characters
8. WHEN keypad test is selected, THE Controller SHALL echo each key press on the LCD
9. THE Controller SHALL allow test cancellation via Stop key
10. THE Controller SHALL enforce all safety interlocks during diagnostic tests

### Requirement 23: Fault Logging and History

**User Story:** As a service technician, I want to view fault history, so that I can diagnose intermittent problems and track system reliability.

#### Acceptance Criteria

1. WHERE EEPROM storage is available, THE Controller SHALL maintain a fault log
2. THE Controller SHALL store the most recent 20 fault events in the fault log
3. FOR EACH fault event, THE Controller SHALL store:
   - Fault code
   - Timestamp (if RTC is available) or event counter
   - Cycle state when fault occurred
   - Temperature at fault occurrence
4. WHERE Service Mode is active, THE Controller SHALL provide fault log viewing function
5. THE Controller SHALL display fault log entries in reverse chronological order (most recent first)
6. THE Controller SHALL allow navigation through fault log using Up and Down keys
7. THE Controller SHALL provide a "Clear Fault Log" function in Service Mode
8. WHEN fault log is cleared, THE Controller SHALL require confirmation before deletion
9. THE Controller SHALL validate fault log data integrity using checksums
10. WHEN fault log data is corrupted, THE Controller SHALL initialize an empty log

### Requirement 24: System Initialization and Startup

**User Story:** As a safety engineer, I want safe and predictable system startup, so that the controller initializes to a known safe state.

#### Acceptance Criteria

1. WHEN power is applied, THE Controller SHALL execute initialization sequence within 3 seconds
2. THE Controller SHALL initialize all digital outputs to LOW (off) state before any other initialization
3. THE Controller SHALL initialize the Door_Sensor input with pull-up resistor enabled
4. THE Controller SHALL initialize I2C communication for LCD
5. THE Controller SHALL initialize OneWire communication for Temperature_Sensor
6. THE Controller SHALL initialize the Keypad interface
7. THE Controller SHALL display startup message on LCD showing firmware version
8. THE Controller SHALL perform self-test of critical components (LCD, Temperature_Sensor, Door_Sensor)
9. WHEN self-test detects a failure, THE Controller SHALL enter FAULT state and display fault code
10. WHEN self-test passes, THE Controller SHALL transition to IDLE state
11. THE Controller SHALL load Program definitions from EEPROM or use defaults if EEPROM is invalid
12. THE Controller SHALL load PID parameters from EEPROM or use defaults if EEPROM is invalid
13. THE Controller SHALL verify EEPROM data integrity using checksums before loading
14. WHEN EEPROM data is corrupted, THE Controller SHALL initialize with factory defaults and log the event

### Requirement 25: Watchdog and System Monitoring

**User Story:** As a reliability engineer, I want watchdog monitoring, so that the system can recover from software faults.

#### Acceptance Criteria

1. THE Controller SHALL enable the ATmega328P hardware watchdog timer
2. THE Controller SHALL configure watchdog timeout to 2 seconds
3. THE Controller SHALL reset the watchdog timer in the main loop at intervals not exceeding 1 second
4. WHEN the watchdog timer expires, THE Controller SHALL perform a system reset
5. THE Controller SHALL detect watchdog reset during startup initialization
6. WHEN watchdog reset is detected, THE Controller SHALL increment a reset counter in EEPROM
7. WHEN watchdog reset counter exceeds 3 in a 24-hour period, THE Controller SHALL enter FAULT state requiring service
8. THE Controller SHALL log watchdog reset events with timestamp (if persistent storage is available)
9. THE Controller SHALL provide watchdog reset counter viewing in Service Mode
10. THE Controller SHALL allow watchdog reset counter clearing in Service Mode

### Requirement 26: EEPROM Data Management

**User Story:** As a software engineer, I want reliable EEPROM storage, so that configuration and logs persist across power cycles.

#### Acceptance Criteria

1. THE Controller SHALL use EEPROM to store persistent data including:
   - Program parameters (6 programs × parameters)
   - PID tuning parameters (Kp, Ki, Kd)
   - Service code
   - Fault log (20 entries)
   - Cycle statistics (total cycles, total hours)
   - Watchdog reset counter
2. THE Controller SHALL organize EEPROM data with defined memory map and version identifier
3. THE Controller SHALL compute and store CRC16 checksum for each data block
4. WHEN reading EEPROM data, THE Controller SHALL verify checksum before using data
5. WHEN checksum verification fails, THE Controller SHALL use factory default values for that data block
6. THE Controller SHALL implement wear-leveling for frequently updated data (fault log, statistics)
7. THE Controller SHALL limit EEPROM write operations to configuration changes and fault events
8. THE Controller SHALL NOT write to EEPROM during normal cycle operation
9. THE Controller SHALL provide EEPROM data export function in Service Mode (display as hex values)
10. THE Controller SHALL provide factory reset function that restores all EEPROM data to defaults

### Requirement 27: Timing and Real-Time Constraints

**User Story:** As a control engineer, I want predictable timing behavior, so that the system meets real-time control requirements.

#### Acceptance Criteria

1. THE Controller SHALL execute the main control loop at intervals not exceeding 100 milliseconds
2. THE Controller SHALL read Door_Sensor state every control loop iteration
3. THE Controller SHALL update LCD display at intervals not exceeding 500 milliseconds
4. THE Controller SHALL scan Keypad at intervals not exceeding 100 milliseconds
5. THE Controller SHALL read Temperature_Sensor at intervals not exceeding 1000 milliseconds
6. THE Controller SHALL update PID_Controller calculation at intervals not exceeding 1000 milliseconds
7. THE Controller SHALL update Reversing_Pattern state at intervals not exceeding 100 milliseconds
8. THE Controller SHALL respond to door open event within 100 milliseconds
9. THE Controller SHALL respond to Stop key press within 200 milliseconds
10. THE Controller SHALL use non-blocking I/O for all sensor and display operations
11. THE Controller SHALL NOT use delay() function in main control loop
12. THE Controller SHALL use millis() or micros() for all timing measurements

### Requirement 28: Power Loss and Recovery

**User Story:** As an operator, I want the system to handle power interruptions safely, so that cycles can resume or restart appropriately.

#### Acceptance Criteria

1. WHEN power is lost during cycle operation, THE Controller SHALL lose all volatile state
2. WHEN power is restored, THE Controller SHALL perform normal startup initialization
3. THE Controller SHALL NOT automatically resume interrupted cycles after power restoration
4. THE Controller SHALL return to IDLE state after power restoration
5. WHERE cycle resume is desired, THE Controller SHALL require manual restart by operator
6. THE Controller SHALL log power loss events in fault log (if timestamp or event counter is available)
7. THE Controller SHALL display "POWER RESTORED" message on first startup after power loss
8. THE Controller SHALL verify all safety interlocks before allowing any operation after power restoration
9. THE Controller SHALL NOT retain cycle progress or timer state across power cycles
10. WHERE future enhancement is desired, THE Controller SHALL reserve EEPROM space for cycle state persistence

### Requirement 29: Anti-Chatter and Output Stability

**User Story:** As a reliability engineer, I want stable output control, so that relays are not damaged by excessive switching.

#### Acceptance Criteria

1. THE Controller SHALL implement anti-chatter logic for all relay outputs
2. THE Controller SHALL enforce minimum 5 second interval between Heater_Relay state changes
3. THE Controller SHALL enforce minimum 2 second interval between motor relay state changes
4. THE Controller SHALL implement hysteresis in temperature-based heater control
5. WHEN temperature is rising toward setpoint, THE Controller SHALL deactivate heater at setpoint
6. WHEN temperature is falling from setpoint, THE Controller SHALL activate heater at setpoint minus 3°C
7. THE Controller SHALL filter Temperature_Sensor readings to prevent noise-induced output changes
8. THE Controller SHALL ignore transient Door_Sensor state changes shorter than 50 milliseconds
9. THE Controller SHALL count relay activation cycles and store in EEPROM for maintenance tracking
10. WHERE relay activation count exceeds 100,000 cycles, THE Controller SHALL display maintenance reminder in Service Mode

### Requirement 30: Display Content and Formatting

**User Story:** As an operator, I want clear and informative display content, so that I can understand system status at a glance.

#### Acceptance Criteria

1. WHEN in IDLE state, THE Controller SHALL display:
   - Line 1: "IMESA DRYER READY"
   - Line 2: Current temperature
   - Line 3: Last program name
   - Line 4: Status message or prompt
2. WHEN in cycle operation, THE Controller SHALL display:
   - Line 1: Program name and cycle state
   - Line 2: Current temperature and target temperature
   - Line 3: Remaining time (MM:SS format)
   - Line 4: Progress bar or percentage
3. WHEN in FAULT state, THE Controller SHALL display:
   - Line 1: "FAULT DETECTED"
   - Line 2: Fault code and description
   - Line 3: Timestamp or event number
   - Line 4: "Press OK to acknowledge"
4. WHEN in menu navigation, THE Controller SHALL display:
   - Line 1: Menu title
   - Lines 2-3: Menu items with selection indicator
   - Line 4: Navigation hints
5. THE Controller SHALL use consistent abbreviations for display text
6. THE Controller SHALL right-align numeric values for readability
7. THE Controller SHALL use degree symbol (°) for temperature display
8. THE Controller SHALL use progress indicators (bars, percentages) for long operations
9. THE Controller SHALL flash critical messages (faults, warnings) at 1 Hz rate
10. THE Controller SHALL clear display content completely before writing new screen to prevent artifacts

### Requirement 31: Cycle Statistics and Usage Tracking

**User Story:** As a maintenance manager, I want usage statistics, so that I can schedule preventive maintenance appropriately.

#### Acceptance Criteria

1. THE Controller SHALL track and store the following statistics in EEPROM:
   - Total cycle count
   - Total operating hours
   - Total heater activation hours
   - Total motor operating hours
   - Cycles per program type
2. THE Controller SHALL increment cycle count when a cycle completes successfully
3. THE Controller SHALL accumulate operating hours based on cycle duration
4. THE Controller SHALL accumulate heater hours when Heater_Relay is active
5. THE Controller SHALL accumulate motor hours when either motor relay is active
6. WHERE Service Mode is active, THE Controller SHALL display usage statistics
7. THE Controller SHALL provide statistics reset function in Service Mode requiring confirmation
8. THE Controller SHALL update statistics in EEPROM at cycle completion (not during cycle)
9. THE Controller SHALL use wear-leveling for statistics storage to extend EEPROM life
10. WHERE statistics exceed display range, THE Controller SHALL display in appropriate units (hours, thousands of cycles)

### Requirement 32: IMESA ES Series Behavior Reverse-Engineering

**User Story:** As a product manager, I want the controller to replicate IMESA ES Series behavior, so that operators familiar with IMESA dryers can use this controller without retraining.

#### Acceptance Criteria

1. THE Controller SHALL replicate IMESA ES Series user interface terminology where known
2. THE Controller SHALL replicate IMESA ES Series menu structure where documented
3. THE Controller SHALL replicate IMESA ES Series program names and parameters where available
4. THE Controller SHALL replicate IMESA ES Series fault codes and messages where documented
5. THE Controller SHALL replicate IMESA ES Series cycle flow (heating, drying, cooldown) where observed
6. THE Controller SHALL replicate IMESA ES Series drum reversing patterns where documented
7. THE Controller SHALL replicate IMESA ES Series safety interlocks and fault responses where known
8. WHERE IMESA ES Series behavior is unknown, THE Controller SHALL implement industrial best practices
9. WHERE IMESA ES Series behavior is proprietary, THE Controller SHALL implement functionally equivalent behavior
10. THE Controller SHALL document all assumptions and inferences made about IMESA ES Series behavior
11. THE Controller SHALL identify areas requiring validation against actual IMESA ES Series hardware
12. THE Controller SHALL NOT copy proprietary IMESA firmware code or trade secrets

### Requirement 33: Research and Documentation Requirements

**User Story:** As a development team lead, I want clear documentation of reverse-engineering scope, so that the team understands what is known versus inferred.

#### Acceptance Criteria

1. THE development team SHALL research IMESA ES Series documentation including:
   - User manuals
   - Service manuals
   - Installation guides
   - Parts catalogs
   - Any available technical specifications
2. THE development team SHALL document findings from IMESA ES Series research in a separate reference document
3. THE development team SHALL categorize each behavioral requirement as:
   - Confirmed (from official documentation)
   - Observed (from physical hardware testing)
   - Inferred (from industry standards and best practices)
   - Unknown (requires further investigation)
4. THE development team SHALL identify IMESA ES Series features that cannot be replicated due to:
   - Proprietary technology
   - Unavailable documentation
   - Hardware limitations
   - Safety or legal constraints
5. THE development team SHALL document all assumptions made during implementation
6. THE development team SHALL create a validation test plan comparing controller behavior to IMESA ES Series behavior
7. THE development team SHALL identify test cases requiring access to actual IMESA ES Series hardware
8. THE development team SHALL document any deviations from IMESA ES Series behavior with justification
9. THE development team SHALL maintain a list of open questions requiring clarification
10. THE development team SHALL NOT reverse-engineer IMESA firmware through decompilation or disassembly

### Requirement 34: Default Program Parameters

**User Story:** As a system integrator, I want sensible default program parameters, so that the controller is usable immediately after installation.

#### Acceptance Criteria

1. THE Controller SHALL provide the following default Program parameters:

   **Towels Program:**

   - Target temperature: 75°C
   - Cycle duration: 45 minutes
   - Forward time: 30 seconds
   - Reverse time: 30 seconds
   - Pause time: 2 seconds
   - Cooldown: 10 minutes

   **Bed Sheets Program:**

   - Target temperature: 70°C
   - Cycle duration: 50 minutes
   - Forward time: 40 seconds
   - Reverse time: 20 seconds
   - Pause time: 2 seconds
   - Cooldown: 10 minutes

   **Mixed Load Program:**

   - Target temperature: 65°C
   - Cycle duration: 40 minutes
   - Forward time: 35 seconds
   - Reverse time: 25 seconds
   - Pause time: 2 seconds
   - Cooldown: 8 minutes

   **Heavy Cotton Program:**

   - Target temperature: 80°C
   - Cycle duration: 60 minutes
   - Forward time: 45 seconds
   - Reverse time: 15 seconds
   - Pause time: 2 seconds
   - Cooldown: 12 minutes

   **Delicate Load Program:**

   - Target temperature: 50°C
   - Cycle duration: 30 minutes
   - Forward time: 20 seconds
   - Reverse time: 40 seconds
   - Pause time: 3 seconds
   - Cooldown: 8 minutes

   **Work Wear Program:**

   - Target temperature: 75°C
   - Cycle duration: 55 minutes
   - Forward time: 35 seconds
   - Reverse time: 25 seconds
   - Pause time: 2 seconds
   - Cooldown: 10 minutes

2. THE Controller SHALL use these default parameters when EEPROM is uninitialized or corrupted
3. THE Controller SHALL allow all default parameters to be modified in Service Mode
4. THE Controller SHALL provide a "Restore Factory Defaults" function that resets all programs to these values

### Requirement 35: Default PID Parameters

**User Story:** As a control engineer, I want conservative default PID parameters, so that the system is stable on first startup even if not optimally tuned.

#### Acceptance Criteria

1. THE Controller SHALL use the following default PID parameters:
   - Proportional gain (Kp): 2.0
   - Integral gain (Ki): 0.5
   - Derivative gain (Kd): 1.0
   - Sample time: 1000 milliseconds
   - Output range: 0 to 100
2. THE Controller SHALL use these default parameters when EEPROM is uninitialized or corrupted
3. THE Controller SHALL allow PID parameters to be modified in Service Mode
4. THE Controller SHALL validate PID parameter ranges before saving:
   - Kp: 0.1 to 10.0
   - Ki: 0.0 to 5.0
   - Kd: 0.0 to 5.0
5. THE Controller SHALL provide a "Restore Default PID" function in Service Mode
6. THE Controller SHALL display current PID parameters in Service Mode with 2 decimal places
7. THE Controller SHALL allow PID parameter adjustment in increments of 0.1
8. WHEN PID parameters are modified, THE Controller SHALL require a test cycle before saving permanently
9. THE Controller SHALL provide a "Test PID" function that runs a 10-minute test cycle with new parameters
10. WHEN test cycle completes, THE Controller SHALL display temperature stability metrics and prompt to save or discard

### Requirement 36: Safety Interlock Summary

**User Story:** As a safety certification engineer, I want a comprehensive summary of all safety interlocks, so that I can verify compliance with industrial safety standards.

#### Acceptance Criteria

1. THE Controller SHALL implement the following safety interlocks at all times:
   - Door open → All outputs OFF within 100ms
   - Temperature sensor fault → Heater OFF immediately
   - Over-temperature (>100°C) → Heater OFF immediately, motor continues
   - Stop key pressed → All outputs OFF within 200ms
   - Both motor relays active simultaneously → All outputs OFF, enter FAULT state
   - Invalid temperature reading → Heater OFF immediately
   - LCD communication failure → Enter FAULT state, all outputs OFF
   - Watchdog timeout → System reset
2. THE Controller SHALL NOT allow heater activation when:
   - Door is open
   - Temperature sensor is faulted
   - Temperature exceeds 100°C
   - System is in FAULT state
   - Temperature reading is invalid
3. THE Controller SHALL NOT allow motor activation when:
   - Door is open
   - System is in FAULT state
4. THE Controller SHALL enforce minimum 2 second pause between motor direction changes
5. THE Controller SHALL implement all safety interlocks in hardware-independent manner (not relying on relay hardware interlocks)
6. THE Controller SHALL test all safety interlocks during startup self-test where possible
7. THE Controller SHALL provide safety interlock test functions in Service Mode
8. THE Controller SHALL log all safety interlock activations in fault log
9. THE Controller SHALL NOT provide any override or bypass for safety interlocks except in Service Mode diagnostic tests
10. WHERE Service Mode diagnostic tests bypass safety interlocks, THE Controller SHALL display prominent warning and require explicit confirmation

### Requirement 37: Code Quality and Maintainability

**User Story:** As a software maintenance engineer, I want well-structured code, so that the system can be maintained and enhanced over time.

#### Acceptance Criteria

1. THE Controller firmware SHALL be organized into logical modules:
   - Main control loop
   - State machine
   - Hardware abstraction layer (sensors, outputs, HMI)
   - PID controller
   - Menu system
   - EEPROM management
   - Fault handling
   - Program management
2. THE firmware SHALL use meaningful variable and function names
3. THE firmware SHALL include header comments for each module describing purpose and responsibilities
4. THE firmware SHALL include function comments describing parameters, return values, and side effects
5. THE firmware SHALL use const and PROGMEM qualifiers for read-only data
6. THE firmware SHALL avoid global variables where possible, using encapsulation
7. THE firmware SHALL use enumerations for state definitions and fault codes
8. THE firmware SHALL implement hardware abstraction to facilitate testing and porting
9. THE firmware SHALL include compile-time configuration options for hardware variations
10. THE firmware SHALL follow consistent coding style (indentation, bracing, naming conventions)
11. THE firmware SHALL include version number and build date in startup display
12. THE firmware SHALL compile without warnings using Arduino IDE or PlatformIO

### Requirement 38: Testing and Validation Requirements

**User Story:** As a quality assurance engineer, I want comprehensive testing requirements, so that the system can be validated before deployment.

#### Acceptance Criteria

1. THE development team SHALL create a test plan covering:
   - Unit tests for PID controller
   - Unit tests for state machine transitions
   - Integration tests for sensor reading and filtering
   - Integration tests for safety interlocks
   - System tests for complete cycle execution
   - Hardware-in-the-loop tests with actual sensors and relays
2. THE test plan SHALL include test cases for:
   - Normal cycle operation for each program
   - Door opening during each cycle state
   - Temperature sensor failure during operation
   - Over-temperature condition
   - Stop key during operation
   - Power loss and recovery
   - Menu navigation and parameter editing
   - PID auto-tune procedure
   - All diagnostic tests
   - EEPROM corruption recovery
3. THE test plan SHALL define acceptance criteria for each test case
4. THE test plan SHALL include performance tests for:
   - Control loop timing
   - Response time to door open
   - Response time to stop key
   - Temperature control stability
   - Display update rate
5. THE test plan SHALL include stress tests for:
   - Continuous operation for 8 hours
   - Rapid start/stop cycles
   - Repeated door open/close
   - Temperature sensor noise injection
6. THE test plan SHALL include comparison tests against IMESA ES Series behavior where hardware is available
7. THE development team SHALL document test results and any deviations from expected behavior
8. THE development team SHALL perform regression testing after any firmware modifications
9. THE development team SHALL validate all safety interlocks with physical hardware before deployment
10. THE development team SHALL create a commissioning checklist for field installation

### Requirement 39: Performance Requirements

**User Story:** As a system architect, I want defined performance requirements, so that the system meets industrial responsiveness standards.

#### Acceptance Criteria

1. THE Controller SHALL execute main control loop with cycle time not exceeding 100 milliseconds
2. THE Controller SHALL respond to door open event within 100 milliseconds from sensor state change
3. THE Controller SHALL respond to Stop key press within 200 milliseconds from key detection
4. THE Controller SHALL update LCD display within 500 milliseconds of state change
5. THE Controller SHALL complete startup initialization within 3 seconds of power application
6. THE Controller SHALL read and filter temperature within 1 second of request
7. THE Controller SHALL complete PID calculation within 50 milliseconds
8. THE Controller SHALL scan keypad with maximum 100 millisecond latency
9. THE Controller SHALL maintain timing accuracy within ±2% for cycle duration
10. THE Controller SHALL maintain temperature control accuracy within ±3°C of setpoint during steady state
11. THE Controller SHALL achieve temperature setpoint within 10 minutes of cycle start under normal conditions
12. THE Controller SHALL complete EEPROM write operations within 100 milliseconds

### Requirement 40: Memory and Resource Constraints

**User Story:** As an embedded systems engineer, I want defined resource constraints, so that the firmware fits within Arduino Nano limitations.

#### Acceptance Criteria

1. THE Controller firmware SHALL occupy no more than 28KB of flash memory (leaving 4KB margin)
2. THE Controller firmware SHALL use no more than 1.5KB of SRAM (leaving 0.5KB margin)
3. THE Controller SHALL use no more than 512 bytes of EEPROM for configuration data
4. THE Controller SHALL use no more than 512 bytes of EEPROM for fault log and statistics
5. THE Controller SHALL implement stack overflow detection if feasible
6. THE Controller SHALL minimize dynamic memory allocation (avoid malloc/free)
7. THE Controller SHALL use static allocation for all data structures where possible
8. THE Controller SHALL store constant strings in PROGMEM to conserve SRAM
9. THE Controller SHALL store program definitions in PROGMEM to conserve SRAM
10. THE Controller SHALL report memory usage (flash, SRAM) during compilation
11. WHERE memory constraints are exceeded, THE Controller SHALL prioritize safety-critical functions over convenience features

### Requirement 41: Environmental and Operating Conditions

**User Story:** As an installation engineer, I want defined environmental requirements, so that I can ensure proper operating conditions.

#### Acceptance Criteria

1. THE Controller SHALL operate correctly in ambient temperature range 5°C to 40°C
2. THE Controller SHALL operate correctly in relative humidity range 20% to 80% non-condensing
3. THE Controller SHALL tolerate supply voltage range 4.5V to 5.5V (USB power or regulated supply)
4. THE Controller SHALL tolerate supply voltage transients up to ±10% for 100 milliseconds
5. THE Controller SHALL implement brown-out detection if supported by hardware
6. THE Controller SHALL reset safely if supply voltage drops below 4.0V
7. THE Controller SHALL tolerate electrical noise typical of industrial laundry environments
8. THE Controller SHALL use shielded cables for temperature sensor connections
9. THE Controller SHALL implement software debouncing for all digital inputs
10. THE Controller SHALL use optically isolated relay drivers if high-voltage switching is required (implementation detail)
11. WHERE installation environment exceeds specified conditions, THE Controller SHALL display warning in Service Mode

### Requirement 42: User Feedback and Indicators

**User Story:** As an operator, I want clear feedback for my actions, so that I know the system has responded to my inputs.

#### Acceptance Criteria

1. WHEN a valid key is pressed, THE Controller SHALL provide immediate visual feedback on LCD
2. WHEN an invalid key is pressed, THE Controller SHALL ignore the input without feedback
3. WHEN a cycle starts, THE Controller SHALL display "CYCLE STARTING" message for 2 seconds
4. WHEN a cycle completes, THE Controller SHALL display "CYCLE COMPLETE" message until acknowledged
5. WHEN a fault occurs, THE Controller SHALL flash the fault message at 1 Hz rate
6. WHEN a parameter is saved, THE Controller SHALL display "SAVED" confirmation for 2 seconds
7. WHEN an invalid parameter value is entered, THE Controller SHALL display "INVALID VALUE" error for 2 seconds
8. WHERE hardware supports buzzer, THE Controller SHALL provide audible feedback for:
   - Cycle complete (3 short beeps)
   - Fault detected (continuous beeping until acknowledged)
   - Key press (single short beep)
9. WHERE hardware supports status LED, THE Controller SHALL indicate:
   - Normal operation (green LED steady)
   - Cycle active (green LED blinking)
   - Fault state (red LED steady)
10. THE Controller SHALL provide progress indication during long operations (auto-tune, cycle execution)

### Requirement 43: Accessibility and Usability

**User Story:** As a laundry facility manager, I want the interface to be usable by operators with varying technical skills, so that training time is minimized.

#### Acceptance Criteria

1. THE Controller SHALL use simple, clear language in all display messages
2. THE Controller SHALL avoid technical jargon in operator-facing menus
3. THE Controller SHALL provide context-sensitive help text on the fourth LCD line
4. THE Controller SHALL use consistent navigation patterns throughout all menus
5. THE Controller SHALL provide visual indication of current selection in all menus
6. THE Controller SHALL allow navigation using only Up, Down, OK, and Cancel keys for basic operation
7. THE Controller SHALL provide confirmation prompts for destructive actions (clear log, factory reset)
8. THE Controller SHALL allow operation with minimal reference to documentation for common tasks
9. THE Controller SHALL display units (°C, minutes) with all numeric values
10. THE Controller SHALL use intuitive icons or symbols where LCD character set supports them
11. WHERE language localization is desired, THE Controller SHALL organize all display strings for easy translation

### Requirement 44: Maintenance and Serviceability

**User Story:** As a service technician, I want easy access to diagnostic information, so that I can troubleshoot problems efficiently.

#### Acceptance Criteria

1. WHERE Service Mode is active, THE Controller SHALL display:
   - Firmware version and build date
   - Total operating hours
   - Cycle count
   - Relay activation counts
   - Last fault code and timestamp
   - Current sensor readings with high precision
2. THE Controller SHALL provide diagnostic tests for each hardware component
3. THE Controller SHALL provide real-time monitoring of all inputs and outputs
4. THE Controller SHALL provide access to fault log with detailed information
5. THE Controller SHALL provide PID tuning interface with real-time response display
6. THE Controller SHALL provide EEPROM data viewing and editing capabilities
7. THE Controller SHALL provide factory reset function with confirmation
8. THE Controller SHALL provide firmware version display on startup screen
9. THE Controller SHALL log all Service Mode access with timestamp
10. THE Controller SHALL provide a "System Info" screen showing hardware configuration and status
11. WHERE remote diagnostics are desired, THE Controller SHALL reserve serial port for future communication interface

### Requirement 45: Reverse-Engineering Validation and Compliance

**User Story:** As a project manager, I want clear validation criteria for reverse-engineering accuracy, so that I can assess how closely the controller matches IMESA ES Series behavior.

#### Acceptance Criteria

1. THE development team SHALL create a comparison matrix documenting:
   - IMESA ES Series observed behavior
   - Controller implemented behavior
   - Match status (exact, approximate, different, unknown)
   - Justification for any differences
2. THE development team SHALL identify the following aspects for validation:
   - Menu structure and terminology
   - Program names and default parameters
   - Cycle state sequence and timing
   - Fault codes and messages
   - Safety interlock behavior
   - Display layout and content
   - Key assignments and navigation
   - Temperature control characteristics
   - Drum reversing patterns
3. THE development team SHALL categorize validation status as:
   - Validated (tested against actual IMESA hardware)
   - Documented (confirmed from IMESA documentation)
   - Inferred (based on industry standards)
   - Assumed (best engineering judgment)
   - Unknown (requires further research)
4. THE development team SHALL document any IMESA ES Series features that are:
   - Not implemented (out of scope)
   - Cannot be implemented (hardware limitations)
   - Intentionally different (safety or legal reasons)
5. THE development team SHALL maintain a list of open questions requiring:
   - Access to IMESA documentation
   - Access to IMESA hardware for testing
   - Clarification from IMESA technical support
6. THE development team SHALL document all proprietary IMESA features that cannot be replicated
7. THE development team SHALL ensure no IMESA copyrighted material is included in firmware or documentation
8. THE development team SHALL ensure no IMESA trademarks are used inappropriately
9. THE development team SHALL include disclaimer that this is a reverse-engineered compatible controller, not an official IMESA product
10. THE development team SHALL document the legal and ethical boundaries of the reverse-engineering effort

### Requirement 46: Future Enhancement Provisions

**User Story:** As a product manager, I want provisions for future enhancements, so that the controller can be extended without major redesign.

#### Acceptance Criteria

1. THE Controller SHALL reserve EEPROM address space for future data storage needs
2. THE Controller SHALL reserve unused Arduino Nano pins for future expansion
3. THE Controller SHALL implement modular firmware architecture to facilitate feature additions
4. THE Controller SHALL use version numbering scheme supporting major and minor revisions
5. THE Controller SHALL implement EEPROM data format versioning to support migration
6. WHERE serial communication is desired, THE Controller SHALL reserve UART pins (D0, D1)
7. WHERE additional sensors are desired, THE Controller SHALL reserve analog input pins
8. WHERE network connectivity is desired, THE Controller SHALL document interface requirements for external module
9. THE Controller SHALL implement menu system extensibility for adding new menu items
10. THE Controller SHALL implement program table extensibility for adding new programs
11. THE Controller SHALL document all reserved resources and expansion points in technical documentation
12. THE Controller SHALL maintain backward compatibility with previous EEPROM data formats where feasible

### Requirement 47: Documentation Deliverables

**User Story:** As a project stakeholder, I want comprehensive documentation, so that the system can be installed, operated, and maintained effectively.

#### Acceptance Criteria

1. THE development team SHALL deliver the following documentation:
   - Requirements specification (this document)
   - Design specification
   - User manual
   - Service manual
   - Installation guide
   - Test plan and test results
   - Source code with inline comments
   - Hardware interface specification
   - EEPROM memory map
   - Reverse-engineering comparison matrix
2. THE User Manual SHALL include:
   - Safety warnings and precautions
   - Operating instructions for all modes
   - Program descriptions and recommendations
   - Troubleshooting guide for common issues
   - Fault code reference
3. THE Service Manual SHALL include:
   - Service Mode access and functions
   - Diagnostic procedures
   - PID tuning procedures
   - Parameter adjustment procedures
   - Fault log interpretation
   - Component replacement procedures
   - Calibration procedures (if applicable)
4. THE Installation Guide SHALL include:
   - Hardware requirements
   - Wiring diagrams
   - Pin assignments
   - Power supply requirements
   - Environmental requirements
   - Commissioning checklist
   - Initial setup procedures
5. THE Hardware Interface Specification SHALL include:
   - Pin assignments with descriptions
   - Electrical specifications for all I/O
   - Sensor specifications and wiring
   - Relay specifications and ratings
   - LCD and keypad specifications
   - Recommended component part numbers
6. THE Source Code SHALL include:
   - Module organization and dependencies
   - Build instructions
   - Configuration options
   - Version history
   - Known issues and limitations
7. ALL documentation SHALL be maintained in version control
8. ALL documentation SHALL include version number and revision date
9. ALL documentation SHALL be reviewed and approved before release
10. ALL documentation SHALL be provided in PDF format for distribution

### Requirement 48: Compliance and Standards

**User Story:** As a compliance officer, I want the controller to meet relevant industrial standards, so that it can be legally deployed in commercial environments.

#### Acceptance Criteria

1. THE Controller design SHALL consider the following standards (compliance verification is implementation responsibility):
   - IEC 61010 (Safety requirements for electrical equipment)
   - IEC 61000 (Electromagnetic compatibility)
   - UL 2158 (Commercial laundry equipment)
   - CE marking requirements (if applicable)
2. THE Controller SHALL implement safety interlocks consistent with industrial machinery safety standards
3. THE Controller SHALL implement fault handling consistent with fail-safe design principles
4. THE Controller SHALL use relay ratings appropriate for the connected loads
5. THE Controller SHALL implement electrical isolation between control circuits and power circuits (implementation detail)
6. THE documentation SHALL include safety warnings consistent with industrial equipment standards
7. THE documentation SHALL include electrical hazard warnings
8. THE documentation SHALL include proper grounding and bonding requirements
9. THE development team SHALL identify any standards compliance testing required before deployment
10. THE development team SHALL document any standards compliance limitations or exclusions
11. WHERE certification is required, THE development team SHALL provide necessary documentation and support for certification process

### Requirement 49: Known Limitations and Constraints

**User Story:** As a system architect, I want documented limitations, so that stakeholders have realistic expectations of controller capabilities.

#### Acceptance Criteria

1. THE Controller SHALL document the following known limitations:
   - Arduino Nano memory constraints limit program count and feature complexity
   - DS18B20 sensor accuracy is ±0.5°C, limiting temperature control precision
   - 20x4 LCD character display limits information density compared to graphical displays
   - 4x4 keypad limits input options compared to touchscreen interfaces
   - No real-time clock without external RTC module, limiting timestamp accuracy
   - No network connectivity without external module
   - No data logging beyond fault log without external storage
   - Single temperature sensor provides drum air temperature only, not fabric temperature
   - Relay-based control limits switching frequency compared to solid-state control
2. THE Controller SHALL document the following design constraints:
   - No water temperature control (out of scope)
   - No moisture sensing (hardware not specified)
   - No load weight sensing (hardware not specified)
   - No automatic door locking (hardware not specified)
   - No remote monitoring without additional hardware
   - No multi-language support in base implementation
   - No user authentication beyond service code
3. THE Controller SHALL document the following operational constraints:
   - Cycle progress is lost on power failure
   - EEPROM has limited write endurance (100,000 cycles typical)
   - Temperature sensor cable length limited to 3 meters for reliable operation
   - I2C bus length limited to 1 meter for reliable LCD communication
4. THE Controller SHALL document workarounds or mitigations for known limitations where applicable
5. THE Controller SHALL document which limitations could be addressed in future hardware revisions

### Requirement 50: Reverse-Engineering Scope and Boundaries

**User Story:** As a legal advisor, I want clear documentation of reverse-engineering scope, so that intellectual property boundaries are respected.

#### Acceptance Criteria

1. THE reverse-engineering effort SHALL be limited to:
   - Observable user interface behavior
   - Documented specifications from public sources
   - Industry-standard control algorithms
   - Functional behavior replication without code copying
2. THE reverse-engineering effort SHALL NOT include:
   - Decompilation or disassembly of IMESA firmware
   - Extraction of proprietary algorithms from IMESA hardware
   - Copying of IMESA copyrighted documentation or materials
   - Use of IMESA trade secrets or confidential information
   - Circumvention of IMESA security measures or access controls
3. THE Controller SHALL be an independent implementation based on:
   - Publicly available information
   - Industry best practices
   - Original engineering design
   - Functional requirements analysis
4. THE Controller documentation SHALL include:
   - Clear statement that this is not an IMESA product
   - Disclaimer of any affiliation with IMESA
   - Statement that IMESA trademarks are property of IMESA
   - Attribution of any IMESA documentation used as reference
5. THE Controller SHALL NOT use IMESA branding, logos, or trademarks
6. THE Controller SHALL NOT claim compatibility or endorsement by IMESA
7. THE Controller SHALL NOT interfere with IMESA intellectual property rights
8. THE development team SHALL consult legal counsel if any intellectual property questions arise
9. THE development team SHALL document all information sources used in reverse-engineering
10. THE development team SHALL maintain clean-room development practices where proprietary information boundaries exist

## Reverse-Engineering Analysis

### What is Known from Source Material

The following information is explicitly provided and forms the confirmed foundation of the specification:

1. **Hardware Platform**: Arduino Nano with ATmega328P microcontroller
2. **Pin Assignments**: All digital I/O pins are specified for sensors, outputs, and interfaces
3. **Sensor Type**: DS18B20 one-wire digital temperature sensor on pin 12
4. **Display Type**: 20x4 I2C LCD using standard Arduino libraries
5. **Input Device**: 4x4 matrix keypad with defined key mapping
6. **Control Outputs**: Three relay outputs for heater, motor forward, and motor reverse
7. **Safety Input**: Door sensor with defined electrical characteristics
8. **Target Behavior**: Controller should replicate IMESA ES Series Industrial Tumble Dryer behavior
9. **Scope Exclusions**: No water temperature control, focus on drum drying only
10. **Control Requirements**: PID temperature control with auto-tune capability required

### What Must Be Inferred

The following aspects are not explicitly documented and must be inferred from industrial dryer best practices:

1. **IMESA ES Series Specifics**:

   - Exact menu structure and terminology
   - Specific program names and parameters
   - Fault code numbering and messages
   - Display layout and formatting conventions
   - Drum reversing pattern timing
   - Cooldown behavior and duration
   - End-of-cycle behavior (anti-crease, alerts)

2. **Control Characteristics**:

   - PID tuning parameters appropriate for industrial dryer thermal mass
   - Temperature control deadband and hysteresis values
   - Heater switching frequency limits
   - Motor direction change timing requirements

3. **Safety Behavior**:

   - Specific fault response sequences
   - Fault latching versus auto-recovery decisions
   - Service mode access control requirements
   - Diagnostic test procedures

4. **User Interface Details**:
   - Menu hierarchy depth and organization
   - Navigation patterns and key assignments beyond basic directions
   - Parameter editing workflows
   - Confirmation and cancellation patterns

### What Must Be Validated During Development

The following aspects require validation through testing and research:

1. **IMESA ES Series Comparison**:

   - Access to IMESA user manual for terminology and menu structure
   - Access to IMESA service manual for technical specifications
   - Observation of actual IMESA ES Series hardware behavior if available
   - Comparison of cycle timing and temperature profiles
   - Validation of fault codes and messages

2. **Thermal Characteristics**:

   - Actual dryer thermal time constant for PID tuning
   - Heater power and heating rate
   - Cooldown rate with motor running
   - Temperature sensor placement and response time

3. **Mechanical Characteristics**:

   - Motor direction change timing requirements
   - Drum inertia and stopping time
   - Optimal reversing patterns for different fabric types
   - Motor duty cycle limitations

4. **Electrical Characteristics**:
   - Relay contact ratings and lifetime
   - Heater current and power requirements
   - Motor current and power requirements
   - Electrical noise environment and filtering needs

### What Remains Unknown Until Hardware Analysis

The following information cannot be determined without access to IMESA hardware or documentation:

1. **Proprietary Features**:

   - Any patented control algorithms
   - Proprietary sensor fusion techniques
   - Advanced features not visible in basic operation
   - Factory calibration procedures

2. **Undocumented Behavior**:

   - Service mode hidden functions
   - Factory test modes
   - Diagnostic codes and procedures
   - Calibration data and procedures

3. **Implementation Details**:
   - Exact PID parameters used by IMESA
   - Specific filtering algorithms for sensor noise
   - Exact timing values for all operations
   - Memory organization and data structures

### Research Tasks for Development Team

The development team must complete the following research tasks:

1. **Documentation Research**:

   - Obtain IMESA ES Series user manual
   - Obtain IMESA ES Series service manual if available
   - Research IMESA ES Series specifications from public sources
   - Review industrial dryer standards and best practices

2. **Competitive Analysis**:

   - Study similar industrial dryer controllers
   - Identify common features and terminology
   - Understand industry-standard program types and parameters
   - Research typical fault handling approaches

3. **Technical Research**:

   - Research PID tuning for industrial dryer applications
   - Research drum reversing patterns and their effects
   - Research temperature sensor placement and filtering
   - Research relay lifetime and switching frequency limits

4. **Validation Planning**:
   - Define test cases for comparing to IMESA behavior
   - Identify aspects requiring IMESA hardware access
   - Plan bench testing with representative thermal load
   - Plan field testing in actual laundry environment

### Assumptions and Engineering Decisions

The following assumptions have been made in this specification:

1. **Control Approach**: PID control is appropriate for this application (industry standard)
2. **Safety Philosophy**: Fail-safe design with immediate shutdown on any fault (conservative approach)
3. **User Interface**: Menu-driven interface is appropriate for industrial operators (common pattern)
4. **Program Count**: Six programs are sufficient for typical commercial laundry (based on common practice)
5. **Temperature Range**: 30°C to 85°C covers typical industrial drying needs (industry standard)
6. **Cycle Duration**: 5 to 180 minutes covers typical drying cycles (industry standard)
7. **Reversing Pattern**: Configurable forward/reverse timing is beneficial (prevents tangling)
8. **Cooldown**: Automatic cooldown is required for safety (industry best practice)
9. **Fault Logging**: 20 fault entries provide adequate history (reasonable for EEPROM constraints)
10. **Service Code**: Simple numeric code is adequate for service mode protection (common approach)

These assumptions should be validated against IMESA ES Series behavior where possible and adjusted if conflicts are discovered.

## Conclusion

This requirements specification provides a comprehensive foundation for developing an Arduino Nano-based industrial tumble dryer controller that reverse-engineers the IMESA ES Series behavior. The specification is structured to clearly distinguish between confirmed requirements, inferred behavior, and areas requiring further research.

The development team must treat this as a living document, updating it as additional information about IMESA ES Series behavior becomes available through research, documentation review, or hardware testing. All deviations from IMESA behavior must be documented with justification, and all assumptions must be validated during the development and testing phases.

The specification prioritizes safety, reliability, and industrial-grade quality while acknowledging the constraints of the Arduino Nano platform and the limitations of reverse-engineering without access to proprietary information.
