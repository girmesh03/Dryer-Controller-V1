---
inclusion: always
---

# Task Execution Protocol - MANDATORY

This protocol defines **six mandatory steps** that MUST be followed when executing **each task** listed in `.kiro/specs/imesa-dryer-controller/tasks.md`. No shortcuts. No exceptions.

---

## Step 1: Pre-Git Requirement (Before Task Execution)

**Purpose**: Ensure complete and accurate Git branch information to prevent issues during new branch creation and checkout.

**Actions**:

1. **Check Current State**:

   - Execute `git status` to check:
     - Current branch name
     - Uncommitted changes (staged/unstaged files)
     - Untracked files
   - Execute `git branch -vv` to display:
     - All local branches
     - Branch tracking information
     - Ahead/behind status relative to remote

2. **Update Remote Information**:

   - Execute `git fetch origin` to update remote tracking information
   - Verify remote branches are synchronized

3. **Handle Uncommitted Changes**:

   - **IF uncommitted changes exist**:
     - Stay on the current branch where uncommitted changes exist
     - Execute `git add .` to stage all changes
     - Execute `git commit -m "descriptive commit message"` with clear description
     - Execute `git push origin <current-branch>` to push to remote
     - Execute `git checkout main` (or appropriate base branch)
     - Execute `git merge <feature-branch>` to merge changes
     - Execute `git push origin main` to push merged changes
     - Execute `git branch -d <feature-branch>` to delete local branch
     - Execute `git push origin --delete <feature-branch>` to delete remote branch
     - **Think twice before acting** - verify branch names and merge targets

4. **Synchronize Local with Remote**:

   - **IF local branch is behind remote**:
     - Execute `git pull origin <branch>` to synchronize
   - **IF merge conflicts detected**:
     - **HALT immediately**
     - Prompt user to resolve conflicts manually
     - Wait for user confirmation before proceeding

5. **Create Feature Branch**:

   - Execute `git checkout -b <descriptive-branch-name>`
   - Use clear, descriptive branch names matching task number and description

6. **Verify Clean State**:
   - Execute `git status` to confirm clean working directory
   - Confirm correct branch is checked out
   - Proceed to Step 2 only after verification

---

## Step 2: Comprehensive and Extremely Deep Codebase Analysis

**Purpose**: Capture every single detail of the codebase to ensure absolute alignment with requirements, designs, specifications, and constraints.

**Critical Analysis Areas**:

### Codebase Analysis (Complete Deep Dive):

1. **Existing File Structure Inventory**:

   - List ALL files in `src/`, `include/`, `lib/`, `test/`
   - Identify which modules exist: hal/, control/, app/, storage/, util/
   - Check for `config.h` in `include/` directory
   - Verify `platformio.ini` configuration
   - Note any deviations from planned structure in `.kiro/specs/imesa-dryer-controller/design.md`

2. **Existing Code Analysis**:

   - **For each existing .h file**:
     - Read complete file with `readCode` tool
     - Document all class/struct definitions
     - Document all function signatures
     - Document all constants and enums
     - Note all `#include` dependencies
     - Verify header guards are present
   - **For each existing .cpp file**:
     - Read complete file with `readCode` tool
     - Document all implemented functions
     - Note all global variables (should be minimal)
     - Check for PROGMEM usage for constants
     - Verify non-blocking patterns (no `delay()` calls)
   - **For main.cpp**:
     - Analyze `setup()` function: what's initialized, in what order
     - Analyze `loop()` function: timing logic, module calls, watchdog reset
     - Check loop period enforcement (should be 100ms)
     - Verify all modules are instantiated and updated

3. **Interface Compliance Check**:

   - Compare existing interfaces against design document specifications
   - **ISensorHAL**: Verify methods match design (readDoorSensor, readTemperature, isTemperatureSensorValid, update)
   - **IOutputHAL**: Verify methods match design (setHeater, setMotorForward, setMotorReverse, emergencyStop, getters)
   - **IDisplayHAL**: Verify methods match design (init, clear, setCursor, print, setBacklight)
   - **IKeypadHAL**: Verify methods match design (getKey, update, Key enum)
   - **IPIDController**: Verify methods match design (setSetpoint, setParams, compute, reset, getOutput)
   - **ISafetyInterlock**: Verify methods match design (update, isSafe, getFaultCode, clearFault, canActivateHeater, canActivateMotor)
   - **IStateMachine**: Verify methods match design (start, stop, pause, resume, update, getState, getRemainingTime)

4. **Data Structure Verification**:

   - Check if `Program` struct matches design (name[17], targetTemp, duration, forwardTime, reverseTime, pauseTime, cooldownDuration, checksum)
   - Check if `CycleData` struct exists and matches design
   - Check if `FaultEntry` and `FaultLog` structures match design
   - Check if `PIDState` and `PIDParams` structures match design
   - Check if `ReversingState` structure matches design
   - Check if `Statistics` structure matches design
   - Verify all enums: CycleState, FaultCode, Key, MenuContext

5. **Pin Configuration Verification**:

   - Check `config.h` or `platformio.ini` for pin definitions
   - Verify: PIN_TEMP_SENSOR=12, PIN_DOOR_SENSOR=6, PIN_HEATER_RELAY=9, PIN_MOTOR_FWD_RELAY=10, PIN_MOTOR_REV_RELAY=11
   - Verify: Keypad rows A0-A3, columns D2-D5
   - Verify: LCD I2C on A4(SDA), A5(SCL), address 0x27

6. **Library Dependencies Check**:

   - Read `platformio.ini` to verify library versions
   - Confirm: OneWire ^2.3.7, DallasTemperature ^3.11.0, LiquidCrystal_I2C ^1.1.2, Keypad ^3.1.1
   - Check if libraries are actually used in code

7. **Memory Usage Analysis**:

   - Check for PROGMEM usage on constant strings (F() macro)
   - Check for PROGMEM usage on default programs array
   - Verify no dynamic allocation (no `new`, `malloc`, `String` class)
   - Look for bit fields in flag structures
   - Estimate current SRAM usage based on global variables

8. **Safety Implementation Check**:

   - Verify door sensor debouncing (50ms minimum)
   - Verify temperature sensor filtering (moving average, 3+ samples)
   - Verify relay anti-chatter (heater 5s, motor 2s)
   - Verify motor mutual exclusion logic
   - Verify minimum 2s pause between motor direction changes
   - Check for emergency stop implementation (<100ms response)
   - Verify watchdog timer setup and reset calls

9. **Timing Implementation Check**:

   - Verify main loop uses `millis()` for timing (no blocking delays)
   - Check loop period enforcement (100ms target)
   - Verify temperature reads are non-blocking (750ms DS18B20 conversion)
   - Check display updates are throttled (500ms interval)
   - Verify PID updates at 1s intervals

10. **EEPROM Implementation Check**:
    - Check if EEPROM memory map matches design (addresses 0x0000-0x03FF)
    - Verify CRC8 and CRC16 checksum functions exist
    - Check for magic number validation (0xDEADBEEF)
    - Verify program storage with checksum validation
    - Check fault log circular buffer implementation

### Specification Analysis:

**Requirements** (from `.kiro/specs/imesa-dryer-controller/requirements.md`):

1. **Identify Current Task Requirements**:

   - Read task description from `.kiro/specs/imesa-dryer-controller/tasks.md`
   - Extract requirement numbers referenced in task (e.g., "Requirements: 2.1-2.4, 2.10, 27.2")
   - Read EACH referenced requirement section from requirements.md
   - Document ALL acceptance criteria for each requirement
   - Note specific values: timeouts, thresholds, pin numbers, timing constraints

2. **Safety Requirements Analysis**:

   - Requirement 2 (Door Safety Interlock): <100ms response, LOW=open, HIGH=closed, 50ms debounce
   - Requirement 3 (Temperature Sensing): DS18B20 on pin 12, 1s read interval, 3-sample filter, -10°C to 150°C range, 3-strike fault
   - Requirement 4 (Heater Control): Pin 9, HIGH=on, 5s anti-chatter, PID controlled, safety interlocks
   - Requirement 5 (Motor Forward): Pin 10, HIGH=on, 2s anti-chatter, mutual exclusion, 2s pause on direction change
   - Requirement 6 (Motor Reverse): Pin 11, HIGH=on, 2s anti-chatter, mutual exclusion, 2s pause on direction change
   - Requirement 19 (Fault Detection): All fault types, <100ms emergency stop, fault latching
   - Requirement 20 (Over-Temperature): 100°C fault threshold, 80°C safe threshold

3. **HMI Requirements Analysis**:

   - Requirement 7 (LCD Display): 20x4 I2C, address 0x27, A4/A5 pins, 500ms update interval
   - Requirement 8 (Keypad): 4x4 matrix, rows A0-A3, cols D2-D5, 50ms debounce, key mapping
   - Requirement 9 (Main Menu): Hierarchical structure, 60s timeout, navigation keys
   - Requirement 10 (Auto Mode): Program selection, cycle execution, progress display
   - Requirement 11 (Manual Mode): Manual control, 90°C limit, safety interlocks

4. **Control Requirements Analysis**:

   - Requirement 14 (PID Control): Kp/Ki/Kd parameters, anti-windup, 1s update, 50% relay threshold
   - Requirement 15 (PID Auto-Tune): Relay feedback test, Ziegler-Nichols, 15-30 min duration
   - Requirement 16 (Reversing Pattern): Forward/reverse/pause times, 2s minimum pause
   - Requirement 17 (State Machine): 8 states (IDLE, STARTING, HEATING, DRYING, COOLDOWN, COMPLETE, PAUSED, FAULT)
   - Requirement 18 (Cooldown): Heater off, motor continues, configurable duration

5. **Storage Requirements Analysis**:

   - Requirement 12 (Program Definition): 6 programs, PROGMEM defaults, EEPROM overrides, CRC validation
   - Requirement 13 (Parameter Editing): Service mode, validation, restore defaults
   - Requirement 26 (EEPROM Management): Memory map, checksums, factory reset, corruption recovery

6. **Timing Requirements Analysis**:
   - Requirement 27 (Main Loop): 100ms period, non-blocking, task scheduling, watchdog reset
   - Requirement 39 (Response Times): Door <100ms, Stop key <200ms, Display <500ms

**Design** (from `.kiro/specs/imesa-dryer-controller/design.md`):

1. **Architecture Verification**:

   - Confirm layered architecture: Hardware → HAL → Control → Application
   - Verify module decomposition matches design: hal/, control/, app/, storage/, util/
   - Check execution model: 100ms cooperative multitasking, time-sliced
   - Verify timing budget: Critical path ~15ms per loop (15% CPU)

2. **Interface Design Compliance**:

   - For current task, identify which interfaces are involved
   - Read interface definitions from design document
   - Compare with actual code (if exists)
   - Verify method signatures match exactly
   - Check return types, parameter types, const correctness

3. **Data Model Compliance**:

   - Verify struct definitions match design byte-for-byte
   - Check field order (important for EEPROM storage)
   - Verify checksum field placement
   - Confirm PROGMEM usage for default data
   - Check enum value assignments

4. **Algorithm Compliance**:

   - **PID Controller**: Verify formula matches design (P + I + D terms, anti-windup, derivative-on-measurement)
   - **Reversing Pattern**: Verify state machine matches design (FORWARD → PAUSE → REVERSE → PAUSE)
   - **Safety Interlock**: Verify all checks match design (door, temp sensor, temp range, relay conflict)
   - **Temperature Filtering**: Verify moving average implementation (3+ samples)
   - **Debouncing**: Verify 50ms stable time implementation

5. **State Machine Design**:

   - Verify state definitions match design exactly
   - Check transition guards (safety checks before transitions)
   - Verify state behaviors (entry, active, exit actions)
   - Check timeout handling in each state
   - Verify fault handling transitions

6. **EEPROM Design Compliance**:

   - Verify memory map addresses match design exactly
   - Check CRC8/CRC16 algorithm implementations
   - Verify magic number and version handling
   - Check circular buffer implementation for fault log
   - Verify wear leveling strategy

7. **Display Layout Compliance**:

   - Verify screen layouts match design (idle, running, fault, menu, edit)
   - Check text formatting (20 char width, 4 lines)
   - Verify progress bar implementation
   - Check flashing behavior for faults

8. **Timing Design Compliance**:
   - Verify task scheduling matches design table
   - Check frequency and duration for each task
   - Verify priority handling (critical tasks first)
   - Confirm non-blocking patterns throughout

**Constraints** (from `.kiro/steering/tech.md` and design):

1. **Memory Constraints**:

   - Flash: Must fit in <28KB (30,720 bytes available, target 28,500 bytes)
   - SRAM: Must fit in <1.5KB (2,048 bytes available, target 1,550 bytes)
   - EEPROM: 1KB total (1,024 bytes), memory map 0x0000-0x03FF
   - Leave margin: 7% flash, 24% SRAM minimum

2. **Timing Constraints**:

   - Main loop period: 100ms (enforced, non-blocking)
   - Critical path: <15ms per loop
   - Door response: <100ms
   - Stop key response: <200ms
   - Temperature read: <1s (non-blocking)
   - Display update: <500ms
   - Watchdog timeout: 2s

3. **Hardware Constraints**:

   - Arduino Nano: ATmega328P, 16MHz, 5V logic
   - DS18B20: 750ms conversion time, OneWire protocol
   - LCD I2C: PCF8574 backpack, address 0x27
   - Keypad: 4x4 matrix, requires scanning
   - Relays: Mechanical, require anti-chatter protection

4. **Safety Constraints**:

   - Emergency stop: <100ms response time (CRITICAL)
   - Door interlock: Must prevent heater/motor when open
   - Temperature limits: -10°C to 150°C valid, 100°C fault, 80°C safe
   - Relay mutual exclusion: Forward and reverse never both active
   - Motor pause: Minimum 2s between direction changes
   - Heater anti-chatter: Minimum 5s between state changes
   - Motor anti-chatter: Minimum 2s between state changes

5. **Code Constraints**:
   - No dynamic allocation (no `new`, `malloc`, `String` class)
   - All constants in PROGMEM (F() macro for strings)
   - No blocking delays (use `millis()` timing)
   - All inputs debounced (50ms minimum)
   - All EEPROM data CRC validated
   - Header guards in all .h files
   - Dependency injection for testability

**Tasks** (from `.kiro/specs/imesa-dryer-controller/tasks.md`):

1. **Current Task Identification**:

   - Read task number and description
   - Note if task has subtasks (e.g., 2.1, 2.2, 2.3)
   - Identify task category: HAL, Control, Application, Storage, Integration, Testing

2. **Task Deliverables**:

   - List all files to be created (e.g., "Create sensor_hal.h with ISensorHAL interface")
   - List all files to be modified (e.g., "Update main.cpp to instantiate SensorHAL")
   - List all functions to be implemented
   - List all tests to be written (if task includes testing)

3. **Task Dependencies**:

   - Identify prerequisite tasks (must be completed first)
   - Identify dependent tasks (will use this task's output)
   - Check if checkpoint task (requires validation before proceeding)

4. **Task Requirements Reference**:

   - Extract requirement numbers from task description
   - Cross-reference with requirements.md
   - Ensure all acceptance criteria are addressed

5. **Task Success Criteria**:
   - Define what "done" means for this task
   - List verification steps
   - Identify test cases to validate

**Analysis Outcome**:

Document findings in structured format:

```
CODEBASE STATE:
- Existing modules: [list]
- Missing modules: [list]
- Existing interfaces: [list with compliance status]
- Memory usage estimate: Flash [X]KB / 28KB, SRAM [Y]KB / 1.5KB

CURRENT TASK:
- Task number: [N]
- Description: [full description]
- Requirements: [list with acceptance criteria]
- Deliverables: [files to create/modify]
- Dependencies: [prerequisite tasks]

COMPLIANCE VERIFICATION:
- Requirements: [compliant/gaps identified]
- Design: [compliant/gaps identified]
- Constraints: [within limits/concerns]

IMPLEMENTATION READINESS:
- [Ready to proceed / Blockers identified]
- [List any clarifications needed]
```

**Proceed to Step 3 only after completing this comprehensive analysis.**

---

## Step 3: Comprehensive and Extremely Deep Analysis of Previously Implemented Tasks (N - 1)

**Purpose**: Understand all previously implemented tasks to ensure consistency, avoid duplication, and maintain architectural patterns.

**Actions**:

1. **Identify Previous Tasks**:

   - Read `.kiro/specs/imesa-dryer-controller/tasks.md` completely
   - Identify all tasks marked as completed (checked boxes: `- [x]`)
   - For current task N, list all tasks 1 through N-1
   - Note checkpoint tasks (5, 9, 12, 17, 21, 24) and their validation status

2. **Analyze Each Previous Task**:

   **For EACH completed task, perform the following analysis:**

   a. **Task Scope Analysis**:

   - Read task description and all subtasks
   - Identify what was supposed to be implemented
   - List all files that should have been created/modified
   - Note all requirements referenced in the task

   b. **Implementation Verification**:

   - **Check if files exist**: Use `listDirectory` to verify file presence
   - **Read implementation**: Use `readCode` for each file to see actual implementation
   - **Verify completeness**: Confirm all deliverables from task description are present
   - **Check for partial implementation**: Identify any TODO comments or incomplete functions

   c. **Code Pattern Analysis**:

   - **Naming conventions**: How are classes, functions, variables named?
   - **File organization**: How is code structured within files?
   - **Header structure**: What's the pattern for header guards, includes, interface definitions?
   - **Implementation structure**: What's the pattern for .cpp files (includes, constructors, methods)?
   - **Error handling**: How are errors handled? Return codes? Exceptions? Assertions?
   - **Documentation**: What's the comment style? Doxygen? Inline comments?

   d. **Interface Pattern Analysis**:

   - **Abstract interfaces**: Are pure virtual base classes used? Naming convention (I prefix)?
   - **Concrete implementations**: How do they inherit from interfaces?
   - **Dependency injection**: How are dependencies passed? Constructor? Setters?
   - **Method signatures**: What's the pattern for parameters? Const correctness?

   e. **Memory Management Analysis**:

   - **PROGMEM usage**: How are constants stored? F() macro usage?
   - **Static vs dynamic**: Any dynamic allocation? (Should be none)
   - **Buffer sizes**: What sizes are used for buffers? Fixed or configurable?
   - **Struct packing**: Any alignment considerations?

   f. **Timing Pattern Analysis**:

   - **Non-blocking patterns**: How is `millis()` used for timing?
   - **State tracking**: How are timing states tracked (lastTime variables)?
   - **Timeout handling**: How are timeouts implemented?
   - **Periodic tasks**: How are periodic updates scheduled?

   g. **Safety Pattern Analysis**:

   - **Input validation**: How are inputs validated?
   - **Bounds checking**: How are array accesses protected?
   - **Fault handling**: How are faults detected and reported?
   - **Emergency stop**: How is emergency stop implemented?

   h. **Testing Pattern Analysis** (if tests exist):

   - **Test framework**: What testing framework is used?
   - **Test organization**: How are tests structured?
   - **Mock objects**: How are hardware dependencies mocked?
   - **Test naming**: What's the naming convention for tests?
   - **Assertion style**: What assertion macros are used?

3. **Consistency Verification**:

   **Cross-Task Consistency Checks:**

   a. **Naming Consistency**:

   - Are class names consistent? (e.g., SensorHAL, OutputHAL, DisplayHAL)
   - Are method names consistent? (e.g., update(), init(), get/set patterns)
   - Are variable names consistent? (e.g., camelCase, snake_case)
   - Are constant names consistent? (e.g., UPPER_CASE, prefix conventions)
   - Are file names consistent? (e.g., snake_case.h/cpp)

   b. **Structural Consistency**:

   - Do all HAL modules follow same structure?
   - Do all control modules follow same structure?
   - Do all application modules follow same structure?
   - Is header/implementation split consistent?
   - Are include guards consistent?

   c. **Interface Consistency**:

   - Do all interfaces use pure virtual methods?
   - Do all interfaces have virtual destructors?
   - Do all concrete classes override all interface methods?
   - Is const correctness applied consistently?
   - Are parameter passing conventions consistent (const ref, value, pointer)?

   d. **Error Handling Consistency**:

   - Is error reporting consistent across modules?
   - Are return codes used consistently?
   - Are fault codes used consistently?
   - Is logging consistent?

   e. **Documentation Consistency**:

   - Are all public methods documented?
   - Is documentation style consistent?
   - Are parameters documented?
   - Are return values documented?
   - Are preconditions/postconditions documented?

   f. **Testing Consistency**:

   - Are all modules tested?
   - Is test coverage consistent?
   - Are test patterns consistent?
   - Are mock objects used consistently?

4. **Gap Analysis**:

   **Identify Missing or Incomplete Elements:**

   a. **Missing Implementations**:

   - List tasks marked complete but with missing files
   - List functions declared but not implemented
   - List interfaces defined but not used
   - List tests planned but not written

   b. **Incomplete Implementations**:

   - Identify TODO comments in code
   - Identify stub functions (empty or placeholder implementations)
   - Identify missing error handling
   - Identify missing input validation
   - Identify missing documentation

   c. **Inconsistencies**:

   - List naming inconsistencies across modules
   - List structural inconsistencies
   - List interface inconsistencies
   - List documentation inconsistencies

   d. **Technical Debt**:

   - Identify code that doesn't follow patterns
   - Identify code that violates constraints (dynamic allocation, blocking delays)
   - Identify code that doesn't meet requirements (timing, safety)
   - Identify code that needs refactoring

   e. **Integration Gaps**:

   - Identify modules that aren't integrated in main.cpp
   - Identify interfaces that aren't wired together
   - Identify missing initialization code
   - Identify missing update calls in main loop

   f. **Testing Gaps**:

   - Identify modules without tests
   - Identify critical paths without tests
   - Identify safety features without tests
   - Identify property tests not implemented

5. **Pattern Extraction for Current Task**:

   **Based on previous tasks, extract patterns to apply:**

   a. **File Creation Pattern**:

   ```
   Example from previous tasks:
   - Header file: include/hal/sensor_hal.h
   - Implementation: src/hal/sensor_hal.cpp
   - Test file: test/test_sensor_hal/test_temperature_filtering.cpp
   ```

   b. **Header File Pattern**:

   ```cpp
   Example structure:
   #ifndef MODULE_NAME_H
   #define MODULE_NAME_H

   #include <Arduino.h>
   // Other includes...

   // Interface definition (if applicable)
   class IModuleName {
   public:
       virtual ~IModuleName() {}
       virtual void method() = 0;
   };

   // Concrete implementation
   class ModuleName : public IModuleName {
   private:
       // Private members
   public:
       ModuleName(/* params */);
       void method() override;
   };

   #endif // MODULE_NAME_H
   ```

   c. **Implementation File Pattern**:

   ```cpp
   Example structure:
   #include "module_name.h"

   ModuleName::ModuleName(/* params */)
       : member1(value1), member2(value2) {
       // Constructor body
   }

   void ModuleName::method() {
       // Implementation
   }
   ```

   d. **Main.cpp Integration Pattern**:

   ```cpp
   Example:
   // Global instances (or in setup)
   SensorHAL sensorHAL(PIN_TEMP, PIN_DOOR);

   void setup() {
       // Initialize module
       sensorHAL.init();
   }

   void loop() {
       // Update module
       sensorHAL.update();
   }
   ```

   e. **Test File Pattern**:

   ```cpp
   Example:
   #include <unity.h>
   #include "module_name.h"

   void setUp() {
       // Setup before each test
   }

   void tearDown() {
       // Cleanup after each test
   }

   void test_specific_behavior() {
       // Arrange
       // Act
       // Assert
       TEST_ASSERT_EQUAL(expected, actual);
   }

   int main() {
       UNITY_BEGIN();
       RUN_TEST(test_specific_behavior);
       return UNITY_END();
   }
   ```

6. **Dependency Analysis**:

   **For current task N, identify dependencies on previous tasks:**

   a. **Direct Dependencies**:

   - Which previous tasks must be complete for task N to work?
   - Which interfaces does task N depend on?
   - Which data structures does task N depend on?
   - Which constants/enums does task N depend on?

   b. **Integration Dependencies**:

   - Which modules must be initialized before task N's module?
   - Which modules must be updated before task N's module in main loop?
   - Which modules does task N's module call?

   c. **Testing Dependencies**:

   - Which mock objects are needed for task N's tests?
   - Which test utilities are needed?
   - Which previous tests provide examples for task N's tests?

7. **Checkpoint Verification**:

   **If approaching or past a checkpoint task (5, 9, 12, 17, 21):**

   a. **Checkpoint 5 (HAL Validation)**:

   - Verify all HAL tests pass
   - Verify sensor readings on hardware (if available)
   - Verify relay outputs on hardware (if available)
   - Verify LCD display and keypad input (if available)

   b. **Checkpoint 9 (Control Layer Validation)**:

   - Verify all control layer tests pass
   - Verify PID control with simulated input
   - Verify reversing pattern timing
   - Verify safety interlocks trigger correctly

   c. **Checkpoint 12 (Storage Layer Validation)**:

   - Verify all storage layer tests pass
   - Verify EEPROM read/write operations
   - Verify checksum validation and corruption recovery
   - Verify fault logging and statistics tracking

   d. **Checkpoint 17 (Application Layer Validation)**:

   - Verify all application layer tests pass
   - Verify state machine transitions on hardware
   - Verify menu navigation on LCD/keypad
   - Verify program execution end-to-end

   e. **Checkpoint 21 (System Integration Validation)**:

   - Verify all integration tests pass
   - Verify complete cycle execution for all 6 programs
   - Verify all safety interlocks on hardware
   - Verify fault handling for all fault types
   - Verify menu system on LCD/keypad

**Analysis Outcome**:

Document findings in structured format:

```
PREVIOUS TASKS SUMMARY:
- Completed tasks: [list with checkmarks]
- Incomplete tasks: [list with issues]
- Last checkpoint: [number and status]

ESTABLISHED PATTERNS:
- File structure: [pattern description]
- Naming conventions: [pattern description]
- Interface design: [pattern description]
- Error handling: [pattern description]
- Testing approach: [pattern description]

CONSISTENCY STATUS:
- Naming: [consistent/issues found]
- Structure: [consistent/issues found]
- Interfaces: [consistent/issues found]
- Documentation: [consistent/issues found]

GAPS IDENTIFIED:
- Missing implementations: [list]
- Incomplete implementations: [list]
- Inconsistencies: [list]
- Technical debt: [list]
- Integration gaps: [list]
- Testing gaps: [list]

DEPENDENCIES FOR CURRENT TASK:
- Required previous tasks: [list]
- Required interfaces: [list]
- Required data structures: [list]
- Required constants: [list]

PATTERNS TO APPLY:
- File creation: [pattern]
- Code structure: [pattern]
- Integration: [pattern]
- Testing: [pattern]

READINESS ASSESSMENT:
- [Ready to proceed / Blockers identified]
- [List any issues that must be resolved first]
```

**Proceed to Step 4 only after completing this analysis.**

---

## Step 4: Task Execution Without Deviation

**Purpose**: Implement the task with absolute adherence to requirements, designs, specifications, and constraints.

### Mandatory Compliance Documents:

#### Requirements Compliance:

**For EACH requirement referenced in the task:**

1. **Requirement Extraction**:

   - Read requirement section from `.kiro/specs/imesa-dryer-controller/requirements.md`
   - Extract requirement number (e.g., "Requirement 2: Door Safety Interlock")
   - Read user story
   - Read ALL acceptance criteria (numbered list)

2. **Acceptance Criteria Checklist**:

   - Create checklist for EACH acceptance criterion
   - Mark each criterion as you implement it
   - Verify each criterion is testable
   - Document how each criterion is satisfied in code

3. **Requirement Traceability**:
   - Add comment in code referencing requirement number
   - Example: `// Requirement 2.5: Door open must deactivate heater within 100ms`
   - Link code sections to specific acceptance criteria
   - Document any design decisions related to requirement

**Example Requirement Compliance:**

```
Requirement 2: Door Safety Interlock
Acceptance Criteria:
☐ 2.1: Controller SHALL read Door_Sensor on pin 6
   → Implementation: pinMode(PIN_DOOR_SENSOR, INPUT_PULLUP) in constructor
☐ 2.2: Door_Sensor SHALL use internal pull-up
   → Implementation: INPUT_PULLUP mode set
☐ 2.3: LOW = door open, HIGH = door closed
   → Implementation: return digitalRead(doorPin) == HIGH
☐ 2.4: Door open SHALL deactivate heater within 100ms
   → Implementation: Checked every 100ms loop, emergencyStop() called
☐ 2.10: Door_Sensor SHALL be debounced with 50ms stable time
   → Implementation: Debounce logic with millis() tracking
```

#### Design Compliance:

**For EACH design element in the task:**

1. **Interface Compliance**:

   - Read interface definition from `.kiro/specs/imesa-dryer-controller/design.md`
   - Copy interface signature EXACTLY (method names, parameters, return types, const)
   - Implement ALL pure virtual methods
   - Add virtual destructor if interface
   - Verify const correctness

2. **Data Structure Compliance**:

   - Read struct/class definition from design document
   - Copy field names and types EXACTLY
   - Maintain field order (critical for EEPROM)
   - Include checksum field if specified
   - Use correct sizes (uint8_t, uint16_t, uint32_t, float)
   - Add padding if needed for alignment

3. **Algorithm Compliance**:

   - Read algorithm description from design document
   - Implement algorithm EXACTLY as specified
   - Use specified formulas (e.g., PID: output = Kp*error + Ki*integral + Kd\*derivative)
   - Apply specified constraints (e.g., anti-windup, clamping)
   - Use specified constants (e.g., thresholds, timeouts)

4. **State Machine Compliance**:

   - Implement states EXACTLY as defined in design
   - Implement transitions EXACTLY as defined
   - Implement guards (conditions) for transitions
   - Implement entry/exit actions for states
   - Implement timeout handling as specified

5. **Timing Compliance**:

   - Use non-blocking patterns (millis() timing)
   - Implement specified intervals (e.g., 100ms loop, 1s PID update)
   - Track last execution time for periodic tasks
   - Verify timing budget (task duration < period)

6. **Memory Compliance**:
   - Use PROGMEM for constants (F() macro for strings)
   - Use static allocation only (no new/malloc)
   - Use bit fields for boolean flags
   - Estimate memory usage (flash and SRAM)
   - Verify within budget (<28KB flash, <1.5KB SRAM)

#### Code Compliance:

**For ALL code written:**

1. **File Structure Compliance**:

   - Place files in correct directories (hal/, control/, app/, storage/, util/)
   - Use correct naming convention (snake_case.h/cpp)
   - Match header and implementation file names
   - Include header guards in all .h files
   - Format: `#ifndef MODULE_NAME_H` / `#define MODULE_NAME_H` / `#endif`

2. **Header File Compliance**:

   ```cpp
   // Required structure:
   #ifndef MODULE_NAME_H
   #define MODULE_NAME_H

   // System includes
   #include <Arduino.h>

   // Library includes
   #include <LibraryName.h>

   // Project includes
   #include "other_module.h"

   // Constants (PROGMEM if applicable)
   const uint8_t CONSTANT_NAME = value;

   // Enumerations
   enum class EnumName {
       VALUE1,
       VALUE2
   };

   // Interface (if applicable)
   class IModuleName {
   public:
       virtual ~IModuleName() {}
       virtual void method() = 0;
   };

   // Concrete class
   class ModuleName : public IModuleName {
   private:
       // Private members
       uint8_t privateMember;
       uint32_t lastUpdateTime;

   public:
       // Constructor
       ModuleName(uint8_t param);

       // Public methods
       void method() override;
       void update();
   };

   #endif // MODULE_NAME_H
   ```

3. **Implementation File Compliance**:

   ```cpp
   // Required structure:
   #include "module_name.h"

   // Constructor implementation
   ModuleName::ModuleName(uint8_t param)
       : privateMember(param), lastUpdateTime(0) {
       // Initialization code
       pinMode(privateMember, INPUT);
   }

   // Method implementations
   void ModuleName::method() {
       // Implementation
       // Add requirement traceability comments
       // Requirement X.Y: Description
   }

   void ModuleName::update() {
       // Non-blocking timing pattern
       uint32_t now = millis();
       if (now - lastUpdateTime >= INTERVAL) {
           lastUpdateTime = now;
           // Periodic task
       }
   }
   ```

4. **Coding Standards Compliance**:

   - **Naming**:
     - Classes: PascalCase (e.g., SensorHAL, PIDController)
     - Methods: camelCase (e.g., readTemperature, setHeater)
     - Variables: camelCase (e.g., currentTemp, lastTime)
     - Constants: UPPER_CASE (e.g., PIN_HEATER, TEMP_MAX)
     - Private members: camelCase with no prefix
     - Enums: PascalCase for type, UPPER_CASE for values
   - **Formatting**:
     - Indentation: 4 spaces (no tabs)
     - Braces: Opening brace on same line
     - Line length: <120 characters
     - Blank lines: Between functions, logical sections
   - **Comments**:
     - Document all public methods
     - Add requirement traceability comments
     - Explain non-obvious logic
     - Use `//` for single-line, `/* */` for multi-line

5. **Safety Compliance**:

   - **Input Validation**:
     - Validate all inputs (range checks, null checks)
     - Reject invalid inputs gracefully
     - Example: `if (temp < TEMP_MIN || temp > TEMP_MAX) return false;`
   - **Bounds Checking**:
     - Check array indices before access
     - Check buffer sizes before writes
     - Example: `if (index >= ARRAY_SIZE) return;`
   - **Error Handling**:
     - Check return values from functions
     - Handle errors gracefully (don't crash)
     - Log errors for debugging
     - Example: `if (!sensor.read()) { handleError(); return; }`
   - **Defensive Programming**:
     - Initialize all variables
     - Use const for read-only parameters
     - Avoid global mutable state
     - Use assertions for invariants (in debug builds)

6. **Timing Compliance**:

   - **Non-Blocking Patterns**:

     ```cpp
     // CORRECT: Non-blocking
     uint32_t lastTime = 0;
     void update() {
         uint32_t now = millis();
         if (now - lastTime >= INTERVAL) {
             lastTime = now;
             // Do work
         }
     }

     // WRONG: Blocking
     void update() {
         delay(INTERVAL);  // NEVER USE delay()!
         // Do work
     }
     ```

   - **Debouncing Pattern**:

     ```cpp
     // Debounce implementation
     bool lastState = false;
     uint32_t lastChangeTime = 0;
     bool stableState = false;

     void update() {
         bool currentState = digitalRead(pin);
         if (currentState != lastState) {
             lastChangeTime = millis();
             lastState = currentState;
         }
         if (millis() - lastChangeTime >= DEBOUNCE_TIME) {
             stableState = lastState;
         }
     }
     ```

   - **Anti-Chatter Pattern**:

     ```cpp
     // Anti-chatter for relay
     uint32_t lastChange = 0;
     bool currentState = false;

     void setState(bool newState) {
         if (newState == currentState) return;
         if (millis() - lastChange < MIN_INTERVAL) return;

         digitalWrite(pin, newState ? HIGH : LOW);
         currentState = newState;
         lastChange = millis();
     }
     ```

7. **Memory Compliance**:

   - **PROGMEM Usage**:

     ```cpp
     // Store strings in flash
     const char STR_READY[] PROGMEM = "READY";

     // Use F() macro for literals
     display.print(F("Temperature: "));

     // Store arrays in flash
     const Program DEFAULT_PROGRAMS[6] PROGMEM = {
         {"Towels", 75, 45, 30, 30, 2, 10, 0},
         // ...
     };

     // Read from PROGMEM
     Program prog;
     memcpy_P(&prog, &DEFAULT_PROGRAMS[index], sizeof(Program));
     ```

   - **Static Allocation**:

     ```cpp
     // CORRECT: Static allocation
     char buffer[20];
     uint8_t data[10];

     // WRONG: Dynamic allocation
     char* buffer = new char[20];  // NEVER USE new!
     char* buffer = malloc(20);    // NEVER USE malloc!
     String str = "text";          // NEVER USE String class!
     ```

   - **Bit Fields**:
     ```cpp
     // Save SRAM with bit fields
     struct Flags {
         uint8_t doorClosed : 1;
         uint8_t tempValid : 1;
         uint8_t heaterActive : 1;
         uint8_t motorFwdActive : 1;
         uint8_t motorRevActive : 1;
         uint8_t faultLatched : 1;
         uint8_t unused : 2;
     };  // Only 1 byte instead of 6
     ```

8. **Testing Compliance**:
   - Write tests for all new code
   - Follow established test patterns
   - Use mock objects for hardware dependencies
   - Test normal cases, edge cases, error cases
   - Verify all acceptance criteria are tested

### Implementation Rules:

1. **Read Before Writing**:

   - Read ALL related code before modifying
   - Understand existing patterns
   - Identify integration points
   - Check for naming conflicts

2. **Implement Incrementally**:

   - Start with interface/header
   - Implement basic functionality
   - Add error handling
   - Add documentation
   - Write tests
   - Integrate with main.cpp

3. **Verify Continuously**:

   - Check compilation after each file
   - Run tests after each implementation
   - Verify memory usage periodically
   - Check for warnings (should be zero)

4. **Document Thoroughly**:

   - Add requirement traceability comments
   - Document public APIs
   - Explain non-obvious logic
   - Add TODO comments for future work (if any)

5. **Follow Patterns**:
   - Use patterns from previous tasks
   - Maintain consistency
   - Don't introduce new patterns without justification

### Implementation Verification:

**Before marking task complete, verify ALL of the following:**

1. **Compilation Verification**:

   - [ ] Code compiles without errors
   - [ ] Code compiles without warnings
   - [ ] All includes are resolved
   - [ ] All dependencies are satisfied

2. **Requirement Verification**:

   - [ ] All acceptance criteria are implemented
   - [ ] All acceptance criteria are testable
   - [ ] Requirement traceability comments are added
   - [ ] No requirements are missed

3. **Design Verification**:

   - [ ] Interface matches design exactly
   - [ ] Data structures match design exactly
   - [ ] Algorithms match design exactly
   - [ ] State machine matches design exactly
   - [ ] Timing matches design exactly

4. **Code Quality Verification**:

   - [ ] Naming conventions followed
   - [ ] Formatting consistent
   - [ ] Comments adequate
   - [ ] No code duplication
   - [ ] No dead code

5. **Safety Verification**:

   - [ ] All inputs validated
   - [ ] All bounds checked
   - [ ] All errors handled
   - [ ] No unsafe operations

6. **Memory Verification**:

   - [ ] No dynamic allocation
   - [ ] PROGMEM used for constants
   - [ ] Memory usage estimated
   - [ ] Within budget (<28KB flash, <1.5KB SRAM)

7. **Timing Verification**:

   - [ ] No blocking delays
   - [ ] Non-blocking patterns used
   - [ ] Timing budget met
   - [ ] Watchdog reset called (if in main loop)

8. **Integration Verification**:

   - [ ] Module instantiated in main.cpp (if needed)
   - [ ] Module initialized in setup() (if needed)
   - [ ] Module updated in loop() (if needed)
   - [ ] Dependencies wired correctly

9. **Testing Verification**:

   - [ ] Unit tests written
   - [ ] Unit tests pass
   - [ ] Property tests written (if applicable)
   - [ ] Property tests pass (if applicable)
   - [ ] Integration tests updated (if applicable)

10. **Documentation Verification**:
    - [ ] Public APIs documented
    - [ ] Requirement traceability added
    - [ ] Non-obvious logic explained
    - [ ] README updated (if needed)

**Verification Checklist Output:**

```
IMPLEMENTATION COMPLETE:
- Files created: [list]
- Files modified: [list]
- Lines of code: [count]
- Memory usage: Flash [X]KB, SRAM [Y]KB

REQUIREMENTS SATISFIED:
- [List each requirement with checkmark]

DESIGN COMPLIANCE:
- Interface: [✓ compliant]
- Data structures: [✓ compliant]
- Algorithms: [✓ compliant]
- Timing: [✓ compliant]

CODE QUALITY:
- Compilation: [✓ no errors, no warnings]
- Naming: [✓ consistent]
- Formatting: [✓ consistent]
- Comments: [✓ adequate]

SAFETY:
- Input validation: [✓ complete]
- Bounds checking: [✓ complete]
- Error handling: [✓ complete]

TESTING:
- Unit tests: [✓ written and passing]
- Property tests: [✓ written and passing / N/A]
- Integration tests: [✓ updated / N/A]

INTEGRATION:
- main.cpp: [✓ updated / N/A]
- Dependencies: [✓ wired correctly]

READY FOR REVIEW: [YES / NO]
```

**Proceed to Step 5 only after completing implementation and verification.**

---

## Step 5: User Review and Feedback Integration

**Purpose**: Request user review of the implementation and apply any required updates or changes.

**Actions**:

1. **Present Implementation**:

   - Summarize what was implemented
   - List all files created/modified
   - Highlight key features and functionality
   - Note any deviations or decisions made (if any)

2. **Request Review**:

   - Ask user to review the implementation
   - Request feedback on functionality, design, code quality
   - Inquire about any changes or improvements needed

3. **Handle Feedback**:

   - **IF user requests changes**:
     - Document requested changes clearly
     - Implement changes following same protocol (Steps 2-4)
     - Re-request review after changes
     - Repeat until user is satisfied
   - **IF user approves without changes**:
     - Confirm explicit approval to proceed
     - Move to Step 6 for Git operations

4. **Verification Before Proceeding**:
   - Ensure user has explicitly stated approval
   - Confirm no additional changes are needed
   - Get clear go-ahead for Git operations

**Do NOT proceed to Step 6 without explicit user approval.**

---

## Step 6: Post-Git Requirement (After Task Completion)

**Purpose**: Add, commit, push implementation, checkout, merge, and synchronize between local and remote repositories. Delete related branches after detailed verification.

**Actions**:

1. **Verify Current State**:

   - Execute `git status` to check:
     - Current branch (should be feature branch)
     - All modified/created files
     - No unintended changes
   - Execute `git branch -vv` to display branch tracking information
   - Execute `git fetch origin` to update remote tracking information

2. **Stage and Commit Changes**:

   - Review all changes carefully: `git diff`
   - Stage all changes: `git add .`
   - Verify staged changes: `git status`
   - Commit with descriptive message: `git commit -m "feat: [Task N] Descriptive task title and summary"`
   - Use conventional commit format: `feat:`, `fix:`, `refactor:`, `docs:`, etc.

3. **Push Feature Branch**:

   - Push to remote: `git push origin <feature-branch>`
   - Verify push success
   - Confirm remote branch exists: `git branch -r`

4. **Checkout Base Branch**:

   - Checkout main/master: `git checkout main` (or appropriate base branch)
   - Verify clean state: `git status`
   - Pull latest changes: `git pull origin main`

5. **Merge Feature Branch**:

   - **CRITICAL: Think twice before merging**
   - Verify you are on correct base branch: `git branch`
   - Merge feature branch: `git merge <feature-branch>`
   - **IF merge conflicts occur**:
     - **HALT immediately**
     - Prompt user to resolve conflicts manually
     - Wait for user confirmation
     - Verify resolution: `git status`
   - **IF merge successful**:
     - Verify merged changes: `git log --oneline -5`
     - Confirm all expected files are present

6. **Push Merged Changes**:

   - Push to remote: `git push origin main`
   - Verify push success
   - Confirm remote is updated: `git log origin/main --oneline -5`

7. **Delete Feature Branch (Local and Remote)**:

   - **CRITICAL: Verify merge success before deleting**
   - Confirm feature branch is fully merged: `git branch --merged`
   - Delete local branch: `git branch -d <feature-branch>`
   - Delete remote branch: `git push origin --delete <feature-branch>`
   - Verify deletion: `git branch -a` (feature branch should not appear)

8. **Final Synchronization Verification**:

   - Execute `git status` - should show clean working directory
   - Execute `git branch -vv` - should show main branch in sync with origin
   - Execute `git log --oneline -5` - should show recent commit
   - Confirm local and remote are synchronized

9. **Cleanup Verification**:
   - Verify no orphaned branches: `git branch -a`
   - Verify no uncommitted changes: `git status`
   - Verify correct branch: `git branch` (should be on main)

**Task Completion Confirmed**: All changes committed, pushed, merged, and branches cleaned up.

---

## Protocol Enforcement

**This protocol is MANDATORY for EVERY task listed in `.kiro/specs/imesa-dryer-controller/tasks.md`.**

- **No shortcuts allowed**
- **No exceptions permitted**
- **All six steps must be completed in order**
- **Each step must be verified before proceeding to next**
- **User approval required before Step 6**

**Failure to follow this protocol will result in:**

- Inconsistent implementations
- Git conflicts and branch issues

**Success in following this protocol ensures:**

- Clean Git history
- Consistent code quality
- Exact alignment with specifications
- Maintainable codebase
- Predictable behavior
- Secure implementation

---

**REMEMBER**: This protocol is your roadmap to success. Follow it religiously.
