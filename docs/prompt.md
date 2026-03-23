# Industrial Tumble Dryer Controller - Complete Requirements, Design and Implementation Tasks Specification

# Role

You are a senior embedded systems architect, industrial appliance controls analyst, and reverse-engineering documentation specialist with deep expertise in:

- industrial tumble dryer control systems
- Arduino/embedded firmware architectures
- safety-critical machine behavior
- menu-driven HMI design
- relay and motor control
- temperature control systems
- program specification writing
- implementation task decomposition

Your task is to produce a **complete requirements, design, and implementation tasks specification** for an Arduino Nano based industrial tumble dryer controller that reverse-engineers the behavior of the **IMESA ES Series Industrial Tumble Dryer** as closely as possible using the information provided below and any explicitly stated constraints.

You must write a **professional-grade specification document** that an embedded development team could use to build the controller.

---

# Objective

Generate a complete, structured specification that defines:

1. System requirements
2. Functional requirements
3. Non-functional requirements
4. Safety requirements
5. Hardware I/O requirements
6. User interface and menu requirements
7. Program/cycle behavior requirements
8. Control logic requirements
9. Fault and recovery requirements
10. Reverse-engineering targets for IMESA ES Series behavior
11. Software architecture requirements
12. Implementation tasks and development phases
13. Verification and test requirements
14. Deliverable definitions

The specification must be detailed enough that a team can implement the system directly from it.

---

# CRITICAL: Memory Optimization Requirements

**Arduino Nano has severely limited resources:**

- Flash Memory: 32 KB (30 KB usable after bootloader)
- SRAM: 2 KB
- EEPROM: 1 KB

**MANDATORY OPTIMIZATION RULES:**

Every single character, variable, function, string, and implementation MUST be aggressively optimized for memory efficiency:

1. **Code Size Optimization:**

   - Use PROGMEM for all constant strings, lookup tables, and read-only data
   - Minimize library dependencies; use only essential libraries
   - Prefer function pointers and lookup tables over switch/case statements where appropriate
   - Use bit fields and packed structures to minimize SRAM usage
   - Avoid C++ features that bloat code (exceptions, RTTI, virtual functions unless absolutely necessary)
   - Use compiler optimization flags (-Os for size)

2. **SRAM Optimization:**

   - Minimize global variables; use local variables where possible
   - Reuse buffers and temporary storage
   - Use uint8_t and uint16_t instead of int where appropriate
   - Avoid deep function call stacks
   - Use F() macro for all Serial.print strings
   - Minimize string concatenation; use fixed buffers
   - Avoid dynamic memory allocation (malloc/new)

3. **String and Display Optimization:**

   - Store all LCD strings in PROGMEM
   - Use abbreviated menu text where possible
   - Reuse display buffers
   - Minimize string length while maintaining clarity

4. **Data Structure Optimization:**

   - Use enums with explicit uint8_t backing
   - Pack boolean flags into bitfields
   - Use unions for overlapping data
   - Minimize structure padding

5. **EEPROM Optimization:**

   - Store only essential persistent data
   - Use compact data formats
   - Implement efficient CRC (CRC8 instead of CRC32 if acceptable)

6. **Library Selection:**

   - Choose lightweight library alternatives
   - Consider implementing simple functions directly instead of including large libraries
   - Verify library memory footprint before inclusion

7. **Verification:**
   - Monitor Flash and SRAM usage after each implementation task
   - Report memory usage statistics
   - Refactor if usage exceeds 85% of available memory
   - Document memory optimization techniques used

**FAILURE TO OPTIMIZE WILL RESULT IN NON-FUNCTIONAL FIRMWARE.**

---

# Primary Instruction

You must produce a **complete requirements, design, and implementation tasks specification** for reverse engineering the **IMESA ES Series Industrial Tumble Dryer**.

You must:

- use the information provided in this document as the source input
- preserve its intent
- expand it into a full specification document
- organize it professionally
- ensure every implementation is memory-optimized for Arduino Nano constraints

Do **not** write a simplified summary.
Do **not** write only a high-level idea.
Do **not** skip implementation tasks.
Do **not** leave requirements vague.
Do **not** produce implementation code directly; produce the specification only.

---

# Non-Negotiable Rules

- Do **not** add unsupported assumptions as if they are facts
- Do **not** invent manufacturer-specific details unless they are clearly marked as assumptions or derived engineering recommendations
- Do **not** omit any provided constraint or behavioral requirement
- Do **not** weaken safety rules
- Do **not** turn this into a casual explanation
- Do **not** produce only code or only a design sketch
- Do **not** skip reverse-engineering considerations
- Do **not** ignore industrial-grade reliability expectations
- Do **not** include water temperature features in the spec
- Do **not** include unrelated appliance features
- Do **not** leave ambiguity where a specification decision is required; instead, define the requirement clearly or mark it as an assumption/unknown
- Do **not** include automated unit tests in the specification
- **MUST** include self-test requirements for startup validation
- **MUST** instruct that hardware testing will be performed by user uploading and running code on actual hardware

---

# Scope

The specification must cover an Arduino Nano based controller for an industrial tumble dryer that includes, at minimum:

- door safety interlock
- heater relay control
- forward and reverse drum motor control
- temperature sensing using DS18B20
- 20x4 I2C LCD
- 4x4 keypad
- menu-driven user interface
- predefined drying programs
- editable temperature and duration settings
- closed-loop temperature control using PID
- PID auto-tune
- robust sensor noise handling
- industrial fault behavior
- state-machine-based program execution
- safe startup and shutdown behavior
- program/service/diagnostic menus

The specification must reflect an industrial appliance controller rather than a hobby project.

---

# Core Design Goal

Create an industrial dryer controller that behaves like a real production dryer controller:
safe, stable, fault-aware, configurable, and easy to operate.
Do not create a toy demo. The system must behave as an industrial appliance controller.

---

# Non-negotiable Success Criteria

- **UX parity:** Menus, cycle parameters, reversing patterns, cooling/anti-crease end-of-cycle behavior, and in-cycle parameter editing behave like ES Series (within keypad/LCD constraints)
- **Safety parity:** Door interlock and fault behavior immediately de-energize all outputs; no heater chatter due to sensor noise or unstable PID; motor forward/reverse interlock with enforced dead-time; over-temperature and sensor-fault cut-outs
- **Reliability:** Deterministic state machine; watchdog; brownout resilience; EEPROM with CRC; event/fault logging with timestamps; EMI-aware input filtering; no blocking I/O in control loop
- **Memory efficiency:** Every implementation must be optimized for Arduino Nano's limited Flash/SRAM/EEPROM
- **Hardware testability:** User will upload and test on actual hardware; no automated unit tests required

---

# Key IMESA ES Series Behaviors to Mirror (Context for Reverse-Engineering)

- Standard reversing drum across ES models (reversal pattern configurable by speed/rotation/pause timing)
- Multi-step heating, programmable cool-down, and anti-crease that keeps alternating drum motion with pauses post-cycle to avoid wrinkles
- IM10 color touchscreen on higher-end ES (free program editing via "Wizard"), IM.7EASY 4-digit keypad control on entry models; unlimited/large program sets; in-cycle parameter modifications. We must replicate these with our keypad+LCD HMI
- Residual-humidity program modes and controlled airflow for efficiency (we'll provide an optional humidity-estimation path and permit a future humidity sensor)
- 180° large door with interlocks; "filter clean" indication at end of cycle (replicate as a service reminder and door/fan interlock logic)
- Some reseller literature and distributors use "IMSEA ES Series" wording; treat this as IMESA branding

---

# Control Loop Timing

10 Hz control tick baseline (100 ms):

- T0: read keypad (debounced), read door input
- T+5 ms: DS18B20 state machine step (nonblocking conversion/read)
- T+10 ms: filter update, plausibility checks
- T+20 ms: PID compute (if in HOLD/RAMP)
- T+30 ms: heater gating FSM (respect min on/off/hysteresis)
- T+40 ms: drive FWD/REV FSM with dead-time
- T+60 ms: refresh HMI (paged; 5-10 Hz visible)
- T+80 ms: log async, EEPROM flush if dirty (rate-limited)

---

# Hardware Inputs and Outputs

## Inputs

1. **Door sensor**

   - Digital input with pull-up
   - Door open = LOW
   - Door closed = HIGH
   - Digital pin: D6

2. **Temperature sensor**

   - DS18B20 one-wire sensor
   - Reads dryer drum temperature
   - Digital pin: D12

3. **Keypad**
   - 4x4 keypad
   - Library: Keypad.h
   - Keypad rows: A0..A3
   - Keypad cols: D2..D5
   - Key mapping:
     ```
     {'1', '2', '3', 'A'},  // Row 0
     {'4', '5', '6', 'B'},  // Row 1
     {'7', '8', '9', 'C'},  // Row 2
     {'*', '0', '#', 'D'}   // Row 3
     ```
   - Required key meanings:
     - 4 = Up
     - 6 = Down
     - 8 = Left
     - 2 = Right
     - 5 = OK / Enter
     - A = Start
     - B = Stop
   - Other keys may be used for navigation, editing, confirmation, service, diagnostics, and mode selection

## Outputs

1. **Heater relay**

   - Digital output
   - HIGH = heater contactor ON
   - LOW = heater contactor OFF
   - Digital pin: D9

2. **Motor forward relay**

   - Digital output
   - HIGH = motor runs forward
   - LOW = motor stop
   - Digital pin: D10

3. **Motor reverse relay**
   - Digital output
   - HIGH = motor runs reverse
   - LOW = motor stop
   - Digital pin: D11

## Display

- LCD 20x4
- I2C connection via SDA/SCL: A4 (SDA), A5 (SCL)
- Use: Wire.h and LiquidCrystal_I2C.h

---

# Platform and Code Requirements

- Target platform: **Arduino Nano**
- Language: **Arduino C++ subset**
- Build system: **PlatformIO**
- Deliver code that is modular, readable, maintainable, and **memory-optimized**
- Use appropriate additional libraries beyond the ones stated when needed, but prioritize lightweight alternatives
- The implementation must be complete and compile-ready
- Every implementation must be integrated with main.cpp

---

# Safety Rules

These rules are absolute.

1. **If the door is open at any time, everything must turn OFF immediately**

   - heater OFF
   - motor forward OFF
   - motor reverse OFF
   - program paused or faulted as appropriate
   - require safe recovery before restart

2. **Never allow heater ON when the system is in an unsafe state**

   - door open
   - temperature sensor failure
   - invalid temperature
   - PID instability
   - output chatter risk
   - startup fault
   - stop condition
   - emergency fault

3. **Prevent relay chatter**

   - add minimum ON/OFF time logic
   - add hysteresis / deadband
   - debounce sensor and keypad inputs
   - add fail-safe logic
   - PID output must be rate-limited or filtered so the heater relay does not rapidly toggle

4. **Thermal runaway protection**

   - define over-temperature fault threshold
   - define sensor-disconnect fault
   - define abnormal rate-of-rise detection
   - define maximum heating time protection if temperature does not respond correctly

5. **Motor safety**

   - forward and reverse must never energize at the same time
   - enforce break-before-make direction switching
   - add direction change delay
   - motor must be off before reverse-to-forward or forward-to-reverse transitions

6. **Industrial-grade fault handling**
   - fault states must be explicit
   - faults must be displayed clearly
   - faults must require operator acknowledgment or safe reset
   - machine must fail safe

---

# Functional Requirements

- Boot screen with model name, firmware version, self-test status
- Main menu:
  - Pre-defined programs by Item Type (e.g., Towels, Bed Sheets, Mixed, Delicates, Heavy Cotton, Synthetic)
  - Each program has default Temperature Setpoint, Drum Pattern (forward/reverse on/off times), Airflow/Heater duty constraints, and Duration
  - Per-cycle temporary edits allowed for Temperature and Duration (and optionally drum pattern timings)
  - Manual Mode: set temperature then duration then run
  - PID menu: view/change P/I/D, enable AutoTune, view FOPDT model, restore defaults
  - Service menu: input tests (toggle relays with interlocks enforced), sensor readouts, door test, EEPROM reset, diagnostics, firmware info
  - Settings: units °C/°F, contrast/brightness (if supported), sound on/off, program table editor, language placeholders
- Run-time states (finite state machine):
  - Idle → ProgramSelect → Confirm → Preheat → RampToSetpoint → Regulate → CoolDown → Complete
  - Anti-crease (periodic brief tumbles post-complete)
  - Pause (operator stop) and Fault (latched)
  - Door Open immediately forces Fault from any state; all outputs safe-off
- Drum control:
  - Reversing pattern typical of industrial dryers: e.g., Forward 30-60 s, Stop 3-10 s, Reverse 30-60 s, Stop 3-10 s, repeat. Make per-program configurable
  - Never energize forward and reverse simultaneously; enforce a neutral stop interval before direction change
- Temperature control:
  - DS18B20 sampling ~2-4 Hz; use median-of-N then exponential moving average; clamp outliers; detect sensor faults (no device, CRC fail, implausible rate-of-change)
  - PID setpoint with ramping to avoid overshoot; bumpless transfer when switching modes or entering AutoTune
  - Output to heater relay via a time-proportioning window (e.g., 5-15 s window), but with strict relay safety gating:
    - Minimum on time and minimum off time (e.g., ≥10 s each, configurable)
    - Hysteresis/deadband around setpoint for additional chatter immunity
    - Output rate limiter (slew) so duty changes are gradual
    - Freeze heater on sensor fault, door open, or AutoTune transitions until safe conditions are validated
  - Cool-down: after duration or stop, run drum without heat until temperature drops below a safe threshold (e.g., 40-50 °C) or a time limit
- PID AutoTune:
  - Provide a guided AutoTune routine (Relay/Åström-Hägglund method or library default):
    - Pre-conditions: empty drum, door closed, ambient near X °C
    - Operator prompt and safety checks before starting
    - Limit heater cycling frequency; respect min on/off times
    - Compute and store tuned P/I/D; allow operator to accept or reject
  - Also offer model-based initial tuning using FOPDT
- FOPDT Modeling:
  - Provide a step-test utility: apply a safe, bounded heater step with drum pattern fixed; measure process gain K, dead time L, time constant τ
  - Derive initial PI/PID gains using IMC or Cohen-Coon style formulas; show computed values and allow the operator to apply
- Programs (example defaults; editable and stored in EEPROM):
  - Towels: 75 °C, 40 min, Fwd 45 s / Stop 5 s / Rev 45 s / Stop 5 s
  - Bed Sheets: 60 °C, 45 min, Fwd 60 s / Stop 6 s / Rev 60 s / Stop 6 s
  - Delicates: 50 °C, 30 min, longer stop intervals to reduce wrinkling
  - Heavy Cotton: 80 °C, 50 min
  - Mixed: 65 °C, 35 min
  - Add more as needed. Allow per-program airflow/heater cap if you implement it later

---

# Menu Structure Requirements

## 1) Main Menus

- Auto
- Manual

## 2) Auto Menus

The user interface must include program menus for predefined drying programs such as:

- towel
- bed sheet
- and other suitable industrial dryer items

Each program must include at minimum:

- item name
- predefined temperature setpoint
- predefined duration
- appropriate cooldown phase
- ability for temporary user modification of Pre-defined temperature and duration before start
- view current temperature (Sp and Pv)
- view remaining time
- view fault status
- start
- stop
- pause/resume
- motor behavior if needed

## 3) Manual Menus

The menu must allow the operator to:

- Set temperature
- Set duration
- Run the dryer

The UI must also allow:

- view current temperature (Sp and Pv)
- view remaining time
- start
- stop
- pause/resume
- view fault status
- appropriate cooldown phase
- ability for user modification of temperature and duration before start
- motor behavior if needed

## 4) PID Temperature Control

Implement temperature control using PID.

Requirements:

- use a stable closed-loop control approach
- read DS18B20 temperature reliably
- filter sensor noise
- maintain temperature near setpoint
- avoid chatter
- support tuning for industrial thermal inertia
- account for the dryer as a FOPDT (First Order Plus Dead Time) process model
- use the FOPDT understanding to guide PID tuning, filtering, and auto-tune logic

## 5) PID Auto-Tune

Implement PID auto-tuning:

- allow tuning on demand from a service/admin menu
- generate appropriate PID constants for the dryer thermal system
- store tuned values in EEPROM
- allow fallback to default safe tuning if no valid parameters exist
- protect against unsafe autotune execution
- autotune must not compromise safety

## 6) Sensor Noise Handling

Implement robust sensor noise handling:

- moving average or median filtering
- outlier rejection
- plausibility checks
- sensor timeout detection
- invalid reading handling
- smoothing without excessive lag
- fault if sensor data is unstable or missing

## 7) Motor Operation

The motor must support:

- forward rotation
- reverse rotation
- safe direction changes
- configurable direction timing if needed
- alternating run logic if the industrial dryer requires periodic reversing
- motor OFF during fault, stop, or door-open event

## 8) Display Behavior

The LCD 20x4 must show:

- current mode
- selected program
- current temperature
- target temperature
- remaining time
- heater state
- motor state
- door state
- fault messages
- menu prompts

The display should be clear, compact, and operator-friendly.

---

# Keypad / UI Behavior

Use the keypad intelligently for navigation and editing.

Minimum suggested behavior:

- 4 / 6 = up/down
- 8 / 2 = left/right
- 5 = OK / select / confirm
- A = start
- B = stop / cancel
- other keys may be assigned to secondary functions such as:
  - quick program jump
  - edit mode
  - save
  - service menu
  - autotune request
  - manual test mode
  - parameter reset

The UI must be intuitive and safe.

---

# Required Arduino/PlatformIO Libraries

Use the stated libraries and any additional necessary libraries for a robust implementation, such as:

- Wire.h
- LiquidCrystal_I2C.h
- Keypad.h
- OneWire.h
- DallasTemperature.h
- PID_v1.h
- EEPROM.h

You may add other small, appropriate helper libraries only if truly needed, but prefer a clean and minimal dependency set. Always prioritize lightweight alternatives to minimize Flash/SRAM usage.

---

# Architecture Requirements

Implement the software as a well-structured state machine.

## Suggested states

- BOOT
- IDLE
- MENU
- PROGRAM_SELECT
- PARAM_EDIT
- READY
- START_DELAY
- RUNNING_HEAT
- RUNNING_COOLDOWN
- MOTOR_RUN_FORWARD
- MOTOR_RUN_REVERSE
- PAUSED
- STOPPING
- FAULT
- AUTOTUNE
- SERVICE

## Control logic requirements

- All state transitions must be explicit
- All outputs must be controlled by the state machine only
- No output should be left floating or implicitly active
- The state machine must prioritize safety over operation

---

# Data and Parameter Management

Implement persistent storage for:

- PID constants
- user-selected default program
- calibrated thresholds
- optionally program presets

Use EEPROM safely:

- validate stored data
- include versioning / checksum / magic number
- fall back to defaults if EEPROM data is invalid
- minimize EEPROM usage due to 1 KB limit

---

# Industrial Behavior Expectations

The controller should behave like a real industrial dryer controller, including:

- safe start sequence
- safe stop sequence
- door-open interruption handling
- fault recovery
- controlled heater activation
- stable process response
- operator feedback
- no unsafe shortcuts

---

# Fault Conditions to Detect

At minimum detect and handle:

- door open
- DS18B20 disconnected
- invalid temperature reading
- overtemperature
- temperature sensor stuck / unreasonable change
- PID output instability
- relay chatter risk
- startup self-test failure
- keypad failure if detectable
- memory data corruption
- unsafe state transition
- motor direction conflict

---

# Self-Test Requirements

At startup, perform self-test checks such as:

- sensor presence
- LCD initialization
- keypad responsiveness if possible
- EEPROM data validity
- output defaults safe state
- door status
- temperature plausibility

If any critical check fails, enter safe fault state.

**Note:** Self-test is the ONLY automated testing required. All other testing will be performed by the user uploading and running the code on actual hardware.

---

# Software Architecture

- Non-blocking cooperative scheduler using millis(), task periods for:
  - Fast loop (10-50 ms): keypad scan, state machine tick, outputs arbitration
  - Medium loop (200-500 ms): DS18B20 poll, filtering, UI refresh
  - Slow loop (1-5 s): EEPROM deferred writes, statistics, heartbeat
- Modules (files):
  - config_pins.h, config_build.h
  - io_abstraction.{h,cpp} (digital in/out, debouncing)
  - ds18b20_sensor.{h,cpp} (read, filter, plausibility)
  - pid_control.{h,cpp} (PID, auto-tune wrapper, gating logic)
  - fopdt_model.{h,cpp} (identification, parameter store)
  - drum_control.{h,cpp} (reversing state machine, interlocks)
  - heater_control.{h,cpp} (time-proportioning window + safety gating)
  - ui_lcd.{h,cpp} (views, navigation)
  - keypad_input.{h,cpp} (mapping, short/long press handling)
  - eeprom_store.{h,cpp} (programs, settings, CRC)
  - faults.{h,cpp} (codes, latching logic, history)
  - app.{h,cpp} (global state machine)
  - main.cpp (setup, loop)
- Coding standards: clear enums for states and faults, constexpr for timings, no magic numbers, bounds-checking on all user inputs
- **All modules must be integrated with main.cpp**

---

# Data Persistence

- EEPROM layout with versioning and CRC:
  - Header (version, CRC)
  - Settings (units, UI config)
  - Program table (N entries, each with name, setpoint, duration, drum timings)
  - PID gains, FOPDT parameters
  - Fault history ring buffer (minimal due to 1 KB EEPROM limit)
- Provide migration if version changes
- Use CRC8 instead of CRC32 to save space

---

# Heater Relay Gating (Explicit Requirements)

- Time-proportioning with window W (e.g., 10 s)
- Enforce min_on_time and min_off_time (e.g., ≥10 s configurable)
- Add setpoint deadband (e.g., ±0.5-1.5 °C configurable)
- MV slew limit per window to prevent abrupt changes
- Suppress heater on transitions when PV quality is degraded (e.g., high noise, sensor unstable, during AutoTune, right after door close until PV stabilizes)
- Log suppressed toggles for diagnostics

---

# FOPDT-Based Initial Tuning (Implement Utility)

- Conduct safe step test: from 0% to a small bounded heater duty; ensure min on/off and monitor PV
- Estimate K (gain), τ (time constant), L (dead time)
- Compute initial PI/PID gains using an IMC-style method targeting conservative closed-loop behavior; display and let operator accept

---

# AutoTune

- Relay autotune cycle respecting relay protection rules
- Abort on door open, sensor fault, overtemp, or operator cancel
- Show progress and estimated Ku/Tu if applicable; compute P/I/D; let operator review and accept

---

# Testing and Validation

**CRITICAL: No automated unit tests are required.**

Testing approach:

- Provide a bench test mode (no relays energized) with simulated DS18B20 PV for logic validation
- User will upload firmware to Arduino Nano hardware
- User will perform on-target tests: IO loopback jig to verify outputs are never co-energized improperly
- User will conduct acceptance tests:
  - Door open from any state → heater and motor off within <=100 ms tick; fault latched
  - Minimum relay on/off times always respected
  - Drum direction reversal always includes stop dwell
  - PID tracks setpoint within ±2-3 °C steady-state for recommended loads after tuning
  - Cool-down proceeds until threshold or timeout
  - EEPROM contents persist across reset; CRC protects corruption
  - Watchdog resets enter safe state and report cause

**After each implementation task, instruct the user to upload and test the code on hardware.**

---

# Assumptions and Research Tasks (Reverse-Engineering IMESA ES Series)

- Research IMESA ES Series dryer manuals/datasheets/user guides to mirror:
  - Menu names/terminology, default programs and temperatures, reversing patterns, cool-down/anti-crease behaviors, alarm tones, and fault message style
  - Any special cycles (e.g., wet/auto moisture sensing if documented; if absent, time/temperature-based)
- Document each replicated behavior and any deviations due to unavailable proprietary details
- Do not copy proprietary firmware; implement original code that achieves functionally equivalent behavior

---

# Compliance and Safety Disclaimers

- High voltage and moving machinery are hazardous. This firmware assumes proper electrical design with certified contactors, interlocks, thermal cutouts, and emergency stop per local standards (e.g., IEC/UL). Always include hardware safeties independent of software. This project is for qualified personnel.

---

# Acceptance Criteria (Summary)

- Compiles and runs on Arduino Nano under PlatformIO with documented libraries
- Meets all functional, safety, and UI requirements above
- Demonstrates stable temperature control with no heater relay chatter; respects min on/off
- Door open behavior is immediate and latched; requires deliberate recovery
- Provides FOPDT identification and AutoTune with operator guidance and safe constraints
- EEPROM persistence with CRC and versioning
- Clear, operator-friendly LCD UI reflecting IMESA-class UX
- Flash usage ≤85% of available 30 KB
- SRAM usage ≤85% of available 2 KB
- EEPROM usage ≤85% of available 1 KB

---

# Coding Requirements

The final code must:

- compile on Arduino Nano via PlatformIO
- use clear constants and enums
- avoid blocking delays where possible
- use millis() for timing
- be modular and readable
- avoid magic numbers
- include comments for safety-critical logic
- handle all states deterministically
- keep heater OFF on uncertainty
- keep motor OFF on uncertainty
- prefer fail-safe defaults
- **be aggressively optimized for memory (Flash/SRAM/EEPROM)**
- **use PROGMEM for all constant data**
- **use F() macro for all Serial.print strings**
- **minimize global variables**
- **use uint8_t and uint16_t instead of int where appropriate**

---

# Important Constraints

- Do not assume unsafe behavior
- Do not keep heater ON when temperature data is uncertain
- Do not let noisy sensor readings cause relay chatter
- Do not energize forward and reverse at the same time
- Do not let the dryer run with the door open
- Do not omit fault handling
- Do not produce placeholder code where real logic is required
- Do not exceed Arduino Nano memory limits
- Do not include automated unit tests
- **Every implementation must be integrated with main.cpp**

---

# MANDATORY: Task Execution Protocol

**This protocol defines six mandatory steps that MUST be followed when executing EACH implementation task. No shortcuts. No exceptions.**

## Step 1: Pre-Git Requirement (Before Task Execution)

**Purpose:** Ensure complete and accurate Git branch information to prevent issues during new branch creation and checkout.

**Actions:**

1. **Check Current State:**

   - Execute `git status` to check:
     - Current branch name
     - Uncommitted changes (staged/unstaged files)
     - Untracked files
   - Execute `git branch -vv` to display:
     - All local branches
     - Branch tracking information
     - Ahead/behind status relative to remote

2. **Update Remote Information:**

   - Execute `git fetch origin` to update remote tracking information
   - Verify remote branches are synchronized

3. **Handle Uncommitted Changes:**

   - **IF uncommitted changes exist:**
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

4. **Synchronize Local with Remote:**

   - **IF local branch is behind remote:**
     - Execute `git pull origin <branch>` to synchronize
   - **IF merge conflicts detected:**
     - **HALT immediately**
     - Prompt user to resolve conflicts manually
     - Wait for user confirmation before proceeding

5. **Create Feature Branch:**

   - Execute `git checkout -b <descriptive-branch-name>`
   - Use clear, descriptive branch names matching task number and description

6. **Verify Clean State:**
   - Execute `git status` to confirm clean working directory
   - Confirm correct branch is checked out
   - Proceed to Step 2 only after verification

---

## Step 2: Comprehensive and Extremely Deep Codebase Analysis

**Purpose:** Capture every single detail of the codebase to ensure absolute alignment with requirements, designs, specifications, and constraints.

**Critical Analysis Areas:**

### Codebase Analysis (Complete Deep Dive):

- Read and understand ALL existing code files
- Identify current architecture patterns
- Document existing state machine implementation
- Analyze memory usage (Flash/SRAM/EEPROM)
- Identify optimization opportunities
- Document all dependencies and libraries
- Understand integration points with main.cpp

### Specification Analysis:

**Requirements:**

- Review ALL functional requirements
- Review ALL safety requirements
- Review ALL hardware I/O requirements
- Review ALL UI/menu requirements
- Review ALL control logic requirements
- Review ALL fault handling requirements

**Design:**

- Review software architecture requirements
- Review state machine design
- Review module decomposition
- Review data structures and memory layout
- Review EEPROM layout and CRC implementation

**Constraints:**

- Arduino Nano memory limits (Flash: 30 KB, SRAM: 2 KB, EEPROM: 1 KB)
- Memory optimization requirements
- Safety rules (absolute)
- Industrial behavior expectations
- Integration with main.cpp requirement

**Tasks:**

- Identify current task number and description
- Understand task objectives and deliverables
- Identify dependencies on previous tasks
- Understand acceptance criteria

**Analysis Outcome:**

- Complete understanding of what exists
- Complete understanding of what needs to be implemented
- Clear plan for memory-optimized implementation
- Confidence in maintaining consistency with existing code

**Proceed to Step 3 only after completing this comprehensive analysis.**

---

## Step 3: Comprehensive and Extremely Deep Analysis of Previously Implemented Tasks (N - 1)

**Purpose:** Understand all previously implemented tasks to ensure consistency, avoid duplication, and maintain architectural patterns.

**Actions:**

1. **Identify Previous Tasks:**

   - Review specification document
   - Identify all tasks marked as completed (checked boxes)
   - For current task N, analyze tasks 1 through N-1

2. **Analyze Each Previous Task:**

   - What was implemented?
   - Which files were created/modified?
   - What patterns were established?
   - What memory optimization techniques were used?
   - How was integration with main.cpp achieved?
   - What safety mechanisms were implemented?

3. **Consistency Verification:**

   - Coding style consistency
   - Naming convention consistency
   - State machine pattern consistency
   - Error handling pattern consistency
   - Memory optimization pattern consistency

4. **Gap Analysis:**
   - What functionality is still missing?
   - What dependencies exist for current task?
   - What interfaces need to be maintained?

**Analysis Outcome:**

- Complete understanding of implementation history
- Clear picture of established patterns
- Confidence in maintaining consistency
- Awareness of potential conflicts or duplications

**Proceed to Step 4 only after completing this analysis.**

---

## Step 4: Task Execution Without Deviation

**Purpose:** Implement the task with absolute adherence to requirements, designs, specifications, and constraints.

### Mandatory Compliance Documents:

#### Requirements Compliance:

- Implement EXACTLY what the task requires
- Do not add features not specified
- Do not omit required features
- Follow ALL safety rules
- Follow ALL memory optimization rules

#### Design Compliance:

- Follow established architecture patterns
- Maintain state machine design
- Follow module decomposition
- Use established data structures
- Follow EEPROM layout design

#### Code Compliance:

- Use clear constants and enums
- Avoid magic numbers
- Include comments for safety-critical logic
- Use PROGMEM for constant data
- Use F() macro for Serial.print strings
- Minimize global variables
- Use uint8_t and uint16_t appropriately
- Integrate with main.cpp

### Implementation Rules:

- Write memory-optimized code
- Test compilation after implementation
- Verify Flash/SRAM/EEPROM usage
- Document memory usage
- Ensure safety rules are enforced
- Ensure fail-safe defaults

### Implementation Verification:

- Code compiles without errors
- Code compiles without warnings
- Memory usage is within limits (≤85%)
- All safety rules are implemented
- Integration with main.cpp is complete
- Task acceptance criteria are met

**CRITICAL: Each implementation task MUST result in meaningful, visible changes. The user must be able to see that the task has been completed and the system has progressed.**

**Proceed to Step 5 only after completing implementation and verification.**

---

## Step 5: User Review and Feedback Integration

**Purpose:** Request user review of the implementation and apply any required updates or changes.

**Actions:**

1. **Present Implementation:**

   - Summarize what was implemented
   - List all files created/modified
   - Highlight key features and functionality
   - Report memory usage (Flash/SRAM/EEPROM)
   - Note any deviations or decisions made (if any)

2. **Request Hardware Testing:**

   - **Instruct user to upload the code to Arduino Nano hardware**
   - **Instruct user to test the implemented functionality**
   - **Request feedback on hardware behavior**
   - Ask if the implementation works as expected
   - Inquire about any issues or unexpected behavior

3. **Handle Feedback:**

   - **IF user requests changes:**
     - Document requested changes clearly
     - Implement changes following same protocol (Steps 2-4)
     - Re-request review after changes
     - Repeat until user is satisfied
   - **IF user approves without changes:**
     - Confirm explicit approval to proceed
     - Move to Step 6 for Git operations

4. **Verification Before Proceeding:**
   - Ensure user has explicitly stated approval
   - Confirm no additional changes are needed
   - Get clear go-ahead for Git operations

**Do NOT proceed to Step 6 without explicit user approval.**

---

## Step 6: Post-Git Requirement (After Task Completion)

**Purpose:** Add, commit, push implementation, checkout, merge, and synchronize between local and remote repositories. Delete related branches after detailed verification.

**Actions:**

1. **Verify Current State:**

   - Execute `git status` to check:
     - Current branch (should be feature branch)
     - All modified/created files
     - No unintended changes
   - Execute `git branch -vv` to display branch tracking information
   - Execute `git fetch origin` to update remote tracking information

2. **Stage and Commit Changes:**

   - Review all changes carefully: `git diff`
   - Stage all changes: `git add .`
   - Verify staged changes: `git status`
   - Commit with descriptive message: `git commit -m "feat: [Task N] Descriptive task title and summary"`
   - Use conventional commit format: `feat:`, `fix:`, `refactor:`, `docs:`, etc.

3. **Push Feature Branch:**

   - Push to remote: `git push origin <feature-branch>`
   - Verify push success
   - Confirm remote branch exists: `git branch -r`

4. **Checkout Base Branch:**

   - Checkout main/master: `git checkout main` (or appropriate base branch)
   - Verify clean state: `git status`
   - Pull latest changes: `git pull origin main`

5. **Merge Feature Branch:**

   - **CRITICAL: Think twice before merging**
   - Verify you are on correct base branch: `git branch`
   - Merge feature branch: `git merge <feature-branch>`
   - **IF merge conflicts occur:**
     - **HALT immediately**
     - Prompt user to resolve conflicts manually
     - Wait for user confirmation
     - Verify resolution: `git status`
   - **IF merge successful:**
     - Verify merged changes: `git log --oneline -5`
     - Confirm all expected files are present

6. **Push Merged Changes:**

   - Push to remote: `git push origin main`
   - Verify push success
   - Confirm remote is updated: `git log origin/main --oneline -5`

7. **Delete Feature Branch (Local and Remote):**

   - **CRITICAL: Verify merge success before deleting**
   - Confirm feature branch is fully merged: `git branch --merged`
   - Delete local branch: `git branch -d <feature-branch>`
   - Delete remote branch: `git push origin --delete <feature-branch>`
   - Verify deletion: `git branch -a` (feature branch should not appear)

8. **Final Synchronization Verification:**

   - Execute `git status` - should show clean working directory
   - Execute `git branch -vv` - should show main branch in sync with origin
   - Execute `git log --oneline -5` - should show recent commit
   - Confirm local and remote are synchronized

9. **Cleanup Verification:**
   - Verify no orphaned branches: `git branch -a`
   - Verify no uncommitted changes: `git status`
   - Verify correct branch: `git branch` (should be on main)

**Task Completion Confirmed:** All changes committed, pushed, merged, and branches cleaned up.

---

## Protocol Enforcement

**This protocol is MANDATORY for EVERY implementation task.**

- **No shortcuts allowed**
- **No exceptions permitted**
- **All six steps must be completed in order**
- **Each step must be verified before proceeding to next**
- **User approval required before Step 6**

**Failure to follow this protocol will result in:**

- Inconsistent implementations
- Git conflicts and branch issues
- Memory limit violations
- Safety rule violations

**Success in following this protocol ensures:**

- Clean Git history
- Consistent code quality
- Exact alignment with specifications
- Maintainable codebase
- Predictable behavior
- Secure implementation
- Memory-optimized code

---

# Required Deliverable Structure

Your response must be formatted in markdown and must contain the following sections in order:

## 1. Title

A clear document title naming the project and purpose.

## 2. Purpose and Scope

State what the specification covers and what it does not cover.

## 3. System Overview

Describe the overall industrial dryer controller concept.

## 4. Reverse-Engineering Target Summary

Describe the IMESA ES Series behavior that the specification is trying to replicate.

## 5. Assumptions and Unknowns

List all unknown or not-yet-verified details and explicitly mark them.

## 6. Hardware Requirements

Specify:

- controller platform
- sensors
- actuators
- display
- keypad
- wiring expectations
- electrical interface constraints

## 7. Input/Output Requirements

Define every input and output signal clearly, including:

- signal type
- logic level
- active state
- default safe state
- safety dependency

## 8. Functional Requirements

List all required functions, including:

- program selection
- menu navigation
- start/stop behavior
- cycle control
- drum rotation control
- heater logic
- cooldown
- anti-crease behavior if applicable
- pause/resume
- skip behavior if supported
- service mode
- diagnostics
- fault recovery

## 9. Non-Functional Requirements

Include:

- reliability
- maintainability
- industrial behavior
- safety precedence
- deterministic operation
- human readability
- extensibility
- memory efficiency

## 10. Safety Requirements

Define all mandatory safety rules, including:

- door-open response
- stop-state response
- output shutdown behavior
- fault latching
- motor interlock rules
- heater lockout rules
- safe recovery rules
- fail-safe defaults

## 11. State Machine Requirements

Define the required controller states and transitions.
Include states such as:

- boot
- idle
- menu
- program selection
- parameter edit
- preheat
- run
- reverse
- cooldown
- pause
- stop
- fault
- service
- completion

## 12. Cycle / Program Requirements

Specify how drying programs shall be represented.
Define requirements for:

- preset programs
- editable parameters
- duration handling
- temperature handling
- drum reversing timing
- cooldown
- end-of-cycle behavior

## 13. User Interface Requirements

Define the LCD and keypad behavior, including:

- screen layout
- navigation model
- button mapping
- edit behavior
- confirmation behavior
- alarm display
- fault display
- status feedback

## 14. Control Logic Requirements

Define the rules governing:

- heater activation
- motor forward/reverse control
- delay timing
- relay interlocks
- temperature regulation
- sensor validation
- transitions between operating states

## 15. Fault Handling Requirements

Specify:

- fault detection conditions
- fault priorities
- fault messages
- latching rules
- operator recovery rules
- safe shutdown sequence

## 16. Startup and Shutdown Requirements

Define:

- power-on self-test
- safe initialization
- output defaults
- reset behavior
- normal shutdown
- emergency shutdown
- recovery after power loss

## 17. Program/Parameter Storage Requirements

If persistent settings are to be used, define:

- stored values
- validation
- default values
- corruption handling

## 18. Software Architecture Requirements

Define the required software decomposition into modules or components, such as:

- I/O abstraction
- keypad handling
- LCD handling
- state machine
- fault manager
- program manager
- motor control
- heater control
- temperature processing
- persistence

## 19. Implementation Task Breakdown

Provide a complete implementation task specification, organized into:

- phase
- objective
- inputs
- outputs
- dependencies
- acceptance criteria
- validation steps
- memory optimization requirements
- integration with main.cpp requirements

**CRITICAL: Each task must be meaningful and result in visible progress. Tasks must not be trivial or placeholder implementations.**

## 20. Verification and Test Requirements

Define how the implementation should be tested:

- Self-test at startup (automated)
- Hardware testing by user (manual)
- Safety tests
- Fault tests
- Operator workflow tests

**Note: No automated unit tests required. User will upload and test on hardware.**

## 21. Documentation Requirements

Specify what documents should be produced by the development effort, such as:

- wiring map
- pin assignment table
- fault table
- menu tree
- state chart
- testing checklist

## 22. Final Deliverables

State the exact output package expected from the implementation team.

---

# Detailed Requirements the Specification Must Include

## A. Hardware and Signal Definition

The specification must define each of the following:

### Inputs

- Door sensor
- Temperature sensor (DS18B20)
- Keypad
- Any additional inputs required for diagnostics or safety logic if needed

### Outputs

- Heater relay
- Motor forward relay
- Motor reverse relay
- LCD I2C
- Any indicator or status outputs if included in the design

For every signal, define:

- purpose
- electrical type
- active state
- default safe state
- debounce/filtering requirement if applicable
- failure interpretation
- safety consequences

---

## B. UI/Menu Specification

The specification must define a menu system that includes at least:

- Auto mode
- Manual mode
- Program selection
- Parameter editing
- Service/diagnostic mode
- Fault display
- Status display
- Start/stop confirmation flow

It must also define:

- how the user moves through menus
- what each key does
- what happens if invalid keys are pressed
- how edits are saved or canceled
- how the interface returns to safe state after inactivity or fault

---

## C. Program Requirements (Auto Mode Different From Manual Mode)

The specification must require support for industrial dryer programs such as:

- towel
- bed sheet
- mixed load
- heavy cotton
- delicate load
- other suitable industrial dryer items

For each program, define:

- item name
- target temperature
- target duration
- drum reversing pattern
- cooldown behavior
- parameter edit limits
- default start behavior

If a program table is to be included, define its required fields and validation rules.

---

## D. Temperature Control Requirements

The specification must require:

- closed-loop temperature control using PID
- DS18B20 sensor reading
- noise filtering
- plausibility checks
- fault detection for invalid temperature data
- temperature stability requirements
- anti-chatter logic
- safe heater shutdown on uncertainty

The specification must also define:

- how the controller responds to sensor failure
- how it behaves during cooldown
- how it handles overshoot
- how it prevents unstable heater switching

---

## E. Motor Control Requirements

The specification must require:

- forward drum rotation
- reverse drum rotation
- safe direction changes
- enforced stop interval between direction changes
- motor off on fault or door open
- realistic industrial reversing patterns

The specification must also define:

- whether motor timing is fixed or configurable
- how reversal timing is stored
- what happens during stop or pause

---

## F. Safety and Fault Rules

The specification must explicitly require:

- immediate shutdown when the door opens
- immediate shutdown when stop is pressed
- latch of unsafe fault states
- no heater with door open
- no motor with door open
- no simultaneous forward and reverse relay activation
- safe startup defaults
- safe reset conditions

Faults to include:

- door open
- sensor failure
- invalid temperature
- overtemperature
- relay conflict
- startup failure
- keypad failure if detectable
- memory corruption if persistent storage is used
- unsafe state transition
- output chatter risk
- any additional industrial fault required by the design

---

## G. Reverse-Engineering and Research Tasks

The specification must require the implementation team to research and compare:

- IMESA ES Series manuals
- user guides
- service documentation
- program behavior
- menu terminology
- cycle flow
- fault messages
- reversing logic
- cooldown behavior
- anti-crease/end-of-cycle behavior

The specification must state that:

- proprietary firmware must not be copied
- the implementation must be original
- any undocumented behavior must be marked as inferred
- any missing detail must be identified as a research task

---

## H. Design Requirements

The specification must define a software design that uses:

- explicit state machine logic
- modular code structure
- deterministic execution
- non-blocking timing where practical
- clear separation of concerns
- safe fallback behavior
- memory-optimized implementation

It must also define the recommended software modules and what each module is responsible for.

---

## I. Implementation Task Requirements

The implementation tasks section must break the project into small actionable tasks such as:

- requirement analysis
- hardware mapping
- state machine design
- UI design
- sensor layer implementation
- relay control implementation
- temperature filtering
- fault handling
- program table logic
- EEPROM/persistence if needed
- bench validation
- final system verification

Each task must include:

- purpose
- dependencies
- expected output
- completion criteria
- test method (hardware testing by user)
- memory optimization requirements
- integration with main.cpp requirements

**CRITICAL: Each task must be meaningful and produce visible, testable results.**

---

# Output Style Requirements

Your output must be:

- professional
- structured
- exhaustive
- technically precise
- easy for an engineering team to follow

Do not write in a casual tone.

Do not use vague placeholders such as "etc." when a requirement can be specified.

Do not produce implementation code unless specifically requested; produce the specification only.

---

# Quality Bar

The final specification must be strong enough that a separate AI agent or engineering team could use it to build the complete controller without needing additional clarification for the basic design.

It must read like a real industrial project specification document, not a short prompt.

---

# Implementation Steps (High-Level Overview)

1. Project scaffolding, pin config, safe IO defaults, watchdog setup
2. LCD and keypad integration with base menu framework
3. DS18B20 driver, filtering, fault detection
4. Drum control state machine with safe reversing and door interlock
5. Heater control with time-proportioning and relay gating
6. PID integration with bumpless transfer and anti-windup
7. FOPDT identification tool and initial tuning computation
8. AutoTune integration and UI flow
9. Program table with EEPROM persistence and editor
10. Fault system, history, and diagnostics
11. Polishing: UI, help screens, service tools
12. Final system verification and documentation

**Each step must follow the mandatory Task Execution Protocol (6 steps).**

---

# Final Instruction

Act as if you are delivering a professional industrial control firmware specification package for an Arduino Nano dryer controller.

Prioritize:

- safety
- reliability
- maintainability
- memory efficiency
- industrial-grade behavior

The system is a realistic industrial dryer controller, not a simple hobby project.

Now produce the complete requirements, design, and implementation tasks specification following the structure defined above.

**Remember:**

- Every implementation must be memory-optimized
- Every implementation must be integrated with main.cpp
- Every implementation task must follow the 6-step Task Execution Protocol
- User will upload and test on hardware (no automated unit tests)
- Each task must produce meaningful, visible progress
- Flash usage must stay ≤85% of 30 KB
- SRAM usage must stay ≤85% of 2 KB
- EEPROM usage must stay ≤85% of 1 KB

---

# END OF PROMPT

This is the complete consolidated prompt for generating the requirements, design, and implementation tasks specification for the Arduino Nano based IMESA ES Series Industrial Tumble Dryer Controller.
