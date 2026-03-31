# Final System Verification Report (Template)

Project: Industrial Dryer Controller  
Target: ESP32-WROOM-32 (`esp32doit-devkit-v1`)  

## Build Information

- Date:
- Tester:
- Hardware: (relay board type, sensor type, LCD backpack, etc.)
- Firmware version (boot screen `FW: vX.Y.Z`):
- Build date/time (service `MEMORY INFO` page 1):
- PlatformIO “Project Inspect”:
  - Flash used:
  - RAM used:

Service menu `MEMORY INFO` page 0:
- FLASH: ____ / ____ K (____%)
- HEAP: ____ / ____ K (____%)
- EEP (reserved map): ____ / ____ B (____%)

## 12.1 Safety Verification

### Door interlock (all operational states)

- [ ] RUNNING_HEAT: start cycle → open door → **all outputs OFF within 100ms**
- [ ] FAULT screen shows `DOOR OPEN` → close door → press `B` to clear
- [ ] RUNNING_COOLDOWN: open door → motor stops immediately
- [ ] PAUSED: open door → verify no outputs energize when resume attempted
- [ ] ANTI_CREASE: open door → drum stops immediately and exits to IDLE

### Motor interlock / dead-time

- [ ] I/O TEST: attempt FWD and REV simultaneously → interlock prevents both ON
- [ ] I/O TEST: switch FWD → REV quickly → **2s dead-time** enforced (motor OFF for 2s before reversing)

### Heater safety gating

- [ ] Door open during heating → heater OFF immediately
- [ ] Sensor disconnect during heating → heater OFF after ~2s (sensor fault)
- [ ] Over-temp (>95°C) → heater OFF immediately
- [ ] Attempt to start cycle with door open → cycle does not start, heater remains OFF

### Minimum heater on/off times

- [ ] HEATER TEST at 50% duty → heater ON ≥10s before OFF
- [ ] Heater OFF ≥10s before ON again

### STOP behavior (operator emergency stop)

- [ ] During heating press `B/STOP` → heater and motor OFF within 100ms
- [ ] Fault is **not** latched by STOP (restart possible without clearing faults)

## 12.2 Temperature Control Verification

- [ ] Manual Mode at 60°C: stabilizes within ±3°C after 10–15 min
- [ ] No oscillations >±2°C over a 5-min observation window
- [ ] No rapid relay chatter (min on/off times respected)
- [ ] Setpoint ramp: edit 60→80°C, observe ~2°C/s ramp behavior
- [ ] Anti-windup: 85°C saturates; reduce to 75°C, recovery is prompt
- [ ] Bumpless transfer: pause/resume does not cause output jump
- [ ] Over-temp protection triggers at >95°C
- [ ] Thermal runaway triggers when >15°C rise in 30s while heater ON

## 12.3 EEPROM Persistence Verification

- [ ] Program persistence: edit `TOWELS` temp to 70°C → reboot → still 70°C
- [ ] PID gains persistence: AutoTune accept → reboot → PID VIEW matches
- [ ] FOPDT persistence: FOPDT accept → reboot → parameters still present
- [ ] Fault history persistence: trigger 3 faults → reboot → history still present
- [ ] Corruption recovery:
  - [ ] Corrupt CRC8 (offset 0x003) → reboot → defaults restored
  - [ ] Boot screen shows `EEPROM RESET`

Suggested corruption method (one-time test):
1) Temporarily add the following to `setup()` in `src/main.cpp`, upload once, then remove it:
   - `EEPROM.write(0x003, EEPROM.read(0x003) ^ 0xFF);`
   - `EEPROM.commit();`
2) Reboot to confirm the controller detects invalid CRC and performs factory reset.
- [ ] Factory reset tool restores defaults and clears fault history

## 12.4 Cycle Execution Verification

### Auto Mode programs (all 6)

- [ ] Towels
- [ ] Bed Sheets
- [ ] Delicates
- [ ] Heavy Cotton
- [ ] Mixed Load
- [ ] Synthetic

For each program, verify:
- [ ] Setpoint correct
- [ ] Duration countdown correct
- [ ] Drum pattern correct (FWD/STOP/REV/STOP)

### Manual Mode combinations

- [ ] 50°C / 20 min
- [ ] 80°C / 60 min
- [ ] 65°C / 35 min

### In-cycle edits

- [ ] TEMP edit applies and ramps smoothly
- [ ] TIME edit updates remaining time immediately

### Pause / resume

- [ ] Pause after ~5 min heating, resume OK
- [ ] Pause near end of heating, resume OK
- [ ] Pause during cooldown, resume OK

### Cooldown + Anti-crease

- [ ] Cooldown starts when heating duration ends (heater OFF, drum continues)
- [ ] Cooldown exits when PV <45°C or after 15 min timeout
- [ ] Anti-crease tumbles 10s every 5 min (verify at least 3 cycles)
- [ ] Anti-crease exits on door open or `B/STOP`

### Abort

- [ ] Abort during heating returns to IDLE with outputs OFF
- [ ] Abort during cooldown returns to IDLE with outputs OFF
- [ ] Abort during anti-crease returns to IDLE with outputs OFF

## 12.5 Fault Handling Verification

- [ ] Door open fault within 100ms + correct message
- [ ] Sensor fault after ~2s + correct message
- [ ] Over-temp fault immediate + correct message
- [ ] Thermal runaway fault + correct message
- [ ] Heating timeout fault (long test)
- [ ] Fault latching requires operator acknowledgement (`B`)
- [ ] Attempt to clear without conditions OK shows `CONDITIONS NOT OK`
- [ ] Fault history shows codes + timestamp + temperature
- [ ] Ring buffer overwrite: trigger >10 faults, oldest overwritten
- [ ] Watchdog reset detection + logged in history
- [ ] Brownout reset detection + logged in history

## 12.6 Memory Verification (ESP32)

Record:
- PlatformIO “Project Inspect” Flash/RAM usage
- Service menu `MEMORY INFO` page 0 values (Flash/Heap/EEP)

Notes:
- EEPROM reserved CRC map is **512 bytes** (design layout); ESP32 EEPROM emulation is initialized as 1024 bytes.

## Final Sign-Off

- Overall result: PASS / FAIL
- Known issues / limitations:
- Next actions:

Signed:
