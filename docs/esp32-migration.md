# ESP32 Migration Notes (ESP32-WROOM-32 / esp32doit-devkit-v1)

The project was originally developed under **Arduino Nano (ATmega328P)** memory constraints, which required splitting features across multiple build environments and compile-time flags.

The hardware target is now **ESP32-WROOM-32** (PlatformIO `board = esp32doit-devkit-v1`) so the firmware can include **all features** without flag-based feature splitting.

## PlatformIO

- Environment: `esp32` (see `platformio.ini`)
- Framework: Arduino
- Monitor: 115200
- Note: `check_memory.py` and Nano flash-limit enforcement are no longer used.

## Pin Mapping (ESP32)

### Keypad (4x4)

- Rows (inputs, `INPUT_PULLUP`): `16, 17, 18, 19`
- Cols (outputs): `13, 4, 5, 15`

⚠️ `GPIO4`, `GPIO5`, and `GPIO15` are **ESP32 strapping pins**. For reliable boots:
- Do **not** hold any keypad key during reset / power-up.
- If you ever see boot issues, the fix is to move keypad columns off strapping pins (requires using other GPIOs or adding small external resistors / an I/O expander).

### Relays (active-LOW)

- `PIN_HEATER_RELAY = 14`
- `PIN_MOTOR_FWD_RELAY = 27`
- `PIN_MOTOR_REV_RELAY = 26`
- `PIN_AUX_RELAY = 25` (reserved relay channel)

Relays are treated as **active-LOW** in firmware:
- `HIGH` at boot keeps the relay **OFF** (NO contact open)
- `LOW` energizes the relay (NO contact closes)

### Door Sensor

- `PIN_DOOR_SENSOR = 33`
- Mode: `INPUT_PULLUP`
- Logic: `LOW = door closed`, `HIGH = door open`

### Buzzer

- `PIN_BUZZER = 23`

### DS18B20

- `PIN_TEMP_SENSOR = 32`

⚠️ DS18B20 requires an external pull-up resistor on the data line (typically **4.7k** to **3.3V**).

### LCD (I2C)

- `SDA = 21`
- `SCL = 22`
- LCD address remains `0x27` (as configured in `include/config_build.h`)

#### LCD reliability note (ESP32 + PCF8574 backpacks)

Some LCD backpacks + the `LiquidCrystal_I2C` driver can exhibit a “missing first character” symptom on specific screens (e.g. `FAULT HISTORY` showing as `AULT HISTORY`).

Mitigations used in this project:
- Prefer **partial updates** and avoid repeated `lcd.clear()` in service tools (prevents flicker and reduces edge-case writes).
- Carefully **pad lines** so writes do not wrap past 20 columns.

If you still see missing characters (screen-specific):
- Verify the LCD backpack is powered correctly (3.3V-safe I2C pull-ups; avoid 5V pull-ups into ESP32).
- Verify SDA/SCL wiring is short and solid, with a common ground.
- As a temporary workaround, printing a leading space before a header can “shift” the visible text if the first character is intermittently dropped by the LCD/backpack.

## EEPROM on ESP32

ESP32 uses **flash-emulated EEPROM**. `EEPROMStore::init()` calls `EEPROM.begin(1024)` (1KB total) and writes are finalized with `EEPROM.commit()` when changes are flushed.

Notes:
- The current CRC-protected reserved map uses the first **512 bytes** (Appendix D / `design.md`).
- The remaining bytes are intentionally left unused for future phases.
- If EEPROM CRC validation fails at boot and an auto factory-reset occurs, the boot screen shows `EEPROM RESET`.

## Watchdog / Reset Cause

- Reset cause is obtained via `esp_reset_reason()` and mapped to the existing UI behavior:
  - Boot screen can indicate `LAST RESET: WDT` or `LAST RESET: BROWNOUT`
  - WDT/Brownout events are logged into EEPROM fault history as acknowledge-only faults
- A Task Watchdog is initialized (2s nominal) and kicked from `loop()`.

## Why This Pin Set (Stability + Safety)

- Keeps **relays off strapping pins** to prevent unintended energization or boot issues.
- Uses **strapping pins only for keypad columns** (`GPIO4/5/15`) to avoid external pull-ups on the keypad, with the operational rule:
  - Do **not** hold keypad keys during reset / power-up.
- Keeps the **door interlock** on a pin that supports `INPUT_PULLUP` (`GPIO33`) to match the safety logic:
  - `LOW = closed`, `HIGH = open`
- Keeps all **relay outputs active-LOW** and driven **HIGH** during initialization to keep NO contacts open at boot.

## Wiring Recommendations

- DS18B20 data pull-up: add **4.7k** from `PIN_TEMP_SENSOR` to **3.3V**.
- Relay inputs: if your relay board does not already include input pull-ups, add **10k pull-up** to **3.3V** on each relay input to keep relays **OFF** at boot (active-LOW).

For the full wiring/pin assignment table, see `docs/wiring-esp32.md`.
