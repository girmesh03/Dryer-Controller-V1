# Wiring Diagram + Pin Assignment (ESP32-WROOM-32 / esp32doit-devkit-v1)

This project targets **ESP32 (30‑pin DOIT DevKit v1)**. The definitive pin mapping is in `include/config_pins.h`.

## Safety First

- Never connect AC mains directly to the ESP32. Use **isolated relays/contactors** and proper enclosures.
- Ensure all modules share a **common GND** reference (ESP32 ↔ relay board ↔ sensors).
- ESP32 is **3.3V logic**. Avoid I2C pull-ups to 5V.

## Pin Assignment Table (Firmware)

| Function | Symbol | ESP32 GPIO | Notes |
|---|---:|---:|---|
| Door switch input | `PIN_DOOR_SENSOR` | `33` | `INPUT_PULLUP`, `LOW=closed`, `HIGH=open` |
| DS18B20 data | `PIN_TEMP_SENSOR` | `32` | Requires **4.7kΩ pull-up to 3.3V** |
| Heater relay | `PIN_HEATER_RELAY` | `14` | **Active‑LOW** (LOW=ON, HIGH=OFF) |
| Motor relay (FWD) | `PIN_MOTOR_FWD_RELAY` | `27` | **Active‑LOW** |
| Motor relay (REV) | `PIN_MOTOR_REV_RELAY` | `26` | **Active‑LOW** |
| AUX relay (reserved) | `PIN_AUX_RELAY` | `25` | **Active‑LOW**, reserved channel |
| Buzzer | `PIN_BUZZER` | `23` | Driven with `tone()` |
| I2C SDA | `PIN_I2C_SDA` | `21` | LCD backpack SDA |
| I2C SCL | `PIN_I2C_SCL` | `22` | LCD backpack SCL |

## Keypad (4x4) Wiring

The Keypad is scanned via the Keypad library using:

- Rows: `PIN_KEYPAD_ROW = {16, 17, 18, 19}`
- Cols: `PIN_KEYPAD_COL = {13, 4, 5, 15}`

⚠️ `GPIO4`, `GPIO5`, and `GPIO15` are **ESP32 strapping pins**. Operational rule:
- Do **not** hold any keypad key during reset/power-up.

## Relay Board Wiring (4‑channel, Active‑LOW)

Firmware assumes a **low-level trigger** relay module:
- `HIGH` = relay input idle → **NO contact stays open**
- `LOW` = relay energized → **NO contact closes**

Recommended:
- Power relay coil side from an appropriate supply (often 5V) per relay board design.
- Ensure the relay board inputs are compatible with **3.3V logic**. If not, use a transistor/driver stage.

## Door Switch Wiring

- Use a simple switch to **GND**.
- Firmware enables internal pull-up on GPIO33:
  - Door **closed** → pin reads `LOW`
  - Door **open** → pin reads `HIGH`

## DS18B20 Wiring (3.3V)

- `DQ` (data) → GPIO32
- `VDD` → 3.3V
- `GND` → GND
- Pull-up resistor: **4.7kΩ from DQ to 3.3V**

## LCD (I2C, PCF8574 backpack)

- SDA → GPIO21
- SCL → GPIO22
- Address: `0x27` (`include/config_build.h`)

Important:
- Many LCD backpacks include pull-ups to **5V** when powered at 5V. Ensure pull-ups are **not** pulling SDA/SCL above 3.3V.
- If the LCD backpack must be powered from 5V, use a proper **I2C level shifter**.

