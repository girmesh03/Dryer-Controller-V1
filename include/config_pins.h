#ifndef CONFIG_PINS_H
#define CONFIG_PINS_H

#include <Arduino.h>

// Input pins
// Door switch:
// - Must use internal pull-up (INPUT_PULLUP).
// - Wiring: switch to GND when CLOSED -> LOW=closed, HIGH=open.
constexpr uint8_t PIN_DOOR_SENSOR = 33;

// DS18B20 (OneWire) temperature sensor
constexpr uint8_t PIN_TEMP_SENSOR = 32;

// Output pins (active-LOW relays: LOW=ON, HIGH=OFF)
constexpr uint8_t PIN_HEATER_RELAY = 14;    // Relay CH1
constexpr uint8_t PIN_MOTOR_FWD_RELAY = 27; // Relay CH2
constexpr uint8_t PIN_MOTOR_REV_RELAY = 26; // Relay CH3
constexpr uint8_t PIN_AUX_RELAY = 25;       // Relay CH4 (reserved / future use)

// Audible feedback / alarm
constexpr uint8_t PIN_BUZZER = 23; // tone() output

// Keypad pins (4x4)
// IMPORTANT (ESP32):
// - Rows are configured as INPUT_PULLUP by the Keypad library; these pins must
//   support internal pull-ups.
// - Avoid holding keys during reset because some column pins are strapping pins.
constexpr uint8_t PIN_KEYPAD_ROW[4] = {16, 17, 18, 19};
constexpr uint8_t PIN_KEYPAD_COL[4] = {13, 4, 5, 15};

// I2C pins (ESP32 default, but explicitly initialized in code)
constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;

// Door sensor logic
constexpr bool DOOR_CLOSED = LOW;
constexpr bool DOOR_OPEN = HIGH;

#endif
