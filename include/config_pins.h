#ifndef CONFIG_PINS_H
#define CONFIG_PINS_H

#include <Arduino.h>

// Input pins
constexpr uint8_t PIN_DOOR_SENSOR = 6;  // D6, INPUT_PULLUP, LOW=open, HIGH=closed
constexpr uint8_t PIN_TEMP_SENSOR = 12; // D12, OneWire DS18B20

// Output pins (active-HIGH)
constexpr uint8_t PIN_HEATER_RELAY = 9;    // D9, HIGH=ON
constexpr uint8_t PIN_MOTOR_FWD_RELAY = 10; // D10, HIGH=ON
constexpr uint8_t PIN_MOTOR_REV_RELAY = 11; // D11, HIGH=ON

// Audible feedback / alarm
constexpr uint8_t PIN_BUZZER = 8; // D8, tone() output

// Keypad pins
constexpr uint8_t PIN_KEYPAD_ROW[4] = {A0, A1, A2, A3};
constexpr uint8_t PIN_KEYPAD_COL[4] = {2, 3, 4, 5};

// I2C pins (hardware defined)
// SDA = A4, SCL = A5

// Door sensor logic
constexpr bool DOOR_OPEN = LOW;
constexpr bool DOOR_CLOSED = HIGH;

#endif

