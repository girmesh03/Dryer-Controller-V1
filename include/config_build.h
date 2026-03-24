#ifndef CONFIG_BUILD_H
#define CONFIG_BUILD_H

#include <Arduino.h>
#include <avr/pgmspace.h>

// Firmware identification (store in Flash / PROGMEM)
const char FW_VERSION[] PROGMEM = "1.0.0";
const char BUILD_DATE[] PROGMEM = __DATE__;
const char BUILD_TIME[] PROGMEM = __TIME__;

// Memory limits (85% of Arduino Nano resources)
constexpr uint16_t FLASH_LIMIT_BYTES = 25500;  // 85% of 30 KB usable
constexpr uint16_t SRAM_LIMIT_BYTES = 1740;    // 85% of 2 KB
constexpr uint16_t EEPROM_LIMIT_BYTES = 870;   // 85% of 1 KB

// Cooperative scheduler timing (milliseconds)
constexpr uint8_t TICK_FAST_MS = 50;      // 20 Hz (keypad + door)
constexpr uint8_t TICK_CONTROL_MS = 100; // 10 Hz (baseline control)
constexpr uint16_t TICK_MEDIUM_MS = 250; // 4 Hz
constexpr uint16_t TICK_SLOW_MS = 1000;  // 1 Hz

// Safety thresholds (°C)
constexpr uint8_t OVER_TEMP_THRESHOLD_C = 95;
constexpr uint8_t COOLDOWN_THRESHOLD_C = 45;
constexpr uint8_t MIN_TEMP_C = 40;
constexpr uint8_t MAX_TEMP_C = 85;

// Timing constants
constexpr uint8_t DEBOUNCE_TIME_MS = 50;
constexpr uint16_t MOTOR_DIRECTION_DELAY_MS = 2000;
constexpr uint8_t HEATER_MIN_ON_TIME_S = 10;
constexpr uint8_t HEATER_MIN_OFF_TIME_S = 10;
constexpr uint16_t WATCHDOG_TIMEOUT_MS = 2000;

// LCD
constexpr uint8_t LCD_I2C_ADDR = 0x27;

#endif

