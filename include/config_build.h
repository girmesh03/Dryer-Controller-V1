#ifndef CONFIG_BUILD_H
#define CONFIG_BUILD_H

#include <Arduino.h>
#include <avr/pgmspace.h>

// Feature flags (compile-time)
//  - Set to 0 to compile out the Service menu/tools and save Flash/SRAM.
//  - Override from PlatformIO build_flags: -DENABLE_SERVICE_MENU=0
#ifndef ENABLE_SERVICE_MENU
#define ENABLE_SERVICE_MENU 1
#endif

// Firmware identification (store in Flash / PROGMEM)
constexpr char FW_VERSION[] PROGMEM = "1.0.0";
constexpr char BUILD_DATE[] PROGMEM = __DATE__;
constexpr char BUILD_TIME[] PROGMEM = __TIME__;

// Memory limits (85% of Arduino Nano resources)
constexpr uint16_t FLASH_LIMIT = 25500; // 85% of 30 KB usable
constexpr uint16_t SRAM_LIMIT = 1740;   // 85% of 2 KB
constexpr uint16_t EEPROM_LIMIT = 870;  // 85% of 1 KB

// Cooperative scheduler timing (milliseconds)
constexpr uint8_t FAST_LOOP_PERIOD = 100;    // 10 Hz
constexpr uint16_t MEDIUM_LOOP_PERIOD = 250; // 4 Hz
constexpr uint16_t SLOW_LOOP_PERIOD = 1000;  // 1 Hz

// Safety thresholds (°C)
constexpr uint8_t OVER_TEMP_THRESHOLD = 95;
constexpr uint8_t COOLDOWN_THRESHOLD = 45;
constexpr uint8_t MIN_TEMP = 40;
constexpr uint8_t MAX_TEMP = 85;

// Timing constants
constexpr uint8_t DEBOUNCE_TIME_MS = 50;
constexpr uint16_t MOTOR_DIRECTION_DELAY_MS = 2000;
constexpr uint8_t HEATER_MIN_ON_TIME_S = 10;
constexpr uint8_t HEATER_MIN_OFF_TIME_S = 10;
constexpr uint16_t WATCHDOG_TIMEOUT_MS = 2000;

// LCD
constexpr uint8_t LCD_I2C_ADDR = 0x27;

#endif
