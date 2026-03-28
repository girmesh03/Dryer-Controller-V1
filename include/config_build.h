#ifndef CONFIG_BUILD_H
#define CONFIG_BUILD_H

#include <Arduino.h>
#include <pgmspace.h>

// Feature flags (compile-time)
//  - Set to 0 to compile out the Service menu/tools and save Flash/SRAM.
//  - Override from PlatformIO build_flags: -DENABLE_SERVICE_MENU=0
#ifndef ENABLE_SERVICE_MENU
#define ENABLE_SERVICE_MENU 1
#endif

// Operator Settings Menu (compile-time)
//  - Override from PlatformIO build_flags: -DENABLE_SETTINGS_MENU=0
#ifndef ENABLE_SETTINGS_MENU
#define ENABLE_SETTINGS_MENU 1
#endif

// Program editor inside the Settings menu (compile-time)
//  - Override from PlatformIO build_flags: -DENABLE_PROGRAM_EDITOR=0
#if ENABLE_SETTINGS_MENU
#ifndef ENABLE_PROGRAM_EDITOR
#define ENABLE_PROGRAM_EDITOR 1
#endif
#else
#define ENABLE_PROGRAM_EDITOR 0
#endif

// Cycle execution UI/logic (compile-time)
//  - Operator build should keep this enabled.
//  - Service/commissioning builds may disable to save Flash: -DENABLE_CYCLE_EXECUTION=0
#ifndef ENABLE_CYCLE_EXECUTION
#define ENABLE_CYCLE_EXECUTION 1
#endif

// Granular Service feature flags (compile-time)
//  - These allow compiling only the service tools you need for a given bench session.
//  - Override from PlatformIO build_flags, e.g. -DENABLE_SERVICE_FOPDT_ID=1
#if ENABLE_SERVICE_MENU
#ifndef ENABLE_SERVICE_DRUM_TEST
#define ENABLE_SERVICE_DRUM_TEST 1
#endif
#ifndef ENABLE_SERVICE_HEATER_TEST
#define ENABLE_SERVICE_HEATER_TEST 1
#endif
#ifndef ENABLE_SERVICE_PID_VIEW
#define ENABLE_SERVICE_PID_VIEW 1
#endif
#ifndef ENABLE_SERVICE_IO_TEST
#define ENABLE_SERVICE_IO_TEST 1
#endif
#ifndef ENABLE_SERVICE_FOPDT_ID
#define ENABLE_SERVICE_FOPDT_ID 1
#endif
#ifndef ENABLE_SERVICE_AUTOTUNE
#define ENABLE_SERVICE_AUTOTUNE 1
#endif
#ifndef ENABLE_SERVICE_FACTORY_RESET
#define ENABLE_SERVICE_FACTORY_RESET 1
#endif
#else
#define ENABLE_SERVICE_DRUM_TEST 0
#define ENABLE_SERVICE_HEATER_TEST 0
#define ENABLE_SERVICE_PID_VIEW 0
#define ENABLE_SERVICE_IO_TEST 0
#define ENABLE_SERVICE_FOPDT_ID 0
#define ENABLE_SERVICE_AUTOTUNE 0
#define ENABLE_SERVICE_FACTORY_RESET 0
#endif

// Firmware identification (store in Flash / PROGMEM)
constexpr char FW_VERSION[] PROGMEM = "1.0.0";
constexpr char BUILD_DATE[] PROGMEM = __DATE__;
constexpr char BUILD_TIME[] PROGMEM = __TIME__;

// Legacy Arduino Nano memory limits (kept for reference).
// NOTE: This project now targets ESP32, and these limits are not enforced.
constexpr uint16_t FLASH_LIMIT = 25500;
constexpr uint16_t SRAM_LIMIT = 1740;
constexpr uint16_t EEPROM_LIMIT = 870;

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
