#include <Arduino.h>
#include <Wire.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#if defined(__AVR__)
#include <avr/io.h>
#include <avr/wdt.h>
#endif
#if defined(ESP32)
#include <esp_system.h>
#include <esp_task_wdt.h>
#endif

#include "config_build.h"
#include "config_pins.h"
#include "ds18b20_sensor.h"
#include "drum_control.h"
#include "heater_control.h"
#include "pid_control.h"
#include "eeprom_store.h"
#include "io_abstraction.h"
#include "faults.h"
#include "ui_lcd.h"
#include "keypad_input.h"
#include "app.h"
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_FOPDT_ID
#include "fopdt_model.h"
#endif

#ifndef ENABLE_SERIAL_DEBUG
#define ENABLE_SERIAL_DEBUG 0
#endif

IOAbstraction io;
EEPROMStore eepromStore;
FaultManager faultMgr;

bool g_was_watchdog_reset = false;
uint8_t g_reset_cause_flags = 0u; // bit0=WDT, bit1=BROWNOUT

// Phase 5 service-mode flag for HEATER TEST.
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_HEATER_TEST
uint8_t g_heater_test_active = 0u;
#endif
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_FOPDT_ID
uint8_t g_fopdt_active = 0u;
#endif

#if ENABLE_SETTINGS_MENU
EEPROMStore::Settings g_settings;
#endif

// Phase 6+ will manage this setpoint during RUNNING_HEAT.
float g_setpoint_c = 0.0f;

UILCD lcd;
KeypadInput keypad;
AppStateMachine app;
DS18B20Sensor tempSensor;
DrumControl drumControl;
HeaterControl heaterControl;
PIDControl pidController;
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_FOPDT_ID
FOPDTModel fopdt;
#endif

namespace
{
#if ENABLE_SERIAL_DEBUG
  void printResetCause_()
  {
#if defined(ESP32)
    const esp_reset_reason_t reason = esp_reset_reason();
    Serial.print(F("Reset cause: "));
    switch (reason)
    {
    case ESP_RST_POWERON:
      Serial.println(F("POWER-ON"));
      break;
    case ESP_RST_BROWNOUT:
      Serial.println(F("BROWN-OUT"));
      break;
    case ESP_RST_EXT:
      Serial.println(F("EXTERNAL"));
      break;
    case ESP_RST_SW:
      Serial.println(F("SOFTWARE"));
      break;
    case ESP_RST_PANIC:
      Serial.println(F("PANIC"));
      break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      Serial.println(F("WATCHDOG"));
      break;
    default:
      Serial.println(F("UNKNOWN"));
      break;
    }
#elif defined(__AVR__)
    const uint8_t flags = MCUSR;
    Serial.print(F("RESET FLAGS (MCUSR)=0x"));
    Serial.println(flags, HEX);
    if (flags & _BV(PORF))
      Serial.println(F("Reset cause: POWER-ON"));
    if (flags & _BV(EXTRF))
      Serial.println(F("Reset cause: EXTERNAL"));
    if (flags & _BV(BORF))
      Serial.println(F("Reset cause: BROWN-OUT"));
    if (flags & _BV(WDRF))
      Serial.println(F("Reset cause: WATCHDOG"));
#else
    Serial.println(F("Reset cause: UNKNOWN"));
#endif
  }
#endif

  void captureResetCause_()
  {
#if defined(__AVR__)
    const uint8_t flags = MCUSR;
    g_was_watchdog_reset = (flags & _BV(WDRF)) != 0;
    g_reset_cause_flags = 0u;
    if (flags & _BV(WDRF))
      g_reset_cause_flags |= 0x01u;
    if (flags & _BV(BORF))
      g_reset_cause_flags |= 0x02u;
    MCUSR = 0;
    wdt_disable();
#elif defined(ESP32)
    const esp_reset_reason_t reason = esp_reset_reason();
    g_reset_cause_flags = 0u;
    const bool wdt = (reason == ESP_RST_TASK_WDT) || (reason == ESP_RST_INT_WDT) || (reason == ESP_RST_WDT);
    const bool brownout = (reason == ESP_RST_BROWNOUT);
    if (wdt)
      g_reset_cause_flags |= 0x01u;
    if (brownout)
      g_reset_cause_flags |= 0x02u;
    g_was_watchdog_reset = wdt;
#else
    g_was_watchdog_reset = false;
    g_reset_cause_flags = 0u;
#endif
  }

  void watchdogInit_()
  {
#if defined(__AVR__)
    wdt_enable(WDTO_2S);
#elif defined(ESP32)
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
    esp_task_wdt_config_t cfg = {};
    cfg.timeout_ms = WATCHDOG_TIMEOUT_MS;
    cfg.idle_core_mask = 0u;
    cfg.trigger_panic = false;
    (void)esp_task_wdt_init(&cfg);
#else
    const int timeout_s = static_cast<int>((WATCHDOG_TIMEOUT_MS + 999u) / 1000u);
    (void)esp_task_wdt_init(timeout_s, true);
#endif
    (void)esp_task_wdt_add(NULL);
#endif
  }

  inline void watchdogKick_()
  {
#if defined(__AVR__)
    wdt_reset();
#elif defined(ESP32)
    (void)esp_task_wdt_reset();
#endif
  }
} // namespace

void setup()
{
  captureResetCause_();

#if defined(ESP32)
  // Ensure I2C uses the configured pins (SDA/SCL).
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
#endif

  eepromStore.init();
#if ENABLE_SETTINGS_MENU
  (void)eepromStore.loadSettings(g_settings);
#endif
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_FOPDT_ID
  // Phase 7.4: load persisted FOPDT parameters (even if not used immediately).
  {
    float k = 0.0f;
    float tau = 0.0f;
    float l = 0.0f;
    (void)eepromStore.loadFOPDT(k, tau, l);
  }
#endif

  io.init();
  faultMgr.init();

  // Phase 10.9: log watchdog/brownout reset causes as acknowledge-only faults.
  if ((g_reset_cause_flags & 0x01u) != 0u)
  {
    faultMgr.setFault(FaultCode::WATCHDOG_RESET);
  }
  else if ((g_reset_cause_flags & 0x02u) != 0u)
  {
    faultMgr.setFault(FaultCode::BROWNOUT);
  }

  pidController.init();
  {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (eepromStore.loadPIDGains(kp, ki, kd))
    {
      pidController.setTunings(kp, ki, kd);
    }
  }

  tempSensor.init();
  drumControl.init();
  heaterControl.init();
#if ENABLE_SERVICE_MENU && ENABLE_SERVICE_FOPDT_ID
  fopdt.init();
#endif
  lcd.init();
#if ENABLE_SETTINGS_MENU
  lcd.setTempUnit(g_settings.temp_unit);
#endif
  keypad.init();
  app.init();

#if ENABLE_SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("Industrial Dryer Controller - Phase 6"));
  printResetCause_();

  Serial.print(F("EEPROM CRC valid: "));
  Serial.println(eepromStore.isValid() ? F("YES") : F("NO"));
  Serial.println(F("IO initialized: outputs forced OFF, door input pull-up enabled"));
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Enable watchdog after initialization completes.
  watchdogInit_();
}

void loop()
{
  // Reset watchdog - must be called every loop iteration
  watchdogKick_();
  const uint32_t now = millis();

  static uint32_t last_keypad_ms = 0;
  static uint32_t last_io_ms = 0;
  static uint32_t last_app_ms = 0;
  static uint32_t last_drum_ms = 0;
  static uint32_t last_heater_ms = 0;
  static uint32_t last_lcd_ms = 0;
  static uint32_t last_temp_ms = 0;
  static uint32_t last_slow_ms = 0;
  static bool last_door_closed = true;

  // Keypad scan loop: 20 Hz (50ms)
  if (now - last_keypad_ms >= 50u)
  {
    last_keypad_ms = now;
    keypad.update();

    const auto key = keypad.getKey();
    if (key != KeypadInput::Key::NONE)
    {
#if ENABLE_SETTINGS_MENU
      if (g_settings.sound_enabled != 0u)
      {
        io.beepKey();
      }
#endif
      app.handleKeyPress(key);
    }
  }

  // IO loop: 20 Hz (50ms) for door response; debounce remains >=50ms.
  if (now - last_io_ms >= 50u)
  {
    last_io_ms = now;
    io.update();

    // Door state change reporting (useful for Phase 1 bench verification).
    const bool door_closed = io.isDoorClosed();
    if (door_closed != last_door_closed)
    {
      last_door_closed = door_closed;
#if ENABLE_SERIAL_DEBUG
      Serial.println(door_closed ? F("DOOR: CLOSED") : F("DOOR: OPEN"));
#endif
    }
  }

  // Fault manager + App state machine: 10 Hz (100ms)
  if (now - last_app_ms >= FAST_LOOP_PERIOD)
  {
    last_app_ms = now;

    faultMgr.update();
    if (faultMgr.hasFault())
    {
      io.emergencyStop();
      if (app.getCurrentState() != SystemState::FAULT)
      {
        app.transitionTo(SystemState::FAULT);
      }
    }

    app.update();
  }

  // Drum control: 10 Hz (100ms)
  if (now - last_drum_ms >= FAST_LOOP_PERIOD)
  {
    last_drum_ms = now;
    drumControl.update();

    const auto dir = drumControl.getCurrentDirection();
    io.setMotorForward(dir == DrumControl::Direction::FORWARD);
    io.setMotorReverse(dir == DrumControl::Direction::REVERSE);
  }

  // Heater control: 10 Hz (100ms)
  if (now - last_heater_ms >= FAST_LOOP_PERIOD)
  {
    last_heater_ms = now;

    if (tempSensor.isValid() && app.getCurrentState() == SystemState::RUNNING_HEAT)
    {
      const float temp_c = tempSensor.getTemperature();
      const float pid_out = pidController.compute(temp_c); // 0-100%
      g_setpoint_c = pidController.getTargetSetpoint();
      heaterControl.setDutyCycle(pid_out);
    }

    heaterControl.update();
    io.setHeaterRelay(heaterControl.isHeaterOn());
  }

  // DS18B20 state machine: 4 Hz (250ms)
  if (now - last_temp_ms >= MEDIUM_LOOP_PERIOD)
  {
    last_temp_ms = now;
    tempSensor.update();
  }

  // LCD refresh: >= 5 Hz (200ms)
  if (now - last_lcd_ms >= 200u)
  {
    last_lcd_ms = now;
    lcd.update();
  }

  // Slow tick: EEPROM deferred writes, diagnostics, heartbeat (1 Hz).
  if (now - last_slow_ms >= SLOW_LOOP_PERIOD)
  {
    last_slow_ms = now;
    eepromStore.update(now);

#if ENABLE_SERIAL_DEBUG
    Serial.print(F("TEMP raw="));
    Serial.print(tempSensor.getRawTemperature(), 2);
    Serial.print(F("C filt="));
    Serial.print(tempSensor.getTemperature(), 2);
    Serial.print(F("C valid="));
    Serial.print(tempSensor.isValid() ? F("Y") : F("N"));
    Serial.print(F(" fault="));
    Serial.println(tempSensor.getFaultCode());
#endif

    // Heartbeat LED (visible progress on bench).
    digitalWrite(LED_BUILTIN, (digitalRead(LED_BUILTIN) == LOW) ? HIGH : LOW);
  }
}
