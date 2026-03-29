#ifndef EEPROM_STORE_H
#define EEPROM_STORE_H

#include <Arduino.h>

class EEPROMStore {
public:
  // EEPROM Settings block (20 bytes, packed)
  struct Settings {
    uint8_t temp_unit;       // 0=Celsius, 1=Fahrenheit
    uint8_t sound_enabled;   // 0=off, 1=on
    uint8_t lcd_contrast;    // 0-255 (reserved for future HW)
    uint8_t filter_interval; // cycles between reminders
    uint8_t reserved[16];
  } __attribute__((packed));

  // EEPROM Program entry (40 bytes, packed)
  struct Program {
    char name[13];         // 12 chars + '\0'
    uint8_t temp_setpoint; // °C
    uint8_t duration_min;  // Minutes
    uint8_t fwd_time_s;    // Seconds
    uint8_t rev_time_s;    // Seconds
    uint8_t stop_time_s;   // Seconds
    uint8_t duty_limit;    // 0-100 (%)
    uint8_t reserved[21];
  } __attribute__((packed));

  static_assert(sizeof(Settings) == 20u, "EEPROM Settings must be 20 bytes");
  static_assert(sizeof(Program) == 40u, "EEPROM Program must be 40 bytes");

  // Fault history entry (20 bytes, packed)
  struct FaultEntry {
    uint8_t code;          // FaultCode (Phase 10)
    uint32_t timestamp_s;  // seconds since boot
    float temperature_c;   // temperature at fault (°C)
    uint8_t reserved[11];
  } __attribute__((packed));

  static_assert(sizeof(FaultEntry) == 20u, "EEPROM FaultEntry must be 20 bytes");

  static constexpr uint8_t PROGRAM_COUNT = 6u;
  static constexpr uint8_t FAULT_HISTORY_COUNT = 10u;

  void init();
  void update(uint32_t now_ms); // Call at 1 Hz (slow loop)
  void update();               // Convenience wrapper

  bool isValid() const;

  // Settings
  bool loadSettings(Settings& settings) const;
  void saveSettings(const Settings& settings);
  void requestSaveSettings(const Settings& settings);

  // Programs
  bool loadProgram(uint8_t index, Program& prog) const;
  void saveProgram(uint8_t index, const Program& prog);
  void requestSaveProgram(uint8_t index, const Program& prog);
  static void getDefaultProgram(uint8_t index, Program& prog);

  // PID gains (Kp, Ki, Kd)
  bool loadPIDGains(float& kp, float& ki, float& kd) const;
  void savePIDGains(float kp, float ki, float kd);
  void requestSavePIDGains(float kp, float ki, float kd);

  // FOPDT parameters (K, tau, L)
  bool loadFOPDT(float& k, float& tau, float& l) const;
  void storeFOPDT(float k, float tau, float l); // immediate write (legacy helper)
  void saveFOPDT(float k, float tau, float l);
  void requestSaveFOPDT(float k, float tau, float l);

  // Factory reset of reserved EEPROM map
  void factoryReset();

  // Fault history ring buffer (Phase 10)
  void logFault(uint8_t code, uint32_t timestamp_s, float temp_c);
  bool getFaultHistory(uint8_t index, FaultEntry& entry) const; // index 0 = most recent
  void clearFaultHistory();

private:
  static constexpr uint16_t EEPROM_USED_BYTES = 0x200; // 512 bytes reserved (Appendix D)

  static constexpr uint16_t ADDR_MAGIC = 0x000;
  static constexpr uint16_t ADDR_VERSION = 0x002;
  static constexpr uint16_t ADDR_CRC8 = 0x003;

  static constexpr uint16_t ADDR_SETTINGS = 0x004;  // 20 bytes
  static constexpr uint16_t ADDR_PROGRAMS = 0x018;  // 6 * 40 bytes
  static constexpr uint16_t ADDR_PID = 0x108;  // 3 * float
  static constexpr uint16_t ADDR_FOPDT = 0x114; // 3 * float
  static constexpr uint16_t ADDR_FAULT_HISTORY = 0x120; // 200 bytes
  static constexpr uint16_t ADDR_FILTER_COUNTER = 0x1E8; // 4 bytes
  static constexpr uint16_t ADDR_FAULT_HISTORY_HEAD = 0x1EC; // 1 byte (ring buffer head index)

  static constexpr uint16_t MAGIC_DR = 0x4452; // 'D''R'
  static constexpr uint8_t VERSION = 0x01;

  static constexpr uint32_t MIN_WRITE_INTERVAL_MS = 5000;

  struct {
    uint32_t last_write_ms;
    uint8_t flags;
    Settings settings_cache;
  } state_;

  struct {
    Settings settings;
    Program program;
    uint8_t program_index;
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float fopdt_k;
    float fopdt_tau;
    float fopdt_l;
  } pending_;

  static constexpr uint8_t FLAG_VALID = 1u << 0;
  static constexpr uint8_t FLAG_PENDING_SETTINGS = 1u << 1;
  static constexpr uint8_t FLAG_PENDING_PROGRAM = 1u << 2;
  static constexpr uint8_t FLAG_PENDING_PID = 1u << 3;
  static constexpr uint8_t FLAG_PENDING_FOPDT = 1u << 4;

  void setFlag(uint8_t flag, bool value);
  bool getFlag(uint8_t flag) const;

  uint8_t computeCRC8() const;
  void writeHeaderAndCRC();

  void writeBytes(uint16_t addr, const uint8_t* data, uint8_t len);
  void readBytes(uint16_t addr, uint8_t* data, uint8_t len) const;

  void writeU16(uint16_t addr, uint16_t value);
  uint16_t readU16(uint16_t addr) const;

  void writeU32(uint16_t addr, uint32_t value);
  uint32_t readU32(uint16_t addr) const;

  void writeFloat(uint16_t addr, float value);
  float readFloat(uint16_t addr) const;
};

#endif
