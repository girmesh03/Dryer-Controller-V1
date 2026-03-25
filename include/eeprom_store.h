#ifndef EEPROM_STORE_H
#define EEPROM_STORE_H

#include <Arduino.h>

class EEPROMStore {
public:
  void init();
  void update(uint32_t now_ms); // Call at 1 Hz (slow loop)

  bool isValid() const;

  // PID gains (Kp, Ki, Kd)
  bool loadPIDGains(float& kp, float& ki, float& kd) const;
  void requestSavePIDGains(float kp, float ki, float kd);

  // FOPDT parameters (K, tau, L)
  bool loadFOPDT(float& k, float& tau, float& l) const;
  void storeFOPDT(float k, float tau, float l);
  void requestSaveFOPDT(float k, float tau, float l);

  // Factory reset of reserved EEPROM map
  void factoryReset();

private:
  static constexpr uint16_t EEPROM_USED_BYTES = 0x200; // 512 bytes reserved (Appendix D)

  static constexpr uint16_t ADDR_MAGIC = 0x000;
  static constexpr uint16_t ADDR_VERSION = 0x002;
  static constexpr uint16_t ADDR_CRC8 = 0x003;

  static constexpr uint16_t ADDR_PID = 0x108;  // 3 * float
  static constexpr uint16_t ADDR_FOPDT = 0x114; // 3 * float

  static constexpr uint16_t MAGIC_DR = 0x4452; // 'D''R'
  static constexpr uint8_t VERSION = 0x01;

  static constexpr uint32_t MIN_WRITE_INTERVAL_MS = 5000;

  struct {
    uint32_t last_write_ms;
    uint8_t flags;
  } state_;

  struct {
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float fopdt_k;
    float fopdt_tau;
    float fopdt_l;
  } pending_;

  static constexpr uint8_t FLAG_VALID = 1u << 0;
  static constexpr uint8_t FLAG_PENDING_PID = 1u << 1;
  static constexpr uint8_t FLAG_PENDING_FOPDT = 1u << 2;

  void setFlag(uint8_t flag, bool value);
  bool getFlag(uint8_t flag) const;

  uint8_t computeCRC8() const;
  void writeHeaderAndCRC();

  void writeU16(uint16_t addr, uint16_t value);
  uint16_t readU16(uint16_t addr) const;

  void writeFloat(uint16_t addr, float value);
  float readFloat(uint16_t addr) const;
};

#endif
