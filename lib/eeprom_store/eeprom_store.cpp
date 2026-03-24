#include "eeprom_store.h"

#include <Arduino.h>
#include <EEPROM.h>

namespace {
uint8_t crc8Update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x80u) {
      crc = static_cast<uint8_t>((crc << 1u) ^ 0x07u);
    } else {
      crc = static_cast<uint8_t>(crc << 1u);
    }
  }
  return crc;
}
} // namespace

void EEPROMStore::setFlag(uint8_t flag, bool value) {
  if (value) {
    state_.flags |= flag;
  } else {
    state_.flags &= static_cast<uint8_t>(~flag);
  }
}

bool EEPROMStore::getFlag(uint8_t flag) const {
  return (state_.flags & flag) != 0u;
}

bool EEPROMStore::isValid() const {
  return getFlag(FLAG_VALID);
}

void EEPROMStore::writeU16(uint16_t addr, uint16_t value) {
  EEPROM.update(addr, static_cast<uint8_t>((value >> 8u) & 0xFFu));
  EEPROM.update(static_cast<int>(addr + 1u), static_cast<uint8_t>(value & 0xFFu));
}

uint16_t EEPROMStore::readU16(uint16_t addr) const {
  const uint16_t hi = static_cast<uint16_t>(EEPROM.read(addr));
  const uint16_t lo = static_cast<uint16_t>(EEPROM.read(static_cast<int>(addr + 1u)));
  return static_cast<uint16_t>((hi << 8u) | lo);
}

void EEPROMStore::writeFloat(uint16_t addr, float value) {
  union {
    float f;
    uint8_t b[4];
  } u;
  u.f = value;

  for (uint8_t i = 0; i < 4u; i++) {
    EEPROM.update(static_cast<int>(addr + i), u.b[i]);
  }
}

float EEPROMStore::readFloat(uint16_t addr) const {
  union {
    float f;
    uint8_t b[4];
  } u;

  for (uint8_t i = 0; i < 4u; i++) {
    u.b[i] = EEPROM.read(static_cast<int>(addr + i));
  }

  return u.f;
}

uint8_t EEPROMStore::computeCRC8() const {
  uint8_t crc = 0x00u;

  for (uint16_t addr = 0u; addr < EEPROM_USED_BYTES; addr++) {
    if (addr == ADDR_CRC8) {
      continue;
    }
    const uint8_t data = EEPROM.read(static_cast<int>(addr));
    crc = crc8Update(crc, data);
  }

  return crc;
}

void EEPROMStore::writeHeaderAndCRC() {
  writeU16(ADDR_MAGIC, MAGIC_DR);
  EEPROM.update(ADDR_VERSION, VERSION);

  const uint8_t crc = computeCRC8();
  EEPROM.update(ADDR_CRC8, crc);

  setFlag(FLAG_VALID, true);
}

void EEPROMStore::factoryReset() {
  // Initialize entire reserved map to deterministic defaults (0x00),
  // then write header + CRC. This ensures CRC validation passes at boot.
  for (uint16_t addr = 0u; addr < EEPROM_USED_BYTES; addr++) {
    EEPROM.update(static_cast<int>(addr), 0x00u);
  }

  // Default PID gains (design doc): Kp=10.0, Ki=0.5, Kd=2.0
  writeFloat(ADDR_PID + 0u, 10.0f);
  writeFloat(ADDR_PID + 4u, 0.5f);
  writeFloat(ADDR_PID + 8u, 2.0f);

  // Default FOPDT parameters: 0.0
  writeFloat(ADDR_FOPDT + 0u, 0.0f);
  writeFloat(ADDR_FOPDT + 4u, 0.0f);
  writeFloat(ADDR_FOPDT + 8u, 0.0f);

  writeHeaderAndCRC();
  state_.last_write_ms = millis();
  setFlag(FLAG_PENDING_PID, false);
  setFlag(FLAG_PENDING_FOPDT, false);
}

void EEPROMStore::init() {
  state_.last_write_ms = 0;
  state_.flags = 0;

  // Initialize pending data from current EEPROM contents (even if invalid).
  pending_.pid_kp = readFloat(ADDR_PID + 0u);
  pending_.pid_ki = readFloat(ADDR_PID + 4u);
  pending_.pid_kd = readFloat(ADDR_PID + 8u);
  pending_.fopdt_k = readFloat(ADDR_FOPDT + 0u);
  pending_.fopdt_tau = readFloat(ADDR_FOPDT + 4u);
  pending_.fopdt_l = readFloat(ADDR_FOPDT + 8u);

  const uint16_t magic = readU16(ADDR_MAGIC);
  const uint8_t version = EEPROM.read(ADDR_VERSION);
  const uint8_t stored_crc = EEPROM.read(ADDR_CRC8);
  const uint8_t computed_crc = computeCRC8();

  const bool ok = (magic == MAGIC_DR) && (version == VERSION) && (stored_crc == computed_crc);
  if (!ok) {
    factoryReset();
    return;
  }

  setFlag(FLAG_VALID, true);
}

void EEPROMStore::update(uint32_t now_ms) {
  if (!isValid()) {
    return;
  }

  const bool pending_pid = getFlag(FLAG_PENDING_PID);
  const bool pending_fopdt = getFlag(FLAG_PENDING_FOPDT);
  if (!pending_pid && !pending_fopdt) {
    return;
  }

  if (now_ms - state_.last_write_ms < MIN_WRITE_INTERVAL_MS) {
    return;
  }

  if (pending_pid) {
    writeFloat(ADDR_PID + 0u, pending_.pid_kp);
    writeFloat(ADDR_PID + 4u, pending_.pid_ki);
    writeFloat(ADDR_PID + 8u, pending_.pid_kd);
    setFlag(FLAG_PENDING_PID, false);
  }

  if (pending_fopdt) {
    writeFloat(ADDR_FOPDT + 0u, pending_.fopdt_k);
    writeFloat(ADDR_FOPDT + 4u, pending_.fopdt_tau);
    writeFloat(ADDR_FOPDT + 8u, pending_.fopdt_l);
    setFlag(FLAG_PENDING_FOPDT, false);
  }

  writeHeaderAndCRC();
  state_.last_write_ms = now_ms;
}

bool EEPROMStore::loadPIDGains(float& kp, float& ki, float& kd) const {
  if (!isValid()) {
    return false;
  }

  kp = readFloat(ADDR_PID + 0u);
  ki = readFloat(ADDR_PID + 4u);
  kd = readFloat(ADDR_PID + 8u);

  // NaN check without pulling in libm.
  if (!((kp == kp) && (ki == ki) && (kd == kd))) {
    return false;
  }

  return true;
}

void EEPROMStore::requestSavePIDGains(float kp, float ki, float kd) {
  pending_.pid_kp = kp;
  pending_.pid_ki = ki;
  pending_.pid_kd = kd;
  setFlag(FLAG_PENDING_PID, true);
}

bool EEPROMStore::loadFOPDT(float& k, float& tau, float& l) const {
  if (!isValid()) {
    return false;
  }

  k = readFloat(ADDR_FOPDT + 0u);
  tau = readFloat(ADDR_FOPDT + 4u);
  l = readFloat(ADDR_FOPDT + 8u);

  if (!((k == k) && (tau == tau) && (l == l))) {
    return false;
  }

  return true;
}

void EEPROMStore::requestSaveFOPDT(float k, float tau, float l) {
  pending_.fopdt_k = k;
  pending_.fopdt_tau = tau;
  pending_.fopdt_l = l;
  setFlag(FLAG_PENDING_FOPDT, true);
}

