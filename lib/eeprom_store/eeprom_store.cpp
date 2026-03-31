#include "eeprom_store.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <pgmspace.h>

#include "config_build.h"

namespace {
inline void eepromWrite8IfChanged(uint16_t addr, uint8_t value) {
  const int a = static_cast<int>(addr);
  const uint8_t cur = EEPROM.read(a);
  if (cur != value) {
    EEPROM.write(a, value);
  }
}

inline void eepromCommitIfNeeded() {
#if defined(ESP32)
  (void)EEPROM.commit();
#endif
}

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

const EEPROMStore::Settings kDefaultSettings PROGMEM = {
    0u,   // temp_unit (C)
    1u,   // sound_enabled
    128u, // lcd_contrast
    50u,  // filter_interval (Req 43 default)
    {0u},
};

const EEPROMStore::Program kDefaultPrograms[EEPROMStore::PROGRAM_COUNT] PROGMEM = {
    {"TOWELS", 75u, 40u, 45u, 45u, 5u, 100u, {0u}},
    {"BED SHEETS", 60u, 45u, 60u, 60u, 6u, 100u, {0u}},
    {"DELICATES", 50u, 30u, 40u, 40u, 8u, 100u, {0u}},
    {"HEAVY COTTON", 80u, 50u, 50u, 50u, 5u, 100u, {0u}},
    {"MIXED LOAD", 65u, 35u, 50u, 50u, 5u, 100u, {0u}},
    {"SYNTHETIC", 55u, 25u, 45u, 45u, 6u, 100u, {0u}},
};

void getDefaultSettings_(EEPROMStore::Settings& out) {
  memcpy_P(&out, &kDefaultSettings, sizeof(EEPROMStore::Settings));
  out.reserved[0] = 0u; // keep deterministic even if toolchain differs
}

bool settingsSane_(const EEPROMStore::Settings& s) {
  if (s.temp_unit > 1u) return false;
  if (s.sound_enabled > 1u) return false;
  if (s.filter_interval == 0u) return false;
  return true;
}

bool programSane_(const EEPROMStore::Program& p) {
  if (p.temp_setpoint < MIN_TEMP || p.temp_setpoint > MAX_TEMP) return false;
  if (p.duration_min < 10u || p.duration_min > 120u) return false;
  if (p.fwd_time_s < 30u || p.fwd_time_s > 90u) return false;
  if (p.rev_time_s < 30u || p.rev_time_s > 90u) return false;
  if (p.stop_time_s < 3u || p.stop_time_s > 15u) return false;
  if (p.duty_limit > 100u) return false;
  return true;
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

bool EEPROMStore::wasFactoryResetThisBoot() const {
  return getFlag(FLAG_INIT_FACTORY_RESET);
}

void EEPROMStore::writeBytes(uint16_t addr, const uint8_t* data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    eepromWrite8IfChanged(static_cast<uint16_t>(addr + i), data[i]);
  }
}

void EEPROMStore::readBytes(uint16_t addr, uint8_t* data, uint8_t len) const {
  for (uint8_t i = 0; i < len; i++) {
    data[i] = EEPROM.read(static_cast<int>(addr + i));
  }
}

void EEPROMStore::writeU16(uint16_t addr, uint16_t value) {
  eepromWrite8IfChanged(addr, static_cast<uint8_t>((value >> 8u) & 0xFFu));
  eepromWrite8IfChanged(static_cast<uint16_t>(addr + 1u), static_cast<uint8_t>(value & 0xFFu));
}

uint16_t EEPROMStore::readU16(uint16_t addr) const {
  const uint16_t hi = static_cast<uint16_t>(EEPROM.read(addr));
  const uint16_t lo = static_cast<uint16_t>(EEPROM.read(static_cast<int>(addr + 1u)));
  return static_cast<uint16_t>((hi << 8u) | lo);
}

void EEPROMStore::writeU32(uint16_t addr, uint32_t value) {
  eepromWrite8IfChanged(static_cast<uint16_t>(addr + 0u), static_cast<uint8_t>((value >> 24u) & 0xFFu));
  eepromWrite8IfChanged(static_cast<uint16_t>(addr + 1u), static_cast<uint8_t>((value >> 16u) & 0xFFu));
  eepromWrite8IfChanged(static_cast<uint16_t>(addr + 2u), static_cast<uint8_t>((value >> 8u) & 0xFFu));
  eepromWrite8IfChanged(static_cast<uint16_t>(addr + 3u), static_cast<uint8_t>(value & 0xFFu));
}

uint32_t EEPROMStore::readU32(uint16_t addr) const {
  const uint32_t b0 = static_cast<uint32_t>(EEPROM.read(static_cast<int>(addr + 0u)));
  const uint32_t b1 = static_cast<uint32_t>(EEPROM.read(static_cast<int>(addr + 1u)));
  const uint32_t b2 = static_cast<uint32_t>(EEPROM.read(static_cast<int>(addr + 2u)));
  const uint32_t b3 = static_cast<uint32_t>(EEPROM.read(static_cast<int>(addr + 3u)));
  return (b0 << 24u) | (b1 << 16u) | (b2 << 8u) | b3;
}

void EEPROMStore::writeFloat(uint16_t addr, float value) {
  union {
    float f;
    uint8_t b[4];
  } u;
  u.f = value;

  for (uint8_t i = 0; i < 4u; i++) {
    eepromWrite8IfChanged(static_cast<uint16_t>(addr + i), u.b[i]);
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
  eepromWrite8IfChanged(ADDR_VERSION, VERSION);

  const uint8_t crc = computeCRC8();
  eepromWrite8IfChanged(ADDR_CRC8, crc);

  setFlag(FLAG_VALID, true);
}

void EEPROMStore::factoryReset() {
  state_.flags = 0u;

  // Initialize entire reserved map to deterministic defaults (0x00),
  // then write defaults and header + CRC. This ensures CRC validation passes at boot.
  for (uint16_t addr = 0u; addr < EEPROM_USED_BYTES; addr++) {
    eepromWrite8IfChanged(addr, 0x00u);
  }

  // Default Settings.
  {
    Settings settings;
    getDefaultSettings_(settings);
    writeBytes(ADDR_SETTINGS, reinterpret_cast<const uint8_t*>(&settings), sizeof(Settings));
    state_.settings_cache = settings;
    pending_.settings = settings;
  }

  // Default Programs.
  for (uint8_t i = 0u; i < PROGRAM_COUNT; i++) {
    Program prog;
    getDefaultProgram(i, prog);
    const uint16_t base = static_cast<uint16_t>(ADDR_PROGRAMS + (static_cast<uint16_t>(i) * sizeof(Program)));
    writeBytes(base, reinterpret_cast<const uint8_t*>(&prog), sizeof(Program));
  }

  // Default PID gains (design doc): Kp=10.0, Ki=0.5, Kd=2.0
  writeFloat(ADDR_PID + 0u, 10.0f);
  writeFloat(ADDR_PID + 4u, 0.5f);
  writeFloat(ADDR_PID + 8u, 2.0f);

  // Default FOPDT parameters: 0.0
  writeFloat(ADDR_FOPDT + 0u, 0.0f);
  writeFloat(ADDR_FOPDT + 4u, 0.0f);
  writeFloat(ADDR_FOPDT + 8u, 0.0f);

  // Filter clean counter defaults to 0 cycles.
  writeU32(ADDR_FILTER_COUNTER, 0u);

  writeHeaderAndCRC();
  eepromCommitIfNeeded();
  state_.last_write_ms = millis();
  setFlag(FLAG_PENDING_SETTINGS, false);
  setFlag(FLAG_PENDING_PROGRAM, false);
  setFlag(FLAG_PENDING_PID, false);
  setFlag(FLAG_PENDING_FOPDT, false);
}

void EEPROMStore::init() {
  state_.last_write_ms = 0;
  state_.flags = 0;
  setFlag(FLAG_INIT_FACTORY_RESET, false);

#if defined(ESP32)
  // ESP32 EEPROM is flash-emulated and must be initialized with a size.
  // Initialize the full 1 KB region (matches AVR EEPROM capacity / design.md).
  // The CRC-protected reserved map currently uses the first 512 bytes (Appendix D).
  if (!EEPROM.begin(EEPROM_TOTAL_BYTES)) {
    // If initialization fails, keep FLAG_VALID cleared; callers will fall back.
    return;
  }
#endif

  // Initialize cache to defaults in case EEPROM is invalid.
  getDefaultSettings_(state_.settings_cache);
  pending_.settings = state_.settings_cache;
  pending_.program_index = 0u;

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
    setFlag(FLAG_INIT_FACTORY_RESET, true);
    return;
  }

  setFlag(FLAG_VALID, true);

  // Load Settings cache.
  readBytes(ADDR_SETTINGS, reinterpret_cast<uint8_t*>(&state_.settings_cache), sizeof(Settings));
  state_.settings_cache.reserved[15] = 0u; // deterministic, also touches the struct to avoid warnings

  if (!settingsSane_(state_.settings_cache)) {
    getDefaultSettings_(state_.settings_cache);
  }
}

void EEPROMStore::update(uint32_t now_ms) {
  if (!isValid()) {
    return;
  }

  const bool pending_settings = getFlag(FLAG_PENDING_SETTINGS);
  const bool pending_program = getFlag(FLAG_PENDING_PROGRAM);
  const bool pending_pid = getFlag(FLAG_PENDING_PID);
  const bool pending_fopdt = getFlag(FLAG_PENDING_FOPDT);
  if (!pending_settings && !pending_program && !pending_pid && !pending_fopdt) {
    return;
  }

  if (now_ms - state_.last_write_ms < MIN_WRITE_INTERVAL_MS) {
    return;
  }

  if (pending_settings) {
    writeBytes(ADDR_SETTINGS, reinterpret_cast<const uint8_t*>(&pending_.settings), sizeof(Settings));
    state_.settings_cache = pending_.settings;
    setFlag(FLAG_PENDING_SETTINGS, false);
  }

  if (pending_program) {
    const uint8_t idx = pending_.program_index;
    if (idx < PROGRAM_COUNT) {
      const uint16_t base = static_cast<uint16_t>(ADDR_PROGRAMS + (static_cast<uint16_t>(idx) * sizeof(Program)));
      writeBytes(base, reinterpret_cast<const uint8_t*>(&pending_.program), sizeof(Program));
    }
    setFlag(FLAG_PENDING_PROGRAM, false);
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
  eepromCommitIfNeeded();
  state_.last_write_ms = now_ms;
}

void EEPROMStore::update() {
  update(millis());
}

bool EEPROMStore::loadSettings(Settings& settings) const {
  if (!isValid()) {
    return false;
  }
  settings = state_.settings_cache;
  return true;
}

void EEPROMStore::saveSettings(const Settings& settings) {
  requestSaveSettings(settings);
}

void EEPROMStore::requestSaveSettings(const Settings& settings) {
  pending_.settings = settings;
  setFlag(FLAG_PENDING_SETTINGS, true);
}

bool EEPROMStore::loadProgram(uint8_t index, Program& prog) const {
  if (!isValid()) {
    return false;
  }
  if (index >= PROGRAM_COUNT) {
    return false;
  }

  const uint16_t base = static_cast<uint16_t>(ADDR_PROGRAMS + (static_cast<uint16_t>(index) * sizeof(Program)));
  readBytes(base, reinterpret_cast<uint8_t*>(&prog), sizeof(Program));
  prog.name[12] = '\0';

  if (!programSane_(prog)) {
    getDefaultProgram(index, prog);
  }

  return true;
}

void EEPROMStore::saveProgram(uint8_t index, const Program& prog) {
  requestSaveProgram(index, prog);
}

void EEPROMStore::requestSaveProgram(uint8_t index, const Program& prog) {
  if (index >= PROGRAM_COUNT) {
    return;
  }
  pending_.program_index = index;
  pending_.program = prog;
  pending_.program.name[12] = '\0';
  setFlag(FLAG_PENDING_PROGRAM, true);
}

void EEPROMStore::getDefaultProgram(uint8_t index, Program& prog) {
  if (index >= PROGRAM_COUNT) {
    index = 0u;
  }
  memcpy_P(&prog, &kDefaultPrograms[index], sizeof(Program));
  prog.name[12] = '\0';
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

void EEPROMStore::savePIDGains(float kp, float ki, float kd) {
  requestSavePIDGains(kp, ki, kd);
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

void EEPROMStore::storeFOPDT(float k, float tau, float l) {
  if (!isValid()) {
    return;
  }

  pending_.fopdt_k = k;
  pending_.fopdt_tau = tau;
  pending_.fopdt_l = l;

  writeFloat(ADDR_FOPDT + 0u, k);
  writeFloat(ADDR_FOPDT + 4u, tau);
  writeFloat(ADDR_FOPDT + 8u, l);

  writeHeaderAndCRC();
  eepromCommitIfNeeded();
  state_.last_write_ms = millis();
  setFlag(FLAG_PENDING_FOPDT, false);
}

void EEPROMStore::saveFOPDT(float k, float tau, float l) {
  requestSaveFOPDT(k, tau, l);
}

void EEPROMStore::requestSaveFOPDT(float k, float tau, float l) {
  pending_.fopdt_k = k;
  pending_.fopdt_tau = tau;
  pending_.fopdt_l = l;
  setFlag(FLAG_PENDING_FOPDT, true);
}

void EEPROMStore::logFault(uint8_t code, uint32_t timestamp_s, float temp_c) {
  if (!isValid()) {
    return;
  }
  if (code == 0u) {
    return;
  }

  FaultEntry entry;
  entry.code = code;
  entry.timestamp_s = timestamp_s;
  entry.temperature_c = temp_c;
  for (uint8_t i = 0u; i < sizeof(entry.reserved); i++) {
    entry.reserved[i] = 0u;
  }

  uint8_t head = EEPROM.read(static_cast<int>(ADDR_FAULT_HISTORY_HEAD));
  if (head >= FAULT_HISTORY_COUNT) {
    head = 0u;
  }

  const uint16_t base =
      static_cast<uint16_t>(ADDR_FAULT_HISTORY + (static_cast<uint16_t>(head) * sizeof(FaultEntry)));
  writeBytes(base, reinterpret_cast<const uint8_t*>(&entry), sizeof(FaultEntry));

  head = static_cast<uint8_t>((head + 1u) % FAULT_HISTORY_COUNT);
  eepromWrite8IfChanged(ADDR_FAULT_HISTORY_HEAD, head);

  writeHeaderAndCRC();
  eepromCommitIfNeeded();

  // Fault logs are infrequent; treat as an immediate write for wear + rate limiting.
  state_.last_write_ms = millis();
}

bool EEPROMStore::getFaultHistory(uint8_t index, FaultEntry& entry) const {
  if (!isValid()) {
    return false;
  }
  if (index >= FAULT_HISTORY_COUNT) {
    return false;
  }

  uint8_t head = EEPROM.read(static_cast<int>(ADDR_FAULT_HISTORY_HEAD));
  if (head >= FAULT_HISTORY_COUNT) {
    head = 0u;
  }

  // Return most-recent-first ordering for UI convenience.
  const uint8_t phys =
      static_cast<uint8_t>((head + FAULT_HISTORY_COUNT - 1u - index) % FAULT_HISTORY_COUNT);

  const uint16_t base =
      static_cast<uint16_t>(ADDR_FAULT_HISTORY + (static_cast<uint16_t>(phys) * sizeof(FaultEntry)));
  readBytes(base, reinterpret_cast<uint8_t*>(&entry), sizeof(FaultEntry));

  return entry.code != 0u;
}

void EEPROMStore::clearFaultHistory() {
  if (!isValid()) {
    return;
  }

  for (uint16_t addr = 0u; addr < static_cast<uint16_t>(FAULT_HISTORY_COUNT * sizeof(FaultEntry)); addr++) {
    eepromWrite8IfChanged(static_cast<uint16_t>(ADDR_FAULT_HISTORY + addr), 0x00u);
  }
  eepromWrite8IfChanged(ADDR_FAULT_HISTORY_HEAD, 0u);

  writeHeaderAndCRC();
  eepromCommitIfNeeded();
  state_.last_write_ms = millis();
}
