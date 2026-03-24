#ifndef DS18B20_SENSOR_H
#define DS18B20_SENSOR_H

#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>

class DS18B20Sensor {
public:
  void init();
  void update(); // Call at ~4 Hz (250ms)

  float getTemperature() const;    // Filtered (EMA) temperature
  float getRawTemperature() const; // Most recent raw reading
  bool isValid() const;
  uint8_t getFaultCode() const; // 0=OK, 1=CRC/READ, 2=Disconnect, 3=Implausible

private:
  enum class State : uint8_t { IDLE, REQUEST_CONVERSION, WAIT_CONVERSION, READ_DATA };

  OneWire one_wire_;
  DallasTemperature sensors_;
  uint16_t conversion_wait_ms_;

  struct {
    State state;
    uint32_t state_start_ms;

    float raw_readings[5];
    uint8_t reading_index;
    uint8_t sample_count;

    float filtered_temp;
    float ema_temp;
    float last_valid_temp;
    uint32_t last_valid_ms;

    uint8_t fault_counter;
    uint8_t fault_code;
    uint8_t valid : 1;
  } state_;

  float computeMedian_() const;
  bool isPlausible_(float temp_c) const;
  bool checkRateOfChange_(float new_temp_c, uint32_t now_ms) const;
  float applyEMA_(float new_value);
  void recordFault_(uint8_t code, uint32_t now_ms);
};

#endif
