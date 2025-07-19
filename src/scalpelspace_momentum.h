/*******************************************************************************
 * @file scalpelspace_momentum.h
 * @brief Momentum sensor hub library for Arduino.
 *******************************************************************************
 */

#ifndef SCALPELSPACE_MOMENTUM_H
#define SCALPELSPACE_MOMENTUM_H

#include <SPI.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "momentum_driver/momentum_driver.h"

#ifdef __cplusplus
}
#endif

class Momentum {
public:
  /**
   * @param csPin Chip-select pin for SPI.
   * @param serial Print device for serial printing (defaults to Serial).
   * @param spi SPI bus instance (defaults to SPI).
   * @param settings SPI settings (defaults to 4MHz, MSB first, MODE0).
   */

  explicit Momentum(uint8_t csPin, Print &serial = Serial, SPIClass &spi = SPI,
                    const SPISettings &settings = SPISettings(4000000, MSBFIRST,
                                                              SPI_MODE0));

  // Init.
  void begin();

  // Serial interface management.
  void setSerialOutput(Print &serial);

  // Request version.
  momentum_status_t getVersion(version_t &version);

  // Request sensor data.
  momentum_status_t getQuat(sensor_data_t &data);
  momentum_status_t getGyro(sensor_data_t &data);
  momentum_status_t getAccel(sensor_data_t &data);
  momentum_status_t getLinAccel(sensor_data_t &data);
  momentum_status_t getGrav(sensor_data_t &data);
  momentum_status_t getEnv(sensor_data_t &data);
  momentum_status_t getDateTime(sensor_data_t &data);
  momentum_status_t getCoord(sensor_data_t &data);
  momentum_status_t getAltSpeed(sensor_data_t &data);
  momentum_status_t getHeading(sensor_data_t &data);
  momentum_status_t getStats(sensor_data_t &data);
  void getAll(sensor_data_t &data);

  // Command reset.
  momentum_status_t reset(void);

  // Command LED.
  momentum_status_t setLED(uint8_t led_i, uint8_t r, uint8_t g, uint8_t b);

  // Print version.
  void printVersion(const version_t &v);

  // Print sensor data
  void printData(const sensor_data_t &data);
  void printDataSingleLine(const sensor_data_t &data);

private:
  uint8_t _csPin;
  Print *_serial;
  SPIClass *_spi;
  SPISettings _settings;
  uint8_t _sequence;
  momentum_frame_t _frame;

  momentum_status_t requestData(uint8_t frameType, sensor_data_t &data,
                                version_t &version);

  void sendMomentumFrame(const momentum_frame_t &frame);
};

#endif
