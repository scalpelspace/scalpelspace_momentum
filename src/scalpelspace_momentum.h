/*******************************************************************************
 * @file scalpelspace_momentum.cpp
 * @brief Momentum sensor hub library for Arduino.
 *******************************************************************************
 */

#ifndef SCALPELSPACE_MOMENTUM_H
#define SCALPELSPACE_MOMENTUM_H

#include "momentum_driver.h"
#include <SPI.h>

class Momentum {
public:
  explicit Momentum(uint8_t csPin);
  void begin();
  momentum_status_t requestData(uint8_t frameType, sensor_data_t &data);

  void getAll(sensor_data_t &data);
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

  void printData(const sensor_data_t &data);
  void printDataSingleLine(const sensor_data_t &data);

private:
  uint8_t _csPin;
  momentum_frame_t _frame;

  void sendMomentumFrame(const momentum_frame_t &frame);
};

#endif
