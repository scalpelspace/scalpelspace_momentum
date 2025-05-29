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
  void printData(const sensor_data_t &data);
  void printDataSingleLine(const sensor_data_t &data);

private:
  uint8_t _csPin;
  momentum_frame_t _frame;
};

#endif
