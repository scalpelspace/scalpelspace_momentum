/*******************************************************************************
 * @file scalpelspace_momentum.cpp
 * @brief Momentum sensor hub library for Arduino.
 *******************************************************************************
 */

#include "scalpelspace_momentum.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "momentum_driver.h"

#ifdef __cplusplus
}
#endif

Momentum::Momentum(uint8_t csPin) : _csPin(csPin) {}

void Momentum::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  SPI.begin();
}

momentum_status_t Momentum::requestData(uint8_t frameType,
                                        sensor_data_t &data) {
  // PHASE 1: Send command
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(frameType);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  // TODO: Validate! Small pause to let the SPI peripheral device to load
  delayMicroseconds(50);

  // PHASE 2: Read response
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);

  // Read header
  uint8_t sof = SPI.transfer(0x00);
  uint8_t type = SPI.transfer(0x00);
  uint8_t seq = SPI.transfer(0x00);
  uint8_t len = SPI.transfer(0x00);

  // Validate basic frame
  if (sof != MOMENTUM_START_OF_FRAME || type != frameType ||
      len > MOMENTUM_MAX_DATA_SIZE) {
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    return MOMENTUM_ERROR_BAD_FRAME;
  }

  // Fill struct
  _frame.start_of_frame = sof;
  _frame.frame_type = type;
  _frame.sequence = seq;
  _frame.length = len;

  // Read payload
  for (uint8_t i = 0; i < len; ++i) {
    _frame.payload[i] = SPI.transfer(0x00);
  }

  // Read CRC
  uint8_t lo = SPI.transfer(0x00);
  uint8_t hi = SPI.transfer(0x00);

  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  // Store CRC and parse
  _frame.crc = (uint16_t)lo | ((uint16_t)hi << 8);
  return parse_momentum_frame(&_frame, &data);
}

void Momentum::printData(const sensor_data_t &d) {
  Serial.print("Quat: ");
  Serial.print(d.bno085_quaternion_i, 6);
  Serial.print(',');
  Serial.print(d.bno085_quaternion_j, 6);
  Serial.print(',');
  Serial.print(d.bno085_quaternion_k, 6);
  Serial.print(',');
  Serial.println(d.bno085_quaternion_real, 6);

  Serial.print("Gyro: ");
  Serial.print(d.bno085_gyro_x, 6);
  Serial.print(',');
  Serial.print(d.bno085_gyro_y, 6);
  Serial.print(',');
  Serial.println(d.bno085_gyro_z, 6);

  Serial.print("Accel: ");
  Serial.print(d.bno085_accel_x, 6);
  Serial.print(',');
  Serial.print(d.bno085_accel_y, 6);
  Serial.print(',');
  Serial.println(d.bno085_accel_z, 6);

  Serial.print("LinAcc: ");
  Serial.print(d.bno085_lin_accel_x, 6);
  Serial.print(',');
  Serial.print(d.bno085_lin_accel_y, 6);
  Serial.print(',');
  Serial.println(d.bno085_lin_accel_z, 6);

  Serial.print("Grav: ");
  Serial.print(d.bno085_gravity_x, 6);
  Serial.print(',');
  Serial.print(d.bno085_gravity_y, 6);
  Serial.print(',');
  Serial.println(d.bno085_gravity_z, 6);

  Serial.print("Temp: ");
  Serial.print(d.bmp390_temperature, 3);
  Serial.print(" Pressure: ");
  Serial.println(d.bmp390_pressure, 3);

  Serial.print("GPS Dt: ");
  Serial.print(d.gps_hour);
  Serial.print(':');
  Serial.print(d.gps_minute);
  Serial.print(':');
  Serial.println(d.gps_second);

  Serial.print("GPS Pos: ");
  Serial.print(d.gps_latitude, 6);
  Serial.print(d.gps_lat_dir);
  Serial.print(", ");
  Serial.print(d.gps_longitude, 6);
  Serial.print(d.gps_lon_dir);
  Serial.println();

  Serial.print("Fix: ");
  Serial.print(d.gps_fix_quality);
  Serial.print(" Sats: ");
  Serial.print(d.gps_satellites);
  Serial.print(" HDOP: ");
  Serial.println(d.gps_hdop);

  Serial.print("Alt: ");
  Serial.print(d.gps_altitude, 3);
  Serial.print(" Geoid: ");
  Serial.println(d.gps_geoid_sep);
  Serial.println();
}

void Momentum::printDataSingleLine(const sensor_data_t &d) {
  Serial.print(d.bno085_quaternion_i, 6);
  Serial.print(',');
  Serial.print(d.bno085_quaternion_j, 6);
  Serial.print(',');
  Serial.print(d.bno085_quaternion_k, 6);
  Serial.print(',');
  Serial.print(d.bno085_quaternion_real, 6);
  Serial.print(',');
  Serial.print(d.bno085_gyro_x, 6);
  Serial.print(',');
  Serial.print(d.bno085_gyro_y, 6);
  Serial.print(',');
  Serial.print(d.bno085_gyro_z, 6);
  Serial.print(',');
  Serial.print(d.bno085_accel_x, 6);
  Serial.print(',');
  Serial.print(d.bno085_accel_y, 6);
  Serial.print(',');
  Serial.print(d.bno085_accel_z, 6);
  Serial.print(',');
  Serial.print(d.bno085_lin_accel_x, 6);
  Serial.print(',');
  Serial.print(d.bno085_lin_accel_y, 6);
  Serial.print(',');
  Serial.print(d.bno085_lin_accel_z, 6);
  Serial.print(',');
  Serial.print(d.bno085_gravity_x, 6);
  Serial.print(',');
  Serial.print(d.bno085_gravity_y, 6);
  Serial.print(',');
  Serial.print(d.bno085_gravity_z, 6);
  Serial.print(',');
  Serial.print(d.bmp390_temperature, 3);
  Serial.print(',');
  Serial.print(d.bmp390_pressure, 3);
  Serial.print(',');
  Serial.print(d.gps_hour);
  Serial.print(',');
  Serial.print(d.gps_minute);
  Serial.print(',');
  Serial.print(d.gps_second);
  Serial.print(',');
  Serial.print(d.gps_latitude, 6);
  Serial.print(',');
  Serial.print(d.gps_lat_dir);
  Serial.print(',');
  Serial.print(d.gps_longitude, 6);
  Serial.print(',');
  Serial.print(d.gps_lon_dir);
  Serial.print(',');
  Serial.print(d.gps_fix_quality);
  Serial.print(',');
  Serial.print(d.gps_satellites);
  Serial.print(',');
  Serial.print(d.gps_hdop);
  Serial.print(',');
  Serial.print(d.gps_altitude, 3);
  Serial.print(',');
  Serial.println(d.gps_geoid_sep);
}
