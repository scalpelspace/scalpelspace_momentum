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
  // -- IMU Quaternion ------------------------
  Serial.print("Quat: ");
  Serial.print(d.bno085_quaternion_i, 6);
  Serial.print(", ");
  Serial.print(d.bno085_quaternion_j, 6);
  Serial.print(", ");
  Serial.print(d.bno085_quaternion_k, 6);
  Serial.print(", ");
  Serial.println(d.bno085_quaternion_real, 6);

  Serial.print("QuatAcc (rad): ");
  Serial.print(d.bno085_quaternion_accuracy_rad, 6);
  Serial.print("  (deg): ");
  Serial.println(d.bno085_quaternion_accuracy_deg, 6);

  // -- IMU Gyroscope -------------------------
  Serial.print("Gyro: ");
  Serial.print(d.bno085_gyro_x, 6);
  Serial.print(", ");
  Serial.print(d.bno085_gyro_y, 6);
  Serial.print(", ");
  Serial.println(d.bno085_gyro_z, 6);

  // -- IMU Accelerometer ---------------------
  Serial.print("Accel: ");
  Serial.print(d.bno085_accel_x, 6);
  Serial.print(", ");
  Serial.print(d.bno085_accel_y, 6);
  Serial.print(", ");
  Serial.println(d.bno085_accel_z, 6);

  // -- IMU Linear Acceleration ---------------
  Serial.print("LinAcc: ");
  Serial.print(d.bno085_lin_accel_x, 6);
  Serial.print(", ");
  Serial.print(d.bno085_lin_accel_y, 6);
  Serial.print(", ");
  Serial.println(d.bno085_lin_accel_z, 6);

  // -- IMU Gravity Vector --------------------
  Serial.print("Grav: ");
  Serial.print(d.bno085_gravity_x, 6);
  Serial.print(", ");
  Serial.print(d.bno085_gravity_y, 6);
  Serial.print(", ");
  Serial.println(d.bno085_gravity_z, 6);

  // -- Barometer / Temperature ---------------
  Serial.print("Temp (deg C): ");
  Serial.print(d.bmp390_temperature, 3);
  Serial.print("  Pressure (Pa): ");
  Serial.println(d.bmp390_pressure, 3);

  // -- GPS Date & Time -----------------------
  Serial.print("GPS Date: ");
  // Note: gps_year is stored as two-digit year; adjust if you want full YYYY
  Serial.print(d.gps_year + 2000);
  Serial.print("/");
  Serial.print(d.gps_month);
  Serial.print("/");
  Serial.print(d.gps_day);
  Serial.print("  Time: ");
  Serial.print(d.gps_hour);
  Serial.print(":");
  Serial.print(d.gps_minute);
  Serial.print(":");
  Serial.println(d.gps_second);

  // -- GPS Position --------------------------
  Serial.print("GPS Pos: ");
  Serial.print(d.gps_latitude, 6);
  Serial.print(d.gps_lat_dir);
  Serial.print(", ");
  Serial.print(d.gps_longitude, 6);
  Serial.print(d.gps_lon_dir);
  Serial.println();

  // -- GPS Altitude / Geoid Separation -------
  Serial.print("Alt (m): ");
  Serial.print(d.gps_altitude_m, 3);
  Serial.print("  GeoidSep (m): ");
  Serial.println(d.gps_geoid_sep_m, 3);

  // -- GPS Speed & Heading -------------------
  Serial.print("Speed (knots): ");
  Serial.print(d.gps_speed_knots, 3);
  Serial.print("  Course (deg): ");
  Serial.print(d.gps_course_deg, 3);
  Serial.print("  MagVar (deg): ");
  Serial.print(d.gps_magnetic_deg, 3);
  Serial.print(d.gps_mag_dir);
  Serial.println();

  // -- GPS Fix / Satellites / HDOP -----------
  Serial.print("Fix Type: ");
  Serial.print(d.gps_position_fix); // renamed from gps_fix_quality
  Serial.print("  Sats: ");
  Serial.print(d.gps_satellites);
  Serial.print("  HDOP: ");
  Serial.println(d.gps_hdop, 3);

  Serial.println(); // blank line for readability
}

void Momentum::printDataSingleLine(const sensor_data_t &d) {
  // -- Quaternion (i, j, k, real)
  Serial.print(d.bno085_quaternion_i, 6);
  Serial.print(',');
  Serial.print(d.bno085_quaternion_j, 6);
  Serial.print(',');
  Serial.print(d.bno085_quaternion_k, 6);
  Serial.print(',');
  Serial.print(d.bno085_quaternion_real, 6);
  Serial.print(',');

  // -- Quaternion accuracy (rad, deg)
  Serial.print(d.bno085_quaternion_accuracy_rad, 6);
  Serial.print(',');
  Serial.print(d.bno085_quaternion_accuracy_deg, 6);
  Serial.print(',');

  // -- Gyro (x, y, z)
  Serial.print(d.bno085_gyro_x, 6);
  Serial.print(',');
  Serial.print(d.bno085_gyro_y, 6);
  Serial.print(',');
  Serial.print(d.bno085_gyro_z, 6);
  Serial.print(',');

  // -- Accel (x, y, z)
  Serial.print(d.bno085_accel_x, 6);
  Serial.print(',');
  Serial.print(d.bno085_accel_y, 6);
  Serial.print(',');
  Serial.print(d.bno085_accel_z, 6);
  Serial.print(',');

  // -- Linear Accel (x, y, z)
  Serial.print(d.bno085_lin_accel_x, 6);
  Serial.print(',');
  Serial.print(d.bno085_lin_accel_y, 6);
  Serial.print(',');
  Serial.print(d.bno085_lin_accel_z, 6);
  Serial.print(',');

  // -- Gravity (x, y, z)
  Serial.print(d.bno085_gravity_x, 6);
  Serial.print(',');
  Serial.print(d.bno085_gravity_y, 6);
  Serial.print(',');
  Serial.print(d.bno085_gravity_z, 6);
  Serial.print(',');

  // -- Temperature, Pressure
  Serial.print(d.bmp390_temperature, 3);
  Serial.print(',');
  Serial.print(d.bmp390_pressure, 3);
  Serial.print(',');

  // -- GPS Date (day, month, year-2000)
  Serial.print(d.gps_day);
  Serial.print(',');
  Serial.print(d.gps_month);
  Serial.print(',');
  Serial.print(d.gps_year);
  Serial.print(',');

  // -- GPS Time (hour, minute, second)
  Serial.print(d.gps_hour);
  Serial.print(',');
  Serial.print(d.gps_minute);
  Serial.print(',');
  Serial.print(d.gps_second);
  Serial.print(',');

  // -- GPS Lat (value, direction)
  Serial.print(d.gps_latitude, 6);
  Serial.print(',');
  Serial.print(d.gps_lat_dir);
  Serial.print(',');

  // -- GPS Lon (value, direction)
  Serial.print(d.gps_longitude, 6);
  Serial.print(',');
  Serial.print(d.gps_lon_dir);
  Serial.print(',');

  // -- GPS Altitude, GeoidSep
  Serial.print(d.gps_altitude_m, 3);
  Serial.print(',');
  Serial.print(d.gps_geoid_sep_m, 3);
  Serial.print(',');

  // -- GPS Speed, Course, Magnetic Variation, MagDir
  Serial.print(d.gps_speed_knots, 3);
  Serial.print(',');
  Serial.print(d.gps_course_deg, 3);
  Serial.print(',');
  Serial.print(d.gps_magnetic_deg, 3);
  Serial.print(',');
  Serial.print(d.gps_mag_dir);

  // -- GPS Fix, Satellites, HDOP
  Serial.print(d.gps_position_fix);
  Serial.print(',');
  Serial.print(d.gps_satellites);
  Serial.print(',');
  Serial.print(d.gps_hdop, 3);
  Serial.print(',');

  Serial.println(); // end-of-line
}
