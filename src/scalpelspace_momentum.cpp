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

void Momentum::sendMomentumFrame(const momentum_frame_t &frame) {
  // 1) Byte-pointer into the struct
  const uint8_t *txPtr = reinterpret_cast<const uint8_t *>(&frame);

  // 2) Total bytes = 4 (sof, type, seq, len) + payload length + 2 (crc)
  size_t txLen = 4 + frame.length + sizeof(frame.crc);

  // 3) Start transaction and lower CS
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);

  // 4) Send each byte
  for (size_t i = 0; i < txLen; ++i) {
    SPI.transfer(txPtr[i]);
  }

  // 5) Raise CS & end transaction
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

void Momentum::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  SPI.begin();
}

momentum_status_t Momentum::requestData(uint8_t frameType,
                                        sensor_data_t &data) {
  // Create a request frame
  momentum_frame_t request;
  memset(&request, 0, sizeof(request)); // Zero-initialize everything
  request.start_of_frame = MOMENTUM_START_OF_REQUEST_FRAME;
  request.frame_type = frameType;
  request.sequence = 0; // TODO: Not yet implemented
  request.length = 0;   // No payload
  build_crc(&request);

  // Send the request frame
  sendMomentumFrame(request);

  // TODO: Validate! Small pause to let the SPI peripheral device to load
  delayMicroseconds(50);

  // Clock in response
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);

  // Read response header
  uint8_t sof = SPI.transfer(0x00);
  uint8_t type = SPI.transfer(0x00);
  uint8_t seq = SPI.transfer(0x00);
  uint8_t len = SPI.transfer(0x00);

  // Validate basic response frame header
  if (sof != MOMENTUM_START_OF_RESPONSE_FRAME || type != frameType ||
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
  return parse_momentum_response_frame(&_frame, &data);
}

void Momentum::getAll(sensor_data_t &data) {
  requestData(MOMENTUM_FRAME_TYPE_IMU_QUAT, data);
  requestData(MOMENTUM_FRAME_TYPE_IMU_GYRO, data);
  requestData(MOMENTUM_FRAME_TYPE_IMU_ACCEL, data);
  requestData(MOMENTUM_FRAME_TYPE_IMU_LINACCEL, data);
  requestData(MOMENTUM_FRAME_TYPE_IMU_GRAV, data);
  requestData(MOMENTUM_FRAME_TYPE_BAR_ENV, data);
  requestData(MOMENTUM_FRAME_TYPE_GPS_DATETIME, data);
  requestData(MOMENTUM_FRAME_TYPE_GPS_COORD, data);
  requestData(MOMENTUM_FRAME_TYPE_GPS_ALT_SPEED, data);
  requestData(MOMENTUM_FRAME_TYPE_GPS_HEAD, data);
  requestData(MOMENTUM_FRAME_TYPE_GPS_STATS, data);
}

momentum_status_t Momentum::getQuat(sensor_data_t &data) {
  return requestData(MOMENTUM_FRAME_TYPE_IMU_QUAT, data);
}

momentum_status_t Momentum::getGyro(sensor_data_t &data) {
  return requestData(MOMENTUM_FRAME_TYPE_IMU_GYRO, data);
}

momentum_status_t Momentum::getAccel(sensor_data_t &data) {
  return requestData(MOMENTUM_FRAME_TYPE_IMU_ACCEL, data);
}

momentum_status_t Momentum::getLinAccel(sensor_data_t &data) {
  return requestData(MOMENTUM_FRAME_TYPE_IMU_LINACCEL, data);
}

momentum_status_t Momentum::getGrav(sensor_data_t &data) {
  return requestData(MOMENTUM_FRAME_TYPE_IMU_GRAV, data);
}

momentum_status_t Momentum::getEnv(sensor_data_t &data) {
  return requestData(MOMENTUM_FRAME_TYPE_BAR_ENV, data);
}

momentum_status_t Momentum::getDateTime(sensor_data_t &data) {
  return requestData(MOMENTUM_FRAME_TYPE_GPS_DATETIME, data);
}

momentum_status_t Momentum::getCoord(sensor_data_t &data) {
  return requestData(MOMENTUM_FRAME_TYPE_GPS_COORD, data);
}

momentum_status_t Momentum::getAltSpeed(sensor_data_t &data) {
  return requestData(MOMENTUM_FRAME_TYPE_GPS_ALT_SPEED, data);
}

momentum_status_t Momentum::getHeading(sensor_data_t &data) {
  return requestData(MOMENTUM_FRAME_TYPE_GPS_HEAD, data);
}

momentum_status_t Momentum::getStats(sensor_data_t &data) {
  return requestData(MOMENTUM_FRAME_TYPE_GPS_STATS, data);
}

void Momentum::printData(const sensor_data_t &d) {
  // -- IMU Quaternion ------------------------
  Serial.print("Quat: ");
  Serial.print(d.quaternion_i, 6);
  Serial.print(", ");
  Serial.print(d.quaternion_j, 6);
  Serial.print(", ");
  Serial.print(d.quaternion_k, 6);
  Serial.print(", ");
  Serial.println(d.quaternion_real, 6);

  Serial.print("QuatAcc (rad): ");
  Serial.print(d.quaternion_accuracy_rad, 6);
  Serial.print("  (deg): ");
  Serial.println(d.quaternion_accuracy_deg, 6);

  // -- IMU Gyroscope -------------------------
  Serial.print("Gyro: ");
  Serial.print(d.gyro_x, 6);
  Serial.print(", ");
  Serial.print(d.gyro_y, 6);
  Serial.print(", ");
  Serial.println(d.gyro_z, 6);

  // -- IMU Accelerometer ---------------------
  Serial.print("Accel: ");
  Serial.print(d.accel_x, 6);
  Serial.print(", ");
  Serial.print(d.accel_y, 6);
  Serial.print(", ");
  Serial.println(d.accel_z, 6);

  // -- IMU Linear Acceleration ---------------
  Serial.print("LinAcc: ");
  Serial.print(d.lin_accel_x, 6);
  Serial.print(", ");
  Serial.print(d.lin_accel_y, 6);
  Serial.print(", ");
  Serial.println(d.lin_accel_z, 6);

  // -- IMU Gravity Vector --------------------
  Serial.print("Grav: ");
  Serial.print(d.gravity_x, 6);
  Serial.print(", ");
  Serial.print(d.gravity_y, 6);
  Serial.print(", ");
  Serial.println(d.gravity_z, 6);

  // -- Barometer / Temperature ---------------
  Serial.print("Temp (deg C): ");
  Serial.print(d.temperature, 3);
  Serial.print("  Pressure (Pa): ");
  Serial.println(d.pressure, 3);

  // -- GPS Date & Time -----------------------
  Serial.print("GPS Date: ");
  // Note: year is stored as two-digit year; adjust if you want full YYYY
  Serial.print(d.year + 2000);
  Serial.print("/");
  Serial.print(d.month);
  Serial.print("/");
  Serial.print(d.day);
  Serial.print("  Time: ");
  Serial.print(d.hour);
  Serial.print(":");
  Serial.print(d.minute);
  Serial.print(":");
  Serial.println(d.second);

  // -- GPS Position --------------------------
  Serial.print("GPS Pos: ");
  Serial.print(d.latitude, 6);
  Serial.print(d.latitude_dir);
  Serial.print(", ");
  Serial.print(d.longitude, 6);
  Serial.print(d.longitude_dir);
  Serial.println();

  // -- GPS Altitude / Geoid Separation -------
  Serial.print("Alt (m): ");
  Serial.print(d.altitude, 3);
  Serial.print("  GeoidSep (m): ");
  Serial.println(d.geoid_sep, 3);

  // -- GPS Speed & Heading -------------------
  Serial.print("Speed (knots): ");
  Serial.print(d.ground_speed, 3);
  Serial.print("  Course (deg): ");
  Serial.print(d.ground_course, 3);
  Serial.print("  MagVar (deg): ");
  Serial.print(d.magnetic_var, 3);
  Serial.print(d.magnetic_var_dir);
  Serial.println();

  // -- GPS Fix / Satellites / HDOP -----------
  Serial.print("Fix Type: ");
  Serial.print(d.gps_position_fix); // renamed from gps_fix_quality
  Serial.print("  Sats: ");
  Serial.print(d.satellites);
  Serial.print("  HDOP: ");
  Serial.println(d.hdop, 3);

  Serial.println(); // blank line for readability
}

void Momentum::printDataSingleLine(const sensor_data_t &d) {
  // -- Quaternion (i, j, k, real)
  Serial.print(d.quaternion_i, 6);
  Serial.print(',');
  Serial.print(d.quaternion_j, 6);
  Serial.print(',');
  Serial.print(d.quaternion_k, 6);
  Serial.print(',');
  Serial.print(d.quaternion_real, 6);
  Serial.print(',');

  // -- Quaternion accuracy (rad, deg)
  Serial.print(d.quaternion_accuracy_rad, 6);
  Serial.print(',');
  Serial.print(d.quaternion_accuracy_deg, 6);
  Serial.print(',');

  // -- Gyro (x, y, z)
  Serial.print(d.gyro_x, 6);
  Serial.print(',');
  Serial.print(d.gyro_y, 6);
  Serial.print(',');
  Serial.print(d.gyro_z, 6);
  Serial.print(',');

  // -- Accel (x, y, z)
  Serial.print(d.accel_x, 6);
  Serial.print(',');
  Serial.print(d.accel_y, 6);
  Serial.print(',');
  Serial.print(d.accel_z, 6);
  Serial.print(',');

  // -- Linear Accel (x, y, z)
  Serial.print(d.lin_accel_x, 6);
  Serial.print(',');
  Serial.print(d.lin_accel_y, 6);
  Serial.print(',');
  Serial.print(d.lin_accel_z, 6);
  Serial.print(',');

  // -- Gravity (x, y, z)
  Serial.print(d.gravity_x, 6);
  Serial.print(',');
  Serial.print(d.gravity_y, 6);
  Serial.print(',');
  Serial.print(d.gravity_z, 6);
  Serial.print(',');

  // -- Temperature, Pressure
  Serial.print(d.temperature, 3);
  Serial.print(',');
  Serial.print(d.pressure, 3);
  Serial.print(',');

  // -- GPS Date (day, month, year-2000)
  Serial.print(d.day);
  Serial.print(',');
  Serial.print(d.month);
  Serial.print(',');
  Serial.print(d.year);
  Serial.print(',');

  // -- GPS Time (hour, minute, second)
  Serial.print(d.hour);
  Serial.print(',');
  Serial.print(d.minute);
  Serial.print(',');
  Serial.print(d.second);
  Serial.print(',');

  // -- GPS Lat (value, direction)
  Serial.print(d.latitude, 6);
  Serial.print(',');
  Serial.print(d.latitude_dir);
  Serial.print(',');

  // -- GPS Lon (value, direction)
  Serial.print(d.longitude, 6);
  Serial.print(',');
  Serial.print(d.longitude_dir);
  Serial.print(',');

  // -- GPS Altitude, GeoidSep
  Serial.print(d.altitude, 3);
  Serial.print(',');
  Serial.print(d.geoid_sep, 3);
  Serial.print(',');

  // -- GPS Speed, Course, Magnetic Variation, MagDir
  Serial.print(d.ground_speed, 3);
  Serial.print(',');
  Serial.print(d.ground_course, 3);
  Serial.print(',');
  Serial.print(d.magnetic_var, 3);
  Serial.print(',');
  Serial.print(d.magnetic_var_dir);

  // -- GPS Fix, Satellites, HDOP
  Serial.print(d.gps_position_fix);
  Serial.print(',');
  Serial.print(d.satellites);
  Serial.print(',');
  Serial.print(d.hdop, 3);
  Serial.print(',');

  Serial.println(); // end-of-line
}

momentum_status_t Momentum::setLED(uint8_t led_i, uint8_t r, uint8_t g,
                                   uint8_t b) {
  // Prepare LED payload
  led_data_t led_data = {led_i, r, g, b};

  // Build full frame
  momentum_frame_t command;
  memset(&command, 0, sizeof(command)); // Zero-initialize everything
  command.start_of_frame = MOMENTUM_START_OF_COMMAND_FRAME;
  command.frame_type = MOMENTUM_FRAME_TYPE_LED;
  command.sequence = 0;                   // TODO: Not yet implemented
  build_led_payload(&command, &led_data); // Length updated within
  build_crc(&command);

  // Send the command frame
  sendMomentumFrame(command);

  return MOMENTUM_OK;
}
