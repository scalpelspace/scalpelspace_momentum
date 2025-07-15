/*******************************************************************************
 * @file scalpelspace_momentum.cpp
 * @brief Momentum sensor hub library for Arduino.
 *******************************************************************************
 */

#include "scalpelspace_momentum.h"

Momentum::Momentum(uint8_t csPin, Print &serial, SPIClass &spi,
                   const SPISettings &settings)
    : _csPin(csPin), _serial(&serial), _spi(&spi), _settings(settings),
      _sequence(0) {}

void Momentum::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  _spi->begin();
}

void Momentum::setSerialOutput(Print &serial) { _serial = &serial; }

void Momentum::sendMomentumFrame(const momentum_frame_t &frame) {
  // 1) Byte-pointer into the struct.
  const uint8_t *txPtr = reinterpret_cast<const uint8_t *>(&frame);

  // 2) Total bytes = 4 (sof, type, seq, len) + payload length + 2 (crc).
  size_t txLen = 4 + frame.length + sizeof(frame.crc);

  // 3) Start transaction and lower CS.
  _spi->beginTransaction(_settings);
  digitalWrite(_csPin, LOW);

  // 4) Send each byte.
  for (size_t i = 0; i < txLen; ++i) {
    _spi->transfer(txPtr[i]);
  }

  // 5) Raise CS & end transaction.
  digitalWrite(_csPin, HIGH);
  _spi->endTransaction();
}

momentum_status_t Momentum::requestData(uint8_t frameType, sensor_data_t &data,
                                        version_t &version) {
  // Create a request frame.
  momentum_frame_t request;
  memset(&request, 0, sizeof(request)); // Zero-initialize everything.
  request.start_of_frame = MOMENTUM_START_OF_REQUEST_FRAME;
  request.frame_type = frameType;
  request.sequence = _sequence++;
  request.length = 0; // No payload.
  build_crc(&request);

  // Send the request frame.
  sendMomentumFrame(request);

  // TODO: Validate! Small pause to let the SPI peripheral device to load.
  delayMicroseconds(50);

  // Clock in response.
  _spi->beginTransaction(_settings);
  digitalWrite(_csPin, LOW);

  // Read response header.
  uint8_t sof = _spi->transfer(0x00);
  uint8_t type = _spi->transfer(0x00);
  uint8_t seq = _spi->transfer(0x00);
  uint8_t len = _spi->transfer(0x00);

  // Validate basic response frame header.
  if (sof != MOMENTUM_START_OF_RESPONSE_FRAME || type != frameType ||
      len > MOMENTUM_MAX_DATA_SIZE) {
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    return MOMENTUM_ERROR_BAD_FRAME;
  }

  // Fill struct.
  _frame.start_of_frame = sof;
  _frame.frame_type = type;
  _frame.sequence = seq;
  _frame.length = len;

  // Read payload.
  for (uint8_t i = 0; i < len; ++i) {
    _frame.payload[i] = _spi->transfer(0x00);
  }

  // Read CRC.
  uint8_t lo = _spi->transfer(0x00);
  uint8_t hi = _spi->transfer(0x00);

  digitalWrite(_csPin, HIGH);
  _spi->endTransaction();

  // Store CRC and parse.
  _frame.crc = (uint16_t)lo | ((uint16_t)hi << 8);
  return parse_momentum_response_frame(&_frame, &data, &version);
}

momentum_status_t Momentum::getVersion(version_t &version) {
  sensor_data_t dummy;
  return requestData(MOMENTUM_FRAME_TYPE_VERSION, dummy, version);
}

void Momentum::getAll(sensor_data_t &data) {
  version_t dummy;
  requestData(MOMENTUM_FRAME_TYPE_IMU_QUAT, data, dummy);
  requestData(MOMENTUM_FRAME_TYPE_IMU_GYRO, data, dummy);
  requestData(MOMENTUM_FRAME_TYPE_IMU_ACCEL, data, dummy);
  requestData(MOMENTUM_FRAME_TYPE_IMU_LINACCEL, data, dummy);
  requestData(MOMENTUM_FRAME_TYPE_IMU_GRAV, data, dummy);
  requestData(MOMENTUM_FRAME_TYPE_BAR_ENV, data, dummy);
  requestData(MOMENTUM_FRAME_TYPE_GPS_DATETIME, data, dummy);
  requestData(MOMENTUM_FRAME_TYPE_GPS_COORD, data, dummy);
  requestData(MOMENTUM_FRAME_TYPE_GPS_ALT_SPEED, data, dummy);
  requestData(MOMENTUM_FRAME_TYPE_GPS_HEAD, data, dummy);
  requestData(MOMENTUM_FRAME_TYPE_GPS_STATS, data, dummy);
}

momentum_status_t Momentum::getQuat(sensor_data_t &data) {
  version_t dummy;
  return requestData(MOMENTUM_FRAME_TYPE_IMU_QUAT, data, dummy);
}

momentum_status_t Momentum::getGyro(sensor_data_t &data) {
  version_t dummy;
  return requestData(MOMENTUM_FRAME_TYPE_IMU_GYRO, data, dummy);
}

momentum_status_t Momentum::getAccel(sensor_data_t &data) {
  version_t dummy;
  return requestData(MOMENTUM_FRAME_TYPE_IMU_ACCEL, data, dummy);
}

momentum_status_t Momentum::getLinAccel(sensor_data_t &data) {
  version_t dummy;
  return requestData(MOMENTUM_FRAME_TYPE_IMU_LINACCEL, data, dummy);
}

momentum_status_t Momentum::getGrav(sensor_data_t &data) {
  version_t dummy;
  return requestData(MOMENTUM_FRAME_TYPE_IMU_GRAV, data, dummy);
}

momentum_status_t Momentum::getEnv(sensor_data_t &data) {
  version_t dummy;
  return requestData(MOMENTUM_FRAME_TYPE_BAR_ENV, data, dummy);
}

momentum_status_t Momentum::getDateTime(sensor_data_t &data) {
  version_t dummy;
  return requestData(MOMENTUM_FRAME_TYPE_GPS_DATETIME, data, dummy);
}

momentum_status_t Momentum::getCoord(sensor_data_t &data) {
  version_t dummy;
  return requestData(MOMENTUM_FRAME_TYPE_GPS_COORD, data, dummy);
}

momentum_status_t Momentum::getAltSpeed(sensor_data_t &data) {
  version_t dummy;
  return requestData(MOMENTUM_FRAME_TYPE_GPS_ALT_SPEED, data, dummy);
}

momentum_status_t Momentum::getHeading(sensor_data_t &data) {
  version_t dummy;
  return requestData(MOMENTUM_FRAME_TYPE_GPS_HEAD, data, dummy);
}

momentum_status_t Momentum::getStats(sensor_data_t &data) {
  version_t dummy;
  return requestData(MOMENTUM_FRAME_TYPE_GPS_STATS, data, dummy);
}

momentum_status_t Momentum::reset(void) {
  // Build full frame.
  momentum_frame_t command;
  memset(&command, 0, sizeof(command)); // Zero-initialize everything.
  command.start_of_frame = MOMENTUM_START_OF_COMMAND_FRAME;
  command.frame_type = MOMENTUM_FRAME_TYPE_RESET;
  command.sequence = _sequence++;
  command.length = 0;
  build_crc(&command);

  // Send the command frame.
  sendMomentumFrame(command);

  return MOMENTUM_OK;
}

momentum_status_t Momentum::setLED(uint8_t led_i, uint8_t r, uint8_t g,
                                   uint8_t b) {
  // Prepare LED payload.
  led_data_t led_data = {led_i, r, g, b};

  // Build full frame.
  momentum_frame_t command;
  memset(&command, 0, sizeof(command)); // Zero-initialize everything.
  command.start_of_frame = MOMENTUM_START_OF_COMMAND_FRAME;
  command.frame_type = MOMENTUM_FRAME_TYPE_LED;
  command.sequence = _sequence++;
  build_led_payload(&command, &led_data); // Length updated within.
  build_crc(&command);

  // Send the command frame.
  sendMomentumFrame(command);

  return MOMENTUM_OK;
}

void Momentum::printVersion(const version_t &v) {
  // MAJOR.MINOR.PATCH-IDENTIFIER.
  _serial->print(v.major);
  _serial->print(".");
  _serial->print(v.minor);
  _serial->print(".");
  _serial->print(v.patch);
  _serial->print("-");
  _serial->println(v.identifier);
}

void Momentum::printData(const sensor_data_t &d) {
  // -- IMU quaternion.
  _serial->print("Quat: ");
  _serial->print(d.quaternion_i, 6);
  _serial->print(", ");
  _serial->print(d.quaternion_j, 6);
  _serial->print(", ");
  _serial->print(d.quaternion_k, 6);
  _serial->print(", ");
  _serial->println(d.quaternion_real, 6);

  _serial->print("QuatAcc (rad): ");
  _serial->print(d.quaternion_accuracy_rad, 6);
  _serial->print("  (deg): ");
  _serial->println(d.quaternion_accuracy_deg, 6);

  // -- IMU gyroscope.
  _serial->print("Gyro: ");
  _serial->print(d.gyro_x, 6);
  _serial->print(", ");
  _serial->print(d.gyro_y, 6);
  _serial->print(", ");
  _serial->println(d.gyro_z, 6);

  // -- IMU accelerometer.
  _serial->print("Accel: ");
  _serial->print(d.accel_x, 6);
  _serial->print(", ");
  _serial->print(d.accel_y, 6);
  _serial->print(", ");
  _serial->println(d.accel_z, 6);

  // -- IMU linear acceleration.
  _serial->print("LinAcc: ");
  _serial->print(d.lin_accel_x, 6);
  _serial->print(", ");
  _serial->print(d.lin_accel_y, 6);
  _serial->print(", ");
  _serial->println(d.lin_accel_z, 6);

  // -- IMU gravity vector.
  _serial->print("Grav: ");
  _serial->print(d.gravity_x, 6);
  _serial->print(", ");
  _serial->print(d.gravity_y, 6);
  _serial->print(", ");
  _serial->println(d.gravity_z, 6);

  // -- Barometer/temperature.
  _serial->print("Temp (deg C): ");
  _serial->print(d.temperature, 3);
  _serial->print("  Pressure (Pa): ");
  _serial->println(d.pressure, 3);

  // -- GNSS date & time.
  _serial->print("GNSS Date: ");
  // Note: year is stored as two-digit year; adjust if you want full YYYY.
  _serial->print(d.year + 2000);
  _serial->print("/");
  _serial->print(d.month);
  _serial->print("/");
  _serial->print(d.day);
  _serial->print("  Time: ");
  _serial->print(d.hour);
  _serial->print(":");
  _serial->print(d.minute);
  _serial->print(":");
  _serial->println(d.second);

  // -- GNSS position.
  _serial->print("GNSS Pos: ");
  _serial->print(d.latitude, 6);
  _serial->print(" (");
  _serial->print(d.latitude_dir);
  _serial->print(")");
  _serial->print(", ");
  _serial->print(d.longitude, 6);
  _serial->print(" (");
  _serial->print(d.longitude_dir);
  _serial->print(")");
  _serial->println();

  // -- GNSS altitude/geoid separation.
  _serial->print("Alt (m): ");
  _serial->print(d.altitude, 3);
  _serial->print("  GeoidSep (m): ");
  _serial->println(d.geoid_sep, 3);

  // -- GNSS speed & heading.
  _serial->print("Speed (knots): ");
  _serial->print(d.ground_speed, 3);
  _serial->print("  Course (deg): ");
  _serial->print(d.ground_course, 3);
  _serial->print("  MagVar (deg): ");
  _serial->print(d.magnetic_var, 3);
  _serial->print(d.magnetic_var_dir);
  _serial->println();

  // -- GNSS fix/satellites/HDOP.
  _serial->print("Fix Type: ");
  _serial->print(d.gps_position_fix);
  _serial->print("  Sats: ");
  _serial->print(d.satellites);
  _serial->print("  HDOP: ");
  _serial->println(d.hdop, 3);

  _serial->println();
}

void Momentum::printDataSingleLine(const sensor_data_t &d) {
  // -- Quaternion (i, j, k, real).
  _serial->print(d.quaternion_i, 6);
  _serial->print(',');
  _serial->print(d.quaternion_j, 6);
  _serial->print(',');
  _serial->print(d.quaternion_k, 6);
  _serial->print(',');
  _serial->print(d.quaternion_real, 6);
  _serial->print(',');

  // -- Quaternion accuracy (rad, deg).
  _serial->print(d.quaternion_accuracy_rad, 6);
  _serial->print(',');
  _serial->print(d.quaternion_accuracy_deg, 6);
  _serial->print(',');

  // -- Gyro (x, y, z).
  _serial->print(d.gyro_x, 6);
  _serial->print(',');
  _serial->print(d.gyro_y, 6);
  _serial->print(',');
  _serial->print(d.gyro_z, 6);
  _serial->print(',');

  // -- Accel (x, y, z).
  _serial->print(d.accel_x, 6);
  _serial->print(',');
  _serial->print(d.accel_y, 6);
  _serial->print(',');
  _serial->print(d.accel_z, 6);
  _serial->print(',');

  // -- Linear Accel (x, y, z).
  _serial->print(d.lin_accel_x, 6);
  _serial->print(',');
  _serial->print(d.lin_accel_y, 6);
  _serial->print(',');
  _serial->print(d.lin_accel_z, 6);
  _serial->print(',');

  // -- Gravity (x, y, z).
  _serial->print(d.gravity_x, 6);
  _serial->print(',');
  _serial->print(d.gravity_y, 6);
  _serial->print(',');
  _serial->print(d.gravity_z, 6);
  _serial->print(',');

  // -- Temperature, Pressure.
  _serial->print(d.temperature, 3);
  _serial->print(',');
  _serial->print(d.pressure, 3);
  _serial->print(',');

  // -- GNSS Date (day, month, year-2000).
  _serial->print(d.day);
  _serial->print(',');
  _serial->print(d.month);
  _serial->print(',');
  _serial->print(d.year);
  _serial->print(',');

  // -- GNSS Time (hour, minute, second).
  _serial->print(d.hour);
  _serial->print(',');
  _serial->print(d.minute);
  _serial->print(',');
  _serial->print(d.second);
  _serial->print(',');

  // -- GNSS Lat (value, direction).
  _serial->print(d.latitude, 6);
  _serial->print(',');
  _serial->print(d.latitude_dir);
  _serial->print(',');

  // -- GNSS Lon (value, direction).
  _serial->print(d.longitude, 6);
  _serial->print(',');
  _serial->print(d.longitude_dir);
  _serial->print(',');

  // -- GNSS altitude, geoid separation.
  _serial->print(d.altitude, 3);
  _serial->print(',');
  _serial->print(d.geoid_sep, 3);
  _serial->print(',');

  // -- GNSS speed, course, magnetic variation, magnetic variation direction.
  _serial->print(d.ground_speed, 3);
  _serial->print(',');
  _serial->print(d.ground_course, 3);
  _serial->print(',');
  _serial->print(d.magnetic_var, 3);
  _serial->print(',');
  _serial->print(d.magnetic_var_dir);

  // -- GNSS fix, satellites, HDOP.
  _serial->print(d.gps_position_fix);
  _serial->print(',');
  _serial->print(d.satellites);
  _serial->print(',');
  _serial->print(d.hdop, 3);
  _serial->print(',');

  _serial->println();
}
