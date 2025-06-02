/*******************************************************************************
 * @file momentum_driver.h
 * @brief Momentum driver for SPI based communication.
 *******************************************************************************
 */

#ifndef MOMENTUM__DRIVER_H
#define MOMENTUM__DRIVER_H

/** Includes. *****************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/** CPP guard open. ***********************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/** Definitions. **************************************************************/

#define MOMENTUM_MAX_DATA_SIZE 32

#define MOMENTUM_START_OF_FRAME 0xAA

#define MOMENTUM_FRAME_TYPE_IMU_QUAT 0x21
#define MOMENTUM_FRAME_TYPE_IMU_GYRO 0x22
#define MOMENTUM_FRAME_TYPE_IMU_ACCEL 0x23
#define MOMENTUM_FRAME_TYPE_IMU_LINACCEL 0x24
#define MOMENTUM_FRAME_TYPE_IMU_GRAV 0x25
#define MOMENTUM_FRAME_TYPE_BAR_ENV 0x26
#define MOMENTUM_FRAME_TYPE_GPS_DATETIME 0x27
#define MOMENTUM_FRAME_TYPE_GPS_COORD 0x28
#define MOMENTUM_FRAME_TYPE_GPS_ALT_SPEED 0x29
#define MOMENTUM_FRAME_TYPE_GPS_HEAD 0x2A
#define MOMENTUM_FRAME_TYPE_GPS_STATS 0x2B

#define MOMENTUM_CRC_INITIAL 0xFFFF

/** Public types. *************************************************************/

/**
 * @brief Momentum sensor hub SPI communication frame.
 */
typedef struct __attribute__((packed)) {
  uint8_t start_of_frame;                  // Start of frame (SOF) for syncing.
  uint8_t frame_type;                      // Frame type identifier.
  uint8_t sequence;                        // Roll-over counter.
  uint8_t length;                          // Number of data payload bytes.
  uint8_t payload[MOMENTUM_MAX_DATA_SIZE]; // Data payload.
  uint16_t crc;                            // CRC-16 of frame_type...data[-1].
} momentum_frame_t;

/**
 * @brief Momentum sensor hub SPI communication frame processing status.
 */
typedef enum {
  MOMENTUM_OK = 0,
  MOMENTUM_ERROR_CRC = 1,        // Invalid CRC.
  MOMENTUM_ERROR_FRAME_TYPE = 2, // Invalid frame type.
  MOMENTUM_ERROR_BAD_FRAME = 3,  // Invalid start of frame (SOF) or length.
} momentum_status_t;

/**
 * @brief Momentum sensor hub sensor data.
 */
typedef struct {
  float bno085_quaternion_i;
  float bno085_quaternion_j;
  float bno085_quaternion_k;
  float bno085_quaternion_real;
  float bno085_quaternion_accuracy_rad;
  float bno085_quaternion_accuracy_deg;
  float bno085_gyro_x;
  float bno085_gyro_y;
  float bno085_gyro_z;
  float bno085_accel_x;
  float bno085_accel_y;
  float bno085_accel_z;
  float bno085_lin_accel_x;
  float bno085_lin_accel_y;
  float bno085_lin_accel_z;
  float bno085_gravity_x;
  float bno085_gravity_y;
  float bno085_gravity_z;
  float bmp390_temperature;
  float bmp390_pressure;
  uint8_t gps_position_fix;
  uint8_t gps_year;       // RTC date, year.
  uint8_t gps_month;      // RTC date, month.
  uint8_t gps_day;        // RTC date, day.
  uint8_t gps_hour;       // RTC time, hour.
  uint8_t gps_minute;     // RTC time, minute.
  uint8_t gps_second;     // RTC time, second.
  float gps_latitude;     // Latitude in decimal degrees.
  char gps_lat_dir;       // Latitude Direction (N/S).
  float gps_longitude;    // Longitude in decimal degrees.
  char gps_lon_dir;       // Longitude Direction (E/W).
  float gps_altitude_m;   // Altitude in meters.
  float gps_geoid_sep_m;  // Geoidal Separation.
  float gps_speed_knots;  // Speed over the ground in knots.
  float gps_course_deg;   // Course over ground in degrees.
  float gps_magnetic_deg; // Magnetic variation in degrees.
  char gps_mag_dir;       // Magnetic variation direction (E/W).
  uint8_t gps_satellites; // Number of Satellites.
  float gps_hdop;         // Horizontal Dilution of Precision (HDOP).
} sensor_data_t;

/** Public functions. *********************************************************/

/**
 * @brief Compute CRC-16-CCITT over a byte buffer.
 *
 * @param crc Initial CRC value (usually 0xFFFF).
 * @param buf Pointer to the data bytes to checksum.
 * @param len Number of bytes in buf.
 *
 * @return The updated CRC16.
 */
uint16_t crc16_ccitt(uint16_t crc, const uint8_t *buf, size_t len);

/**
 * @brief Build and append CRC to a momentum_frame_t.
 *
 * Calculates the CRC-16-CCITT over the frame header and payload, then writes
 * the resulting 16-bit CRC into f->crc.
 *
 * @param f Pointer to the frame whose CRC field will be updated.
 */
void build_crc(momentum_frame_t *f);

/**
 * @brief Verify the CRC of a received frame.
 *
 * Recomputes CRC-16-CCITT over the received frame_type, sequence, length,
 * and payload bytes, then compares against the 16-bit CRC field in the frame.
 *
 * @param f Pointer to the received frame.
 *
 * @return true if the computed CRC matches f->crc, false otherwise.
 */
bool verify_crc(const momentum_frame_t *f);

/**
 * @brief Pack quaternion data into the frame payload.
 *
 * Serializes quaternion components (i, j, k, real) and quaternion accuracy (in
 * radians and degrees) from the provided sensor data into the frame payload.
 * Updates f->length to reflect the number of bytes written.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source quaternion values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_quaternion_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack quaternion data from the frame payload.
 *
 * Deserializes quaternion components (i, j, k, real) and quaternion accuracy
 * (in radians and degrees) from the provided frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update quaternion values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_quaternion_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack gyroscope data into the frame payload.
 *
 * Serializes gyroscope readings (x, y, z) from the provided sensor data into
 * the frame payload. Updates f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source gyroscope values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gyro_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack gyroscope data from the frame payload.
 *
 * Deserializes gyroscope readings (x, y, z) from the provided frame payload
 * into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update gyroscope values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gyro_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack accelerometer data into the frame payload.
 *
 * Serializes accelerometer readings (x, y, z) from the provided sensor data
 * into the frame payload. Updates f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source accelerometer values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_accel_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack accelerometer data from the frame payload.
 *
 * Deserializes accelerometer readings (x, y, z) from the provided frame payload
 * into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update accelerometer values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_accel_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack linear acceleration data into the frame payload.
 *
 * Serializes linear acceleration readings (x, y, z) from the provided sensor
 * data into the frame payload. Updates f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source linear acceleration
 *          values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_lin_accel_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack linear acceleration data from the frame payload.
 *
 * Deserializes linear acceleration readings (x, y, z) from the provided frame
 * payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update linear acceleration values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_lin_accel_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack gravity vector data into the frame payload.
 *
 * Serializes gravity vector components (x, y, z) from the provided sensor data
 * into the frame payload. Updates f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source gravity values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gravity_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack gravity vector data from the frame payload.
 *
 * Deserializes gravity vector components (x, y, z) from the provided frame
 * payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update gravity values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gravity_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack barometer temperature and pressure data into the frame payload.
 *
 * Serializes barometric temperature and pressure readings from the provided
 * sensor data into the frame payload. Updates f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source temperature and
 *          pressure values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_pressure_temp_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack barometer temperature and pressure data from the frame payload.
 *
 * Deserializes barometric temperature and pressure readings from the provided
 * frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update temperature and pressure
 *          values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_pressure_temp_payload(const momentum_frame_t *f,
                                    sensor_data_t *s);

/**
 * @brief Pack GPS date and time data into the frame payload.
 *
 * Serializes GPS date and time fields (hour, minute, second, day, month, year)
 * from the provided sensor data into the frame payload. Updates f->length
 * accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source GPS date/time values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gps_datetime_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack GPS date and time data from the frame payload.
 *
 * Deserializes GPS date and time fields (hour, minute, second, day, month,
 * year) from the provided frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update GPS date/time values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gps_datetime_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack GPS coordinate data into the frame payload.
 *
 * Serializes GPS position fields (latitude, latitude direction, longitude,
 * longitude direction) from the provided sensor data into the frame payload.
 * Updates f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source GPS coordinates.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gps_coord_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack GPS coordinate data from the frame payload.
 *
 * Deserializes GPS position fields (latitude, latitude direction, longitude,
 * longitude direction) from the provided frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update GPS coordinates.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gps_coord_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack GPS altitude and speed data into the frame payload.
 *
 * Serializes GPS altitude fields (altitude, geoidal separation) and speed
 * (speed knots) from the provided sensor data into the frame payload. Updates
 * f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source GPS altitude and
 *          speed values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gps_altitude_speed_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack GPS altitude and speed data from the frame payload.
 *
 * Deserializes GPS altitude fields (altitude, geoidal separation) and speed
 * (speed knots) from the provided frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update GPS altitude and speed
 *          values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gps_altitude_speed_payload(const momentum_frame_t *f,
                                         sensor_data_t *s);

/**
 * @brief Pack GPS heading data into the frame payload.
 *
 * Serializes GPS heading fields (course, magnetic variation, magnetic variation
 * direction) from the provided sensor data into the frame payload. Updates
 * f->length accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source GPS heading values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gps_heading_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack GPS heading data from the frame payload.
 *
 * Deserializes GPS heading fields (course, magnetic variation, magnetic
 * variation direction) from the provided frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update GPS heading values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gps_heading_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Pack GPS status and statistics into the frame payload.
 *
 * Serializes GPS status fields (position fix type, number of satellites, HDOP)
 * from the provided sensor data into the frame payload. Updates f->length
 * accordingly.
 *
 * @param f Pointer to the frame to populate.
 * @param s Pointer to the sensor_data_t containing source GPS status values.
 *
 * @return Number of bytes written into f->payload.
 */
uint8_t build_gps_stats_payload(momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Unpack GPS status and statistics from the frame payload.
 *
 * Deserializes GPS status fields (position fix type, number of satellites,
 * HDOP) from the provided frame payload into sensor data.
 *
 * @param f Pointer to the frame to read.
 * @param s Pointer to the sensor_data_t to update GPS status values.
 *
 * @return Number of bytes read (== f->length).
 */
uint8_t parse_gps_stats_payload(const momentum_frame_t *f, sensor_data_t *s);

/**
 * @brief Top-level parser for any incoming momentum_frame_t.
 *
 * @param f Pointer to received, packed frame.
 * @param s Dest sensor_data_t to populate on success.
 *
 * @return  MOMENTUM_OK on success, otherwise an error code:
 *          - MOMENTUM_ERROR_CRC if CRC mismatch.
 *          - MOMENTUM_ERROR_FRAME_TYPE if unknown frame_type.
 *          - MOMENTUM_ERROR_BAD_FRAME if SOF/length bad.
 */
momentum_status_t parse_momentum_frame(const momentum_frame_t *f,
                                       sensor_data_t *s);

/** CPP guard close. **********************************************************/

#ifdef __cplusplus
}
#endif

#endif
