/*******************************************************************************
 * @file get_version.ino
 * @brief Get the running Momentum dev board firmware version.
 *
 * Application/full demos: https://github.com/scalpelspace/momentum_demo.
 *******************************************************************************
 */

#include <scalpelspace_momentum.h>

Momentum momentum(10);  // SPI CS pin 10.

sensor_data_t data;  // Sensor data variable.

void setup() {
  Serial.begin(9600);  // Set baud rate.
  momentum.begin();    // Begin communication with Momentum.

  // Print version information.
  version_t version;               // Crate variable to hold firmware version.
  momentum.getVersion(version);    // Get version.
  momentum.printVersion(version);  // Print the Momentum firmware version.
}

void loop() {}
