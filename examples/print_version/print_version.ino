/*******************************************************************************
 * @file print_version.ino
 * @brief Get the running Momentum dev board firmware version.
 *******************************************************************************
 */

#include <scalpelspace_momentum.h>

Momentum momentum(10);  // SPI CS pin 10 (out-of-the-box default).
// Default: Serial for printing and SPI (4 MHz, mode 0, MSB first).

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
