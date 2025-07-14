/*******************************************************************************
 * @file print_data.ino
 * @brief Momentum dev board sensor data print example.
 *
 * Application/full demos: https://github.com/scalpelspace/momentum_demo.
 *******************************************************************************
 */

#include <scalpelspace_momentum.h>

Momentum momentum(10);  // SPI CS pin 10.

void setup() {
  Serial.begin(9600);  // Set baud rate.
  momentum.begin();      // Begin communication with Momentum.

  // Print version information.
  version_t version;               // Crate variable to hold firmware version.
  momentum.getVersion(version);    // Get version.
  momentum.printVersion(version);  // Print the Momentum firmware version.

  delay(2000);  // 2 second delay.
}

void loop() {
  // Create variable to track all data.
  sensor_data_t data;

  // Get all data.
  momentum.getAll(data);

  // momentum.printDataSingleLine(data); // Print data as a single line.
  momentum.printData(data);  // Print data nicely.

  delay(250);  // 250 millisecond delay between prints.
}
