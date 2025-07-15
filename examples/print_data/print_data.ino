/*******************************************************************************
 * @file print_data.ino
 * @brief Momentum dev board sensor data print example.
 *
 * Application/full demos: https://github.com/scalpelspace/momentum_demo.
 *******************************************************************************
 */

#include <scalpelspace_momentum.h>

Momentum momentum(10);  // SPI CS pin 10.

sensor_data_t data;  // Create variable to track all data.


void setup() {
  Serial.begin(9600);  // Set baud rate.
  momentum.begin();    // Begin communication with Momentum.
}

void loop() {
  // Get all data.
  momentum.getAll(data);

  momentum.printData(data);  // Print data nicely.
  // Or alternatively print .csv style data in a single line:
  // momentum.printDataSingleLine(data); // Print data as a single line.

  delay(250);  // 250 millisecond delay between prints.
}
