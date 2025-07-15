/*******************************************************************************
 * @file basic.ino
 * @brief Basic sensor data access and print example.
 *
 * Application/full demos: https://github.com/scalpelspace/momentum_demo.
 *******************************************************************************
 */

#include <scalpelspace_momentum.h>

Momentum momentum(10);  // SPI CS pin 10 (out-of-the-box default).
// Default: Serial for printing and SPI (4 MHz, mode 0, MSB first).

sensor_data_t data;  // Sensor data variable.

void setup() {
  Serial.begin(9600);  // Set baud rate.
  momentum.begin();    // Begin communication with Momentum.
}

void loop() {
  momentum.getLinAccel(data);

  Serial.print("lin_accel_x:");
  Serial.print(data.lin_accel_x, 6);
  Serial.print(",lin_accel_y:");
  Serial.print(data.lin_accel_y, 6);
  Serial.print(",lin_accel_z:");
  Serial.println(data.lin_accel_z, 6);

  delay(5);  // 5 millisecond delay between prints.
}
