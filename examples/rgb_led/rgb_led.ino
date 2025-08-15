/*******************************************************************************
 * @file rgb_led.ino
 * @brief Momentum dev board simple RGB LED control example.
 *******************************************************************************
 */

/*******************************************************************************
 * WARNING!
 * The WS2812B can get surprisingly bright. Be responsible and set the LED
 * brightness by starting at a low value (1 out of 255).
 *******************************************************************************
 */

#include <scalpelspace_momentum.h>

Momentum momentum(10);  // SPI CS pin 10 (out-of-the-box default).
// Default: Serial for printing and SPI (4 MHz, mode 0, MSB first).

void setup() {
  momentum.begin();  // Initialize Momentum.

  // LED index [0 to 255] = 0 (onboard WS2812B LED).
  // Red       [0 to 255] = 2.
  // Green     [0 to 255] = 1.
  // Blue      [0 to 255] = 3.
  momentum.setLED(0, 2, 1, 3);  // Onboard LED is set to dim blueish purple.
}

void loop() {

  delay(3000);  // 3 second delay between LED colour changes.

  momentum.setLED(0, 7, 0, 0);  // Onboard LED is set to 7/255 bright red.

  delay(3000);  // 3 second delay between LED colour changes.

  momentum.setLED(0, 0, 3, 0);  // Onboard LED is set to 3/255 bright green.

  delay(3000);  // 3 second delay between LED colour changes.

  momentum.setLED(0, 0, 0, 2);  // Onboard LED is set to 2/255 bright blue.
}
