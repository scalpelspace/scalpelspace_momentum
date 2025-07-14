# scalpelspace_momentum

Arduino library for the Momentum dev board.

> See [`momentum_demo`](https://github.com/scalpelspace/momentum_demo) for full
> project examples using this Arduino library.

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [scalpelspace_momentum](#scalpelspace_momentum)
  * [1 Overview](#1-overview)
  * [2 Simple Example](#2-simple-example)
<!-- TOC -->

</details>

---

## 1 Overview

Utilizes a local copy of [
`momentum_driver`](https://github.com/scalpelspace/momentum_driver).

- Git submodule not utilized per [Arduino's
  `library-registry` FAQ](https://github.com/arduino/library-registry/blob/main/FAQ.md#are-git-submodules-supported).

---

## 2 Simple Example

```cpp
#include <scalpelspace_momentum.h>

Momentum momentum(10);  // SPI CS pin 10

sensor_data_t data;  // Sensor data variable

void setup() {
  Serial.begin(9600);  // Set baud rate
  momentum.begin();    // Begin communication with Momentum
}

void loop() {
  momentum.getLinAccel(data);

  Serial.print("lin_accel_x:");
  Serial.print(data.lin_accel_x, 6);
  Serial.print(",lin_accel_y:");
  Serial.print(data.lin_accel_y, 6);
  Serial.print(",lin_accel_z:");
  Serial.println(data.lin_accel_z, 6);

  delay(1);  // Small delay
}
```
