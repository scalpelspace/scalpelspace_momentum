# scalpelspace_momentum

Arduino library for SPI communication with the ScalpelSpace Momentum dev board.

> See [`momentum_demo`](https://github.com/scalpelspace/momentum_demo) for full
> project examples using this Arduino library.

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [scalpelspace_momentum](#scalpelspace_momentum)
  * [1 Overview](#1-overview)
  * [2 Examples](#2-examples)
<!-- TOC -->

</details>

---

## 1 Overview

Utilizes a local copy of [
`momentum_driver`](https://github.com/scalpelspace/momentum_driver) for low
level SPI operations (local copy only includes the C code files):

1. [src/momentum_driver](src/momentum_driver).

- Git submodule not utilized per [Arduino's
  `library-registry` FAQ](https://github.com/arduino/library-registry/blob/main/FAQ.md#are-git-submodules-supported).

---

## 2 Examples

Examples can be found in [`examples`](examples).

The minimum code is as shown below:

```cpp
#include <scalpelspace_momentum.h>

Momentum momentum(10);  // SPI CS pin 10 (out-of-the-box default).
// Default: Serial for printing and SPI (4 MHz, mode 0, MSB first).

sensor_data_t data;  // Sensor data variable.
version_t version;   // Firmware version variable.

void setup() {
  momentum.begin();  // Begin communication with Momentum.

  // ...
  // momentum.get ...
  // ...
}

void loop() {
  // ...
  // momentum.get ...
  // ...
}
```
