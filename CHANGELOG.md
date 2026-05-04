# Changelog

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [Changelog](#changelog)
  * [v0.1.0 (2025-07-14)](#v010--2025-07-14-)
  * [v0.2.0 (2025-07-15)](#v020--2025-07-15-)
  * [v0.2.1 (2025-07-20)](#v021--2025-07-20-)
  * [v0.3.0 (2025-07-28)](#v030--2025-07-28-)
  * [v0.3.1 (2025-08-15)](#v031--2025-08-15-)
  * [v0.3.2 (2025-08-16)](#v032--2025-08-16-)
  * [v0.4.0 (2026-05-04)](#v040--2026-05-04-)
<!-- TOC -->

</details>

---

## [v0.1.0 (2025-07-14)](https://github.com/scalpelspace/scalpelspace_momentum/releases/tag/v0.1.0)

- Initial release.

---

## [v0.2.0 (2025-07-15)](https://github.com/scalpelspace/scalpelspace_momentum/releases/tag/v0.2.0)

**Additions:**

- Add `print_version.ino` example.

**Modifications:**

- Abstract and default Serial and SPI objects used within the `Momentum` class.
- Implement an initial private `_sequence` member within the `Momentum` class.
    - No frame checking logic yet.
- Minor documentation improvements.
- General code cleanup following initial release.

---

## [v0.2.1 (2025-07-20)](https://github.com/scalpelspace/scalpelspace_momentum/releases/tag/v0.2.1)

**Modifications:**

- Update `momentum_driver` for SPI specific driver usage.

---

## [v0.3.0 (2025-07-28)](https://github.com/scalpelspace/scalpelspace_momentum/releases/tag/v0.3.0)

**Modifications:**

- Update `momentum_driver` for improved IMU sensor data frame type range.

---

## [v0.3.1 (2025-08-15)](https://github.com/scalpelspace/scalpelspace_momentum/releases/tag/v0.3.1)

**Modifications:**

- Update maintainer information.
- Remove repeating section of docstring in example files.

---

## [v0.3.2 (2025-08-16)](https://github.com/scalpelspace/scalpelspace_momentum/releases/tag/v0.3.2)

**Additions:**

- Add MIT license.

---

## [v0.4.0 (2026-05-04)](https://github.com/scalpelspace/scalpelspace_momentum/releases/tag/v0.4.0)

**Additions:**

- Add `CHANGELOG.md`.

**Modifications:**

- Refactor rename `LICENSE.txt` to `LICENSE`.
- Update `momentum_driver` to version `v0.3.3`.
    - Refactor rename "GPS" to "GNSS".
    - Implement magnetometer data.
