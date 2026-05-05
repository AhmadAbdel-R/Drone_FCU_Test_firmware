# Drone FCU Test Firmware

This repo contains FCU-side integration test firmware for ESP32.

The active firmware project is now organized under:

- `firmware/esp32_fcu_test/`

## What It Covers

- Sensor readout (IMU, barometer, distance)
- GPS parsing
- nRF24 receive path
- Basic HMI status rendering
- SAMD21 I2C ping check

## Build

1. Open `firmware/esp32_fcu_test` in PlatformIO.
2. Build and upload.
3. Monitor serial for sensor and link output.

## Purpose

This is a test harness for FCU subsystem integration before full production firmware split.
