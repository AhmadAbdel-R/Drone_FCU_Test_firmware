# Drone FCU Test Firmware

Integration-heavy FCU test firmware for ESP32.

## Active Firmware Folder

- `firmware/esp32_fcu_test/`

## What This Test Build Covers

- IMU, barometer, and distance sensor reads
- GPS parsing
- nRF receive path
- SAMD21 I2C health check
- simple on-device status UI

## Purpose

This is a test harness, not final flight firmware.
It helps validate subsystem behavior before splitting code into stricter production modules.

## Build

1. Open `firmware/esp32_fcu_test` in PlatformIO.
2. Build and upload.
3. Watch serial + screen output for sensor/comms state.
