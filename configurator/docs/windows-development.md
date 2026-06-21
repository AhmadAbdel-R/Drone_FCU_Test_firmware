# Windows Development

This app is Windows-only and targets Windows 10/11 x64.

## Development

```powershell
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE\configurator
npm.cmd install
npm.cmd run dev
```

The app can run against `Mock FCU` without hardware. USB serial requires the FCU to be flashed with:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32-s3-mini-configurator -t upload
```

## Tests

```powershell
npm.cmd test
```

## Native Modules

USB uses a child-process serial bridge by default. The NW.js renderer does not load the native `serialport` addon directly, because ABI mismatches can hard-crash the app during COM-port refresh.

The packaged app includes:

- `serial-bridge\serial-bridge.cjs`
- the `serialport` dependency subset under `node_modules`
- a local `serial-bridge\node.exe` copied from the build machine when available

If the bridge is missing, the app falls back to Chromium Web Serial when that runtime feature is available.

## Bluetooth LE

BLE uses Chromium/Web Bluetooth with the FCU GATT service:

- service `b7f3b8f0-6e6a-4d7a-9df7-2c13f0c00001`
- RX write characteristic `b7f3b8f1-6e6a-4d7a-9df7-2c13f0c00001`
- TX notify characteristic `b7f3b8f2-6e6a-4d7a-9df7-2c13f0c00001`

The app chunks binary frames into 160-byte writes. Hardware validation is still required before relying on BLE for bench operations.
