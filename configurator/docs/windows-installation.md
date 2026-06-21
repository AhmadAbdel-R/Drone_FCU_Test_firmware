# Windows Installation

## Portable ZIP

1. Build the package:

```powershell
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE\configurator
npm.cmd run package:windows
```

2. Open `release\FCU-Configurator-Portable-x64.zip`.
3. Extract the folder and run `AeroForge Configurator.exe`.

The portable package includes NW.js, the built app, and the app manifest. It does not install drivers.

## Firmware Target

Flash the bench configurator firmware before connecting over USB or BLE:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32-s3-mini-configurator -t upload
```

The normal flight env keeps the configurator endpoint disabled unless its build flags are changed.

## USB COM Ports

Open the app, select `USB Serial`, then click `Refresh USB`. The app lists real Windows COM ports through the packaged serial bridge, for example `COM3 - USB Serial Device`. Select the ESP32 COM port and click `Connect`.

## BLE Pairing

BLE is disabled at boot unless `BLE_DEFAULT_ENABLED` is set. In the configurator env, press the boot/GPIO0 button three times inside the click window while the FCU is bench-idle to toggle BLE advertising. The configured status LEDs blink while advertising and go solid when connected.
