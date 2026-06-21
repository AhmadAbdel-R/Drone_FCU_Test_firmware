# Windows Release Checklist

## Local Checks

```powershell
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE\configurator
npm.cmd test
npm.cmd run build
npm.cmd run package:windows
```

Expected artifacts:

- `build\AeroForge Configurator\AeroForge Configurator.exe`
- `release\FCU-Configurator-Portable-x64.zip`

## Optional Signed Installer

Install Inno Setup 6 and the Windows SDK, then run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\release-windows.ps1 -CertificateThumbprint "<sha1-thumbprint>" -InnoSetupCompiler "C:\Program Files (x86)\Inno Setup 6\ISCC.exe"
```

This builds the portable app, signs the signable files, compiles `installer\FCU-Configurator.iss`, and signs the installer output when the certificate is available.

## Release Notes

Call out these constraints in any release:

- BLE transport is implemented but hardware validation is still pending.
- Firmware update UI and protocol IDs exist, but the FCU returns `NOT_SUPPORTED`.
- Deadman motor test requires props removed, frame restrained, disarmed idle state, throttle zero, and a live hold action.
