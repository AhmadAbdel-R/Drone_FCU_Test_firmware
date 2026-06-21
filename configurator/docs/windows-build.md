# Windows Build

Build the Vue/NW app shell:

```powershell
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE\configurator
npm.cmd run build
npm.cmd run package:windows
```

Current output:

- `dist/` contains the built app assets.
- `build\AeroForge Configurator\` contains the portable NW.js app folder.
- `release\FCU-Configurator-Portable-x64.zip` contains the portable Windows x64 app.

Optional installer build:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\release-windows.ps1 -InnoSetupCompiler "C:\Program Files (x86)\Inno Setup 6\ISCC.exe"
```

Signing requires a certificate thumbprint:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\release-windows.ps1 -CertificateThumbprint "<sha1-thumbprint>"
```
