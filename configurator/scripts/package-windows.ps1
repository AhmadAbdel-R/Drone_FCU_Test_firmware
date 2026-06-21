$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root

$AppName = "AeroForge Configurator"
$BuildRoot = Join-Path $Root "build"
$AppRoot = Join-Path $BuildRoot $AppName
$ReleaseRoot = Join-Path $Root "release"
$RuntimeRoot = Join-Path $Root "node_modules\nw\nwjs-v0.98.0-win-x64"
$ZipPath = Join-Path $ReleaseRoot "FCU-Configurator-Portable-x64.zip"

if (-not (Test-Path (Join-Path $RuntimeRoot "nw.exe"))) {
  throw "NW.js Windows runtime was not found at $RuntimeRoot. Run npm install first."
}

npm.cmd install
npm.cmd run build

if (Test-Path $AppRoot) {
  Remove-Item -LiteralPath $AppRoot -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $AppRoot | Out-Null
New-Item -ItemType Directory -Force -Path $ReleaseRoot | Out-Null

Copy-Item -Path (Join-Path $RuntimeRoot "*") -Destination $AppRoot -Recurse -Force
Copy-Item -Path (Join-Path $Root "dist") -Destination (Join-Path $AppRoot "dist") -Recurse -Force
Copy-Item -Path (Join-Path $Root "serial-bridge") -Destination (Join-Path $AppRoot "serial-bridge") -Recurse -Force
New-Item -ItemType Directory -Force -Path (Join-Path $AppRoot "assets") | Out-Null
Copy-Item -Path (Join-Path $Root "src\assets\aeroforge-icon.svg") -Destination (Join-Path $AppRoot "assets\aeroforge-icon.svg") -Force
New-Item -ItemType Directory -Force -Path (Join-Path $AppRoot "node_modules") | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $AppRoot "node_modules\@serialport") | Out-Null
Copy-Item -Path (Join-Path $Root "node_modules\@serialport\*") -Destination (Join-Path $AppRoot "node_modules\@serialport") -Recurse -Force
foreach ($ModuleName in @("serialport", "debug", "ms", "node-addon-api", "node-gyp-build")) {
  $ModulePath = Join-Path $Root "node_modules\$ModuleName"
  if (Test-Path $ModulePath) {
    Copy-Item -Path $ModulePath -Destination (Join-Path $AppRoot "node_modules\$ModuleName") -Recurse -Force
  }
}
$NodeCommand = Get-Command node.exe -ErrorAction SilentlyContinue
if ($NodeCommand) {
  Copy-Item -Path $NodeCommand.Source -Destination (Join-Path $AppRoot "serial-bridge\node.exe") -Force
} else {
  Write-Warning "node.exe was not found on PATH; COM bridge will require node.exe on the target machine."
}

$Manifest = Get-Content -Path (Join-Path $Root "nwjs\package.json") -Raw | ConvertFrom-Json
$Manifest.main = "dist/index.html"
$Manifest.window.icon = "assets/aeroforge-icon.svg"
$Manifest | ConvertTo-Json -Depth 8 | Set-Content -Path (Join-Path $AppRoot "package.json") -Encoding UTF8
Copy-Item -Path (Join-Path $AppRoot "nw.exe") -Destination (Join-Path $AppRoot "$AppName.exe") -Force

if (Test-Path $ZipPath) {
  Remove-Item -LiteralPath $ZipPath -Force
}
Compress-Archive -Path (Join-Path $AppRoot "*") -DestinationPath $ZipPath -Force

Write-Host "Portable app folder created at $AppRoot"
Write-Host "Portable ZIP created at $ZipPath"
