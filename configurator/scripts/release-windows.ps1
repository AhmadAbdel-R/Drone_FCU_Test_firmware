param(
  [switch]$SkipTests,
  [string]$CertificateThumbprint = "",
  [string]$InnoSetupCompiler = ""
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root

if (-not $SkipTests) {
  npm.cmd test
}

& (Join-Path $PSScriptRoot "package-windows.ps1")

$AppRoot = Join-Path $Root "build\AeroForge Configurator"
if ($CertificateThumbprint -ne "") {
  & (Join-Path $PSScriptRoot "sign-windows.ps1") -Path $AppRoot -CertificateThumbprint $CertificateThumbprint
}

if ($InnoSetupCompiler -eq "") {
  $DefaultIscc = "${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe"
  if (Test-Path $DefaultIscc) {
    $InnoSetupCompiler = $DefaultIscc
  }
}

if ($InnoSetupCompiler -ne "") {
  & $InnoSetupCompiler (Join-Path $Root "installer\FCU-Configurator.iss")
  if ($CertificateThumbprint -ne "") {
    & (Join-Path $PSScriptRoot "sign-windows.ps1") -Path (Join-Path $Root "release") -CertificateThumbprint $CertificateThumbprint
  }
} else {
  Write-Host "Inno Setup compiler was not found; skipped installer build."
}
