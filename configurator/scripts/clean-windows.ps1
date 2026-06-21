$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root
Remove-Item -Recurse -Force -ErrorAction SilentlyContinue "$Root\dist", "$Root\build", "$Root\release"
