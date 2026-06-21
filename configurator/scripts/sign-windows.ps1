param(
  [Parameter(Mandatory = $true)]
  [string]$Path,
  [Parameter(Mandatory = $true)]
  [string]$CertificateThumbprint,
  [string]$TimestampUrl = "http://timestamp.digicert.com"
)

$ErrorActionPreference = "Stop"

function Find-SignTool {
  $Candidates = @(
    "${env:ProgramFiles(x86)}\Windows Kits\10\bin\*\x64\signtool.exe",
    "${env:ProgramFiles}\Windows Kits\10\bin\*\x64\signtool.exe"
  )
  foreach ($Pattern in $Candidates) {
    $Match = Get-ChildItem -Path $Pattern -ErrorAction SilentlyContinue |
      Sort-Object FullName -Descending |
      Select-Object -First 1
    if ($Match) {
      return $Match.FullName
    }
  }
  throw "signtool.exe was not found. Install the Windows SDK or pass signing through CI."
}

if (-not (Test-Path $Path)) {
  throw "Path not found: $Path"
}

$SignTool = Find-SignTool
$Targets = Get-ChildItem -Path $Path -Recurse -File -Include *.exe,*.msi,*.dll
if ($Targets.Count -eq 0) {
  throw "No signable files found under $Path"
}

foreach ($Target in $Targets) {
  & $SignTool sign /fd SHA256 /tr $TimestampUrl /td SHA256 /sha1 $CertificateThumbprint $Target.FullName
}

Write-Host "Signed $($Targets.Count) file(s) under $Path"
