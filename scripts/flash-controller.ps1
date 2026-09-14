param(
    [Parameter(Mandatory = $true)]
    [string]$Port
)

$ErrorActionPreference = "Stop"
$projectDirectory = Join-Path $PSScriptRoot "..\apps\controller"

if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) {
    throw "idf.py was not found. Run this script from an ESP-IDF 5.0 PowerShell."
}

Write-Host "When 'Connecting...' appears: hold BOOT, tap EN/RESET, then release BOOT when writing starts."
Push-Location $projectDirectory
try {
    & idf.py -p $Port flash monitor
    if ($LASTEXITCODE -ne 0) { throw "Flash or monitor command failed." }
}
finally {
    Pop-Location
}
