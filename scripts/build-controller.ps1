param(
    [switch]$Clean
)

$ErrorActionPreference = "Stop"
$projectDirectory = Join-Path $PSScriptRoot "..\apps\controller"

if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) {
    throw "idf.py was not found. Run this script from an ESP-IDF 5.0 PowerShell."
}

Push-Location $projectDirectory
try {
    if ($Clean -and (Test-Path "build")) {
        & idf.py fullclean
        if ($LASTEXITCODE -ne 0) { throw "idf.py fullclean failed." }
    }

    & idf.py build
    if ($LASTEXITCODE -ne 0) { throw "idf.py build failed." }
}
finally {
    Pop-Location
}
