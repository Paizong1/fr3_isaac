param([string]$IsaacPython = 'C:\app\issacsim\python.bat')

$ErrorActionPreference = 'Stop'
$asset = Join-Path $PSScriptRoot '..\..\fairino3_robotiq.usd'
& $IsaacPython (Join-Path $PSScriptRoot 'scripts\run_fr3_native.py') --asset $asset --headless --test-frames 60
if ($LASTEXITCODE) { exit $LASTEXITCODE }
Write-Host 'Windows Isaac Sim validation passed.'
