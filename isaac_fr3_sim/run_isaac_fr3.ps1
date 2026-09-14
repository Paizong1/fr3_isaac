param(
  [int]$DomainId = 42,
  [string]$IsaacPython = 'C:\app\issacsim\python.bat',
  [string]$Asset = '',
  [int]$CameraWidth = 640,
  [int]$CameraHeight = 480,
  [double]$CameraTickRate = 10.0,
  [switch]$Ros2 = $true,
  [switch]$FreezeBanana,
  [switch]$BananaContactProxy = $true,
  [switch]$StableGraspDemo,
  [switch]$BilateralGraspDemo,
  [switch]$FrictionGrasp = $true,
  [switch]$StartupTracking,
  [switch]$UseActiveViewportRgb = $true
)

$ErrorActionPreference = 'Stop'
if (-not $Asset) {
  $Asset = Join-Path $PSScriptRoot 'assets\fairino3_robotiq_complete.usd'
}
if ($Ros2) {
  wsl.exe -d Ubuntu-22.04 -u paizong -- true | Out-Null
  . (Join-Path $PSScriptRoot 'setup_isaac_ros2_env.ps1') -IsaacRoot (Split-Path $IsaacPython -Parent) -DomainId $DomainId
  $runnerArgs = @((Join-Path $PSScriptRoot 'scripts\run_fr3_scene.py'), '--asset', $Asset, '--domain-id', $DomainId, '--camera-width', $CameraWidth, '--camera-height', $CameraHeight, '--camera-tick-rate', $CameraTickRate)
  if ($UseActiveViewportRgb) { $runnerArgs += '--rgb-from-active-viewport' }
  if ($FreezeBanana) { $runnerArgs += '--freeze-banana' }
  if ($BananaContactProxy) { $runnerArgs += '--banana-contact-proxy' }
  if ($StableGraspDemo) { $runnerArgs += '--stable-grasp-demo' }
  if ($BilateralGraspDemo) { $runnerArgs += '--bilateral-grasp-demo' }
  if ($FrictionGrasp) { $runnerArgs += '--friction-grasp' }
  if ($StartupTracking) { $runnerArgs += '--startup-tracking' }
  & $IsaacPython @runnerArgs
} else {
  & $IsaacPython (Join-Path $PSScriptRoot 'scripts\run_fr3_native.py') --asset $Asset
}
if ($LASTEXITCODE) { exit $LASTEXITCODE }
