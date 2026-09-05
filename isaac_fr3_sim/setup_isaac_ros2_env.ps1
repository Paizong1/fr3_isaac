param([string]$IsaacRoot = 'C:\app\issacsim', [int]$DomainId = 42)

$ErrorActionPreference = 'Stop'
$env:ROS_DISTRO = 'humble'
$env:RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
$env:ROS_DOMAIN_ID = "$DomainId"
$env:ROS_LOCALHOST_ONLY = '0'
$rosRoot = Join-Path $IsaacRoot 'exts\isaacsim.ros2.core\humble'
$env:PATH = "$rosRoot\bin;$rosRoot\lib;$env:PATH"
Write-Host "Isaac Sim ROS 2 Humble environment configured."
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"
