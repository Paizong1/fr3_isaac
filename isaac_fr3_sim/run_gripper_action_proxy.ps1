param([int]$DomainId = 42)

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$wslRoot = "/mnt/$($root.Substring(0, 1).ToLower())$(($root.Substring(2) -replace '\\', '/'))"
wsl.exe -d Ubuntu-22.04 -u paizong -- bash -lc "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=$DomainId && python3 '$wslRoot/scripts/gripper_action_proxy.py' --domain-id $DomainId"
