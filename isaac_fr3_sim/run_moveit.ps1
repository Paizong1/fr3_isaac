param([int]$DomainId = 42)

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $root
$wslRepo = "/mnt/$($repoRoot.Substring(0, 1).ToLower())$(($repoRoot.Substring(2) -replace '\\', '/'))"
wsl.exe -d Ubuntu-22.04 -u paizong -- bash -lc "source /opt/ros/humble/setup.bash && source '$wslRepo/install/setup.bash' && export ROS_DOMAIN_ID=$DomainId && ros2 launch fairino3_v6_moveit2_config move_group.launch.py use_sim_time:=true"
