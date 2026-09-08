param(
    [int]$DomainId = 42,
    [switch]$StartMoveGroup,
    [switch]$FrictionGrasp
)

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $root
$wslRepo = "/mnt/$($repoRoot.Substring(0, 1).ToLower())$(($repoRoot.Substring(2) -replace '\\', '/'))"
$startMoveGroupValue = if ($StartMoveGroup) { 'true' } else { 'false' }
$frictionArgs = if ($FrictionGrasp) { ' gripper_slow_close:=false require_gripper_stall:=true' } else { '' }

wsl.exe -d Ubuntu-22.04 -u paizong -- bash -lc "source /opt/ros/humble/setup.bash && source '$wslRepo/install/setup.bash' && export ROS_DOMAIN_ID=$DomainId ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp && ros2 launch fairino3_v6_moveit2_config pick_banana.launch.py use_sim_time:=true start_move_group:=$startMoveGroupValue$frictionArgs"
