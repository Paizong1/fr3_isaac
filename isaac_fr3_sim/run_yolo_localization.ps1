param(
    [int]$DomainId = 42,
    [string]$Weights = "yolov8n.pt",
    [double]$Confidence = 0.25,
    [string]$TargetFrame = "base_link",
    # ponytail: Isaac RGB and depth helpers can run on separate ticks; this is safe only before the arm starts moving.
    [double]$SyncToleranceSec = 120.0
)

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $root
$wslRepo = "/mnt/$($repoRoot.Substring(0, 1).ToLower())$(($repoRoot.Substring(2) -replace '\\', '/'))"
$script = "$wslRepo/src/fairino3_v6_moveit2_config/scripts/yolov8_overlay_node.py"
$weightsPath = "$wslRepo/$Weights"

wsl.exe -d Ubuntu-22.04 -u paizong -- bash -lc "source /opt/ros/humble/setup.bash && source /home/paizong/.venvs/fr3-yolo/bin/activate && export ROS_DOMAIN_ID=$DomainId ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp && python3 '$script' --ros-args -p image_in:=/wrist_camera/image_raw_direct -p depth_image:=/wrist_camera/depth/image_raw_direct -p depth_camera_info:=/wrist_camera/depth/camera_info_direct -p detector_mode:=rgb -p min_score:=$Confidence -p sync_tolerance_sec:=$SyncToleranceSec -p target_detection_classes:='banana_rgb' -p publish_3d:=true -p target_frame:='$TargetFrame'"
