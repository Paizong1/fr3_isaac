param(
    [Parameter(Mandatory)] [double]$X,
    [Parameter(Mandatory)] [double]$Y,
    [Parameter(Mandatory)] [double]$Z,
    [string]$FrameId = "base_link",
    [int]$DomainId = 42,
    [double]$Rate = 5.0
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$wslRoot = "/mnt/$($root.Substring(0, 1).ToLower())$(($root.Substring(2) -replace '\\', '/'))"
$script = "$wslRoot/scripts/publish_target_snapshot.py"
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$xArg = $X.ToString($culture)
$yArg = $Y.ToString($culture)
$zArg = $Z.ToString($culture)
$rateArg = $Rate.ToString($culture)

wsl.exe -d Ubuntu-22.04 -u paizong -- bash -lc "source /opt/ros/humble/setup.bash && source /home/paizong/.venvs/fr3-yolo/bin/activate && export ROS_DOMAIN_ID=$DomainId ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp && python3 '$script' --x $xArg --y $yArg --z $zArg --frame-id '$FrameId' --rate $rateArg --domain-id $DomainId"
