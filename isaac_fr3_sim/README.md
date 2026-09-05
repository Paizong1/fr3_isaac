# Isaac Sim FR3 迁移骨架

这是 FR3 + Robotiq 2F-85 从 Gazebo 迁移到 Isaac Sim 的最小运行入口。

## 运行

Isaac Sim 和全部仿真逻辑运行在 Windows；Ubuntu、ROS 2、MoveIt2 均不是运行前提。

在 Windows 的 Isaac Sim 安装目录中运行（通常为 `C:\app\issacsim`）：

```powershell
& C:\app\issacsim\python.bat scripts\run_fr3_native.py
```

资源受限机器默认以腕部相机 `320×240 @ 10Hz` 运行，降低 RTX 渲染压力；可用
`--camera-width`、`--camera-height` 和 `--camera-tick-rate` 覆盖。

脚本默认寻找仓库上级目录的 `fairino3_robotiq.usd`。也可以显式指定：

```powershell
& C:\app\issacsim\python.bat scripts\run_fr3_native.py --asset C:\path\fairino3_robotiq_portable.usd
```

默认启动 Isaac Sim GUI、加载你提供的 `fairino3_robotiq.usd`、创建地面物理环境、初始化 Articulation，并执行机械臂与夹爪轨迹。ROS 2 Bridge 仅在显式使用启动器的 `-Ros2` 时启用。

若只运行 Windows-native 模式，不需要设置任何 ROS 环境变量。

Windows 无界面验收：

```powershell
.\test_windows.ps1
```

## 当前阶段（Windows Bridge 与场景基线）

Windows-native 阶段交付：使用你提供的 FR3+Robotiq 合并 USD、地面/工作台/目标物体、Articulation 控制和 Windows 一键启动。已验证该 USD 在 Isaac Sim 6.0.1 中包含 6 个 FR3 关节和 6 个 Robotiq 关节。

Bridge 冒烟测试（自动设置 Isaac Sim 自带 ROS 2 Humble 运行库）：

```powershell
.\run_isaac_fr3.ps1 -Ros2
```

当前 Bridge 脚本创建 `/ROS_FR3` 图和 ROS 2 context，并通过 Isaac 内置 `rclpy` 发布 `/clock` 与 `/joint_states`；完整 TF、RGB-D 和命令订阅将继续逐个接入。

完整 USD 现在还包含 Isaac 原生场景基线：`/World/Ground`、`/World/Worktable` 和 `/World/Banana`，用于先验证相机视野和深度数据。

已检查到原始 USD 的 root references 指向 `../ROS_JIXIEBI/...`；用户提供的合并文件可直接显示 FR3 与 Robotiq。脚本运行时补充腕部相机和 ROS 图，不改写源 USD。

```powershell
& C:\app\issacsim\python.bat scripts\prepare_fr3_usd.py `
  --source C:\path\fairino3_robotiq.usd `
  --output C:\path\fairino3_robotiq_portable.usd `
  --payload-root C:\path\fairino3_v6_2
```

接口名称保持与 Gazebo 版本一致，见 `config/topics.yaml`。

## 任务 12：香蕉二维检测与叠加

在 Isaac 场景已发布 RGB 后，于另一 PowerShell 窗口运行：

```powershell
.\run_yolo_overlay.ps1 -DomainId 42
```

首次运行前，执行一次 `./install_yolo_overlay_deps.ps1`；它会提示输入 WSL
用户的 sudo 密码并创建 `/home/paizong/.venvs/fr3-yolo`。

节点订阅 `/wrist_camera/image_raw`（Sensor Data QoS），发布原始二维
`vision_msgs/Detection2DArray` 到 `/yolo/detections`，以及叠加图到
`/yolo/dbg_image`。每条观测保留源图像的时间戳与 frame_id、bbox、类别和
score；默认不订阅深度或 TF，也不发布 `/yolo/target_pose`。三维定位留待任务 13。

它要求 WSL 已具备 `ros-humble-cv-bridge`、`ros-humble-vision-msgs`、OpenCV 和
`ultralytics`；启动时若 YOLO 无法导入，叠加图会显示错误并继续转发原图，便于诊断。

## 任务 13：香蕉三维定位

停止任务 12 的二维节点后，改用：

```powershell
.\run_yolo_localization.ps1 -DomainId 42 -TargetFrame base_link
```

节点在同一 RGB 观测时间戳上融合深度、CameraInfo 和 TF，发布兼容 MoveIt 的
`/yolo/target_pose`，以及供世界模型消费的 `/yolo/detections_3d`
(`vision_msgs/Detection3DArray`)。香蕉的稳定 ID 为 `banana-1`，其中包含类别、score、
目标 frame、时间戳、三维中心和由 2D bbox/深度估计的尺寸。

## 任务 14：执行抓取至抬升

完成工作区编译后，在已运行 MoveIt 的情况下，于新的 PowerShell 窗口运行：

```powershell
.\run_pick_banana.ps1 -DomainId 42
```

它订阅 `/yolo/target_pose`，执行接近、下探、夹爪闭合和抬升，并在
`/world_model/grasp_state` 发布带仿真时间戳的 JSON 状态；检测器先报告 `detected` 和
`localized`，抓取器再报告 `approach`、`contact`、`grasped`、`lifted` 或 `failed`
（包含阶段性失败原因）。加上 `-StartMoveGroup` 可由该
窗口同时启动 MoveIt；否则复用已启动的 MoveIt。放置位置尚未定义，因此安全地停在
`lifted`，不会擅自执行放置动作。

## 验收顺序

1. 生成的 portable USD 加载无 missing reference。
2. FR3 关节在 Isaac Sim 中稳定站立，模型内部的 Articulation Root 存在且无嵌套 root 报错。
3. `/fairino3_v6_robot/wrist_camera` 存在且姿态正确。
4. `run_fr3_native.py` 能在 Windows 中完成地面物理、工作台/目标物体加载、Articulation 初始化与重复关节轨迹控制。

当前脚本会在 home 姿态和任务姿态之间平滑往返，用于 Windows 端持续调试仿真链路；可用 `--cycle-frames` 调整周期。

Robotiq 资产的独立导入文件仍保留在 `assets/robotiq_2f_85/robotiq_2f_85.usda`，但主流程以用户提供的已合并 USD 为准。源 USD 的 NewtonMimic follower 关节目前仍有“未设置有限限位”的 PhysX 提示，不影响加载；下一步再做物理参数校准。
