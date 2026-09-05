#!/usr/bin/env python3
"""Expand the FR3 + Robotiq xacro entirely on Windows (no ROS required)."""
import os
import shutil
import tempfile
import xacro


def main() -> int:
    here = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    output = os.path.join(os.path.dirname(__file__), "..", "assets", "fr3_robotiq_combined.urdf")
    with tempfile.TemporaryDirectory() as temp:
        fairino = os.path.join(temp, "fairino_description")
        robotiq = os.path.join(temp, "robotiq_description")
        shutil.copytree(os.path.join(here, "src", "robotiq_description"), robotiq)
        os.makedirs(os.path.join(fairino, "urdf"), exist_ok=True)
        shutil.copy2(os.path.join(here, "src", "fairino3_v6.urdf"), os.path.join(fairino, "urdf", "fairino3_v6.urdf"))
        main_xacro = os.path.join(temp, "fairino3_v6_robot.urdf.xacro")
        shutil.copy2(os.path.join(here, "src", "fairino3_v6_moveit2_config", "config", "fairino3_v6_robot.urdf.xacro"), main_xacro)
        shutil.copy2(os.path.join(here, "src", "fairino3_v6_moveit2_config", "config", "fairino3_v6_robot.ros2_control.xacro"), os.path.join(temp, "fairino3_v6_robot.ros2_control.xacro"))
        initial_positions = os.path.join(here, "src", "fairino3_v6_moveit2_config", "config", "initial_positions.yaml")
        for root, _, files in os.walk(temp):
            for name in files:
                if name.endswith((".xacro", ".urdf")):
                    path = os.path.join(root, name)
                    text = open(path, encoding="utf-8").read()
                    text = text.replace("$(find fairino_description)", fairino.replace(os.sep, "/"))
                    text = text.replace("$(find robotiq_description)", robotiq.replace(os.sep, "/"))
                    open(path, "w", encoding="utf-8").write(text)
        doc = xacro.process_file(main_xacro, mappings={"add_robotiq_gripper": "true", "add_wrist_camera": "true", "use_gazebo_ros2_control": "false", "use_fixed_base": "true", "enable_grasp_fix": "false", "initial_positions_file": initial_positions})
        os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
        urdf_text = doc.toxml().replace(fairino.replace(os.sep, "/"), os.path.join(here, "src").replace(os.sep, "/"))
        urdf_text = urdf_text.replace(robotiq.replace(os.sep, "/"), os.path.join(here, "src", "robotiq_description").replace(os.sep, "/"))
        urdf_text = urdf_text.replace("package://fairino_description", os.path.join(here, "src").replace(os.sep, "/"))
        urdf_text = urdf_text.replace("package://robotiq_description", os.path.join(here, "src", "robotiq_description").replace(os.sep, "/"))
        open(output, "w", encoding="utf-8").write(urdf_text)
    print(f"created {os.path.abspath(output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
