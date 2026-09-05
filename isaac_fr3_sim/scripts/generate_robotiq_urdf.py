#!/usr/bin/env python3
"""Expand the bundled Robotiq xacro on Windows without ROS."""
import argparse
import os
import shutil
import tempfile


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--package", default=os.path.join(os.path.dirname(__file__), "..", "..", "src", "robotiq_description"))
    p.add_argument("--output", default=os.path.join(os.path.dirname(__file__), "..", "assets", "robotiq_2f_85.urdf"))
    args = p.parse_args()
    package = os.path.abspath(args.package)
    output = os.path.abspath(args.output)
    with tempfile.TemporaryDirectory() as temp:
        copied = os.path.join(temp, "robotiq_description")
        shutil.copytree(package, copied)
        for root, _, files in os.walk(copied):
            for name in files:
                if name.endswith((".xacro", ".xml")):
                    path = os.path.join(root, name)
                    text = open(path, encoding="utf-8").read()
                    text = text.replace("$(find robotiq_description)", copied.replace(os.sep, "/"))
                    open(path, "w", encoding="utf-8").write(text)
        import xacro
        doc = xacro.process_file(os.path.join(copied, "urdf", "robotiq_2f_85_gripper.urdf.xacro"), mappings={"include_ros2_control": "false"})
        os.makedirs(os.path.dirname(output), exist_ok=True)
        open(output, "w", encoding="utf-8").write(doc.toxml())
    print(f"created {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
