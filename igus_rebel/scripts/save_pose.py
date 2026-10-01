#!/usr/bin/python3
"""Save the current arm position as a named MoveIt pose: ros2 run igus_rebel save_pose.py <name>"""
import math
import re
import sys

import rclpy
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import JointState

JOINTS = [f"joint{i}" for i in range(1, 7)]


def main():
    if len(sys.argv) != 2 or not re.fullmatch(r"[A-Za-z0-9_]+", sys.argv[1]):
        print("usage: save_pose.py <name>   (letters, digits, _)")
        sys.exit(1)
    name = sys.argv[1]

    rclpy.init()
    node = rclpy.create_node("save_pose")
    got = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: got.update({k: v for k, v in zip(m.name, m.position) if k in JOINTS}), 10)
    for _ in range(50):
        if len(got) == 6:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(got) != 6:
        print("no /joint_states from the arm - is the launch running?")
        sys.exit(1)

    # Edit the source SRDF (symlink-install points the installed one at it)
    srdf = (get_package_prefix("igus_rebel_moveit_fhtw").replace("/install/", "/src/igus/igus_rebel/")
            + "/config/igus_complete.srdf")
    text = open(srdf).read()
    block = f"    <group_state name=\"{name}\" group=\"igus_rebel_arm\">\n" + "".join(
        f"        <joint name=\"{j}\" value=\"{got[j]:.4f}\"/>\n" for j in JOINTS) + "    </group_state>\n"
    pattern = re.compile(rf"    <group_state name=\"{name}\" group=\"igus_rebel_arm\">.*?</group_state>\n", re.S)
    if pattern.search(text):
        text = pattern.sub(block, text)
        action = "updated"
    else:
        text = text.replace("    <disable_collisions", block + "    <disable_collisions", 1)
        action = "added"
    open(srdf, "w").write(text)
    print(f"{action} pose \"{name}\": " + "  ".join(f"{j}={math.degrees(got[j]):.1f}deg" for j in JOINTS))
    print("restart the launch to use it")


if __name__ == "__main__":
    main()
