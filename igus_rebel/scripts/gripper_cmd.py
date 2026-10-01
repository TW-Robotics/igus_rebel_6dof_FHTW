#!/usr/bin/python3
"""Open or close the Schunk gripper: ros2 run igus_rebel gripper_cmd.py open|close"""
import sys

import rclpy
from std_srvs.srv import Trigger


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd not in ("open", "close"):
        print("usage: gripper_cmd.py open|close")
        sys.exit(1)
    rclpy.init()
    node = rclpy.create_node("gripper_cmd")
    client = node.create_client(Trigger, f"/gripper/{cmd}")
    if not client.wait_for_service(timeout_sec=3.0):
        print("gripper node not running")
        sys.exit(1)
    fut = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    print(fut.result().message if fut.result() else "no response")


if __name__ == "__main__":
    main()
