#!/usr/bin/python3
"""Schunk gripper on the ReBeL digital outputs: DO30 = open, DO31 = close.

Offers
  /gripper/open, /gripper/close            std_srvs/Trigger
  /gripper_controller/gripper_cmd          control_msgs/GripperCommand (used by MoveIt)
and publishes the jaw position on /joint_states so RViz/MoveIt show the gripper state.
There is no sensor feedback: the commanded state is assumed after switch_time.
"""
import sys
import time

import rclpy
from control_msgs.action import GripperCommand
from igus_rebel_msgs.msg import DigitalOutput
from igus_rebel_msgs.srv import SetDigitalOutput
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

JOINT = "schunk_jaw_left_joint"


class Gripper(Node):
    def __init__(self):
        super().__init__("gripper")
        self.declare_parameter("open_output", 30)
        self.declare_parameter("close_output", 31)
        self.declare_parameter("stroke", 0.006)      # must match jaw_stroke in the URDF
        self.declare_parameter("switch_time", 0.5)   # seconds the jaws need to move
        self.do_open = self.get_parameter("open_output").value
        self.do_close = self.get_parameter("close_output").value
        self.stroke = self.get_parameter("stroke").value
        self.switch_time = self.get_parameter("switch_time").value
        self.position = self.stroke  # assume open until the first command

        group = ReentrantCallbackGroup()
        self.dio = self.create_client(SetDigitalOutput, "/set_digital_output", callback_group=group)
        self.create_service(Trigger, "~/open", lambda req, res: self.on_request(res, opening=True), callback_group=group)
        self.create_service(Trigger, "~/close", lambda req, res: self.on_request(res, opening=False), callback_group=group)
        ActionServer(self, GripperCommand, "/gripper_controller/gripper_cmd", self.on_action, callback_group=group)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.05, self.publish_state)

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [JOINT]
        msg.position = [self.position]
        self.js_pub.publish(msg)

    def set_outputs(self, opening):
        if not self.dio.service_is_ready():
            return False
        # Switch the other output off first, so both valves are never on together
        on, off = (self.do_open, self.do_close) if opening else (self.do_close, self.do_open)
        for out, state in ((off, False), (on, True)):
            self.dio.call_async(SetDigitalOutput.Request(output=DigitalOutput(output=out, is_on=state)))
        self.position = self.stroke if opening else 0.0
        self.get_logger().info("opening" if opening else "closing")
        return True

    def on_request(self, res, opening):
        res.success = self.set_outputs(opening)
        res.message = ("opening" if opening else "closing") if res.success else "/set_digital_output not available (driver running?)"
        return res

    def on_action(self, goal_handle):
        opening = goal_handle.request.command.position >= self.stroke / 2
        result = GripperCommand.Result()
        if not self.set_outputs(opening):
            goal_handle.abort()
            return result
        time.sleep(self.switch_time)
        result.position = self.position
        result.reached_goal = True
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    executor.add_node(Gripper())
    executor.spin()


if __name__ == "__main__":
    main()
