"""Bring up rebel + MoveIt (FHTW) + RealSense (optional point cloud for RViz)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _schedule_realsense(context, *args, **kwargs):
    """Defer rs_launch.py so V4L2 is not contended during ros2_control / MoveIt init."""
    delay_sec = float(LaunchConfiguration("realsense_start_delay").perform(context))
    realsense_share = get_package_share_directory("realsense2_camera")
    launch_realsense = LaunchConfiguration("launch_realsense")
    realsense_pointcloud = LaunchConfiguration("realsense_pointcloud")

    realsense_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "pointcloud.enable": realsense_pointcloud,
        }.items(),
    )
    group = GroupAction(
        condition=IfCondition(launch_realsense),
        actions=[realsense_inc],
    )
    if delay_sec <= 0.0:
        return [group]
    return [TimerAction(period=delay_sec, actions=[group])]


def generate_launch_description():
    rebel_share = get_package_share_directory("igus_rebel")
    moveit_share = get_package_share_directory("igus_rebel_moveit_fhtw")

    hardware_protocol = LaunchConfiguration("hardware_protocol")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gui = LaunchConfiguration("use_gui")

    rebel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rebel_share, "launch", "rebel.launch.py")
        ),
        launch_arguments={
            "hardware_protocol": hardware_protocol,
        }.items(),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "move_group.launch.py")
        ),
        launch_arguments={
            "hardware_protocol": hardware_protocol,
            "use_sim_time": use_sim_time,
            "use_gui": use_gui,
        }.items(),
    )

    # Give ros2_control spawners a moment before MoveIt connects
    delayed_move_group = TimerAction(period=3.0, actions=[move_group_launch])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_protocol",
                default_value="rebel",
                description="URDF / control mode: rebel, mock_hardware, or gazebo",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="MoveIt and bundled RViz use_sim_time",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Start RViz from move_group.launch.py",
            ),
            DeclareLaunchArgument(
                "launch_realsense",
                default_value="true",
                description="If false, skip camera; run rs_launch.py yourself with any args",
            ),
            DeclareLaunchArgument(
                "realsense_start_delay",
                default_value="8.0",
                description="Seconds before starting rs_launch.py (0 = immediate; reduces EBUSY)",
            ),
            DeclareLaunchArgument(
                "realsense_pointcloud",
                default_value="true",
                description="Forward to rs_launch: pointcloud.enable (RViz PointCloud2 display)",
            ),
            rebel_launch,
            OpaqueFunction(function=_schedule_realsense),
            delayed_move_group,
        ]
    )
