from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    if not IfCondition(LaunchConfiguration("launch_realsense")).evaluate(context):
        return []
    delay_sec = float(LaunchConfiguration("realsense_start_delay").perform(context))
    realsense_share = get_package_share_directory("realsense2_camera")
    # Resolve parent arguments before entering the isolated camera launch scope.
    realsense_pointcloud = LaunchConfiguration("realsense_pointcloud").perform(context)
    initial_reset = LaunchConfiguration("realsense_initial_reset").perform(context)

    realsense_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "pointcloud.enable": realsense_pointcloud,
            # Reset is opt-in: this D435i can disappear from USB after a hardware reset.
            "initial_reset": initial_reset,
            # 640x480 keeps the point cloud light enough for a fast octomap
            "depth_module.depth_profile": "640,480,30",
            "rgb_camera.color_profile": "640,480,30",
            # depth pixel-aligned to colour, used by the plant_manipulation object map
            "align_depth.enable": "true",
        }.items(),
    )
    group = GroupAction(
        # rs_launch.py validates every launch configuration as a camera parameter.
        # Do not forward robot, MoveIt, or object-map launch arguments into it.
        scoped=True,
        forwarding=False,
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

    # Static shelf collision object (waits for move_group itself)
    shelf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("igus_scene"), "launch", "shelf.launch.py")
        )
    )

    # Schunk gripper on DO30 (open) / DO31 (close); MoveIt GripperCommand action + jaw joint state
    gripper_node = Node(package="igus_rebel", executable="gripper_node.py", output="screen", respawn=True, respawn_delay=2.0)

    # Foxglove: ws://<this-pc>:8765 - robot (URDF+TF), shelf (/scene_markers),
    # octomap (/occupied_cells_vis_array), camera (/camera/camera/color/image_raw/compressed)
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[{"port": 8765, "send_buffer_limit": 100000000}],
        condition=IfCondition(LaunchConfiguration("foxglove")),
    )

    # Display the authoritative MoveIt map, including loaded maps and live changes.
    octomap_markers = Node(
        package="igus_rebel",
        executable="octomap_markers",
        output="screen",
        parameters=[{"cube_scale": ParameterValue(LaunchConfiguration("octomap_cube_scale"), value_type=float)}],
        condition=IfCondition(LaunchConfiguration("foxglove")),
    )

    # Buttons/status for the Foxglove "Robot Control" panel (waits for move_group, respawns until it is up)
    foxglove_control = Node(
        package="plant_manipulation",
        executable="foxglove_control",
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        condition=IfCondition(LaunchConfiguration("foxglove")),
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
                "octomap_cube_scale",
                default_value="1.0",
                description="Foxglove cube size multiplier; visualization only",
            ),
            DeclareLaunchArgument(
                "foxglove",
                default_value="true",
                description="Start Foxglove bridge (port 8765), MoveIt octomap markers, and controls",
            ),
            DeclareLaunchArgument(
                "launch_realsense",
                default_value="true",
                description="If false, skip camera; run rs_launch.py yourself with any args",
            ),
            DeclareLaunchArgument(
                "realsense_initial_reset",
                default_value="false",
                description="Opt-in camera hardware reset on startup",
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
            shelf_launch,
            gripper_node,
            foxglove_bridge,
            octomap_markers,
            foxglove_control,
        ]
    )
