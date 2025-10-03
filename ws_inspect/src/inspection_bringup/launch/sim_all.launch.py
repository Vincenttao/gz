#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LAUNCH_DIR = Path(__file__).resolve().parent


def _launch_setup(context):
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(LAUNCH_DIR / "sim_world.launch.py")),
        launch_arguments={
            "world": world,
            "headless": headless,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(LAUNCH_DIR / "sim_nav2.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    mission_node = Node(
        package="inspection_mission",
        executable="mission_node",
        name="mission_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "loop": True,
                "alarm_stop": True,
                "waypoints": [
                    {
                        "header": {"frame_id": "map"},
                        "pose": {
                            "position": {"x": 2.0, "y": 0.0, "z": 0.0},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                        },
                    },
                    {
                        "header": {"frame_id": "map"},
                        "pose": {
                            "position": {"x": 6.0, "y": 1.5, "z": 0.0},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.7071, "w": 0.7071},
                        },
                    },
                ],
            },
        ],
    )

    thermal_node = Node(
        package="inspection_thermal",
        executable="thermal_alarm_node",
        name="thermal_alarm_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "trigger_distance": 2.0,
                "hysteresis": 0.3,
                "check_rate_hz": 10,
                "hot_targets": [
                    {
                        "position": {"x": 5.5, "y": 1.2, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    {
                        "position": {"x": 9.0, "y": -0.5, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                ],
            },
        ],
    )

    return [world_launch, nav2_launch, mission_node, thermal_node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="port_belt_corridor_fixed.sdf",
                description="Simulation world filename located in sim_world_assets/worlds",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run Gazebo without rendering windows",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation clock",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
