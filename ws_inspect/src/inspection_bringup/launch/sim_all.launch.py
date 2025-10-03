#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LAUNCH_DIR = Path(__file__).resolve().parent


def _as_bool(value: str) -> bool:
    return value.lower() in {"true", "1", "yes", "on"}


def _launch_setup(context):
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    use_sim_time_raw = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time_bool = _as_bool(use_sim_time_raw)
    use_sim_time_arg = "true" if use_sim_time_bool else "false"

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
            "use_sim_time": use_sim_time_arg,
        }.items(),
    )

    mission_params = PathJoinSubstitution(
        [FindPackageShare("inspection_mission"), "params", "mission.yaml"]
    )

    thermal_params = PathJoinSubstitution(
        [FindPackageShare("inspection_thermal"), "params", "thermal.yaml"]
    )

    mission_node = Node(
        package="inspection_mission",
        executable="mission_node",
        name="mission_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time_bool},
            mission_params,
        ],
    )

    thermal_node = Node(
        package="inspection_thermal",
        executable="thermal_alarm_node",
        name="thermal_alarm_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time_bool},
            thermal_params,
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
