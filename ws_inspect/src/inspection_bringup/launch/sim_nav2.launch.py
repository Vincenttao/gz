#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def _launch_setup(context):
    use_sim_time_value = LaunchConfiguration("use_sim_time").perform(context)
    map_path = LaunchConfiguration("map").perform(context)
    params_file = str(CONFIG_DIR / "nav2_params.yaml")
    localization_params = str(CONFIG_DIR / "localization.yaml")

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("nav2_bringup")) / "launch" / "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time_value,
            "params_file": params_file,
        }.items(),
    )

    actions = [navigation_launch]

    if map_path:
        localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory("nav2_bringup")) / "launch" / "localization_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time_value,
                "map": map_path,
                "params_file": localization_params,
            }.items(),
        )
        actions.append(localization_launch)

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation clock",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Absolute path to an occupancy grid YAML for AMCL (leave empty to skip localization)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
