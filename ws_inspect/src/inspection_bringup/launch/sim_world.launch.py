#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration


def _launch_setup(context):
    assets_dir = Path(__file__).resolve().parents[2] / "sim_world_assets"
    world_name = LaunchConfiguration("world").perform(context)
    headless_value = LaunchConfiguration("headless").perform(context).lower()
    world_path = (assets_dir / "worlds" / world_name).resolve()

    command = ["gz", "sim", "-r", str(world_path)]
    if headless_value in (
        "true",
        "1",
        "yes",
    ):  # ensure headless option toggles renderer
        command.append("--headless-rendering")

    return [
        ExecuteProcess(
            cmd=command,
            output="screen",
            additional_env={"GZ_SIM_RESOURCE_PATH": str(assets_dir)},
        )
    ]


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
            SetEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH",
                str(Path(__file__).resolve().parents[2] / "sim_world_assets"),
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                str(Path(__file__).resolve().parents[2] / "sim_world_assets"),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
