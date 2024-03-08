from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description."""

    default_location = "braemabuel"
    default_tif_file = default_location + ".tif"
    default_tif_color_file = default_location + "_color.tif"

    # tif loader node
    tif_loader = Node(
        package="grid_map_geo",
        namespace="grid_map_geo",
        executable="test_tif_loader",
        name="tif_loader",
        parameters=[
            {"tif_path": LaunchConfiguration("tif_path")},
            {"tif_color_path": LaunchConfiguration("tif_color_path")},
        ],
        output="screen",
        emulate_tty=True,
    )

    # rviz node
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", f'{Path(get_package_share_directory("snowsampler_rviz")) / "launch" / "config.rviz"}'],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            DeclareLaunchArgument(
                "location",
                default_value=default_location,
                description="Location.",
            ),
            DeclareLaunchArgument(
                "tif_path",
                default_value=f'{Path(get_package_share_directory("adaptive_snowsampler")) / "resources" / default_tif_file}',
                description="Full path to the elevation map file.",
            ),
            DeclareLaunchArgument(
                "tif_color_path",
                default_value=f'{Path(get_package_share_directory("adaptive_snowsampler")) / "resources" / default_tif_color_file}',
                description="Full path to the elevation texture file.",
            ),
        
            tif_loader,
            rviz,
        ]
    )