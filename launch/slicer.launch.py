from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("drone_frame",     default_value="base_link"),
        DeclareLaunchArgument("world_frame",     default_value="map"),
        DeclareLaunchArgument("slice_thickness", default_value="0.2"),
        DeclareLaunchArgument("unknown_as_free", default_value="false"),

        Node(
            package="octomap_2d_slicer",
            executable="octomap_2d_slicer_node",
            name="octomap_2d_slicer",
            output="screen",
            parameters=[{
                "drone_frame":     LaunchConfiguration("drone_frame"),
                "world_frame":     LaunchConfiguration("world_frame"),
                "slice_thickness": LaunchConfiguration("slice_thickness"),
                "unknown_as_free": LaunchConfiguration("unknown_as_free"),
            }],
            remappings=[
                # remap if your topics differ
                # ("octomap_binary", "/octomap_binary"),
                # ("octomap_2d_slice", "/map_2d"),
            ],
        ),
    ])