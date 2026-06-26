from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("mavlink_sensor_bridge"),
        "config",
        "mavlink_sensor_bridge.yaml",
    ])

    return LaunchDescription([
        Node(
            package="mavlink_sensor_bridge",
            executable="mavlink_sensor_bridge_node",
            name="mavlink_sensor_bridge_node",
            output="screen",
            parameters=[config_file],
        )
    ])
