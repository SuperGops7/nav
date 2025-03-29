import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    ld = LaunchDescription()

    rviz_config_path = os.path.join(
        get_package_share_directory("trajectory_capture"),
        "rviz",
        "trajectory_visual.rviz",
    )

    # Map server
    map_file_path = os.path.join(
        get_package_share_directory("trajectory_capture"), "maps", "map.yaml"
    )
    rviz_start = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": True}],
    )
    lifecycle_nodes = ["map_server"]
    use_sim_time = True
    autostart = True

    map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "yaml_filename": map_file_path,
                "topic_name": "map",
                "frame_id": "map",
            }
        ],
    )

    tf_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "1.951",
            "0.535",
            "-0.015",
            "0.071",
            "0.000",
            "-0.007",
            "map",
            "odom",
        ],
        output="screen",
    )

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    ld.add_action(tf_transform_node)
    ld.add_action(rviz_start)
    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
