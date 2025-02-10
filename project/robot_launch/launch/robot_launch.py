import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource, YamlLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    package_name = 'robp_launch'

    # Paths to different launch files
    phidgets_launch_path = os.path.join(
        launch_ros.substitutions.FindPackageShare(package_name).perform(None),
        'launch',
        'phidgets_launch.py'
    )

    frames_launch_path = os.path.join(
        launch_ros.substitutions.FindPackageShare(package_name).perform(None),
        'launch',
        'frames_launch.xml'
    )

    # yaml_launch_path = os.path.join(
    #     launch_ros.substitutions.FindPackageShare(package_name).perform(None),
    #     'launch',
    #     'yaml_launch.yaml'
    # )

    # Include Python launch file
    phidgets_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(phidgets_launch_path)
    )

    # Include XML launch file
    frames_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(frames_launch_path)
    )

    # # Include YAML launch file
    # include_yaml = IncludeLaunchDescription(
    #     YamlLaunchDescriptionSource(yaml_launch_path)
    # )

    # Start nodes
    odometry = Node(
        package='odometry',
        executable='odometry',
        output='screen'
    )

    lidar = Node(
        package='lidar',
        executable='lidar',
        output='screen'
    ) 

    detection = Node(
        package='detection',
        executable='detection',
        output='screen'
    )

    object_mapping = Node(
        package='object_mapping',
        executable='object_mapping',
        output='screen'
    )

    # Static Transforms
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '1', 'map', 'odom'],
        output='screen'
    )

    static_tf_base_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.1', '0.1', '0', '0', '1', 'base_link', 'lidar_link'],
        output='screen'
    )

    return LaunchDescription([
        phidgets_launch,
        frames_launch,
        odometry,
        lidar,
        detection,
        object_mapping,
        static_tf_map_odom,
        static_tf_base_lidar
    ])
