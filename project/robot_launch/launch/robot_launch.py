import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource, YamlLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    package_name = 'robp_launch'

    phidgets_path = os.path.join(
        launch_ros.substitutions.FindPackageShare(package_name).perform(None),
        'launch',
        'phidgets_launch.py'
    )

    front_camera_path = os.path.join(
        launch_ros.substitutions.FindPackageShare(package_name).perform(None),
        'launch',
        'rs_d435i_launch.py'
    )


    frames_path = os.path.join(
        launch_ros.substitutions.FindPackageShare(package_name).perform(None),
        'launch',
        'frames_launch.xml'
    )


    arm_camera_path = os.path.join(
        launch_ros.substitutions.FindPackageShare(package_name).perform(None),
        'launch',
        'arm_camera_launch.yaml'
    )

    lidar_path = os.path.join(
        launch_ros.substitutions.FindPackageShare(package_name).perform(None),
        'launch',
        'lidar_launch.yaml'
    )

    phidgets_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(phidgets_path)
    )

    front_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(front_camera_path)
    )

    frames_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(frames_path)
    )

    lidar_launch = IncludeLaunchDescription(
        YamlLaunchDescriptionSource(lidar_path)
    )

    arm_camera_launch = IncludeLaunchDescription(
        YamlLaunchDescriptionSource(arm_camera_path)
    )


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

    drive_control = Node(
        package='drive_control',
        executable='drive_control',
        screen='screen'
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
        arm_camera_launch,
        phidgets_launch,
        frames_launch,
        lidar_launch,
        front_camera_launch,
        drive_control,
        odometry,
        lidar,
        detection,
        object_mapping,
        static_tf_map_odom,
        static_tf_base_lidar
    ])
