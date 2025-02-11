import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
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
        output='screen'
    )

    perception = Node(
        package='perception_pkg',
        executable='detection_node',
        output='screen'
    )

    object_mapping = Node(
        package='object_mapping',
        executable='object_mapping',
        output='screen'
    )


    return LaunchDescription([
        drive_control,
        odometry,
        lidar,
        perception,
        object_mapping
    ])
