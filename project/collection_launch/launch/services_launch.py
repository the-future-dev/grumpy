import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

pkg_name_arm_services = 'arm_srvs'

def generate_launch_description():
    # Start nodes
    arm_camera = Node(
        package=pkg_name_arm_services,
        executable='arm_camera_service',
        output='screen'
    )
    drop = Node(
        package=pkg_name_arm_services,
        executable='drop_service',
        output='screen'
    )
    pick = Node(
        package=pkg_name_arm_services,
        executable='pick_service',
        output='screen'
    )

    # For later integration
    # positioning = Node(
    #     package=pkg_name_arm_services,
    #     executable='positioning_service',
    #     output='screen'
    # )

    return LaunchDescription([
        arm_camera,
        drop,
        pick
    ])