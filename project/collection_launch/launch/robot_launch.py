import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    package_name = 'robp_launch'

    phidgets_launch = ExecuteProcess(
        cmd=['ros2', 'launch', package_name, 'phidgets_launch.py'],
        output='screen' 
        )

    front_camera_launch = ExecuteProcess(
        cmd=['ros2', 'launch', package_name, 'rs_d435i_launch.py'],
        output='screen' 
        )

    frames_launch = ExecuteProcess(
            cmd=['ros2', 'launch', package_name, 'frames_launch.xml'],
            output='screen' 
        )
    
    lidar_launch =  ExecuteProcess(
            cmd=['ros2', 'launch', package_name, 'lidar_launch.yaml'],
            output='screen' 
        )

    arm_camera_launch = ExecuteProcess(
            cmd=['ros2', 'launch', package_name, 'arm_camera_launch.yaml'],
            output='screen' 
        )
    
    arm_communication = ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/hiwonder_arm', '-v6'],   
    )
    

    # Static Transforms
    static_tf_base_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.09', '0.12', '0', '0', '0', '1', 'base_link', 'lidar_link'],
        output='screen'
    )

    return LaunchDescription([
        arm_camera_launch,
        phidgets_launch,
        frames_launch,
        lidar_launch,
        front_camera_launch,
        static_tf_base_lidar,
        arm_communication
    ])
