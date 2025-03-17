import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Path to the RViz config file 
    rviz_config_file = os.path.join(
        os.path.expanduser('~'), 
        'dd2419_ws', 'rviz', 'default_rviz.rviz'
    )

    # # RViz Node
    # rviz_node = Node(
    #     package='rviz2',                
    #     executable='rviz2',              
    #     output='screen',               
    #     arguments=['-d', rviz_config_file]  
    # )


    # Start nodes
    localization = Node(
        package='localization',
        executable='localization',
        output='screen'
    )

    icp = Node(
        package='icp_node',
        executable='icp_node',
        output='screen'
    ) 

    # odometry = Node(
    #     package='odometry',
    #     executable='odometry',
    #     output='screen'
    # )



    # Static Transforms
    static_tf_base_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.09', '0.12', '0', '0', '0',  '1', 'base_link', 'lidar_link'],
        output='screen'
    )

    # static_tf_base_imu = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0.09', '0.12', '0.7071', '0.7071', '0',  '0', 'base_link', 'imu_link'],
    #     output='screen'
    # )


    return LaunchDescription([
        # rviz_node,
        icp,
        localization,
        static_tf_base_lidar,
        # static_tf_base_imu
    ])
