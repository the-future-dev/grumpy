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

    # RViz Node
    rviz_node = Node(
        package='rviz2',                
        executable='rviz2',              
        output='screen',               
        arguments=['-d', rviz_config_file]  
    )


    # Start nodes
    localization = Node(
        package='localization',
        executable='localization',
        output='screen'
    )

    # odometry = Node(
    #     package='odometry',
    #     executable='odometry',
    #     output='screen'
    # )

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
        arguments=['0', '0', '0', '0', '0', '0',  '1', 'map', 'odom'],
        output='screen'
    )

    static_tf_base_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.09', '0.12', '0', '0', '0',  '1', 'base_link', 'lidar_link'],
        output='screen'
    )


    return LaunchDescription([
        rviz_node,
        lidar,
        detection,
        object_mapping,
        localization,
        static_tf_map_odom,
        static_tf_base_lidar
    ])
