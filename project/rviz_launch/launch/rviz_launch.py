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

    #  Start nodes
    localization = Node(
        package='odometry',
        executable='odometry',
        output='screen'
    )

    icp_node = Node(
        package='icp_node',
        executable='icp_node',
        output='screen'
    )

    occupancy_grid_map = Node(
        package='occupancy_grid_map',
        executable='occupancy_grid_map',
        output='screen'
    )

    planner = Node(
        package='planner',
        executable='planner_exploration',
        output='screen'
    )

    a_star = Node(
        package='planner',
        executable='a_star_algorithm',
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

    static_tf_base_lidar = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0.09', '0.12', '0', '0', '0',  '1', 'base_link', 'lidar_link'],
    output='screen'
    )

    return LaunchDescription([
        drive_control,
        occupancy_grid_map,
        planner,
        a_star,
        localization,
        icp_node,
        perception,
        object_mapping,
        static_tf_base_lidar
    ])

