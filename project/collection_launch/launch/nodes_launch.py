import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Start nodes
    localization = Node(
        package='localization',
        executable='localization',
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
        executable='planner_collection',
        output='screen'
    )

    a_star = Node(
        package='planner',
        executable='a_star_algorithm',
        output='screen'
    )

    drive_control = Node(
        package='drive_control',
        executable='drive_2',
        output='screen'
    )

    perception = Node(
        package='perception_pkg',
        executable='detection_node',
        output='screen',
        parameters=[{'mode': 'collection'}]
    )

    object_mapping = Node(
        package='object_mapping',
        executable='object_mapping',
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
        object_mapping
    ])
