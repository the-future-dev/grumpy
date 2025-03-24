import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Start nodes
    brain_exploration = Node(
        package='brain',
        executable='brain_exploration',
        output='screen'
    )

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

    local_obstacle_avoidance = Node(
        package='drive_control',
        executable='local_obstacle_avoidance',
        output = 'screen'
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

    drive_path = Node(
        package='drive_control',
        executable='drive_2',
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
        brain_exploration,
        drive_path,
        occupancy_grid_map,
        local_obstacle_avoidance,
        planner,
        a_star,
        localization,
        icp_node,
        # perception,
        # object_mapping
    ])
