import launch
from launch import LaunchDescription
from launch_ros.actions import Node  # Only import Node from launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory  # Import this for package directory lookup

def generate_launch_description():
    # Define the path to the RViz configuration file
    # rviz_config_file = os.path.join(
    #     get_package_share_directory('perception_pkg'), 'launch', 'perception.rviz'
    # )

    # Declare launch actions
    return LaunchDescription([
        # Launch RViz2 with the specified config file
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config_file]
        # ),
        
        # Launch static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_odom',
            arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_lidar',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
            output='screen'
        ),

        # Start odometry node
        Node(
            package='odometry',
            executable='odometry',
            name='odometry',
            output='screen'
        ),
        
        # Start detection node
        Node(
            package='perception_pkg',
            executable='detection_node',
            name='detection',
            output='screen'
        ),
        
        # Start lidar node
        Node(
            package='lidar',
            executable='lidar',
            name='lidar',
            output='screen'
        ),

        # Start object mapping node
        Node(
            package='object_mapping',
            executable='object_mapping',
            name='object_mapping',
        ),
    ])
