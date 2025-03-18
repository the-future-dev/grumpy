#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.node import Node

import rclpy.time
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from tf2_ros import TransformException
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped, Pose
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from grumpy_interfaces.msg import ObjectDetection1D
import pandas as pd
import os

import tf2_geometry_msgs

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class WorkspaceInit(Node):

    def __init__(self):
        super().__init__('workspace_init')
        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,history=QoSHistoryPolicy.KEEP_LAST)

        # Initialize publisher to publish the workspace perimeter
        self.perimeter_pub = self.create_publisher(Path, '/workspace_init/perimeter', qos_profile)

        self.get_logger().info('Initalized workspace init, will now try to publish perimeter')

        self.publish_perimeter()
    
    def publish_perimeter(self)->None:
        """
        Method that publishes the perimeter of the workspace
        """
        # assumes you are running from DD2419_ws
        # file_path = 'src/workspace_2.tsv'

        # more dynamic way to do it
        # Get the absolute path of the current script
        current_dir = os.path.dirname(os.path.abspath(__file__))

        # Go three levels up and into src where the workspace file is located
        file_path = os.path.abspath(os.path.join(current_dir, '../../../src/workspace_2.tsv'))

        # make sure file path includes correct placement of the workspace file
        assert 'dd2419_ws/src/workspace_2.tsv' in file_path

        # read file
        df = pd.read_csv(file_path, sep = '\t')

        ws_points = df.values
        ws_points = ws_points/100

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg() # to have a time
        path.header.frame_id = 'map'
        first = True
        
        for x,y in ws_points:
            if first:
                x_0 = x
                y_0 = y
                first = False
            
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)

        # Go back to first point to close perimeter    
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = x_0
        pose.pose.position.y = y_0
        path.poses.append(pose)


        self.get_logger().info('Publishing the perimeter') 
        self.perimeter_pub.publish(path)

    


def main():
    rclpy.init()
    node = WorkspaceInit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()