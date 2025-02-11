#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node



from tf2_ros import TransformException, TransformBroadcaster
from sensor_msgs.msg import  Imu, LaserScan
from geometry_msgs.msg import PoseStamped

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Localization(Node):

    def __init__(self):
        super().__init__('Localization')

        # Initialize the transform listener and assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Intialize subscriber to imu
        self.create_subscription(Imu, '/imu/data_raw', self.imu_callback , 10)

        # Initalize publisher to publish new pose
        self.new_pose_pub = self.create_publisher(PoseStamped, 'dead_reckoning_position', 10)

        # Initalize subscriber to lidar node
        self.create_subscription(LaserScan, '/scan', self.lidar_callback , 10)

        # Intialize to true to be able to save the first lidar scan
        self.first_lidar_scan = True



    def imu_callback(self, msg: Imu):
        """
        Callback that publishes pose of the robot given odometry information from motor encoders and imu.
        """
        from_frame_rel = 'odom'
        to_frame_rel = 'base_link'

        time = rclpy.time.Time().from_msg(msg.header.stamp)

        # Wait for the transform asynchronously
        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time
        )

        # Spin until transform found or `timeout_sec` seconds has passed
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        # lookup transform between object and odom
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                time)          
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.header.frame_id = 'odom'

        # Set x y and z position based on transform from odometry node
        pose_stamped.pose.position.x = t.transform.translation.x
        pose_stamped.pose.position.y = t.transform.translation.y
        pose_stamped.pose.position.z = t.transform.translation.z

        # Set orientation based on data from the imu
        pose_stamped.pose.orientation.x = msg.orientation.x
        pose_stamped.pose.orientation.y = msg.orientation.y
        pose_stamped.pose.orientation.z = msg.orientation.z
        pose_stamped.pose.orientation.w = msg.orientation.w

        self.new_pose_pub.publish(pose_stamped)


    def lidar_callback(self, msg: LaserScan):
        if self.first_lidar_scan:
            self.lidar_control = msg
            self.first_lidar_scan = False
        else:
            # Do ICP on new lidar scan and broadcast the new transform between map and odom
            pass
        

def main():
    rclpy.init()
    node = Localization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()