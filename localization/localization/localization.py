#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from tf2_ros import TransformException
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped, Pose
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener




class Localization(Node):

    def __init__(self):
        super().__init__('Localization')

        # Initialize the transform listener and assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize the path subsrciber, to know when to look for odometry information from the wheel encoders
        self.create_subscription(Path, 'path', self.odometry_callback , 10)

        # Initalize publisher to publish new pose
        self.new_pose_pub = self.create_publisher(PoseStamped, 'new_dead_reckoning_position', 10)



    def odometry_callback(self, msg: Path):
        """
        Callback that publishes pose of the robot given odometry information.
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
        pose_stamped.pose.position.x = t.transform.translation.x
        pose_stamped.pose.position.y = t.transform.translation.y
        pose_stamped.pose.position.z = t.transform.translation.z
        pose_stamped.pose.orientation.x = t.transform.rotation.x
        pose_stamped.pose.orientation.y = t.transform.rotation.y
        pose_stamped.pose.orientation.z = t.transform.rotation.z
        pose_stamped.pose.orientation.w = t.transform.rotation.w

        self.new_pose_pub.publish(pose_stamped)
        

  
  


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