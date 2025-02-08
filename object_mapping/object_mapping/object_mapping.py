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




class ObjectMapping(Node):

    def __init__(self):
        super().__init__('object_mapping')

        # Intialize object list
        self.object_list = []

        # Set margin for adding new object
        self.margin = 0.1

        # Intialize map file
        self.filepath = '/home/robot/dd2419_ws/object_map.txt'
        with open(self.filepath, 'w') as file:
            pass # creates empty file

        # Initialize the transform listener and assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
         # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(PointCloud2, '/camera/depth/color/ds_points', self.detection_callback, 10)

    def detection_callback(self, msg: PointCloud2):
        """
        Callback that when a pointcloud of an object is published it writes the position from the transform published at the same time into a text file.
        """
        for object_name in ['Red_Sphere', 'Green_Cube']:

            from_frame_rel = 'odom'
            to_frame_rel = object_name
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
                continue
            
            # set parameters for potential new object or box
            label = 'R' if object_name=='Red_Sphere' else 'G'
            x = float(round(t.transform.translation.x, 2))
            y = float(round(t.transform.translation.y, 2))
            angle = 0

            # if object list is empty add the first entry otherwise look for duplicates
            if not self.object_list:
                self.object_list.append((label,x,y,angle))
                with open(self.filepath, 'a') as file:
                    file.write(f"{label} {x} {y} {angle}\n")
            
            else:
                # Go over current objects to not add duplicates
                coordinates_list = []
                for obj in self.object_list:
                    _,x_obj,y_obj,_ = obj
                    coordinates_list.append((x_obj,y_obj))

                coordinates_array = np.array(coordinates_list)

                differences = np.abs(coordinates_array - (x,y))
                matches = np.all(differences <= self.margin, axis=1)
                if np.any(matches):
                    continue

                # If new object is further away than the margin it is added in the map file and the object list
                self.object_list.append((label,x,y,angle))
                with open(self.filepath, 'a') as file:
                    file.write(f"{label} {x} {y} {angle}\n")


def main():
    rclpy.init()
    node = ObjectMapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()