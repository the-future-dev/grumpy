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
        self.filepath = 'object_map.txt'
        with open(self.filepath, 'w') as file:
            pass # creates empty file

        # Initialize the transform listener and assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize the subscriber to the object poses 
        self.create_subscription(Pose, '/object_poses', self.object_pose_callback, 10)
        
    
    def object_pose_callback(self, msg: Pose):
        """
        Writes the object poses and removes (or at least tries) duplicates
        """
        label = 'L'
        x = float(round(msg.position.x, 2))
        y = float(round(msg.position.y, 2))
        angle = 0
        
        #if object list is empty add the first entry otherwise look for duplicates
        if not self.object_list:
            self.object_list.append((label,x,y,angle))

            with open(self.filepath, 'a') as file:
                file.write(f"{label} {x} {y} {angle}\n")
        
        else:
            # Go over current objects to not add duplicates
            duplicate = False
            for obj in self.object_list:
                _,x_obj,y_obj,_ = obj

                if math.hypot(x-x_obj, y-y_obj) < self.margin:
                    duplicate = True

                
            if not duplicate:
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