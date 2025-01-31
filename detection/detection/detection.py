#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

import tf2_geometry_msgs

from geometry_msgs.msg import Pose, TransformStamped


import ctypes
import struct


class Detection(Node):

    def __init__(self):
        super().__init__('detection')
        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/ds_points', 10)

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.cloud_callback, 10)
        
        # Initialize the transform listener and assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        


    def place_object_frame(self, points, frame_id, stamp, object_name):
        """Take points that correspond to an object and place a frame for it in RViz"""
        x_points = []
        y_points = []
        z_points = []
        for point in points:
            x,y,z = point
            x_points.append(x)
            y_points.append(y)
            z_points.append(z)
        
        x_obj = np.mean(x_points)
        y_obj = np.mean(y_points)
        z_obj = np.mean(z_points)

        # crete pose from these mean values
        pose = Pose()
        pose.position.x = x_obj
        pose.position.y = y_obj
        pose.position.z = z_obj

        # the camera's orientation is not the same as the object, which is assumed to have a similar definition of a coordinate system as the aruco markers in part 2
        q_euler = quaternion_from_euler(math.pi/2, 0, -math.pi/2, 'rxyz')
        pose.orientation.x = q_euler[0]
        pose.orientation.y = q_euler[1]
        pose.orientation.z = q_euler[2]
        pose.orientation.w = q_euler[3]
        
        # Now we want to express the pose in the map frame, hence we get the transform between the frames
        to_frame_rel = 'map'
        from_frame_rel = frame_id
        time = rclpy.time.Time().from_msg(stamp)

        # Wait for the transform asynchronously
        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time
        )

        # Spin until transform found or `timeout_sec` seconds has passed
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        # lookup transform from frame id to map 
        try:
            t_map_cam = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, time)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # transform the marker pose so that it is expressed in map coordinates 
        pose = tf2_geometry_msgs.do_transform_pose(pose, t_map_cam)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = object_name
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        self.tf_broadcaster.sendTransform(t)


    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
        """
        #print(msg.header.frame_id) # camera_depth_optical_frame

        # camera_link offset from base_link
        cam_x_offset = 0.08987
        cam_y_offset = 0.0175
        cam_z_offset = 0.10456
        
        # Convert ROS -> NumPy
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]
        #print('shape of points before filtering', np.shape(points))

        relevant_indices = [] 
        for j, point in enumerate(points):
            x,y,z = point
            # subtract camera offset from coordinates so that they are expressed in base_link coordinates
            x += cam_x_offset
            y += cam_y_offset
            z += cam_z_offset
            if math.hypot(x,y,z) <= 0.9 and z >= 0:
                relevant_indices.append(j) # append indices of points that are above ground and no further than 0.9m from the robot

        # filter so that only points of interest remains
        gen = gen[relevant_indices, :]
        points = points[relevant_indices, :]

        # convert these points to color
        colors = np.empty(points.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            colors[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            colors[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            colors[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        colors = colors.astype(np.float32) / 255
        #print('shape of colors', np.shape(colors))

        red_vote = 0
        green_vote = 0
        factor_r_g = 2
        factor_rg_b = 1
        red_points = []
        green_points = []
        for ind, color in enumerate(colors):
            r,g,b = color
            if r>factor_r_g*g and r>factor_rg_b*b:
                red_vote += 1
                red_points.append(points[ind, :])
            elif g>factor_r_g*r and g>factor_rg_b*b:
                green_vote += 1
                green_points.append(points[ind, :])


        if red_vote > green_vote:
            self.get_logger().info('\033[31m Detected the red sphere \033[0m') 
            #msg_object_detected = self.points_to_pointcloud(msg.header, red_points, r=255, g=0, b=0)
            msg_object_detected = pc2.create_cloud_xyz32(msg.header, red_points)
            self._pub.publish(msg_object_detected)
            self.place_object_frame(red_points, msg.header.frame_id, msg.header.stamp, 'Red_Sphere')
        elif red_vote < green_vote:
            self.get_logger().info('\033[32m Detected the green cube \033[0m')
            #msg_object_detected = self.points_to_pointcloud(msg.header, green_points, r=0, g=255, b=0)
            msg_object_detected = pc2.create_cloud_xyz32(msg.header, green_points)
            self._pub.publish(msg_object_detected)
            self.place_object_frame(green_points, msg.header.frame_id, msg.header.stamp, 'Green_Cube')
        



def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()