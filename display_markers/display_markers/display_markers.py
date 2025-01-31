#!/usr/bin/env python

import math
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import tf2_geometry_msgs

from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped


class DisplayMarkers(Node):

    def __init__(self):
        super().__init__('display_markers')

        # Initialize the transform listener and assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to aruco marker topic and call callback function on each received message
        self.create_subscription(
            MarkerArray, '/aruco/markers', self.aruco_callback, 10)

    def aruco_callback(self, msg: MarkerArray):
        
        # take the latest detected aruco marker
        marker = msg.markers[-1]

        #print('Header frame id', msg.header.frame_id) # the frame_id is camera_color_optical_frame
        #print('Aruco frame id', marker.header.frame_id) # the frame_id is camera_color_optical_frame

        pose = marker.pose.pose

        # change the orientation to display the orientation difference between the aruco marker and the camera frame     
        to_frame_rel = marker.header.frame_id
        from_frame_rel = 'aruco/marker'+str(marker.id)
        time = rclpy.time.Time().from_msg(marker.header.stamp)
        try:
            t_camera_aruco = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, time)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        pose.orientation.x = t_camera_aruco.transform.rotation.x
        pose.orientation.y = t_camera_aruco.transform.rotation.y
        pose.orientation.z = t_camera_aruco.transform.rotation.z
        pose.orientation.w = t_camera_aruco.transform.rotation.w

        # A more hard coded version of what is done above
        # # camera's orientation is not the same as the map frame or aruco frame
        # q_euler = quaternion_from_euler(math.pi/2, 0, -math.pi/2, 'rxyz')
        # pose.orientation.x = q_euler[0]
        # pose.orientation.y = q_euler[1]
        # pose.orientation.z = q_euler[2]
        # pose.orientation.w = q_euler[3]


        # lookup transform from the frame id of the latest marker to map 
        to_frame_rel = 'map'
        from_frame_rel = marker.header.frame_id
        time = rclpy.time.Time().from_msg(marker.header.stamp)
       
        try:
            t_map_optical = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, time)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # transform the marker pose so that it is expressed in map coordinates 
        pose = tf2_geometry_msgs.do_transform_pose(pose, t_map_optical)

        # Broadcast the transform between the map frame and the detected aruco marker
        t = TransformStamped()
        t.header.stamp = marker.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = '/aruco/detected' + str(marker.id)
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        self.tf_broadcaster.sendTransform(t)
    
    
def main():
    rclpy.init()
    node = DisplayMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
