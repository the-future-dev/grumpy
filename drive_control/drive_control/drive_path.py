
import rclpy
import numpy as np
import rclpy.clock
from rclpy.node import Node
import rclpy.time
from robp_interfaces.msg import DutyCycles
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool

class DrivePathNode(Node):

    def __init__(self):
        super().__init__('drive_path_node')

        #Drive control inputs
        self.vel_forward = 0.15
        self.vel_rotate = 0.1
        self.vel_small_rotate = 0.025
        self.vel_arrived = 0.05

        #Stop variable
        self.stop = False

        #Create buffer to look for transform 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        #Create publisher to publish motor control
        self.motor_publisher = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)
        self.drive_feedback = self.create_publisher(Bool, '/drive/feedback', 1)

        #Subscription to path from planner
        self.create_subscription(Path, 'path/next_goal', self.path_cb, 1)
        self.create_subscription(Bool, 'map/occupied_zone', self.stop_cb, 1)

    def path_cb(self, msg:Path):

        # Initialize stop mand feedback message
        msg_stop = DutyCycles()
        msg_stop.duty_cycle_right = 0.0
        msg_stop.duty_cycle_left = 0.0
        msg_feedback = Bool()

        #Get offset 
        x_offset, y_offset = self.find_offset()

        #Iterate thorugh poses in path
        for pose in msg.poses:
          result = self.set_drive_input(pose.pose.position.x, pose.pose.position.y, x_offset, y_offset)
          #If return false, means no more execution in path, break loop
          if result == False:
              break
          
        msg_feedback.data = result
        
        #Publish robot to stop and publish message
        self.motor_publisher.publish(msg_stop)
        self.drive_feedback.publish(msg_feedback)

    def stop_cb(self, msg:Bool):

        self.stop = msg.data

    def find_offset(self):
        #Function which for each path calculates teh offset between map and odom
        #to take into consideration since using odom transform

        #Finding transform from map to odom
        tf_future = self.tf_buffer.wait_for_transform_async('odom', 'map', self.get_clock().now())
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            tf = self.tf_buffer.lookup_transform('odom', 'map', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'could not transform{ex}')
            return 0, 0
        
        #Creating offset variables
        x_offset = tf.transform.translation.x
        y_offset = tf.transform.translation.y

        return x_offset, y_offset

    def set_drive_input(self, x, y, x_offset, y_offset):

        msg = DutyCycles()
        sample_point = PointStamped()

        sample_point.header.frame_id = 'map'
        sample_point.header.stamp = rclpy.time.Time()
        sample_point.point.x = x
        sample_point.point.y = y
        sample_point.point.z = 0.0

        #While loop that rotates robot until aligned   
        while True:

            if self.stop == True:
                return False

            tf_future = self.tf_buffer.wait_for_transform_async('base_link', 'odom', self.get_clock().now())
            rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

            try:
                tf = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(f'could not transform{ex}')
                continue

            #Transform point from map frame to base_link
            point_transform = tf2_geometry_msgs.do_transform_point(sample_point, tf)
            x = point_transform.point.x
            y = point_transform.point.y

            if abs(x - x_offset) < 0.05 and abs(y - y_offset) < 0.05:
                #Reached subpoint, slow speed
                msg.duty_cycle_left = self.vel_arrived
                msg.duty_cycle_right = self.vel_arrived
                self.motor_publisher.publish(msg)
                self.get_logger().info(f'SUCCESS, point reached')
                return True
            
            elif (x - x_offset) < abs(y - y_offset):
                #Turning condition
                vel_left, vel_right = self.robot_rotate(y, y_offset)
            else:
                #Otherwise driving forward
                vel_left, vel_right = self.robot_forward(y, y_offset)

            msg.duty_cycle_left = vel_left
            msg.duty_cycle_right = vel_right
            self.motor_publisher.publish(msg)

    def robot_rotate(self, y, y_offset):
            
            if (y - y_offset) >= 0.0:
                #Turn left
                return -self.vel_rotate, self.vel_rotate
            elif (y - y_offset) < 0.0:
                #Turn right
                return self.vel_rotate, -self.vel_rotate
    
    def robot_forward(self, y, y_offset):
            
            if abs(y - y_offset) < 0.05:
                #Straight forward
                return self.vel_forward, self.vel_forward
            elif (y - y_offset) > 0.05:
                #Smal turn left
                vel_left = self.vel_forward - self.vel_small_rotate
                vel_right = self.vel_forward + self.vel_small_rotate
                return vel_left, vel_right
            elif (y - y_offset) < -0.05:
                #Small turn right
                vel_left = self.vel_forward + self.vel_small_rotate
                vel_right = self.vel_forward - self.vel_small_rotate
                return vel_left, vel_right

def main():
    rclpy.init()
    node = DrivePathNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
