import math
import rclpy
import rclpy.clock
from rclpy.node import Node
import rclpy.time
from robp_interfaces.msg import DutyCycles
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
import random
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool

class SampleDriveControlNode(Node):

    def __init__(self):
        super().__init__('sample_drive_control_node')

        self.vel_forward = 0.13
        self.vel_rotate = 0.09
        self.vel_small_rotate = 0.015
        self.vel_arrived = 0.05

        # stop variable
        self.stop = False
        self.drive_to_free = False

        #Create buffer to look for transform 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        #Create publisher to publish motor control
        self.motor_publisher = self.create_publisher(DutyCycles, 'motor/duty_cycles', 10)
        self.drive_feedback = self.create_publisher(Bool, 'drive/feedback', 1)

        self.create_subscription(Path, 'drive/path', self.path_cb, 1)
        self.create_subscription(Bool, 'drive/stop', self.stop_cb, 1)
        self.create_subscription(Path, 'avoidance/drive_to_free', self.drive_free_cb, 1)

    def path_cb(self, msg:Path):
        #Call back that iterates in poses and drives to them, can maybe implement offset if it is needed

        self.drive_to_free = False

        for pose in msg.poses:
          result =  self.set_drive_input(pose.pose.position.x, pose.pose.position.y)
          if result == False:
              break

        msg_stop = DutyCycles()
        msg_stop.duty_cycle_right = 0.0
        msg_stop.duty_cycle_left = 0.0
        self.motor_publisher.publish(msg_stop)

        msg_feedback = Bool()
        msg_feedback.data = result
        self.drive_feedback.publish(msg_feedback)
    
    def stop_cb(self, msg:Bool):
        #Callback that sets stop variable to True if stop
        self.stop = msg.data

    def drive_free_cb(self, msg:Path):
        
        self.drive_to_free = True 
        self.stop = False
        
        for pose in msg.poses:
            self.set_drive_input(pose.pose.position.x, pose.pose.position.y)

        self.drive_to_free = False
        msg_stop = DutyCycles()
        msg_stop.duty_cycle_left = 0.0
        msg_stop.duty_cycle_right = 0.0
        self.motor_publisher.publish(msg_stop)

    def set_drive_input(self, x, y):

        msg = DutyCycles()
        sample_point = PointStamped()

        sample_point.header.frame_id = 'map'
        sample_point.header.stamp = rclpy.time.Time()
        sample_point.point.x = x
        sample_point.point.y = y
        sample_point.point.z = 0.0

        #While loop that rotates robot until aligned   
        while True:

            if self.stop == True and self.drive_to_free == False:
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
        
            #If y is zero and x > 0 means perfect alignment otherwise turning
            if x >= 0.0 and abs(y) < 0.05:
                #Stop turning
                msg.duty_cycle_right = self.vel_arrived
                msg.duty_cycle_left = self.vel_arrived
                self.motor_publisher.publish(msg)
                break
            elif y >= 0.0:
                #Turn left
                msg.duty_cycle_right = self.vel_rotate
                msg.duty_cycle_left = -self.vel_rotate
                self.motor_publisher.publish(msg)
            elif y < 0.0:
                #Turn right
                msg.duty_cycle_right = -self.vel_rotate
                msg.duty_cycle_left = self.vel_rotate
                self.motor_publisher.publish(msg)
 
        #While loop that drives forward until reaching point
        while True:

            if self.stop == True and self.drive_to_free == False:
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

            if abs(x) < 0.05:
                #Stop driving
                msg.duty_cycle_right = self.vel_arrived
                msg.duty_cycle_left = self.vel_arrived
                self.motor_publisher.publish(msg)
                self.get_logger().info(f'SUCCESS, point reached')
                return True
            elif y > 0.05:
                #Small turn left
                msg.duty_cycle_right = self.vel_forward + self.vel_small_rotate
                msg.duty_cycle_left = self.vel_forward - self.vel_small_rotate
                self.motor_publisher.publish(msg)
            elif y < 0.05:
                #Small turn right
                msg.duty_cycle_right = self.vel_forward - self.vel_small_rotate
                msg.duty_cycle_left = self.vel_forward + self.vel_small_rotate
                self.motor_publisher.publish(msg)
            else:
                #Drive forward
                msg.duty_cycle_right = self.vel_forward
                msg.duty_cycle_left = self.vel_forward
                self.motor_publisher.publish(msg)

def main():
    rclpy.init()
    node = SampleDriveControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()