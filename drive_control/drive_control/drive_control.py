
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

class SampleDriveControlNode(Node):

    def __init__(self):
        super().__init__('sample_drive_control_node')

        self.vel_forward = 0.2
        self.vel_rotate = 0.1
        self.vel_small_rotate = 0.02

        #Create buffer to look for transform 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        #Create publisher to publish motor control
        self.motor_publisher = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)

        #Call method 
        self.set_drive_input()

    def set_drive_input(self):

        msg = DutyCycles()
        sample_point = PointStamped()

        sample_point.header.frame_id = 'map'
        sample_point.header.stamp = rclpy.time.Time()
        sample_point.point.x = random.uniform(-0.75, 0.75)
        sample_point.point.y = random.uniform(-0.75, 0.75)
        sample_point.point.z = 0.0

        #print(sample_point.point.x, sample_point.point.y)

        self.get_logger().info(f'START TURNING')
        #While loop that rotates robot until aligned   
        while True:

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
            if x >= 0.0 and abs(y) < 0.01:
                #Stop turning
                msg.duty_cycle_right = 0.0
                msg.duty_cycle_left = 0.0
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

        self.get_logger().info(f'START DRIVING FORWARD')
 
        #While loop that drives forward until reaching point
        while True:
            
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

            if abs(x) < 0.01:
                #Stop driving
                print(x,y)
                msg.duty_cycle_right = 0.0
                msg.duty_cycle_left = 0.0
                self.motor_publisher.publish(msg)
                self.get_logger().info(f'SUCCESS, point reached')
                break
            elif y > 0.01:
                #Small turn left
                msg.duty_cycle_right = self.vel_forward + self.vel_small_rotate
                msg.duty_cycle_left = self.vel_forward - self.vel_small_rotate
                self.motor_publisher.publish(msg)
            elif y < 0.01:
                #Small turn right
                msg.duty_cycle_right = self.vel_forward - self.vel_small_rotate
                msg.duty_cycle_left = self.vel_forward + self.vel_small_rotate
                self.motor_publisher.publish(msg)
            else:
                #Drive forward
                msg.duty_cycle_right = self.vel_forward
                msg.duty_cycle_left = self.vel_forward
                self.motor_publisher.publish(msg)

        self.set_drive_input()

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
