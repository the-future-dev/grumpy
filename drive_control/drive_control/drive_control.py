
import math
import rclpy
from rclpy.node import Node
import rclpy.time
from robp_interfaces.msg import DutyCycles
from geometry_msgs.msg import TwistStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import random

class SampleDriveControlNode(Node):

    def __init__(self, sample_x, sample_y):
        super().__init__('sample_drive_control_node')

        self.vel_forward = 0.1
        self.vel_rotate = 0.05

        #Create buffer to look for transform 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        #Create publisher to publish motor control
        self.motor_publisher = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)

    def set_drive_input(self, point_x, point_y):

        msg = DutyCycles()
        point_yaw = math.atan2(point_y, point_x)
        MAX_ITERATIONS = 100
        iteration = 0

        #While loop that rotates robot until aligned   
        while True:

            try:
                pose_transform = self.tf_buffer.lookup_transform('/map','/base_link', rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(f'could nor transform')
            
            rob_yaw = 2 * math.atan2(
                pose_transform.transform.rotation.z,
                pose_transform.transform.rotation.w)
                
            diff_yaw = point_yaw - rob_yaw
            diff_yaw = (diff_yaw + math.pi) % (2 * math.pi) - math.pi
                
            if abs(diff_yaw) < 0.1:
                msg.duty_cycle_right = 0.0
                msg.duty_cycle_left = 0.0
                self.motor_publisher.publish(msg)
                break
            elif diff_yaw < 0:
                msg.duty_cycle_right = -self.vel_rotate
                msg.duty_cycle_left = self.vel_rotate
                self.motor_publisher.publish(msg)
            elif diff_yaw > 0:
                msg.duty_cycle_right = self.vel_rotate
                msg.duty_cycle_left = -self.vel_rotate
                self.motor_publisher.publish(msg)

        #While loop that drives forward until reaching point
        while True:
            try:
                pose_transform = self.tf_buffer.lookup_transform('/map','/base_link', rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(f'could nor transform')

            rob_x = pose_transform.transform.translation.x
            rob_y = pose_transform.transform.translation.y

            diff_x = abs(rob_x - point_x)
            diff_y = abs(rob_y - point_y)

            if diff_x < 0.1 and diff_y < 0.1:
                msg.duty_cycle_right = 0.0
                msg.duty_cycle_left = 0.0
                self.motor_publisher.publish(msg)
                break
            else:
                msg.duty_cycle_right = self.vel_forward
                msg.duty_cycle_left = self.vel_forward
                self.motor_publisher.publish(msg)


def main():
    rclpy.init()
    node = SampleDriveControlNode()

    try:
        while True:
            point_x = random.uniform(-2, 2)
            point_y = random.uniform(-2, 2)
            node.set_drive_input(point_x, point_y)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
