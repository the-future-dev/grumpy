import rclpy
from rclpy.node import Node
import rclpy.time

from robp_interfaces.msg import DutyCycles
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from grumpy_interfaces.msg import ObjectDetection1D


class AlignTopic(Node):
    
    def __init__(self):
        super().__init__('align_topic') 

        self.align_status_pub = self.create_publisher(Bool, 'brain/align_status', 1)

        #Create buffer to look for transform 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        #Create publisher to publish motor control
        self.motor_publisher = self.create_publisher(DutyCycles, 'motor/duty_cycles', 10)

        self.create_subscription(ObjectDetection1D, 'brain/action/align', self.align_cb, 1)
    
    def align_cb(self, msg:ObjectDetection1D):
        # align robot to object
        x, y = msg.pose.position.x, msg.pose.position.y
        self.get_logger().info(f'align callback got x: {x}, y: {y}')
        msg_motor = DutyCycles()
        sample_point = PointStamped()

        sample_point.header.frame_id = 'map'
        sample_point.header.stamp = rclpy.time.Time()
        sample_point.point.x = x/100.0
        sample_point.point.y = y/100.0
        sample_point.point.z = 0.0
        from_frame = 'map'
        vel_arrived = 0.0
        right_extra = 1.075
        vel_rotate = 0.07  # 0.09 before
        max_aligns = 3

        while True:
            tf_future = self.tf_buffer.wait_for_transform_async('base_link', from_frame, self.get_clock().now())
            rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

            try:
                tf_odom_base_link = self.tf_buffer.lookup_transform('base_link', from_frame, rclpy.time.Time()) # get latest available transform
            except TransformException as ex:
                self.get_logger().info(f'could not transform{ex}')
                continue

            #Transform point from odom to base_link
            point_base_link = tf2_geometry_msgs.do_transform_point(sample_point, tf_odom_base_link)
            x = point_base_link.point.x
            y = point_base_link.point.y

            if abs(y) <= 0.15:
                vel_rotate  = 0.05
                max_aligns -= 1

            #If y is zero and x > 0 means perfect alignment otherwise turning
            if x>=0:
                if (abs(y) < 0.025 or max_aligns < 0):
                    #Stop turning
                    msg_motor.duty_cycle_right = vel_arrived*right_extra
                    msg_motor.duty_cycle_left = vel_arrived
                    self.motor_publisher.publish(msg_motor)
                    status_msg = Bool()
                    status_msg.data = True
                    self._logger.info(f'Align topic says robot is aligned')
                    self.align_status_pub.publish(status_msg) 
                    break
                elif y >= 0.0:
                    #Turn left
                    msg_motor.duty_cycle_right = vel_rotate*right_extra
                    msg_motor.duty_cycle_left = -vel_rotate
                    self.motor_publisher.publish(msg_motor)
                elif y < 0.0:
                    #Turn right
                    msg_motor.duty_cycle_right = -vel_rotate*right_extra
                    msg_motor.duty_cycle_left = vel_rotate
                    self.motor_publisher.publish(msg_motor)
            else:
                if y >= 0.0:
                    #Turn right
                    msg_motor.duty_cycle_right = -vel_rotate*right_extra
                    msg_motor.duty_cycle_left = vel_rotate
                    self.motor_publisher.publish(msg_motor)
                else:
                    #Turn left
                    msg_motor.duty_cycle_right = vel_rotate*right_extra
                    msg_motor.duty_cycle_left = -vel_rotate
                    self.motor_publisher.publish(msg_motor)


def main():
    rclpy.init()
    node = AlignTopic()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
