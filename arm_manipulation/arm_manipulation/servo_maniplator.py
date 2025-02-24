import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

class ServoAngleControlNode(Node):

    def __init__(self):
        super().__init__('arm_control_node')

        self.servo_angle_publisher = self.create_publisher(
            Int16MultiArray,
            'multi_servo_cmd_sub',
            1)
        
        self.servo_subscription = self.create_subscription(
            Int16MultiArray,
            '/servo_pos_publisher',
            self.servo_callback,
            1)
        
        self.get_logger().info("Keyboard Motor Control initialized")
        
    def servo_callback(self, msg:Int16MultiArray):
        print('Hello:', msg)
        

    def publish_servo_angles(self):
        
        self.servo_angle_publisher.publish()

def main(args=None):
    rclpy.init()
    node = ServoAngleControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()