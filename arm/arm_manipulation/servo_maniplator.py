import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

class ServoAngleControlNode(Node):

    def __init__(self):
        super().__init__('arm_control_node')

        self.servo_angle_publisher = self.create_publisher(
            Int16MultiArray,
            '/multi_servo_cmd_sub',
            10)
        
        self.servo_subscription = self.create_subscription(
            Int16MultiArray,
            '/keyboard_control/keys',
            self.servo_callback,
            10)
        
        self.get_logger().info("Keyboard Motor Control initialized")
        
    def servo_callback(self, msg:Int16MultiArray):
        print('Hello:', msg)
        

    def publish_servo_angles(self):
        
        self.servo_angle_publisher.publish(msg)

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