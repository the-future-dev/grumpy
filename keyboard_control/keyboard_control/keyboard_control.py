import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles
from geometry_msgs.msg import TwistStamped

class MotorControlNode(Node):

    def __init__(self):
        super().__init__('motor_control_node')
        self.speed_forward = 0.1
        self.speeed_rotate = 0.1
        self.motor_publisher = self.create_publisher(
            DutyCycles,
            '/motor/duty_cycles',
            10)
        
        self.subscription = self.create_subscription(
            TwistStamped,
            '/keyboard_control/keys',
            self.twist_callback,
            10)
        
        self.get_logger().info("Keyboard Motor Control initialized")
        
    def twist_callback(self, msg:TwistStamped): 
        linear = msg.twist.linear.x
        angular = msg.twist.angular.z
            
        self.publish_duty_cycles(linear, angular)

    def publish_duty_cycles(self, linear, angular):
        if linear > 0 and angular == 0:
            duty_cycle_left = self.speed_forward
            duty_cycle_right = self.speed_forward
        elif linear < 0 and angular == 0:
            duty_cycle_left = -self.speed_forward
            duty_cycle_right = -self.speed_forward
        elif linear == 0 and angular > 0:
            duty_cycle_left = -self.speeed_rotate
            duty_cycle_right = self.speeed_rotate
        elif linear == 0 and angular < 0:
            duty_cycle_left = self.speeed_rotate
            duty_cycle_right = -self.speeed_rotate
        else:
            duty_cycle_left = 0.0
            duty_cycle_right = 0.0

        # NO diagonal movement at the moment:    
            # duty_cycle_left = linear + angular
            # duty_cycle_right = linear - angular

        msg = DutyCycles()
        msg.duty_cycle_left = max(-1.0, min(1.0, duty_cycle_left))
        msg.duty_cycle_right =  max(-1.0, min(1.0, duty_cycle_right))

        # self.get_logger().info(f"Publishing duty cycles: left={duty_cycle_left}, right={duty_cycle_right}")
        #self.get_logger().info(f"TYPE{type(duty_cycle_left)}")
        self.motor_publisher.publish(msg)

def main(args=None):
    rclpy.init()
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()