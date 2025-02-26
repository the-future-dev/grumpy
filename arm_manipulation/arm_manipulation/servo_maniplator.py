import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Pose
import numpy as np

class ServoThetaControlNode(Node):

    def __init__(self):
        super().__init__('servo_thetas_control_node')

        # Origin of servo 5 in base_link frame:
        self.x_origin_servo5 = -0.00450
        self.y_origin_servo5 = -0.04750
        self.z_origin_servo5 =  0.12915

        # Constants in the robot arm:
        # Links:
        # From joint of servo 5 to joint of servo 4:
        self.l1 = 0.10048
        # From joint of servo 4 to joint of servo 3:
        self.l2 = 0.094714
        # From joint of servo 3 to joint of servo 2 + from joint servo 2 to griping point
        self.l3 = 0.05071 + 0.11260

        # Origin of servo 4 in rhoz-plane
         


        # Sets angles of the servos for different tasks, as well as time for the arm to move into these positions:
        # Arm pointing straight up, used for reset and driving around
        self.initial_thetas = [1000, 12000, 12000, 12000, 12000, 12000]
        # Angles when the arm camera has a view over the entire pick-up area
        self.view_thetas = [-1, -1, 3000, 17500, 9000, -1]
        # Angles from which the inverse kinematics are calculated for picking up objects
        self.pre_grasp_thetas = [-1, -1, 3000, 14500, 6000, -1]
        # Angles for droping objects into the bins
        self.drop_thetas = [-1 , -1, 3000, 14500, 9000, -1]

        # Standard angle movement times to all positions
        self.times = [1500, 1500, 1500, 1500, 1500, 1500]

        self.servo_angle_publisher = self.create_publisher(
            Int16MultiArray,
            '/multi_servo_cmd_sub',
            1
        )
        
        self.servo_subscriber = self.create_subscription(
            Int16MultiArray,
            '/servo_pos_publisher',
            self.servo_callback,
            1
        )
        
        self.object_subscriber = self.create_subscription(
            Pose,
            '/object_position',
            self.get_position,
            1
        )
        
    def servo_callback(self, msg:Int16MultiArray):
        current_servo_angles = msg.data

        assert current_servo_angles is list

        self.get_logger.info('Got the angles of the servos')

    
    def get_position(self, msg:Pose):
        x, y, z = msg.position.x, msg.position.y, msg.position.z

        assert x and y and z is float

        self.get_logger.info('Got the 1D Pose of the object')

        self.publish_servo_angles(x, y, z)


    def publish_servo_angles(self, x, y, z):

        # Initializes the message with required informaiton
        msg = Int16MultiArray()
        msg.layout.dim[0] = {'label':'', 'size': 0, 'stride': 0}
        msg.layout.data_offset = 0
        
        # The angle that servo 6 has to move to so that the arm points towards the object
        theta_servo6 = self.initial_thetas[5] + self.get_delta_theta_6(x, y)

        # The hypotenuse (rho) from the position of servo 5 to the object position in the xy-plane
        rho = np.sqrt(np.power(x - self.x_origin_servo5, 2) + np.power(y - self.y_origin_servo5, 2))



        self.servo_angle_publisher.publish(msg)

    def get_delta_theta_6(self, x, y):
        """
        Args:
            x: x-position of the object in base_link
            y: y-position of the object in base_link
        Returns:
            delta_theta_6: degrees that servo 6 has to rotate from its position
        """
        x_dist = x - self.x_origin_servo5
        y_dist = y - self.y_origin_servo5

        return np.arctan2(y_dist/x_dist)

def main(args=None):
    rclpy.init()
    node = ServoThetaControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()