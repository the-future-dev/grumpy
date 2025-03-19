import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Pose
import numpy as np
import time

class PickService(Node):

    def __init__(self):
        super().__init__('pick_srv')

        # Origin of servo 5 in base_link frame:
        self.x_origin_servo5 = -0.00450
        self.y_origin_servo5 = -0.04750
        self.z_origin_servo5 =  0.12915
        self.theta_servo5    =  60

        # Constants in the robot arm:
        # Links:
        # From joint of servo 5 to joint of servo 4:
        self.l1 = 0.10048
        # From joint of servo 4 to joint of servo 3:
        self.l2 = 0.094714
        # From joint of servo 3 to joint of servo 2 + from joint servo 2 to griping point
        self.l3 = 0.05071 + 0.11260

        # Origin of servo 4 in rho+z-plane
        self.z_origin_servo4   = self.z_origin_servo5 + self.l1 * np.sin(np.deg2rad(90) - np.deg2rad(self.theta_servo5))
        self.rho_origin_servo4 = self.l1 * np.cos(np.deg2rad(90) - np.deg2rad(self.theta_servo5))
         
        # Sets angles of the servos for different tasks, as well as time for the arm to move into these positions:
        # Arm pointing straight up, used for reset and driving around
        self.initial_thetas = [1000, 12000, 12000, 12000, 12000, 12000]
        # Angles when the arm camera has a view over the entire pick-up area
        self.view_thetas = [-1, -1, 3000, 17500, 9000, -1]
        # Angles from which the inverse kinematics are calculated for picking up objects
        self.pre_grasp_thetas = [-1, -1, 3000, 14500, 6000, -1]

        # Standard angle movement times to all positions
        self.times = [1000, 1000, 1000, 1000, 1000, 1000]

        self.srv = self.create_service(
            Pose, 
            'pick_up_object', 
            self.pick_up_sequence
        )

        self.servo_angle_publisher = self.create_publisher(
            Int16MultiArray,
            '/multi_servo_cmd_sub',
            1
        )
        
        # self.servo_subscriber = self.create_subscription(
        #     Int16MultiArray,
        #     '/servo_pos_publisher',
        #     self.servo_callback,
        #     1
        # )

        
    # def servo_callback(self, msg:Int16MultiArray):
    #     current_servo_angles = msg.data

    #     assert current_servo_angles is list

    #     self.get_logger.info('Got the angles of the servos')


    def pick_up_sequence(self, request, response):
        """
        Args:
            request: Pose, required, the position and orientation of the object
        Returns:
            response: Bool, if the pick up was successful or not
        Other functions:
            Controlls the pick up sequence
            Calls the publishing function which publishes the servo angles to the arm for each step in the sequence
        """

        step = "Start"
        x, y, z = 0, 0, 0

        while step != "Success" or step != "Failure":
            match step:
                case "Start":# Make sure the arm is in the initial position
                    self.publish_angles(self.initial_thetas)
                    step = "GetPosition"

                case "GetPosition":# Get the position of the object from the request
                    # Assert that the request has the correct type
                    assert request.pose is Pose, self._logger.error('request was not type Pose')
                    # Get the position of the object from the request
                    x, y, z = self.extract_object_position(request.pose)

                case "RotateBase":# Move servo 6/base to the correct angle
                    # The angle that servo 6 has to move to so that the arm points towards the object
                    theta_servo6 = self.initial_thetas[5] + self.get_delta_theta_6(x, y) * 100
                    thetas = [-1, -1, -1, -1, -1, theta_servo6]
                    self.publish_angles(thetas)
                    step = "PickOrigin"

                case "PickOrigin": # Move the arm to the position from were the inverse kinematics are calculated
                    thetas = [-1, -1, -1, -1, self.theta_servo5 * 100, -1]
                    self.publish_angles(thetas)
                    step = "InverseKinematics"

                case "InverseKinematics": # Move the arm to the grasp position calculated by the inverse kinematics
                    delta_theta_servo3, delta_theta_servo4 = self.inverse_kinematics(x, y, z)
                    theta_servo3 = self.initial_thetas[2] + delta_theta_servo3 * 100
                    theta_servo4 = self.initial_thetas[3] + delta_theta_servo4 * 100
                    thetas = [-1, -1, theta_servo3, theta_servo4, -1, -1]
                    self.publish_angles(thetas)
                
                case "GraspObject": # Grasp the object
                    thetas = [11000, -1, -1, -1, -1, -1]
                    self.publish_angles(thetas, times=[3000, 2000, 2000, 2000, 2000, 2000])
                    step = "Finish"

                case "Finish": # Finish the pick up sequence by going back to the initial position, but not for the gripper
                    initial_thetas = self.initial_thetas.copy()
                    initial_thetas[0] = -1
                    self.publish_angles(initial_thetas, times=[2000, 2000, 2000, 2000, 2000, 2000])
                    step = "Success"
                    
        response.success = True if step == "Success" else False
        
        return response



    
    def extract_object_position(self, pose:Pose):
        """
        Args:
            msg: Pose, required, the position and orientation of the object
        Returns:
            x: Float, x-position of the object in base_link frame
            y: Float, y-position of the object in base_link frame
            z: Float, z-position of the object in base_link frame
        Other functions:

        """

        x, y, z = pose.position.x, pose.position.y, pose.position.z

        assert x is float, self._logger.error('x was not type float')
        assert y is float, self._logger.error('y was not type float')
        assert z is float, self._logger.error('z was not type float')

        self.get_logger.info('Got the position of the object')

        return x, y, z


    def get_delta_theta_6(self, x, y):
        """
        Args:
            x: Float, required, x-position of the object in base_link frame
            y: Float, required, y-position of the object in base_link frame
        Returns:
            delta_theta_6: Float, degrees that servo 6 has to rotate from its position
        Other functions:

        """

        x_dist = x - self.x_origin_servo5
        y_dist = y - self.y_origin_servo5

        return np.rad2deg(np.arctan2(y_dist/x_dist))
    

    def inverse_kinematics(self, x, y, z):
        """
        Args:
            x: Float, required, x-position of the object in base_link frame
            y: Float, required, y-position of the object in base_link frame
            z: Float, required, z-position of the object in base_link frame
        Returns:
            delta_theta_3: Float, degrees that servo 3 has to rotate from its position
            delta_theta_4: Float, degrees that servo 4 has to rotate from its position
        Other functions:

        """
        # The hypotenuse (rho) from the position of servo 5 to the object position in the xy-plane
        rho_dist = np.sqrt(np.power(x - self.x_origin_servo5, 2) + np.power(y - self.y_origin_servo5, 2)) - self.rho_origin_servo4
        z_dist = z - self.z_origin_servo4

        delta_theta_servo3 = - np.arccos((rho_dist ** 2 + z_dist ** 2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3))
        delta_theta_servo4 = (np.arctan2(z_dist / rho_dist) - 
                              np.arctan2((self.l2 + self.l3 * np.cos(delta_theta_servo3)) / (self.l3 * np.sin(delta_theta_servo3))))
        delta_theta_servo3 = np.rad2deg(delta_theta_servo3)
        delta_theta_servo4 = np.rad2deg(delta_theta_servo4) - (90 - self.theta_servo5)

        return delta_theta_servo3, delta_theta_servo4
        
    
    def publish_angles(self, angles, times=[1000, 1000, 1000, 1000, 1000, 1000]):
        """
        Args:
            angles: List, required, the angles for each servo to be set to
            times:  List, optional, the times for each servo to get to the given angle
        Returns:
        
        Other functions:

        """

        # Initializes the message with required informaiton
        msg = Int16MultiArray()
        msg.layout.dim[0] = {'label':'', 'size': 0, 'stride': 0}
        msg.layout.data_offset = 0

        # Concatenates the angles and times
        msg.data = angles + times

        self.servo_angle_publisher.publish(msg)

        # Makes the code wait until the arm has had the time to move to the given angles
        time.sleep(np.max(times) + 0.5)

def main(args=None):
    rclpy.init()
    pickService = PickService()

    try:
        rclpy.spin(pickService)
    except KeyboardInterrupt:
        pass

    pickService.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()