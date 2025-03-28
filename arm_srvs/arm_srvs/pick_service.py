from grumpy_interfaces.srv import PickAndDropObject

import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Pose
import numpy as np
import time
import arm_srvs.utils as utils

class PickService(Node):

    def __init__(self):
        super().__init__('pick_srv')

        # # Origin of servo 5 in base_link frame:
        # self.x_origin_servo5 = -0.00450
        # self.y_origin_servo5 = -0.04750
        # self.z_origin_servo5 =  0.12915
        # self.theta_servo5    =  60

        # # Constants in the robot arm links:
        # self.l1 = 0.10048  # From joint of servo 5 to joint of servo 4:
        # self.l2 = 0.094714  # From joint of servo 4 to joint of servo 3:
        # self.l3 = 0.05071 + 0.11260  # From joint of servo 3 to joint of servo 2 + from joint servo 2 to griping point

        # # Origin of servo 4 in rho+z-plane
        # self.z_origin_servo4   = self.z_origin_servo5 + self.l1 * np.sin(np.deg2rad(90) - np.deg2rad(self.theta_servo5))
        # self.rho_origin_servo4 = self.l1 * np.cos(np.deg2rad(90) - np.deg2rad(self.theta_servo5))
         
        # # Sets angles of the servos for different tasks, as well as time for the arm to move into these positions:
        # self.initial_thetas = [1000, 12000, 12000, 12000, 12000, 12000]  # Arm pointing straight up, used for reset and driving around

        # self.times = [1000, 1000, 1000, 1000, 1000, 1000]  # Standard angle movement times to all positions

        self.current_angles = utils.initial_thetas  # Keeps track of the angles of the servos published under /servo_pos_publisher
        
        # Create the pick service
        self.srv = self.create_service(
            PickAndDropObject, 
            '/arm_services/pick_object', 
            self.pick_up_sequence
        )

        # Create the publisher and subscriber for the angles of the servos
        self.servo_angle_publisher = self.create_publisher(
            Int16MultiArray,
            '/multi_servo_cmd_sub',
            1
        )
        
        self.servo_subscriber = self.create_subscription(
            Int16MultiArray,
            '/servo_pos_publisher',
            self.current_servos,
            1
        )


    def pick_up_sequence(self, request, response):
        """
        Args:
            request: Pose, required, the position and orientation of the object
        Returns:
            response: bool, if the pick up was successful or not
        Other functions:
            Controlls the pick up sequence
            Calls the publishing function which publishes the servo angles to the arm for each step in the sequence
        """

        step = "Start"
        x, y, z = 0, 0, 0
        end_strings = ["Success", "Failure"]

        while step not in end_strings:
            self._logger.info(f'{step}')  # Log the current step
            times = utils.times  # Set the times to the standard times
            match step:
                case "Start":  # Make sure the arm is in the initial position
                    thetas = utils.initial_thetas
                    next_step = "GetPosition"  # Next step

                case "GetPosition":  # Get the position of the object from the request
                    # x, y, z = self.extract_object_position(request.pose)  # Get the position of the object from the request
                    x, y, z = utils.extract_object_position(request.pose)
                    thetas = utils.still_thetas  # Do not move the arm
                    next_step = "RotateBase"  # Next step

                case "RotateBase":  # Move servo 6/base to the correct angle
                    # Calculate the change of the angle for servo 6, then new angle of servo 6, round and convert to int
                    theta_servo6 = round(utils.initial_thetas[5] + self.get_delta_theta_6(x, y) * 100)
                    thetas = utils.still_thetas  
                    thetas[5] = theta_servo6  # Only servo 6 is moved
                    next_step = "PickOrigin"  # Next step

                case "PickOrigin":  # Move the arm to the position from were the inverse kinematics are calculated
                    thetas = utils.still_thetas
                    thetas[4] = round(utils.theta_servo5 * 100)  # Set the angle for servo 5 for inverse kinematics
                    next_step = "InverseKinematics"  # Next step

                case "InverseKinematics":  # Move the arm to the grasp position calculated by the inverse kinematics
                    delta_theta_servo3, delta_theta_servo4 = self.inverse_kinematics(x, y, z)  # Calculate change of the angles for servo 3 and 4
                    theta_servo3 = round(utils.initial_thetas[2] + delta_theta_servo3 * 100)  # New angle of servo 3, round and convert to int
                    theta_servo4 = round(utils.initial_thetas[3] + delta_theta_servo4 * 100)  # New angle of servo 4, round and convert to int
                    # self._logger.info(f'{theta_servo3, theta_servo4}')
                    thetas = utils.still_thetas
                    thetas[2], thetas[3] = theta_servo3, theta_servo4  # Only servo 3 and 4 are moved
                    next_step = "GraspObject"  # Next step
                
                case "GraspObject":  # Grasp the object
                    thetas = utils.still_thetas
                    thetas[0] = 10000  # Close the gripper
                    times[0] = 3000  # Set the time to slowly close the gripper
                    next_step = "DrivePosition"  # Next step

                case "DrivePosition":  # Finish the pick up sequence by going back to the initial position, but not for the gripper
                    thetas = utils.drive_thetas
                    times = [2 * t for t in times]  # Longer time might be needed to move the arm back a far distance
                    next_step = "Success"  # End the FSM
            
            self.check_angles_and_times(thetas, times)  # Assert that the angles and times are in the correct format
            
            if self.publish_angles(thetas, times):  # Publish the angles to the arm and check if the arm has moved to the correct angles
                step = next_step
            else:
                step = "Failure"
        
        self._logger.info(f'{step}')
        response.success = True if step == "Success" else False
        
        return response
    

    def current_servos(self, msg:Int16MultiArray):
        """
        Args:
            msg: Int16MultiArray, required, the angles of the servos
        Returns:

        Other functions:
            Listens to what the angles of the servos currently are and sets a self variable to these angles
        """

        current_angles = msg.data

        assert isinstance(current_angles, list), self._logger.error('angles is not of type list')
        assert len(current_angles) == 6, self._logger.error('angles was not of length 6')
        assert all(isinstance(angle, int) for angle in current_angles), self._logger.error('angles was not of type int')

        self.current_angles = current_angles


    def extract_object_position(self, pose:Pose):
        """
        Args:
            msg: Pose, required, the position and orientation of the object
        Returns:
            x: float, x-position of the object in base_link frame
            y: float, y-position of the object in base_link frame
            z: float, z-position of the object in base_link frame
        Other functions:

        """

        x, y, z = pose.position.x, pose.position.y, pose.position.z

        assert isinstance(x, float), self._logger.error('x was not type float')
        assert isinstance(x, float), self._logger.error('y was not type float')
        assert isinstance(x, float), self._logger.error('z was not type float')

        self._logger.info('Got the position of the object')

        return x, y, z


    def get_delta_theta_6(self, x, y):
        """
        Args:
            x: float, required, x-position of the object in base_link frame
            y: float, required, y-position of the object in base_link frame
        Returns:
            delta_theta_6: float, degrees that servo 6 has to rotate from its position
        Other functions:

        """

        x_dist = x - utils.x_origin_servo5
        y_dist = y - utils.y_origin_servo5

        # Calculate the angle for servo 6 in radians and convert to degrees
        return np.rad2deg(np.arctan2(y_dist, x_dist))
    

    def inverse_kinematics(self, x, y, z):
        """
        Args:
            x: float, required, x-position of the object in base_link frame
            y: float, required, y-position of the object in base_link frame
            z: float, required, z-position of the object in base_link frame
        Returns:
            delta_theta_3: float, degrees that servo 3 has to rotate from its position
            delta_theta_4: float, degrees that servo 4 has to rotate from its position
        Other functions:

        """
        # The hypotenuse (rho) from the origin of servo 5 to the object position in the xy-plane minus the distance servo 4 has already moved
        rho_dist = np.sqrt(np.power(x - utils.x_origin_servo5, 2) + np.power(y - utils.y_origin_servo5, 2)) - utils.rho_origin_servo4
        z_dist = z - utils.z_origin_servo4  # The z position of the object minus the z position of servo 4

        # self._logger.info(f'rho {rho_dist}, z {z_dist}')

        # Calculate the angles for servo 3 and 4 in radians
        cos_d_t_servo3 = (rho_dist ** 2 + z_dist ** 2 - utils.l2 ** 2 - utils.l3 ** 2) / (2 * utils.l2 * utils.l3)
        delta_theta_servo3 = - np.arctan2(np.sqrt(1 - cos_d_t_servo3 ** 2), cos_d_t_servo3)
        delta_theta_servo4 = (np.arctan2(z_dist, rho_dist) - 
                              np.arctan2(utils.l3 * np.sin(delta_theta_servo3), utils.l2 + (utils.l3 * np.cos(delta_theta_servo3))))
        
        # self._logger.info(f'servo 3 {np.rad2deg(delta_theta_servo3)}, servo 4 { (- np.rad2deg(delta_theta_servo4)) + (90 - self.theta_servo5)}')
        
        # Convert the angles to degrees and adjust for the initial angle of servo 5
        delta_theta_servo3 = np.rad2deg(delta_theta_servo3)
        delta_theta_servo4 = (- np.rad2deg(delta_theta_servo4)) + (90 - utils.theta_servo5)

        return delta_theta_servo3, delta_theta_servo4

    
    def check_angles_and_times(self, angles, times):
        """
        Args:
            angles: list, required, the angles for each servo to be set to
            times:  list, required, the times for each servo to get to the given angle
        Returns:
            
        Other functions:
            Raises error if the angles and times are not in the correct format, length or interval
        """

        assert isinstance(angles, list), self._logger.error('angles is not of type list')
        assert isinstance(times, list), self._logger.error('times is not of type list')
        assert len(angles) == 6, self._logger.error('angles was not of length 6')
        assert len(times) == 6, self._logger.error('times was not of length 6')
        assert all(isinstance(angle, int) for angle in angles), self._logger.error('angles was not of type int')
        assert all(isinstance(time, int) for time in times), self._logger.error('times was not of type int')
        assert all(1000 <= time <= 5000 for time in times), self._logger.error('times was not within the interval [1000, 5000]')
        assert (0 <= angles[0] <= 11000) or (angles[0] == -1), self._logger.error(f'servo 1 was not within the interval [0, 11000] or -1, got {angles[0]}')
        assert (0 <= angles[1] <= 24000) or (angles[1] == -1), self._logger.error(f'servo 2 was not within the interval [0, 24000] or -1, got {angles[1]}')
        assert (2500 <= angles[2] <= 21000) or (angles[2] == -1), self._logger.error(f'servo 3 was not within the interval [2500, 21000] or -1, got {angles[2]}')
        assert (3000 <= angles[3] <= 21500) or (angles[3] == -1), self._logger.error(f'servo 4 was not within the interval [3000, 21500] or -1, got {angles[3]}')
        assert (6000 <= angles[4] <= 18000) or (angles[4] == -1), self._logger.error(f'servo 5 was not within the interval [6000, 18000] or -1, got {angles[4]}')
        assert (0 <= angles[5] <= 20000) or (angles[5] == -1), self._logger.error(f'servo 6 was not within the interval [0, 20000] or -1, got {angles[5]}')

        self._logger.info('Checked the angles and times')
        
    
    def publish_angles(self, angles, times):
        """
        Args:
            angles: list, required, the angles for each servo to be set to
            times:  list, required, the times for each servo to get to the given angle
        Returns:
            bool, if the arm has moved to the correct angles
        Other functions:
            Publishes the angles of the servos to the arm in the correct format
        """

        # Initializes the message with required informaiton
        msg = Int16MultiArray()
        # msg.layout.dim[0] = {'label':'', 'size': 0, 'stride': 0}
        # msg.layout.data_offset = 0

        msg.data = angles + times  # Concatenates the angles and times

        self.servo_angle_publisher.publish(msg)

        time.sleep(np.max(times) / 1000 + 0.5)  # Makes the code wait until the arm has had the time to move to the given angles

        return self.current_angles == angles  # Checks if the arm has moved to the correct angles

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