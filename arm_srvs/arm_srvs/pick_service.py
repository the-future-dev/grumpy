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
                    thetas    = utils.initial_thetas
                    next_step = "GetPosition"  # Next step

                case "GetPosition":  # Get the position of the object from the request
                    x, y, z   = utils.extract_object_position(self, request.pose)
                    thetas    = utils.still_thetas  # Do not move the arm
                    next_step = "RotateBase"  # Next step

                case "RotateBase":  # Move servo 6/base to the correct angle
                    # Calculate the change of the angle for servo 6, then new angle of servo 6, round and convert to int
                    theta_servo6 = round(utils.initial_thetas[5] + utils.get_delta_theta_6(x, y) * 100)
                    thetas       = utils.still_thetas.copy()  # Move part of arm
                    thetas[5]    = theta_servo6  # Only servo 6 is moved
                    next_step    = "PickOrigin"  # Next step

                case "PickOrigin":  # Move the arm to the position from were the inverse kinematics are calculated
                    thetas    = utils.still_thetas.copy()  # Move part of arm
                    thetas[4] = round(utils.theta_servo5_pick * 100)  # Set the angle for servo 5 for inverse kinematics
                    next_step = "InverseKinematics"  # Next step

                case "InverseKinematics":  # Move the arm to the grasp position calculated by the inverse kinematics
                    delta_theta_servo3, delta_theta_servo4 = self.inverse_kinematics(x, y, z)  # Calculate change of the angles for servo 3 and 4
                    theta_servo3 = round(utils.initial_thetas[2] + delta_theta_servo3 * 100)  # New angle of servo 3, round and convert to int
                    theta_servo4 = round(utils.initial_thetas[3] + delta_theta_servo4 * 100)  # New angle of servo 4, round and convert to int
                    thetas       = utils.still_thetas.copy()  # Move part of arm
                    thetas[2], thetas[3] = theta_servo3, theta_servo4  # Only servo 3 and 4 are moved
                    next_step    = "GraspObject"  # Next step
                
                case "GraspObject":  # Grasp the object
                    thetas    = utils.still_thetas.copy()  # Move part of arm
                    thetas[0] = 10000  # Close the gripper
                    times[0]  = 3000  # Set the time to slowly close the gripper
                    next_step = "DrivePosition"  # Next step

                case "DrivePosition":  # Finish the pick up sequence by going back to the initial position, but not for the gripper
                    thetas    = utils.drive_thetas
                    times     = [2000] * 6  # Longer time might be needed to move the arm back a far distance
                    next_step = "Success"  # End the FSM
            
            utils.check_angles_and_times(self, thetas, times)  # Assert that the angles and times are in the correct format
            
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

        # Calculate the angles for servo 3 and 4 in radians
        cos_d_t_servo3 = (rho_dist ** 2 + z_dist ** 2 - utils.l_4_3 ** 2 - utils.l_3_2_ee ** 2) / (2 * utils.l_4_3 * utils.l_3_2_ee)
        delta_theta_servo3 = - np.arctan2(np.sqrt(1 - cos_d_t_servo3 ** 2), cos_d_t_servo3)
        delta_theta_servo4 = (np.arctan2(z_dist, rho_dist) - 
                              np.arctan2(utils.l_3_2_ee * np.sin(delta_theta_servo3), utils.l_4_3 + (utils.l_3_2_ee * np.cos(delta_theta_servo3))))
        
        # Convert the angles to degrees and adjust for the initial angle of servo 5
        delta_theta_servo3 = np.rad2deg(delta_theta_servo3)
        delta_theta_servo4 = (- np.rad2deg(delta_theta_servo4)) + (90 - utils.theta_servo5_pick)

        return delta_theta_servo3, delta_theta_servo4
        
    
    def publish_angles(self, angles, times):
        """
        Args:
            angles: list, required, the angles for each servo to be set to
            times : list, required, the times for each servo to get to the given angle
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

        # return utils.changed_thetas_correctly(angles, self.current_angles)  # Checks if the arm has moved to the correct angles
        return True


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