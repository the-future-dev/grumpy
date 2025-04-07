from grumpy_interfaces.srv import PickAndDropObject

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np
import time
import arm_srvs.utils as utils

class DropService(Node):

    def __init__(self):
        super().__init__('drop_srv')

        self.current_angles = utils.initial_thetas  # Keeps track of the angles of the servos published under /servo_pos_publisher

        # Create group for the service and subscriber that will run on different threads
        self.service_cb_group    = MutuallyExclusiveCallbackGroup()
        self.subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        # Create the drop service
        self.srv = self.create_service(
            PickAndDropObject, 
            '/arm_services/drop_object', 
            self.drop_sequence,
            callback_group=self.service_cb_group
        )

        # Create the publisher and subscriber for the angles of the servos
        self.servo_angle_publisher = self.create_publisher(
            Int16MultiArray,
            '/multi_servo_cmd_sub',
            1
        )
        
        self.servo_subscriber = self.create_subscription(
            JointState,
            '/servo_pos_publisher',
            self.current_servos,
            1,
            callback_group=self.subscriber_cb_group
        )

    
    def drop_sequence(self, request, response):
        """
        Args:
            request: Pose, required, the position and orientation of the box
        Returns:
            response: bool, if the drop was successful or not
        Other functions:
            Controlls the drop sequence
            Calls the publishing function which publishes the servo angles to the arm for each step in the sequence
        """

        step        = "Start"
        x, y, z     = 0, 0, 0
        end_strings = ["Success", "Failure"]

        while step not in end_strings:
            self._logger.info(f'{step}')  # Log the current step
            times = utils.times  # Set the times to the standard times
            match step:
                case "Start":  # Make sure the arm is in the initial position but does not drop the object
                    thetas    = utils.initial_thetas.copy()  # Copy the initial thetas
                    thetas[0] = -1  # Make sure the gripper does not move
                    next_step = "GetPosition"  # Next step

                case "GetPosition":  # Get the position of the box from the request
                    x, y, _   = utils.extract_object_position(self, request.pose)  # Get the position of the box from the request
                    thetas    = utils.still_thetas  # Do not move the arm
                    next_step = "RotateBase"  # Next step

                case "RotateBase":  # Move servo 6/base to the correct angle
                    # Calculate the change of the angle for servo 6, then new angle of servo 6, round and convert to int
                    theta_servo6 = round(utils.initial_thetas[5] + utils.get_delta_theta_6(x, y) * 100)
                    thetas       = utils.still_thetas.copy()  # Move part of arm
                    thetas[5]    = theta_servo6  # Only servo 6 is moved
                    next_step    = "DropAngles"  # Next step

                case "DropAngles":  # Sets the angles for the arm to drop the object
                    thetas    = utils.drop_thetas
                    next_step = "DropObject"  # Next step

                case "DropObject":  # Drops the object
                    thetas    = utils.still_thetas.copy()  # Move part of arm
                    thetas[0] = 1000  # Only move and open the gripper
                    next_step = "DrivePosition"  # Next step

                case "DrivePosition":  # Finish the drop sequence by going back to the initial position
                    thetas    = utils.initial_thetas
                    next_step = "Success"  # End the FSM
            
            utils.check_angles_and_times(self, thetas, times)  # Assert that the angles and times are in the correct format
            
            if self.publish_angles(thetas, times):  # Publish the angles to the arm and check if the arm has moved to the correct angles
                step = next_step
            else:
                step = "Failure"
        
        self._logger.info(f'{step}')
        response.success = True if step == "Success" else False
        
        return response


    def current_servos(self, msg:JointState):
        """
        Args:
            msg: JointState, required, information about the servos positions, velocities and efforts
        Returns:

        Other functions:
            Listens to what the angles of the servos currently are and sets a self variable to these angles
        """

        current_angles = msg.position

        assert isinstance(current_angles, list), self._logger.error('angles is not of type list')
        assert len(current_angles) == 6, self._logger.error('angles was not of length 6')
        assert all(isinstance(angle, int) for angle in current_angles), self._logger.error('angles was not of type int')

        self.current_angles = current_angles

    
    def publish_angles(self, angles, times):
        """
        Args:
            angles: list, required, the angles for each servo to be set to
            times : list, required, the times for each servo to get to the given angle
        Returns:
             : bool, if the arm has moved to the correct angles
        Other functions:
            Publishes the angles of the servos to the arm in the correct format
        """

        msg      = Int16MultiArray()  # Initializes the message
        msg.data = angles + times  # Concatenates the angles and times

        self.servo_angle_publisher.publish(msg)

        time.sleep(np.max(times) / 1000 + 0.5)  # Makes the code wait until the arm has had the time to move to the given angles

        return utils.changed_thetas_correctly(angles, self.current_angles)  # Checks if the arm has moved to the correct angles

def main(args=None):
    rclpy.init()
    dropServie = DropService()

    # Use MultiThreadedExecutor to allow concurrent callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(dropServie)

    try:
        executor.spin()
    except KeyboardInterrupt:
        dropServie.destroy_node()
    finally:
        dropServie.destroy_node()
        rclpy.shutdown()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()