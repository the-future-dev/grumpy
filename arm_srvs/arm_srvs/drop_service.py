from grumpy_interfaces.srv import PickAndDropObject, PositionRobot

import rclpy
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

        self.box_pose = Pose()  # The position of the object in base_link frame
        self.min_x    = 0.0  # The closest x-position of an object in base_link frame 

        # Create group for the service and subscriber that will run on different threads
        self.service_cb_group    = MutuallyExclusiveCallbackGroup()
        self.subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        self.position_node       = rclpy.create_node('position_node_drop')  # Create a node for the position service

        # Create the drop service
        self.srv = self.create_service(
            PickAndDropObject, 
            '/arm_services/drop_object', 
            self.drop_sequence,
            callback_group=self.service_cb_group
        )

        # Create clients for the used services and wait for them to be available
        self.position_client = self.position_node.create_client(
            PositionRobot, 
            '/arm_services/position_robot'
        )

        while not self.position_client.wait_for_service(timeout_sec=1.0):
            self._logger.info('Waiting for /arm_services/position_robot...')

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

        Returns:
            response: bool, if the drop was successful or not
        Other functions:
            Controlls the drop sequence
            Calls on the positioning service for the robot to put it in the correct position for droping an object
            Calls the publishing function which publishes the servo angles to the arm for each step in the sequence
        """

        step        = "Start"  # The current step in the FSM
        x, y        = 0, 0  # The x and y position of the box
        z           = utils.z_origin_servo4 - 0.05  # The height at which the object is dropped
        end_strings = ["Success", "Failure"]  # The end strings of the FSM
        first_drop  = True  # If it is the first try to drop the object

        while step not in end_strings:
            self._logger.info(f'{step}')  # Log the current step
            times = utils.times  # Set the times to the standard times

            match step:
                case "Start":  # Make sure the arm is in the drive position with an object in the gripper
                    thetas    = utils.drive_thetas
                    next_step = "PositonRobot"  # Next step

                case "PositonRobot":  # Call the position robot service to get to the position and the position of the box
                    thetas         = utils.still_thetas  # Do not move the arm
                    req            = PositionRobot.Request()  # Create a request for the position robot service
                    req.label.data = "BOX"  # Position the robot for the box
                    future         = self.position_client.call_async(req)
                    rclpy.spin_until_future_complete(self.position_node, future)
                    res            = future.result()  # Get the result of the service call

                    if res.success:
                        x, y, _   = utils.extract_object_position(self, res.pose)  # x and y position of the box
                        next_step = "DropPosition"  # Next step
                    else:
                        self._logger.error('Positioning service call failed')
                        next_step = "Failure"  # End the FSM

                case "DropPosition":  # Move servo 6/base to the correct angle
                    thetas = utils.still_thetas.copy()  # Move part of the arm

                    theta_servo6               = utils.get_theta_6(x, y)  # Calculate the new angle for servo 6
                    theta_servo5               = round(utils.theta_servo5_pick * 100)  # Set the angle for servo 5 for inverse kinematics
                    theta_servo3, theta_servo4 = utils.inverse_kinematics(x, y, z)  # Calculate change of the angles for servo 3 and 4

                    thetas[2], thetas[3], thetas[4], thetas[5] = theta_servo3, theta_servo4, theta_servo5, theta_servo6  # Set the angles for the servos
                    next_step                                  = "DropObject"  # Next step

                case "DropObject":  # Drops the object
                    thetas    = utils.still_thetas.copy()  # Move part of the arm
                    thetas[0] = 3000  # Only move and open the gripper

                    if first_drop:
                        first_drop = False  # Tried to drop the object once, should make it on the second try at least
                        next_step  = "DropObject"  # Next step
                    else:
                        next_step  = "DrivePosition"  # Next step

                case "DrivePosition":  # Finish the drop sequence by going back to the initial position
                    thetas    = utils.initial_thetas
                    next_step = "Success"  # End the FSM
            
            utils.check_angles_and_times(self, thetas, times)  # Assert that the angles and times are in the correct format
            
            if self.publish_angles(thetas, times):  # Publish the angles to the arm and check if the arm has moved to the correct angles
                step = next_step

            else:  # If the arm did not move to the correct angles, try to move the arm to the same angles again
                self._logger.error('Move error All: The arm did not move to the correct angles, trying again') 
        
        self._logger.info(f'{step}')
        response.success = True if step == "Success" else False

        self.box_pose = Pose()  # Reset the box pose
        self.min_x    = 0.0  # Reset the minimum x-position of the object
        
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

        # assert isinstance(current_angles, list), self._logger.error('angles is not of type list')
        # assert len(current_angles) == 6, self._logger.error('angles was not of length 6')
        # assert all(isinstance(angle, int) for angle in current_angles), self._logger.error('angles was not of type int')

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

        if angles == utils.still_thetas:  # If the arm is not moving, there is no need to publish the angles
            return True

        msg      = Int16MultiArray()  # Initializes the message
        msg.data = angles + times  # Concatenates the angles and times

        self.servo_angle_publisher.publish(msg)

        time.sleep(np.max(times) / 1000 + 0.75)  # Makes the code wait until the arm has had the time to move to the given angles

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