from grumpy_interfaces.srv import *

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int16MultiArray, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np
import time
import arm_srvs.utils as utils

class DropService(Node):

    def __init__(self):
        super().__init__('drop_srv')

        self.current_angles = utils.initial_thetas  # Keeps track of the angles of the servos published under /servo_pos_publisher
        self.object_in_gripper = False  # Keeps track of the object in the gripper

        # Create group for the service and subscriber that will run on different threads
        self.service_cb_group    = MutuallyExclusiveCallbackGroup()
        self.subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        self.position_node       = rclpy.create_node('position_node_drop')  # Create a node for the position service
        self.arm_cam_node        = rclpy.create_node('arm_camera_node_drop')  # Create a node for the arm camera service

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

        self.arm_cam_client = self.arm_cam_node.create_client(
            ArmCameraDetection, 
            '/arm_services/arm_camera'
        )

        while not self.position_client.wait_for_service(timeout_sec=1.0):
            self._logger.info('Waiting for /arm_services/position_robot service...')

        while not self.arm_cam_client.wait_for_service(timeout_sec=1.0):
            self._logger.info('Waiting for /arm_services/arm_camera service...')

        # Create the publishers and subscribers
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

        self.arm_cam_detection_subscriber = self.create_subscription(
            Bool,
            '/detection_arm_cam/object_in_gripper',
            self.object_in_gripper_callback,
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

        while step not in end_strings:
            self._logger.info(f'{step}')  # Log the current step
            times  = utils.times  # Set the times to the standard times
            thetas = utils.still_thetas.copy()  # Do not move the arm

            match step:
                case "Start":  # Make sure the arm is in the drive position with an object in the gripper
                    thetas    = utils.drive_thetas
                    next_step = "PositonRobot"  # Next step

                case "PositonRobot":  # Call the position robot service to get to the position and the position of the box
                    req            = PositionRobot.Request()  # Create a request for the position robot service
                    req.box        = True  # Position the robot for the box
                    future         = self.position_client.call_async(req)
                    rclpy.spin_until_future_complete(self.position_node, future)
                    res            = future.result()  # Get the result of the service call

                    if res.success:
                        next_step = "ViewPosition"  # Next step
                    else:
                        self._logger.error('Positioning service call failed')
                        next_step = "Failure"  # End the FSM

                case "ViewPosition":  # Get the position of the object from the arm camera
                    thetas    = utils.view_thetas  # Move arm to view the object
                    next_step = "GetPosition"  # Next step

                case "GetPosition":  # Call the arm camera service to get the position of the object
                    req       = ArmCameraDetection.Request()  # Create the request, no information is needed
                    req.box   = True  # Set the box to True, because we are positioning for a drop
                    req.grasp = False  # Set the grasp to False, because we are not grasping an object
                    future    = self.arm_cam_client.call_async(req)
                    rclpy.spin_until_future_complete(self.arm_cam_node, future)
                    res       = future.result()  # The response of the service call

                    if res.success:
                        x, y, _   = utils.extract_object_position(self, res.pose)  # Get the x and y position of the detected box
                        next_step = "DropPosition"  # Next step
                    else:
                        self._logger.error('Arm camera service call failed')
                        thetas    = utils.initial_thetas  # Move the arm to the initial position
                        next_step = "Failure"  # End the FSM

                case "DropPosition":  # Move arm to correct position to drop the object
                    theta_servo6               = utils.get_theta_6(x, y)  # Calculate the new angle for servo 6
                    theta_servo5               = round(utils.theta_servo5_pick * 100)  # Set the angle for servo 5 for inverse kinematics
                    theta_servo3, theta_servo4 = utils.inverse_kinematics(x, y, z)  # Calculate change of the angles for servo 3 and 4

                    thetas[2], thetas[3], thetas[4], thetas[5] = theta_servo3, theta_servo4, theta_servo5, theta_servo6  # Set the angles for the servos
                    next_step                                  = "DropObject"  # Next step

                case "DropObject":  # Drops the object
                    thetas[0] = 3000  # Only move and open the gripper
                    next_step = "CheckPosition"  # Next step

                case "CheckPosition":  # Finish the drop sequence by going back to the initial position
                    thetas[4] = 12000
                    next_step = "CheckObject"  # End the FSM

                case "CheckObject":  # Check if the object is in the gripper
                    if not self.object_in_gripper:
                        thetas    = utils.initial_thetas
                        next_step = "Success"  # End the FSM
                    else:
                        self._logger.error('Object in gripper, trying again')
                        next_step = "DropPosition"  # Try to view the object again
            
            utils.check_angles_and_times(self, thetas, times)  # Assert that the angles and times are in the correct format
            
            if self.publish_angles(thetas, times):  # Publish the angles to the arm and check if the arm has moved to the correct angles
                step = next_step

            else:  # If the arm did not move to the correct angles, try to move the arm to the same angles again
                self._logger.error('Move error All: The arm did not move to the correct angles, trying again') 
        
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

        # assert isinstance(current_angles, list), self._logger.error('angles is not of type list')
        # assert len(current_angles) == 6, self._logger.error('angles was not of length 6')
        # assert all(isinstance(angle, int) for angle in current_angles), self._logger.error('angles was not of type int')

        self.current_angles = current_angles

    
    def object_in_gripper_callback(self, msg:Bool):
        """
        Args:
            msg: Bool, required, if an object is in the gripper or not
        Returns:

        Other functions:
            Listens to the object_in_gripper topic and sets a self variable to this value
        """

        self.object_in_gripper = msg.data  # Set the object_in_gripper variable to the value of the message

    
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

        if angles == utils.still_thetas:  # If the arm is not moving, there is no need to publish the angles
            return True

        msg      = Int16MultiArray()  # Initializes the message
        msg.data = angles + times  # Concatenates the angles and times

        for _ in range(2):
            self.servo_angle_publisher.publish(msg)

            time.sleep(np.max(times) / 1000)  # Makes the code wait until the arm has had the time to move to the given angles

        # self._logger.info(f'current: {self.current_angles}, sent: {angles}')

        # return utils.changed_thetas_correctly(angles, self.current_angles)  # Checks if the arm has moved to the correct angles
        return True

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