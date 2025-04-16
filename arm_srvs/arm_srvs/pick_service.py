from grumpy_interfaces.srv import *

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import time
import arm_srvs.utils as utils

class PickService(Node):

    def __init__(self):
        super().__init__('pick_srv')

        self.current_angles = utils.initial_thetas  # Keeps track of the angles of the servos published under /servo_pos_publisher

        # Create group for the service and subscriber that will run on different threads
        self.service_cb_group    = MutuallyExclusiveCallbackGroup()
        self.subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        self.position_node       = rclpy.create_node('position_node_pick')  # Create a node for the position service
        self.arm_cam_node        = rclpy.create_node('arm_camera_node')  # Create a node for the arm camera service
        
        # Create the pick service
        self.srv = self.create_service(
            PickAndDropObject, 
            '/arm_services/pick_object', 
            self.pick_up_sequence,
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
            self._logger.info('Waiting for /arm_services/position_robot...')

        while not self.arm_cam_client.wait_for_service(timeout_sec=1.0):
            self._logger.info('Waiting for /arm_services/arm_camera service...')

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


    def pick_up_sequence(self, request, response):
        """
        Args:

        Returns:
            response: bool, if the pick up was successful or not
        Other functions:
            Controlls the pick up sequence
            Calls on the positioning service for the robot to put it in the correct position for picking up an object
            Calls on the arm camera service to get the position of the object
            Calls the publishing function which publishes the servo angles to the arm for each step in the sequence
        """

        step        = "Start"  # Start step of the FSM
        end_strings = ["Success", "Failure"]  # End strings for the FSM
        x, y, z     = 0, 0, -0.005  # The x, y and z position of the object, z is set to -0.02 becasue the object is always on the ground
        label       = ""  # The label of the object to be picked up
        first_grasp = True  # If it is the first try to grasp the object

        while step not in end_strings:
            self._logger.info(f'Pick Service: {step}')  # Log the current step
            times = utils.times  # Set the times to the standard times

            match step:
                case "Start":  # Make sure the arm is in the initial position
                    thetas    = utils.initial_thetas
                    next_step = "PositonRobot"  # Next step

                case "PositonRobot":  # Call the position robot service to get to the position of the object
                    thetas = utils.still_thetas  # Do not move the arm
                    req    = PositionRobot.Request()  # Create the an empty request, robot will position to pick up an object
                    future = self.position_client.call_async(req)
                    rclpy.spin_until_future_complete(self.position_node, future)
                    res    = future.result()  # The response of the service call

                    if res.success:
                        label     = res.label.data  # Get the label of the object
                        next_step = "ViewPosition"  # Next step
                    else:
                        self._logger.error('Positioning service call failed')
                        next_step = "Failure"  # End the FSM

                case "ViewPosition":  # Get the position of the object from the arm camera
                    thetas    = utils.view_thetas  # Move arm to view the object
                    next_step = "GetPosition"  # Next step

                case "GetPosition":  # Call the arm camera service to get the position of the object
                    thetas = utils.still_thetas  # Do not move the arm
                    req    = ArmCameraDetection.Request()  # Create the request, no information is needed
                    future = self.arm_cam_client.call_async(req)
                    rclpy.spin_until_future_complete(self.arm_cam_node, future)
                    res    = future.result()  # The response of the service call

                    if res.success:
                        x, y, _   = utils.extract_object_position(self, res.pose)  # Get the x and y position of the detected object
                        next_step = "PickUp"  # Next step
                    else:
                        self._logger.error('Arm camera service call failed')
                        next_step = "Failure"  # End the FSM

                case "PickUp":  # Move the arm to the pick up position
                    thetas = utils.still_thetas.copy()  # Move part of the arm

                    theta_servo6               = utils.get_theta_6(x, y)  # Calculate the new angle for servo 6
                    theta_servo5               = round(utils.theta_servo5_pick * 100)  # Set the angle for servo 5 for inverse kinematics
                    theta_servo3, theta_servo4 = utils.inverse_kinematics(x, y, z)  # Calculate change of the angles for servo 3 and 4

                    thetas[2], thetas[3], thetas[4], thetas[5] = theta_servo3, theta_servo4, theta_servo5, theta_servo6  # Set the angles for the servos
                    times[2], times[3], times[4], times[5]     = 2000, 2000, 1000, 1000  # Set the time for the servos to move to the new angles
                    next_step                                  = "GraspObject"  # Next step
                
                case "GraspObject":  # Grasp the object
                    thetas    = utils.still_thetas.copy()  # Move part of the arm
                    # Close the gripper to different degrees depending on the object
                    if label == "CUBE":
                        thetas[0] = 10500
                    elif label == "SPHERE":
                        thetas[0] = 9500
                    elif label == "PUPPY":
                        thetas[0] = 13000
                    else:
                        self._logger.error(f'Unknown object label: {label}')
                        thetas[0] = 10500  # Default value for the gripper

                    if first_grasp:
                        first_grasp = False  # Tried to pick the object once, should make it on the second try at least
                        next_step  = "GraspObject"  # Next step
                    else:
                        next_step  = "DrivePosition"  # Next step
                    
                    times[0]  = 3000  # Set the time to slowly close the gripper

                case "DrivePosition":  # Finish the pick up sequence by going back to the initial position, but not for the gripper
                    thetas    = utils.drive_thetas
                    times     = [2000] * 6  # Longer time might be needed to move the arm back a far distance
                    next_step = "CheckObject"  # End the FSM

                case "CheckObject":  # Check if the object is in the gripper
                    thetas    = utils.still_thetas  # Do not move the arm

                    ################  TODO: Create a topic that publishes that an object is in the gripper  ################

                    next_step = "Success"  # End the FSM
            
            utils.check_angles_and_times(self, thetas, times)  # Assert that the angles and times are in the correct format and intervals
            
            if self.publish_angles(thetas, times):  # Publish the angles to the arm and check if the arm has moved to the correct angles
                step = next_step

            elif step == "PickUp":  # To get to the grasping position, the arm has to be in an allowed initial position
                self._logger.error('Move error PickUp: The arm did not move to the correct angles, trying again') 
                for _ in range(2):  # Try to move the arm to the initial angles a maximum of 2 times
                    if self.publish_angles(utils.initial_thetas, [2000] * 6):  
                        break  # Break when the arm has moved to the initial angles and try PickUp again

            else:  # If the arm did not move to the correct angles, try to move the arm to the same angles again
                self._logger.error('Move error Other: The arm did not move to the correct angles, trying again') 
        
        self._logger.info(f'Pick Service: {step}')
        response.success = True if step == "Success" else False
        
        return response
    

    def current_servos(self, msg:JointState):
        """
        Args:
            msg: JointState, required, information about the servos
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

            time.sleep(np.max(times) / 1000 + 0.75)  # Makes the code wait until the arm has had the time to move to the given angles

        # self._logger.info(f'current: {self.current_angles}, sent: {angles}')

        # return utils.changed_thetas_correctly(angles, self.current_angles)  # Checks if the arm has moved to the correct angles
        return True


def main(args=None):
    rclpy.init()
    pickService = PickService()

    # Use MultiThreadedExecutor to allow concurrent callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pickService)

    try:
        executor.spin()
    # except KeyboardInterrupt:
    #     pass
    finally:
        pickService.destroy_node()
        rclpy.shutdown()

    # rclpy.shutdown()

if __name__ == '__main__':
    main()