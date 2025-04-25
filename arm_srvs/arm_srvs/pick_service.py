from grumpy_interfaces.srv import *

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int16MultiArray, Bool
# from sensor_msgs.msg import JointState
import numpy as np
import time
import arm_srvs.utils as utils

class PickService(Node):

    def __init__(self):
        super().__init__('pick_srv')

        # self.current_angles    = utils.initial_thetas  # Keeps track of the angles of the servos published under /servo_pos_publisher
        self.object_in_gripper = False  # Keeps track of if an object is in the gripper or not

        # Create group for the service and subscriber that will run on different threads
        self.service_cb_group    = MutuallyExclusiveCallbackGroup()
        self.subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        self.position_node       = rclpy.create_node('position_node_pick')  # Create a node for the position service
        self.arm_cam_node        = rclpy.create_node('arm_camera_node_pick')  # Create a node for the arm camera service
        
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
            self._logger.info('Waiting for /arm_services/position_robot service...')

        while not self.arm_cam_client.wait_for_service(timeout_sec=1.0):
            self._logger.info('Waiting for /arm_services/arm_camera service...')

        # Create the publishers and subscribers
        self.servo_angle_publisher = self.create_publisher(
            Int16MultiArray,
            '/multi_servo_cmd_sub',
            1
        )
        
        # self.servo_subscriber = self.create_subscription(
        #     JointState,
        #     '/servo_pos_publisher',
        #     self.current_servos,
        #     1,
        #     callback_group=self.subscriber_cb_group
        # )

        self.arm_cam_detection_subscriber = self.create_subscription(
            Bool,
            '/detection_arm_cam/object_in_gripper',
            self.object_in_gripper_callback,
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

        step           = "Start"  # Start step of the FSM
        end_strings    = ["Success", "Failure"]  # End strings for the FSM
        x, y, z        = 0.0, 0.0, -0.005  # The x, y and z position of the object, z is set to -0.005 becasue the object is always on the ground
        label          = ""  # The label of the object to be picked up
        grasp_position = False  # If the arm is in the grasp position or not
        num_failures   = 0  # Number of failures to pick up the object

        while step not in end_strings:
            self._logger.info(f'Pick Service: {step}')  # Log the current step
            times  = utils.times.copy()  # Set the times to the standard times
            thetas = utils.still_thetas.copy()  # Set the angles to the standard angles

            match step:
                case "Start":  # Make sure the arm is in the initial position
                    thetas = utils.initial_thetas
                    step   = "PositionRobot"  # Next step

                case "PositionRobot":  # Call the position robot service to get to the position of the object
                    res = self.position_robot(box=False, backup=False, pos_x=0.0, pos_y=0.0)  # Call the position robot service

                    if res.success:
                        label  = res.label.data  # Get the label of the object
                        thetas = utils.view_thetas_pick # Move arm to view the object
                        step   = "GetPosition"  # Next step
                    else:
                        self._logger.error('Positioning service call failed')
                        thetas = utils.initial_thetas
                        step   = "Failure"  # End the FSM

                case "GetPosition":  # Call the arm camera service to get the position of the object
                    res = self.arm_camera(box=False, grasp=False)  # Call the arm camera service

                    if res.success:
                        x, y, _ = utils.extract_object_position(node=self, pose=res.pose)  # Get the x and y position of the detected object

                        if (x >= 0.22 or x <= 0.15 or y >= 0.05 or y <= -0.15):  # If the object is out of reach in the non grasp position
                            step = "RepositionRobot"  # Reposition the robot

                        else:
                            step = "InverseKinematics"  # Next step

                    else:
                        self._logger.error('Arm camera service call failed')
                        thetas = utils.initial_thetas
                        step   = "Failure"  # End the FSM

                case "RepositionRobot":  # Call the position robot service to get to the position of the object
                    res = self.position_robot(box=False, backup=False, pos_x=x, pos_y=y)  # Call the position robot service

                    if res.success:
                        step = "GetPosition"  # Next step
                        
                    else:
                        self._logger.error('Positioning service call failed')
                        thetas = utils.initial_thetas
                        step   = "Failure"  # End the FSM
                
                case "InverseKinematics":  # Move the arm to the pick up position
                    thetas[5]            = utils.get_theta_6(x=x, y=y)  # Calculate the new angle for servo 6
                    thetas[4]            = round(utils.theta_servo5_pick * 100)  # Set the angle for servo 5 for inverse kinematics
                    thetas[2], thetas[3] = utils.inverse_kinematics(node=self, x=x, y=y, z=z)  # Calculate change of the angles for servo 3 and 4
                    
                    if not grasp_position:
                        grasp_position = True
                        step           = "GetGraspPosition"  # Next step

                    else:
                        step = "GraspObject"  # Next step

                case "GetGraspPosition":  # Call the arm camera service to get the position of the object
                    res = self.arm_camera(box=False, grasp=True)  # Call the arm camera service

                    if res.success:
                        dx, dy, _ = utils.extract_object_position(node=self, pose=res.pose)  # Get the adjustment to the x and y position
                        x, y      = x + dx, y + dy  # Add the adjustment to the position of the object
                        step      = "InverseKinematics"  # Next step

                    else:
                        self._logger.error('Arm camera service call failed in grasp position, trying to grasp anyway')
                        step = "GraspObject"  # Try to grasp the object anyway
                
                case "GraspObject":  # Grasp the object
                    try:
                        thetas[0] = utils.grasp_thetas[label]  # Close the gripper to different degrees depending on the object

                    except KeyError:
                        self._logger.error(f'Unknown object label: {label}')
                        thetas[0] = 10500  # Default value for the gripper
                    
                    times[0] = 2500  # Set the time to slowly close the gripper
                    step     = "PreCheckObject"  # Next step

                case "PreCheckObject":  # Position to check if the object is in the gripper
                    thetas = utils.check_object_thetas  # Make sure can lift the object
                    step   = "CheckObject"  # End the FSM

                case "CheckObject":  # Check if the object is in the gripper
                    time.sleep(0.5)  # Wait for the image from the arm camera to be processed 
                    if self.object_in_gripper:
                        thetas = utils.drive_thetas
                        step   = "Success"  # End the FSM

                    else:
                        self._logger.error('Object not in gripper, trying again')
                        x, y           = 0.0, 0.0
                        num_failures  += 1
                        thetas[0]      = 3000
                        grasp_position = False  # Set the grasp position to False, because the arm has to be in the view position again
                        step           = "ViewPosition"  # Try to view the object again
                        if num_failures > 2:
                            step = 'Failure'
            
            utils.check_angles_and_times(node=self, angles=thetas, times=times)  # Assert that the angles and times are in the correct format and intervals
            self.publish_angles(angles=thetas, times=times)  # Publish the angles to the arm

            # if self.publish_angles(angles=thetas, times=times):  # Publish the angles to the arm and check if the arm has moved to the correct angles
            #     step = next_step

            # elif step == "PickUp":  # To get to the grasping position, the arm has to be in an allowed initial position
            #     self._logger.error('Move error PickUp: The arm did not move to the correct angles, trying again') 
            #     for _ in range(2):  # Try to move the arm to the initial angles a maximum of 2 times
            #         if self.publish_angles(utils.initial_thetas, [2000] * 6):  
            #             break  # Break when the arm has moved to the initial angles and try PickUp again

            # else:  # If the arm did not move to the correct angles, try to move the arm to the same angles again
            #     self._logger.error('Move error Other: The arm did not move to the correct angles, trying again') 
        
        self._logger.info(f'Pick Service: {step}')
        response.success = True if step == "Success" else False
        
        return response
    

    # def current_servos(self, msg:JointState):
    #     """
    #     Args:
    #         msg: JointState, required, information about the servos
    #     Returns:

    #     Other functions:
    #         Listens to what the angles of the servos currently are and sets a self variable to these angles
    #     """

    #     current_angles = msg.position

    #     # assert isinstance(current_angles, list), self._logger.error('angles is not of type list')
    #     # assert len(current_angles) == 6, self._logger.error('angles was not of length 6')
    #     # assert all(isinstance(angle, int) for angle in current_angles), self._logger.error('angles was not of type int')

    #     self.current_angles = current_angles

    
    def object_in_gripper_callback(self, msg:Bool):
        """
        Args:
            msg: Bool, required, if an object is in the gripper or not
        Returns:

        Other functions:
            Listens to the object_in_gripper topic and sets a self variable to this value
        """

        self.object_in_gripper = msg.data  # Set the object_in_gripper variable to the value of the message


    def position_robot(self, box:bool, backup:bool, pos_x:float, pos_y:float):
        """
        Args:
            box   : bool, required, if the object is a box or not
            backup: bool, required, if the robot should back up after dropping the object or not
            pos_x : float, required, x-position of the object in base_link frame
            pos_y : float, required, y-position of the object in base_link frame
        Returns:
            response: PositionRobot.Response, the response of the position robot service
        Other functions:
            Calls the position robot service to get the position of the object
        """
        req                 = PositionRobot.Request()  # Create an request with empty label -> position to pick up an object
        req.box             = box  # Set the box to False, because we are not positioning for a drop
        req.backup          = backup  # Only used to back up the robot after droping
        req.pose.position.x = pos_x  # Set the x position of the object
        req.pose.position.y = pos_y  # Set the y position of the object

        future = self.position_client.call_async(req)
        rclpy.spin_until_future_complete(self.position_node, future)

        return future.result()  # Return the response of the service call
    

    def arm_camera(self, box:bool, grasp:bool):
        """
        Args:
            box  : bool, required, if the object is a box or not
            grasp: bool, required, if the arm is in the grasp position or not
        Returns:
            response: ArmCameraDetection.Response, the response of the arm camera service
        Other functions:
            Calls the arm camera service to get the position of the object
        """

        req       = ArmCameraDetection.Request()  # Create the request, no information is needed
        req.box   = box  # Set the box to False, because we are not positioning for a drop
        req.grasp = grasp  # If we are in the grasp position or not

        future = self.arm_cam_client.call_async(req)
        rclpy.spin_until_future_complete(self.arm_cam_node, future)

        return future.result()  # Return the response of the service call

    
    def publish_angles(self, angles:list, times:list):
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
            return

        msg      = Int16MultiArray()  # Initializes the message
        msg.data = angles + times  # Concatenates the angles and times

        for _ in range(2):
            self.servo_angle_publisher.publish(msg)

            time.sleep(np.max(times) / 1000)  # Makes the code wait until the arm has had the time to move to the given angles

        # self._logger.info(f'current: {self.current_angles}, sent: {angles}')

        # return utils.changed_thetas_correctly(angles, self.current_angles)  # Checks if the arm has moved to the correct angles
        # return True


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