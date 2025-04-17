from grumpy_interfaces.srv import PositionRobot
from grumpy_interfaces.msg import ObjectDetection1D

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from robp_interfaces.msg import DutyCycles
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
import time

class PositioningService(Node):

    def __init__(self):
        super().__init__('positioning_srv')

        self.service_cb_group    = MutuallyExclusiveCallbackGroup()
        self.subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        # Set speeds for the robot to move
        self.vel_forward       = 0.07  # 0.03 before
        self.vel_rotate        = 0.06  # 0.02 before
        self.correct_right     = 1.05  # Correction of the right wheel
        self.rotation_per_turn = 6.00  # Degrees of rotation per turn msg with vel_rotate = 0.06 and correct_right = 1.08
        self.movement_per_forw = 0.026  # Moved distance per forward msg with vel_forward = 0.07 and correct_right = 1.08

        self.object_pose  = Pose()  # The position of the object in base_link frame
        self.object_label = ""  # The label of the object
        self.object_found = False  # Flag to check if the object was found
        self.min_x        = 0.0  # The closest x-position of an object in base_link frame
        self.look_for_box = False  # Flag to check if the robot should only look for a box

        # Target distance-parameters to the goal object/box
        self.unrealistic_x =  0.30
        self.x_stop_pick   =  0.20  # The x-position where the robot should stop when driving with the RGB-D camera to an object
        self.x_stop_drop   =  0.20  # The x-position where the robot should stop when driving with the RGB-D camera to a box
        self.y_offset      = -0.05  # The y-position where the robot should stop rotating with the RGB-D camera
        self.y_tol         =  0.02  # The tolerance for the y-position when driving with the RGB-D camera
        
        # Create the positioning service
        self.srv = self.create_service(
            PositionRobot, 
            '/arm_services/position_robot', 
            self.positioning_sequence,
            callback_group=self.service_cb_group
        )

        # Create the publisher to the wheel motors
        self.motor_publisher = self.create_publisher(
            DutyCycles,
            'motor/duty_cycles', 
            10
        )
        
        # Create the subscriber to the object detection
        self.servo_subscriber = self.create_subscription(
            ObjectDetection1D,
            '/perception/object_poses',
            self.get_object_pose,
            10,
            callback_group=self.subscriber_cb_group
        )

    
    def positioning_sequence(self, request, response):
        """
        Args:
            request.label: String, required, the label of the goal object
        Returns:
            response: bool, if the positioning was successful or not
        Other functions:
            Controlls the positioning sequence
            Calls the publishing function which publishes the velocities to the wheel motors for each step in the sequence
        """

        # step        = "Start"  # The current step in the FSM
        # end_strings = ["Success", "Failure"]  # The end strings of the FSM

        if request.label.data == "BOX":
            self._logger.info(f'positioning_sequence: Looking for a box')
            self.look_for_box = True
        else:
            self._logger.info(f'positioning_sequence: Looking for an object')
            self.look_for_box = False

        self._logger.info(f'Getting the position of the {'BOX' if self.look_for_box else 'OBJECT'} from perception.')
        time.sleep(0.75)  # Sleep for 0.5 second to give the robot time to move and object detection time to update

        while not self.object_found:
            self.publish_robot_movement(-1.0, 0.0)
        
        goal_x = self.object_pose.position.x  # The x-position of the object/box in base_link frame from perception
        goal_y = self.object_pose.position.y  # The y-position of the object/box in base_link frame from perception

        self._logger.info(f'Found at x: {goal_x}, y: {goal_y}')
        self.publish_robot_movement(goal_x, goal_y)  # Drive the robot to the object/box

        # while step not in end_strings:

        #     time.sleep(0.75)  # Sleep for 0.5 second to give the robot time to move and object detection time to update
        #     self._logger.info(f'{step}')  # Log the current step

        #     goal_x = self.object_pose.position.x  # The x-position of the object in base_link frame from perception
        #     goal_y = self.object_pose.position.y  # The y-position of the object in base_link frame from perception
        #     # self._logger.info(f'x: {goal_x}, y: {goal_y}')

        #     match step:
        #         case "Start":  # Check if an object was found
        #             if goal_x == 0.0 and goal_y == 0.0:  # If no object/box has been seen
        #                 self.publish_robot_movement(-1.0, 0.0)  # Turn the robot to find an object/box
        #                 if self.object_found:
        #                     self._logger.info(f'positioning_sequence: in Start --> DriveRobotWithRGB-D')
        #                 else:
        #                     self._logger.error(f'positioning_sequence: Could not find an object/box')
        #                     step = "Failure"  # End the FSM
        #                     break
                    
        #             if request.label.data == "BOX":
        #                 step = "BoxDriveRobotWithRGB-D"  # Next step
        #             else:
        #                 step = "DriveRobotWithRGB-D"  # Next step

        #         case "BoxDriveRobotWithRGB-D":  # Drive the robot using the RGB-D camera as feedback to a box
        #             if goal_x <= self.x_stop_drop:
        #                 self._logger.info(f'positioning_sequence: in B --> Success')
        #                 step = "Success"  # End the FSM
        #             else:
        #                 self._logger.info(f'positioning_sequence: in B --> BoxDriveRobotWithRGB-D')
        #                 self.publish_robot_movement(goal_x, goal_y)  # Drive the robot
        #                 step = "BoxDriveRobotWithRGB-D"  # Next step

        #         case "DriveRobotWithRGB-D":  # Drive the robot using the RGB-D camera as feedback
        #             if goal_x <= self.x_stop_pick:  # End condition for driving the robot using the RGB-D camera
        #                 self._logger.info(f'positioning_sequence: in Object --> DriveRobotWithoutRGB-D')
        #                 step = "DriveRobotWithoutRGB-D"  # Next step
        #             else:
        #                 self._logger.info(f'positioning_sequence: in Object --> DriveRobotWithRGB-D')
        #                 self.publish_robot_movement(goal_x, goal_y)  # Drive the robot
        #                 step = "DriveRobotWithRGB-D"  # Next step
                
        #         case "DriveRobotWithoutRGB-D":  # Drive the robot without the RGB-D camera
        #             self._logger.info(f'positioning_sequence: in DriveRobotWithoutRGB-D --> Success')
        #             for _ in range(6):
        #                 time.sleep(0.5)  # Sleep for 0.5 second to give the robot time to move
        #                 self.publish_robot_movement(goal_x, self.y_offset)  # Drive the robot forward multiple times
        #             step = "Success"  # End the FSM
        
        # self.publish_robot_movement(0.0, 0.0)  # Stop the robot
        # self._logger.info(f'{step}')

        response.success    = True # if step == "Success" else False
        response.label.data = self.object_label  # Return the label of the closest object
        response.pose       = self.object_pose  # Return the pose of the closest object

        self.min_x          = 0.0  # Reset the minimum x-position of the object
        self.object_pose    = Pose()  # Reset the object pose
        self.object_label   = ""  # Reset the object label
        self.object_found   = False  # Reset the object found flag
        
        return response


    def get_object_pose(self, msg:ObjectDetection1D):
        """
        Args:
            msg: ObjectDetection1D, required, x, y and z coordinates of the object in base_link
        Returns:

        Other functions:
            Updates the poisition of the object while the robot is alinging itself
            Logic to choose the closest object
        """

        pose  = msg.pose
        label = msg.label.data

        assert isinstance(pose.position.x, float), self._logger.error('x was not type float')
        assert isinstance(pose.position.y, float), self._logger.error('y was not type float')
        assert isinstance(label, str), self._logger.error('label was not type str')

        # If there are more than one object, choose the closest one. First seen object is used as the base line. 
        # The allowed discrepancy is 0.01m for the object detection from the RGB-D camera

        if self.look_for_box:
            if label == "BOX":
                self.object_pose  = pose
                self.object_label = label
                self.min_x        = pose.position.x
                self.object_found = True
            else: # If the object is not a box, ignore it
                self._logger.info(f'get_object_pose: Ignoring object {label}')
        elif self.unrealistic_x < pose.position.x <= self.min_x or self.min_x == 0.0:
            self.object_pose  = pose
            self.object_label = label
            self.min_x        = pose.position.x  # Update the minimum x-position of the object
            self.object_found = True

    
    def publish_robot_movement(self, x, y):
        """
        Args:
            x: float, required, the x-position of the object in base_link frame
            y: float, required, the y-position of the object in base_link frame
        Returns:
        
        Other functions:
            Publishes the velocities to the wheel motors
        """

        assert isinstance(x, float), self._logger.error('x was not type float')
        assert isinstance(y, float), self._logger.error('y was not type float')

        msg = DutyCycles()

        # if x == 0 and y == 0:  # Stop the robot
        #     msg.duty_cycle_right = 0.0
        #     msg.duty_cycle_left  = 0.0

        # elif np.isclose(y, self.y_offset, atol=self.y_tol):
        #     msg.duty_cycle_right = self.vel_forward * self.multi_forward
        #     msg.duty_cycle_left  = self.vel_forward * self.multi_forward

        if x < 0.0:  # Turn the robot to find the object
            left_turns = 0
            right_turns = 0

            # Turn left until the an object is found
            while not self.object_found:
                if left_turns < 10:
                    msg.duty_cycle_right = self.vel_rotate * self.correct_right
                    msg.duty_cycle_left  = -self.vel_rotate
                    left_turns += 1
                elif right_turns < 20:
                    msg.duty_cycle_right = -self.vel_rotate * self.correct_right
                    msg.duty_cycle_left  = self.vel_rotate
                    right_turns += 1
                else:
                    self._logger.info('publish_robot_movement: No box/object found')
                    msg.duty_cycle_right = 0
                    msg.duty_cycle_left  = 0
                    self.motor_publisher.publish(msg)
                    break
                
                self.motor_publisher.publish(msg)  # Publish the velocities to the wheel motors

                time.sleep(0.5)  # Sleep for 0.5 second to give the robot time to turn
        else:
            self._logger.info('publish_robot_movement: Driving to object')
            turns   = abs(round(np.rad2deg(np.arctan(abs(y) / x)) / self.rotation_per_turn) + (1 if y > 0 else -1)) # Calculate the number of turns needed to align the robot with the object
            dist    = np.sqrt(x ** 2 + y ** 2)
            forward = round((dist - (self.x_stop_drop if self.look_for_box else self.x_stop_pick)) / self.movement_per_forw) # Calculate the number of forward movements needed to get to the object
            
            self._logger.info(f'publish_robot_movement: turns: {turns}, forward: {forward} - {(x - (self.x_stop_drop if self.look_for_box else self.x_stop_pick))}')

            msg.duty_cycle_right = (self.vel_rotate if y > 0 else -self.vel_rotate) * self.correct_right
            msg.duty_cycle_left  = -self.vel_rotate if y > 0 else self.vel_rotate

            for _ in range(turns):
                time.sleep(0.5)  # Sleep for 0.5 second to give the robot time to move
                self.motor_publisher.publish(msg)  # Publish the velocities to the wheel motors

            msg.duty_cycle_right = self.vel_forward * self.correct_right
            msg.duty_cycle_left  = self.vel_forward

            for _ in range(forward):
                time.sleep(0.5)  # Sleep for 0.5 second to give the robot time to move
                self.motor_publisher.publish(msg)  # Publish the velocities to the wheel motors



        # else:
        #     # The robot should not turn faster than the maximum turn velocity and should turn slower the lower the y-value is
        #     turn_vel = min(self.vel_rotate, abs(y - self.y_offset))

        #     # Turn left if y > y_offset, otherwise turn right
        #     msg.duty_cycle_right = turn_vel if y > self.y_offset else -turn_vel
        #     msg.duty_cycle_left  = -turn_vel if y > self.y_offset else turn_vel  

        #     # Also drive forward while turning but depending on how much turning is needed and maximum the forward velocity
        #     msg.duty_cycle_right += min(self.vel_forward, self.vel_forward * self.y_tol / turn_vel)
        #     msg.duty_cycle_left  += min(self.vel_forward, self.vel_forward * self.y_tol / turn_vel)

        # msg.duty_cycle_right *= 1.075  # Adjust the right wheel speed to compensate for it moving slower
        # self.motor_publisher.publish(msg)  # Publish the velocities to the wheel motors


def main(args=None):
    rclpy.init()
    positioningService = PositioningService()

    # Use MultiThreadedExecutor to allow concurrent callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(positioningService)

    try:
        executor.spin()
    except KeyboardInterrupt:
        positioningService.destroy_node()
    finally:
        positioningService.destroy_node()
        rclpy.shutdown()

    rclpy.shutdown()

if __name__ == '__main__':
    main()