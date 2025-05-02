from grumpy_interfaces.srv import PositionRobot
from grumpy_interfaces.msg import ObjectDetection1D

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from robp_interfaces.msg import DutyCycles
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
from collections import Counter
import arm_srvs.utils as utils
import time

class PositioningService(Node):

    def __init__(self):
        super().__init__('positioning_srv')

        self.service_cb_group    = MutuallyExclusiveCallbackGroup()
        self.subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        # Set speeds for the robot to move
        self.vel_forward       = 0.08  # 0.07 before
        self.vel_rotate        = 0.06  # 0.05 before
        self.correct_right     = 1.00  # Correction of the right wheel # 1.00
        self.rotation_per_turn = 4.00  # Degrees of rotation per turn msg # 5.50
        self.movement_per_forw = 0.041  # Moved distance per forward msg # 0.0275

        self.object_pose  = Pose()  # The position of the object in base_link frame
        self.object_label = []  # The label of the object
        self.object_found = False  # Flag to check if the object was found
        self.min_x        = 1.5  # The closest x-position of an object in base_link frame
        self.look_for_box = False  # Flag to check if the robot should only look for a box
        self.update       = True  # Flag to check if the driving should be updated with new perception data

        # Target distance-parameters to the goal object/box
        self.unrealistic_x = 0.30  # The x-position where the RGB-D camera should not be able to locate an object
        self.x_stop_goal   = 0.20  # The desired length in the x-direction from the goal object/box
        self.switch_pick   = 0.375  # The x-position where the robot should stop updating the position of the object with the RGB-D camera
        self.switch_drop   = 0.45  # The x-position where the robot should stop updating the position of the box with the RGB-D camera
        self.x_stop_box    = 0.30  # The desired length in the x-direction from the box
        self.y_offset      = utils.y_origin_servo5  # The y-position where the robot should stop rotating with the RGB-D camera
        
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
            request.box        : bool, required, if the robot should only look for a box
            request.pose       : Pose, required, the position of the goal object in base_link frame when addition
            request.backup     : bool, required, if the robot should only back up
        Returns:
            response.success   : bool, if the positioning was successful or not
            response.label.data: String, the label of the closest object
        Other functions:
            Controlls the positioning sequence
            Calls the publishing function which publishes the velocities to the wheel motors for each step in the sequence
        """
        
        self.look_for_box   = request.box  # Reset the look for box flag
        self.min_x          = 1.5  # Reset the minimum x-position of the object
        self.object_pose    = Pose()  # Reset the object pose
        self.object_label   = []  # Reset the object label
        self.object_found   = False  # Reset the object found flag
        self.update         = True  # Reset the update flag

        time.sleep(0.5)  # Sleep for 0.5 second to recive the perception messages

        position_to = 'BOX' if request.box else 'OBJECT'

        if request.pose.position.x != 0.0 or request.pose.position.y != 0.0:
            self.update       = False  # Do not update the position of the object with the RGB-D camera as it will not see it
            self._logger.info(f'positioning_sequence: Repositioning given x: {request.pose.position.x}, y: {request.pose.position.y}')

            self.publish_robot_movement(x=request.pose.position.x, y=request.pose.position.y)

            response.success    = True  # if step == "Success" else False
            response.label.data = ''  # Not needed, but required for the response

        elif request.backup:
            self.update = False  # Do not update the position of the object with the RGB-D camera as the robot should only back up
            self._logger.info(f'positioning_sequence: Backing up from the {position_to}')

            self.publish_robot_movement(x=0.0, y=0.0)

            response.success    = True  # if step == "Success" else False
            response.label.data = ''  # Not needed, but required for the response

        else:
            if not self.object_found:
                self._logger.info(f'positioning_sequence: Did not find {'a' if self.look_for_box else 'an'} {position_to} initially, trying to find one')
                self.publish_robot_movement(x=-1.0, y=0.0)
            
            if self.object_found:
                self._logger.info(f'positioning_sequence: Found {'a' if self.look_for_box else 'an'} {position_to}, driving to it')

                self.publish_robot_movement(x=self.object_pose.position.x, y=self.object_pose.position.y)  # Drive the robot to the object/box

                self._logger.info(f'positioning_sequence: At the {position_to}')
                response.success    = True
                counter             = Counter(self.object_label)
                most_common         = counter.most_common(1)[0][0]  # Returns the most common string
                response.label.data = most_common  # Return the label of the closest object
                self._logger.info(f'label: {most_common}')

            else:
                self._logger.error(f'positioning_sequence: Could not find an {'a' if self.look_for_box else 'an'} {position_to}, returning failure')
                response.success    = False
                response.label.data = ""
        
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
        time  = rclpy.time.Time().from_msg(msg.header.stamp)

        assert isinstance(pose.position.x, float), self._logger.error('x was not type float')
        assert isinstance(pose.position.y, float), self._logger.error('y was not type float')
        assert isinstance(label, str), self._logger.error('label was not type str')

        # If there are more than one object, choose the closest one. First seen object is used as the base line. 
        # The allowed discrepancy is 0.01m for the object detection from the RGB-D camera
        if self.get_clock().now().nanoseconds < time.nanoseconds + 3e9:
            if self.look_for_box:
                if label == "BOX" and (self.unrealistic_x < pose.position.x <= self.min_x + 0.02):
                    self.object_pose  = pose
                    self.object_label.append(label)
                    self.min_x        = pose.position.x
                    self.object_found = True

            elif label in ['PUPPY', 'CUBE', 'SPHERE'] and (self.unrealistic_x < pose.position.x <= self.min_x + 0.02):
                self.object_pose  = pose
                self.object_label.append(label)
                self.min_x        = pose.position.x  # Update the minimum x-position of the object
                self.object_found = True

    
    def publish_robot_movement(self, x:float, y:float):
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

        if x == -1.0:  # Turn the robot to find the object
            left_turns   = 0
            right_turns  = 0

            # Turn left until the an object is found
            while not self.object_found:
                if left_turns < 15:
                    msg.duty_cycle_right = self.vel_rotate * self.correct_right
                    msg.duty_cycle_left  = -self.vel_rotate
                    left_turns += 1

                elif right_turns < 30:
                    msg.duty_cycle_right = -self.vel_rotate * self.correct_right
                    msg.duty_cycle_left  = self.vel_rotate
                    right_turns += 1
                    if right_turns == 30:
                        left_turns  = 0

                else:
                    msg.duty_cycle_right = 0.0
                    msg.duty_cycle_left  = 0.0
                    self.motor_publisher.publish(msg)
                    break

                self.motor_publisher.publish(msg)  # Publish the velocities to the wheel motors
                time.sleep(0.4)  # Sleep for 0.5 second to give the robot time to turn

        else:
            if not self.update and self.look_for_box:
                x_offset = self.switch_drop
                turns    = round(np.rad2deg(np.arctan(abs(y - self.y_offset) / x)) / self.rotation_per_turn) if x != 0.0 else 0 # Calculate the number of turns needed to align the robot with the object
                forward  = round(abs(max(0, x - x_offset)) / self.movement_per_forw) # Calculate the number of forward movements needed to get to the object
            else:
                x_offset = self.x_stop_goal if not self.look_for_box else self.x_stop_box
                turns    = round(np.rad2deg(np.arctan(abs(y - self.y_offset) / x)) / self.rotation_per_turn) if x != 0.0 else 0 # Calculate the number of turns needed to align the robot with the object
                forward  = round(abs(x - x_offset) / self.movement_per_forw) # Calculate the number of forward movements needed to get to the object
            
            # forward     = 0
            # turns       = 5
            # self.update = False
            # self._logger.info(f'Updated turns and forward: {turns} : {y}, {forward} : {x}')

            while forward > 0 or turns > 0:  # While the robot is not done moving
                # self._logger.info(f'Turns: {turns}, Forward: {forward}')
                msg.duty_cycle_right, msg.duty_cycle_left = 0.0, 0.0 # Initialize the duty cycles to 0
                
                if turns > 0:  # If the robot needs to turn
                    msg.duty_cycle_right += self.vel_rotate if y > self.y_offset else -self.vel_rotate
                    msg.duty_cycle_left  += -self.vel_rotate if y > self.y_offset else self.vel_rotate
                    turns                -= 1
                
                elif forward > 0:  # If the robot needs to move forward and does not have to turn too much
                    msg.duty_cycle_right += self.vel_forward if x >= x_offset else -self.vel_forward
                    msg.duty_cycle_left  += self.vel_forward if x >= x_offset else -self.vel_forward
                    forward              -= 1

                msg.duty_cycle_right *= self.correct_right  # Adjust the right wheel speed to compensate for it moving slower

                self.motor_publisher.publish(msg)  # Publish the velocities to the wheel motors
                time.sleep(0.4)  # Sleep for 0.5 second to give the robot time to move
                
                new_x, new_y = self.object_pose.position.x, self.object_pose.position.y  # Get the new positions of the object/box in base_link frame from perception

                if new_x < x and self.update:  # If the new perception has a closer x-position than the previous one
                    x, y    = new_x, new_y  # Update the position of the object/box
                    # Update the number of turns and forward movements needed to get to the object/box
                    turns   = round(np.rad2deg(np.arctan(abs(y - self.y_offset) / x)) / self.rotation_per_turn)
                    forward = round(abs(x - x_offset) / self.movement_per_forw)
                    # self._logger.info(f'Updated turns and forward: {turns} : {y}, {forward} : {x}')
                    if x <= (self.switch_drop if self.look_for_box else self.switch_pick):  # If the robot is close enough to the object
                        # self._logger.info(f'Not updating the position of the object anymore')
                        self.update = False


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
