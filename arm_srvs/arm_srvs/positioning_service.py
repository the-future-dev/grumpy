from grumpy_interfaces.srv import PositionRobot
from grumpy_interfaces.msg import ObjectDetection1D

import rclpy
import rclpy.logging
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
        self.vel_forward = 0.05
        self.vel_rotate  = 0.02

        # Goal object position and label:
        self.object_pose  = Pose()
        self.object_label = ""

        # Target distance-parameters to the goal object/box
        self.x_stop_pick =  0.35  # The x-position where the robot should stop when driving with the RGB-D camera to an object
        self.x_stop_drop =  0.15  # The x-position where the robot should stop when driving with the RGB-D camera to a box
        self.y_offset    = -0.075  # The y-position where the robot should stop rotating with the RGB-D camera
        self.y_tol       =  0.02  # The tolerance for the y-position when driving with the RGB-D camera

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
            1,
            callback_group=self.subscriber_cb_group
        )

    
    def positioning_sequence(self, request, response):
        """
        Args:
            request.pose:  Pose, required, the position and orientation of the goal object
            request.label: String, required, the label of the goal object
        Returns:
            response: bool, if the positioning was successful or not
        Other functions:
            Controlls the positioning sequence
            Calls the publishing function which publishes the velocities to the wheel motors for each step in the sequence
        """

        step = "Start"
        end_strings = ["Success", "Failure"]

        while step not in end_strings:
            time.sleep(1.0) # Sleep for 0.1 second to give the robot time to move
            self._logger.info(f'{step}')  # Log the current step
            goal_x = self.object_pose.position.x
            goal_y = self.object_pose.position.y
            match step:
                case "Start":  # Ask if we see an/the goal object
                    # self.check_object(request.label.data, request.pose)  # Check if the object could be the correct one
                    next_step = "DriveRobotWithRGB-D"  # Next step

                case "DriveRobotWithRGB-D":  # Drive the robot using the RGB-D camera as feedback
                    if request.label.data == 'B':
                        self._logger.info(f'positioning_sequence: in B')
                        if np.isclose(goal_y, self.y_offset, atol=self.y_tol) and goal_x <= self.x_stop_drop:
                            self._logger.info(f'positioning_sequence: in B --> Success')
                            self.publish_robot_movement(0.0, 0.0)  # Stop the robot
                            next_step = "Success"  # End the FSM
                        else:
                            self._logger.info(f'positioning_sequence: in B --> DriveRobotWithRGB-D')
                            self.publish_robot_movement(goal_x, goal_y)  # Drive the robot
                            next_step = "DriveRobotWithRGB-D"  # Next step
                    else:
                        # End condition for driving the robot using the RGB-D camera
                        self._logger.info(f'positioning_sequence: in Object')
                        if np.isclose(goal_y, self.y_offset, atol=self.y_tol) and goal_x <= self.x_stop_pick:
                            self._logger.info(f'positioning_sequence: in Object --> DriveRobotWithoutRGB-D')
                            self.publish_robot_movement(0.0, 0.0)  # Stop the robot
                            next_step = "DriveRobotWithoutRGB-D"  # Next step
                        else:
                            self._logger.info(f'positioning_sequence: in Object --> DriveRobotWithRGB-D')
                            self.publish_robot_movement(goal_x, goal_y)  # Drive the robot
                            next_step = "DriveRobotWithRGB-D"  # Next step
                
                case "DriveRobotWithoutRGB-D":  # Drive the robot without the RGB-D camera
                    self._logger.info(f'positioning_sequence: in DriveRobotWithoutRGB-D')
                    for _ in range(6):
                        time.sleep(1.0)  
                        self.publish_robot_movement(goal_x, self.y_offset)  # Drive the robot forward multiple times
                    self.publish_robot_movement(0.0, 0.0)  # Stop the robot
                    next_step = "Success"  # End the FSM
            
            step = next_step
            
        self._logger.info(f'{step}')
        response.success = True if step == "Success" else False
        
        return response


    def get_object_pose(self, msg:ObjectDetection1D):
        """
        Args:
            msg: ObjectDetection1D, required, x, y and z coordinates of the object in base_link
        Returns:

        Other functions:
            Updates the poisition of the object while the robot is alinging itself
        """

        pose  = msg.pose
        label = msg.label.data

        assert isinstance(pose.position.x, float), self._logger.error('x was not type float')
        assert isinstance(pose.position.y, float), self._logger.error('y was not type float')
        assert isinstance(pose.position.z, float), self._logger.error('z was not type float')
        assert isinstance(label, str), self._logger.error('label was not type str')

        self.object_pose  = pose
        self.object_label = label


    def check_object(self, label, pose):
        """
        Args:
            label: String, required, the class of the requested goal object
            pose: Pose, required, the position and orientation of the requested goal object
        Returns:
            goal_object: bool, if the object is the requested goal object
        Other functions:

        """

        assert isinstance(label, str), self._logger.error('label was not type str')
        assert isinstance(pose, Pose), self._logger.error('pose was not type Pose')

        # Check if the object from perception is the requested goal object
        goal_object = (label == self.object_label and
                       np.isclose(self.object_pose.position.x, pose.position.x, atol=0.10) and
                       np.isclose(self.object_pose.position.y, pose.position.y, atol=0.10) and
                       np.isclose(self.object_pose.position.z, pose.position.z, atol=0.10)
                       )

        return goal_object

    
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

        if x == 0 and y == 0:  # Stop the robot
            msg.duty_cycle_right = 0.0
            msg.duty_cycle_left = 0.0
        elif np.isclose(y, self.y_offset, atol=self.y_tol):
            msg.duty_cycle_right = self.vel_forward
            msg.duty_cycle_left  = self.vel_forward
        else:
            # The robot should not turn faster than the maximum turn velocity and should turn slower the lower the y-value is
            turn_vel = min(self.vel_rotate, abs(y - self.y_offset))

            # Turn left if y > y_offset, otherwise turn right
            msg.duty_cycle_right = turn_vel if y > self.y_offset else -turn_vel
            msg.duty_cycle_left  = -turn_vel if y > self.y_offset else turn_vel  

            # Also drive forward while turning but depending on how much turning is needed
            msg.duty_cycle_right += self.vel_forward * self.y_tol / turn_vel
            msg.duty_cycle_left  += self.vel_forward * self.y_tol / turn_vel

        self.motor_publisher.publish(msg)  # Publish the velocities to the wheel motors

        # time.sleep(1.0) # Sleep for 0.1 second to give the robot time to move


def main(args=None):
    rclpy.init()
    positioningService = PositioningService()

    # Use MultiThreadedExecutor to allow concurrent callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(positioningService)

    try:
        executor.spin()
    # except KeyboardInterrupt:
    #     pass
    finally:
        positioningService.destroy_node()
        rclpy.shutdown()

    # rclpy.shutdown()

if __name__ == '__main__':
    main()