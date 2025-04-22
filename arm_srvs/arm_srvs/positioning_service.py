from grumpy_interfaces.srv import PositionRobot
from grumpy_interfaces.msg import ObjectDetection1D

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from robp_interfaces.msg import DutyCycles
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
from std_msgs.msg import Bool

class PositioningService(Node):

    def __init__(self):
        super().__init__('positioning_srv')

        self.service_cb_group    = MutuallyExclusiveCallbackGroup()
        self.subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        # Object detection variables
        self.object_pose  = Pose()  # The position of the object in base_link frame
        self.object_label = ""  # The label of the object
        self.object_found = False  # Flag to check if the object was found
        self.min_x        = 0.0  # The closest x-position of an object in base_link frame
        self.look_for_box = False  # Flag to check if the robot should only look for a box

        # Objects where we will place the element with respect to base_link
        self.desired_object_x = -0.17    # +17 cm in meters
        self.desired_object_y = +0.07    # +7 cm in meters

        # Target distance-parameters to the goal object/box
        self.unrealistic_x =  0.30
        
        # Create the positioning service
        self.srv = self.create_service(
            PositionRobot, 
            '/arm_services/position_robot', 
            self.positioning_sequence,
            callback_group=self.service_cb_group
        )

        # Create the publisher to the drive control path
        self.path_publisher = self.create_publisher(
            Path,
            'drive/path', 
            10
        )

        # Subscribe to drive feedback
        self.drive_result = False
        self.create_subscription(
            Bool,
            'drive/feedback',
            self.drive_feedback_callback,
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

    def drive_feedback_callback(self, msg):
        """
        Callback for drive feedback
        """
        self.drive_result = msg.data
    
    def positioning_sequence(self, request, response):
        """
        Simplified positioning sequence that gets a single object position and sends it to drive_2
        
        Args:
            request.box        : bool, required, if the robot should only look for a box
            request.pose       : Pose, required, the position of the goal object in base_link frame when addition
        Returns:
            response: bool, if the positioning was successful or not
        """
        # Reset vars and determine what we're looking for
        self.object_found = False
        self.object_pose = Pose()
        self.object_label = ""
        self.min_x = 0.0  # Reset this at the start of each call
        
        # Reset drive result at the beginning of each service call
        self.drive_result = False
        
        if request.label.data == "BOX":
            self._logger.info('positioning_sequence: Looking for a box')
            self.look_for_box = True
        else:
            self._logger.info('positioning_sequence: Looking for an object')
            self.look_for_box = False

        # Wait a bit to receive object detection
        timeout_counter = 0
        max_timeout = 20  # 10 seconds (20 * 0.5)
        
        while not self.object_found and timeout_counter < max_timeout:
            self._logger.info(f'Waiting for {"BOX" if self.look_for_box else "OBJECT"} detection...')
            time.sleep(0.5)
            timeout_counter += 1
        
        if not self.object_found:
            self._logger.error('No object found within timeout period')
            response.success = False
            return response
            
        # Object found, get its position
        object_x = self.object_pose.position.x
        object_y = self.object_pose.position.y
        
        # Validate that we have reasonable position values
        if object_x <= 0 or abs(object_y) > 1.0:
            self._logger.error(f'Invalid object position: x={object_x}, y={object_y}')
            response.success = False
            return response
        
        self._logger.info(f'Found {self.object_label} at x: {object_x}, y: {object_y}')
        
        # Calculate the position where the robot should go
        # The robot needs to move to a position that will put the object at the desired coordinates
        # This is essentially a coordinate transformation
        robot_target_x = object_x + self.desired_object_x
        robot_target_y = object_y + self.desired_object_y
        
        self._logger.info(f'Driving to position at x: {robot_target_x}, y: {robot_target_y} to align object at x={self.desired_object_x}m, y={self.desired_object_y}m')
        
        # Create and publish path to drive_2 node
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'base_link'
        
        pose_stamped = PoseStamped()
        pose_stamped.header = path_msg.header
        pose_stamped.pose.position.x = robot_target_x
        pose_stamped.pose.position.y = robot_target_y
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        
        path_msg.poses.append(pose_stamped)
        
        # Publish the path
        self.path_publisher.publish(path_msg)
        
        # Wait for drive to complete with timeout
        drive_timeout = 0
        max_drive_timeout = 60  # 30 seconds
        
        while not self.drive_result and drive_timeout < max_drive_timeout:
            time.sleep(0.5)
            drive_timeout += 1
            
        if not self.drive_result:
            self._logger.error('Drive timed out or failed')
            response.success = False
            return response
            
        # Drive completed successfully
        self._logger.info('Positioning completed successfully')
        
        response.success = True
        response.label.data = self.object_label
        response.pose = self.object_pose
        
        # Reset the object detection variables
        self.min_x = 0.0
        self.object_pose = Pose()
        self.object_label = ""
        self.object_found = False
        self.drive_result = False  # Reset drive result after completion
        
        return response

    def get_object_pose(self, msg:ObjectDetection1D):
        """
        Gets object pose from perception
        
        Args:
            msg: ObjectDetection1D, required, x, y and z coordinates of the object in base_link
        """
        pose = msg.pose
        label = msg.label.data

        try:
            assert isinstance(pose.position.x, float), "x was not type float"
            assert isinstance(pose.position.y, float), "y was not type float"
            assert isinstance(label, str), "label was not type str"
        except AssertionError as e:
            self._logger.error(f"Validation error: {str(e)}")
            return  # Skip this message if validation fails

        # Filter objects based on whether we're looking for a box or other objects
        if self.look_for_box:
            if label == "BOX":
                self.object_pose = pose
                self.object_label = label
                self.min_x = pose.position.x
                self.object_found = True
                self._logger.debug(f"Found BOX at x={pose.position.x}, y={pose.position.y}")
        else:  # Looking for other objects
            # Only consider objects within a realistic range and closer than previously seen
            if self.unrealistic_x < pose.position.x:
                # If this is the first object seen or it's closer than previous ones
                if self.min_x == 0.0 or pose.position.x < self.min_x:
                    self.object_pose = pose
                    self.object_label = label
                    self.min_x = pose.position.x
                    self.object_found = True
                    self._logger.debug(f"Found {label} at x={pose.position.x}, y={pose.position.y}")

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



# step        = "Start"  # The current step in the FSM
# end_strings = ["Success", "Failure"]  # The end strings of the FSM

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
            
#             if request.box:  # If the robot should only look for a box
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


# if x == 0 and y == 0:  # Stop the robot
#     msg.duty_cycle_right = 0.0
#     msg.duty_cycle_left  = 0.0 
# 
# elif np.isclose(y, self.y_offset, atol=self.y_tol):
#     msg.duty_cycle_right = self.vel_forward * self.multi_forward
#     msg.duty_cycle_left  = self.vel_forward * self.multi_forward 
# 
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