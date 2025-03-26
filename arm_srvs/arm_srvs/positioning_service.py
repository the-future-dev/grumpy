from grumpy_interfaces.srv import PositionRobot
from grumpy_interfaces.msg import ObjectDetection1D

import rclpy
import rclpy.logging
from robp_interfaces.msg import DutyCycles
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Pose
import numpy as np
import time

class PositioningService(Node):

    def __init__(self):
        super().__init__('positioning_srv')

        # Set speeds for the robot to move
        self.vel_forward = 0.1
        self.vel_rotate = 0.05
        self.vel_small_rotate = 0.01
        self.vel_arrived = 0.08

        # Goal object position and label:
        self.object_pose = Pose()
        self.object_label = ""

        # Create the positioning service
        self.srv = self.create_service(
            PositionRobot, 
            '/arm_services/position_robot', 
            self.positioning_sequence
        )

        # Create the publisher to the wheel motors
        self.motor_publisher = self.create_publisher(
            DutyCycles,
            '/motor/duty_cycles', 
            10
        )
        
        # Create the subscriber to the object detection
        self.servo_subscriber = self.create_subscription(
            ObjectDetection1D,
            '/perception/object_poses',
            self.get_object_pose,
            1
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
            self._logger.info(f'{step}')
            x = self.object_pose.position.x
            y = self.object_pose.position.y
            match step:
                case "Start":  # Ask if we see an/the goal object
                    # self.check_object(request.label, request.pose)  # Check if the object is the correct one
                    next_step = "RotateRobot"  # Move to the next step

                case "DriveRobot":  # Rotate the robot until it points towards the goal object
                    if np.isclose(y, 0, atol=0.05) and x <= 0.30:  # End condition for driving the robot using the RGB-D camera
                        self.publish_robot_movement(0.0, 0.0)  # Stop the robot
                        next_step = "Success"  # Move to the next step
                    else:
                        self.publish_robot_movement(x, y)  # Drive the robot
                        next_step = "DriveRobot"
            
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

        pose = msg.pose
        label = msg.label

        assert isinstance(pose.position.x, float), self._logger.error('x was not type float')
        assert isinstance(pose.position.y, float), self._logger.error('y was not type float')
        assert isinstance(pose.position.z, float), self._logger.error('z was not type float')
        assert isinstance(label, str), self._logger.error('label was not type str')

        self.object_pose = pose
        self.object_label = label


    def extract_object_position(self, pose:Pose):
        """
        Args:
            msg: Pose, required, the position and orientation of the object
        Returns:
            x: float, x-position of the object in base_link frame
            y: float, y-position of the object in base_link frame
            z: float, z-position of the object in base_link frame
        Other functions:

        """

        x, y, z = pose.position.x, pose.position.y, pose.position.z

        assert isinstance(x, float), self._logger.error('x was not type float')
        assert isinstance(x, float), self._logger.error('y was not type float')
        assert isinstance(x, float), self._logger.error('z was not type float')


        self._logger.info('Got the position of the object')

        return x, y, z
    

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

        goal_object = (label == self.object_label and
                       np.isclose(self.object_pose.position.x, pose.position.x, atol=0.10) and
                       np.isclose(self.object_pose.position.y, pose.position.y, atol=0.10) and
                       np.isclose(self.object_pose.position.z, pose.position.z, atol=0.10)
                       )

        return goal_object

    # def get_delta_theta_6(self, x, y):
    #     """
    #     Args:
    #         x: float, required, x-position of the object in base_link frame
    #         y: float, required, y-position of the object in base_link frame
    #     Returns:
    #         delta_theta_6: float, degrees that servo 6 has to rotate from its position
    #     Other functions:

    #     """

    #     x_dist = x - self.x_origin_servo5
    #     y_dist = y - self.y_origin_servo5

    #     # Calculate the angle for servo 6 in radians and convert to degrees
    #     return np.rad2deg(np.arctan2(y_dist, x_dist))


    # def check_angles_and_times(self, angles, times):
    #     """
    #     Args:
    #         angles: List, required, the angles for each servo to be set to
    #         times:  List, required, the times for each servo to get to the given angle
    #     Returns:
            
    #     Other functions:
    #         Raises error if the angles and times are not in the correct format, length or interval
    #     """

    #     assert isinstance(angles, list), self._logger.error('angles is not of type list')
    #     assert isinstance(times, list), self._logger.error('times is not of type list')
    #     assert len(angles) == 6, self._logger.error('angles was not of length 6')
    #     assert len(times) == 6, self._logger.error('times was not of length 6')
    #     assert all(isinstance(angle, int) for angle in angles), self._logger.error('angles was not of type int')
    #     assert all(isinstance(time, int) for time in times), self._logger.error('times was not of type int')
    #     assert all(1000 <= time <= 5000 for time in times), self._logger.error('times was not within the interval [1000, 5000]')
    #     assert (0 <= angles[0] <= 11000) or (angles[0] == -1), self._logger.error(f'servo 1 was not within the interval [0, 11000] or -1, got {angles[0]}')
    #     assert (0 <= angles[1] <= 24000) or (angles[1] == -1), self._logger.error(f'servo 2 was not within the interval [0, 24000] or -1, got {angles[1]}')
    #     assert (2500 <= angles[2] <= 21000) or (angles[2] == -1), self._logger.error(f'servo 3 was not within the interval [2500, 21000] or -1, got {angles[2]}')
    #     assert (3000 <= angles[3] <= 21500) or (angles[3] == -1), self._logger.error(f'servo 4 was not within the interval [3000, 21500] or -1, got {angles[3]}')
    #     assert (6000 <= angles[4] <= 18000) or (angles[4] == -1), self._logger.error(f'servo 5 was not within the interval [6000, 18000] or -1, got {angles[4]}')
    #     assert (0 <= angles[5] <= 20000) or (angles[5] == -1), self._logger.error(f'servo 6 was not within the interval [0, 20000] or -1, got {angles[5]}')

    #     self._logger.info('Checked the angles and times')

    
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
        elif np.isclose(y, 0, atol=0.05):
            msg.duty_cycle_right = self.vel_forward
            msg.duty_cycle_left = self.vel_forward
        else:
            if y > 0:
                msg.duty_cycle_right = self.vel_rotate
                msg.duty_cycle_left = -self.vel_rotate
            elif y < 0:
                msg.duty_cycle_right = -self.vel_rotate
                msg.duty_cycle_left = self.vel_rotate


        # Publish the velocities to the wheel motors
        self.motor_publisher.publish(msg)

        self._logger.info('Published the robot movement')


def main(args=None):
    rclpy.init()
    positioningService = PositioningService()

    try:
        rclpy.spin(positioningService)
    except KeyboardInterrupt:
        pass

    positioningService.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()