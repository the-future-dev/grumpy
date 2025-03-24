from grumpy_interfaces.srv import PositionRobot

import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Pose
import numpy as np
import time

class DropService(Node):

    def __init__(self):
        super().__init__('drop_srv')

        # Origin of servo 5 in base_link frame:
        self.x_origin_servo5 = -0.00450
        self.y_origin_servo5 = -0.04750
        self.z_origin_servo5 =  0.12915
        self.theta_servo5    =  60

        # Constants in the robot arm links:
        self.l1 = 0.10048  # From joint of servo 5 to joint of servo 4:
        self.l2 = 0.094714  # From joint of servo 4 to joint of servo 3:
        self.l3 = 0.05071 + 0.11260  # From joint of servo 3 to joint of servo 2 + from joint servo 2 to griping point

        # Origin of servo 4 in rho+z-plane
        self.z_origin_servo4   = self.z_origin_servo5 + self.l1 * np.sin(np.deg2rad(90) - np.deg2rad(self.theta_servo5))
        self.rho_origin_servo4 = self.l1 * np.cos(np.deg2rad(90) - np.deg2rad(self.theta_servo5))
         
        # Sets angles of the servos for different tasks, as well as time for the arm to move into these positions:
        self.initial_thetas = [1000, 12000, 12000, 12000, 12000, 12000]  # Arm pointing straight up, used for reset and driving around
        self.view_thetas = [-1, -1, 3000, 17500, 9000, -1]  # Angles when the arm camera has a view over the entire pick-up area
        self.drop_thetas = [-1 , -1, 3000, 14500, 9000, -1]  # Angles for droping objects into the bins

        self.times = [1000, 1000, 1000, 1000, 1000, 1000]  # Standard angle movement times to all positions

        self.current_angles = self.initial_thetas  # Keeps track of the angles of the servos published under /servo_pos_publisher

        # Create the drop service
        self.srv = self.create_service(
            PositionRobot, 
            'drop_object', 
            self.drop_sequence
        )

        # Create the publisher and subscriber for the angles of the servos
        self.servo_angle_publisher = self.create_publisher(
            Int16MultiArray,
            '/multi_servo_cmd_sub',
            1
        )
        
        self.servo_subscriber = self.create_subscription(
            Int16MultiArray,
            '/servo_pos_publisher',
            self.current_servos,
            1
        )

    
    def drop_sequence(self, request, response):
        """
        Args:
            request: Pose, required, the position and orientation of the box
        Returns:
            response: Bool, if the drop was successful or not
        Other functions:
            Controlls the pick up sequence
            Calls the publishing function which publishes the servo angles to the arm for each step in the sequence
        """

        step = "Start"
        x, y, z = 0, 0, 0
        end_strings = ["Success", "Failure"]

        while step not in end_strings:
            self._logger.info(f'{step}')
            times = self.times
            match step:
                case "Start":  # Make sure the arm is in the initial position but does not drop the object
                    thetas = self.initial_thetas.copy()  # Copy the initial thetas
                    thetas[0] = -1  # Make sure the gripper does not move
                    next_step = "GetPosition"  # Move to the next step

                case "GetPosition":  # Get the position of the box from the request
                    assert isinstance(request.pose, Pose), self._logger.error(f'request was not type Pose')  # Assert that the request has the correct type
                    x, y, z = self.extract_object_position(request.pose)  # Get the position of the box from the request
                    thetas = [-1, -1, -1, -1, -1, -1]  # Do not move the arm
                    next_step = "RotateBase"  # Move to the next step

                case "RotateBase":  # Move servo 6/base to the correct angle
                    # Calculate the change of the angle for servo 6, then new angle of servo 6, round and convert to int
                    theta_servo6 = round(self.initial_thetas[5] + self.get_delta_theta_6(x, y) * 100)
                    thetas = [-1, -1, -1, -1, -1, theta_servo6]  # Only servo 6 is moved
                    next_step = "DropAngles"  # Move to the next step

                case "DropAngles":  # Sets the angles for the arm to drop the object
                    thetas = self.drop_thetas
                    next_step = "DropObject"  # Move to the next step

                case "DropObject":  # Drops the object
                    thetas = [1000, -1, -1, -1, -1, -1]  # Only move and open the gripper
                    next_step = "DrivePosition"  # Move to the next step

                case "DrivePosition":  # Finish the drop sequence by going back to the initial position
                    thetas = self.initial_thetas
                    next_step = "Success"  # End the FSM
            
            self.check_angles_and_times(thetas, times)  # Assert that the angles and times are in the correct format
            
            if self.publish_angles(thetas, times):  # Publish the angles to the arm and check if the arm has moved to the correct angles
                step = next_step
            else:
                step = "Failure"
        
        self._logger.info(f'{step}')
        response.success = True if step == "Success" else False
        
        return response


    def current_servos(self, msg:Int16MultiArray):
        current_angles = msg.data

        assert isinstance(current_angles, list), self._logger.error('angles is not of type list')
        assert len(current_angles) == 6, self._logger.error('angles was not of length 6')
        assert all(isinstance(current_angles, int) for angle in current_angles), self._logger.error('angles was not of type int')

        self.get_logger.info('Got the angles of the servos')

        self.current_angles = current_angles


    def extract_object_position(self, pose:Pose):
        """
        Args:
            msg: Pose, required, the position and orientation of the object
        Returns:
            x: Float, x-position of the object in base_link frame
            y: Float, y-position of the object in base_link frame
            z: Float, z-position of the object in base_link frame
        Other functions:

        """

        x, y, z = pose.position.x, pose.position.y, pose.position.z

        assert isinstance(x, float), self._logger.error('x was not type float')
        assert isinstance(x, float), self._logger.error('y was not type float')
        assert isinstance(x, float), self._logger.error('z was not type float')


        self._logger.info('Got the position of the object')

        return x, y, z
    

    def get_delta_theta_6(self, x, y):
        """
        Args:
            x: Float, required, x-position of the object in base_link frame
            y: Float, required, y-position of the object in base_link frame
        Returns:
            delta_theta_6: Float, degrees that servo 6 has to rotate from its position
        Other functions:

        """

        x_dist = x - self.x_origin_servo5
        y_dist = y - self.y_origin_servo5

        # Calculate the angle for servo 6 in radians and convert to degrees
        return np.rad2deg(np.arctan2(y_dist, x_dist))


    def check_angles_and_times(self, angles, times):
        """
        Args:
            angles: List, required, the angles for each servo to be set to
            times:  List, required, the times for each servo to get to the given angle
        Returns:
            
        Other functions:
            Raises error if the angles and times are not in the correct format, length or interval
        """

        assert isinstance(angles, list), self._logger.error('angles is not of type list')
        assert isinstance(times, list), self._logger.error('times is not of type list')
        assert len(angles) == 6, self._logger.error('angles was not of length 6')
        assert len(times) == 6, self._logger.error('times was not of length 6')
        assert all(isinstance(angle, int) for angle in angles), self._logger.error('angles was not of type int')
        assert all(isinstance(time, int) for time in times), self._logger.error('times was not of type int')
        assert all(1000 <= time <= 5000 for time in times), self._logger.error('times was not within the interval [1000, 5000]')
        assert (0 <= angles[0] <= 11000) or (angles[0] == -1), self._logger.error(f'servo 1 was not within the interval [0, 11000] or -1, got {angles[0]}')
        assert (0 <= angles[1] <= 24000) or (angles[1] == -1), self._logger.error(f'servo 2 was not within the interval [0, 24000] or -1, got {angles[1]}')
        assert (2500 <= angles[2] <= 21000) or (angles[2] == -1), self._logger.error(f'servo 3 was not within the interval [2500, 21000] or -1, got {angles[2]}')
        assert (3000 <= angles[3] <= 21500) or (angles[3] == -1), self._logger.error(f'servo 4 was not within the interval [3000, 21500] or -1, got {angles[3]}')
        assert (6000 <= angles[4] <= 18000) or (angles[4] == -1), self._logger.error(f'servo 5 was not within the interval [6000, 18000] or -1, got {angles[4]}')
        assert (0 <= angles[5] <= 20000) or (angles[5] == -1), self._logger.error(f'servo 6 was not within the interval [0, 20000] or -1, got {angles[5]}')

        self._logger.info('Checked the angles and times')

    
    def publish_angles(self, angles, times):
        """
        Args:
            angles: List, required, the angles for each servo to be set to
            times:  List, optional, the times for each servo to get to the given angle
        Returns:
        
        Other functions:
            Publishes the angles of the servos to the arm in the correct format
        """

        # Initializes the message with required informaiton
        msg = Int16MultiArray()
        # msg.layout.dim[0] = {'label':'', 'size': 0, 'stride': 0}
        # msg.layout.data_offset = 0

        msg.data = angles + times  # Concatenates the angles and times

        self.servo_angle_publisher.publish(msg)

        time.sleep(np.max(times) / 1000 + 0.5)  # Makes the code wait until the arm has had the time to move to the given angles

        return self.current_angles == angles  # Checks if the arm has moved to the correct angles

def main(args=None):
    rclpy.init()
    dropServie = DropService()

    try:
        rclpy.spin(dropServie)
    except KeyboardInterrupt:
        pass

    dropServie.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()