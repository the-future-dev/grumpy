from grumpy_interfaces.srv import PickAndDropObject

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

        # Constants in the robot arm:
        # Links:
        # From joint of servo 5 to joint of servo 4:
        self.l1 = 0.10048
        # From joint of servo 4 to joint of servo 3:
        self.l2 = 0.094714
        # From joint of servo 3 to joint of servo 2 + from joint servo 2 to griping point
        self.l3 = 0.05071 + 0.11260

        # Origin of servo 4 in rho+z-plane
        self.z_origin_servo4   = self.z_origin_servo5 + self.l1 * np.sin(np.deg2rad(90) - np.deg2rad(self.theta_servo5))
        self.rho_origin_servo4 = self.l1 * np.cos(np.deg2rad(90) - np.deg2rad(self.theta_servo5))
         
        # Sets angles of the servos for different tasks, as well as time for the arm to move into these positions:
        # Arm pointing straight up, used for reset and driving around
        self.initial_thetas = [1000, 12000, 12000, 12000, 12000, 12000]
        # Angles when the arm camera has a view over the entire pick-up area
        self.view_thetas = [-1, -1, 3000, 17500, 9000, -1]
        # Angles from which the inverse kinematics are calculated for picking up objects
        self.pre_grasp_thetas = [-1, -1, 3000, 14500, 6000, -1]
        # Angles for droping objects into the bins
        self.drop_thetas = [-1 , -1, 3000, 14500, 9000, -1]

        # Standard angle movement times to all positions
        self.times = [1000, 1000, 1000, 1000, 1000, 1000]

        self.srv = self.create_service(
            PickAndDropObject, 
            'drop_object', 
            self.drop_sequence
        )

        self.servo_angle_publisher = self.create_publisher(
            Int16MultiArray,
            '/multi_servo_cmd_sub',
            1
        )
        
        # self.servo_subscriber = self.create_subscription(
        #     Int16MultiArray,
        #     '/servo_pos_publisher',
        #     self.servo_callback,
        #     1
        # )
        
    # def servo_callback(self, msg:Int16MultiArray):
    #     current_servo_angles = msg.data

    #     assert current_servo_angles is list

    #     self.get_logger.info('Got the angles of the servos')

    
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

        while step != "Success" or step != "Failure":
            times = self.times
            match step:
                case "Start":  # Make sure the arm is in the initial position but does not drop the object
                    thetas = self.initial_thetas.copy()  # Copy the initial thetas
                    thetas[0] = -1  # Make sure the gripper does not move
                    step = "GetPosition"  # Move to the next step

                case "GetPosition":  # Get the position of the box from the request
                    assert request.pose is Pose, self._logger.error('request was not type Pose')  # Assert that the request has the correct type
                    x, y, z = self.extract_object_position(request.pose)  # Get the position of the box from the request
                    thetas = [-1, -1, -1, -1, -1, -1]  # Do not move the arm
                    step = "DropAngles"  # Move to the next step

                case "DropAngles":  # Sets the angles for the arm to drop the object
                    thetas = self.drop_thetas
                    step = "DropObject"  # Move to the next step

                case "DropObject":  # Drops the object
                    thetas[0] = [1000, -1, -1, -1, -1, -1]  # Only move and open the gripper
                    step = "DrivePosition"  # Move to the next step

                case "DrivePosition":  # Finish the drop sequence by going back to the initial position
                    thetas = self.initial_thetas
                    step = "Success"  # End the FSM
            
            self.check_angles_and_times(thetas, times)  # Assert that the angles and times are in the correct format
            
            self.publish_angles(thetas, times)  # Publish the angles to the arm
                    
        response.success = True if step == "Success" else False
        
        return response


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

        assert x is float, self._logger.error('x was not type float')
        assert y is float, self._logger.error('y was not type float')
        assert z is float, self._logger.error('z was not type float')

        self.get_logger.info('Got the position of the object')

        return x, y, z


    def check_angles_and_times(self, angles, times):
        """
        Args:
            angles: List, required, the angles for each servo to be set to
            times:  List, required, the times for each servo to get to the given angle
        Returns:
            
        Other functions:
            Raises error if the angles and times are not in the correct format, length or interval
        """

        assert len(angles) == 6, self._logger.error('angles was not of length 6')
        assert len(times) == 6, self._logger.error('times was not of length 6')
        assert all(isinstance(angle, int) for angle in angles), self._logger.error('angles was not of type int')
        assert all(isinstance(time, int) for time in times), self._logger.error('times was not of type int')
        assert all(1000 <= time <= 5000 for time in times), self._logger.error('times was not within the interval [1000, 5000]')
        assert (0 <= angles[0] <= 11000) or (angles[0] == -1), self._logger.error('servo 1 was not within the interval [0, 11000] or -1')
        assert (0 <= angles[1] <= 24000) or (angles[1] == -1), self._logger.error('servo 2 was not within the interval [0, 24000] or -1')
        assert (2500 <= angles[2] <= 21000) or (angles[2] == -1), self._logger.error('servo 3 was not within the interval [2500, 21000] or -1')
        assert (3000 <= angles[3] <= 21500) or (angles[3] == -1), self._logger.error('servo 4 was not within the interval [3000, 21500] or -1')
        assert (6000 <= angles[4] <= 18000) or (angles[4] == -1), self._logger.error('servo 5 was not within the interval [6000, 18000] or -1')
        assert (0 <= angles[5] <= 20000) or (angles[5] == -1), self._logger.error('servo 6 was not within the interval [0, 20000] or -1')

        self.get_logger.info('Checked the angles and times')

    
    def publish_angles(self, angles, times=[1000, 1000, 1000, 1000, 1000, 1000]):
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
        msg.layout.dim[0] = {'label':'', 'size': 0, 'stride': 0}
        msg.layout.data_offset = 0

        msg.data = angles + times  # Concatenates the angles and times

        self.servo_angle_publisher.publish(msg)

        time.sleep(np.max(times) + 0.5)  # Makes the code wait until the arm has had the time to move to the given angles

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