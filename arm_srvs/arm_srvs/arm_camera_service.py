from grumpy_interfaces.srv import ArmCameraDetection

import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CompressedImage
import cv2
import cv_bridge
import numpy as np
import time
import arm_srvs.utils as utils

class ArmCameraService(Node):

    def __init__(self):
        super().__init__('arm_camera_srv')

        self.current_angles = utils.initial_thetas  # Keeps track of the angles of the servos published under /servo_pos_publisher

        self.image = None  # The image data from the arm camera

        self.bridge = cv_bridge.CvBridge()  # Bridge for converting between ROS messages and OpenCV images

        # Create the drop service
        self.srv = self.create_service(
            ArmCameraDetection, 
            '/arm_services/arm_camera', 
            self.camera_sequence
        )

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
            1
        )

        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/arm_camera/image_raw/compressed',
            self.get_image_data,
            1
        )

    
    def camera_sequence(self, request, response):
        """
        Args:
            
        Returns:
            response.pose   : Pose, the position of the object in the base_link frame
            response.success: bool, if the camera sequence was successful or not
        Other functions:
            Controlls the camera sequence
            Calls the publishing function which publishes the servo angles to the arm for each step in the sequence
        """

        step = "Start"
        end_strings = ["Success", "Failure"]

        while step not in end_strings:
            self._logger.info(f'{step}')  # Log the current step
            times = utils.times  # Set the times to the standard times
            match step:
                case "Start":  # Make sure the arm is in the initial position
                    thetas    = utils.initial_thetas
                    next_step = "ViewPosition"  # Next step

                case "ViewPosition":  # Set the arm to the view position
                    thetas    = utils.view_thetas  # Set the goal angles to the view thetas
                    next_step = "GetObjectPosition"  # Next step

                case "GetObjectPosition":  # Extract the position of the object from the arm camera
                    response.pose.position.x, response.pose.position.y = self.get_object_position()  # Get the position of the object
                    thetas    = utils.still_thetas  # Do not move the arm servos
                    next_step = "DrivePosition"  # Next step

                case "DrivePosition":  # Finish the viewing sequence by going back to the initial position
                    thetas    = utils.initial_thetas
                    next_step = "Success"  # End the FSM
            
            utils.check_angles_and_times(self, thetas, times)  # Assert that the angles and times are in the correct format
            
            if self.publish_angles(thetas, times):  # Publish the angles to the arm and check if the arm has moved to the correct angles
                step = next_step
            else:
                step = "Failure"
        
        self._logger.info(f'{step}')
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

    
    def get_image_data(self, msg:CompressedImage):
        """
        Args:
            msg: CompressedImage, required, the image message from the arm camera
        Returns:

        Other functions:
            Listens to the image message from the arm camera and sets a self variable to this data
        """

        self.image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')  # Convert the image message to an OpenCV image


    def get_object_position(self):
        """
        Args:

        Returns:
            x: float, the x-coordinate of the object in the base_link frame
            y: float, the y-coordinate of the object in the base_link frame
        Other functions:
            Uses the image data from the arm camera to detect objects and their positions
        """

        time.sleep(1.0)
        
        object_centers = []  # Initialize object centers
        x, y           = 0, 0  # No object found, set to 0, 0

        # undistorted_image = cv2.undistort(self.image, utils.intrinsic_mtx, utils.dist_coeffs)  # Undistort the image
        hsv_image         = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)  # Convert to HSV for color-based object detection

        mask            = self.create_masks(hsv_image)  # Create a combined mask for red, green, and blue objects
        mask            = cv2.medianBlur(mask, 5)
        mask            = self.clean_mask(mask)  # Clean each mask using morphological operations
        x, y            = self.find_objects(mask, hsv_image)  # Find object centers

        self._logger.info(f'get_object_position: Found {len(object_centers)} number of objects in total')

        if object_centers:  # If object(s) were found
            x, y = object_centers[0]  # Set the position of the first object found

        x, y = self.pixel_to_base_link(x, y)  # Transform the position to the base_link frame

        return x, y


    def create_masks(self, hsv_image):
        """
        Args:
            hsv_image: np.array, required, the image in HSV format
        Returns:
            x: float, the x-coordinate of the object in the base_link frame
            y: float, the y-coordinate of the object in the base_link frame
        Other functions:
            Uses the image data from the arm camera to detect objects and their positions
        """
        
        # Define HSV ranges
        lower_red1, upper_red1   = np.array([0, 120, 70]), np.array([10, 255, 255])
        lower_red2, upper_red2   = np.array([170, 120, 70]), np.array([180, 255, 255])
        lower_green, upper_green = np.array([35, 100, 50]), np.array([85, 255, 255])
        lower_blue, upper_blue   = np.array([100, 150, 50]), np.array([140, 255, 255])

        # Create masks for each color
        mask_red1  = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask_red2  = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_mask   = mask_red1 | mask_red2  # Combine both red masks
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        blue_mask  = cv2.inRange(hsv_image, lower_blue, upper_blue)

        return cv2.bitwise_or(red_mask, cv2.bitwise_or(green_mask, blue_mask))


    def clean_mask(self, mask):
        """
        Args:
            mask: np.array, required, the mask to be cleaned
        Returns:
            mask: np.array, the cleaned mask
        Other functions:
            
        """
        
        kernel = np.ones((5, 5), np.uint8)  # Define kernel for morphological operations
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Open to remove noise
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Close to fill holes

        return mask


    def find_objects(self, mask, image):
        """
        Args:
            mask: np.array, required, the mask to find objects in
        Returns:
            centers: list, the centers of the objects found
        Other functions:
            
        """

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Find contours in the mask
        centers     = []
        max_area    = 0  # Current max area of a found contour
        cx, cy      = 0, 0  # Zero if no center was found

        self._logger.info(f'find_objects: Found {len(contours)} of contours')

        for contour in contours:  # Loop through all contours
            if cv2.contourArea(contour) > max(max_area, 25):  # Uses the greatest found area 
                self._logger.info(f'find_objects: The size of the area of the object was {cv2.contourArea(contour)}')
                (cx, cy), radius = cv2.minEnclosingCircle(contour)  # Get the center and radius of the enclosing circle
                cx, cy, radius = int(cx), int(cy), int(radius)
                centers.append((cx, cy))
                self._logger.info(f'find_objects: cx = {cx} and cy = {cy}')

                # M = cv2.moments(contour)  # Calculate the moments of the contour
                # if M["m00"] != 0:  # Avoid division by zero
                #     cx = int(M["m10"] / M["m00"])  
                #     cy = int(M["m01"] / M["m00"])
                #     centers.append((cx, cy))
                #     self._logger.info(f'find_objects: cx = {cx} and cy = {cy}')

                # cv2.circle(image, (cx, cy), radius, (255, 255, 255), 2)
                # cv2.circle(image, (cx, cy), 5, (0, 0, 0), -1)

        self._logger.info(f'find_objects: Found {len(centers)} of centers')

        # cv2.imshow("Detected Objects", image)
        # cv2.waitKey(1)

        return (cx, cy)


    def pixel_to_base_link(self, x_pixel, y_pixel):
        """
        Args:
            x: float, required, the pixel x-coordinate of the object in the arm camera image
            y: float, required, the pixel y-coordinate of the object in the arm camera image
        Returns:
            x: float, the x-coordinate of the object in the base_link frame
            y: float, the y-coordinate of the object in the base_link frame
        Other functions:
            
        """

        fx, fy = utils.intrinsic_mtx[0, 0], utils.intrinsic_mtx[1, 1]  # Focal lengths from the intrinsic matrix
        cx, cy = utils.intrinsic_mtx[0, 2], utils.intrinsic_mtx[1, 2]  # Principal point from the intrinsic matrix

        x = (y_pixel - cy) * (utils.cam_pos.position.z / fy) + utils.cam_pos.position.x
        y = (x_pixel - cx) * (utils.cam_pos.position.z / fx) + utils.cam_pos.position.y

        return x, y

    
    def publish_angles(self, angles, times):
        """
        Args:
            angles: list, required, the angles for each servo to be set to
            times : list, required, the times for each servo to get to the given angle
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

        return utils.changed_thetas_correctly(angles, self.current_angles)  # Checks if the arm has moved to the correct angles

def main(args=None):
    rclpy.init()
    armCameraServie = ArmCameraService()

    try:
        rclpy.spin(armCameraServie)
    except KeyboardInterrupt:
        pass

    armCameraServie.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()