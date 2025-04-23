from grumpy_interfaces.srv import ArmCameraDetection

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import CompressedImage, Image
import cv2
import cv_bridge
import numpy as np
import arm_srvs.utils as utils
import time

class ArmCameraService(Node):

    def __init__(self):
        super().__init__('arm_camera_srv')

        self.pick_center = 400
        self.image       = None  # The image data from the arm camera
        self.bridge      = cv_bridge.CvBridge()  # Bridge for converting between ROS messages and OpenCV images

        # Create group for the service and subscriber that will run on different threads
        self.service_cb_group    = MutuallyExclusiveCallbackGroup()
        self.subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        # Create the arm camera service
        self.srv = self.create_service(
            ArmCameraDetection, 
            '/arm_services/arm_camera', 
            self.camera_sequence,
            callback_group=self.service_cb_group
        )

        # Create a publisher and subscriber for images from the arm camera
        self.image_publisher_ = self.create_publisher(
            Image, 
            '/arm_services/object_tracking/image_raw', 
            10
        )

        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/arm_camera/image_raw/compressed',
            self.get_image_data,
            10,
            callback_group=self.subscriber_cb_group
        )

    
    def camera_sequence(self, request, response):
        """
        Args:
            request.grasp   : bool, required, if the arm is in the grasp position or not
            request.box     : bool, required, if the arm camera is looking for the box position or not
        Returns:
            response.pose   : Pose, the position of the object in the base_link frame
            response.success: bool, if the camera sequence was successful or not
        Other functions:
            
        """

        time.sleep(0.5)  # Let the arm camera picture stabilize before trying to find an object
        found_object = False  # Flag to check if an object was found
        x, y         = 0.0, 0.0  # The x and y position of the object

        for _ in range(3):  # Try to get the object position a maximum of 3 times
            if request.box:
                x, y = self.get_box_position()
            else:
                x, y = self.get_object_position(request.grasp)  # Get the position of the object
            if x > 0.0:
                found_object = True  # If the object was found, set the flag to true
                self._logger.info(f'Found {'box' if request.box else 'object'} at: x: {x}, y: {y}')
                break

        response.success         = found_object  # Set the success flag to true if the object was found
        response.pose.position.x = x  # Set the position of the object in the response
        response.pose.position.y = y  # Set the position of the object in the response
        
        return response

    
    def get_image_data(self, msg:CompressedImage):
        """
        Args:
            msg: CompressedImage, required, the image message from the arm camera
        Returns:

        Other functions:
            Gets the image data from the arm camera and converts it to an OpenCV image
        """
        
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')  # Convert the image message to an OpenCV image


    def get_object_position(self, grasp_position):
        """
        Args:

        Returns:
            x: float, the x-coordinate of the object in the base_link frame
            y: float, the y-coordinate of the object in the base_link frame
        Other functions:
            Uses the image data from the arm camera to detect object(s) and its/their position(s)
        """
        
        x, y   = 0.0, 0.0  # Object position in base link
        image  = self.image.copy()  # Makes sure the same image is used for the whole function

        undistorted_image = cv2.undistort(image, utils.intrinsic_mtx, utils.dist_coeffs)  # Undistort the image
        hsv_image         = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2HSV)  # Convert to HSV for eaiser color-based object detection

        mask = self.create_masks(hsv_image)  # Create a combined mask for red, green, and blue objects
        mask = cv2.medianBlur(mask, 5)  # Apply a median blur to the mask to reduce salt and pepper noise
        mask = self.clean_mask(mask)  # Clean each mask using morphological operations

        # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Find contours in the mask, only external contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # Find contours in the mask, all contours inc. internal
        areas = [cv2.contourArea(c) for c in contours]  # Calculate the area of each contour

        if len(areas) > 0:  # If there are any contours found
            max_index = np.argmax(areas)  # Get the index of the largest contour
            contour   = contours[max_index]  # Choose the largest contour

            (cx, cy), radius = cv2.minEnclosingCircle(contour)  # Get the center and radius of the enclosing circle
            cx, cy, radius   = int(cx), int(cy), int(radius)  # Convert to integers
            
            cv2.circle(image, (cx, cy), radius, (255, 255, 255), 2)  # Draw the enclosing circle in the image
            cv2.circle(image, (cx, cy), 5, (0, 0, 0), -1)  # Draw the center of the circle in the image

            self._logger.info(f'get_object_position: cx = {cx} and cy = {cy}')

            if grasp_position:
                x, y = self.pixel_to_adjust_in_base_link(cx, cy)
                cv2.circle(image, (int(utils.intrinsic_mtx[0, 2]), self.pick_center), 10, (0, 255, 0), -1)  # Draw the point to adjust to
            else:
                if cy < 420:
                    x, y = self.pixel_to_base_link(cx, cy)  # Transform the position to the base_link frame
        
        if x == 0.0 and y == 0.0:
            self._logger.info(f'get_object_position: NO OBJECTS FOUND')
            
        self.publish_image(image)  # Publish the image with or without the detected object(s)

        return x, y


    def create_masks(self, hsv_image):
        """
        Args:
            hsv_image: np.array, required, the image in HSV format
        Returns:
            mask: np.array, the combined mask for red, green, and blue colors
        Other functions:

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

        return cv2.bitwise_or(red_mask, cv2.bitwise_or(green_mask, blue_mask))  # Combine all masks


    def clean_mask(self, mask):
        """
        Args:
            mask: np.array, required, the image to be cleaned
        Returns:
            mask: np.array, the cleaned mask
        Other functions:
            
        """
        
        kernel = np.ones((3, 3), np.uint8)  # Define kernel for morphological operations
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Open to remove noise
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Close to fill holes

        return mask
    

    def get_box_position(self):
        """
        Args:
            None
        Returns:
            x: float, the x-coordinate of the box in the base_link frame
            y: float, the y-coordinate of the box in the base_link frame
        Other functions:
            Uses the image data from the arm camera to detect the box position
        """

        x, y      = 0.30, 0.0  # Box position in base link
        cx, cy    = 0.0, 0.0
        num_lines = 0
        image     = self.image.copy()  # Makes sure the same image is used for the whole function

        undistorted_image = cv2.undistort(image, utils.intrinsic_mtx, utils.dist_coeffs)  # Undistort the image
        gray_scale        = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)  # Convert image to grayscale
        gray_equalized    = cv2.equalizeHist(gray_scale)  # Improve contrasts
        gray_blurred      = cv2.bilateralFilter(gray_equalized, 9, 75, 75)  # Add blur to simplify the image
        edges             = cv2.Canny(gray_blurred, 50, 150, apertureSize=3)  # Use canny edge detection

        lines_list = []
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=100, maxLineGap=15)

        if lines is not None:
            for points in lines:  # Iterate over points
                x1, y1, x2, y2 = points[0]  # Extracted points nested in the list
                lines_list.append([(x1,y1),(x2,y2)])

                self._logger.info(f'{x1}, {x2}, {y1}, {y2}')

                if y1 < 330 and y2 < 330:
                    num_lines += 1
                    cx        += ((x2 - x1) / 2 + x1)
                    cy        += ((y2 - y1) / 2 + y1)

                    cv2.line(image, (x1,y1), (x2,y2), (0,255,0), 2)

            cx /= num_lines
            cy /= num_lines

            cv2.circle(image, (int(cx), int(cy)), 10, (0, 0, 255), -1)

            self._logger.info(f'Box is in pixels x: {cx} and y: {cy}')

        self.publish_image(image)

        return x, y  # Return the x and y coordinates of the box position


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

        # Calculate the x and y coordinates in the base_link frame, inverted (minus) because the rows and columns increase opposite 
        # to the base_link frame
        x = - (y_pixel - cy) * (utils.cam_pos.position.z / fy) + utils.cam_pos.position.x
        y = - (x_pixel - cx) * (utils.cam_pos.position.z / fx) + utils.cam_pos.position.y

        return x, y
    

    def pixel_to_adjust_in_base_link(self, x_pixel, y_pixel):
        """
        Args:
            x: float, required, the pixel x-coordinate of the object in the arm camera image
            y: float, required, the pixel y-coordinate of the object in the arm camera image
        Returns:
            x: float, the amount the x-coordinate of the object in the base_link frame should be adjusted
            y: float, the amount the y-coordinate of the object in the base_link frame should be adjusted
        Other functions:
            
        """

        cx, cy = utils.intrinsic_mtx[0, 2], self.pick_center  # Principal point from the intrinsic matrix
        pixels_per_meter = 5000  # The number of pixels per meter at the current placement of the arm camera, used for the adjustment

        adjustment_to_x = - (y_pixel - cy) / pixels_per_meter
        adjustment_to_y = - (x_pixel - cx) / pixels_per_meter

        return adjustment_to_x, adjustment_to_y
    

    def publish_image(self, image):
        """
        Args:
            image: np.array, required, the image with the object(s) drawn to be published
        Returns:
        
        Other functions:
            Publishes the image to an image topic
        """
         
        image_message = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')  # Convert the image to a ROS message
        self.image_publisher_.publish(image_message)


def main(args=None):
    rclpy.init()
    armCameraService = ArmCameraService()

    # Use MultiThreadedExecutor to allow concurrent callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(armCameraService)

    try:
        executor.spin()
    except KeyboardInterrupt:
        armCameraService.destroy_node()
    finally:
        armCameraService.destroy_node()
        rclpy.shutdown()

    rclpy.shutdown()

if __name__ == '__main__':
    main()