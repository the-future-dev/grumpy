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

        self.pick_center  = 400  # The pixel in the y direction where the object should end up for easy pick up
        self.min_area     = 1000  # The minimum required area of the object to be considered as an object
        self.comp_percent = 0.05  # The maximum distortion compensation for the x and y-direction when the camera is tilted
        self.slope_degree = 200.0  # How fast the maximum distortion compensation should increase with the distance to the image center
        self.cam_pose     = ''  # String to get the pose of the arm camera in base_link frame
        self.image        = None  # The image data from the arm camera
        self.bridge       = cv_bridge.CvBridge()  # Bridge for converting between ROS messages and OpenCV images

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
        found_object  = False  # Flag to check if an object was found
        self.cam_pose = request.cam_pose.data  # Get the pose of the arm camera in base_link frame
        x, y          = -1.0, -1.0  # The x and y position of the object

        if self.cam_pose in ['View Drop', 'View Left', 'View Right']:
            self.min_area = 500  # Set the minimum area to 1000 when trying to find an object
        else:
            self.min_area = 1000

        for _ in range(2):  # Try to get the object position a maximum of 3 times
            if request.box:
                x, y = self.get_box_position()

            else:
                x, y = self.get_object_position(grasp_position=request.grasp)  # Get the position of the object

            if x != -1.0 and y != -1.0:
                found_object = True  # If the object was found, set the flag to true
                self._logger.info(f'Found {'a box' if request.box else 'an object'} at: x: {x}, y: {y}')
                break

        if not found_object:
            self._logger.info(f'Arm camera could not find {'a box' if request.box else 'an object'}')

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


    def get_object_position(self, grasp_position:bool):
        """
        Args:

        Returns:
            x: float, the x-coordinate of the object in the base_link frame
            y: float, the y-coordinate of the object in the base_link frame
        Other functions:
            Uses the image data from the arm camera to detect object(s) and its/their position(s)
        """

        plushies = True
        x, y     = -1.0, -1.0  # Object position in base link
        image    = self.image.copy()  # Makes sure the same image is used for the whole function

        if plushies:
            undistorted_image = cv2.undistort(image, utils.intrinsic_mtx, utils.dist_coeffs)  # Undistort the image
            gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)

            # Apply Gaussian blurs
            blur1 = cv2.GaussianBlur(gray, (5, 5), 0)
            blur2 = cv2.GaussianBlur(gray, (9, 9), 0)

            # Compute DoG
            dog = cv2.absdiff(blur1, blur2)

            # Threshold the DoG image
            _, thresh = cv2.threshold(dog, 10, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            # _, binary_mask = cv2.threshold(dog, 10, 255, cv2.THRESH_BINARY)

            # # Optional: Clean up the result with morphological operations
            # kernel = np.ones((3, 3), np.uint8)
            # binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)

            # # Find contours
            # contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        else:
            undistorted_image = cv2.undistort(image, utils.intrinsic_mtx, utils.dist_coeffs)  # Undistort the image
            hsv_image         = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2HSV)  # Convert to HSV for eaiser color-based object detection

            mask = self.create_masks(hsv_image)  # Create a combined mask for red, green, and blue objects
            mask = cv2.medianBlur(mask, 5)  # Apply a median blur to the mask to reduce salt and pepper noise
            mask = self.clean_mask(mask)  # Clean each mask using morphological operations

        # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Find contours in the mask, only external contours
        contours, _ = cv2.findContours(mask if not plushies else thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # Find contours in the mask, all contours inc. internal
        areas = [cv2.contourArea(c) for c in contours]  # Calculate the area of each contour

        max_area = np.max(areas) if len(areas) > 0 else 0
        if len(areas) > 0 and max_area > self.min_area:  # If there are any contours found
            max_index = np.argmax(areas)  # Get the index of the largest contour
            contour   = contours[max_index]  # Choose the largest contour
            # self._logger.info(f'Max area was: {max_area}')

            (cx, cy), radius = cv2.minEnclosingCircle(contour)  # Get the center and radius of the enclosing circle
            cx, cy, radius   = int(cx), int(cy), int(radius)  # Convert to integers
            
            cv2.circle(image, (cx, cy), radius, (255, 255, 255), 2)  # Draw the enclosing circle in the image
            cv2.circle(image, (cx, cy), 5, (0, 0, 0), -1)  # Draw the center of the circle in the image

            for c in contours:
                (temp_cx, temp_cy), temp_radius = cv2.minEnclosingCircle(c)  # Get the center and radius of the enclosing circle
                temp_cx, temp_cy, temp_radius   = int(temp_cx), int(temp_cy), int(temp_radius / 2)  # Convert to integers

                cv2.circle(image, (temp_cx, temp_cy), temp_radius, (102, 255, 255), 2)  # Draw the enclosing circle in the image
                # cv2.circle(image, (temp_cx, temp_cy), 5, (153, 255, 153), -1)  # Draw the center of the circle in the image

            self._logger.info(f'get_object_position: cx = {cx} and cy = {cy}, number of countours: {len(contours)}')

            if grasp_position:
                x, y = self.pixel_to_adjust_in_base_link(cx, cy)
                cv2.circle(image, (int(utils.intrinsic_mtx[0, 2]), self.pick_center), 10, (0, 255, 0), -1)  # Draw the point to adjust to
            else:
                x, y = self.pixel_to_base_link_new(cx, cy)  # Transform the position to the base_link frame
        
        self._logger.info(f'Max area was: {max_area}')

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
        lower_red1, upper_red1   = np.array([0, 100, 100]), np.array([10, 255, 255])
        lower_red2, upper_red2   = np.array([160, 100, 100]), np.array([179, 255, 255])
        lower_green, upper_green = np.array([35, 100, 100]), np.array([85, 255, 255])
        lower_blue, upper_blue   = np.array([100, 100, 100]), np.array([140, 255, 255])
        lower_brown, upper_brown = np.array((10, 100, 20)), np.array((20, 255, 200))


        # Create masks for each color
        mask_red1  = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask_red2  = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_mask   = mask_red1 | mask_red2  # Combine both red masks
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        blue_mask  = cv2.inRange(hsv_image, lower_blue, upper_blue)
        brown_mask = cv2.inRange(hsv_image, lower_brown, upper_brown)

        return cv2.bitwise_or(red_mask, cv2.bitwise_or(green_mask, cv2.bitwise_or(brown_mask, blue_mask)))  # Combine all masks


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

        x, y      = -1.0, -1.0  # Box position in base link
        cx, cy    = 0.0, 0.0
        x_1_tot, x_2_tot, y_1_tot, y_2_tot = 0.0, 0.0, 0.0, 0.0
        num_lines = 0
        image     = self.image.copy()  # Makes sure the same image is used for the whole function

        undistorted_image = cv2.undistort(image, utils.intrinsic_mtx, utils.dist_coeffs)  # Undistort the image
        gray_scale        = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)  # Convert image to grayscale
        gray_equalized    = cv2.equalizeHist(gray_scale)  # Improve contrasts
        gray_blurred      = cv2.bilateralFilter(gray_equalized, 9, 75, 75)  # Add blur to simplify the image
        edges             = cv2.Canny(gray_blurred, 50, 100, apertureSize=3)  # Use canny edge detection ## was 50, 150

        lines_list = []
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=80, minLineLength=80, maxLineGap=5) ## was t=80, min=80
        # cv2.line(image, (220,350), (420,350), (0,255,100), 2)
        # cv2.line(image, (220,350), (220,480), (0,255,100), 2)
        # cv2.line(image, (420,350), (420,480), (0,255,100), 2)


        if lines is not None:
            for points in lines:  # Iterate over points
                x1, y1, x2, y2 = points[0]  # Extracted points nested in the list
                lines_list.append([(x1,y1),(x2,y2)])
                
                #if y1 < 330 or y2 < 330
                if (y1 < 350 or y2 < 350 or 
                    x1 < 220 or x1 > 420 or
                    x2 < 220 or x2 > 420):  # Ignore lines that are too close to the top or left of the image
                    num_lines += 1
                    cx        += ((x2 - x1) / 2 + x1)
                    cy        += ((y2 - y1) / 2 + y1)

                    x_1_tot += x1
                    x_2_tot += x2
                    y_1_tot += y1
                    y_2_tot += y2

                    cv2.line(image, (x1,y1), (x2,y2), (0,255,0), 2)

            if num_lines >= 1:
                cx /= num_lines
                cy /= num_lines
                # x_1_tot /= num_lines
                # x_2_tot /= num_lines
                # y_1_tot /= num_lines
                # y_2_tot /= num_lines
                # cx       = ((x_2_tot - x_1_tot) / 2 + x_1_tot)
                # cy       = ((y_2_tot - y_1_tot) / 2 + y_1_tot)

                self._logger.info(f'num_lines: {num_lines}')
                
                cv2.circle(image, (int(cx), int(cy)), 10, (0, 0, 255), -1)

                # x, y = self.pixel_to_base_link_general(cx, cy)  # Transform the position to the base_link frame
                x, y = self.pixel_to_base_link_new(cx, cy)

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
    

    def pixel_to_base_link_new(self, x_pixel, y_pixel):
        """
        Args:
            x: float, required, the pixel x-coordinate of the object in the arm camera image
            y: float, required, the pixel y-coordinate of the object in the arm camera image
        Returns:
            x: float, the x-coordinate of the object in the base_link frame
            y: float, the y-coordinate of the object in the base_link frame
        Other functions:
            
        """

        angles, cam_pose = utils.cam_poses[self.cam_pose]  # Get the pose of the arm camera in base_link frame

        # self._logger.info(f'cam pose: {cam_pose.position.x, cam_pose.position.y, cam_pose.position.z}')
        fx, fy = utils.intrinsic_mtx[0, 0], utils.intrinsic_mtx[1, 1]  # Focal lengths from the intrinsic matrix
        cx, cy = utils.intrinsic_mtx[0, 2], utils.intrinsic_mtx[1, 2]  # Principal point from the intrinsic matrix

        # Convert pixel coordinates to cartesian, inverted because the rows and columns increase opposite to the base_link frame

        x_image                   = - (y_pixel - cy) * (cam_pose.position.z / fy)
        y_image                   = - (x_pixel - cx) * (cam_pose.position.z / fx)
        distortion_compensation_x = 1 - self.comp_percent * (self.slope_degree ** (abs(y_pixel - cy) / cy - 1) - 1 / self.slope_degree)
        distortion_compensation_y = 1 - self.comp_percent * (self.slope_degree ** (abs(x_pixel - cx) / cx - 1) - 1 / self.slope_degree)

        # self._logger.info(f'x_image: {x_image}, y_image: {y_image}')

        # Calculate angle and distance to x_image and y_image from the camera
        angle    = np.rad2deg(np.arctan2(y_image, x_image))  # Angle to the object in radians
        distance = np.sqrt(x_image**2 + y_image**2)  # Distance to the object in meters

        if (sum_angles := (utils.theta_arm_cam + np.sum(angles[1:]))) % 90 == 0:
            center_offset = 0

        else:
            center_offset = cam_pose.position.z / np.tan(np.deg2rad(sum_angles))

        # self._logger.info(f'sum angles: {sum_angles}, tan: {np.tan(np.deg2rad(sum_angles))}')
        # self._logger.info(f'angle: {angle}, dist: {distance}, center: {center_offset}')

        # Calculate the x and y coordinates in the base_link frame
        x = (cam_pose.position.x +
             center_offset * np.cos(np.deg2rad(angles[0])) +
             distance * np.cos(np.deg2rad(angles[0] + angle)) * distortion_compensation_x)
        y = (cam_pose.position.y +
             center_offset * np.sin(np.deg2rad(angles[0])) +
             distance * np.sin(np.deg2rad(angles[0] + angle)) * distortion_compensation_y)

        return x, y  # x, y in base_link
    

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
    

    def pixel_to_base_link_general(self, x_pixel, y_pixel):
        """
        Args:
            x: float, required, the pixel x-coordinate of the object in the arm camera image
            y: float, required, the pixel y-coordinate of the object in the arm camera image
        Returns:
            x: float, the amount the x-coordinate of the object in the base_link frame should be adjusted
            y: float, the amount the y-coordinate of the object in the base_link frame should be adjusted
        Other functions:
            
        """

        fx, fy = utils.intrinsic_mtx[0, 0], utils.intrinsic_mtx[1, 1]  # Focal lengths from the intrinsic matrix
        cx, cy = utils.intrinsic_mtx[0, 2], utils.intrinsic_mtx[1, 2]  # Principal point from the intrinsic matrix

        base_link_x = - (y_pixel - cy) * (utils.cam_r_t_box['z'] / fy) + utils.cam_r_t_box['x'] + np.tan(np.deg2rad(utils.theta_cam_z)) * utils.cam_r_t_box['z']
        base_link_y = - (x_pixel - cx) * (utils.cam_r_t_box['z'] / fx) + utils.cam_r_t_box['y']

        return base_link_x, base_link_y  # x, y in base_link
    

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