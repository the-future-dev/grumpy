import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage, Image
import cv2
import cv_bridge
import numpy as np
import time

class DetectionArmCam(Node):

    def __init__(self):
        super().__init__('detection_arm_cam')

        self.pick_center = 400
        self.bridge      = cv_bridge.CvBridge()  # Bridge for converting between ROS messages and OpenCV images
        self.num_images  = 0  # Number of images processed

        # Camera parameters for calibration and undistortion of the image
        self.intrinsic_mtx = np.array([[438.783367, 0.000000, 305.593336],
                                       [0.000000, 437.302876, 243.738352],
                                       [0.000000, 0.000000, 1.000000]])
        self.dist_coeffs   = np.array([-0.361976, 0.110510, 0.001014, 0.000505, 0.000000])

        # Create publishers and a subscriber for images and object detection from the arm camera
        self.image_publisher_ = self.create_publisher(
            Image, 
            '/detection_arm_cam/image_raw/object_in_gripper',
            10
        )

        self.object_in_gripper_pub = self.create_publisher(
            Bool,
            '/detection_arm_cam/object_in_gripper',
            1
        )

        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/arm_camera/image_raw/compressed',
            self.process_image,
            10
        )

        self._logger.info('Detection arm camera up and running')
    
    def process_image(self, msg:CompressedImage):
        """
        Args:
            msg: CompressedImage, required, the image message from the arm camera
        Returns:

        Other functions:
            Gets the image data from the arm camera and converts it to an OpenCV image
            Calls the function to see if an object is in the gripper
            Publishes if an object is in the gripper or not
        """
        
        self.num_images += 1  # Increment the number of images processed
        if self.num_images % 5 == 0:  # Every third image check if an object is in the gripper
            pub_msg = Bool()

            image        = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')  # Convert the image message to an OpenCV image
            pub_msg.data = self.object_in_gripper(image)  # Call the function to see if an object is in the gripper

            self.object_in_gripper_pub.publish(pub_msg)  # Publish the result to the object_in_gripper topic


    def object_in_gripper(self, image):
        """
        Args:
            image: np.array, required, the image from the arm camera
        Returns:
            object_found: bool, True if an object is found in the gripper, False otherwise
        Other functions:
            Calls the function to publish the image if an object is detected
        """
        
        object_found = False  # Initialize the object_found variable to False

        height        = image.shape[0]  # Get the height of the image
        offset_y      = int(height * (2/3))  # Calculate the offset to crop the image
        cropped_image = image[offset_y:, :]  # Crop the image to the lower third to reduce processing time

        hsv_image     = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)  # Convert to HSV for eaiser color-based object detection

        mask = self.create_masks(hsv_image)  # Create a combined mask for red, green, and blue objects
        mask = cv2.medianBlur(mask, 5)  # Apply a median blur to the mask to reduce salt and pepper noise
        mask = self.clean_mask(mask)  # Clean each mask using morphological operations

        # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Find contours in the mask, only external contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # Find contours in the mask, all contours inc. internal
        areas = [cv2.contourArea(c) for c in contours]  # Calculate the area of each contour

        if len(areas) > 0 and np.max(areas) > 500:  # If there are any contours found
            max_index = np.argmax(areas)  # Get the index of the largest contour
            contour   = contours[max_index]  # Choose the largest contour

            (cx, cy), radius = cv2.minEnclosingCircle(contour)  # Get the center and radius of the enclosing circle
            cx, cy, radius   = int(cx), int(cy) + offset_y, int(radius)  # Convert to integers
            
            cv2.circle(image, (cx, cy), radius, (255, 255, 255), 2)  # Draw the enclosing circle in the image
            cv2.circle(image, (int(self.intrinsic_mtx[0, 2]), self.pick_center), 10, (0, 255, 0), -1)  # Compare adjust center to actual center

            object_found = True  # Set the object_found variable to True
            
            self.publish_image(image)  # Publish the image with or without the detected object(s)

        return object_found


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
            mask: np.array, required, the mask to be cleaned
        Returns:
            mask: np.array, the cleaned mask
        Other functions:
            
        """
        
        kernel = np.ones((3, 3), np.uint8)  # Define kernel for morphological operations
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Open to remove noise
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Close to fill holes

        return mask
    

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
    detectionArmCam = DetectionArmCam()

    try:
        rclpy.spin(detectionArmCam)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()