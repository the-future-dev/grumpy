from grumpy_interfaces.srv import ArmCameraDetection

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import CompressedImage, Image
import cv2
import cv_bridge
import numpy as np
import arm_srvs.utils as utils
from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation
from PIL import Image as PILImage
import torch
import os

class ArmCameraService(Node):

    def __init__(self):
        super().__init__('arm_camera_srv')

        self.image  = None  # The image data from the arm camera
        self.bridge = cv_bridge.CvBridge()  # Bridge for converting between ROS messages and OpenCV images
        
        # Initialize the segmentation model and preprocessor
        self.preprocessor = AutoImageProcessor.from_pretrained("google/deeplabv3_mobilenet_v2_1.0_513")
        self.model = AutoModelForSemanticSegmentation.from_pretrained("google/deeplabv3_mobilenet_v2_1.0_513")
        
        # Setup filepath for saving detection images
        self.output_dir = os.path.join(os.path.expanduser('~'), 'dd2419_ws', 'outputs')
        os.makedirs(self.output_dir, exist_ok=True)  # Create directory if it doesn't exist
        self.filepath = os.path.join(self.output_dir, 'rgb_detection.png')
        
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
            
        Returns:
            response.pose   : Pose, the position of the object in the base_link frame
            response.success: bool, if the camera sequence was successful or not
        Other functions:
            
        """

        found_object = False  # Flag to check if an object was found
        x, y         = 0.0, 0.0  # The x and y position of the object

        for _ in range(3):  # Try to get the object position a maximum of 3 times
            x, y = self.get_object_position()  # Get the position of the object
            if x != 0.0:
                found_object = True  # If the object was found, set the flag to true
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


    def get_object_position(self):
        """
        Args:

        Returns:
            x: float, the x-coordinate of the object in the base_link frame
            y: float, the y-coordinate of the object in the base_link frame
        Other functions:
            Uses deep learning segmentation to detect objects and their positions
        """
        
        cx, cy = 0, 0  # No object found, set to 0, 0
        image  = self.image  # Makes sure the same image is used for the whole function

        if image is None:
            self._logger.info(f'get_object_position: No image available')
            return 0.0, 0.0

        undistorted_image = cv2.undistort(image, utils.intrinsic_mtx, utils.dist_coeffs)  # Undistort the image
        
        # Convert OpenCV image to PIL format for the model
        pil_image = PILImage.fromarray(cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2RGB))
        
        # Find object using segmentation model
        self.get_logger().info('Starting plushy detection with segmentation model...')
        cx, cy = self.find_plushy_position(pil_image)
        self.get_logger().info('Completed plushy detection')
        
        detection_img = image.copy()  # Create a copy for drawing
        
        if cx is not None and cy is not None:
            self._logger.info(f'get_object_position: cx = {cx} and cy = {cy}')
            
            # Draw detection on the image with a star marker
            # Draw circle
            cv2.circle(detection_img, (cx, cy), 15, (255, 255, 255), 2)
            
            # Draw star (*) shape
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(detection_img, '*', (cx-10, cy+10), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
            
            # Save the image with detection to file
            cv2.imwrite(self.filepath, detection_img)
            self._logger.info(f'Saved detection image to: {self.filepath}')
            
            x, y = self.pixel_to_base_link(cx, cy)  # Transform the position to the base_link frame
        else:
            self._logger.info(f'get_object_position: NO OBJECTS FOUND')
            x, y = 0.0, 0.0  # No object found, set to 0, 0
            
        self.publish_image(detection_img)  # Publish the image with or without the detected object(s)

        return x, y
        
    def find_plushy_position(self, image):
        """
        Find the position of the plushy in the given image.
        
        Args:
            image: PIL Image
        
        Returns:
            tuple: (center_x, center_y) coordinates in the original image space,
                  or (None, None) if the plushy cannot be found
        """
        # Process the image
        inputs = self.preprocessor(images=image, return_tensors="pt")
        with torch.no_grad():  # Add no_grad for inference efficiency
            outputs = self.model(**inputs)
        predicted_mask = self.preprocessor.post_process_semantic_segmentation(outputs)[0]
        
        # Find the unique classes in the segmentation mask
        unique_classes = torch.unique(predicted_mask).numpy()
        
        # Find the class with the most pixels (excluding background class 0)
        class_counts = {}
        for cls in unique_classes:
            if cls == 0:  # Skip background
                continue
            count = (predicted_mask == cls).sum().item()
            class_counts[cls] = count
        
        # Determine the plushy class
        if class_counts:
            plushy_class = max(class_counts, key=class_counts.get)
        else:
            # No foreground objects found
            return None, None
        
        # Create a binary mask for the plushy
        plushy_mask = (predicted_mask == plushy_class).numpy()
        
        # Get dimensions
        img_height, img_width = image.height, image.width
        mask_height, mask_width = plushy_mask.shape
        
        # Find the center of the plushy
        y_indices, x_indices = np.where(plushy_mask)
        if len(y_indices) > 0 and len(x_indices) > 0:
            # Find center in mask coordinates
            mask_center_y = int(np.mean(y_indices))
            mask_center_x = int(np.mean(x_indices))
            
            # Scale to original image coordinates
            center_y = int(mask_center_y * (img_height / mask_height))
            center_x = int(mask_center_x * (img_width / mask_width))
            
            return center_x, center_y
        else:
            return None, None


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