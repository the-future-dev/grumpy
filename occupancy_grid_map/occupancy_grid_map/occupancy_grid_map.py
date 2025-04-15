
import rclpy
from rclpy.node import Node
import rclpy.time
import numpy as np
from sensor_msgs.msg import LaserScan
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf_transformations
from shapely.geometry.polygon import Polygon
from shapely.geometry import Point
import matplotlib.pyplot as plt
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from scipy.ndimage import binary_dilation
from grumpy_interfaces.msg import ObjectDetection1DArray, ObjectDetection1D
from occupancy_grid_map.workspace_utils import Workspace
from time import sleep
from matplotlib.colors import BoundaryNorm
from nav_msgs.msg import OccupancyGrid

class OccupancyGridMapNode(Node):
    
    #Initialzie oppucancy grid node
    def __init__(self):
        super().__init__('occupancy_grid_map_node') 

        #Inititalize workspace
        self.ws_utils = Workspace()
        self.workspace = self.ws_utils.workspace
        self.polygon = Polygon(self.workspace.T)
        
        #Values for grid cells
        self.unknown = -1
        self.free = 0
        self.obstacle = 1
        self.outside_inflate = 2
        self.outside = 3
        self.object_box = 4

        self.grid = self.ws_utils.create_grid()
        self.counter = 0
        self.phase = 'collection'  ###############    PHASE    ##########

        #Transfrom between lidar link and map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        #Create publisher fo grid
        self.grid_pub = self.create_publisher(Int16MultiArray, 'map/gridmap', 10)

        #Fill outside grid with 
        self.fill_outside_grid()

        if self.phase == 'collection':
            self.map = self.ws_utils.map
            self.map_to_grid(self.map, self.object_box)
        elif self.phase == 'exploration':
            self.obstacle_subscription = self.create_subscription(ObjectDetection1DArray, 'object_mapping/object_poses', self.object_cb, 1)

        #Subscribe to both lidar scan
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 1)


    #Function which fills the space outside workspace as occupied, very slow now but have not succeded with other
    def fill_outside_grid(self):

        #Go through all points to see if inside or outside, making the initialization slow
        for i in range(self.ws_utils.grid_xlength):
            for j in range(self.ws_utils.grid_ylength):

                x_ind, y_ind = self.ws_utils.convert_grid_to_map(i, j)

                point = Point(x_ind, y_ind)
                if not self.polygon.contains(point):
                    self.grid[j, i] = self.outside

        return
    
    def object_cb(self, msg:ObjectDetection1DArray):

        x_list = []
        y_list = []
        value = self.object_box

        for obj in msg.detected_objects:
            x_list.append(100*obj.pose.position.x)
            y_list.append(100*obj.pose.position.y)
    
        points = np.array([x_list,
                            y_list])


        self.map_to_grid(points, value)
        self.publish_grid()

    #Lidar callback calculates detected object from laser scan, would implement to only run every xth time
    def lidar_cb(self, msg:LaserScan):

        self.counter += 1

        if self.counter % 5 != 0:
            return
        
        #Get data from message
        min_angle = msg.angle_min
        lower_bound = 0.30
        upper_bound = msg.range_max
        inc = msg.angle_increment
        ranges = np.array(msg.ranges)
 
        #Create mask to filter out ranges to far away or to close
        indices = np.arange(len(ranges))
        boundary_mask = (ranges >= lower_bound) & (ranges <= upper_bound)

        indices = indices[boundary_mask] #Filter out scans with ranges to large and small
        ranges = ranges[boundary_mask]
        angles = min_angle + indices * inc

        lidar_x = ranges * np.cos(angles) #Calculate lidar point in lidar frame
        lidar_y = ranges * np.sin(angles)

        self.point_to_map(msg, lidar_x, lidar_y, self.obstacle)
        free_x, free_y, unknown_x, unknown_y = self.raytrace_float(lidar_x, lidar_y)
        self.point_to_map(msg, free_x, free_y, self.free)

        self.point_to_map(msg, unknown_x, unknown_y, self.unknown)
        self.publish_grid()

    #Transform point from one message to map
    def point_to_map(self, msg, x_points, y_points, value):

        #Looking for transform from msg-frame to map
        tf_future = self.tf_buffer.wait_for_transform_async('map', msg.header.frame_id, msg.header.stamp)
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform{ex}')
            return

        #Taking transformation and create transformation matrix
        tf_translation = np.array([tf.transform.translation.x, 
                                   tf.transform.translation.y,
                                   tf.transform.translation.z])
        tf_quaternion = np.array([tf.transform.rotation.x,
                                  tf.transform.rotation.y,
                                  tf.transform.rotation.z,
                                  tf.transform.rotation.w])
        tf_rotation = tf_transformations.quaternion_matrix(tf_quaternion)[:3, :3]

        tf_matrix = np.zeros((4,4))
        tf_matrix[3, 3] = 1
        tf_matrix[:3, :3] = tf_rotation
        tf_matrix[:3, 3] = tf_translation

        #create array to multiply transform to, homogenous added 1
        measure_points = np.vstack([x_points, y_points, np.zeros_like(x_points), np.ones_like(y_points)])
    
        transformed_points = 100*(tf_matrix @ measure_points) #Scale to centimeters

        self.map_to_grid(transformed_points, value)

    #Convert map coordinate to grid indices and set as occupied or free
    def map_to_grid(self, map_points, value):

        x_map = map_points[0, :]
        y_map = map_points[1, :]

        x_grid_points, y_grid_points = self.ws_utils.convert_map_to_grid(x_map, y_map)

        x_grid_points, y_grid_points = self.filter_points(x_grid_points, y_grid_points, value)

        self.grid[y_grid_points, x_grid_points] = value

        if value == self.object_box:
            self.inflate_grid(x_grid_points, y_grid_points, value)

    def inflate_grid(self, x_grid_points, y_grid_points, value):
        #Function which inflated the new points and a new grid and then merges it with old grid

        inflate_size = int(6/self.ws_utils.resolution)
        inflate_matrix = np.ones((2 * inflate_size + 1, 2 * inflate_size + 1))
        
        new_grid = np.zeros_like(self.grid)
        new_grid[y_grid_points, x_grid_points] = 1
        new_grid = binary_dilation(new_grid, structure=inflate_matrix)*value

        mask_zeros = new_grid == 0
        new_grid[mask_zeros] = -1

        self.grid = np.maximum(self.grid, new_grid)

    def publish_grid(self):

        #Publish grid
        msg_grid = Int16MultiArray()
        msg_grid.data = self.grid.flatten().tolist()
        
        dim1 = MultiArrayDimension()
        dim2 = MultiArrayDimension()
        
        dim1.label = 'rows'
        dim2.label = 'columns'

        dim1.size = self.grid.shape[0]
        dim2.size = self.grid.shape[1]

        msg_grid.layout.dim = [dim1, dim2]

        cmap = plt.cm.get_cmap('Paired', 6)
        norm = BoundaryNorm([-1, 0, 1, 2, 3, 4, 5], cmap.N)
        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid, cmap=cmap, norm=norm, interpolation='nearest', origin='lower')
        cbar = plt.colorbar()
        cbar.set_ticks([-1, 0, 1, 2, 3, 4])
        plt.savefig('/home/group5/dd2419_ws/outputs/fantastic_map')
        plt.close()

        self.grid_pub.publish(msg_grid)
    
    def filter_points(self, x_grid_points, y_grid_points, value):

        #Mask to filter out of bounds points
        mask_in_bounds = (x_grid_points < self.grid.shape[1]) & (y_grid_points < self.grid.shape[0]) & (x_grid_points > 0) & (y_grid_points > 0) 
        x_grid_points = x_grid_points[mask_in_bounds]
        y_grid_points = y_grid_points[mask_in_bounds]

        #Mask to filter object and boxes and outside workspace
        mask_workspace_object_box = self.grid[y_grid_points, x_grid_points] >= self.outside
        x_grid_points = x_grid_points[~mask_workspace_object_box]
        y_grid_points = y_grid_points[~mask_workspace_object_box]

        # #If placing an object or box we do not want to care about where in workspace
        # if value == self.object_box:
        #     return x_grid_points, y_grid_points

        #Mask to filter out free space when we want o set unknown space
        if value == self.unknown:
            mask_not_free = self.grid[y_grid_points, x_grid_points] != self.free
            x_grid_points = x_grid_points[mask_not_free]
            y_grid_points = y_grid_points[mask_not_free]

        return x_grid_points, y_grid_points

    #Creating a linspace between 
    def raytrace_float(self, lidar_x, lidar_y):

        start = np.zeros_like(lidar_x)
        x_line = np.linspace(start, lidar_x, 10000)
        y_line = np.linspace(start, lidar_y, 10000)

        x_line = x_line[:-100, :]
        y_line = y_line[:-100, :]

        x_free = np.concatenate(x_line)
        y_free = np.concatenate(y_line)

        mask_rgbd_scope = (x_free > 0.2) & (x_free < 1.4) & (x_free > abs(0.7*y_free))  #Cone shape
        
        x_unknown = x_free[~mask_rgbd_scope]
        y_unknown = y_free[~mask_rgbd_scope]

        x_free = x_free[mask_rgbd_scope]
        y_free = y_free[mask_rgbd_scope]

        return x_free, y_free, x_unknown, y_unknown


    # def publish_grid(self):
    #     """Creates and publishes the OccupancyGrid message."""
    #     try:
    #         # Create the OccupancyGrid message
    #         grid_msg = OccupancyGrid()

    #         # --- Fill Header ---
    #         grid_msg.header.stamp = self.get_clock().now().to_msg()
    #         grid_msg.header.frame_id = 'map' # Grid is in the map frame

    #         # --- Fill Info ---
    #         grid_msg.info.map_load_time = grid_msg.header.stamp
    #         grid_msg.info.resolution = self.ws_utils.resolution # Meters per cell
    #         grid_msg.info.width = self.grid.shape[1] # Columns
    #         grid_msg.info.height = self.grid.shape[0] # Rows

    #         # Origin pose (position and orientation) of the grid cell (0, 0) in the map frame
    #         grid_msg.info.origin = Pose()
    #         grid_msg.info.origin.position.x = self.ws_utils.origin_x # Meters
    #         grid_msg.info.origin.position.y = self.ws_utils.origin_y # Meters
    #         grid_msg.info.origin.position.z = 0.0 # Assuming 2D map
    #         # Orientation (usually identity quaternion for a non-rotated map)
    #         grid_msg.info.origin.orientation.x = self.ws_utils.origin_orientation_x
    #         grid_msg.info.origin.orientation.y = self.ws_utils.origin_orientation_y
    #         grid_msg.info.origin.orientation.z = self.ws_utils.origin_orientation_z
    #         grid_msg.info.origin.orientation.w = self.ws_utils.origin_orientation_w

    #         # --- Prepare Grid Data ---
    #         # The data needs to be a flattened list/array of int8 values
    #         # Map your internal grid values to OccupancyGrid standard (-1, 0, 100)
    #         # Make a copy to avoid modifying the internal grid representation directly if needed
    #         publish_grid_data = self.grid.copy()

    #         # Map custom values to standard OccupancyGrid values
    #         # Example: Map custom_outside and custom_object_box to occupied (100)
    #         publish_grid_data[publish_grid_data == self.custom_object_box] = self.obstacle # Map object box to 100
    #         # Add mappings for other custom values if you have them (e.g., self.custom_outside)
    #         # publish_grid_data[publish_grid_data == self.custom_outside] = self.obstacle # Or -1 if unknown

    #         # Ensure values are within the valid range [-1, 100] and correct type (int8)
    #         # Clip values just in case (optional, good practice)
    #         np.clip(publish_grid_data, -1, 100, out=publish_grid_data)

    #         # Flatten the grid data in row-major order (standard for OccupancyGrid)
    #         grid_msg.data = publish_grid_data.flatten().astype(np.int8).tolist()

    #         # --- Publish the message ---
    #         self.grid_pub.publish(grid_msg)

    #         # --- Optional: Save image (keep if useful for debugging) ---
    #         # self.save_grid_image() # Call the save image function

    #     except Exception as e:
    #         self.get_logger().error(f"Error in publish_grid: {e}")

def main():
    rclpy.init()
    node = OccupancyGridMapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()


######   IF PLOTTING    ########
# cmap = plt.cm.get_cmap('viridis', 5)
# grid_values = {
#     self.unknown: 'gray',
#     self.free: 'white',
#     self.obstacle: 'black',
#     self.object_box: 'blue',
#     self.outside: 'lightgray'
# }

# plt.figure(figsize=(10, 10))
# plt.imshow(self.grid, cmap=cmap, interpolation='nearest', origin='lower')
# cbar = plt.colorbar()
# cbar.set_ticks([0, 1, 2, 3, 4])
# plt.savefig('fantastic_map')
# plt.close()