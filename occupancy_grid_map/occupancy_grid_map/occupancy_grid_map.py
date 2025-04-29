import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf_transformations
from shapely.geometry.polygon import Polygon
from shapely.geometry import Point
import matplotlib.pyplot as plt
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, String
from scipy.ndimage import binary_dilation
from grumpy_interfaces.msg import ObjectDetection1DArray, ObjectDetection1D
from occupancy_grid_map.workspace_utils import Workspace
from time import sleep
from matplotlib.colors import BoundaryNorm
from nav_msgs.msg import OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Pose


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
        self.phase = self.ws_utils.phase
        self.goal_point = None
        self.action = 'Other'

        #Transfrom between lidar link and map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        #Create publisher fo grid
        self.grid_pub = self.create_publisher(Int16MultiArray, 'map/gridmap', 10)
        self.visualize_pub = self.create_publisher(OccupancyGrid, 'map/visualized', 1)
        self.cells_pub = self.create_publisher(GridCells, 'map/grid_cells', 1)

        #Fill outside grid with 
        self.fill_outside_grid()

        if self.phase == 'collection':
            self.map = self.ws_utils.map
            self.map_to_grid(self.map, self.object_box)
            self.create_subscription(ObjectDetection1D, 'planner_collection/next_goal', self.goal_pos_cb, 2)
            self.create_subscription(String, 'brain/pick_drop_status', self.remove_cb, 1)
        elif self.phase == 'exploration':
            self.create_subscription(ObjectDetection1DArray, 'object_mapping/object_poses', self.object_cb, 1)

        #Subscribe to both lidar scan
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 1)


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

    def goal_pos_cb(self, msg:ObjectDetection1D):

        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y

        self.goal_point = np.array([[pos_x],
                                    [pos_y]])

        self.get_logger().info(f'Getting goal pose {self.goal_point}')

    def remove_cb(self, msg:String):

        self.action = msg.data
        if self.action == 'Pick_Success': # only Pick_Success before
            self.map_to_grid(self.goal_point, self.free)

        self.action = 'Other'
        self.get_logger().info(f'Getting remove cb {msg.data}')
    
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

        # if value == self.object_box:
        #     self.inflate_grid(x_grid_points, y_grid_points, value)

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

        cmap = plt.cm.get_cmap('Paired', 8)
        norm = BoundaryNorm([-2, -1, 0, 1, 2, 3, 4, 5, 6], cmap.N)
        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid, cmap=cmap, norm=norm, interpolation='nearest', origin='lower')
        cbar = plt.colorbar()
        cbar.set_ticks([-2, -1, 0, 1, 2, 3, 4, 5])
        plt.savefig('/home/group5/dd2419_ws/outputs/fantastic_map')
        plt.close()

        self.grid_pub.publish(msg_grid)

        self.visualize_grid_as_cells()
    
    def filter_points(self, x_grid_points, y_grid_points, value):

        #Mask to filter out of bounds points
        mask_in_bounds = (x_grid_points < self.grid.shape[1]) & (y_grid_points < self.grid.shape[0]) & (x_grid_points > 0) & (y_grid_points > 0) 
        x_grid_points = x_grid_points[mask_in_bounds]
        y_grid_points = y_grid_points[mask_in_bounds]

        if self.action != 'Pick_Success':

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

        mask_rgbd_scope = (x_free > 0.2) & (x_free < 1.4) & (x_free > abs(0.85*y_free))  #Cone shape
        #mask_rgbd_scope = (x_free > 0) & (x_free < 1.5) & (y_free > -0.4) & (y_free < 0.4)  #Box shape
        
        x_unknown = x_free[~mask_rgbd_scope]
        y_unknown = y_free[~mask_rgbd_scope]

        x_free = x_free[mask_rgbd_scope]
        y_free = y_free[mask_rgbd_scope]

        return x_free, y_free, x_unknown, y_unknown

    def visualize_grid_as_cells(self):
        # Create separate publishers for different cell types
        cells_msg = GridCells()
        cells_msg.header.frame_id = "map"
        cells_msg.header.stamp = self.get_clock().now().to_msg()
        cells_msg.cell_width = self.ws_utils.resolution / 100.0
        cells_msg.cell_height = self.ws_utils.resolution / 100.0

        # Get occupied cells
        occupied_y, occupied_x = np.where(self.grid == self.object_box)
        
        cells_msg.cells = []
        for x, y in zip(occupied_x, occupied_y):
            map_x, map_y = self.ws_utils.convert_grid_to_map(x, y)
            pose = Pose()
            pose.position.x = map_x / 100.0  # Convert to meters
            pose.position.y = map_y / 100.0
            pose.position.z = 0.0
            cells_msg.cells.append(pose.position)

        self.cells_pub.publish(cells_msg)


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
