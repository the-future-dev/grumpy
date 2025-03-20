
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

class OccupancyGridMapNode(Node):
    
    #Initialzie oppucancy grid node
    def __init__(self):
        super().__init__('occupancy_grid_map_node') 

        #Choose if Exploration or Collecttion, could be implemented with argument later
        self.workspace = np.array([[-220, 220, 450, 700, 700, 546, 546, -220],
                                   [-130, -130, 66, 66, 284, 284, 130, 130]])
        #self.workspace = np.array([[-220, 220, 220, -220],
                                  #[-130, -130, 130, 130]])

        self.polygon = Polygon(self.workspace.T)
        self.inflate_polygon = self.polygon.buffer(-25)

        #Initial data
        self.frame_id = 'map'
        self.map_xlength = np.max(self.workspace[0])*2
        self.map_ylength = np.max(self.workspace[1])*2
        self.counter = 0
        
        #Values for grid cells
        self.unknown = -1
        self.free = 0
        self.obstacle = 1
        self.outside = 2
        self.object_box = 3

        #Create grid
        self.resolution = 3 #Can be changed 
        self.grid_xlength = int(self.map_xlength/self.resolution)
        self.grid_ylength = int(self.map_ylength/self.resolution)
        self.grid = np.full((self.grid_ylength, self.grid_xlength), self.unknown, dtype=np.int16)

        #Transfrom between lidar link and map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        #Create publisher fo grid
        self.grid_pub = self.create_publisher(Int16MultiArray, 'map/gridmap', 10)

        #Fill outside grid with 
        self.fill_outside_grid()

        #Subscribe to both lidar scan
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 1)

    #Function which fills the space outside workspace as occupied, very slow now but have not succeded with other
    def fill_outside_grid(self):

        #Go through all points to see if inside or outside, making the initialization slow
        for i in range(self.grid_xlength):
            for j in range(self.grid_ylength):

                x_ind = (i * self.resolution) - self.map_xlength/2
                y_ind = (j * self.resolution) - self.map_ylength/2

                point = Point(x_ind, y_ind)
                if not self.inflate_polygon.contains(point):
                    self.grid[j, i] = self.outside
        return

    #Lidar callback calculates detected object from laser scan, would implement to only run every xth time
    def lidar_cb(self, msg:LaserScan):

        self.counter += 1
        if self.counter % 5 != 0: #Only use every Xth scan 
            return
        
        #Get data from message
        min_angle = msg.angle_min
        lower_bound = 0.2
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
        free_x, free_y = self.raytrace_float(lidar_x, lidar_y)
        self.point_to_map(msg, free_x, free_y, self.free)
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

        inflate = 0.3*100 #Scale to centimeters, introduce inflation to wokr in configuration space
        x_map = map_points[0, :]
        y_map = map_points[1, :]

        x_grid_points = np.floor((x_map + self.map_xlength/2)/self.resolution) #Convert to grid indices
        y_grid_points = np.floor((y_map + self.map_ylength/2)/self.resolution)
        inflate = np.floor(inflate/self.resolution)

        x_grid_points = x_grid_points.astype(int) #Make sure int index
        y_grid_points = y_grid_points.astype(int)
        inflate = inflate.astype(int)

        x_grid_points, y_grid_points = self.filter_points(x_grid_points, y_grid_points, value)

        self.grid[y_grid_points, x_grid_points] = value

        if value != self.free:
            self.inflate_grid(x_grid_points, y_grid_points, value)

    def inflate_grid(self, x_grid_points, y_grid_points, value):
        #Function which inflated the new points and a new grid and then merges it with old grid

        inflate_size = int(10/self.resolution)
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

        cmap = plt.cm.get_cmap('viridis', 5)
        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid, cmap=cmap, interpolation='nearest', origin='lower')
        cbar = plt.colorbar()
        cbar.set_ticks([0, 1, 2, 3, 4])
        plt.savefig('fantastic_map')
        plt.close()

        self.grid_pub.publish(msg_grid)
    
    def filter_points(self, x_grid_points, y_grid_points, value):

        #Mask to filter out of bounds points
        mask_in_bounds = (x_grid_points < self.grid.shape[1]) & (y_grid_points < self.grid.shape[0])
        x_grid_points = x_grid_points[mask_in_bounds]
        y_grid_points = y_grid_points[mask_in_bounds]

        #If placing an an object or box we do not want to care about where in workspace
        if value == self.object_box:
            return x_grid_points, y_grid_points
        
        #Mask to filter out grids outside workspace
        mask_workspace = self.grid[y_grid_points, x_grid_points] == self.outside 
        x_grid_points = x_grid_points[~mask_workspace]
        y_grid_points = y_grid_points[~mask_workspace]

        #filter out object values so they are not set as something else
        mask_not_object_box = self.grid[y_grid_points, x_grid_points] != self.object_box
        x_grid_points = x_grid_points[mask_not_object_box]
        y_grid_points = y_grid_points[mask_not_object_box]

        return x_grid_points, y_grid_points

    #Creating a linspace between 
    def raytrace_float(self, lidar_x, lidar_y):

        start = np.zeros_like(lidar_x)
        x_line = np.linspace(start, lidar_x, 10000)
        y_line = np.linspace(start, lidar_y, 10000)
    
        x_free = np.concatenate(x_line)
        y_free = np.concatenate(y_line)

        mask_rgbd_scope = (x_free > 0.2) & (x_free < 1.7) & (x_free > abs(y_free))  #Cone shape
        #mask_rgbd_scope = (x_free > 0) & (x_free < 1.5) & (y_free > -0.4) & (y_free < 0.4)  #Box shape
        
        x_free = x_free[mask_rgbd_scope]
        y_free = y_free[mask_rgbd_scope]

        return x_free, y_free

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