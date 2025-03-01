
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

class OccupancyGridMapNode(Node):
    
    #Initialzie oppucancy grid node
    def __init__(self):
        super().__init__('occupancy_grid_map_node') 

        #Choose if Exploration or Collecttion, could be implemented with argument later
        #self.workspace = np.array([[-220, 220, 450, 700, 700, 546, 546, -200],
                                   #[-130, -130, 66, 66, 284, 284, 130, 130]])
        self.workspace = np.array([[-220, 220, 220, -220],
                                   [-130, -130, 130, 130]])

        self.polygon = Polygon(self.workspace.T)
        self.inflate_polygon = self.polygon.buffer(-30)

        #Initial data
        self.frame_id = 'map'
        self.map_xlength = np.max(self.workspace[0])*2
        self.map_ylength = np.max(self.workspace[1])*2
        self.counter = 0
        
        #Values for grid cells
        self.unknown = -1
        self.free = 0
        self.obstacle = 1
        self.object = 2
        self.box = 3
        self.outside = 4

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

        #If collection, call function which maps a given file
        self.given_objects_boxes()

        self.seen_x, self.seen_y = self.rgbd_scope()

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
                if not self.polygon.contains(point):
                    self.grid[j, i] = self.outside
                if (not self.inflate_polygon.contains(point)) and (self.polygon.contains(point)):
                    self.grid[j, i] = self.obstacle
        return
    
    def given_objects_boxes(self):
        #Function to set given objects from a map into grid map in collection
        
        B = 4
        object_box_map = np.array([[1, 2, 3, B, 3],
                               [-199, 206, 187, 16, -206],
                               [-116, -100, 118, 122, 107]])
        
        mask_objects = object_box_map[0, :] < 4

        object_map = object_box_map[:, mask_objects]
        box_map = object_box_map[:, ~mask_objects]

        self.map_to_grid(object_map[1:, :], self.object)
        self.map_to_grid(box_map[1:, :], self.box)

    def rgbd_scope(self):
        #Function to set scope which will be determined as seen for exploration. 

        N1 = 20
        N2 = 20
        start = np.zeros(N1)
        x_edge = np.ones(N1)*1.5 #Change 1.5 to scope where rgbd can se
        y_edge = np.linspace(-0.5, 0.5, N1)
        
        x_lines = np.linspace(start, x_edge, N2)
        y_lines = np.linspace(start, y_edge, N2)

        x_free = np.concatenate(x_lines)
        y_free = np.concatenate(y_lines)

        return x_free, y_free
        
    #Lidar callback calculates detected object from laser scan, would implement to only run every xth time
    def lidar_cb(self, msg:LaserScan):

        self.counter += 1
        if self.counter % 20 != 0: #Only use every Xth scan 
            return
        
        #Get data from message
        min_angle = msg.angle_min
        lower_bound = msg.range_min
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
        self.point_to_map(msg, self.seen_x, self.seen_y, self.free)

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

        #Set indice to either occupied or free depending on bool
    
        self.grid[y_grid_points, x_grid_points] = value

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

        self.grid_pub.publish(msg_grid)
    
    def filter_points(self, x_grid_points, y_grid_points, value):

        #Mask to filter out of bounds points
        mask_in_bounds = (x_grid_points < self.grid.shape[1]) & (y_grid_points < self.grid.shape[0])

        x_grid_points = x_grid_points[mask_in_bounds]
        y_grid_points = y_grid_points[mask_in_bounds]
        
        #Mask to filter out grids outside workspace
        mask_workspace = self.grid[y_grid_points, x_grid_points] == self.outside 

        x_grid_points = x_grid_points[~mask_workspace]
        y_grid_points = y_grid_points[~mask_workspace]

        #filter out object values so they are not set as something else

        if (value != self.object) and (value != self.box):
            mask_not_object = self.grid[y_grid_points, x_grid_points] != self.object
            x_grid_points = x_grid_points[mask_not_object]
            y_grid_points = y_grid_points[mask_not_object]
            
        if value == self.free:
            mask_unknown = self.grid[y_grid_points, x_grid_points] == self.unknown
            x_grid_points = x_grid_points[mask_unknown]
            y_grid_points = y_grid_points[mask_unknown]

        return x_grid_points, y_grid_points

    #Creating a linspace between 
    # def raytrace_float(self, lidar_x, lidar_y):

    #     start = np.zeros_like(lidar_x)
    #     x_line = np.linspace(start, lidar_x, 50)
    #     y_line = np.linspace(start, lidar_y, 50)
    #     x_line = x_line[:-10]
    #     y_line = y_line[:-10]
        
    #     x_free = np.concatenate(x_line)
    #     y_free = np.concatenate(y_line)

    #     return x_free, y_free

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