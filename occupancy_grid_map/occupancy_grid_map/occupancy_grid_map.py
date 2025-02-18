
import rclpy
from rclpy.node import Node
import rclpy.time
from geometry_msgs.msg import PointStamped
import numpy as np
from sensor_msgs.msg import LaserScan
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf_transformations
from shapely.geometry.polygon import Polygon
from shapely.geometry import Point
import matplotlib.pyplot as plt

class OccupancyGridMapNode(Node):
    
    #Initialzie oppucancy grid node
    def __init__(self):
        super().__init__('occupancy_grid_map_node') 

        self.workspace = np.array([[-220, 220, 450, 700, 700, 546, 546, -200],
                                   [-130, -130, 66, 66, 284, 284, 130, 130]])
        self.polygon = Polygon(self.workspace.T)

        #Initial data
        self.frame_id = 'map'
        self.map_xlength = np.max(self.workspace[0])*2
        self.map_ylength = np.max(self.workspace[1])*2
        
        #Values for grid cells
        self.unknown = -1
        self.occupied = 0
        self.free = 1

        #Create grid
        self.resolution = 5 # Can be changed
        self.grid_xlength = int(self.map_xlength/self.resolution)
        self.grid_ylength = int(self.map_ylength/self.resolution)
        self.grid = np.full((self.grid_xlength, self.grid_ylength), self.unknown, dtype=np.int32)

        #Fill outside grid with 
        self.fill_outside_grid()

        fig, ax = plt.subplots()
        cbar = ax.imshow(self.grid, cmap='viridis', origin='lower')

        # Adjust the colorbar ticks
        cbar = plt.colorbar(cbar)
        cbar.set_ticks([self.unknown, self.occupied, self.free])

        plt.savefig('/home/robot/occupancy_grid.png')


        #Transfrom between lidar link and map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Subscribe to both lidar scan
        #self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
    

    #Function which fills the space outside workspace as occupied, very slow now but have not succeded with other
    def fill_outside_grid(self):
        
        #Creat polygon from workspace
        polygon = Polygon(self.workspace.T)

        #Go thrpugh all points to see if inside or outside, why so slow
        for i in range(self.grid_xlength):
            for j in range(self.grid_ylength):

                x_ind = (i * self.resolution) - self.map_xlength/2
                y_ind = -(j * self.resolution) + self.map_ylength/2

                point = Point(x_ind, y_ind)
                if not polygon.contains(point):
                    self.grid[i, j] = self.occupied
        return

    #Lidar callback calculates detected object from laser scan, would implement to only run every xth time
    def lidar_cb(self, msg:LaserScan):
        
        #Get data from message
        min_angle = msg.angle_min
        lower_bound = msg.range_min
        upper_bound = msg.range_max
        inc = msg.angle_increment
        ranges = np.array(msg.ranges)
 
        #Create mask to filter out ranges to far away or to close
        indices = np.arange(len(ranges))
        boundary_mask = (ranges >= lower_bound) & (ranges <= upper_bound)

        indices = indices[boundary_mask]
        ranges = ranges[boundary_mask]
        
        #Calculat x and y according ot lidar link frame
        x_angles = np.cos(min_angle + indices * inc)
        y_angles = np.sin(min_angle + indices * inc)

        lidar_x = ranges*x_angles
        lidar_y = ranges*y_angles

        self.point_to_map(msg, lidar_x, lidar_y, True)
        free_x, free_y = self.bresenham_line(lidar_x, lidar_y)
        self.point_to_map(msg, free_x, free_y, False)

    #Transform point from one message to map
    def point_to_map(self, msg, x_points, y_points, bool):

        #Looking for transform from msg-frame to map
        tf_future = self.tf_buffer.wait_for_transform_async('map', msg.header.frame_id, msg.header.stamp)
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform{ex}')

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

        measure_points = np.vstack([x_points, y_points, np.zeros_like(x_points), np.ones_like(y_points)])
        transformed_points = tf_matrix @ measure_points

        self.map_to_grid(transformed_points, bool)

    #Convert map coordinate to grid indices and set as occupied or free
    def map_to_grid(self, map_points, bool):
        
        grid_points = np.zeros_like(map_points[:2, :])
        
        grid_points[0, :] = np.floor(map_points[0, :]/self.resolution - self.grid_xlength/2)
        grid_points[1, :] = np.floor(-map_points[1, :]/self.resolution + self.grid_ylength/2)

        if bool == True:
            self.grid[grid_points] = self.occupied
        elif bool == False:
            self.grid[grid_points] = self.free

    #Bresenham algorithm that i am not to familiar with, so i have taken it from elsewhere
    def bresenham_line(self, lidar_x, lidar_y):

        free_x = []
        free_y = []
        threshold = 3

        x_0 = np.zeros_like(lidar_x)
        y_0 = np.zeros_like(lidar_y)
        dx = np.abs(lidar_x)
        dy = np.abs(lidar_y)
        sx = np.sign(lidar_x)
        sy = np.sign(lidar_y)

        err = dx - dy
        max_len = np.maximum(dx, dy) + 1

        for i in range(np.max(max_len)):

            err2 = 2*err
            mask_x = err2 > -dy
            mask_y = err2 < dx
            err[mask_x] -= dy[mask_x]
            x_0 += sx[mask_x]
            err[mask_y] += dx[mask_y]
            y_0[mask_y] += sy[mask_y]

            free_x.append(x_0)
            free_y.append(y_0)
            
            if np.all(np.abs(x_0 - lidar_x) < threshold) and np.all((np.abs(y_0 - lidar_y) < threshold)):
                break

        return free_x, free_y

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