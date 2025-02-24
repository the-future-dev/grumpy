
import rclpy
import rclpy.clock
from rclpy.node import Node
import rclpy.time
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
from tf2_ros import TransformException
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import heapq

class ANode:
    def __init__(self, grid_x, grid_y, parent, C, G):
          self.grid_x = grid_x
          self.grid_y = grid_y
          self.parent = parent
          self.C = C
          self.G = G
          self.tot_cost = C + G

    def __lt__(self, other):

        if self.tot_cost != other.tot_cost:
            return self.tot_cost < other.cost
        if self.G != other.G:
            return self.G < other.G
        if self.grid_x != other.grid_x:
            return self.grid_x < other.grid_y
        return self.grid_y < other.grid_y

class AStarAlgorithmNode(Node):

    def __init__(self):
        super().__init__('a_star_algorithm_node')

        # Priotrity ques for A star algorithm
        self.Q = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Initialize parameters for conversion
        self.map_xlength = self.get_parameter('grid.map_xlength').value
        self.map_ylength = self.get_parameter('grid.map_ylength').value
        self.resolution = self.get_parameter('grid.resolution').value

        # Publisher to puclish path to follow
        self.path_pub = self.create_publisher(Path, 'path/Astar', 1)

        # Subscribe to grid topic 
        self.grid = self.create_subscription(Int16MultiArray, 'map/gridmap', self.grid_cb, 10)

        # Initalize goal points, in future subscribe to planning topic
        self.goal_x = 1
        self.goal_y = 1
        self.grid_xg, self.grid_yg = self.map_to_grid(self.goal_x, self.goal_y)
        self.checked = 10   

    def grid_cb(self, msg:Int16MultiArray):
        #Return array from message of grid
        return msg.data
    
    def map_to_grid(self, x, y):
        #Take map coordinates and convert to grid
         
        grid_x = np.floor((x*100 + self.map_xlength/2)/self.resolution)
        grid_y = np.floor((y*100 + self.map_xlength/2)/self.resolution)

        grid_x = grid_x.astype(int)
        grid_y = grid_y.astype(int)

        return grid_x, grid_y

    def grid_to_map(self, grid_x, grid_y):
        #Take grid indices and converts to some x,y in that grid
       
        x = (grid_x*self.resolution - self.map_xlength/2)/100
        y = (grid_y*self.resolution - self.map_ylength/2)/100

        return x, y
    
    def current_pos(self):
        #Uses transform to fins current pose of the robot in the map fram
         
        tf_future = self.tf_buffer.wait_for_transform_async('map', 'base_link', self.get_clock().now)
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            tf = self.tf_buffer.lookup_transform('map', 'baselink', rclpy.time.Time())
        except TransformException as ex:
             self.get_logger().info(f'Could not transform{ex}')
             return

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        grid_x, grid_y = self.map_to_grid(x, y) #Convert to grid indices

        return grid_x, grid_y
    
    def publish_path(self):

        list_poses = self.solve_path_points()

        if len(list_poses) != 0:
            msg_path = Path()
            msg_path.header.frame_id = 'map'
            msg_path.header.stamp = rclpy.time.Time()
            msg_path.poses = list_poses
            self.path_pub.publish(msg_path)
    
    def solve_path_points(self):

        #Take grid current pose
        grid_x, grid_y = self.current_pos()

        G = abs(self.grid_xg - grid_x)**2 + abs(self.grid_yg - grid_y)**2
        node_curr = ANode(grid_x, grid_y, None, 0, G)
        self.Q = [node_curr]
        self.Q = heapq.heapify(self.Q)

        while len(self.Q) != 0:
            node_curr = heapq.heappop(self.Q)
            grid_x = node_curr.grid_x
            grid_y = node_curr.grid_y

            if abs(self.grid_xg - grid_x) < 2 and abs(self.grid_yg - grid_y) < 2:
                pose_list = self.end_point(node_curr)
                return pose_list
            else:
                self.check_new_cells(node_curr)
    
    def check_new_cells(self, node_curr):
        
        neighbor_grid_x = np.array([-1, 0, 1, -1, 1, -1, 0, 1]) + node_curr.grid_x
        neighbor_grid_y = np.array([1, 1, 1, 0, 0, -1, -1 ,-1]) + node_curr.grid_y

        mask_free = self.grid[neighbor_grid_y, neighbor_grid_x] < 1

        next_grid_x = neighbor_grid_x[mask_free]
        next_grid_y = neighbor_grid_y[mask_free]
        
        self.grid[next_grid_y, next_grid_x] = self.checked

        next_nodes = self.create_node(node_curr, next_grid_x, next_grid_y)

        self.store_Q(next_nodes)

        return

    def create_node(self, node_curr, next_grid_x, next_grid_y):
        
        temp_C = abs(next_grid_x - node_curr.grid_x)**2 + abs(next_grid_y - node_curr.grid_y)**2

        if node_curr.parent != None:
            C = temp_C + node_curr.parent.C
        else:
            C = np.zeros_like(next_grid_x) + temp_C
        
        G = abs(next_grid_x - self.grid_xg)**2 + abs(next_grid_y - self.grid_yg)**2
        
        nodes_creator = np.vectorize(lambda d1, d2, d3, d4: ANode(d1, d2, node_curr, d3, d4))
        next_nodes = nodes_creator(next_grid_x, next_grid_y, C, G)

        return next_nodes

    def store_Q(self, next_nodes):

        for i in next_nodes:
            heapq.heappush(self.Q, i)
        
        return

    def end_point(self, node_curr):

        pose_list = []

        while node_curr.parent != None:

            x, y = self.grid_to_map(node_curr.grid_x, node_curr.grid_y)
            
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0

            pose_list.append(pose)
            node_curr = node_curr.parent
        
        pose_list = pose_list.reverse()

        return pose_list


def main():
    rclpy.init()
    node = AStarAlgorithmNode(Node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()