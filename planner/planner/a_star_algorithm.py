
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
from builtin_interfaces.msg import Time

class ANode:
    #Create a A-star Node for queueing
    def __init__(self, grid_x, grid_y, parent, C, G):
          self.grid_x = grid_x
          self.grid_y = grid_y
          self.parent = parent
          self.C = C
          self.G = G
          self.tot_cost = C + G

    def __lt__(self, other):
        #Used for the heapq to order it

        if self.tot_cost != other.tot_cost:
            return self.tot_cost < other.tot_cost
        if self.G != other.G:
            return self.G < other.G
        if self.grid_x != other.grid_x:
            return self.grid_x < other.grid_y
        return self.grid_y < other.grid_y

class AStarAlgorithmNode(Node):

    def __init__(self):
        super().__init__('a_star_algorithm_node')

        # Priotrity ques for A star algorithm
        self.Q = None
        self.grid = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Publisher to puclish path to follow
        self.path_pub = self.create_publisher(Path, 'path/Astar', 10)

        # Subscribe to grid topic 
        self.grid_sub = self.create_subscription(Int16MultiArray, 'map/gridmap', self.grid_cb, 10)

        # Initalize goal points, in future subscribe to planning topic
        self.goal_x = 199
        self.goal_y = 116
        self.grid_xg, self.grid_yg = self.map_to_grid(self.goal_x, self.goal_y)
        self.checked = 10   

        self.publish_path()

    def grid_cb(self, msg:Int16MultiArray):
        #Return array from message of grid
        rows = msg.layout.dim[0].size
        columns = msg.layout.dim[1].size
        data = msg.data
        self.grid = np.array([data]).reshape(rows, columns)
        
    def map_to_grid(self, x, y):
        #Take map coordinates and convert to grid
         
        grid_x = np.floor((x + 440/2)/3)
        grid_y = np.floor((y + 260/2)/3)

        grid_x = grid_x.astype(int)
        grid_y = grid_y.astype(int)

        return grid_x, grid_y

    def grid_to_map(self, grid_x, grid_y):
        #Take grid indices and converts to some x,y in that grid
       
        x = (grid_x*3 - 440/2)/100 #Hard coded parameters right now 
        y = (grid_y*3 - 260/2)/100

        return x, y
    
    def current_pos(self):
        #Uses transform to fins current pose of the robot in the map fram
        tf_future = self.tf_buffer.wait_for_transform_async('map', 'base_link', self.get_clock().now())
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=2)

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as ex:
             self.get_logger().info(f'Could not transform{ex}')
             return

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        grid_x, grid_y = self.map_to_grid(x, y) #Convert to grid indices

        return grid_x, grid_y
    
    def publish_path(self):
        #Function whihc finally publish un simplified path

        list_poses, time = self.solve_path_points()

        if len(list_poses) != 0:
            msg_path = Path() #Create Pth message
            msg_path.header.frame_id = 'map'
            msg_path.header.stamp = time
            msg_path.poses = list_poses

            self.path_pub.publish(msg_path)

        return
    
    def solve_path_points(self):
        #Function which takes the Q and decides whether to go to next item in Q or if at endpoint

        #Take grid current pose
        grid_x, grid_y = self.current_pos()

        G = abs(self.grid_xg - grid_x)**2 + abs(self.grid_yg - grid_y)**2 
        node_curr = ANode(grid_x, grid_y, None, 0, G) #Initial node, current pose
        self.Q = [node_curr] 
        heapq.heapify(self.Q) #heapify to simplify ordering

        while len(self.Q) != 0:
            node_curr = heapq.heappop(self.Q)
            grid_x = node_curr.grid_x
            grid_y = node_curr.grid_y

            if abs(self.grid_xg - grid_x) < 10 and abs(self.grid_yg - grid_y) < 10: #limits can be changed
                pose_list, time = self.end_point(node_curr)
                return pose_list, time
            else:
                self.check_new_cells(node_curr)
    
    def check_new_cells(self, node_curr):
        #Function which sheck neighboring cells around, a bug when node_curr is on edge of grid, need to fix
        
        neighbor_grid_x = np.array([-1, 0, 1, -1, 1, -1, 0, 1]) + node_curr.grid_x
        neighbor_grid_y = np.array([1, 1, 1, 0, 0, -1, -1 ,-1]) + node_curr.grid_y
        

        mask_free = self.grid[neighbor_grid_y, neighbor_grid_x] < 1 #Filter if cells are occupied

        next_grid_x = neighbor_grid_x[mask_free]
        next_grid_y = neighbor_grid_y[mask_free]
        
        self.grid[next_grid_y, next_grid_x] = self.checked #Set as checked to not check again

        next_nodes = self.create_node(node_curr, next_grid_x, next_grid_y) 

        self.store_Q(next_nodes)

        return

    def create_node(self, node_curr, next_grid_x, next_grid_y):
        #Function whihc creates a new node 
        
        temp_C = abs(next_grid_x - node_curr.grid_x)**2 + abs(next_grid_y - node_curr.grid_y)**2

        #Calculate C cost total dpeendnent on parent
        if node_curr.parent != None:
            C = temp_C + node_curr.parent.C
        else:
            C = np.zeros_like(next_grid_x) + temp_C
        
        #G cost from distance
        G = abs(next_grid_x - self.grid_xg)**2 + abs(next_grid_y - self.grid_yg)**2

        #Check if None vector
        if next_grid_x.size == 0:
            return [] 

        #Create nodes from all new points
        nodes_creator = np.vectorize(lambda d1, d2, d3, d4: ANode(d1, d2, node_curr, d3, d4))
        next_nodes = nodes_creator(next_grid_x, next_grid_y, C, G)

        return next_nodes

    def store_Q(self, next_nodes):
        #Function which stores Q in list thorugh heapq

        for i in next_nodes:
            heapq.heappush(self.Q, i)
        
        return

    def end_point(self, node_curr):
        #Function when reaching endpoint criteria to trace back from node to parent

        pose_list = []
        time = Time() #Unsure why this tim object is the only working på not rclpy

        while node_curr.parent != None:

            x, y = self.grid_to_map(node_curr.grid_x, node_curr.grid_y)
            
            pose = PoseStamped() #Create a list of PoseStamped
            pose.header.frame_id = 'map'
            pose.header.stamp = time
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose_list.append(pose)
            node_curr = node_curr.parent
        
        pose_list = pose_list[::-1] #reverse lisst so that first pose is first in list

        return pose_list, time


def main():
    rclpy.init()
    node = AStarAlgorithmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()