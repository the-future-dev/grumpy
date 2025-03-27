
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
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

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
        self.checked = 10   
        self.grid_xg = 0
        self.grid_yg = 0
        self.grid_recieved = False
        self.goal_pose_recieved = False
        self.map_xlength = 1400   #1400 exporation, 440 collection
        self.map_ylength = 568    #568 exploration, 260 collection
        self.resolution = 3

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Publisher to puclish path to follow
        self.path_pub = self.create_publisher(Path, 'Astar/path', 1)

        # Subscribe to grid and next goal topic
        self.create_subscription(PoseStamped, 'Astar/next_goal', self.next_goal_cb, 1)
        self.create_subscription(Int16MultiArray, 'map/gridmap', self.grid_cb, 1)

    def next_goal_cb(self, msg:PoseStamped):
        #Call back when pose revieved

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.grid_xg, self.grid_yg = self.map_to_grid(goal_x, goal_y)
        self.goal_pose_recieved = True

        self.publish_path()

    def grid_cb(self, msg:Int16MultiArray):
        #Return array from message of grid

        if self.grid_recieved == False:

            rows = msg.layout.dim[0].size
            columns = msg.layout.dim[1].size
            data = msg.data
            self.grid = np.array([data]).reshape(rows, columns)

            self.grid_recieved = True
            self.publish_path()
    
    def publish_path(self):
        #Function whihc finally publish un simplified path

        if self.goal_pose_recieved == False or self.grid_recieved == False:
           return
        self.get_logger().info(f'INTO A_STAR')
        result = self.solve_path_points()
        
        if not result:
            msg_empty = Path()
            self.path_pub.publish(msg_empty)
        else:
            list_poses, time = result
            if list_poses:
                msg_path = Path() #Create Pth message
                msg_path.header.frame_id = 'map'
                msg_path.header.stamp = time
                msg_path.poses = list_poses

                self.get_logger().info(f'path will be published from astar')
                self.path_pub.publish(msg_path)
            else:
                msg_empty = Path()
                self.path_pub.publish(msg_empty)

        self.grid_recieved = False
        self.goal_pose_recieved = False

        return
    
    def solve_path_points(self):
        #Function which takes the Q and decides whether to go to next item in Q or if at endpoint

        #Take grid current pose
        grid_x, grid_y = self.current_pos()
        print(self.grid[self.grid_yg, self.grid_xg], self.grid[grid_y, grid_x])

        G = abs(self.grid_xg - grid_x)**2 + abs(self.grid_yg - grid_y)**2 
        node_curr = ANode(grid_x, grid_y, None, 0, G) #Initial node, current pose
        self.Q = [node_curr] 
        heapq.heapify(self.Q) #heapify to simplify ordering

        while len(self.Q) != 0:
            node_curr = heapq.heappop(self.Q)
            grid_x = node_curr.grid_x
            grid_y = node_curr.grid_y

            if abs(self.grid_xg - grid_x) < 5 and abs(self.grid_yg - grid_y) < 5: #limits can be changed
                pose_list, time = self.end_point(node_curr)
                if not pose_list:
                    return None, time
                else:
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
        x_list = []
        y_list = []
        # time = Time() #Unsure why this tim object is the only working på not rclpy
        time = self.get_clock().now().to_msg()

        while node_curr.parent != None:

            x, y = self.grid_to_map(node_curr.grid_x, node_curr.grid_y)

            x_list.append(x)
            y_list.append(y)
            node_curr = node_curr.parent

        x_list = x_list[::-1]
        y_list = y_list[::-1]

        if not x_list:
            return None, time
        
        new_x, new_y = self.reduce_poses(x_list, y_list)

        for i in range(len(new_x)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = time
            pose.pose.position.x = new_x[i]
            pose.pose.position.y = new_y[i]
            pose.pose.position.z = 0.0
            pose_list.append(pose)

        return pose_list, time
    
    def reduce_poses(self, x_list, y_list):

        N = len(x_list) // 8

        cs = interp1d(x_list, y_list)

        x_new = np.linspace(x_list[0], x_list[-1], N)
        y_new = cs(x_new)
        
        return x_new, y_new
    
    def map_to_grid(self, x, y):
        #Take map coordinates and convert to grid
         
        grid_x = np.floor((x + self.map_xlength/2)/self.resolution)
        grid_y = np.floor((y + self.map_ylength/2)/self.resolution)

        grid_x = grid_x.astype(int)
        grid_y = grid_y.astype(int)

        return grid_x, grid_y

    def grid_to_map(self, grid_x, grid_y):
        #Take grid indices and converts to some x,y in that grid
       
        x = (grid_x*self.resolution - self.map_xlength/2)/100 #Hard coded parameters right now 
        y = (grid_y*self.resolution - self.map_ylength/2)/100

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

        grid_x, grid_y = self.map_to_grid(100*x, 100*y) #Convert to grid indices, scale to centimeters

        return grid_x, grid_y

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
