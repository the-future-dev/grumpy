
import rclpy
from rclpy.node import Node
import rclpy.time
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import heapq
from builtin_interfaces.msg import Time
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm
from scipy.ndimage import binary_dilation
from occupancy_grid_map.workspace_utils import Workspace
from time import sleep
from grumpy_interfaces.msg import ObjectDetection1D

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
        self.checked = 5   
        self.grid_xg = 0
        self.grid_yg = 0
        self.grid_rob_x = 0
        self.grid_rob_y = 0
        self.grid_recieved = False
        self.goal_pose_recieved = False
        self.ws_utils = Workspace()
        self.limit = 0
        self.phase = self.ws_utils.phase

        self.obstacle = 1
        self.outside = 3
        self.object_box = 4

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Publisher to puclish path to follow
        self.path_pub = self.create_publisher(Path, 'Astar/path', 1)
        self.inflate_grid_pub = self.create_publisher(Int16MultiArray, 'Astar/inflated_gridmap', 1)

        # Subscribe to grid and next goal topic
        self.create_subscription(ObjectDetection1D, 'Astar/next_goal', self.next_goal_cb, 1)
        self.create_subscription(Int16MultiArray, 'map/gridmap', self.grid_cb, 1)

    def next_goal_cb(self, msg:ObjectDetection1D):
        #Call back when pose revieved

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        action = msg.label.data

        self.grid_xg, self.grid_yg = self.ws_utils.convert_map_to_grid(goal_x, goal_y)

        if action == 'Pick':
            # self.limit = 4 # TODO: If A* is used to get all the way to the object
            self.limit = 9
        else:
            # self.limit = 6 # TODO: If A* is used to get all the way to the box
            self.limit = 12 

        self.goal_pose_recieved = True
        #self.get_logger().info(f'{self.grid_xg}hej')

        self.publish_path()

    def grid_cb(self, msg:Int16MultiArray):
        #Return array from message of grid

        rows = msg.layout.dim[0].size
        columns = msg.layout.dim[1].size
        data = msg.data
        self.grid = np.array([data]).reshape(rows, columns)

        self.inflate_grid(self.outside)
        self.inflate_grid(self.obstacle)
        self.inflate_grid(self.object_box)

        self.publish_inflated_grid()
        self.grid_recieved = True
        
        self.publish_path()

    def publish_inflated_grid(self):

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
        plt.savefig('/home/group5/dd2419_ws/outputs/local_obstacle_map')
        plt.close()

        self.inflate_grid_pub.publish(msg_grid)

    def inflate_grid(self, item):

        if item == self.obstacle:
            size = 35
        if item == self.outside:
            size = 30
        if item == self.object_box:
            size = 30

        inflate_size = int(size/self.ws_utils.resolution)
        inflate_matrix = np.ones((2 * inflate_size + 1, 2 * inflate_size + 1))
        
        new_grid = np.zeros_like(self.grid)
        arg_item = np.argwhere(self.grid == item)

        new_grid[arg_item[:, 0], arg_item[:, 1]] = 1

        if item == self.obstacle:
            arg_outside = np.argwhere(self.grid == self.outside)
            new_grid[arg_outside[:, 0], arg_outside[:, 1]] = 0

        new_grid = binary_dilation(new_grid, structure=inflate_matrix)*item

        mask_zeros = new_grid == 0
        new_grid[mask_zeros] = -1

        self.grid = np.maximum(self.grid, new_grid)

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
        self.grid_rob_x, self.grid_rob_y = self.current_pos()

        if self.grid[self.grid_rob_y, self.grid_rob_x] > 0:
            self.get_logger().info('Not in free space so takes closest free cell to start with')

            free_indices = np.argwhere(self.grid <= 0)

            grid_free_x = free_indices[:, 1]
            grid_free_y = free_indices[:, 0]

            dists = np.sqrt((grid_free_x - self.grid_rob_x)**2 + (grid_free_y - self.grid_rob_y)**2)
            
            min_dist_index = np.argmin(dists)

            self.grid_rob_x = grid_free_x[min_dist_index]
            self.grid_rob_y = grid_free_y[min_dist_index]

        G = abs(self.grid_xg - self.grid_rob_x)**2 + abs(self.grid_yg - self.grid_rob_y)**2 
        node_curr = ANode(self.grid_rob_x, self.grid_rob_y, None, 0, G) #Initial node, current pose
        self.Q = [node_curr] 
        heapq.heapify(self.Q) #heapify to simplify ordering

        while len(self.Q) != 0:
            node_curr = heapq.heappop(self.Q)
            grid_x = node_curr.grid_x
            grid_y = node_curr.grid_y

            if abs(self.grid_xg - grid_x) <= self.limit and abs(self.grid_yg - grid_y) <= self.limit: #limits can be changed, 10 worked good with objects, 13 seems okay with boxes
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

        mask_in_bounds = (neighbor_grid_x < self.grid.shape[1]) & (neighbor_grid_y < self.grid.shape[0]) & (neighbor_grid_x >= 0) & (neighbor_grid_y >= 0)
        neighbor_grid_x = neighbor_grid_x[mask_in_bounds]
        neighbor_grid_y = neighbor_grid_y[mask_in_bounds]

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
        time = self.get_clock().now().to_msg()

        while node_curr.parent != None:

            next_x, next_y = self.ws_utils.convert_grid_to_map(node_curr.grid_x, node_curr.grid_y)

            x_list.append(next_x/100)
            y_list.append(next_y/100)
            self.grid[node_curr.grid_y, node_curr.grid_x] = -2
            node_curr = node_curr.parent

        x_list = x_list[::-1]
        y_list = y_list[::-1]

        if len(x_list) == 0:
            return None, time
        
        if self.phase == 'collection':
            x_list, y_list = self.collection_reduce(x_list, y_list)
        elif self.phasse == 'exploration':
            x_list, y_list = self.exploration_reduce(x_list, y_list)

        # PLOTTING
        
        cmap = plt.cm.get_cmap('Paired', 8)
        norm = BoundaryNorm([-2, -1, 0, 1, 2, 3, 4, 5, 6], cmap.N)
        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid, cmap=cmap, norm=norm, interpolation='nearest', origin='lower')
        cbar = plt.colorbar()
        cbar.set_ticks([-2, -1, 0, 1, 2, 3, 4, 5])
        plt.savefig('/home/group5/dd2419_ws/outputs/astar_map')
        plt.close()

        #Creating message

        for i in range(len(x_list)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = time
            pose.pose.position.x = x_list[i]
            pose.pose.position.y = y_list[i]
            pose.pose.position.z = 0.0
            pose_list.append(pose)

        return pose_list, time

    def check_free_path(self, x_start, y_start, x_end, y_end):
        #Checkin linspace between poses is free or occupied

        x_line = np.linspace(x_start*100, x_end*100, 1000)
        y_line = np.linspace(y_start*100, y_end*100, 1000)

        grid_x_line, grid_y_line = self.ws_utils.convert_map_to_grid(x_line, y_line)

        if np.any((self.grid[grid_y_line, grid_x_line] > 0) & (self.grid[grid_y_line, grid_x_line] < 5)):
            return False
        else:
            return True
        
    def collection_reduce(self, x_list, y_list):
        #Reducing the poses in collection phase

        new_x = [x_list[0]]
        new_y = [y_list[0]]

        curr_x = x_list[0]
        curr_y = y_list[0]

        for i in range(len(x_list) - 1):
            
            free = self.check_free_path(curr_x, curr_y, x_list[i+1], y_list[i+1])
            
            if free == True:
                new_x.pop(-1)
                new_y.pop(-1)
            else:
                curr_x = new_x[-1]
                curr_y = new_y[-1]

            new_x.append(x_list[i+1])
            new_y.append(y_list[i+1])

        new_x.insert(0, x_list[0])
        new_y.insert(0, y_list[0])

        N = len(new_x) * 2

        cs = interp1d(new_x, new_y)
        x_list = np.linspace(new_x[0], x_list[-1], N)
        y_list = cs(x_list)

        self.get_logger().info(f'length of path {len(new_x)}')

        return x_list, y_list

    
    def exploration_reduce(self, x_list, y_list):
        #Reducing the poses in exploration

        if len(x_list) <= 3:
            return x_list, y_list

        N = len(x_list) // 2

        cs = interp1d(x_list, y_list)

        x_new = np.linspace(x_list[0], x_list[-1], N)
        y_new = cs(x_new)
        
        return x_new, y_new
    
    
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

        # grid_x, grid_y = self.map_to_grid(100*x, 100*y)
        grid_x, grid_y = self.ws_utils.convert_map_to_grid(100*x, 100*y)

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
