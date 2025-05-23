
import rclpy
import rclpy.clock
from rclpy.node import Node
import rclpy.time
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
from tf2_ros import TransformException
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray
from builtin_interfaces.msg import Time
from std_msgs.msg import Int16MultiArray
from robp_interfaces.msg import DutyCycles
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from robp_interfaces.msg import DutyCycles
from occupancy_grid_map.workspace_utils import Workspace
import random


class PlannerExplorationNode(Node):
    
    #Initialzie oppucancy grid node
    def __init__(self):
        super().__init__('planner_exploration_node') 

        self.workspace = np.array([[-50, 470, 750, 950, 950, 810, 810, -50],
                                   [-50, -50, 154, 154, 376, 376, 220, 220]])
        
        self.n_corners = self.workspace.shape[1]
        self.counter = 8
        self.grid = None
        self.ws_utils = Workspace()
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.goal_pose_pub = self.create_publisher(PoseStamped, 'planner_ex/next_goal', 1)
        self.corner_pub = self.create_publisher(Bool, 'planner_ex/corner_ex_done', 1)
        self.no_unknown_pub = self.create_publisher(Bool, 'planner_ex/no_unknown_left', 1)

        self.create_subscription(Int16MultiArray, 'map/gridmap', self.grid_cb, 1)
        self.create_subscription(Bool, 'planner_ex/find_goal', self.find_goal_cb, 1)
    
    def grid_cb(self, msg:Int16MultiArray):
        #Return array from message of grid
        rows = msg.layout.dim[0].size
        columns = msg.layout.dim[1].size
        data = msg.data
        self.grid = np.array([data]).reshape(rows, columns)
    
    def find_goal_cb(self, msg:Bool):

        if msg.data == True:
            self.choose_next()

    def choose_next(self):
        #First part of exploration is the go to the corners of the wokrspace
        #Thereafter the robot goes to unkown grid cells in map

        if self.counter < self.n_corners:
            self.corner_goal()
        else:
            self.unknown_goal()
        return

    def corner_goal(self):
        #Incrementing through all corners of workspace and publishing it to a-star

        self.get_logger().info(f'Corner nr {self.counter}')

        msg_goal = PoseStamped()
        msg_goal.header.frame_id = 'map'

        msg_corner_done = Bool()
        msg_corner_done.data = False

        x_corner = self.workspace[0, self.counter]
        y_corner = self.workspace[1, self.counter]

        if x_corner < 0:
            next_x = float(x_corner + 40)
        else:
            next_x = float(x_corner - 40)
        if y_corner < 0:
            next_y = float(y_corner + 40)
        else:
            next_y = float(y_corner - 40)
    
        msg_goal.pose.position.x = next_x
        msg_goal.pose.position.y = next_y
        msg_goal.pose.position.z = 0.0
        self.get_logger().info(f'the goal position is: x: {next_x}, y: {next_y}')


        self.goal_pose_pub.publish(msg_goal)
        self.corner_pub.publish(msg_corner_done)
        self.get_logger().info('Publihsing from planner to brain')

        self.counter += 1
        
        return
    
    def unknown_goal(self):
        #Taking the closets grid cell with a thershold to go explore to next

        msg_goal = PoseStamped()
        msg_goal.header.frame_id = 'map'

        msg_corner_done = Bool()
        msg_corner_done.data = True
            
        rob_x, rob_y = self.current_pos()
        indices_unkown = np.argwhere(self.grid == -1)
       
        if len(indices_unkown) == 0:
            self.get_logger().info(f'exploration finished')
            msg_done = Bool()
            msg_done.data = True 
            self.no_unknown_pub.publish(msg_done)
            return

        grid_x_unknown = indices_unkown[:, 1]
        grid_y_unknown = indices_unkown[:, 0]

        # next_x, next_y = self.grid_to_map(x, y)
        next_x, next_y = self.ws_utils.convert_grid_to_map(grid_x_unknown, grid_y_unknown)

        dists = np.sqrt(abs(next_x - rob_x)**2 + abs(next_y - rob_y)**2)
        dist_msk = dists > 200
        dists = dists[dist_msk]
        next_x = next_x[dist_msk]
        next_y = next_y[dist_msk]

        arg = random.randint(0, len(dists))
    
        msg_goal.pose.position.x = float(next_x[arg])
        msg_goal.pose.position.y = float(next_y[arg])
        msg_goal.pose.position.z = 0.0
        
        self.goal_pose_pub.publish(msg_goal)
        self.corner_pub.publish(msg_corner_done)

        self.get_logger().info(f'Publishing from planner to brain')

        return
    
    def current_pos(self):
        #Giving current position of the robot

        tf_future = self.tf_buffer.wait_for_transform_async('map', 'base_link', self.get_clock().now())
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as ex:
             self.get_logger().info(f'Could not transform{ex}')
             return None, None
        
        rob_x = tf.transform.translation.x
        rob_y = tf.transform.translation.y

        return rob_x, rob_y

def main():
    rclpy.init()
    node = PlannerExplorationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
