
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


class PlannerExplorationNode(Node):
    
    #Initialzie oppucancy grid node
    def __init__(self):
        super().__init__('planner_exploration_node') 

        self.workspace = np.array([[-220, 220, 450, 700, 700, 546, 546, -220],
                                   [-130, -130, 66, 66, 284, 284, 130, 130]])
        
        self.n_corners = self.workspace.shape[1]
        self.counter = 0
        self.grid = None
        self.map_ylength = 568
        self.map_xlength = 1400
        self.resolution = 3
    
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

        msg_goal = PoseStamped()
        msg_goal.header.frame_id = 'map'

        msg_corner_done = Bool()
        msg_corner_done.data = False

        x_corner = self.workspace[0, self.counter]
        y_corner = self.workspace[1, self.counter]

        if x_corner < 0:
            next_x = float(x_corner + 30)
        else:
            next_x = float(x_corner - 30)
        if y_corner < 0:
            next_y = float(y_corner + 30)
        else:
            next_y = float(y_corner - 30)
    
        msg_goal.pose.position.x = next_x
        msg_goal.pose.position.y = next_y
        msg_goal.pose.position.z = 0.0

        self.goal_pose_pub.publish(msg_goal)
        self.corner_pub.publish(msg_corner_done)
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

        x = indices_unkown[:, 1]
        y = indices_unkown[:, 0]

        next_x, next_y = self.grid_to_map(x, y)

        dists = np.sqrt(abs(next_x - rob_x)**2 + abs(next_y - rob_y)**2)
        dist_msk = dists > 200
        dists = dists[dist_msk]
        next_x = next_x[dist_msk]
        next_y = next_y[dist_msk]

        min_index = np.argmin(dists)

        msg_goal.pose.position.x = float(next_x[min_index])
        msg_goal.pose.position.y = float(next_y[min_index])
        msg_goal.pose.position.z = 0.0
        
        self.goal_pose_pub.publish(msg_goal)
        self.corner_pub.publish(msg_corner_done)

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

    def grid_to_map(self, grid_x, grid_y):
        #Take grid indices and converts to some x,y in that grid
       
        x = (grid_x*self.resolution - self.map_xlength/2) #Hard coded parameters right now 
        y = (grid_y*self.resolution - self.map_ylength/2)

        return x, y

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
