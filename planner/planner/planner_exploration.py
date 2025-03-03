
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

class PlannerExplorationNode(Node):
    
    #Initialzie oppucancy grid node
    def __init__(self):
        super().__init__('planner_exploration_node') 

        self.workspace = np.array([[-220, 220, 450, 700, 700, 546, 546, -220],
                                   [-130, -130, 66, 66, 284, 284, 130, 130]])
        
        self.n_corners = self.workspace.shape[1]
        self.counter = 0

        self.grid = None
        self.first = True
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.goal_pose_pub = self.create_publisher(PoseStamped, 'pose/goal_next', 1)
        self.path_pub = self.create_publisher(Path, 'path/next_goal', 1)

        self.grid_sub = self.create_subscription(Int16MultiArray, 'map/gridmap', self.grid_cb, 1)
        self.path_sub = self.create_subscription(Path, 'path/Astar', self.path_cb, 1)
        self.drive_feedback_sub = self.create_subscription(Bool, 'drive/feedback', self.drive_cb, 1)
    
    def grid_cb(self, msg:Int16MultiArray):

            #Return array from message of grid
            rows = msg.layout.dim[0].size
            columns = msg.layout.dim[1].size
            data = msg.data
            self.grid = np.array([data]).reshape(rows, columns)

            if self.first == True:
                self.first = False
                self.choose_next()

    def path_cb(self, msg:Path):
        print('path msg')

        if not msg.poses:
            self.get_logger().warning(f'No path, could alreday be close enough')
            self.choose_next()
            return 

        self.path_pub.publish(msg)
        

    def drive_cb(self, msg:Bool):
        #Callback for feedback from drive control if any issue, when implementing obstacle avoidance

        if msg.data == True:
            self.get_logger().info(f'Getting next point')
            self.choose_next()

        cmap = plt.cm.get_cmap('viridis', 5)

        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid, cmap=cmap, interpolation='nearest', origin='lower')
        cbar = plt.colorbar()
        cbar.set_ticks([0, 1, 2, 3, 4])
        plt.savefig('test')
        plt.close()

    def current_pos(self):

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

    def choose_next(self):

        if self.counter < self.n_corners:
            self.corner_goal()
        else:
            self.unknown_goal()

        return

    def corner_goal(self):

        msg_goal = PoseStamped()
        msg_goal.header.frame_id = 'map'

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

        print(msg_goal.pose.position.x)
        self.goal_pose_pub.publish(msg_goal)
        self.counter += 1
        
        return
    
    def unknown_goal(self):

        msg_goal = PoseStamped()
        msg_goal.header.frame_id = 'map'
            
        rob_x, rob_y = self.current_pos()
        indices_unkown = np.argwhere(self.grid == -1)
       
        if len(indices_unkown) == 0:
            self.get_logger().info(f'exploration finished')
       
        x = indices_unkown[:, 1]
        y = indices_unkown[:, 0]

        next_x, next_y = self.grid_to_map(x, y)
        dists = np.sqrt(abs(next_x - rob_x)**2 + abs(next_y - rob_y)**2)
        dist_msk = dists > 150
        dists = dists[dist_msk]
        next_x = next_x[dist_msk]
        next_y = next_y[dist_msk]

        min_index = np.argmin(dists)

        msg_goal.pose.position.x = float(next_x[min_index])
        msg_goal.pose.position.y = float(next_y[min_index])
        msg_goal.pose.position.z = 0.0
        self.goal_pose_pub.publish(msg_goal)

        return

    def grid_to_map(self, grid_x, grid_y):
        #Take grid indices and converts to some x,y in that grid
       
        x = (grid_x*3 - 1400/2) #Hard coded parameters right now 
        y = (grid_y*3 - 568/2)

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
