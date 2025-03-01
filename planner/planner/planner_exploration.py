
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

        self.grid = None
        self.give_goal = True
        self.give_path = False
    
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

            self.choose_next()

    def path_cb(self, msg:Path):

        if self.give_path == True:

            if not msg.poses:
                self.get_logger().warning(f'Couldnt find path')
                return 
    
            self.path_pub.publish(msg)
            self.give_path = False

    def drive_cb(self, msg:Bool):
        #Callback for feedback from drive control if any issue, when implementing obstacle avoidance

        if msg.data == True:
            self.get_logger().info(f'Getting next point')
            self.give_goal = True

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

        rob_x, rob_y = self.current_pos()

        if rob_x == None:
            return

        indices_unkown = np.argwhere(self.grid == -1)
       
        if len(indices_unkown) == 0:
            self.get_logger().info(f'exploration finished')
       
        x = indices_unkown[:, 1]
        y = indices_unkown[:, 0]

        next_x, next_y = self.grid_to_map(x, y)
        dists = np.sqrt(abs(next_x - rob_x)**2 + abs(next_y - rob_y)**2)
        dist_msk = dists > 10
        dists = dists[dist_msk]
        next_x = next_x[dist_msk]
        next_y = next_y[dist_msk]

        min_index = np.argmin(dists)

        msg_goal = PoseStamped()
        msg_goal.header.frame_id = 'map'
        msg_goal.pose.position.x = float(x[min_index])
        msg_goal.pose.position.y = float(y[min_index])
        msg_goal.pose.position.z = 0.0

        if self.give_goal == True:

            self.goal_pose_pub.publish(msg_goal)
            self.give_goal = False
            self.give_path = True

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
