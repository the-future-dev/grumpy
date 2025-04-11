
import rclpy
import numpy as np
import rclpy.clock
from rclpy.node import Node
import rclpy.time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from std_msgs.msg import Bool
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from occupancy_grid_map.workspace_utils import Workspace

class LocalObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('local_obstacle_avoidance_node')

        #Use grid to do local obstacle avoidance
        self.grid = None
        self.current_pose_list = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Subscription to get path
        self.create_subscription(Path, '/brain/path_list', self.current_path_cb, 1)

        #Publisher to publish when not in free zone 
        self.free_zone_pub = self.create_publisher(Bool, 'avoidance/free_path', 1) #send true if free

        #Subscription of grid 
        self.create_subscription(Int16MultiArray, 'Astar/inflated_gridmap', self.grid_cb, 1)
        #self.create_subscription(Bool, 'avoidance/drive_to_free', self.adjust_cb, 1)

        self.ws_utils = Workspace()
        self.get_logger().info("Local Obstacle Avoidance Initialized")

    def current_path_cb(self, msg:Path):
        # set current path to check if it is free
        self.current_pose_list = msg.poses
        self.get_logger().info(f'Got pose list from brain with length: {len(self.current_pose_list)}')


    def grid_cb(self, msg:Int16MultiArray):
        #Return array from message of grid

        rows = msg.layout.dim[0].size
        columns = msg.layout.dim[1].size
        data = msg.data
        self.grid = np.array([data]).reshape(rows, columns)

        self.local_obstacle_avoidance()

    def local_obstacle_avoidance(self):
        #Function which sets self.stop as true if the grid the car is in is not set as free
        #Next step is to move to nearest free space

        msg_free = Bool()
        if len(self.current_pose_list) == 0:
            return

        free_path = True
        _x,_y = self.get_current_pos()

        x = np.array([pose.pose.position.x for pose in self.current_pose_list]) * 100
        y = np.array([pose.pose.position.y for pose in self.current_pose_list]) * 100
        grid_xi, grid_yi = self.ws_utils.convert_map_to_grid(x, y)
        
        if np.any(self.grid[grid_yi, grid_xi] > 0):
            free_path = False
            self.get_logger().info("Obstacle detected in path")
            
        if not free_path:
            msg_free.data = False
            self.free_zone_pub.publish(msg_free)
            self.get_logger().info("Publishing: Path is not free")
        else:
            msg_free.data = True
            self.free_zone_pub.publish(msg_free)
            # self.get_logger().info("Publishing: Path is free")

    def get_current_pos(self):
        #Getting current pose of robot. 

        # tf_future = self.tf_buffer.wait_for_transform_async('map', 'base_link', self.get_clock().now())
        # rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'could not transform{ex}')

        rob_x = tf.transform.translation.x
        rob_y = tf.transform.translation.y

        return rob_x, rob_y


def main():
    rclpy.init()
    node = LocalObstacleAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()