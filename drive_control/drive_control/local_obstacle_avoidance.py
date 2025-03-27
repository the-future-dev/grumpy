
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
from geometry_msgs.msg import PoseStamped

class LocalObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('local_obstacle_avoidance_node')

        #Use grid to do local obstacle avoidance
        self.grid = None
        self.map_xlength = 1900 
        self.map_ylength = 752  
        self.resolution = 3
        self.adjust = False 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        #Publisher to publish when not in free zone 
        self.free_zone_pub = self.create_publisher(Bool, 'avoidance/free_path', 1) #send true if free
        self.drive_to_free_pub = self.create_publisher(Path, 'avoidance/drive_free', 1)

        #Subscription of grid 
        self.create_subscription(Int16MultiArray, 'map/gridmap', self.grid_cb, 1)
        self.create_subscription(Bool, 'avoidance/drive_to_free', self.adjust_cb, 1)

    def grid_cb(self, msg:Int16MultiArray):
        #Return array from message of grid

        rows = msg.layout.dim[0].size
        columns = msg.layout.dim[1].size
        data = msg.data
        self.grid = np.array([data]).reshape(rows, columns)

        self.local_obstacle_avoidance()
    
    def adjust_cb(self,msg:Bool):
        #Function which is called when in occipied positiona and find nearest free point to drive to 

        free_indices = np.argwhere(self.grid == 0)

        grid_x = free_indices[:, 1]
        grid_y = free_indices[:, 0]

        free_x, free_y = self.grid_to_map(grid_x, grid_y)
        dists = np.sqrt((free_x - self.rob_x)**2 + (free_y - self.rob_y)**2)
        dist_mask = dists > 5
        dists = dists[dist_mask]
        free_x = free_x[dist_mask]
        free_y = free_y[dist_mask]
        
        min_dist_index = np.argmin(dists)
        
        pose = PoseStamped()
        pose.pose.position.x = free_x[min_dist_index]/100
        pose.pose.position.y = free_y[min_dist_index]/100
        pose_list = [pose]

        msg_drive = Path()
        msg_drive.header.frame_id = 'map'
        msg_drive.header.stamp = self.get_clock().now().to_msg()
        msg_drive.poses = pose_list

        self.drive_to_free_pub.publish(msg_drive)

    def local_obstacle_avoidance(self):
        #Function which sets self.stop as true if the grid the car is in is not set as free
        #Next step is to move to nearest free space

        msg_free = Bool()

        self.rob_x, self.rob_y = self.get_current_pos()
        grid_x, grid_y = self.map_to_grid(100*self.rob_x, 100*self.rob_y)

        free_zone = self.grid[grid_y, grid_x] <= 0

        if free_zone == False:
            msg_free.data = False
            self.free_zone_pub.publish(msg_free)
        else:
            msg_free.data = True
            self.free_zone_pub.publish(msg_free) 

    def get_current_pos(self):
        #Getting current pose of robot. 

        tf_future = self.tf_buffer.wait_for_transform_async('map', 'base_link', self.get_clock().now())
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'could not transform{ex}')

        rob_x = tf.transform.translation.x
        rob_y = tf.transform.translation.y

        return rob_x, rob_y

    def map_to_grid(self, x, y):
        #Take map coordinates and convert to grid
         
        grid_x = np.floor((x + self.map_xlength/2)/self.resolution)
        grid_y = np.floor((y + self.map_ylength/2)/self.resolution)

        grid_x = grid_x.astype(int)
        grid_y = grid_y.astype(int)

        return grid_x, grid_y
    
    def grid_to_map(self, grid_x, grid_y):
        #Take grid indices and converts to some x,y in that grid
       
        x = (grid_x*self.resolution - self.map_xlength/2) 
        y = (grid_y*self.resolution - self.map_ylength/2)

        return x, y


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