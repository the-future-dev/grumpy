
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
from std_msgs.msg import Bool


class PlannerCollectionNode(Node):
    
    #Initialzie plsnner collection node
    def __init__(self):
        super().__init__('planner_collection_node') 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        
        self.pose_pub = self.create_publisher(PoseStamped, 'pose/goal_next', 1)#publish goal for astar
        self.path_pub = self.create_publisher(Path, 'path/next_goal', 1)#pblish path for drive_control

        #####HERE CONNECT WITH PICKUP#######

        B = 4
        self.positions = np.array([[1, 2, 3, B, 3],
                             [-199, 206, 187, -60, -206],
                             [-116, -100, 118, -50, 107]])
        
        self.object_poses, self.box_poses = self.filter_box_objects()
        self.object = True #Use to choose if going to object or box

        self.choose_next()

        self.path_sub = self.create_subscription(Path, 'path/Astar', self.path_cb, 1) #subscription of astar
        self.drive_sub = self.create_subscription(Bool, 'drive/feedback', self.drive_cb, 1 ) #subscription to drive control
    

    def filter_box_objects(self):
        #Filter teh boxes and object in spereat groups

        box_mask = self.positions[0, :] > 3
        return self.positions[1:, ~box_mask], self.positions[1:, box_mask]

    def choose_next(self):
        #Choose next point to go to

        #Get current pose
        tf_future = self.tf_buffer.wait_for_transform_async('map', 'base_link', self.get_clock().now())
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as ex:
             self.get_logger().info(f'Could not transform{ex}')
             return
        
        rob_x = tf.transform.translation.x
        rob_y = tf.transform.translation.y

        #Taking the object/box with smallest distance
        if self.object == True:

            dists = abs(self.object_poses[0,:] - rob_x)**2 + abs(self.object_poses[0,:] - rob_y)**2
            low_ind = np.argmin(dists)
            goal_pose = self.object_poses[:, low_ind]
            self.object_poses = np.delete(self.object_poses, low_ind)
        
        elif self.object == False:
            print('box')

            dists = abs(self.box_poses[0,:] - rob_x)**2 + abs(self.box_poses[0,:] - rob_y)**2
            low_ind = np.argmin(dists)
            goal_pose = self.box_poses[:, low_ind]

            print(goal_pose)

        #Publisg goal point to A star
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = Time() #Same issue, need to ask, but time not important here
        pose_msg.pose.position.x = float(goal_pose[0])
        pose_msg.pose.position.y = float(goal_pose[1])
        pose_msg.pose.position.z = 0.0

        self.pose_pub.publish(pose_msg)

    def path_cb(self, msg:Path):
        #Call backf for feedback from astar is the published message is None and publish path to drive control
        #Here poses should be minimized

        if not msg.poses:
            self.get_logger().info(f'already close, start pickup')
            self.object = False
            self.choose_next()
        else:
            self.path_pub.publish(msg)

    def drive_cb(self, msg:Bool):
        #Callback for feedback from drive control if any issue, when implementing obstacle avoidance

        if msg.data == True:
            self.get_logger().info(f'Time for Pickup')
            #Here when asking for Pickup and with success:
            self.object = False
            self.choose_next()

def main():
    rclpy.init()
    node = PlannerCollectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()