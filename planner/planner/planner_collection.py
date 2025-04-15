
import rclpy
import rclpy.clock
from rclpy.node import Node
import rclpy.time
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
from tf2_ros import TransformException
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool
from std_msgs.msg import String
from grumpy_interfaces.msg import ObjectDetection1D


class PlannerCollectionNode(Node):
    
    #Initialzie plsnner collection node
    def __init__(self):
        super().__init__('planner_collection_node') 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.collection_done = self.create_publisher(Bool, 'planner_collection/no_objects_left', 1)
        self.goal_pose_pub = self.create_publisher(ObjectDetection1D, 'planner_collection/next_goal', 1)

        B = 4
        self.positions = np.array([[1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, B, B, B, B, B, B],
                             [456, 87, 914, 884, 439, 83, 269, 580, 599, 937, 292, 286, 360, 860, 397, 522, 929],
                             [-30, -34, 346, 346, 281, 196, 172, 95, 37, 52, 301, 74, 193, 14, 367, -35, 206, 226]])
        
        self.object_poses, self.box_poses = self.filter_box_objects()
        self.object = True #Use to choose if going to object or box

        self.create_subscription(String, 'planner_collection/find_goal', self.find_goal_cb, 1)
    
    def find_goal_cb(self, msg:String):

        if msg.data == 'Pick':
            self.object = True
        else:
            self.object = False

        self.choose_next()

    def filter_box_objects(self):
        #Filter teh boxes and object in spereat groups

        box_mask = self.positions[0, :] > 3
        return self.positions[1:, ~box_mask], self.positions[1:, box_mask]
    
    def get_curr_pos(self):

        tf_future = self.tf_buffer.wait_for_transform_async('map', 'base_link', self.get_clock().now())
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as ex:
             self.get_logger().info(f'Could not transform{ex}')
             return
        
        rob_x = tf.transform.translation.x
        rob_y = tf.transform.translation.y

        return rob_x, rob_y

    def choose_next(self):
        #Choose next point to go to

        if self.object_poses.shape[1] == 0:
            msg_finish = Bool()
            msg_finish.data = True
            self.collection_done.publish(msg_finish)

        #Get current pose
        rob_x, rob_y = self.get_curr_pos()

        #Taking the object/box with smallest distance
        if self.object == True:

            dists = np.sqrt(abs(self.object_poses[0,:] - rob_x)**2 + abs(self.object_poses[0,:] - rob_y)**2)
            low_ind = np.argmin(dists)
            goal_pose = self.object_poses[:, low_ind]
            self.object_poses = np.delete(self.object_poses, low_ind)
        
        elif self.object == False:

            dists = np.sqrt(abs(self.box_poses[0,:] - rob_x)**2 + abs(self.box_poses[0,:] - rob_y)**2)
            low_ind = np.argmin(dists)
            goal_pose = self.box_poses[:, low_ind]

        if dists[low_ind] < 50:
            label = 'Near'
        else:
            label = 'Far'

        self.publish_goal(goal_pose, label)

    def publish_goal(self, goal_pose, label):
        #Publisg goal point to A star

        pose_msg = ObjectDetection1D()
        pose_msg.pose.position.x = float(goal_pose[0])
        pose_msg.pose.position.y = float(goal_pose[1])
        pose_msg.pose.position.z = 0.0
        pose_msg.label = label

        self.pose_pub.publish(pose_msg)

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