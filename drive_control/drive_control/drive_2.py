import math
import rclpy
import rclpy.clock
from rclpy.node import Node
import rclpy.time
from robp_interfaces.msg import DutyCycles
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
import random
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from occupancy_grid_map.workspace_utils import Workspace

class SampleDriveControlNode(Node):

    def __init__(self):
        super().__init__('sample_drive_control_node')

        self.ws_utils = Workspace()
        self.phase = self.ws_utils.phase

        if self.phase == 'collection':

            self.vel_forward = 0.2 # 0.13 before
            self.vel_rotate = 0.09 # 0.09 before
            self.vel_small_rotate = 0.015
            self.vel_arrived = 0.0
            self.right_extra = 1.075

        elif self.phase == 'exploration':

            self.vel_forward = 0.13 # 0.13 before
            self.vel_rotate = 0.09 # 0.09 before
            self.vel_small_rotate = 0.015
            self.vel_arrived = 0.0
            self.right_extra = 1.075

        # stop variable
        self.stop = False

        # Initialize map odom transform variable and subscribe to topic where tf is published
        self.tf_map_odom = None
        self.create_subscription(TransformStamped, 'ICP/have_new_transform', self.map_odom_tf_cb, 1)

        #Create buffer to look for transform 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        #Create publisher to publish motor control
        self.motor_publisher = self.create_publisher(DutyCycles, 'motor/duty_cycles', 10)
        self.drive_feedback = self.create_publisher(Bool, 'drive/feedback', 1)

        self.create_subscription(Path, 'drive/path', self.path_cb, 1)
        self.create_subscription(Bool, 'drive/stop', self.stop_cb, 2)        

    def map_odom_tf_cb(self, msg:TransformStamped):
        """
        Callback that sets the latest map-odom transform from ICP node
        """
        # self.get_logger().info('Updated map-odom transform from ICP node')
        self.tf_map_odom = msg
    
    def path_cb(self, msg:Path):
        #Call back that iterates in poses and drives to them, can maybe implement offset if it is needed

        # if self.stop == True:
        #     self.get_logger().info(f'Stopping ')
        #     msg_stop = DutyCycles()
        #     msg_stop.duty_cycle_right = 0.0
        #     msg_stop.duty_cycle_left = 0.0
        #     self.motor_publisher.publish(msg_stop)
        #     self.stop = False
        #     return

        for pose in msg.poses:
          result =  self.set_drive_input(pose.pose.position.x, pose.pose.position.y, msg.header.frame_id)
          if result == False:
              break

        msg_stop = DutyCycles()
        msg_stop.duty_cycle_right = 0.0
        msg_stop.duty_cycle_left = 0.0
        self.motor_publisher.publish(msg_stop)

        msg_feedback = Bool()
        msg_feedback.data = result
        self.drive_feedback.publish(msg_feedback)
    
    def stop_cb(self, msg:Bool):
        #Callback that sets stop variable to True if stop
        self.stop = msg.data

    # def drive_free_cb(self, msg:Path):
    #     self.get_logger().info('In drive free callback')

    #     self.drive_to_free = True 
    #     self.stop = False
        
    #     for pose in msg.poses:
    #         self.set_drive_input(pose.pose.position.x, pose.pose.position.y)

    #     self.drive_to_free = False
    #     msg_stop = DutyCycles()
    #     msg_stop.duty_cycle_left = 0.0
    #     msg_stop.duty_cycle_right = 0.0
    #     self.motor_publisher.publish(msg_stop)

    def set_drive_input(self, x, y, given_frame_id):

        msg = DutyCycles()
        sample_point = PointStamped()
        from_frame = 'map'

        sample_point.header.frame_id = 'map'
        sample_point.header.stamp = rclpy.time.Time()
        sample_point.point.x = x
        sample_point.point.y = y
        sample_point.point.z = 0.0

        #While loop that rotates robot until aligned   
        while True:

            # if self.stop == True and self.drive_to_free == False:
            #     self.get_logger().info(f'Stopping, in occupied zone')
            #     return False
            tf_future = self.tf_buffer.wait_for_transform_async('base_link', from_frame, self.get_clock().now())
            rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

            try:
                tf_odom_base_link = self.tf_buffer.lookup_transform('base_link', from_frame, rclpy.time.Time()) # get latest available transform
            except TransformException as ex:
                self.get_logger().info(f'could not transform{ex}')
                continue
            
            # # tranform point from map to odom, using transform from callback or if it is not set yet, assume map-odom is the same
            # if self.tf_map_odom is None:
            #     point_odom = sample_point
            # else:   
            #     point_odom  = tf2_geometry_msgs.do_transform_point(sample_point, self.tf_map_odom)

            #Transform point from odom to base_link
            point_base_link = tf2_geometry_msgs.do_transform_point(sample_point, tf_odom_base_link)
            x = point_base_link.point.x
            y = point_base_link.point.y
        
            #If y is zero and x > 0 means perfect alignment otherwise turning
            if x >= 0.0 and abs(y) < 0.15:
                #Stop turning
                msg.duty_cycle_right = self.vel_arrived*self.right_extra
                msg.duty_cycle_left = self.vel_arrived
                self.motor_publisher.publish(msg)
                break
            elif y >= 0.0:
                #Turn left
                msg.duty_cycle_right = self.vel_rotate*self.right_extra
                msg.duty_cycle_left = -self.vel_rotate
                self.motor_publisher.publish(msg)
            elif y < 0.0:
                #Turn right
                msg.duty_cycle_right = -self.vel_rotate*self.right_extra
                msg.duty_cycle_left = self.vel_rotate
                self.motor_publisher.publish(msg)
 
        #While loop that drives forward until reaching point
        while True:

            # if self.stop == True and self.drive_to_free == False:
            #     return False
            tf_future = self.tf_buffer.wait_for_transform_async('base_link', from_frame, self.get_clock().now())
            rclpy.spin_until_future_complete(self, tf_future, timeout_sec=0.5)

            try:
                tf_odom_base_link = self.tf_buffer.lookup_transform('base_link', from_frame, rclpy.time.Time()) # get latest available transform
            except TransformException as ex:
                self.get_logger().info(f'could not transform{ex}')
                continue
            
            # # tranform point from map to odom, using transform from callback or if it is not set yet, assume map-odom is the same
            # if self.tf_map_odom is None:
            #     point_odom = sample_point
            # else:   
            #     point_odom  = tf2_geometry_msgs.do_transform_point(sample_point, self.tf_map_odom)

            #Transform point from odom to base_link
            point_base_link = tf2_geometry_msgs.do_transform_point(sample_point, tf_odom_base_link)
            x = point_base_link.point.x
            y = point_base_link.point.y

            if abs(x) < 0.05:
                #Stop driving
                msg.duty_cycle_right = self.vel_arrived*self.right_extra
                msg.duty_cycle_left = self.vel_arrived
                self.motor_publisher.publish(msg)
                self.get_logger().info(f'SUCCESS, point reached')
                return True
            elif y > 0.05:
                #Small turn left
                msg.duty_cycle_right = (self.vel_forward + self.vel_small_rotate)*self.right_extra
                msg.duty_cycle_left = self.vel_forward - self.vel_small_rotate
                self.motor_publisher.publish(msg)
            elif y < 0.05:
                #Small turn right
                msg.duty_cycle_right = (self.vel_forward - self.vel_small_rotate)*self.right_extra
                msg.duty_cycle_left = self.vel_forward + self.vel_small_rotate
                self.motor_publisher.publish(msg)
            else:
                #Drive forward
                msg.duty_cycle_right = self.vel_forward*self.right_extra
                msg.duty_cycle_left = self.vel_forward
                self.motor_publisher.publish(msg)


    


def main():
    rclpy.init()
    node = SampleDriveControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()