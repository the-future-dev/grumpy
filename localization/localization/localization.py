#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException, TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
from sensor_msgs.msg import  Imu, LaserScan
from robp_interfaces.msg import Encoders
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseStamped, Pose
from nav_msgs.msg import Path

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from scipy.spatial.transform import Rotation as R


class Localization(Node):

    def __init__(self):
        super().__init__('Localization')

        # Initialize the transform listener and assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Initalize publisher to publish pose after update step of EKF
        self.pose_with_cov_pub = self.create_publisher(PoseWithCovarianceStamped, '/localization/dead_reckoning_position', 10)

        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, 'path', 10)
        # Store the path here
        self._path = Path()

        # Subscribe to encoder topic and call callback function on each recieved message
        self.create_subscription(Encoders, '/motor/encoders', self.encoder_callback, 10)

        # Intialize subscriber to imu
        self.create_subscription(Imu, '/imu/data_raw', self.imu_callback , 10)

        # initialization of parameters for EKF
        self.first_encoder_msg = True
        self.vel = np.array([[0.0], [0.0]]) # keep track of velocity to be able to predict position from IMU data
        self.pos = np.array([[0.0], [0.0], [0.0]]) # keep track of position from imu
        
        self.mu_bar = np.array([[0.0], [0.0], [0.0]]) # estimated mean of he robot's position after predict step, in the order (x,y,theta)
        self.mu = np.array([[0.0], [0.0], [0.0]]) # estimated mean of he robot's position after update step, in the order (x,y,theta)
        
        d = np.array([0.0, 0.0, 0.0]) # variance in the order (x,y,theta)
        self.sigma_bar = np.diag(d) # covariance matrix of the robot's position after predict step, initialized as a diagonal matrix
        self.sigma = np.diag(d) # covariance matrix of the robot's position after update step, initialized as a diagonal matrix

        # covariance matrices for the uncertainty in the motion model and measurement model
        d_R = np.array([0.01**2, 0.01**2, 0.01]) # variance in the order (x,y,theta)
        self.R = np.diag(d_R)

        self.Q = np.array([0.01]) # variance in theta, for only updating angle via IMU

        # initialize boolean to get predict and update to happen after each other
        self.new_encoder_pos = False
        
    

    def wrap_angle(self, angle:np.array):
        """
        Method that takes the angle and returns the equivalent in the [-pi,pi] range.
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    
    def encoder_callback(self, msg: Encoders):
        """
        Take motor encoder readings and use them to do the prediction step of the Extended Kalman Filter
        """
        # The kinematic parameters for the differential configuration
        ticks_per_rev = 48 * 64
        wheel_radius =  0.04921 
        base = 0.31 # actual base: 0.3, but can be experimented with to improve performance, found 0.31 to be a bit better

        # Ticks since last message
        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right

        # scaling factor that takes into account that we want radians and also that the number of ticks per revolution yields a fraction of the rotations of the wheels compared to a full revolution
        K = (2*math.pi)/ticks_per_rev 

        # Calculating distance D and angle difference dtheta
        r = wheel_radius
        B = base
        D = (r/2)*K*(delta_ticks_right + delta_ticks_left) # v*dt
        dtheta = (r/B)*K*(delta_ticks_right - delta_ticks_left) # w*dt

        theta = float(self.mu[2])

        # Jacobian of motion model
        G = np.array([[1.0, 0.0, -D*math.sin(theta)],
                      [0.0, 1.0, D*math.cos(theta)],
                      [0.0, 0.0, 1.0]])

        # Doing the predict step
        self.mu_bar = self.mu + np.array([[D*math.cos(theta)],
                                          [D*math.sin(theta)],
                                          [dtheta]])
        
        # set angle to be in the interval [-pi, pi]
        self.mu_bar[2] =  self.wrap_angle(self.mu_bar[2])

        self.sigma_bar = np.dot(np.dot(G, self.sigma), G.T) + self.R

        self.new_encoder_pos = True



    def broadcast_transform(self, stamp, x, y, q):
        """
        Method to broadcast transform between odom and base_link based on updated mean position
        """
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # The robot only exists in 2D, thus we set x and y translation
        # coordinates and set the z coordinate to 0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # For the same reason, the robot can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self._tf_broadcaster.sendTransform(t)

    
    # def publish_path(self, stamp, x, y, q):
    #     """
    #     Method for publishing the path of the robot
    #     """
        
    #     # Get transform between map and odom
    #     to_frame_rel = 'map'
    #     from_frame_rel = 'odom'
    #     time = rclpy.time.Time().from_msg(stamp)

    #     # Wait for the transform asynchronously
    #     tf_future = self.tf_buffer.wait_for_transform_async(
    #         target_frame=to_frame_rel,
    #         source_frame=from_frame_rel,
    #         time=time)

    #     # Spin until transform found or `timeout_sec` seconds has passed
    #     rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

    #     try:
    #         t = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel,time)
        
    #     except TransformException as ex:
    #         self.get_logger().info(
    #             f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
    #         return

    #     pose = PoseStamped()
    #     pose.header = self._path.header

    #     pose.pose.position.x = x
    #     pose.pose.position.y = y
    #     pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

    #     pose.pose.orientation.x = q[0]
    #     pose.pose.orientation.y = q[1]
    #     pose.pose.orientation.z = q[2]
    #     pose.pose.orientation.w = q[3]

    #     pose = tf2_geometry_msgs.do_transform_pose_stamped(pose, t)

    #     # Update the path
    #     self._path.header.stamp = stamp
    #     self._path.header.frame_id = 'map'

    #     self._path.poses.append(pose)
    #     self._path_pub.publish(self._path)


    def publish_pose_with_covariance(self, stamp, x:float, y:float, q:tuple) -> None:
        """
        Method for publishing the current pose with covariance from sigma. Note that the covariance matrix of this message type is 6x6 (x,y,z,roll,pitch,yaw)
        """

        pose_with_covariance = PoseWithCovarianceStamped()
        pose_with_covariance.header.stamp = stamp
        pose_with_covariance.header.frame_id = 'odom'

        pose_with_covariance.pose.pose.position.x = x
        pose_with_covariance.pose.pose.position.y = y
        pose_with_covariance.pose.pose.position.z = 0.0

        pose_with_covariance.pose.pose.orientation.x = q[0]
        pose_with_covariance.pose.pose.orientation.y = q[1]
        pose_with_covariance.pose.pose.orientation.z = q[2]
        pose_with_covariance.pose.pose.orientation.w = q[3]

        covariance_matrix = np.zeros((6, 6), dtype=float)
        covariance_matrix[:2, :2] = self.sigma[:2, :2]
        covariance_matrix[5, 5] = self.sigma[2, 2]
        covariance_matrix[0, 5] = self.sigma[0, 2]
        covariance_matrix[5, 0] = self.sigma[2, 0]
        covariance_matrix[1, 5] = self.sigma[1, 2]
        covariance_matrix[5, 1] = self.sigma[2, 1]

        pose_with_covariance.pose.covariance = covariance_matrix.flatten().tolist()               

        self.pose_with_cov_pub.publish(pose_with_covariance)


    def imu_callback(self, msg: Imu)->None:
        """
        Take IMU data to do the update step of the EKF and then publish the transform between odom and baselink
        """
        # Only update when a new encoder pose is avaiable
        if self.new_encoder_pos is False:
            return
     
        # Set encoder pose to False if a new one when it is received so it has to be set as true in encoder callback
        self.new_encoder_pos = False

        q_imu = np.array([0.0, 0.0, -msg.orientation.z, msg.orientation.w])

        # t = TransformStamped()
        # t.header.stamp = msg.header.stamp
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'imu_test'

        # t.transform.translation.x = 0.0
        # t.transform.translation.y = 0.0
        # t.transform.translation.z = 0.1

        # t.transform.rotation.x = q_imu[0]
        # t.transform.rotation.y = q_imu[1]
        # t.transform.rotation.z = q_imu[2]
        # t.transform.rotation.w = q_imu[3]

        # self._tf_broadcaster.sendTransform(t)

        # Kalman gain
        H = np.array([[0.0, 0.0, 1.0]]) # Only updates angle
        sigma_HT = np.dot(self.sigma_bar, H.T)
        H_sigma_HT = np.dot(np.dot(H, self.sigma_bar), H.T)

        K = np.dot(sigma_HT, np.linalg.inv(H_sigma_HT+self.Q))

        # innovation
        euler_angles = euler_from_quaternion(q_imu)
        theta_imu = euler_angles[2]

        # innovation for angle
        innovation = np.array([theta_imu-self.mu_bar[2]]) 
        innovation = self.wrap_angle(innovation) # wrap angle to always stay in interval [-pi,pi]

        # update mean and covariance matrix
        self.mu = self.mu_bar + np.dot(K, innovation)

        # constrain angle into interval [-pi,pi]
        self.mu[2] =  self.wrap_angle(self.mu[2])

        self.sigma = np.dot((np.eye(3)-np.dot(K, H)), self.sigma_bar)

        x = float(self.mu[0])
        y = float(self.mu[1])
        yaw = float(self.mu[2])

        # logging parameters to troubleshoot
        # self.get_logger().info(f"K: {K}")
        # self.get_logger().info(f"Innovation: {innovation}")
        # self.get_logger().info(f"Sigma: {np.diag(self.sigma)}")

        q = quaternion_from_euler(0.0, 0.0, yaw)
        stamp = msg.header.stamp

        self.broadcast_transform(stamp, x, y, q)
        
        # self.publish_path(stamp, x, y, q)

        self.publish_pose_with_covariance(stamp, x, y, q)


def main():
    rclpy.init()
    node = Localization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()