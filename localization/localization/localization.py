#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node



from tf2_ros import TransformException, TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import  Imu, LaserScan
from robp_interfaces.msg import Encoders
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseStamped
from nav_msgs.msg import Path

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Localization(Node):

    def __init__(self):
        super().__init__('Localization')

        # Initialize the transform listener and assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Initalize publisher to publish pose after update step of EKF
        self.pose_with_cov_pub = self.create_publisher(PoseWithCovarianceStamped, 'dead_reckoning_position', 10)

        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, 'path', 10)
        # Store the path here
        self._path = Path()

        # Subscribe to encoder topic and call callback function on each recieved message
        self.create_subscription(Encoders, '/motor/encoders', self.encoder_callback, 10)

        # Intialize subscriber to imu
        self.create_subscription(Imu, '/imu/data_raw', self.imu_callback , 10)
        self.first_imu_msg = True # to be able to handle dt

        # initialization of parameters for EKF
        self.first_encoder_msg = True
        self.vel = np.array([[0.0], [0.0], [0.0]]) # keep track of velocity to be able to predict position from IMU data
        self.pos = np.array([[0.0], [0.0], [0.0]]) # keep track of position from imu
        
        self.mu_bar = np.array([[0.0], [0.0], [0.0]]) # estimated mean of he robot's position after predict step, in the order (x,y,theta)
        self.mu = np.array([[0.0], [0.0], [0.0]]) # estimated mean of he robot's position after update step, in the order (x,y,theta)
        
        d = np.array([1.0, 1.0, 1.0]) # variance in the order (x,y,theta)
        self.sigma_bar = np.diag(d) # covariance matrix of the robot's position after predict step, initialized as a diagonal matrix
        self.sigma = np.diag(d) # covariance matrix of the robot's position after update step, initialized as a diagonal matrix

        # covariance matrices for the uncertainty in the motion model and measurement model
        d_R = np.array([1.0, 1.0, 1.0]) # variance in the order (x,y,theta)
        self.R = np.diag(d_R)
        d_Q = np.array([1.0, 1.0, 1.0]) # variance in the order (x,y, theta)
        self.Q = np.diag(d_Q)

        # Initalize subscriber to lidar node
        self.create_subscription(LaserScan, '/scan', self.lidar_callback , 10)

        # Intialize to true to be able to save the first lidar scan
        self.first_lidar_scan = True
   
    
    def encoder_callback(self, msg: Encoders):
        """
        Take motor encoder readings and use them to do the prediction step of the Extended Kalman Filter
        """
        # The kinematic parameters for the differential configuration
        dt = 50 / 1000
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

        # Jacobian of motion model
        G = np.array([[1.0, 0.0, -D*np.sin(self.mu[2])],
                      [0.0, 1.0, D*np.cos(self.mu[2])],
                      [0.0, 0.0, 1.0]])

        # Doing the predict step
        self.mu_bar = self.mu + np.array([[D*np.cos(self.mu[2])],
                                          [D*np.sin()],
                                          [dtheta]])
        self.sigma_bar = np.dot(np.dot(G, self.sigma), G.T) + self.R


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
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self._tf_broadcaster.sendTransform(t)

    
    def publish_path(self, x, y, q):
        """
        Method for publishing the path of the robot
        """
        # Update the path
        pose = PoseStamped()
        pose.header = self._path.header

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        self._path.poses.append(pose)
        self._path_pub.publish(self._path)


    def publish_pose_with_covariance(self, stamp, x, y, q):
        """
        Method for publishing the current pose with covariance from sigma. Note that the covariance matrix of this message type is 6x6 (x,y,z,roll,pitch,yaw)
        """

        pose_with_covariance = PoseWithCovarianceStamped()
        pose_with_covariance.header.stamp = stamp
        pose_with_covariance.header.frame_id = 'odom'

        pose_with_covariance.pose.pose.position.x = x
        pose_with_covariance.pose.pose.position.y = y
        pose_with_covariance.pose.pose.position.z = 0

        pose_with_covariance.pose.pose.orientation.x = q[0]
        pose_with_covariance.pose.pose.orientation.y = q[1]
        pose_with_covariance.pose.pose.orientation.z = q[2]
        pose_with_covariance.pose.pose.orientation.w = q[3]

        covariance_matrix = np.zeros((6, 6))
        covariance_matrix[:2, :2] = self.sigma[:2, :2]
        covariance_matrix[5, 5] = self.sigma[2, 2]
        covariance_matrix[0, 5] = self.sigma[0, 2]
        covariance_matrix[5, 0] = self.sigma[2, 0]
        covariance_matrix[1, 5] = self.sigma[1, 2]
        covariance_matrix[5, 1] = self.sigma[2, 1]

        pose_with_covariance.pose.covariance = covariance_matrix.flatten().tolist()               

        self.pose_with_cov_pub.publish(pose_with_covariance)


    def imu_callback(self, msg: Imu):
        """
        Take IMU data to do the update step of the EKF and then publish the transform between odom and baselink
        """
        # If this is the first imu message we use the encoder information only and after that updates with imu 
        if self.first_imu_msg:
            self.mu = self.mu_bar
            self.sigma = self.sigma_bar
            self.t_old = msg.header.stamp.nanosec
            self.first_imu_msg = False
        else:
            # Calculating delta time and updating old time
            current_time = msg.header.stamp.nanosec
            dt = (current_time - self.t_old)*1e-9 # tranform to seconds from nanonseconds
            self.t_old = current_time

            # Estimate position from imu by integrating two times. This is then used as measurement for the update step in the EKF. 
            acc = np.array([[msg.linear_acceleration.x], [msg.linear_acceleration.y], [msg.linear_acceleration.z]])
            self.vel += acc*dt
            self.pos += self.vel*dt

            # Kalman gain, jacobian of measurement model is an indentity matrix and is therefore not included
            K = np.dot(self.sigma_bar, np.linalg.inv(self.sigma_bar+self.Q))

            # innovation
            quaternion_imu = np.array([[msg.orientation.x], [msg.orientation.y], [msg.orientation.z], [msg.orientation.w]])
            euler_angles = euler_from_quaternion(quaternion_imu)
            theta_imu = euler_angles[2]
            innovation = np.array([[self.pos[0]         -self.mu_bar[0]],
                                [self.pos[1]         -self.mu_bar[1]],
                                [theta_imu           -self.mu_bar[2]]])
            
            # update mean and covariance matrix
            self.mu = self.mu_bar + np.dot(K, innovation)
            self.sigma = np.dot((np.eye(3)-K), self.sigma_bar)

        x = self.mu[0]
        y = self.mu[1]
        yaw = self.mu[2]
        q = quaternion_from_euler(0.0, 0.0, yaw)
        stamp = msg.header.stamp

        self.broadcast_transform(stamp, x, y, q)
        
        self.publish_path(x, y, q)

        # Not needed right now
        # self.publish_pose_with_covariance(stamp, x, y, q)


    def lidar_callback(self, msg: LaserScan):
        if self.first_lidar_scan:
            self.lidar_control = msg
            self.first_lidar_scan = False
        else:
            # Do ICP on new lidar scan and broadcast the new transform between map and odom
            pass
        

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