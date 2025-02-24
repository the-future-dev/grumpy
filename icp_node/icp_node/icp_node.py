import rclpy
import rclpy.duration
from rclpy.node import Node

import rclpy.time
from tf2_ros import TransformException,  TransformBroadcaster
from tf_transformations import quaternion_from_matrix, quaternion_matrix
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped, Vector3Stamped
from laser_geometry import LaserProjection
from nav_msgs.msg import Path
import tf2_geometry_msgs

import open3d
from open3d.pipelines.registration import registration_icp, TransformationEstimationPointToPoint, ICPConvergenceCriteria
import numpy as np

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class ICP_node(Node):
    def __init__(self):
        super().__init__('icp_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initalize publisher to publish pose after update step of EKF
        self.pose_with_cov_pub = self.create_subscription(PoseWithCovarianceStamped, '/localization/dead_reckoning_position', self.localization_pose_cb, 10)

        # Initialize the transform buffer
        self.tf_buffer = Buffer()

        # Initialize the transform listener
        self.tf_listener =  TransformListener(self.tf_buffer, self, spin_thread=True)

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        self.proj = LaserProjection()

        # Initalize reference cloud
        self.reference_cloud = PointCloud2()
        self.get_reference = True

        # intialize attribute for transform
        self.t = None
        self.t_mat_old = None
        self.outlier_margin = 0.5

        # Initalize counter
        self.counter = 0
        self.N = 10

        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, 'path', 10)
        # Store the path here
        self._path = Path()

        self.first_reference_cloud = True
        self.reference_counter = 0
        self.num_scans_for_reference = 10



    def extract_points_from_pointcloud(self, pointcloud: PointCloud2):
        """
        Method fro extracting points from pointcloud and filter out points
        """ 
        points = np.asarray([
            (p[0], p[1], p[2]) for p in read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True)
        ], dtype=np.float32)


    def create_open3d_pointcloud(self, transformed_cloud:PointCloud2):
        """
        Method for creating open3d pointscloud from PointCloud2.
        """
        points = np.asarray([
                (p[0], p[1], p[2]) for p in read_points(transformed_cloud, field_names=("x", "y", "z"), skip_nans=True)
            ], dtype=np.float32)


        # Create Open3D PointCloud
        cloud = open3d.geometry.PointCloud()
        cloud.points = open3d.utility.Vector3dVector(points)

        return cloud
    

    def lidar_callback(self, msg: LaserScan):
        """
        Callback for the lidar messages. This is used for the ICP algorithm.
        """

        if self.t is None:
            # send transform satying that icp and map is aligned
            self.t = TransformStamped()
            self.t.header.stamp = msg.header.stamp
            self.t.header.frame_id = "map"
            self.t.child_frame_id = "odom"
            self._tf_broadcaster.sendTransform(self.t)
            return

        # only process every nth scan
        if self.counter > 0:
            self.counter -= 1
            self.t.header.stamp = msg.header.stamp
            self._tf_broadcaster.sendTransform(self.t)
            return

        # Get transform between odom and frame_id
        to_frame_rel = 'odom'
        from_frame_rel = msg.header.frame_id
        time = rclpy.time.Time().from_msg(msg.header.stamp)

        # Wait for the transform asynchronously
        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time)

        # Spin until transform found or `timeout_sec` seconds has passed
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel,time)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return


        # Project lidar to point cloud
        cloud = self.proj.projectLaser(msg)

        # Transform point cloud to odom
        cloud_odom = do_transform_cloud(cloud, t)

        # # Transform point cloud to map
        # transformed_cloud = do_transform_cloud(cloud_odom, self.t)

        # Transform to open3d format
        pointcloud = self.create_open3d_pointcloud(cloud_odom)

        # set reference cloud if it is first scan
        
        if self.get_reference:
            self.reference_cloud = pointcloud
            self.get_reference = False


        if self.t_mat_old is None:
            previous_transform = np.identity(4)
        else:
            previous_transform = self.t_mat_old

        # Do ICP algorithm
        result_icp = registration_icp(source=pointcloud,
                        target=self.reference_cloud,
                        max_correspondence_distance= 0.02,
                        init=previous_transform, # The algorithm start with this and then tries to optimize for a better transform for map-odom
                        estimation_method=TransformationEstimationPointToPoint(),
                        criteria=ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=1000)
                        )

        # Checking overlap and inlier rmse to make sure transform is good to use
        overlap = result_icp.fitness 
        rmse = result_icp.inlier_rmse
       
        if overlap < 0.3 or rmse > 0.1:
            self.get_logger().info(f'Not good enough conditions, fitness: {overlap}, rmse: {rmse}')
            return

        # reset counter since an update is going to be processed
        self.counter = self.N

        # copy result from icp, has to copy to be able to write to it
        transform_matrix = result_icp.transformation.copy()

        # Make sure that we are only operating in the xy-plane
        transform_matrix[2,3] = 0.0 # zero out translation in z
        # no rotations around x or y axises
        transform_matrix[:3, 2] = [0.0, 0.0, 1.0] 
        transform_matrix[2, :3] = [0.0, 0.0, 1.0]

        self.t_mat_old = transform_matrix

        # Update map and odom transform with result from ICP
        self.t.header.frame_id = "map"
        self.t.child_frame_id = "odom"
        self.t.header.stamp = msg.header.stamp

        # Extract translation from transformation matrix
        self.t.transform.translation.x = transform_matrix[0, 3]
        self.t.transform.translation.y = transform_matrix[1, 3]
        self.t.transform.translation.z = 0.0 # again make sure that translation in z is zero

        # Extract rotation as quaternion
        quat = quaternion_from_matrix(transform_matrix)
        self.t.transform.rotation.x = 0.0 # rotation around x should not happen so set it to zero
        self.t.transform.rotation.y = 0.0 # rotation around y should not happen so set it to zero
        self.t.transform.rotation.z = quat[2]
        self.t.transform.rotation.w = quat[3]

        self._tf_broadcaster.sendTransform(self.t)


    def localization_pose_cb(self, msg: PoseWithCovarianceStamped):
        """
        Callback for publishing the path from the odometry with the integration of the transform between map and odom
        """
        if self.t is None:
            return

        pose_with_covariance = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(msg, self.t)

        pose = PoseStamped()
        pose.header = self._path.header

        pose.pose.position.x = pose_with_covariance.pose.pose.position.x
        pose.pose.position.y = pose_with_covariance.pose.pose.position.y
        pose.pose.orientation.z = pose_with_covariance.pose.pose.orientation.z
        pose.pose.orientation.w = pose_with_covariance.pose.pose.orientation.w

        # Update the path
        self._path.header.stamp = msg.header.stamp
        self._path.header.frame_id = 'map'

        self._path.poses.append(pose)
        self._path_pub.publish(self._path)


def main():
    rclpy.init()
    node = ICP_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()