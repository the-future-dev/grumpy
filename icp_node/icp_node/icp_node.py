import rclpy
import rclpy.duration
from rclpy.node import Node

import rclpy.time
from tf2_ros import TransformException,  TransformBroadcaster
from tf_transformations import quaternion_from_matrix, quaternion_matrix, euler_from_quaternion
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points, read_points_numpy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped, Vector3Stamped
from laser_geometry import LaserProjection
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import tf2_geometry_msgs

import open3d
from open3d.pipelines.registration import registration_icp, TransformationEstimationPointToPoint, ICPConvergenceCriteria
import numpy as np
import math

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class ICP_node(Node):
    def __init__(self):
        super().__init__('icp_node')

        self.x = 0.0
        self.y = 0.0

        # Subscribe to lidar
        self.create_subscription(LaserScan,'/scan',self.lidar_callback,1)

        # Initialize subscriber pose from update step of EKF
        self.create_subscription(PoseWithCovarianceStamped, '/localization/dead_reckoning_position', self.localization_pose_cb, 1)

        # Initialize the transform buffer§
        self.tf_buffer = Buffer()

        # Initialize the transform listener
        self.tf_listener =  TransformListener(self.tf_buffer, self, spin_thread=True)

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # initialize calls to transform laserscan to pointcloud
        self.proj = LaserProjection()

        # initialize threshold distance to filter out points close to the robot
        self.threshold_distance = 0.35

        # Initialize reference cloud
        self.reference_clouds = []

        # Initialize bool to handle getting get_new_reference_scan
        self.get_new_reference_scan = True

        # intialize attribute for transform
        self.t = None
        self.t_mat = None

        # Initialize counters
        # self.counter = 0
        # self.N = 10
        self.bad_scan_counter  = 0
        self.threshold_new_scan = 5

        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, 'path', 10)
        # Store the path here
        self._path = Path()


    def create_open3d_pointcloud(self, transformed_cloud:PointCloud2):
        """
        Method for creating open3d pointscloud from PointCloud2 and filtering out points.
        """
        points = read_points_numpy(transformed_cloud, field_names=("x", "y", "z"), skip_nans=True, reshape_organized_cloud=True)

        distance = np.hypot(points[:, 0], points[:, 1])
        distance = distance.reshape(-1, 1)

        # self.get_logger().info(f'Num of points before filter: {points.shape[0]}')
        
        mask,_ = np.where(distance > self.threshold_distance)
        points = points[mask, :]

        # self.get_logger().info(f'Num of points after filter: {points.shape[0]}')

        # Create Open3D PointCloud with points
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
        # if self.counter > 0:                                  # CHANGED SO THAT EVERY SCAN IS PROCESSED
        #     self.counter -= 1
        #     self.t.header.stamp = msg.header.stamp
        #     self._tf_broadcaster.sendTransform(self.t)
        #     return

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
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Project lidar to point cloud
        cloud = self.proj.projectLaser(msg)

        # Transform point cloud to odom
        cloud_odom = do_transform_cloud(cloud, t)

        # Transform point cloud to map
        transformed_cloud = do_transform_cloud(cloud_odom, self.t)

        # Transform to open3d format
        pointcloud = self.create_open3d_pointcloud(transformed_cloud)

        # get new reference scan if bool is true, this is the case in the origin and when set by the brain
        if self.get_new_reference_scan:
            self.get_new_reference_scan = False
            # pos = (self.x,self.y)
            # self.get_logger().info(f'Position from localization when getting new reference scan; x: {self.x}, y: {self.y}')

            # append position of robot for the pointcloud and the pointcloud transformed to odom frame
            # self.reference_clouds.append((pos, pointcloud))
            self.reference_clouds.append(pointcloud)

            # set reference cloud
            # self.ref_cloud = pointcloud
        
        no_good_icp = True
        # self.get_logger().info('-----------------Looking for good ICP result---------------')
        if len(self.reference_clouds)>0:
            # self.get_logger().info(f'number of reference clouds: {len(self.reference_clouds)}')
            # iter = 1
            best_overlap = 0
            for ref_cloud in self.reference_clouds:
                # Do ICP algorithm for the current reference cloud
                result_icp = registration_icp(source=pointcloud,
                        target=ref_cloud,
                        max_correspondence_distance= 0.05,
                        init=np.identity(4), 
                        estimation_method=TransformationEstimationPointToPoint(),
                        criteria=ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=1000)
                        )
                # self.get_logger().info(f'iter: {iter}, overlap: {result_icp.fitness}, rmse: {result_icp.inlier_rmse}')
                
                if result_icp.fitness >= 0.25 and result_icp.inlier_rmse < 0.1:
                    t_mat = result_icp.transformation.copy()
                    new_x_y = np.array([t_mat[0, 3], t_mat[1, 3]])
                    if np.linalg.norm(new_x_y) < 0.10:
                        # norm also okay so we have a good icp result
                        no_good_icp = False
                        if result_icp.fitness > best_overlap:
                            # self.get_logger().info(f'Taking reference cloud from iteration: {iter}')
                            # copy result from icp, have to copy to be able to write to it
                            transform_matrix = t_mat
                    # else:
                        # self.get_logger().info(f'Too high norm for reference cloud in iteration: {iter}')
                # iter +=1

        # # if there is two or more reference scans in the list compare which one is closest and use that as reference cloud when doing ICP algorithm
        # if len(self.reference_clouds) >= 2:
        #     self.get_logger().debug('Have more than one reference cloud')
        #     min_distance = np.inf
        #     for pos_cloud, cloud in self.reference_clouds:
        #         distance = math.hypot(self.x-pos_cloud[0], self.y-pos_cloud[1])

        #         if distance < min_distance:
        #             self.get_logger().debug('setting new reference cloud')
        #             self.ref_cloud = cloud

        # # if this it the first transform set the initial guess to be an indentity matrix, otherwise use the old transform as guess
        # if self.t_mat_old is None:
        #     previous_transform = np.identity(4)
        # else:
        #     previous_transform = self.t_mat_old

        # # Do ICP algorithm
        # result_icp = registration_icp(source=pointcloud,
        #                 target=self.ref_cloud,
        #                 max_correspondence_distance= 0.10,
        #                 init=previous_transform, # The algorithm start with this and then tries to optimize for a better transform for map-odom
        #                 estimation_method=TransformationEstimationPointToPoint(),
        #                 criteria=ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=1000)
        #                 )

        # # Checking overlap and inlier rmse to make sure transform is good to use
        # overlap = result_icp.fitness 
        # rmse = result_icp.inlier_rmse
        # # self.get_logger().info(f'fitness: {overlap}, rmse: {rmse}')

        # if overlap < 0.3 or rmse > 0.1:
        #     # self.get_logger().info(f'Not good enough conditions, fitness: {overlap}, rmse: {rmse}')
        #     self.t.header.stamp = msg.header.stamp
        #     self._tf_broadcaster.sendTransform(self.t)

        #     self.bad_scan_counter += 1

        #     if self.bad_scan_counter >= self.threshold_new_scan:
        #         self.get_logger().info(f'{self.threshold_new_scan} insufficent scans in a row, getting new reference scan')
        #         # self.get_new_reference_scan = True 
        #         self.bad_scan_counter = 0 # reset counter
        #     return

        # if no good icp result was found increment bad scans and send the latest transform
        if no_good_icp:
            self.t.header.stamp = msg.header.stamp
            self._tf_broadcaster.sendTransform(self.t)
            self.bad_scan_counter += 1
            if self.bad_scan_counter >= self.threshold_new_scan:
                self.get_logger().info(f'{self.threshold_new_scan} insufficent scans in a row, getting new reference scan')
                self.get_new_reference_scan = True 
                self.bad_scan_counter = 0 # reset counter
            return


        # # copy result from icp, have to copy to be able to write to it
        # transform_matrix = result_icp.transformation.copy()

        # Make sure that we are only operating in the xy-plane
        transform_matrix[2,3] = 0.0 # zero out translation in z
        # no rotations around x or y axises
        transform_matrix[:3, 2] = [0.0, 0.0, 1.0] 
        transform_matrix[2, :3] = [0.0, 0.0, 1.0]

        # # Compare with previous translation in transform so that the new transform will not move robot too much
        # comp_transforms = np.array([[self.t.transform.translation.x - transform_matrix[0, 3]], 
        #                             [self.t.transform.translation.y - transform_matrix[1, 3]]])

        # if np.linalg.norm(comp_transforms) > 0.1:
        #     # self.get_logger().info(f'The norm was too high: {np.linalg.norm(comp_transforms)}')
        #     self.t.header.stamp = msg.header.stamp
        #     self._tf_broadcaster.sendTransform(self.t)
        #     return 

        # reset bad scan counter so that it is only when multiple scans are bad in a row that triggers a new reference scan
        self.bad_scan_counter = 0
        
        self.get_logger().debug('Updating map-odom tf')
        
        # reset counter since an update is going to be processed
        # self.counter = self.N

        if self.t_mat is None:
            self.t_mat = transform_matrix
        else:
            self.t_mat = transform_matrix @ self.t_mat # increment transform matrix with previous transform matrix

        # Update map and odom transform with result from ICP
        self.t.header.frame_id = "map"
        self.t.child_frame_id = "odom"
        self.t.header.stamp = msg.header.stamp

        # Extract translation from transformation matrix
        self.t.transform.translation.x = self.t_mat[0, 3]
        self.t.transform.translation.y = self.t_mat[1, 3]
        self.t.transform.translation.z = 0.0 # again make sure that translation in z is zero

        # Extract rotation as quaternion
        quat = quaternion_from_matrix(self.t_mat)
        self.t.transform.rotation.x = 0.0 # rotation around x should not happen so set it to zero
        self.t.transform.rotation.y = 0.0 # rotation around y should not happen so set it to zero
        self.t.transform.rotation.z = quat[2]
        self.t.transform.rotation.w = quat[3]

        self._tf_broadcaster.sendTransform(self.t)
        # self.have_new_transform_pub.publish(self.t)

    # def get_new_ref_scan_cb(self, msg:Bool):
    #     """
    #     Callback to set the bool variable get
    #     """
    #     self.get_new_reference_scan = msg.data


    def localization_pose_cb(self, msg: PoseWithCovarianceStamped):
        """
        Callback for publishing the path from the odometry with the integration of the transform between map and odom
        """
        if self.t is None:
            return

        pose_with_covariance = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(msg, self.t)

        pose = PoseStamped()
        pose.header = self._path.header

        [_, _, yaw] = euler_from_quaternion([pose_with_covariance.pose.pose.orientation.x, pose_with_covariance.pose.pose.orientation.y, pose_with_covariance.pose.pose.orientation.z,pose_with_covariance.pose.pose.orientation.w])

        self.x = pose_with_covariance.pose.pose.position.x
        self.y = pose_with_covariance.pose.pose.position.y 
        
        self.get_logger().debug(f'Current position according to localization; x: {self.x}, y: {self.y}, theta: {yaw}')
        
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