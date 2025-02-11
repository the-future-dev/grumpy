#!/usr/bin/env python

import numpy as np

import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, PointField
from rclpy.node import Node

from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose, TransformStamped
import tf2_geometry_msgs

# from perception_pkg.msg import ObjectDetection1D
# from msg import LabelPointStamped

import math
from sklearn.cluster import DBSCAN

class Detection(Node):

    def __init__(self):
        super().__init__('detection')
        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/perception/camera/ground_filter', 10)

        self._pub_detected_objects = self.create_publisher(
            PointCloud2, '/perception/detected_objects', 10)

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)

        # Initialize TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # self.static_transform = self.get_static_transform()
        self.get_logger().info(f"COMPLETED: Node initialization")
    
    def cloud_callback(self, msg: PointCloud2):
        # Convert ROS -> NumPy
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = np.asarray(gen[:, :3])
        # print(f"Esempio punto: {points[0]}")
        colors = gen[:, 3]
        self.get_logger().debug(f"COMPLETED: PointCloud to Numpy")

        # GROUND REMOVAL #######################################################################
        # Distance filter: approximate implementation as no transform
            # TODO: test
            # fast_transformed_points = np.asarray(self.fast_transform(points))
        distances = np.linalg.norm(points, axis=1)
        distance_rel_indexes = np.where(distances <= 0.7)[0]

        gen = gen[distance_rel_indexes, :]
        points = points[distance_rel_indexes, :]

        # Transformation from camera_depth_optical_frame to base_link for correct x,y,z
        points = self.fast_transform2(points)
        
        # Ground and heigh filter
        z_values = np.asarray(points)[:, 2]
        relevant_mask = (np.abs(z_values) <= 0.2) & (z_values > 0.012)  
        relevant_indices = np.where(relevant_mask)[0]

        gen = gen[relevant_indices, :]
        points = points[relevant_indices, :]

        self.get_logger().debug(f"COMPLETED: Filter for distance and height")
        
        msg_object_detected = pc2.create_cloud(msg.header, msg.fields, gen)
        self._pub.publish(msg_object_detected)

        # Point Clustering #######################################################################
        if points.size == 0:
            self.get_logger().debug("No relevant points detected after filtering.")
            # Publish an empty pointcloud or handle it as needed
            empty_cloud = pc2.create_cloud(msg.header, msg.fields, [])
            self._pub_detected_objects.publish(empty_cloud)
            return
         
        filtered_points = points # self.voxel_grid_filter(points, leaf_size=0.05)
        db = DBSCAN(eps=0.5, min_samples=40)
        labels = db.fit_predict(filtered_points)
        self.get_logger().debug(f"COMPLETED: Point Cluster")
        
        detected_objects = []
        classified_labels = []
        unique_labels = set(labels)
        
        if len(unique_labels) > 0:
            for label in unique_labels:
                if label == -1: # Ignore noise
                    continue
                
                cluster_points = filtered_points[labels == label]

                if cluster_points.shape[0] < 10:
                    self.get_logger().info(f"Neglecting small cluster")
                    continue

                # Compute Bounding Box (More Robust)
                # 1. Find min/max along each axis
                min_x = np.min(cluster_points[:, 0])
                max_x = np.max(cluster_points[:, 0])
                min_y = np.min(cluster_points[:, 1])
                max_y = np.max(cluster_points[:, 1])
                min_z = np.min(cluster_points[:, 2])
                max_z = np.max(cluster_points[:, 2])

                bbox_min = np.array([min_x, min_y, min_z])
                bbox_max = np.array([max_x, max_y, max_z])
                bbox_size = bbox_max - bbox_min
                bbox_size_cm = bbox_size * 100 

                # Classify Objects
                if bbox_size_cm[0] >= 3.0 and bbox_size_cm[1] >= 3.0 and bbox_size_cm[2] >= 2.0 :
                    # Classify Objects (using volume as before, but you could also use area if appropriate)
                    volume = np.prod(bbox_size_cm)  # Use cm for volume calculation

                    if volume < 9:
                        self.get_logger().info(
                            f'! Detected Cube or Sphere at {np.mean(cluster_points, axis=0)} § volume: {bbox_size_cm}')
                        classified_labels.append(1)
                    else:  # Larger object (Box)
                        self.get_logger().info(
                            f'! Detected Large Box at {np.mean(cluster_points, axis=0)} § volume: {bbox_size_cm}')
                        classified_labels.append(2)
                    detected_objects.append(cluster_points)
                else:
                    self.get_logger().info(
                        f"Neglecting small cluster {bbox_size_cm}")
            else:
                    self.get_logger().info("No labels found, no clusters to process.")

        # # OBJ Pubblication #######################################################################
        # # Send detected pointcloud
        # # msg_object_detected = pc2.create_cloud(msg.header, msg.fields, gen)
        # # self._pub.publish(msg_object_detected)
        # clusters = [(1,1,1), (1,1,1), (1,1,1)]
        # all_detected_points = np.concatenate(detected_objects, axis=0) if detected_objects else np.array([])
        # msg_detected_objects = pc2.create_cloud(pointcloud_header, msg.fields, all_detected_points)
        # self._pub_detected_objects.publish(msg_detected_objects)

        for i, cluster in enumerate(detected_objects):
            x_points = []
            y_points = []
            z_points = []
            for point in cluster:
                x,y,z = point
                x_points.append(x)
                y_points.append(y)
                z_points.append(z)
            
            x_obj = np.mean(x_points)
            y_obj = np.mean(y_points)
            z_obj = np.mean(z_points)
            
            self.place_object_frame((x_obj, y_obj, z_obj), "base_link", msg.header.stamp, f"Object_{i}")

        #     msg_object_detected = LabelPointStamped()
        #     msg_object_detected.label.data = "Ciao"
        #     ps = PoseStamped()
        #     ps.position.x = x_obj
        #     ps.position.y = y_obj
        #     ps.position.z = z_obj
        #     ps.header.stamp = msg.header.stamp
        #     ps.header.frame_id = "base_link"
        #     msg_object_detected.pose
        #     self._pub_objs.publish(msg_object_detected)
    
    def fast_transform2(self, points):
        """Static Transform from ´´ to ´base_link´"""
        translation = np.array([0.08987, 0.0175, 0.10456])
        rotation = np.array([-0.5, 0.5, -0.5, 0.5])

        def quaternion_matrix(quaternion):
            """Return 4x4 rotation matrix from quaternion."""
            _EPS = np.finfo(float).eps * 4.0

            q = np.array(quaternion, dtype=np.float64, copy=True)
            n = np.dot(q, q)
            if n < _EPS:
                return np.identity(4)
            q *= math.sqrt(2.0 / n)
            q = np.outer(q, q)
            return np.array([
                [1.0 - q[1, 1] - q[2, 2], q[0, 1] - q[2, 3], q[0, 2] + q[1, 3], 0.0],
                [q[0, 1] + q[2, 3], 1.0 - q[0, 0] - q[2, 2], q[1, 2] - q[0, 3], 0.0],
                [q[0, 2] - q[1, 3], q[1, 2] + q[0, 3], 1.0 - q[0, 0] - q[1, 1], 0.0],
                [0.0, 0.0, 0.0, 1.0]])

        R = quaternion_matrix(rotation)[:3, :3]

        transformed_points = np.dot(points, R.T) + translation
        return transformed_points
    
    def voxel_grid_filter(self, points, leaf_size=0.05):
        """Downsamples the point cloud using a voxel grid filter."""
        # Create voxel grid by downsampling points to grid size
        # Points are binned by floor division on leaf size.
        grid_indices = np.floor(points[:, :3] / leaf_size).astype(int)
        unique_grid_indices = np.unique(grid_indices, axis=0)
        # For each grid cell, compute the centroid
        downsampled_points = []
        for idx in unique_grid_indices:
            mask = np.all(grid_indices == idx, axis=1)
            points_in_cell = points[mask]
            centroid = np.mean(points_in_cell, axis=0)
            downsampled_points.append(centroid)
        return np.array(downsampled_points)

    def place_object_frame(self, point, frame_id, stamp, object_name):
        """Take points that correspond to an object and place a frame for it in RViz"""
        x, y, z = point

        # crete pose from these mean values
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # Now we want to express the pose in the map frame, hence we get the transform between the frames
        to_frame_rel = 'odom'
        from_frame_rel = frame_id
        time = rclpy.time.Time().from_msg(stamp)

        # Wait for the transform asynchronously
        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time
        )

        # Spin until transform found or `timeout_sec` seconds has passed
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        # lookup transform from frame id to map 
        try:
            t_map_cam = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, time)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # transform the marker pose so that it is expressed in map coordinates 
        pose = tf2_geometry_msgs.do_transform_pose(pose, t_map_cam)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = object_name
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()