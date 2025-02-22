#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, PointField
from rclpy.node import Node

from grumpy_interfaces.msg import ObjectDetection1D 

from visualization_msgs.msg import Marker

from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose

import math
from sklearn.cluster import DBSCAN
from scipy.spatial import cKDTree
# from sklearn.decomposition import PCA


import ctypes
import struct

class Detection(Node):

    def __init__(self):
        super().__init__('detection')
        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/perception/camera/ground_filter', 10)

        self.marker_publisher = self.create_publisher(Marker, '/perception/object_names_marker', 10)

        self._object_pose = self.create_publisher(ObjectDetection1D, '/perception/object_poses', 10)

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)

        # Initialize TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # self.static_transform = self.get_static_transform()
        self.get_logger().info(f"COMPLETED: Node initialization")

        self.object_data_list = []

    def cloud_callback(self, msg: PointCloud2):
        # Convert ROS -> NumPy
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = np.asarray(gen[:, :3])
        colors_rgb = np.empty(points.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            colors_rgb[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            colors_rgb[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            colors_rgb[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        colors_rgb = colors_rgb.astype(np.float32) / 255

        # GROUND REMOVAL #######################################################################
        # Transform from camera_depth_optical_frame to base_link
        points = self.fast_transform2(points)

        # Analyze relevant points
        #  - 2D disance less than 1.5 meters
        #  - z-axis height less than 20 cm TODO: analyze in "production" (1) obstacles? (2) human noise
        distance_relevance = np.sqrt(points[:, 0]**2 + points[:, 1]**2) < 1.5
        height_relevance = (points[:, 2] <= 0.2) & (points[:, 2] >= 0.010)
        
        relevant_mask = distance_relevance & height_relevance
        relevant_indices = np.where(relevant_mask)[0]
        gen = gen[relevant_indices, :]
        points = points[relevant_indices, :]
        colors_rgb = colors_rgb[relevant_indices, :]

        # Ground filtering 
        # - slope based method
        # - static cut of points with z between [0.010, 0.2] (for efficiency)
        ground_inds, non_ground_inds = self.slope_based_ground_filter(points[:, 0], points[:, 1], points[:, 2])
        non_ground_mask = ~np.isin(np.arange(len(points)), ground_inds, invert=False)
        gen = gen[non_ground_mask]
        points = points[non_ground_mask]
        colors_rgb = colors_rgb[non_ground_mask]

        msgs_pub = pc2.create_cloud(msg.header, msg.fields, gen)
        self._pub.publish(msgs_pub) # TODO: remove debug

        if points.size == 0:
            self.get_logger().debug("No relevant points detected after filtering.")
            return
        
        # Point Clustering #######################################################################
        db = DBSCAN(eps=0.05, min_samples=100)
        labels = db.fit_predict(points)
        
        for label in set(labels):
            if label == -1:  # Ignore noise
                continue

            cluster_indices = np.where(labels == label)[0]
            cluster_points = points[cluster_indices]
            cluster_rgb = colors_rgb[cluster_indices]

            if cluster_points.size == 0:
                continue
            
            # SAVE #################################################################################
            cluster = np.concatenate((cluster_points, cluster_rgb), axis=1)
            self.object_data_list.append(cluster)
            # self.save_object_data()

            # Object Detection and Classification ##################################################
            object_data = self.classify_object(cluster_points, cluster_rgb)
            if object_data:
                x_obj, y_obj, z_obj = object_data['centroid']
                object_label = object_data['label']

                s = ObjectDetection1D()
                s.header.stamp = msg.header.stamp
                s.header.frame_id = "base_link"
                s.label.data = 'B' if object_label == 'Box' else 'O'
                s.pose.position.x = x_obj
                s.pose.position.y = y_obj
                s.pose.position.z = z_obj

                self._object_pose.publish(s)
                self.place_object_frame((x_obj, y_obj, z_obj), "base_link", msg.header.stamp, f"{label}| {object_label}")  # Use label and object_label
            else:
                self.get_logger().info(f"Label {label} Neglecting potential cluster")
    
    def save_object_data(self):
        """Saves all collected object data to a .npz file."""
        data_save_path = "data_SPHERE"
        # Prepare a dictionary to save, where keys are like 'cluster_0', 'cluster_1', etc.
        arrays_to_save = {}
        for i, item in enumerate(self.object_data_list):
            arrays_to_save[f'cluster_{i}'] = item

        np.savez_compressed(data_save_path, **arrays_to_save)

        self.get_logger().info(f"Saved object data to {data_save_path}")


    def fast_transform2(self, points):
        """Static Transform from ´camera_depth_optical_frame´ to ´base_link´"""
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

    def slope_based_ground_filter(self, x, y, z):
        """
        Filters ground points from LiDAR data using a slope-based method.

        Args:
            x, y, z: three 1D numpy array of x-coordinates.

        Returns:
            ground_indices: Indices of points classified as ground.
            non_ground_indices: Indices of points classified as non-ground.
        """
        SLOPE_THRESHOLD = 100
        K = 3

        num_points = len(x)
        ground_indices = []
        non_ground_indices = []

        if len(x) <= K:
            if num_points == 0:
                return np.array([]), np.array([])
            return np.array([]), np.arange(num_points)

        tree = cKDTree(np.column_stack((x, y)))

        # Query for the k-nearest neighbors
        distances, neighbor_indices = tree.query(np.column_stack((x, y)), k=K + 1)
        neighbor_indices = neighbor_indices[:, 1:]  # Exclude self

        # Get difference in z-coordinates of the neighbors
        delta_z = z[neighbor_indices] - z[:, np.newaxis]  # Shape: (num_points, k)
        # Calculate the distances in the x-y plane (vectorized)
        delta_xy = np.sqrt((x[neighbor_indices] - x[:, np.newaxis])**2 + (y[neighbor_indices] - y[:, np.newaxis])**2)
        
        # Handle cases where delta_xy is zero to avoid division by zero.
        delta_xy[delta_xy == 0] = 1e-6 # Or another very small value

        # Calculate slopes (vectorized)
        slopes = np.abs(delta_z / delta_xy)  # Shape: (num_points, k)

        # Find the maximum slope for each point
        max_slopes = np.max(slopes, axis=1)  # Shape: (num_points,)

        # Identify ground points based on the slope threshold
        ground_mask = max_slopes <= SLOPE_THRESHOLD
        ground_indices = np.where(ground_mask)[0]
        non_ground_indices = np.where(~ground_mask)[0]

        return ground_indices, non_ground_indices


    def classify_object(self, cluster_points, cluster_rgb):
        """Classifies a point cloud cluster into an object type."""

        bbox_min = np.min(cluster_points, axis=0)
        bbox_max = np.max(cluster_points, axis=0)
        bbox_size = bbox_max - bbox_min
        bbox_size_cm = bbox_size * 100
        volume = np.prod(bbox_size_cm)

        centroid = np.mean(cluster_points, axis=0)

        return {'label': f"OBJ", 'centroid': centroid, 'bbox_min': bbox_min, 'bbox_max': bbox_max, 'volume': volume} 

        # if (bbox_size_cm[0] + bbox_size_cm[1]) >= 5.0 and bbox_size_cm[2] >= 1.5:  # Size threshold (tune this)
        #     if volume < 1000:
        #         object_label = "Object"
        #     elif volume < 3720:
        #         object_label = "Box"
        #     else:  # Handle cases where the object is larger than the box threshold
        #         object_label = "Large Object"
        #         self.get_logger().info(f"Large Object detected: {bbox_size_cm} | {volume}")
            
        #     self.get_logger().info(f"detected: {object_label} | Box size: {bbox_size_cm} | Volume: {volume} | Cluster point mean: {centroid}")

        #     return {'label': object_label, 'centroid': centroid, 'bbox_min': bbox_min, 'bbox_max': bbox_max}  # Return object data

        # else:
        #     self.get_logger().info(f"Neglecting small cluster {bbox_size_cm}")
        #     return None

    def place_object_frame(self, point, frame_id, stamp, object_name):
        """Take points that correspond to an object and place a frame for it in RViz"""
        x, y, z = point

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        marker = Marker()
        marker.header.frame_id = 'base_link' # Same frame as your transform
        marker.header.stamp = stamp
        marker.ns = "object_names"  # Namespace for your markers (important!)
        marker.id = id(object_name) # Unique ID for each object (use a counter or hash)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x  # Same position as your transform
        marker.pose.position.y = y  # Same position as your transform
        marker.pose.position.z = z  # Same position as your transform
        marker.pose.orientation.x = 0.0 # Orientation for text
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Adjust text size
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0  # Text color (red in this example)
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Alpha (transparency)
        marker.text = object_name
        marker_lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        marker.lifetime = marker_lifetime

        self.marker_publisher.publish(marker)

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