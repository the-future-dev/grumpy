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


# import ctypes
import struct

import os
import ament_index_python.packages as packages

import torch
from perception_pkg.interfaces import ObjectEnum
from perception_pkg.classification_model import PointNetEncoderXYZRGB, PointNetClassifier

from enum import Enum

class ObjectEnum(Enum):
    NOISE = 0
    PUPPY = 1
    CUBE = 2
    SPHERE = 3
    BOX = 4

class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Data IN: Front Camera PointCloud
        self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)

        # Data OUT: 
        self._object_pose = self.create_publisher(ObjectDetection1D, '/perception/object_poses', 10)
        self._pub = self.create_publisher(PointCloud2, '/perception/camera/ground_filter', 10) # TODO: remove debug
        self.marker_publisher = self.create_publisher(Marker, '/perception/object_names_marker', 10) # TODO: remove debug

        # Initialize TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Ros2 initialization completed")

        
        try:
            package_share_directory = packages.get_package_share_directory('perception_pkg')
            model_path = os.path.join(package_share_directory, 'models', '04.pth')
            
            self.DEVICE = "cpu"
            self.classification_model = PointNetClassifier(PointNetEncoderXYZRGB(in_channels=6, out_channels=1024, use_layernorm=True), num_classes=len(ObjectEnum)).to(self.DEVICE)
            self.classification_model.load_state_dict(torch.load(model_path, map_location=self.DEVICE))
            self.get_logger().info(f"Classification model loaded in {self.DEVICE} completed")

        except Exception as e:
            self.get_logger().error(f"Error loading model: {e}")

        self.object_data_list = [] # TODO: remove debug

    def cloud_callback(self, msg: PointCloud2):
        gen = pc2.read_points_numpy(msg, skip_nans=True) # Convert ROS -> NumPy        
        points = np.asarray(gen[:, :3])
        colors_rgb = self.unpack_rgb(gen[:, 3])

        # GROUND REMOVAL #######################################################################
        points = self.fast_transform2(points) # points transformed to the robot's frame 

        # Analyze relevant points
        #  - 2D disance less than 2 meters
        #  - z-axis height between [0.010, 0.2]
        distance_relevance = np.sqrt(points[:, 0]**2 + points[:, 1]**2) < 1.5
        height_relevance = (points[:, 2] <= 0.2) & (points[:, 2] >= 0.010)
        
        relevant_mask = distance_relevance & height_relevance
        relevant_indices = np.where(relevant_mask)[0]
        gen = gen[relevant_indices, :]
        points = points[relevant_indices, :]
        colors_rgb = colors_rgb[relevant_indices, :]

        # Ground filtering 
        # - slope based method
        ground_inds, _ = self.slope_based_ground_filter(points[:, 0], points[:, 1], points[:, 2])
        non_ground_mask = ~np.isin(np.arange(len(points)), ground_inds, invert=False)
        gen = gen[non_ground_mask]
        points = points[non_ground_mask]
        colors_rgb = colors_rgb[non_ground_mask]

        if points.size == 0: #No relevant points detected after filtering.
            return
        
        # Point Clustering #######################################################################
        # eps = 0.02
        # min_samples = 40
        eps = 0.10
        min_samples = 80
        db = DBSCAN(eps=eps, min_samples=min_samples)
        labels = db.fit_predict(points)
        
        for label in set(labels):
            if label == -1:  # Ignore noise
                continue

            cluster_indices = np.where(labels == label)[0]
            cluster_points = points[cluster_indices]
            cluster_rgb = colors_rgb[cluster_indices]

            if cluster_points.size == 0:
                continue
            
            if np.max(cluster_points[:, 2]) >= 0.17:
                # self.get_logger().info(f"Cluster with label {label} is filtered out due to z > 19")
                continue
            
            # SAVE Locally ##########################################################################
            # cluster = np.concatenate((cluster_points, cluster_rgb), axis=1)
            # self.object_data_list.append(cluster)
            # self.save_object_data()

            # Object Classification ################################################################
            cluster = np.concatenate((cluster_points, cluster_rgb), axis=1)
            object_data = self.classify_object(cluster)
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

                packed_rgb = self.pack_rgb(cluster_rgb)
                cluster_pc2 = np.concatenate([cluster_points, packed_rgb], axis=1)

                msgs_pub = pc2.create_cloud(msg.header, msg.fields, cluster_pc2) # TODO: remove debug
                msgs_pub.header.frame_id = "base_link"
                self._pub.publish(msgs_pub) # TODO: remove debug

                self.place_object_frame((x_obj, y_obj, z_obj), "base_link", msg.header.stamp, f"{label}| {object_label}")
            else:
                self.get_logger().info(f"Label {label} Neglecting potential cluster")
    
    def unpack_rgb(self, packed_colors):
        """
        Args:
            packed_colors: shape (n, 1), packed colors as float32
        Returns:
            colors_rgb: shape (n, 3), unpacked RGB values in range [0, 1]
        """
        colors_rgb = np.empty((packed_colors.shape[0], 3), dtype=np.uint8)
        
        for idx, packed in enumerate(packed_colors):
            # Unpack the float back to a 32-bit integer
            s = struct.pack('>f', packed)
            packed_color = struct.unpack('>l', s)[0]
            
            # Extract r, g, b components from the packed integer
            r = (packed_color >> 16) & 255
            g = (packed_color >> 8) & 255
            b = packed_color & 255
            
            # Store the RGB values
            colors_rgb[idx, 0] = r
            colors_rgb[idx, 1] = g
            colors_rgb[idx, 2] = b

        # Convert to float and normalize to [0, 1]
        colors_rgb = colors_rgb.astype(np.float32) / 255.0
        
        return colors_rgb

    def pack_rgb(self, colors_rgb):
        """
        Args:
            colors_rgb: shape (n, 3)
        Returns:
            colors_rgb_packed: shape (n, 1)
        """
        colors_rgb = (colors_rgb * 255).astype(np.uint8)
        packed_colors = np.empty(colors_rgb.shape[0], dtype=np.uint32)
        for idx, color in enumerate(colors_rgb):
            # Reconstruct the packed integer from the RGB values
            r, g, b = color[0], color[1], color[2]
            # Shift and combine to get a single packed integer
            packed_color = (r << 16) | (g << 8) | b
            # Store the packed color
            packed_colors[idx] = packed_color
        colors_floats = []
        for packed in packed_colors:
            # Unpack the integer back into a float (this is the reverse of your initial process)
            s = struct.pack('>l', packed)
            c = struct.unpack('>f', s)[0]
            colors_floats.append(c)

        colors_floats = np.array(colors_floats).reshape(-1, 1)

        return colors_floats
    
    def fast_transform2(self, points):
        """
        Static Transform from ´camera_depth_optical_frame´ to ´base_link´
        """
        translation = np.array([0.08987, 0.0175, 0.10456])
        rotation = np.array([-0.5, 0.5, -0.5, 0.5])

        def quaternion_matrix(quaternion):
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
        K = 1

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

        delta_z = z[neighbor_indices] - z[:, np.newaxis]  # z-coordinates difference of neighbors
        delta_xy = np.sqrt((x[neighbor_indices] - x[:, np.newaxis])**2 + (y[neighbor_indices] - y[:, np.newaxis])**2) # distances in x-y plane
        delta_xy[delta_xy == 0] = 1e-6

        slopes = np.abs(delta_z / delta_xy)  # Shape: (num_points, k)
        max_slopes = np.max(slopes, axis=1)  # Shape: (num_points,)

        # Identify ground points based on the slope threshold
        ground_mask = max_slopes <= SLOPE_THRESHOLD
        ground_indices = np.where(ground_mask)[0]
        non_ground_indices = np.where(~ground_mask)[0]

        return ground_indices, non_ground_indices
    
    def classify_object(self, cluster):
        """Classifies a point cloud cluster into an object type."""
        #  TODO: VOLUME BASED Pre-filtering
        cluster_tensor = torch.tensor(cluster, dtype=torch.float32).unsqueeze(0).to(self.DEVICE)

        pred_values = self.classification_model(cluster_tensor)
        prediction = torch.argmax(pred_values, dim=1)
        pred_string = ObjectEnum(prediction.item()).name

        centroid = np.mean(cluster[:, :3], axis=0)

        return {
            'label': pred_string, 
            'centroid': centroid,
        } 


    def save_object_data(self):
        """
        Saves cluster data to a .npz file.
        """
        data_save_path = "data_BOX"
        arrays_to_save = {}
        for i, item in enumerate(self.object_data_list):
            arrays_to_save[f'cluster_{i}'] = item

        np.savez_compressed(data_save_path, **arrays_to_save)
        self.get_logger().info(f"Saved object data to {data_save_path}")

    def place_object_frame(self, point, frame_id, stamp, object_name):
        """Take points that correspond to an object and place a frame for it in RViz"""
        x, y, z = point

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        marker = Marker()
        marker.header.frame_id = frame_id
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
        marker.color.r = 1.0  # Text color
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Alpha (transparency)
        marker.text = object_name
        marker_lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        marker.lifetime = marker_lifetime

        self.marker_publisher.publish(marker)

    def node_closure(self):
        del self.classification_model
        self.get_logger().info("Classification model unloaded successfully.")
        
        super().destroy_node()

def main():
    rclpy.init()
    node = Detection()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.node_closure()
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()