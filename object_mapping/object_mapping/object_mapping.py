#!/usr/bin/env python

import os
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray
from grumpy_interfaces.msg import ObjectDetection1D, ObjectDetection1DArray

import tf2_geometry_msgs

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from occupancy_grid_map.workspace_utils import Workspace
from shapely.geometry.polygon import Polygon
from shapely.geometry import Point

from enum import Enum
class ObjectEnum(Enum):
    NOISE = 0
    PUPPY = 1
    CUBE = 2
    SPHERE = 3
    BOX = 4

class ObjectVoting:
    def __init__(self, label: ObjectEnum, x: float, y: float, angle: float):
        self.arr_label = np.array([label.value])
        self.arr_x = np.array([x])
        self.arr_y = np.array([y])
        self.arr_angle = np.array([angle])
        # Cache for frequently accessed values
        self._cached_label = None
        self._cached_x = None
        self._cached_y = None
        self._cached_angle = None
        self._needs_update = True

    def add_scan(self, label: ObjectEnum, x: float, y: float, angle: float):
        self.arr_label = np.append(self.arr_label, label.value)
        self.arr_x = np.append(self.arr_x, x)
        self.arr_y = np.append(self.arr_y, y)
        self.arr_angle = np.append(self.arr_angle, angle)
        self._needs_update = True
        # Reset caches
        self._cached_label = None
        self._cached_x = None
        self._cached_y = None
        self._cached_angle = None

    def get_label(self):
        """Get the label most frequently occuring in the array"""
        if self._needs_update or self._cached_label is None:
            self._cached_label = ObjectEnum(np.bincount(self.arr_label).argmax())
            self._needs_update = False
        return self._cached_label

    def get_x(self):
        """Get the mean x value of the object, filtering by label"""
        if self._needs_update or self._cached_x is None:
            label = self.get_label()
            mask = self.arr_label == label.value
            filtered_x = self.arr_x[mask]
            self._cached_x = float(np.mean(filtered_x)) if len(filtered_x) > 0 else 0.0
            self._needs_update = False
        return self._cached_x

    def get_y(self):
        """Get the mean y value of the object, filtering by label"""
        if self._needs_update or self._cached_y is None:
            label = self.get_label()
            mask = self.arr_label == label.value
            filtered_y = self.arr_y[mask]
            self._cached_y = float(np.mean(filtered_y)) if len(filtered_y) > 0 else 0.0
            self._needs_update = False
        return self._cached_y

    def get_angle(self):
        """Get the mean angle value of the object, filtering by label"""
        if self._needs_update or self._cached_angle is None:
            label = self.get_label()
            mask = self.arr_label == label.value
            filtered_angle = self.arr_angle[mask]
            self._cached_angle = float(np.mean(filtered_angle)) if len(filtered_angle) > 0 else 0.0
            self._needs_update = False
        return self._cached_angle

    def merge_scans(self, other_object: 'ObjectVoting'):
        """Merge scans from another ObjectVoting instance into this one."""
        self.arr_label = np.append(self.arr_label, other_object.arr_label)
        self.arr_x = np.append(self.arr_x, other_object.arr_x)
        self.arr_y = np.append(self.arr_y, other_object.arr_y)
        self.arr_angle = np.append(self.arr_angle, other_object.arr_angle)
        # Mark caches as invalid as data has changed
        self._needs_update = True
        self._cached_label = None
        self._cached_x = None
        self._cached_y = None
        self._cached_angle = None

class ObjectMapping(Node):

    def __init__(self):
        super().__init__('object_mapping')

        # Intialize object list
        self.input_object_list = []
        self.output_object_list = []
        self.ws_utils = Workspace()
        self.workspace = self.ws_utils.workspace
        self.polygon = Polygon(self.workspace.T)

        # Set margin for adding new object
        self.margin_object = 0.15
        self.margin_box = 0.20

        # Intialize map file
        self.filepath = os.path.join(os.path.expanduser('~'), 'dd2419_ws', 'outputs', 'object_map.txt')
        with open(self.filepath, 'w') as file:
            pass # creates empty file

        # Initialize the transform listener and assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize the subscriber to the object poses
        self.create_subscription(ObjectDetection1D, '/perception/object_poses', self.object_pose_callback, 10)

        # Initialize publisher to publish non-duplicate object poses
        self.object_pose_pub = self.create_publisher(ObjectDetection1DArray, '/object_mapping/object_poses', 1)
        self.marker_publisher = self.create_publisher(MarkerArray, '/object_mapping/object_names_marker', 10)

    def publish_object_list(self):
        """Publish all detected objects"""
        marker_array = MarkerArray()
        object_detection_array = ObjectDetection1DArray()
        idx = 0

        # Write to file
        with open(self.filepath, 'w') as file:
            for obj in self.output_object_list:
                # Write object information to file
                file_label_str = 'B' if obj.get_label() == ObjectEnum.BOX else f'{obj.get_label().value}'
                file_x = float(round(obj.get_x(), 2))
                file_y = float(round(obj.get_y(), 2))
                file.write(f"{file_label_str} {file_x} {file_y}\n")

                detection1D = ObjectDetection1D()
                detection1D.pose.position.x = obj.get_x()
                detection1D.pose.position.y = obj.get_y()
                detection1D.label.data = obj.get_label().name
                detection1D.pose.position.z = 0.0  # Add z coordinate
                detection1D.pose.orientation.w = 1.0  # Add orientation

                object_detection_array.detected_objects.append(detection1D)

                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.text = obj.get_label().name
                marker.id = idx
                marker.ns = "object_mapping"
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x = detection1D.pose.position.x
                marker.pose.position.y = detection1D.pose.position.y
                marker.pose.position.z = detection1D.pose.position.z
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                marker_lifetime = rclpy.duration.Duration(seconds=3.0).to_msg()
                marker.lifetime = marker_lifetime

                marker_array.markers.append(marker)
                idx += 1

        self.marker_publisher.publish(marker_array)
        self.object_pose_pub.publish(object_detection_array)

    def object_pose_callback(self, msg: ObjectDetection1D):
        """
        Writes the object poses and removes duplicates using vectorized operations
        """
        pose = msg.pose

        # Transform to map frame
        to_frame_rel = 'map'
        from_frame_rel = msg.header.frame_id
        time = rclpy.time.Time().from_msg(msg.header.stamp)

        # Wait for the transform asynchronously
        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time
        )

        # Spin until transform found or `timeout_sec` seconds has passed
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, time)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Transform the marker pose to map coordinates
        pose = tf2_geometry_msgs.do_transform_pose(pose, t)

        label = ObjectEnum[msg.label.data]
        x = pose.position.x
        y = pose.position.y
        angle = 0 # Assuming angle is not derived from pose orientation for now

        # Add the raw detection before checking workspace or merging
        # self.input_object_list.append((label, x, y, angle)) # Optional: Keep track if needed

        if not self.is_within_workspace(x, y):
            self.get_logger().debug(f"Object at ({x:.2f}, {y:.2f}) is outside workspace.")
            return

        # --- Object Association/Update ---
        found_match = False
        if self.output_object_list:
            # Create arrays of all object positions
            positions = np.array([(obj.get_x(), obj.get_y()) for obj in self.output_object_list])

            distances = np.sqrt(np.sum((positions - np.array([x, y]))**2, axis=1))

            # Determine margin based on incoming label
            margin = self.margin_box if label == ObjectEnum.BOX else self.margin_object

            # Find objects within the margin
            matches = (distances < margin)

            if np.any(matches):
                # Add scan to the closest matching object
                # If multiple are within margin, this picks the one with the smallest distance
                match_idx = np.argmin(distances[matches]) # Index within the 'matches' array
                original_indices = np.where(matches)[0] # Original indices in output_object_list
                closest_obj_idx = original_indices[match_idx]

                self.get_logger().debug(f"Adding scan to existing object {closest_obj_idx} (Label: {label.name}, Pos: ({x:.2f}, {y:.2f}))")
                self.output_object_list[closest_obj_idx].add_scan(label, x, y, angle)
                found_match = True
                # Note: Previous logic added scan to *all* matches. Changed to add to closest only.

        if not found_match:
            # No suitable match found, or list is empty. Create new object.
            self.get_logger().debug(f"Creating new object (Label: {label.name}, Pos: ({x:.2f}, {y:.2f}))")
            self.output_object_list.append(ObjectVoting(label, x, y, angle))

        # --- Merge close objects ---
        # This loop continues as long as merges are happening in a pass,
        # as one merge might bring another pair of objects close enough.
        merged_in_pass = True
        while merged_in_pass:
            merged_in_pass = False
            i = 0
            while i < len(self.output_object_list):
                j = i + 1
                while j < len(self.output_object_list):
                    obj_i = self.output_object_list[i]
                    obj_j = self.output_object_list[j]

                    # Calculate distance between mean positions
                    dist = np.sqrt((obj_i.get_x() - obj_j.get_x())**2 + (obj_i.get_y() - obj_j.get_y())**2)

                    # Define merge margin (using smaller margin for simplicity)
                    # Consider if BOX margin should be used if either object is a BOX
                    merge_margin = self.margin_object

                    if dist < merge_margin:
                        self.get_logger().info(f"Merging object {j} into {i} (Labels: {obj_j.get_label().name} -> {obj_i.get_label().name}, Dist: {dist:.2f} < {merge_margin:.2f})")
                        # Merge obj_j into obj_i
                        obj_i.merge_scans(obj_j)
                        # Remove obj_j
                        self.output_object_list.pop(j)
                        merged_in_pass = True
                        # Restart inner check for object 'i' as list changed
                        j = i + 1 # Reset j to re-check against subsequent elements
                        continue # Continue inner loop (j) without incrementing j
                    else:
                        j += 1 # Move to next object j
                # If a merge happened involving object i, restart checks for i
                if merged_in_pass and j == i + 1: # Check if break happened in inner loop
                     # Don't increment i, restart outer loop's inner checks
                     pass # The outer while loop condition handles restart
                else:
                    i += 1 # Move to next object i

        # --- Publish the potentially updated and merged list ---
        self.publish_object_list()

    def is_within_workspace(self, x: float, y: float) -> bool:
        """Check if a point is within the workspace boundaries"""

        point = Point(x*100, y*100)

        if self.polygon.contains(point):
            return True
        else:
            return False

def main():
    rclpy.init()
    node = ObjectMapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()