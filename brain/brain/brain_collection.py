def main():
    print('Hi from brain.')


if __name__ == '__main__':
    main()


#!/usr/bin/env python

import math

import numpy as np
import py_trees
import py_trees_ros

import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.node import Node

import rclpy.time
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from tf2_ros import TransformException
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped, Pose
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from grumpy_interfaces.msg import ObjectDetection1D
import pandas as pd
import os

import tf2_geometry_msgs

from tf2_ros.buffer import Buffer

# Behaviors
class PathIsFree(py_trees.behaviour.Behaviour):
    """
    Behavior to check if path is free
    """
    def __init__(self, node:Node):
        super(PathIsFree, self).__init__('PathIsFree')
        self.node = node

    def update(self):
        """
        Method that will be exectued when behavior is ticked
        """
        if self.node.free_path:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class GotIcpRefScan(py_trees.behaviour.Behaviour):
    """
    Behavior to check if we got another icp reference scan
    """
    def __init__(self, node:Node):
        super(GotIcpRefScan, self).__init__('GotIcpRefScan')
        self.node = node

    def update(self):
        """
        Method that will be exectued when behavior is ticked
        """
        if self.node.icp_ref_scan_set:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class AtRefScanPoint(py_trees.behaviour.Behaviour):
    """
    Behavior to check if icp ref scan point
    """
    def __init__(self, node:Node):
        super(AtRefScanPoint, self).__init__('AtRefScanPoint')
        self.node = node

    def update(self):
        """
        Method that will be exectued when behavior is ticked
        """
        if self.node.at_ref_scan_point:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class NoObjectLeft(py_trees.behaviour.Behaviour):
    """
    Behavior to check if no objects left
    """
    def __init__(self, node:Node):
        super(NoObjectLeft, self).__init__('NoObjectLeft')
        self.node = node

    def update(self):
        """
        Method that will be exectued when behavior is ticked
        """
        if self.node.no_objects_left:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE



class BrainCollection(Node):

    def __init__(self):
        super().__init__('brain_collection')

        # initalize condition values
        self.free_path = True
        self.icp_ref_scan_set = False
        self.at_ref_scan_point = True
        self.no_objects_left = False

        # Create ROS 2 behavior tree
        self.tree = self.create_behaviour_tree()

        # Setup adn start ticking the tree (Managed by ROS)
        self.tree.setup(node=self,timeout=5) 
        self.tree.tick_tock(period_ms=1000) 


    def free_path_cb(self, msg:Bool):
        """
        callback that sets the condition if the path is free
        """
        self.free_path = msg.data

    def icp_got_ref_scan_cb(self, msg:Bool):
        """
        callback that sets the condition variable for having another icp reference scan
        """
        self.icp_ref_scan_set = msg.data

    def icp_at_ref_scan_point_cb(self, msg:Bool):
        """
        callback that sets the condition variable for when the robot is in the calculated ref scan point
        """
        self.at_ref_scan_point = msg.data

    def no_objects_left_cb(self, msg:Bool):
        """
        callback that returns the boolean from the topic that says if there are no objects left to collect
        """    
        self.no_objects_left = msg.data

    def create_behaviour_tree(self) -> py_trees_ros.trees.BehaviourTree:
        """
        method to create the ros implementation of a behaviour tree
        """
        # qos profile for conditions
        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,history=QoSHistoryPolicy.KEEP_LAST)

        # Here all topics needed for the behaviour tree are defined in the order 
        # (unique_name, topic_name, subscriber_type, latched, callback) that have the types (str, str, msgType, bool, func)
        subscribers = py_trees_ros.utilities.Subscribers(node=self,
            subscriber_details =    [    
                                    ('Clear_path_monitor', '/free_path', Bool, True, self.free_path_cb),
                                    ('ICP_new_ref_scan', '/icp_ref_scan', Bool, True, self.icp_got_ref_scan_cb),
                                    ('At_ref_scan_point', '/at_ref_scan_point', Bool, True, self.icp_at_ref_scan_point_cb),
                                    ('No_objects_left', '/no_objects_left', Bool, True, self.no_objects_left_cb)
                                    ])
        

        # Could also be done with blackboard but unsure how that would work
        # # Conditions using the blackboard (which is a place where topics and variables are shared for behviours)
        # Clear_path_monitor = py_trees_ros.subscribers.ToBlackboard(
        #     name="Clear_path_monitor",
        #     topic_name="/free_path",
        #     topic_type=Bool,
        #     qos_profile=qos_profile,
        #     blackboard_variables= {var_free_path:'data'}
        # )

        # # Condition behaviours
        # path_clear_condition = py_trees.behaviours.CheckBlackboardVariableValue(
        #     name="PathFree?",
        #     check=py_trees.common.ComparisonExpression(variable=var_free_path, value=True, operator='==')
        # )

        # # Services
        # # FromConstant will do the same request every time, otherwise use FromBlackBoard
        # go_to_free_space = py_trees_ros.service_clients.FromConstant(
        #     name="GoToFreeSpace",
        #     service_name='TODO',
        #     service_type= 'TODO',
        #     service_request= 'TODO',
        #     wait_for_server_timeout_sec=-3.0 # this means that it is checked every 3 seconds, but it block the execution
        # )

        # Building tree
        # memory=False means that conditions are ticked each iteration, otherwise the tree returns to the behaviour that returned running last
        # branch - obstacle avoidance
        obst_avoid = py_trees.composites.Selector(name="ObstacleAvoidance", memory=False)         
        obst_avoid.add_children([PathIsFree(self)]) # TODO add service to drive to free space if condition false

        # branch - get another icp ref scan
        drive_to_ref_scan_point = py_trees.composites.Selector(name="DriveToRefScanPoint", memory=False)
        drive_to_ref_scan_point.add_children([AtRefScanPoint(self)]) # TODO service to drive to ref scan point

        get_another_icp_ref_scan = py_trees.composites.Selector(name="GetAnotherIcpRefScan", memory=False)
        get_another_icp_ref_scan.add_children([GotIcpRefScan(self), drive_to_ref_scan_point])

        # branch - pick and place objects
        pick_and_place_next = py_trees.composites.Sequence(name='PickPlaceNextObject', memory=False)
        # pick_and_place_next.add_children([]) # TODO add children to this branch        
        
        pick_and_place = py_trees.composites.Selector(name="PickPlaceUntilNoObject", memory=False)
        pick_and_place.add_children([NoObjectLeft(self), pick_and_place_next])

        root = py_trees.composites.Sequence(name='Collection', memory=False)
        root.add_children([obst_avoid, get_another_icp_ref_scan, pick_and_place])

        # return ros wrapping of py_trees behaviour tree
        return py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)




def main():
    rclpy.init()
    node = BrainCollection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()


