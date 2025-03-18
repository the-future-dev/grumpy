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
        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,history=QoSHistoryPolicy.KEEP_LAST)
        
        self.node = node

        self.node.create_subscription(Bool, '/free_path', self.free_path_cb, qos_profile)

        # initalize path as being true
        self.path_free = True

    def free_path_cb(self, msg:Bool):
        """
        callback to set status of behavior
        """
        self.path_free = msg.data

    def update(self):
        """
        Method that will be exectued when behavior is ticked
        """
        if self.path_free:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE





class BrainCollection(Node):

    def __init__(self):
        super().__init__('brain_collection')

         # Create ROS 2 behavior tree
        self.tree = self.create_behaviour_tree()

        # Start ticking the tree (Managed by ROS)
        self.tree.setup(timeout=5)  # Ensure it initializes properly
        self.tree.tick_tock(period_ms=1000)  # Tick at 1Hz


    def create_behaviour_tree(self) -> py_trees_ros.trees.BehaviourTree:
        # variable names for topics used in the behaviour tree
        var_free_path = "free_path"
        

        # qos profile for conditions
        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,history=QoSHistoryPolicy.KEEP_LAST)

        # Conditions using the blackboard (which is a place where topics and variables are shared for behviours)
        Clear_path_monitor = py_trees_ros.subscribers.ToBlackboard(
            name="Clear_path_monitor",
            topic_name="/free_path",
            topic_type=Bool,
            qos_profile=qos_profile,
            blackboard_variables= {var_free_path:'data'}
        )

    


        # Condition behaviours
        path_clear_condition = py_trees.behaviours.CheckBlackboardVariableValue(
            name="PathFree?",
            check=py_trees.common.ComparisonExpression(variable=var_free_path, value=True, operator='==')
        )


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
        root = py_trees.composites.Selector(name="ObstacleAvoidance", memory=False)         
        root.add_children([path_clear_condition])

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


