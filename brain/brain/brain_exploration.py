#!/usr/bin/env python
import py_trees
import py_trees_ros

import rclpy
from rclpy.node import Node

import rclpy.time
from time import sleep

from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


# Behaviours
class LocalObstacleAvoidance(py_trees.behaviour.Behaviour):
    """
    Behaviour that handles local obstacle avoidance
    """
    def __init__(self, node:Node):
        super(LocalObstacleAvoidance, self).__init__('LocalObstacleAvoidance')
        self.node = node
        self.drive_free = True

    def update(self):
        """
        Method that will be exectued when behaviour is ticked
        """
        if self.node.free_path:
            self.stop_robot = True
            self.drive_free = True
            
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info('In occupied space')
            

            # only publish so that action is performed once and to do drive free stop robot has to have been performed
            if self.stop_robot:
                self.node.get_logger().info('Stopping robot')
                self.stop_robot = False
                self.node.set_poses_list = True
                msg = Bool()
                msg.data = True
                self.node.stop_robot_pub.publish(msg)
                self.node.have_path = False
            else:
                if self.drive_free:
                    self.node.get_logger().info('Driving to free space')
                    self.drive_free = False
                    msg = Bool()
                    msg.data = True
                    self.node.drive_to_free_pub.publish(msg)
                else:
                    self.node.get_logger().info('Trying to reach free space')

            return py_trees.common.Status.RUNNING


class NoUnknownSpaceLeft(py_trees.behaviour.Behaviour):
    """
    Bahaviour that checks if exploration is done by checking if there is no unknown space left
    """
    def __init__(self, node:Node):
        super(NoUnknownSpaceLeft, self).__init__('NoUnknownSpaceLeft')
        self.node = node
    
    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        if self.node.no_unknown_left:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class GetIcpScan(py_trees.behaviour.Behaviour):
    """
    Behaviour that gets ICP scan if we do not have a scan for the current corner position, does nothing when exploring unknown space
    """
    def __init__(self, node:Node):
        super(GetIcpScan, self).__init__('GetIcpScan')
        self.node = node
        self.node.have_scan = True
        self.take_ref_scan = True

    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        if self.node.have_scan:
            self.take_ref_scan = True
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info('Do not have reference scan')
            # take reference scan for ICP, but only publish once
            if self.take_ref_scan:
                self.take_ref_scan = False
                self.node.get_logger().info('Publishing Bool to ICP node so that it takes ref scan')
                msg = Bool()
                msg.data = True
                self.node.take_ref_scan_pub.publish(msg)
            else:
                self.node.get_logger().info('Waiting for ICP to take ref scan')

            return py_trees.common.Status.RUNNING

class TurnRobot(py_trees.behaviour.Behaviour):
    """
    Behaviour that turns the robot
    """
    def __init__(self, node:Node):
        super(TurnRobot, self).__init__('TurnRobot')
        self.node = node
    
    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        if self.node.have_turned:
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info('Turning Robot') # TODO implement logic to turn robot 
            self.node.have_turned = True
            return py_trees.common.Status.RUNNING


class GetPath(py_trees.behaviour.Behaviour):
    """
    Bahaviour that finds path using A* algorithm, if there is no goal it begins with getting a goal from the planner 
    """
    def __init__(self, node:Node):
        super(GetPath, self).__init__('GetPath')
        self.node = node
        self.node.get_goal = True
        self.node.get_path = True

    def update(self):
        """
        Method that will be exectued when behaviour is ticked
        """
        if self.node.have_path:
            # if we have a path we want to get path next time we go into this part of the behaviour
            self.node.get_path = True
            self.node.get_logger().info('We have a path to pass on to DriveToGoal')
            return py_trees.common.Status.SUCCESS

        else:
            self.node.get_logger().info('We do not have a path')
            if self.node.have_goal:
                # if we have a goal we want to get goal next time we go into this part of the behaviour
                self.node.get_goal = True
                
                # if we have a goal send it to A* but only do it once
                if self.node.get_path:
                    self.node.get_path = False
                    self.node.get_logger().info('Publishing goal to A*')
                    self.node.send_goal_pub.publish(self.node.goal)
                else:
                    self.node.get_logger().info('Waiting for path from A*')    
            else:
                # only publish to get goal once
                if self.node.get_goal:
                    self.node.get_goal = False

                    self.node.get_logger().info('Publishing Bool to planner to get goal')
                    msg = Bool()
                    msg.data = True
                    self.node.find_goal_pub.publish(msg)

                else:
                    self.node.get_logger().info('Waiting for goal from planner')


            return py_trees.common.Status.RUNNING


class DriveToGoal(py_trees.behaviour.Behaviour):
    """
    Behaviour that sends path message (one pose at the time) to drive control
    """
    def __init__(self, node:Node):
        super(DriveToGoal, self).__init__('DriveToGoal')
        self.node = node
        self.node.set_poses_list = True
        self.empty_list = False
    
    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        if self.empty_list:
            self.empty_list = False
            self.node.set_poses_list = True
            return py_trees.common.Status.SUCCESS
        else:
            if self.node.set_poses_list:
                self.node.set_poses_list = False
                self.poses_list = self.node.path.poses
                self.node.get_logger().info(f'The goal pose from A*; x:{self.node.path.poses[-1].pose.position.x}, y: {self.node.path.poses[-1].pose.position.y}')
            else:
                if not self.node.at_goal:
                    self.node.get_logger().info('Waiting for drive control to reach point')
                    return py_trees.common.Status.RUNNING

            self.node.get_logger().info(f'Poses List length: {len(self.poses_list)}')
            pose = self.poses_list.pop(0)

            if len(self.poses_list)==0:
                self.empty_list = True
                self.node.get_logger().info(f'No poses left, should have reached goal')
                return py_trees.common.Status.RUNNING

            self.node.path.poses = [pose]
            self.node.get_logger().info('Publishing next pose to drive_control')
            self.node.drive_to_goal_pub.publish(self.node.path)

            return py_trees.common.Status.RUNNING


class SetSelf(py_trees.behaviour.Behaviour):
    """
    Bahaviour that sets self variables used in the behaviours
    """
    def __init__(self, node:Node):
        super(SetSelf, self).__init__('SetSelf')
        self.node = node

    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        self.node.have_scan = self.node.corner_ex_done
        self.node.have_path = False
        self.node.have_goal = False
        self.node.have_turned = False
        self.node.at_goal = False

        return py_trees.common.Status.RUNNING


class BrainExploration(Node):
    def __init__(self):
        super().__init__('brain_exploration')

        # initalize self values
        self.have_path = False
        self.have_scan = False
        self.have_goal = False
        self.have_turned = False

        self.at_goal = False
        self.corner_ex_done = False
        self.no_unknown_left = False

        self.free_path = True

        self.goal = None
        self.path = None
        
        # intialize publishers
        self.send_goal_pub = self.create_publisher(PoseStamped, 'Astar/next_goal', 1)
        self.find_goal_pub = self.create_publisher(Bool, 'planner_ex/find_goal', 1)
        self.stop_robot_pub = self.create_publisher(Bool, 'drive/stop', 1)
        self.drive_to_free_pub = self.create_publisher(Bool, 'avoidance/drive_to_free', 1)
        self.drive_to_goal_pub = self.create_publisher(Path, 'drive/path', 1)
        self.take_ref_scan_pub = self.create_publisher(Bool, '/icp_node/get_new_reference_scan', 1)

        # Create ROS 2 behaviour tree
        self.tree = self.create_behaviour_tree()

        # Setup of behaviour tree
        self.tree.setup(node=self,timeout=5) 

        # Sleeping for 5 seconds so that all nodes are initalized properly
        self.get_logger().info('Sleeping for 5 seconds before starting to tick')
        sleep(5)

        # Starting to tick the tree
        self.get_logger().info('Starting to tick the tree')
        self.tree.tick_tock(period_ms=500) 

    def free_path_cb(self, msg:Bool):
        """
        callback that sets the condition if the path is free
        """
        self.get_logger().info(f'Avoidance node is telling us that path_free = {msg.data}')
        self.free_path = msg.data
    
    def set_path_cb(self, msg:Path):
        """
        Callback that sets the path that will be sent to drive control
        """
        if len(msg.poses) == 0:
            self.get_logger().info('Path from A* is empty')
            self.have_goal = False
            self.get_path = True
        else:
            self.get_logger().info('Have received a non-empty path from A*')
            self.path = msg
            self.have_path = True 

    def set_goal_cb(self, msg:PoseStamped):
        """
        Callback that sets the goal that will be sent to A*
        """
        self.get_logger().info('Have received goal from planner')
        self.goal = msg
        self.have_goal = True
    
    def corner_ex_done_cb(self, msg:Bool):
        """
        Callback that sets boolean, True if corner exploration is done, False otherwise
        """
        self.get_logger().info(f'Planner telling us that corner exploration done = {msg.data}')
        self.corner_ex_done = msg.data
    
    def no_unknown_left_cb(self, msg:Bool):
        """
        Callback that sets boolean, True if all exploration is done, False otherwise
        """
        self.get_logger().info(f'Planner telling us that no unknown space left = {msg.data}')
        self.no_unknown_left = msg.data        

    def at_goal_cb(self, msg: Bool):
        """
        Callback that sets boolean, True if drive control is at goal, False otherwise
        """
        self.get_logger().info(f'Drive control telling us that reached_goal = {msg.data}')
        self.at_goal = msg.data

    def have_scan_cb(self, msg: Bool):
        """
        Callback that sets boolean, True if ICP have taken a scan, False otherwise
        """
        self.get_logger().info(f'ICP node telling us that have_scan = {msg.data}')
        self.have_scan = msg.data

    def create_behaviour_tree(self) -> py_trees_ros.trees.BehaviourTree:
        """
        method to create the ros implementation of a behaviour tree
        """

        # Here all topics needed for the behaviour tree are defined in the order 
        # (unique_name, topic_name, subscriber_type, latched, callback) that have the types (str, str, msgType, bool, func)
        py_trees_ros.utilities.Subscribers(
            node=self,
            subscriber_details =    [    
                                    ('Path', '/Astar/path', Path, False, self.set_path_cb),
                                    ('Goal', '/planner_ex/next_goal', PoseStamped, False, self.set_goal_cb),
                                    ('CornerExplorationDone', '/planner_ex/corner_ex_done', Bool, False, self.corner_ex_done_cb),
                                    ('NoUnkownSpaceLeft', '/planner_ex/no_unknown_left', Bool, False, self.no_unknown_left_cb),
                                    ('FreePath', '/avoidance/free_path', Bool, False, self.free_path_cb),
                                    ('AtGoal', '/drive/feedback', Bool, False, self.at_goal_cb),
                                    ('HaveScan', '/icp_node/have_scan', Bool, False, self.have_scan_cb)
                                    ])

        # Building tree
        # memory=False means that conditions are ticked each iteration, otherwise the tree returns to the behaviour that returned running last

        exploration = py_trees.composites.Sequence(name='Exploration', memory=False)
        exploration.add_children([GetIcpScan(self), TurnRobot(self), GetPath(self), DriveToGoal(self), SetSelf(self)])

        exploration_until_no_unknown_space =  py_trees.composites.Selector(name="ExploreUntilNoUnknownSpace", memory=False)
        exploration_until_no_unknown_space.add_children([NoUnknownSpaceLeft(self), exploration])

        # Main sequence
        root = py_trees.composites.Sequence(name='ExplorationRoot', memory=False)
        root.add_children([LocalObstacleAvoidance(self), exploration_until_no_unknown_space])

        # return ros-wrapping of py_trees behaviour tree
        return py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)


def main():
    rclpy.init()
    node = BrainExploration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
