#!/usr/bin/env python
import py_trees
import py_trees_ros

import rclpy
from rclpy.node import Node

import rclpy.time

from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


# Behaviours
class PathIsFree(py_trees.behaviour.Behaviour):
    """
    behaviour to check if path is free
    """
    def __init__(self, node:Node):
        super(PathIsFree, self).__init__('PathIsFree')
        self.node = node

    def update(self):
        """
        Method that will be exectued when behaviour is ticked
        """
        if self.node.free_path:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class GotIcpRefScan(py_trees.behaviour.Behaviour):
    """
    behaviour to check if we got another icp reference scan
    """
    def __init__(self, node:Node):
        super(GotIcpRefScan, self).__init__('GotIcpRefScan')
        self.node = node

    def update(self):
        """
        Method that will be exectued when behaviour is ticked
        """
        if self.node.icp_ref_scan_set:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class AtRefScanPoint(py_trees.behaviour.Behaviour):
    """
    behaviour to check if icp ref scan point
    """
    def __init__(self, node:Node):
        super(AtRefScanPoint, self).__init__('AtRefScanPoint')
        self.node = node

    def update(self):
        """
        Method that will be exectued when behaviour is ticked
        """
        if self.node.at_ref_scan_point:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class NoObjectLeft(py_trees.behaviour.Behaviour):
    """
    behaviour to check if no objects left
    """
    def __init__(self, node:Node):
        super(NoObjectLeft, self).__init__('NoObjectLeft')
        self.node = node

    def update(self):
        """
        Method that will be exectued when behaviour is ticked
        """
        if self.node.no_objects_left:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class FindGoal(py_trees.behaviour.Behaviour):
    """
    Bahaviour that finds goal using exploration planner
    """
    def __init__(self, node:Node):
        super(FindGoal, self).__init__('FindGoal')
        self.node = node

    def update(self):
        """
        Method that will be exectued when behaviour is ticked
        """
        msg = Bool()
        msg.data = True
        self.node.find_goal_pub.publish(msg)
        return py_trees.common.Status.RUNNING


class FindPath(py_trees.behaviour.Behaviour):
    """
    Bahaviour that finds path using A* algorithm
    """
    def __init__(self, node:Node):
        super(FindPath, self).__init__('FindPath')
        self.node = node

    def update(self):
        """
        Method that will be exectued when behaviour is ticked
        """
        self.node.send_goal_pub.publish(self.node.goal)
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


class StopRobot(py_trees.behaviour.Behaviour):
    """
    Behvaiour that stops robot
    """
    def __init__(self, node:Node):
        super(StopRobot, self).__init__('StopRobot')
        self.node = node
    
    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        msg = Bool()
        msg.data = True
        self.node.stop_robot_pub.publish(msg)
        self.node.have_goal = False
        self.node.have_path = False
        return py_trees.common.Status.SUCCESS


class DriveFree(py_trees.behaviour.Behaviour):
    """
    Behaviour that sends message to avoidance node to drive to free space
    """
    def __init__(self, node:Node):
        super(DriveFree, self).__init__('DriveFree')
        self.node = node
    
    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        msg = Bool()
        msg.data = True
        self.node.drive_to_free_pub.publish(msg)

        return py_trees.common.Status.RUNNING

class HavePath(py_trees.behaviour.Behaviour):
    """
    Behaviour that returns success if we got a path otherwise failure
    """
    def __init__(self, node:Node):
        super(HavePath, self).__init__('HavePath')
        self.node = node
    
    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        if self.node.have_path:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class HaveGoal(py_trees.behaviour.Behaviour):
    """
    Behaviour that returns success if we got a goal otherwise failure
    """
    def __init__(self, node:Node):
        super(HaveGoal, self).__init__('HaveGoal')
        self.node = node
    
    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        if self.node.have_goal:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class DriveToGoal(py_trees.behaviour.Behaviour):
    """
    Behaviour that sends path message to drive control
    """
    def __init__(self, node:Node):
        super(DriveToGoal, self).__init__('DriveToGoal')
        self.node = node
    
    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        if self.node.at_goal:
            return py_trees.common.Status.SUCCESS
        else:
            self.node.drive_to_goal_pub.publish(self.node.path)
            return py_trees.common.Status.RUNNING

class HaveScan(py_trees.behaviour.Behaviour):
    """
    Behaviour that returns success if we got a goal otherwise failure
    """
    def __init__(self, node:Node):
        super(HaveScan, self).__init__('HaveScan')
        self.node = node
    
    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        if self.node.have_scan:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class TakeRefScan(py_trees.behaviour.Behaviour):
    """
    Behaviour that publishes boolean to icp node so that it takes another icp reference scan
    """
    def __init__(self, node:Node):
        super(TakeRefScan, self).__init__('TakeRefScan')
        self.node = node

    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        msg = Bool()
        msg.data = True
        self.node.take_ref_scan_pub.publish(msg)
        return py_trees.common.Status.RUNNING

class HaveTurned(py_trees.behaviour.Behaviour):
    """
    Behaviour that returns success if we got a goal otherwise failure
    """
    def __init__(self, node:Node):
        super(HaveTurned, self).__init__('HaveTurned')
        self.node = node
    
    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        if self.node.have_turned:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class Turn(py_trees.behaviour.Behaviour):
    """
    Behaviour that turns the robot so that detection can happen better
    """
    def __init__(self, node:Node):
        super(Turn, self).__init__('Turn')
        self.node = node
    def update(self):
        """
        Method that will be executed when behaviour is ticked TODO: turning of the robot
        """
        self.node.have_turned = True 
        return py_trees.common.Status.RUNNING

class BrainCollection(Node):
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
        self.stop_drive_pub = False

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

        # Setup adn start ticking the tree (Managed by ROS)
        self.tree.setup(node=self,timeout=5) 
        self.tree.tick_tock(period_ms=1000) 


    def free_path_cb(self, msg:Bool):
        """
        callback that sets the condition if the path is free
        """
        self.free_path = msg.data
    
    def set_path_cb(self, msg:Path):
        """
        Callback that sets the path that will be sent to drive control
        """
        if len(msg.poses) == 0:
            self.have_goal = False
        else:
            self.path = msg
            self.have_path = True 

    def set_goal_cb(self, msg:PoseStamped):
        """
        Callback that sets the goal that will be sent to A*
        """
        self.goal = msg
        self.have_goal = True
    
    def corner_ex_done_cb(self, msg:Bool):
        """
        Callback that sets boolean, True if corner exploration is done, False otherwise
        """
        self.corner_ex_done = msg.data
    
    def no_unknown_left_cb(self, msg:Bool):
        """
        Callback that sets boolean, True if all exploration is done, False otherwise
        """
        self.no_unknown_left = msg.data        

    def at_goal_cb(self, msg: Bool):
        """
        Callback that sets boolean, True if drive control is at goal, False otherwise
        """
        self.at_goal = msg.data

    def have_scan_cb(self, msg: Bool):
        """
        Callback that sets boolean, True if ICP have taken a scan, False otherwise
        """
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

        # branch - obstacle avoidance
        drive_to_free = py_trees.composites.Sequence(name='DriveToFree', memory=False)
        drive_to_free.add_children([StopRobot(self), DriveFree(self)])

        obstacle_avoidance = py_trees.composites.Selector(name="ObstacleAvoidance", memory=False)         
        obstacle_avoidance.add_children([PathIsFree(self), drive_to_free]) 

        # branch - exploration
        icp_scan = py_trees.composites.Selector(name='GetIcpScan', memory=False)
        icp_scan.add_children([HaveScan(self), TakeRefScan(self)])

        turn = py_trees.composites.Selector(name='TurnRobot', memory=False)
        turn.add_children([HaveTurned(self), Turn(self)])

        get_goal = py_trees.composites.Selector(name='GetGoal', memory=False)
        get_goal.add_children([HaveGoal(self), FindGoal(self)])

        find_path = py_trees.composites.Sequence(name='FindPath', memory=False)
        find_path.add_children([get_goal, FindPath(self)])

        get_path = py_trees.composites.Selector(name='GetPath', memory=False)
        get_path.add_children([HavePath(self), find_path])

        exploration = py_trees.composites.Sequence(name='Exploration', memory=False)
        exploration.add_children([icp_scan, turn, get_path, DriveToGoal(self), SetSelf(self)])

        exploration_until_no_unknown_space =  py_trees.composites.Selector(name="ExploreUntilNoUnknownSpace", memory=False)
        exploration_until_no_unknown_space.add_children([NoUnknownSpaceLeft(self), exploration])

        # Main sequence
        root = py_trees.composites.Sequence(name='ExplorationRoot', memory=False)
        root.add_children([obstacle_avoidance, exploration_until_no_unknown_space])

        # return ros-wrapping of py_trees behaviour tree
        return py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)



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


