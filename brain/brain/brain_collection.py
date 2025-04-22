#!/usr/bin/env python
import py_trees
import py_trees_ros

import rclpy
from rclpy.node import Node

from time import sleep

from nav_msgs.msg import Path
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from grumpy_interfaces.msg import ObjectDetection1D
from grumpy_interfaces.srv import PickAndDropObject

# Behaviours
class NoObjectsLeft(py_trees.behaviour.Behaviour):
    """
    Bahaviour that checks if exploration is done by checking if there is no unknown space left
    """
    def __init__(self, node:Node):
        super(NoObjectsLeft, self).__init__('NoObjectsLeft')
        self.node = node
    
    def update(self):
        """
        Method that will be executed when behaviour is ticked
        """
        if self.node.no_objects_left:
            self.node.get_logger().info('No objects left, behaviour tree done')
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class ReadyForPickDrop(py_trees.behaviour.Behaviour):
    def __init__(self, node:Node):
        super(ReadyForPickDrop, self).__init__('ReadyForPickDrop')
        self.node = node
    
    def update(self):
        if self.node.at_goal:
            self.node.get_logger().debug('Ready for pick/drop')
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().debug('Not ready for pick/drop')
            return py_trees.common.Status.FAILURE


class PickDrop(py_trees.behaviour.Behaviour):
    def __init__(self, node:Node):
        super(PickDrop, self).__init__('PickDrop')
        self.node = node

    def update(self):
        if self.node.pick_drop_status == 'Not_called':
            if self.node.action == 'Pick':
                self.node.get_logger().info('publishing to do pickup service')
                msg = String()
                msg.data = 'Pick'
                self.node.pick_drop_pub.publish(msg)
            else:
                self.node.get_logger().info('publishing to do drop service')
                msg = String()
                msg.data = 'Drop'
                self.node.pick_drop_pub.publish(msg)
           
            self.node.pick_drop_status = 'Called'
        else:   
            if self.node.pick_drop_status == 'Success':
                self.node.get_logger().info('Pick/Drop Success')
                self.node.have_path = False
                self.node.have_goal = False
                self.node.get_path = True
                self.node.get_goal = True
                self.node.at_goal = False
                self.node.at_sub_goal = False
                self.node.pick_drop_status = 'Not_called'

                if self.node.action == 'Pick':
                    self.node.action = 'Drop'
                else:
                    self.node.action = 'Pick'

            elif self.node.pick_drop_status == 'Failure':
                self.node.get_logger().info('Pick/Drop Failure')
                self.node.have_path = False
                self.node.have_goal = False
                self.node.get_path = True
                self.node.get_goal = True
                self.node.at_goal = False
                self.node.at_sub_goal = False
                self.node.pick_drop_status = 'Not_called'
                if self.node.action == 'Drop':
                    self.node.action = 'Pick' # if drop fails try to pick up new object
            else:
                self.node.get_logger().debug('Waiting for pick/drop service')

        return py_trees.common.Status.RUNNING


class GetPath(py_trees.behaviour.Behaviour):
    """
    Bahaviour that finds path using A* algorithm, if there is no goal it begins with getting a goal from the planner 
    """
    def __init__(self, node:Node):
        super(GetPath, self).__init__('GetPath')
        self.node = node


    def update(self):
        """
        Method that will be exectued when behaviour is ticked
        """
        # self.node.get_logger().debug(f'Booleans: at_sub_goal={self.node.at_sub_goal}, recalculate_path={self.node.recalculate_path}, get_path={self.node.get_path}, get_goal={self.node.get_goal}, have_path={self.node.have_path}, have_goal={self.node.have_goal}')
        if self.node.have_path:
            # if we have a path we want to get path next time we go into this part of the behaviour
            self.node.get_path = True
            self.node.get_logger().debug('We have a path to pass on to DriveToGoal')
            return py_trees.common.Status.SUCCESS

        else:
            self.node.get_logger().debug('We do not have a path')
            if self.node.have_goal:
                # if we have a goal we want to get goal next time we go into this part of the behaviour
                self.node.get_goal = True
                
                # if we have a goal send it to A* but only do it once
                if self.node.get_path:
                    if not self.node.at_sub_goal and self.node.recalculate_path:
                        self.node.get_logger().info('We are waiting for the robot to reach goal')
                        return py_trees.common.Status.RUNNING

                    self.node.get_path = False
                    self.node.get_logger().info('Publishing goal to A*')
                    msg = PoseStamped()
                    msg.pose = self.node.goal.pose


                    self.node.send_goal_pub.publish(msg)
                else:
                    self.node.get_logger().debug('Waiting for path from A*')    
            else:
                # only publish to get goal once
                if self.node.get_goal:
                    self.node.get_goal = False

                    self.node.get_logger().info('Publishing Bool to planner to get goal')
                    msg = String()
                    msg.data = self.node.action
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
            if not self.node.at_sub_goal:
                self.node.get_logger().debug('Waiting for drive control to reach point')
                return py_trees.common.Status.RUNNING
            else:
                self.empty_list = False
                self.node.set_poses_list = True
                self.node.at_goal = True # have reached the final goal
                return py_trees.common.Status.SUCCESS
        else:
            if self.node.set_poses_list:
                self.node.set_poses_list = False
                self.poses_list = self.node.path.poses
                self.node.get_logger().debug(f'The goal pose from A*; x:{self.node.path.poses[-1].pose.position.x}, y: {self.node.path.poses[-1].pose.position.y}')
            else:
                if not self.node.at_sub_goal:
                    self.node.get_logger().debug('Waiting for drive control to reach point')
                    return py_trees.common.Status.RUNNING
                else:
                    if len(self.poses_list)==0:
                        self.empty_list = True
                        self.node.get_logger().info(f'No poses left, should have reached goal')
                        return py_trees.common.Status.RUNNING


            self.node.at_sub_goal = False    
            self.node.get_logger().debug(f'Poses List length: {len(self.poses_list)}')
            
            path = self.node.path
            path.poses = self.poses_list
            self.node.path_list_pub.publish(path)

            if len(self.poses_list)>0:
                pose = self.poses_list.pop(0)

            self.node.path.poses = [pose]
            self.node.get_logger().debug('Publishing next pose to drive_control')
            self.node.drive_to_goal_pub.publish(self.node.path)

            return py_trees.common.Status.RUNNING


class BrainCollection(Node):
    def __init__(self):
        super().__init__('BrainCollection')

        # initalize self values
        self.have_path = False
        self.have_goal = False
        self.recalculate_path = False

        self.get_goal = True
        self.get_path = True

        self.at_sub_goal = True
        self.at_goal = False
        self.no_objects_left = False

        self.free_path = True

        self.goal = None
        self.path = None

        self.action = 'Pick'
        self.pick_drop_status = 'Not_called'

        
        # intialize publishers
        self.send_goal_pub = self.create_publisher(PoseStamped, 'Astar/next_goal', 1)
        self.find_goal_pub = self.create_publisher(String, 'planner_collection/find_goal', 1)
        self.drive_to_goal_pub = self.create_publisher(Path, 'drive/path', 1)
        self.path_list_pub = self.create_publisher(Path, '/brain/path_list', 1)
        self.pick_drop_pub = self.create_publisher(String, 'brain/action/pick_drop', 1)

        # initialize clients
        self.pick = rclpy.create_node('pick_drop_brain_node')
        self.drop = rclpy.create_node('drop_brain_node')

        self.pick_client = self.pick.create_client(PickAndDropObject, '/arm_services/pick_object')
        self.drop_client = self.drop.create_client(PickAndDropObject, '/arm_services/drop_object')

        # Create ROS 2 behaviour tree
        self.tree = self.create_behaviour_tree()

        # Setup of behaviour tree
        self.tree.setup(node=self,timeout=5) 

        # Sleeping for 10 seconds so that all nodes are initalized properly
        self.get_logger().debug('Sleeping for 10 seconds before starting to tick')
        sleep(10)

        # Starting to tick the tree
        self.get_logger().debug('Starting to tick the tree')
        self.tree.tick_tock(period_ms=500) 

    def free_path_cb(self, msg:Bool):
        """
        callback that sets the condition if the path is free
        """
        self.get_logger().debug(f'Avoidance node is telling us that path_free = {msg.data}')
        self.free_path = msg.data
    
        if not self.free_path:
             # only recalculate path once when path is not free
            self.get_logger().info('Path is not free')
            if not self.recalculate_path:
                self.get_logger().info('Setting have_path to false so that A* calculates path again')
                self.recalculate_path = True
                self.have_path = False
                self.get_path = True
                self.have_goal = True
                self.get_goal = True 
                self.set_poses_list = True
        else:
            self.get_logger().debug('Path is free')
            self.recalculate_path = False
    
    def pick_drop_status_cb(self, msg:String):
        """
        Callback that sets the status of the pick drop services
        """
        self.pick_drop_status = msg.data


    def set_path_cb(self, msg:Path):
        """
        Callback that sets the path that will be sent to drive control
        """
        if len(msg.poses) == 0:
            self.get_logger().info('Path from A* is empty')
            self.have_goal = False
            self.get_path = True
        else:
            self.get_logger().debug('Have received a non-empty path from A*')
            self.path = msg
            self.have_path = True
            self.recalculate_path = False 

    def set_goal_cb(self, msg:ObjectDetection1D):
        """
        Callback that sets the goal that will be sent to A*
        """
        self.get_logger().info('Have received goal from planner')

        if msg.label.data == 'Near':
            self.get_logger().info('Goal close to robot go directly to pick/drop')
            self.at_goal = True
        else:
            self.get_logger().info('Goal not close, getting path')
            self.goal = msg
            self.have_goal = True
            self.get_path = True
    
    def no_objects_left_cb(self, msg:Bool):
        """
        Callback that sets boolean, True if all exploration is done, False otherwise
        """
        self.get_logger().debug(f'Planner telling us that no objects left = {msg.data}')
        self.no_objects_left = msg.data        

    def at_sub_goal_cb(self, msg: Bool):
        """
        Callback that sets boolean, True if drive control is at goal, False otherwise
        """
        self.get_logger().debug(f'Drive control telling us that reached_goal = {msg.data}')
        self.at_sub_goal = msg.data

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
                                    ('Goal', '/planner_collection/next_goal', ObjectDetection1D, False, self.set_goal_cb),
                                    ('NoObjectsLeft', '/planner_collection/no_objects_left', Bool, False, self.no_objects_left_cb),
                                    ('FreePath', '/avoidance/free_path', Bool, False, self.free_path_cb),
                                    ('AtGoal', '/drive/feedback', Bool, False, self.at_sub_goal_cb),
                                    ('PickDropTopic', 'brain/pick_drop_status', String, False, self.pick_drop_status_cb)
                                    ])


        # Building tree
        # memory=False means that conditions are ticked each iteration, otherwise the tree returns to the behaviour that returned running last
        get_to_pick_drop = py_trees.composites.Sequence(name='GetToPickDrop', memory=False)
        get_to_pick_drop.add_children([GetPath(self), DriveToGoal(self)])

        ready_for_pick_place = py_trees.composites.Selector(name='ReadyForPickPlace', memory=False)
        ready_for_pick_place.add_children([ReadyForPickDrop(self), get_to_pick_drop])

        collection = py_trees.composites.Sequence(name='Collection', memory=False)
        collection.add_children([ready_for_pick_place, PickDrop(self)])

        # Root
        root =  py_trees.composites.Selector(name="CollectionUntilNoObjectsLeft", memory=False)
        root.add_children([NoObjectsLeft(self), collection])

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
