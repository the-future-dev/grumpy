# Run

<!--  Setup -->
cd ~/dd2419_ws/src/perception_pkg/launch && ros2 launch rosbag_play.py

rviz2 -d rosbag_play.rviz

ros2 bag play --read-ahead-queue-size 100 -l -r 1.0 --clock 100 --start-paused ~/dd2419_ws/bags/milestone_1_bag

<!-- Perception Node -->
ros2 run perception_pkg detection_node

