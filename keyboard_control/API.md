
## Description
Movement of the robot motors through command line interface. 

## how to operate?

### Terminal 1:

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=keyboard_control/keys --param stamped:=true


### Terminal 2:

ros2 run keyboard_control keyboard_control

## Interface

Keys (Swedish Keyboard):
 - i: forward
 - ,: backward
 - j: left
 - l: right
 - k: stop
