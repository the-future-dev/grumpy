# Autonomous robot
-

## Installation

In the workspace folder ex `dd2419_ws`:

`git clone git@github.com:the-future-dev/grumpy.git`

Rename `grumpy` to `src`.

To install the robp_robot depenendency:
`cd ~/dd2419_ws/src/robp_robot`
`git pull`
`git submodule update --init --force --remote`
`cd ~/dd2419_ws`
`colcon build --symlink-install`

