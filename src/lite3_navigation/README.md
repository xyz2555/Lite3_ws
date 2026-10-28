# lite3_navigation

Closed-loop **go-to-point** for the Lite3 from RViz2's **2D Goal Pose**. It reads the robot pose
from TF `odom -> base_link` (published by `lite3_driver`), rotates toward the goal, drives
forward, then aligns to the final yaw. No map is needed. The pose drifts over time (leg odometry),
so use it for goals within a few metres of where the robot started.

## 1. Install (once)

```bash
# driver: publish /lite3/basic_state (needed so the controller knows the robot is standing)
cd ~/lite3_basic_ws && patch -p1 < driver_basic_state.patch

# this package
cp -r lite3_navigation ~/lite3_basic_ws/src/
cd ~/lite3_basic_ws && colcon build --packages-select lite3_driver lite3_navigation
source install/setup.bash

# optional, only for use_mux:=true
sudo apt install ros-humble-twist-mux
```

## 2. Try it without the robot (fake robot)

```bash
ros2 launch lite3_navigation goto.launch.py sim:=true
rviz2      # Fixed Frame = odom; add TF, Odometry (Keep: 1), and a Pose display on /goal_pose
```
Click **2D Goal Pose** in the RViz toolbar, then click-and-drag on the grid (the drag direction is
the final yaw). Watch `ros2 topic echo /lite3/goto/status`.
A virtual obstacle: `... goto.launch.py sim:=true obstacle_x:=1.5` (robot waits, then gives up after `block_timeout`).
Never run the fake robot together with the real `lite3_driver`.

## 3. On the real robot

```bash
ros2 launch lite3_bringup bringup.launch.py            # driver + obstacle_guard (as before)
ros2 service call /lite3/stand_up std_srvs/srv/Trigger  # then wait ~3 s
ros2 service call /lite3/enter_nav std_srvs/srv/Trigger
ros2 launch lite3_navigation goto.launch.py
```
Then use **2D Goal Pose** in RViz2 (Fixed Frame `odom`). Start with a goal 0.5 m ahead.
Keep the remote/estop in hand, an empty area, and do NOT run teleop at the same time (see below).

### Teleop priority (recommended once the basics work)

```bash
ros2 launch lite3_navigation goto.launch.py use_mux:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop
```
Flow: go_to_point -> `/cmd_vel_auto`, teleop -> `/cmd_vel_teleop`, `twist_mux` -> `/cmd_vel` -> obstacle_guard.
Pressing a teleop key **cancels the current goal** (`ABORTED:teleop_override`); send a new goal to resume.

## Status / reasons

`/lite3/goto/status`: `IDLE`, `ROTATE`, `MOVE`, `WAIT_BLOCKED`, `ALIGN`, `SUCCEEDED`, `ABORTED:<reason>` where reason is
`timeout`, `blocked_timeout`, `no_progress`, `pose_stale`, `pose_invalid`, `not_standing`, `teleop_override`, `cancelled`, `disabled`.

Services: `/go_to_point/cancel` (Trigger), `/go_to_point/enable` (SetBool).
Tuning: `config/go_to_point.yaml` (start with `max_vx: 0.3`, `max_wz: 0.5`; use decimals for every float).

## Tests (no ROS needed)

```bash
cd lite3_navigation && python3 -m pytest -q test      # 20 tests incl. closed-loop simulations
```
