# lite3_basic_ws

ROS 2 Humble (Python) workspace for the **DEEP Robotics Lite3** quadruped, built for
the *Mastering Quadruped Robots* workshop. It talks to the Lite3 motion host over UDP,
stops the robot automatically when something is too close, and contains the lab code
used on the slides.

Tested target: Ubuntu 22.04, ROS 2 Humble, Lite3 Venture (no perception host, the
laptop talks to the motion host directly).

```
 teleop / your node                                     Lite3 motion host
        │ /cmd_vel                                     192.168.1.120:43893
        ▼                                                     ▲
 ┌────────────────┐  /lite3/cmd_vel_safe  ┌──────────────┐ UDP │
 │ obstacle_guard │ ────────────────────▶ │ lite3_driver │─────┘
 └────────────────┘                       └──────────────┘◀──── state 0x0901 (UDP :43897)
        ▲   ▲                                    │
        │   └── depth image (optional)           ├─ /lite3/odom, /lite3/imu, TF odom→base_link
        └────── /lite3/ultrasound/{front,back} ◀─┘
```

| Package | What it does |
|---|---|
| `lite3_driver` | UDP bridge: heartbeat, stand/sit, navigation mode, e-stop, `cmd_vel` → robot, odom / IMU / ultrasound / TF from the robot |
| `lite3_safety` | `obstacle_guard`: slows down and stops forward/backward motion near obstacles using the built-in ultrasound and/or a depth camera |
| `lite3_motion` | `move_cli` (one-shot relative move) and `waypoint_runner` (JSON plan), both pause while blocked |
| `lite3_workshop` | Slide labs: `hello_pub`, `go_to_goal` (turtlesim), `square_walk`, `turtle.launch.py` |
| `lite3_bringup` | One launch file that wires everything together |

---

## 1. Build

```bash
mkdir -p ~/lite3_ws && cd ~/lite3_ws
git clone https://github.com/agus-darmawan/lite3_basic_ws.git .
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## 2. Network

| How you connect | Laptop IP | Motion host |
|---|---|---|
| Robot Wi-Fi | 192.168.2.x | `192.168.2.1` |
| Robot Wi-Fi | 192.168.1.x | `192.168.1.120` |
| Ethernet port on the back | 192.168.1.x (static) | `192.168.1.120` |

```bash
ping 192.168.1.120
```

Commands work as soon as the robot is reachable. To **receive** odometry, IMU and
ultrasound, the motion host must send its state to your laptop: SSH into the robot and
point the report address in `~/jy_exe/conf/network.toml` to your laptop IP and port
`43897` (the driver's `local_port`), then restart the motion program. On a Venture
there is no perception host, so this is required.

## 3. Run

```bash
# terminal A: driver + obstacle guard
ros2 launch lite3_bringup bringup.launch.py robot_ip:=192.168.1.120

# terminal B: stand up, then enable velocity control
ros2 service call /lite3/stand_up std_srvs/srv/Trigger
# wait ~3 s until the robot is fully standing
ros2 service call /lite3/enter_nav std_srvs/srv/Trigger

# drive (hold the key; the robot stops 0.5 s after the last command)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Stop at any time:

```bash
ros2 service call /lite3/estop std_srvs/srv/Trigger    # soft e-stop
ros2 service call /lite3/sit_down std_srvs/srv/Trigger # stand/sit toggle
```

Always keep the physical remote in hand as the primary stop.

### One-shot moves and waypoint plans

```bash
ros2 run lite3_motion move_cli --x 1.0                 # forward 1 m
ros2 run lite3_motion move_cli --yaw 90                # turn left 90 deg
ros2 run lite3_motion move_cli --x 0.5 --y 0.2 --speed 0.2

ros2 run lite3_motion waypoint_runner --ros-args -p waypoints_file:=\
$(ros2 pkg prefix lite3_motion)/share/lite3_motion/waypoints/square.json
```

Waypoints are **relative** body-frame moves: rotate `yaw` degrees first (CCW positive),
then move `x` (forward) / `y` (left) metres. Execution is open-loop (time = distance /
speed), so expect drift on long plans.

---

## 4. Obstacle stop (`lite3_safety`)

`obstacle_guard` sits between `/cmd_vel` and the driver:

| Distance to obstacle | Forward command |
|---|---|
| > `slow_distance` (1.0 m) | passed through |
| between stop and slow | scaled down linearly |
| < `stop_distance` (0.5 m) | **zero**, until the path is clear by `stop + hysteresis` |
| no fresh sensor data | zero (`fail_safe: true`) |

Backward motion is gated the same way with the back ultrasound. Turning and side-steps
stay allowed so the operator can steer away (`allow_rotation_when_blocked`,
`allow_lateral_when_blocked`). `move_cli` and `waypoint_runner` pause their clock while
blocked and continue when the path is clear.

**Sources**

1. **Built-in ultrasound** (`use_ultrasound:=true`, default). The driver reads
   `ultrasound[2]` (front, back, metres) from the tail of the `0x0901` state packet and
   publishes `sensor_msgs/Range`. Check that your unit reports it:
   ```bash
   ros2 topic echo /lite3/ultrasound/front --field range
   ```
   If nothing is published, your firmware does not fill this field (or appends extra
   fields after it; set the driver parameter `ultrasound_offset` to the byte offset).
   With `fail_safe: true` the guard will then block forward motion, so switch to the
   depth source or launch with `use_guard:=false`.

2. **Depth camera** (`use_depth:=true`), e.g. an Intel RealSense D435i plugged into the
   laptop or onboard computer, facing forward:
   ```bash
   sudo apt install ros-humble-realsense2-camera
   ros2 launch lite3_bringup bringup.launch.py use_depth:=true launch_realsense:=true
   # or only the depth camera as source:
   ros2 launch lite3_bringup bringup.launch.py use_depth:=true use_ultrasound:=false \
       launch_realsense:=true
   ```
   The guard takes the 5th-percentile depth inside a centre ROI (skipping the lower part
   of the image where the floor appears). Tune `depth_roi`, `depth_offset` (lens to
   front of the robot) and the distances in `lite3_safety/config/obstacle_guard.yaml`.
   Depth-only mode does not protect backward motion.

Monitor and control:

```bash
ros2 topic echo /lite3/obstacle/front_distance     # -1 = no data, inf = clear
ros2 topic echo /lite3/obstacle/blocked
ros2 service call /obstacle_guard/enable std_srvs/srv/SetBool "{data: false}"  # bypass
```

Pass distances as floats on the command line (`stop_distance:=0.6`, not `:=1`).

---

## 5. Workshop labs (`lite3_workshop`)

```bash
export ROS_LOCALHOST_ONLY=1                    # labs 1-3 on your own laptop
ros2 run turtlesim turtlesim_node
ros2 run lite3_workshop hello_pub              # Lab 2
ros2 run lite3_workshop go_to_goal             # Lab 3
ros2 param set /go_to_goal x 2.0
ros2 launch lite3_workshop turtle.launch.py    # launch-file slide

unset ROS_LOCALHOST_ONLY                       # labs 4+ on the robot network
export ROS_DOMAIN_ID=<id from the instructor>
ros2 run lite3_workshop square_walk            # Lab 6, publishes /cmd_vel
```

All workshop nodes send a zero `Twist` on Ctrl+C. They use
`rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` for that: with the default
handler, Ctrl+C shuts the ROS context down before the `finally` block, and the last
publish fails.

---

## 6. ROS interfaces

**lite3_driver**

| Name | Type | Notes |
|---|---|---|
| `cmd_vel_topic` (sub) | `geometry_msgs/Twist` | `cmd_vel` standalone, `/lite3/cmd_vel_safe` in bringup with the guard |
| `/lite3/odom` | `nav_msgs/Odometry` | pose from `pos_world`, twist from `vel_body` |
| `/lite3/imu` | `sensor_msgs/Imu` | from `rpy`, `rpy_vel`, `xyz_acc` |
| `/lite3/ultrasound/front`, `/back` | `sensor_msgs/Range` | only if the unit reports valid values |
| TF `odom → base_link` | | `publish_tf: true` |
| `/lite3/stand_up`, `/lite3/sit_down` | `std_srvs/Trigger` | same stand/sit **toggle** command |
| `/lite3/enter_nav`, `/lite3/exit_nav` | `std_srvs/Trigger` | navigation mode is required for velocity |
| `/lite3/estop` | `std_srvs/Trigger` | soft emergency stop |

Driver parameters: `robot_ip`, `robot_port` (43893), `local_port` (43897),
`cmd_timeout` (0.5 s), `resend_hz` (20), `heartbeat_hz` (4, protocol minimum 2),
`publish_tf`, `rpy_in_degrees` (tilt the robot ~10° and check `/lite3/imu`; set to
`true` if the firmware reports degrees), `ultrasound_offset`, `ultrasound_max_range`.

**Velocity limits** (clamped by the driver): `linear.x` ±1.0 m/s, `linear.y` ±0.5 m/s,
`angular.z` ±1.5 rad/s. Positive `angular.z` turns left (REP 103).

---

## 7. Troubleshooting

| Symptom | Check |
|---|---|
| Robot does not move | standing? `enter_nav` called? guard blocked (`/lite3/obstacle/blocked`)? |
| Moves for half a second, then stops | publish continuously (≥ 2 Hz); the driver stops after `cmd_timeout` |
| No `/lite3/odom` | `network.toml` report address/port not pointing at this laptop |
| Guard always blocked | no ultrasound on this unit → `use_depth:=true` or `use_guard:=false` |
| Nothing on the depth topic | `ros2 topic list | grep depth`; RealSense 4.x uses `/camera/camera/...` |

Damage caused by SDK use is not covered by the manufacturer's warranty. Test new code
at low speed in a clear area with the remote in hand.

---

## 8. Development

```bash
# pure-Python unit tests (no ROS needed)
PYTHONPATH=src/lite3_driver pytest -q src/lite3_driver/test
PYTHONPATH=src/lite3_safety pytest -q src/lite3_safety/test
PYTHONPATH=src/lite3_motion pytest -q src/lite3_motion/test
flake8 --max-line-length 100 src
```

Commits follow [Conventional Commits](https://www.conventionalcommits.org/)
(`feat:`, `fix:`, `refactor:`, `docs:`, `ci:`, `chore:`). See `CONTRIBUTING.md`.

License: MIT
