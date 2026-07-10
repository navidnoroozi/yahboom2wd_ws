# yahboom_2wd_dmpc

ROS 2 / ZeroMQ interface package for running the existing distributed MPC planning algorithm on two commissioned Yahboom 2WD robots.

The package connects the already validated Yahboom ROS 2 interface to the distributed MPC app. It does not replace the low-level Yahboom bridge. The Yahboom bridge still owns the serial connection to the STM32 board, publishes odometry, and receives `cmd_vel`. The `yahboom_2wd_dmpc` package sits above it and provides the robot-side MPC controller nodes and the Ubuntu-VM coordinator node.

## Runtime topology

The first hardware topology is a star:

```text
robot1 Raspberry Pi  <---- ZeroMQ ----\
                                       \
                                        Ubuntu VM coordinator
                                       /
robot2 Raspberry Pi  <---- ZeroMQ ----/
```

The robots do not communicate directly with each other. The Ubuntu VM is the hub.

### Robot-side processes

On `robot1` Raspberry Pi:

```text
yahboom_2wd_bringup/yahboom_2wd_node
  publishes:   /robot1/odom
  subscribes:  /robot1/cmd_vel

yahboom_2wd_dmpc/dmpc_controller_node
  agent_id:    1
  ZMQ REP:     tcp://*:5601
```

On `robot2` Raspberry Pi:

```text
yahboom_2wd_bringup/yahboom_2wd_node
  publishes:   /robot2/odom
  subscribes:  /robot2/cmd_vel

yahboom_2wd_dmpc/dmpc_controller_node
  agent_id:    2
  ZMQ REP:     tcp://*:5602
```

### Ubuntu VM process

On the Ubuntu VM:

```text
yahboom_2wd_dmpc/dmpc_coordinator_ros_node
  subscribes:  /robot1/odom
               /robot2/odom

  publishes:   /robot1/cmd_vel
               /robot2/cmd_vel
               /dmpc/robot1/pose_world
               /dmpc/robot2/pose_world
               /dmpc/robot1/u_world
               /dmpc/robot2/u_world

  ZMQ REQ:     tcp://<robot1-ip>:5601
               tcp://<robot2-ip>:5602
```

## Package skeleton

```text
yahboom_2wd_dmpc/
├── yahboom_2wd_dmpc/
│   ├── consensus_comm.py              # JSON envelope helpers for ZeroMQ messages
│   ├── consensus_config.py            # NetConfig and common DMPC parameters
│   ├── consensus_controller.py        # single-/double-integrator MPC solvers
│   ├── explicit_hybrid_controller.py  # practical safety supervisor
│   ├── dmpc_controller_node.py        # robot-side ZMQ REP controller node
│   ├── dmpc_coordinator_ros_node.py   # VM-side ROS/ZMQ coordinator
│   ├── config_utils.py
│   ├── controller_node.py             # original/SIL-compatible controller node
│   ├── coordinator_node.py            # original/SIL-compatible coordinator node
│   └── __init__.py
├── launch/
│   ├── robot_dmpc_controller.launch.py
│   └── two_robot_dmpc_coordinator.launch.py
├── config/
│   └── two_robot_dmpc.yaml
├── resource/
│   └── yahboom_2wd_dmpc
├── README_MPC_INTERFACE.md
├── package.xml
├── setup.py
└── setup.cfg
```

## Common world/map frame

Each Yahboom robot publishes local odometry. Therefore, after startup it is normal that both robots report approximately:

```text
robot1 local odom: x ≈ 0, y ≈ 0, yaw ≈ 0
robot2 local odom: x ≈ 0, y ≈ 0, yaw ≈ 0
```

This is not enough for distributed MPC. The MPC must know all robot positions in one shared world/map frame before computing formation errors, inter-robot distances, obstacle distances, and collision-avoidance constraints.

The coordinator therefore supports measured initial world poses:

```text
robot1_initial_x
robot1_initial_y
robot1_initial_yaw

robot2_initial_x
robot2_initial_y
robot2_initial_yaw
```

For robot `i`, the coordinator transforms local odometry to world coordinates as:

```text
x_world   = x0_i + cos(yaw0_i) * x_local - sin(yaw0_i) * y_local
y_world   = y0_i + sin(yaw0_i) * x_local + cos(yaw0_i) * y_local
yaw_world = yaw0_i + yaw_local
```

A simple first floor setup is:

```text
world origin: marked point on the floor
world x-axis: marked with tape
robot1:       x0 = 0.0 m, y0 = -0.7 m, yaw0 = 0.0 rad
robot2:       x0 = 0.0 m, y0 = +0.7 m, yaw0 = 0.0 rad
```

The exact values must match the measured physical placement before starting the coordinator. Manual initial-pose offsets are acceptable for the first short, slow tests.

The coordinator publishes the transformed common-frame poses on:

```text
/dmpc/robot1/pose_world
/dmpc/robot2/pose_world
```

Use these topics as the main verification that the MPC is using the intended world-frame positions. The raw `/robot1/odom` and `/robot2/odom` topics may still start near local `(0, 0, 0)`.

For longer experiments or stronger safety claims, use shared external localization such as overhead camera/AprilTags, motion capture, UWB, or another global pose-correction method.

## Installation

Copy this package into the same workspace as the existing Yahboom packages:

```bash
cd ~/yahboom2wd_ws/src
# copy or clone yahboom_2wd_dmpc here

cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src --rosdistro humble \
  --skip-keys "ament_python"
colcon build --symlink-install
source install/setup.bash
```

The `--skip-keys "ament_python"` option is useful on Raspberry Pi / Humble installations where `rosdep` cannot resolve the `ament_python` key although the packages build successfully.

## Python dependencies

Install the base dependencies:

```bash
sudo apt update
sudo apt install -y python3-zmq python3-numpy python3-pip
```

On the Raspberry Pis, keep the user-local solver stack consistent. If you see NumPy/CVXPY/ECOS import errors, use:

```bash
python3 -m pip uninstall -y numpy cvxpy ecos scs osqp clarabel

python3 -m pip install --user --force-reinstall \
  "numpy<2" \
  "cvxpy<1.5" \
  "ecos<2.1" \
  "scs<3.3" \
  "osqp<0.7"

python3 - <<'PY'
import numpy
import cvxpy as cp
print("numpy:", numpy.__version__, numpy.__file__)
print("cvxpy:", cp.__version__)
print("installed solvers:", cp.installed_solvers())
PY
```

The current Raspberry Pi setup should use NumPy `1.x`. A mixed environment with NumPy `2.x` and old compiled solver wheels can fail with:

```text
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x
AttributeError: _ARRAY_API not found
ImportError: numpy.core.multiarray failed to import
```

## Robot-side Yahboom bridge

Start the calibrated Yahboom bridge on each robot first.

Example for `robot1`:

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch yahboom_2wd_bringup yahboom_2wd.launch.py \
  namespace:=robot1 \
  serial_port:=/dev/myserial \
  command_mode:=motion \
  linear_cmd_scale:=1.7 \
  angular_cmd_scale:=1.0 \
  odom_linear_scale:=1.5
```

Use `namespace:=robot2` and the calibrated `robot2` values on the second robot.

## Robot-side DMPC controller nodes

Start the local controller node after the Yahboom bridge is running.

On `robot1`:

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch yahboom_2wd_dmpc robot_dmpc_controller.launch.py \
  agent_id:=1
```

Expected output:

```text
[YahboomDMPCController 1] REP bound at tcp://*:5601 ...
```

On `robot2`:

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch yahboom_2wd_dmpc robot_dmpc_controller.launch.py \
  agent_id:=2
```

Expected output:

```text
[YahboomDMPCController 2] REP bound at tcp://*:5602 ...
```

The controller node may remain quiet after startup. That is normal. It waits for ZeroMQ requests from the VM coordinator.

## VM-side coordinator dry run

Always start with `enable_motion:=false`. In this mode, the coordinator computes the DMPC commands and publishes debug topics, but it publishes zero `cmd_vel` to the robots.

Replace the IP addresses with the real Raspberry Pi LAN IP addresses. Do not use angle brackets in Bash.

Example:

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch yahboom_2wd_dmpc two_robot_dmpc_coordinator.launch.py \
  robot1_controller_endpoint:=tcp://192.168.178.87:5601 \
  robot2_controller_endpoint:=tcp://192.168.178.94:5602 \
  robot1_initial_x:=0.0 \
  robot1_initial_y:=-0.7 \
  robot1_initial_yaw:=0.0 \
  robot2_initial_x:=0.0 \
  robot2_initial_y:=0.7 \
  robot2_initial_yaw:=0.0 \
  enable_motion:=false
```

Expected startup messages include:

```text
REQ -> controller for agent 1: tcp://192.168.178.87:5601
REQ -> controller for agent 2: tcp://192.168.178.94:5602
DMPC coordinator started for namespaces=['robot1', 'robot2'], agent_ids=[1, 2], enable_motion=False
enable_motion=false: the node will compute commands but publish zero cmd_vel.
```

## Dry-run checks

Open another VM terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42
```

Check ROS visibility:

```bash
ros2 topic list | grep robot
ros2 topic hz /robot1/odom
ros2 topic hz /robot2/odom
```

Check DMPC debug outputs:

```bash
ros2 topic list | grep dmpc
ros2 topic echo /dmpc/robot1/pose_world
ros2 topic echo /dmpc/robot2/pose_world
ros2 topic echo /dmpc/robot1/u_world
ros2 topic echo /dmpc/robot2/u_world
```

For the verified dry-run setup with `robot1_initial_y:=-0.7` and `robot2_initial_y:=0.7`, the expected world poses at startup are:

```text
/dmpc/robot1/pose_world -> frame_id = map, position.x = 0.0, position.y = -0.7
/dmpc/robot2/pose_world -> frame_id = map, position.x = 0.0, position.y = +0.7
```

Check that physical motion is disabled:

```bash
ros2 topic echo /robot1/cmd_vel
ros2 topic echo /robot2/cmd_vel
```

With `enable_motion:=false`, the robot command topics should remain zero or nearly zero even if `/dmpc/robot*/u_world` contains nonzero planned world-frame commands.

## Current verified dry-run status

The two-robot dry run has been verified with both robot-side controllers running and the Ubuntu VM coordinator launched with:

```text
robot1: x0 = 0.0 m, y0 = -0.7 m, yaw0 = 0.0 rad
robot2: x0 = 0.0 m, y0 = +0.7 m, yaw0 = 0.0 rad
```

The following conditions have been confirmed:

```text
/dmpc/robot1/pose_world exists and reports frame_id = map, position = (0.0, -0.7, 0.0)
/dmpc/robot2/pose_world exists and reports frame_id = map, position = (0.0, +0.7, 0.0)
/dmpc/robot1/u_world and /dmpc/robot2/u_world are published
/robot1/cmd_vel and /robot2/cmd_vel remain zero when enable_motion=false
```

This means the ROS 2 namespaces, ZeroMQ controller communication, common-frame odometry transform, and dry-run safety gate are working. The next step is a short low-speed test with `enable_motion:=true`.

## First motion-enabled test

Only after the dry-run checks are correct, run a short conservative motion-enabled test. Use a clear test area, low battery-safe speed, a measured initial separation larger than `d_safe`, and be ready to stop the coordinator with `Ctrl+C`.

Recommended first motion-enabled launch:

```bash
ros2 launch yahboom_2wd_dmpc two_robot_dmpc_coordinator.launch.py \
  robot1_controller_endpoint:=tcp://192.168.178.87:5601 \
  robot2_controller_endpoint:=tcp://192.168.178.94:5602 \
  robot1_initial_x:=0.0 \
  robot1_initial_y:=-0.7 \
  robot1_initial_yaw:=0.0 \
  robot2_initial_x:=0.0 \
  robot2_initial_y:=0.7 \
  robot2_initial_yaw:=0.0 \
  max_linear_speed:=0.04 \
  max_angular_speed:=0.20 \
  u_bound:=0.04 \
  formation_hold_enabled:=true \
  formation_hold_metric:=pairwise \
  formation_hold_enter_error:=0.04 \
  formation_hold_exit_error:=0.08 \
  formation_hold_heading_enabled:=true \
  formation_hold_max_angular_speed:=0.12 \
  enable_motion:=true
```

This is intentionally slower than the default commissioning values. If the motion is smooth and the bag looks correct, repeat with the default values.

### Formation hold / deadband layer

The coordinator contains a practical formation-hold layer for real hardware.  When the team is already inside the configured formation error band, the coordinator suppresses translational commands and optionally publishes only an in-place angular command so that the two robots face each other.

Default hold parameters:

```text
formation_hold_enabled: true
formation_hold_metric: pairwise        # pairwise, slot, or both
formation_hold_enter_error: 0.04 m     # enter hold below 4 cm selected formation error
formation_hold_exit_error: 0.08 m      # leave hold above 8 cm selected formation error
formation_hold_min_steps: 2
formation_hold_heading_enabled: true
formation_hold_max_angular_speed: 0.12 rad/s
formation_hold_heading_tolerance_rad: 0.10 rad
```

For the two-robot hardware tests, `formation_hold_metric:=pairwise` is recommended first because it matches the bag-analysis metric `abs(inter_robot_distance - target_pair_distance)`.  Use `formation_hold_metric:=slot` or `both` only when you want to enforce the assigned labeled formation slots and formation orientation more strictly.

The hold state is published on:

```text
/dmpc/two_robot/hold_state
```

where `vector.x = 1.0` means hold mode is active, `vector.y` is the selected hold error, and `vector.z` is the pairwise formation-distance error.

Record a bag before starting the motion-enabled coordinator:

```bash
ros2 bag record -s mcap \
  -o ~/yahboom2wd_ws/bags/two_robot_dmpc_$(date +%Y%m%d_%H%M%S) \
  /robot1/odom \
  /robot2/odom \
  /robot1/cmd_vel \
  /robot2/cmd_vel \
  /dmpc/robot1/pose_world \
  /dmpc/robot2/pose_world \
  /dmpc/robot1/u_world \
  /dmpc/robot2/u_world \
  /dmpc/two_robot/hold_state \
  /robot1/diagnostics \
  /robot2/diagnostics \
  /rosout \
  /tf \
  /tf_static
```

## Conservative first real-robot parameters

The default launch uses:

```text
model             = single_integrator
n_agents          = 2
graph             = complete
M_manual          = 5
u_bound           = 0.08
d_safe            = 0.65 m
formation_margin  = 0.15 m
max_linear_speed  = 0.07 m/s
max_angular_speed = 0.35 rad/s
obstacles_enabled = false
```

These values are commissioning parameters, not final research-tuned parameters. Start without obstacles to validate networking, formation behavior, state feedback, command conversion, and bag recording. Enable obstacle avoidance only after the basic two-robot formation run is repeatable.

## Troubleshooting

### Bash placeholder syntax

Do not type:

```bash
robot1_controller_endpoint:=tcp://<ROBOT1_IP>:5601
```

In Bash, `<ROBOT1_IP>` is interpreted as input redirection. Use the real IP address without angle brackets:

```bash
robot1_controller_endpoint:=tcp://192.168.178.87:5601
```

### `ament_python` rosdep key

If `rosdep` reports:

```text
Cannot locate rosdep definition for [ament_python]
```

but `colcon build` succeeds, run:

```bash
rosdep install --from-paths src -y --ignore-src --rosdistro humble \
  --skip-keys "ament_python"
```

### NumPy / CVXPY / ECOS

If the controller crashes while importing CVXPY or ECOS, reinstall the pinned stack from the Python dependency section.

### ROS 2 launch arguments

`dmpc_controller_node.py` is a plain ZeroMQ/argparse node launched by ROS 2. It must use `parse_known_args()` so that ROS-specific arguments such as `--ros-args -r __node:=...` are ignored.

### `zmq.sleep`

Use Python's standard `time.sleep(0.02)`. Some installed `pyzmq` versions do not provide `zmq.sleep()`.

### Coordinator endpoint parameters

The launch file must pass `robot1_controller_endpoint` and `robot2_controller_endpoint` as separate scalar parameters. Do not construct a ROS parameter array from two `LaunchConfiguration` substitutions, because that can become one concatenated string.

### `self.subscriptions` name collision

Do not assign to `self.subscriptions` inside `dmpc_coordinator_ros_node.py`. `rclpy.node.Node` already has a read-only property with that name. Use `self.odom_subscriptions`.

### Both robots start at `(0, 0)`

This is normal for local odometry but not sufficient for MPC. Always provide measured initial world poses and verify the common world-frame conversion before enabling motion.

Use:

```bash
ros2 topic echo /dmpc/robot1/pose_world
ros2 topic echo /dmpc/robot2/pose_world
```

The `pose_world` topics, not the raw local odometry topics, are the positions used by the coordinator for MPC state construction.

## Git workflow

After a fix is tested on one machine, commit it and pull the same version on every machine:

```bash
cd ~/yahboom2wd_ws/src/yahboom_2wd_dmpc
git status
git diff
git add .
git commit -m "Fix two-robot DMPC hardware interface"
git push
```

Then on `robot1`, `robot2`, and the Ubuntu VM:

```bash
cd ~/yahboom2wd_ws/src/yahboom_2wd_dmpc
git pull

cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select yahboom_2wd_dmpc
source install/setup.bash
```

Keep the Raspberry Pis and the Ubuntu VM on the same package version before each hardware experiment.

## Quantitative formation and safety evaluation

The hardware wrapper now exposes the most important `NetConfig` safety and formation parameters from `consensus_config.py` as ROS 2 launch parameters. This makes the hardware test, the simulation test, and the original distributed MPC configuration traceable to the same quantities.

For the current two-robot commissioning setup, the recommended values are:

```text
n_agents           = 2
d_safe             = 0.65 m
formation_margin   = 0.15 m
formation_rotation = 0.0 rad
d_agent_enter      = 0.70 m
d_agent_exit       = 0.75 m
safety_warning_radius = 1.20 m
```

For `n_agents = 2` and `formation_radius_override = 0.0`, the desired pair distance is:

```text
d_form = d_safe + formation_margin = 0.80 m
```

The corresponding two-agent formation offsets are approximately:

```text
robot1 desired offset = (+0.40, 0.0) m
robot2 desired offset = (-0.40, 0.0) m
```

The safety hysteresis thresholds should be compatible with the target formation. A practical rule for the two-robot setup is:

```text
d_safe < d_agent_enter < d_agent_exit < d_form
```

With the recommended values:

```text
0.65 < 0.70 < 0.75 < 0.80
```

This prevents the safety mode from fighting the desired formation itself. The earlier unexposed `NetConfig` defaults `d_agent_enter = 0.95` and `d_agent_exit = 1.00` are too large for a two-robot target pair distance of `0.80 m`; therefore these values are now explicitly parameterized for the hardware wrapper.

The coordinator publishes additional quantitative debug topics:

```text
/dmpc/two_robot/metrics
/dmpc/two_robot/safety_thresholds
```

`/dmpc/two_robot/metrics` is a `geometry_msgs/msg/Vector3Stamped`:

```text
vector.x = current inter-robot distance [m]
vector.y = desired formation pair distance [m]
vector.z = safety margin = current distance - d_safe [m]
```

`/dmpc/two_robot/safety_thresholds` is also a `Vector3Stamped`:

```text
vector.x = d_safe [m]
vector.y = d_agent_enter [m]
vector.z = d_agent_exit [m]
```

Record these topics in both simulation and hardware bags. They are the easiest way to evaluate whether the robots stay safe and whether the pair distance is converging toward the intended formation distance.

## Parameterized launch example

For a conservative motion-enabled two-robot test:

```bash
ros2 launch yahboom_2wd_dmpc two_robot_dmpc_coordinator.launch.py \
  robot1_controller_endpoint:=tcp://192.168.178.87:5601 \
  robot2_controller_endpoint:=tcp://192.168.178.94:5602 \
  robot1_initial_x:=0.0 \
  robot1_initial_y:=-0.7 \
  robot1_initial_yaw:=0.0 \
  robot2_initial_x:=0.0 \
  robot2_initial_y:=0.7 \
  robot2_initial_yaw:=0.0 \
  max_linear_speed:=0.04 \
  max_angular_speed:=0.20 \
  u_bound:=0.04 \
  d_safe:=0.65 \
  formation_margin:=0.15 \
  safety_warning_radius:=1.20 \
  d_agent_enter:=0.70 \
  d_agent_exit:=0.75 \
  enable_motion:=true
```

Add these topics to the bag recorder:

```text
/dmpc/two_robot/metrics
/dmpc/two_robot/safety_thresholds
```

## Simulation counterpart

Use the companion package:

```text
yahboom_2wd_dmpc_sim
```

It runs on the Ubuntu VM and simulates the same ROS interface as the two real robots:

```text
/robot1/cmd_vel -> simulated robot1 -> /robot1/odom
/robot2/cmd_vel -> simulated robot2 -> /robot2/odom
```

The simulator deliberately publishes local robot odometry starting near `(0, 0, 0)`, so the existing coordinator's world-frame initialization remains part of the test. This lets the same coordinator and controller nodes be used before the hardware run.

Recommended simulation command:

```bash
ros2 launch yahboom_2wd_dmpc_sim two_robot_dmpc_sim.launch.py \
  robot1_initial_x:=0.0 \
  robot1_initial_y:=-0.7 \
  robot1_initial_yaw:=0.0 \
  robot2_initial_x:=0.0 \
  robot2_initial_y:=0.7 \
  robot2_initial_yaw:=0.0 \
  max_linear_speed:=0.04 \
  max_angular_speed:=0.20 \
  u_bound:=0.04 \
  d_safe:=0.65 \
  formation_margin:=0.15 \
  d_agent_enter:=0.70 \
  d_agent_exit:=0.75 \
  enable_motion:=true
```

Then compare the simulated and real bags with:

```bash
ros2 run yahboom_2wd_dmpc_sim analyze_two_robot_bag \
  --bag <bag_folder> \
  --storage sqlite3 \
  --d-safe 0.65 \
  --formation-margin 0.15
```
