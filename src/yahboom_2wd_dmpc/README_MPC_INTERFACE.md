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

## Python environment separation and compatibility checks

The Ubuntu VM intentionally uses two separate Python environments:

```text
ROS 2 Humble and yahboom2wd_ws:
  Python executable: /usr/bin/python3
  Python version:    3.10.12
  ROS distribution: Humble
  colcon executable: /usr/bin/colcon

bahnstrom_ems:
  Python executable: ~/git_repos/bahnstrom_ems/.venv/bin/python
  Python version:    3.12.11
  Environment type:  project-local virtual environment
```

Keep these environments separate. Do not replace `/usr/bin/python3`, do not point Ubuntu's `python3` alternative to Python 3.12, and do not build ROS 2 packages while the `bahnstrom_ems` virtual environment is active.

### Safe workflow for ROS 2 Humble and `yahboom2wd_ws`

Use a fresh terminal or deactivate any active virtual environment:

```bash
deactivate 2>/dev/null || true
hash -r

cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
```

Before a build, source the ROS underlay only. Source `install/setup.bash` after the build succeeds. This avoids keeping deleted package prefixes in `AMENT_PREFIX_PATH`.

Verify the interpreter, ROS distribution, `colcon`, `rclpy`, and `setuptools`:

```bash
echo "VIRTUAL_ENV=$VIRTUAL_ENV"
echo "PYTHONHOME=$PYTHONHOME"
echo "ROS_DISTRO=$ROS_DISTRO"

command -v python3
python3 --version
command -v ros2
command -v colcon
head -n 1 "$(command -v colcon)"

python3 - <<'PY'
import sys
import rclpy
import setuptools

print("Python executable:", sys.executable)
print("Python version:", sys.version)
print("rclpy path:", rclpy.__file__)
print("setuptools version:", setuptools.__version__)
print("setuptools path:", setuptools.__file__)
PY
```

Expected ROS 2 VM baseline:

```text
VIRTUAL_ENV=
PYTHONHOME=
ROS_DISTRO=humble
python3=/usr/bin/python3
Python 3.10.12
colcon=/usr/bin/colcon
colcon shebang=#!/usr/bin/python3
rclpy path under /opt/ros/humble/...
setuptools path=/usr/lib/python3/dist-packages/setuptools/...
```

After sourcing a ROS workspace, a nonempty `PYTHONPATH` is normal. Every entry should belong to ROS 2 Humble or a Python 3.10 workspace. A path containing `python3.12`, `bahnstrom_ems/.venv`, or another virtual environment indicates contamination.

Build with the normal ROS 2 workflow:

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash

rm -rf build/yahboom_2wd_dmpc
rm -rf install/yahboom_2wd_dmpc

colcon build \
  --symlink-install \
  --packages-select yahboom_2wd_dmpc

source install/setup.bash
```

### `setuptools` compatibility diagnostic for `colcon --symlink-install`

A user-local `setuptools 83.0.0` under:

```text
~/.local/lib/python3.10/site-packages
```

caused the error:

```text
error: option --uninstall not recognized
```

The diagnostic build is:

```bash
PYTHONNOUSERSITE=1 colcon build \
  --symlink-install \
  --packages-select yahboom_2wd_dmpc
```

If that build succeeds while the normal build fails, inspect both package locations:

```bash
python3 - <<'PY'
import setuptools
print(setuptools.__version__)
print(setuptools.__file__)
PY

PYTHONNOUSERSITE=1 python3 - <<'PY'
import setuptools
print(setuptools.__version__)
print(setuptools.__file__)
PY
```

The verified compatible Ubuntu package was:

```text
setuptools 59.6.0
/usr/lib/python3/dist-packages/setuptools/
```

Remove only the incompatible user-local copy when needed:

```bash
python3 -m pip uninstall -y setuptools
```

Then verify that normal Python imports the apt-managed copy. Do not set `PYTHONNOUSERSITE=1` globally because the ROS nodes may intentionally use user-local NumPy, CVXPY, ECOS, OSQP, SCS, or PyZMQ packages.

### Safe workflow for `bahnstrom_ems`

Use a separate terminal:

```bash
cd ~/git_repos/bahnstrom_ems
source .venv/bin/activate

echo "VIRTUAL_ENV=$VIRTUAL_ENV"
command -v python
python --version
python -m pip --version
```

Expected:

```text
VIRTUAL_ENV=.../bahnstrom_ems/.venv
python=.../bahnstrom_ems/.venv/bin/python
Python 3.12.11
pip path under .../bahnstrom_ems/.venv/lib/python3.12/site-packages
```

When finished:

```bash
deactivate
hash -r
```

Using separate terminals for ROS 2 and `bahnstrom_ems` is the safest daily workflow.

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

This means the ROS 2 namespaces, ZeroMQ controller communication, common-frame odometry transform, and dry-run safety gate are working. The subsequent motion-enabled and hold-zone-enabled hardware tests have also passed.

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

This conservative motion-enabled workflow has now been validated on the two physical robots. The hold-zone-enabled follow-up test also passed: both robots converged, faced each other, and remained stationary in the safe set.

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

## Verified two-robot hold-zone hardware test

The formation-hold/deadband implementation has passed the first physical two-robot validation.

Observed behavior:

```text
- robot1 and robot2 converged to the configured safe formation
- the pair distance entered the configured formation hold band
- the coordinator suppressed translational motion in hold mode
- the robots corrected their headings until they faced each other
- both robots then remained stationary inside the safe set
- the earlier hunting around the final formation was no longer visually apparent
```

The validated conservative hardware settings were:

```text
max_linear_speed = 0.03 m/s
max_angular_speed = 0.15 rad/s
u_bound = 0.03

d_safe = 0.65 m
formation_margin = 0.15 m
d_agent_enter = 0.68 m
d_agent_exit = 0.72 m

formation_hold_enabled = true
formation_hold_metric = pairwise
formation_hold_enter_error = 0.04 m
formation_hold_exit_error = 0.08 m
formation_hold_min_steps = 2
formation_hold_heading_enabled = true
formation_hold_heading_gain = 1.0
formation_hold_max_angular_speed = 0.12 rad/s
formation_hold_heading_tolerance_rad = 0.10 rad
```

The test validates the practical hardware behavior of the hold layer: formation convergence, hysteretic entry into hold mode, zero translational command inside the hold zone, heading-only correction, and stationary formation keeping. Keep `/dmpc/two_robot/hold_state` in future bags so the activation and release of hold mode remain traceable.

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

These values are commissioning parameters, not final research-tuned parameters. The obstacle-free formation and hold-zone hardware tests are now repeatable and safe. The parameterized one-obstacle scenario has also passed in simulation. The next stage is a hardware obstacle dry run with `enable_motion:=false`, followed by a short 25-second reduced-speed hardware obstacle test.

## Obstacle-avoidance parameterization update

This package version exposes the circular-obstacle geometry and compact small-field avoidance parameters directly through the ROS 2 launch files. This is required before enabling `obstacles_enabled:=true` on the 2 m x 3 m hardware field, because the original pure-Python defaults used a much larger hard-coded obstacle.

The following parameters are now available in both `robot_dmpc_controller.launch.py` and `two_robot_dmpc_coordinator.launch.py`:

```text
obstacles_enabled
obstacle_center_x
obstacle_center_y
obstacle_radius
obstacle_margin
obstacle_warning_radius
d_obs_enter
d_obs_exit
tangential_waypoint_radius
orbit_tangent_lookahead
```

Meaning:

```text
obstacle_center_x, obstacle_center_y  -> obstacle center in the shared map/world frame [m]
obstacle_radius                       -> physical radius of the obstacle [m]
obstacle_margin                       -> safety inflation margin [m]
inflated radius                       -> obstacle_radius + obstacle_margin
d_obs_enter, d_obs_exit               -> clearance hysteresis from the inflated obstacle boundary [m]
obstacle_warning_radius               -> clearance range where obstacle diagnostics/guidance become relevant [m]
tangential_waypoint_radius            -> compact offset around the inflated obstacle used by the tangent waypoint
orbit_tangent_lookahead              -> tangential lookahead used by the obstacle detour heuristic
```

For the first small-field test, use the validated obstacle-activation values:

```text
obstacle_center_x: 1.0
obstacle_center_y: -0.33
obstacle_radius: 0.15
obstacle_margin: 0.10
obstacle_warning_radius: 0.35
d_obs_enter: 0.15
d_obs_exit: 0.25
tangential_waypoint_radius: 0.12
orbit_tangent_lookahead: 0.20
```

The coordinator now publishes additional obstacle diagnostics:

```text
/dmpc/robot1/obstacle_metrics
/dmpc/robot2/obstacle_metrics
/dmpc/two_robot/obstacle_thresholds
```

`/dmpc/robotX/obstacle_metrics` is a `geometry_msgs/msg/Vector3Stamped`:

```text
vector.x = robot-center to obstacle-center distance [m]
vector.y = clearance from inflated obstacle boundary [m]
vector.z = 1.0 if clearance <= d_obs_enter, otherwise 0.0
```

`/dmpc/two_robot/obstacle_thresholds` is a `geometry_msgs/msg/Vector3Stamped`:

```text
vector.x = d_obs_enter [m]
vector.y = d_obs_exit [m]
vector.z = obstacle_warning_radius [m]
```

The formation hold layer now respects obstacle priority. Hold mode is allowed only when the formation error is inside the hold band and all robots have obstacle clearance at least `d_obs_exit`. If a robot is too close to the inflated obstacle, obstacle avoidance has priority and hold is released or blocked.

Recommended first obstacle simulation:

```bash
timeout --signal=SIGINT --kill-after=10s 90s \
ros2 launch yahboom_2wd_dmpc_sim two_robot_dmpc_sim.launch.py \
  robot1_initial_x:=1.0 \
  robot1_initial_y:=-0.7 \
  robot1_initial_yaw:=0.0 \
  robot2_initial_x:=1.0 \
  robot2_initial_y:=0.7 \
  robot2_initial_yaw:=0.0 \
  max_linear_speed:=0.02 \
  max_angular_speed:=0.12 \
  u_bound:=0.02 \
  d_safe:=0.65 \
  formation_margin:=0.15 \
  d_agent_enter:=0.68 \
  d_agent_exit:=0.72 \
  obstacles_enabled:=true \
  obstacle_center_x:=1.0 \
  obstacle_center_y:=-0.33 \
  obstacle_radius:=0.15 \
  obstacle_margin:=0.10 \
  d_obs_enter:=0.15 \
  d_obs_exit:=0.25 \
  obstacle_warning_radius:=0.35 \
  tangential_waypoint_radius:=0.12 \
  orbit_tangent_lookahead:=0.20 \
  formation_hold_enabled:=true \
  enable_motion:=true
```

Record these additional obstacle topics in both simulation and hardware bags:

```text
/dmpc/robot1/obstacle_metrics
/dmpc/robot2/obstacle_metrics
/dmpc/two_robot/obstacle_thresholds
/dmpc/two_robot/hold_state
```

The obstacle test passes only if the minimum inflated-obstacle clearance remains positive, the inter-robot distance remains above `d_safe`, the robots converge toward the safe formation, and hold mode becomes active only after obstacle clearance is outside the exit band.

## Verified obstacle-avoidance activation simulation test

The real obstacle-avoidance **simulation** test has passed. This is the first run where obstacle monitoring was enabled and the obstacle-avoidance logic actually became active.

Validated bag:

```text
/home/navid/yahboom2wd_ws/bags/two_robot_dmpc_motion_20260712_032738
```

Validated configuration:

```text
robot1 initial pose: x=1.0 m, y=-0.7 m, yaw=0.0 rad
robot2 initial pose: x=1.0 m, y=+0.7 m, yaw=0.0 rad

max_linear_speed  = 0.02 m/s
max_angular_speed = 0.12 rad/s
u_bound           = 0.02

d_safe            = 0.65 m
formation_margin  = 0.15 m
d_agent_enter     = 0.68 m
d_agent_exit      = 0.72 m

obstacles_enabled         = true
obstacle_center_x         = 1.0 m
obstacle_center_y         = -0.33 m
obstacle_radius           = 0.15 m
obstacle_margin           = 0.10 m
inflated obstacle radius  = 0.25 m

d_obs_enter              = 0.15 m
d_obs_exit               = 0.25 m
obstacle_warning_radius  = 0.35 m
tangential_waypoint_radius = 0.12 m
orbit_tangent_lookahead    = 0.20 m

formation_hold_enabled = true
```

Quantitative result:

```text
target pair distance:                    0.8000 m
initial inter-robot distance:             1.4000 m
final inter-robot distance:               0.7623 m
final absolute formation-distance error:  0.0377 m
minimum inter-robot distance:             0.7091 m
minimum safety margin to d_safe:          0.0591 m

minimum inflated-obstacle clearance:      0.1200 m
robot1 obstacle-active samples:           24 / 594
robot2 obstacle-active samples:           15 / 594
hold active samples:                       8 / 594
```

This passes the real obstacle-avoidance simulation gate because:

```text
minimum inflated-obstacle clearance > 0.0 m
minimum inter-robot distance > d_safe
obstacle-active samples > 0 for at least one robot
final formation-distance error < 0.05 m
hold mode is present and does not dominate while obstacle activity is occurring
```

The final distance is slightly below the nominal `0.80 m` target, but the absolute error is about `3.8 cm`, which is acceptable for the simulation gate. The minimum inter-robot distance remained safely above `d_safe` with about `5.9 cm` margin. The obstacle clearance stayed positive with about `12 cm` minimum clearance from the inflated obstacle boundary.

This simulation result is the evidence required before moving to hardware obstacle testing. The next stage is a hardware dry run with `enable_motion:=false`, followed by a short reduced-speed physical obstacle test.

## Next exact hardware obstacle-validation step

Do not start with a 60-second physical obstacle run. Use the following sequence.

### 1. Prepare the field

Use the same coordinate convention as the passed simulation:

```text
field length: x = 0.0 m to 3.0 m
field width:  y = -1.0 m to +1.0 m

robot1 start:    x=1.0 m, y=-0.7 m, yaw=0.0 rad
robot2 start:    x=1.0 m, y=+0.7 m, yaw=0.0 rad
obstacle center: x=1.0 m, y=-0.33 m
obstacle radius: 0.15 m
```

Use a soft, lightweight obstacle first. The physical radius should be approximately `0.15 m`; do not use the maximum `0.40 m` obstacle yet.

### 2. Start the Yahboom bridge on both robots

Use the already calibrated bridge commands for `robot1` and `robot2`. Keep the same `ROS_DOMAIN_ID=42` on every terminal.

### 3. Start the robot-side DMPC controllers with the same obstacle parameters

On `robot1`:

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch yahboom_2wd_dmpc robot_dmpc_controller.launch.py \
  agent_id:=1 \
  u_bound:=0.02 \
  d_safe:=0.65 \
  formation_margin:=0.15 \
  d_agent_enter:=0.68 \
  d_agent_exit:=0.72 \
  obstacles_enabled:=true \
  obstacle_center_x:=1.0 \
  obstacle_center_y:=-0.33 \
  obstacle_radius:=0.15 \
  obstacle_margin:=0.10 \
  d_obs_enter:=0.15 \
  d_obs_exit:=0.25 \
  obstacle_warning_radius:=0.35 \
  tangential_waypoint_radius:=0.12 \
  orbit_tangent_lookahead:=0.20
```

On `robot2`, use the same command with:

```text
agent_id:=2
```

### 4. Run a hardware dry run from the Ubuntu VM

This verifies world poses, obstacle metrics, ZeroMQ communication, and zero physical command before any robot moves.

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

timeout --signal=SIGINT --kill-after=10s 15s \
ros2 launch yahboom_2wd_dmpc two_robot_dmpc_coordinator.launch.py \
  robot1_controller_endpoint:=tcp://192.168.178.87:5601 \
  robot2_controller_endpoint:=tcp://192.168.178.94:5602 \
  robot1_initial_x:=1.0 \
  robot1_initial_y:=-0.7 \
  robot1_initial_yaw:=0.0 \
  robot2_initial_x:=1.0 \
  robot2_initial_y:=0.7 \
  robot2_initial_yaw:=0.0 \
  max_linear_speed:=0.02 \
  max_angular_speed:=0.12 \
  u_bound:=0.02 \
  d_safe:=0.65 \
  formation_margin:=0.15 \
  d_agent_enter:=0.68 \
  d_agent_exit:=0.72 \
  obstacles_enabled:=true \
  obstacle_center_x:=1.0 \
  obstacle_center_y:=-0.33 \
  obstacle_radius:=0.15 \
  obstacle_margin:=0.10 \
  d_obs_enter:=0.15 \
  d_obs_exit:=0.25 \
  obstacle_warning_radius:=0.35 \
  tangential_waypoint_radius:=0.12 \
  orbit_tangent_lookahead:=0.20 \
  formation_hold_enabled:=true \
  enable_motion:=false
```

In another VM terminal, check:

```bash
ros2 topic echo /dmpc/robot1/pose_world
ros2 topic echo /dmpc/robot2/pose_world
ros2 topic echo /dmpc/robot1/obstacle_metrics
ros2 topic echo /dmpc/robot2/obstacle_metrics
ros2 topic echo /robot1/cmd_vel
ros2 topic echo /robot2/cmd_vel
```

Pass criteria for the dry run:

```text
robot1 pose_world is close to (1.0, -0.7)
robot2 pose_world is close to (1.0, +0.7)
robot1 inflated-obstacle clearance is close to 0.12 m
robot2 inflated-obstacle clearance is close to 0.78 m
both cmd_vel topics remain zero
```

### 5. Record a hardware obstacle bag

Start the bag before the motion-enabled coordinator:

```bash
mkdir -p ~/yahboom2wd_ws/bags

ros2 bag record -s sqlite3 \
  -o ~/yahboom2wd_ws/bags/two_robot_obstacle_hw_$(date +%Y%m%d_%H%M%S) \
  /robot1/odom \
  /robot2/odom \
  /robot1/cmd_vel \
  /robot2/cmd_vel \
  /dmpc/robot1/pose_world \
  /dmpc/robot2/pose_world \
  /dmpc/robot1/u_world \
  /dmpc/robot2/u_world \
  /dmpc/robot1/obstacle_metrics \
  /dmpc/robot2/obstacle_metrics \
  /dmpc/two_robot/metrics \
  /dmpc/two_robot/safety_thresholds \
  /dmpc/two_robot/obstacle_thresholds \
  /dmpc/two_robot/hold_state \
  /robot1/diagnostics \
  /robot2/diagnostics \
  /rosout \
  /tf \
  /tf_static
```

### 6. Run the first physical obstacle test for 25 seconds

```bash
timeout --signal=SIGINT --kill-after=10s 25s \
ros2 launch yahboom_2wd_dmpc two_robot_dmpc_coordinator.launch.py \
  robot1_controller_endpoint:=tcp://192.168.178.87:5601 \
  robot2_controller_endpoint:=tcp://192.168.178.94:5602 \
  robot1_initial_x:=1.0 \
  robot1_initial_y:=-0.7 \
  robot1_initial_yaw:=0.0 \
  robot2_initial_x:=1.0 \
  robot2_initial_y:=0.7 \
  robot2_initial_yaw:=0.0 \
  max_linear_speed:=0.02 \
  max_angular_speed:=0.12 \
  u_bound:=0.02 \
  d_safe:=0.65 \
  formation_margin:=0.15 \
  d_agent_enter:=0.68 \
  d_agent_exit:=0.72 \
  obstacles_enabled:=true \
  obstacle_center_x:=1.0 \
  obstacle_center_y:=-0.33 \
  obstacle_radius:=0.15 \
  obstacle_margin:=0.10 \
  d_obs_enter:=0.15 \
  d_obs_exit:=0.25 \
  obstacle_warning_radius:=0.35 \
  tangential_waypoint_radius:=0.12 \
  orbit_tangent_lookahead:=0.20 \
  formation_hold_enabled:=true \
  enable_motion:=true
```

This first hardware run only needs to validate safe avoidance behavior. It does not need to fully settle into hold.

Pass criteria for the 25-second run:

```text
minimum inflated-obstacle clearance > 0.0 m
minimum inter-robot distance > 0.65 m
at least one robot has obstacle-active samples > 0
both robots stay inside the 2 m x 3 m field
no abrupt full-speed turn or repeated stop/start chattering appears
both robots stop when the coordinator exits
```

### 7. Analyze before extending the duration

```bash
LATEST_BAG=$(ls -td ~/yahboom2wd_ws/bags/two_robot_obstacle_hw_* | head -1)

ros2 run yahboom_2wd_dmpc_sim analyze_two_robot_bag \
  --bag "$LATEST_BAG" \
  --storage sqlite3 \
  --d-safe 0.65 \
  --formation-margin 0.15

cat "$LATEST_BAG/two_robot_dmpc_analysis.txt"

python3 ~/yahboom2wd_ws/tools/plot_yahboom_team_bag.py \
  --bag "$LATEST_BAG" \
  --namespaces robot1 robot2 \
  --storage-id auto \
  --d-safe 0.65 \
  --formation-margin 0.15 \
  --d-agent-enter 0.68 \
  --d-agent-exit 0.72 \
  --formation-hold-enter-error 0.04 \
  --formation-hold-exit-error 0.08 \
  --show-obstacle always \
  --obstacle-center-x 1.0 \
  --obstacle-center-y -0.33 \
  --obstacle-radius 0.15 \
  --obstacle-margin 0.10
```

Only after the 25-second hardware run passes should the same setup be repeated for `60s`.

## Troubleshooting

### Python 3.12 virtual environment versus ROS 2 Python 3.10

The project-local `bahnstrom_ems/.venv` using Python 3.12.11 is safe when it remains activated only in its own terminal. ROS 2 Humble must continue to use `/usr/bin/python3` version 3.10.12.

If `colcon build --symlink-install` reports:

```text
error: option --uninstall not recognized
```

check the imported `setuptools`:

```bash
python3 - <<'PY'
import sys
import setuptools
print(sys.executable)
print(setuptools.__version__)
print(setuptools.__file__)
PY
```

A user-local `setuptools 83.0.0` under `~/.local/lib/python3.10/site-packages` was incompatible with the Humble symlink-install workflow. Confirm with:

```bash
PYTHONNOUSERSITE=1 colcon build \
  --symlink-install \
  --packages-select yahboom_2wd_dmpc
```

If that succeeds, remove only the user-local override:

```bash
python3 -m pip uninstall -y setuptools
```

The expected apt-managed fallback is `setuptools 59.6.0` under `/usr/lib/python3/dist-packages`.

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

For the next obstacle-avoidance stage, use the same parameterized obstacle geometry, obstacle clearances, and tangent-waypoint settings in hardware that passed in simulation. Do not compare simulation and hardware unless those values are identical.
