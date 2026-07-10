# Yahboom 2WD ROS 2 interface for distributed MPC prototypes

This workspace contains a minimal ROS 2 interface for a custom 2WD Yahboom robot using:

- Yahboom lightweight aluminum alloy 2WD chassis
- L-type 520 encoder DC motors
- Yahboom ROS Robot Control Board V3.0 with STM32F103RCT6-compatible MCU
- Rosmaster Python driver library
- Raspberry Pi 4 running Ubuntu 22.04 and ROS 2 Humble

The design goal is to keep the factory STM32 firmware untouched and run only a small ROS 2 bridge on the Raspberry Pi.

## Current calibrated hardware parameters

For the assembled 2WD Yahboom prototype, use the measured values:

```text
wheel_radius     = 0.0325 m
wheel_separation = 0.120 m
```

The current calibrated launch parameters for `yahboom1` are:

```text
linear_cmd_scale  = 1.7
angular_cmd_scale = 1.0
odom_linear_scale = 1.5
```

Interpretation:

```text
linear_cmd_scale  -> command-side calibration: scales cmd_vel.linear.x before sending it to the Yahboom board
angular_cmd_scale -> command-side calibration: scales cmd_vel.angular.z before sending it to the Yahboom board
odom_linear_scale -> odometry-side calibration: scales board-reported linear velocity before integrating /robot1/odom
```

The open-loop straight-line velocity command was empirically calibrated on `yahboom1`:

```bash
timeout 10s ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10
```

The command publishes `0.1 m/s` at 10 Hz for 10 seconds, so the expected nominal distance is:

```text
0.1 m/s * 10 s = 1.0 m
```

With `linear_cmd_scale = 1.7`, the open-loop command moves the robot approximately 1 m in the x direction.

During the first odometry-feedback `straight` test, `/robot1/odom` reported approximately 1.0 m while the physical tape measurement was approximately 1.5 m. This showed that the board-reported linear odometry was under-scaled. The bridge was therefore extended with:

```text
odom_linear_scale = 1.5
```

After adding `odom_linear_scale = 1.5`, the feedback `straight` test with a 1.0 m reference produced approximately 1.0 m physical travel. Keep this value for `yahboom1` unless repeated 1 m tests show a systematic physical error larger than about 3-5 cm.

## Packages

```text
yahboom2wd_ws/
├── README.md
├── src/
│   ├── yahboom_2wd_driver/
│   │   ├── yahboom_2wd_node.py        # cmd_vel <-> Rosmaster_Lib serial bridge
│   │   ├── serial_probe.py            # checks serial link, firmware version, sensors
│   │   ├── motor_test.py              # low-speed M2/M4 PWM test while robot is lifted
│   │   ├── vendor/
│   │   │   └── Rosmaster_Lib.py       # fallback copy of Yahboom driver library V3.3.9
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   ├── yahboom_2wd_description/
│   │   ├── urdf/
│   │   │   └── yahboom_2wd.urdf.xacro
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   ├── yahboom_2wd_bringup/
│   │   ├── launch/
│   │   │   └── yahboom_2wd.launch.py
│   │   ├── config/
│   │   │   └── yahboom_2wd.yaml
│   │   ├── scripts/
│   │   │   └── install_udev_yahboom.sh
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   ├── yahboom_2wd_tests/
│   │   ├── yahboom_2wd_tests/
│   │   │   └── path_follower_node.py  # odometry-feedback path-following tests
│   │   ├── launch/
│   │   │   └── path_follower.launch.py
│   │   ├── config/
│   │   │   └── path_tests.yaml
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   └── yahboom_2wd_dmpc/
│       ├── yahboom_2wd_dmpc/
│       │   ├── consensus_comm.py          # JSON envelope helpers for ZeroMQ messages
│       │   ├── consensus_config.py        # NetConfig and common DMPC parameters
│       │   ├── consensus_controller.py    # single-/double-integrator MPC solvers
│       │   ├── explicit_hybrid_controller.py
│       │   ├── dmpc_controller_node.py    # robot-side ZMQ REP controller node
│       │   ├── dmpc_coordinator_ros_node.py # VM-side ROS/ZMQ coordinator
│       │   ├── config_utils.py
│       │   ├── controller_node.py         # original/SIL-compatible controller entry point
│       │   ├── coordinator_node.py        # original/SIL-compatible coordinator entry point
│       │   └── __init__.py
│       ├── launch/
│       │   ├── robot_dmpc_controller.launch.py
│       │   └── two_robot_dmpc_coordinator.launch.py
│       ├── config/
│       │   └── two_robot_dmpc.yaml
│       ├── README_MPC_INTERFACE.md
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
└── tools/
    └── plot_yahboom_bag.py
```

The package `yahboom_2wd_dmpc` is the current ROS 2 / ZeroMQ interface layer between the commissioned Yahboom robots and the distributed MPC planning app. It does not replace the low-level Yahboom bridge. It uses `/robotX/odom` as feedback, publishes `/robotX/cmd_vel`, and communicates with robot-side MPC controller nodes through ZeroMQ.

## ROS API per robot namespace

For namespace `robot1`:

Subscribed:

- `/robot1/cmd_vel` (`geometry_msgs/msg/Twist`)

Published:

- `/robot1/odom` (`nav_msgs/msg/Odometry`)
- `/robot1/imu/data` (`sensor_msgs/msg/Imu`)
- `/robot1/battery_state` (`sensor_msgs/msg/BatteryState`)
- `/robot1/encoder_ticks` (`std_msgs/msg/Int32MultiArray`, order `[M1, M2, M3, M4]`)
- `/robot1/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)
- `/tf`
- `/tf_static`

Dynamic TF:

```text
robot1/odom -> robot1/base_footprint
```

The URDF/Xacro and `robot_state_publisher` provide the static internal robot frames such as:

```text
robot1/base_footprint -> robot1/base_link
robot1/base_link      -> robot1/imu_link
robot1/base_link      -> robot1/left_wheel_link
robot1/base_link      -> robot1/right_wheel_link
```


## Odometry-feedback path-following test package

The package `yahboom_2wd_tests` is intended for single-robot validation before moving to the distributed MPC planner. It plays the role of a simple high-level controller: it reads the measured robot pose from odometry, compares it with a reference path, and publishes corrected `Twist` commands to the same `/robot1/cmd_vel` interface that the future MPC planner will use.

This package does not replace the Yahboom STM32 firmware and does not change the low-level motor-control interface. It sits above the existing `yahboom_2wd_driver` bridge.

For namespace `robot1`, the path follower uses:

Subscribed:

- `/robot1/odom` (`nav_msgs/msg/Odometry`)

Published:

- `/robot1/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/robot1/path_test/reference_pose` (`geometry_msgs/msg/PoseStamped`)
- `/robot1/path_test/tracking_error` (`geometry_msgs/msg/Vector3Stamped`)

The `tracking_error` topic is intended for quick plotting and bag inspection:

```text
tracking_error.x = longitudinal or path-progress error [m]
tracking_error.y = lateral/cross-track error [m]
tracking_error.z = heading error [rad]
```

## Path-following test objectives

The standard tests should answer these questions before the distributed MPC planner is connected:

1. Can the robot receive high-level `Twist` commands reliably through `/robot1/cmd_vel`?
2. Does the calibrated Yahboom bridge convert the commanded velocity into approximately correct physical motion?
3. Does `/robot1/odom` provide a useful feedback signal for correcting path deviations?
4. Are the signs of forward velocity, yaw rate, and odometry yaw consistent?
5. Does the robot behave safely during start, stop, and changing-curvature commands?
6. Are the left and right turns reasonably symmetric?
7. Are the recorded bags sufficient to compare commanded trajectory, measured odometry, encoder ticks, and IMU yaw rate?

The path-following tests are a bridge between simple open-loop `cmd_vel` checks and the future distributed MPC planning algorithm.

## Path-following test strategy

Use the following development order:

1. **Open-loop bridge verification**: run the calibrated straight-line `cmd_vel` test and confirm approximately 1 m travel for the 10 s command.
2. **Feedback straight-line tracking**: run the `straight` scenario with odometry feedback and check that lateral and heading errors remain small.
3. **Yaw and angular-command validation**: run `pure_rotation` and verify yaw direction, yaw-rate sign, final rotation angle, and automatic stopping.
4. **Constant-curvature validation**: run `arc` and `circle` to validate simultaneous `linear.x` and `angular.z` commands.
5. **Command-transient validation**: run `stop_and_go` to check stopping, watchdog behavior, and repeated starts.
6. **Changing-curvature validation**: run `sinusoidal` to test smooth left-right curvature transitions.
7. **Bag-based analysis**: record every important test and inspect `/robot1/cmd_vel`, `/robot1/odom`, `/robot1/path_test/reference_pose`, `/robot1/path_test/tracking_error`, encoders, IMU, and diagnostics.
8. **MPC readiness decision**: only move to distributed MPC after the single robot can follow the standard references with predictable errors.

The first feedback test was the **straight** scenario. It separated three issues: command delivery, linear command calibration, and odometry-feedback scaling.

The second feedback test was the **pure_rotation** scenario. It separated yaw sign, angular command behavior, and stopping logic.

Supported scenarios in `yahboom_2wd_tests`:

```text
straight
pure_rotation
arc
circle
stop_and_go
sinusoidal
```

### Development observations so far

- The robot did not receive `/robot1/cmd_vel` when Terminal 2 forgot `export ROS_DOMAIN_ID=42`. All terminals must use the same DDS domain.
- The open-loop straight test showed that `linear_cmd_scale = 1.7` is a good command-side calibration for `yahboom1`.
- The first feedback straight test physically traveled about 1.5 m for a 1.0 m reference, while the plots showed approximately 1.0 m odometry travel. This identified an odometry scaling mismatch.
- Adding `odom_linear_scale = 1.5` to the Yahboom bridge corrected the feedback straight test to approximately 1.0 m physical travel.
- The first pure-rotation test reached nearly 90 degrees but did not stop automatically. The reason was that the generic full-pose finish condition required small x/y error as well as small yaw error.
- The `pure_rotation` logic was fixed so that it uses yaw-only completion, commands `linear.x = 0.0`, and ignores small x/y odometry drift for stopping.
- The refined pure-rotation settings `angular_speed = 0.12`, `max_angular_speed = 0.25`, and `goal_tolerance_yaw = 0.025` produced a slower and more accurate approximately 90-degree turn.
- The left **constant arc** test was run after `straight` and `pure_rotation`. It turned left, stopped automatically, reached approximately 90 degrees, and ended close to the expected quarter-circle endpoint. The observed endpoint was within roughly 10 cm and the trajectory shape was smooth, so the left arc test is considered passed for commissioning.
- The right **constant arc** test looked similar to the left arc test, but with opposite rotation direction. The trajectory was smooth and symmetric enough for commissioning, so the right arc test is considered passed.
- The left and right **circle** tests both completed smooth full-circle trajectories. The final position error was roughly 15-20 cm, mainly visible as a y-axis offset. This is acceptable for first-run sustained-curvature commissioning and the circle test is considered passed with observation.
- The repeated y-axis offset in the circle tests is most likely accumulated cross-track/yaw/curvature error over a long maneuver, not a reason to split global x/y odometry scaling. Keep `odom_linear_scale = 1.5` for now because it was calibrated by the straight test.
- The first stop-and-go run revealed a software issue in `ref_stop_and_go()`: after the third motion segment, the reference jumped from about `0.54 m` to about `0.72 m`, which added one unintended extra move segment. The reference generator was fixed so that the completed cycles are clamped at the configured number of cycles.
- After fixing `ref_stop_and_go()`, the repeated stop-and-go test produced a final odometry displacement of about `0.57 m`, with `dy` close to zero, smooth straight motion, and two visible intermediate stop phases. The stop-and-go test is therefore considered passed for commissioning.
- The **sinusoidal** path test was run after the straight, pure-rotation, arc, circle, and stop-and-go tests. The robot traveled approximately 2 m, followed a smooth low-amplitude S-shaped path with small fluctuations around the reference, and stopped automatically. The final odometry was about `dx = 2.03 m`, `dy = -0.02 m`, so the sinusoidal test is considered passed for commissioning.
- With the sinusoidal test passed, the standard single-robot feedback validation suite for `robot1` is complete. The next hardware step is to assemble `robot2`, repeat the same calibration and standard tests for `robot2`, and identify its own calibration values before running any multi-robot MPC experiment.
- The distributed MPC development strategy is staged: first validate the complete software/ROS/network/MPC pipeline with two robots (`robot1` and `robot2`), then order and commission `robot3` and `robot4` only after the two-robot distributed MPC test is stable. This reduces cost and risk before scaling to the intended four-robot setup.

- `robot2` has now also been assembled, calibrated, and validated with the same standard single-robot tests. Therefore, the next active stage is the two-robot distributed MPC hardware dry run.
- A new package, `yahboom_2wd_dmpc`, was added to connect the commissioned Yahboom ROS 2 interfaces to the existing distributed MPC planning app. The package uses a star topology: each robot runs a local ZeroMQ REP controller node, and the Ubuntu VM runs the ROS/ZMQ coordinator.
- During first deployment of `yahboom_2wd_dmpc`, the Raspberry Pis required a consistent Python optimization stack. The broken combination was user-local `numpy` 2.x with solver wheels compiled against NumPy 1.x. The working Humble/RPi setup pins NumPy below 2 and reinstalls CVXPY/ECOS/SCS/OSQP consistently.
- `rosdep` may report that it cannot resolve the key `ament_python` on the Raspberry Pis even though the packages build successfully. For this workspace, run `rosdep install` with `--skip-keys "ament_python"` when needed.
- The robot-side `dmpc_controller_node.py` was corrected to use `parse_known_args()` so that ROS 2 launch arguments such as `--ros-args -r __node:=...` do not crash the plain ZeroMQ/argparse node.
- The robot-side `dmpc_controller_node.py` was also corrected to use `time.sleep(0.02)` instead of `zmq.sleep(0.02)`, because the installed `pyzmq` version on the Raspberry Pis does not provide `zmq.sleep()`.
- The VM-side coordinator launch was corrected so that `robot1_controller_endpoint` and `robot2_controller_endpoint` are passed as separate scalar parameters. Passing a list of `LaunchConfiguration` objects created one concatenated string instead of a ROS 2 string array.
- The VM-side `dmpc_coordinator_ros_node.py` was corrected to avoid assigning to `self.subscriptions`, because `rclpy.node.Node` already has a read-only property with that name. The coordinator now stores odometry subscriptions in `self.odom_subscriptions`.
- The two robot odometry topics naturally start in separate local frames, usually around `(0, 0, 0)` for each robot. This is normal, but the distributed MPC must not use the raw local odometries directly. The coordinator must transform each local odometry into a shared world/map frame using measured initial poses before building `r_all` for the MPC.
- The dry-run common-frame verification has passed. With measured initial poses `robot1_initial_y = -0.7` and `robot2_initial_y = +0.7`, the coordinator published `/dmpc/robot1/pose_world` at `(0.0, -0.7)` and `/dmpc/robot2/pose_world` at `(0.0, +0.7)` in frame `map`, while `/robot1/cmd_vel` and `/robot2/cmd_vel` remained zero because `enable_motion = false`.
- The debug topics `/dmpc/robot1/pose_world` and `/dmpc/robot2/pose_world` are now the recommended way to verify that the coordinator is using the intended common world-frame positions before enabling physical motion.

## Role of `ROS_DOMAIN_ID`

ROS 2 uses DDS for discovery and communication. `ROS_DOMAIN_ID` separates ROS 2 systems on the same network into independent communication domains.

All terminals and machines that should communicate must use the same value. In this project we use:

```bash
export ROS_DOMAIN_ID=42
```

This must be set in every terminal used for the Yahboom experiment:

- Terminal 1: robot launch
- Terminal 2: `cmd_vel` publishing
- Terminal 3: topic echo, bag recording, plotting, or diagnostics
- Ubuntu VM terminal running the distributed MPC planner

If Terminal 1 launches the robot with `ROS_DOMAIN_ID=42`, but Terminal 2 publishes `/robot1/cmd_vel` without setting `ROS_DOMAIN_ID=42`, the command publisher and the robot node are in different DDS domains. The message will not reach the robot, even though the topic name is correct.

A good quick check is:

```bash
echo $ROS_DOMAIN_ID
ros2 node list
ros2 topic info -v /robot1/cmd_vel
```

`ros2 topic info -v /robot1/cmd_vel` should show one publisher during the command test and one subscriber from the Yahboom driver node.

To avoid forgetting the variable, add this to `~/.bashrc` on each Raspberry Pi and on the Ubuntu VM used for the experiment:

```bash
export ROS_DOMAIN_ID=42
```

Then reload:

```bash
source ~/.bashrc
```

## Command modes

### `motion` mode, recommended first

The driver calls:

```python
bot.set_car_motion(vx, 0.0, wz)
```

This keeps Yahboom's factory motion-control path active. Use this first because the Yahboom driver library documents `set_pid_param()` as affecting `set_car_motion()`.

In `motion` mode, the bridge sends:

```text
vx_board = linear_cmd_scale  * cmd_vel.linear.x
wz_board = angular_cmd_scale * cmd_vel.angular.z
```

The current calibrated command-side values for `yahboom1` are:

```text
linear_cmd_scale  = 1.7
angular_cmd_scale = 1.0
```

The current odometry-side value is configured separately in the bridge:

```text
odom_linear_scale = 1.5
```

### `pwm_diff` mode, commissioning fallback

The driver computes differential wheel commands:

```text
v_left  = vx - wheel_separation/2 * wz
v_right = vx + wheel_separation/2 * wz
```

Then it maps the wheel speeds to `set_motor(M1, M2, M3, M4)` PWM percentages. The Yahboom driver library comments that `set_motor()` is PWM control and does not use encoder speed feedback. Use this mode only for M2/M4 sign and wiring tests unless you later confirm that your firmware exposes a true per-motor speed command.

## One-time installation on each Raspberry Pi

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-rosbag2-storage-mcap \
  python3-serial \
  python3-zmq \
  python3-numpy \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-matplotlib \
  python3-pip \
  git

sudo rosdep init 2>/dev/null || true
rosdep update

mkdir -p ~/yahboom2wd_ws/src
# Copy or clone these packages into ~/yahboom2wd_ws/src, including:
#   yahboom_2wd_driver
#   yahboom_2wd_description
#   yahboom_2wd_bringup
#   yahboom_2wd_tests
#   yahboom_2wd_dmpc

cd ~/yahboom2wd_ws

# On some Raspberry Pi / ROS 2 Humble installations, rosdep cannot resolve
# ament_python although the build system is already installed. In that case,
# skip that key explicitly.
rosdep install --from-paths src -y --ignore-src --rosdistro humble \
  --skip-keys "ament_python"

colcon build --symlink-install

source /opt/ros/humble/setup.bash
source install/setup.bash
```

For the distributed MPC package on the Raspberry Pis, keep the user-local Python optimization stack consistent. The tested fix after a NumPy/CVXPY/ECOS import conflict was:

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

The important point is that `numpy` must be `1.x` on the Raspberry Pis for the current solver wheels. A mixed environment with NumPy `2.x` and old compiled solver wheels causes `_ARRAY_API not found` or `numpy.core.multiarray failed to import`.

Install the udev alias used by Yahboom tutorials:

```bash
cd ~/yahboom2wd_ws
sudo bash src/yahboom_2wd_bringup/scripts/install_udev_yahboom.sh
newgrp dialout
ls -l /dev/myserial
```

## Hardware check on one robot

Keep the robot wheels lifted from the table.

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 run yahboom_2wd_driver yahboom_serial_probe \
  --serial-port /dev/myserial \
  --car-type 4
```

Expected successful output includes:

```text
Rosmaster Serial Opened! Baudrate=115200
MCU version: ...
Battery voltage: ...
Motion vx, vy, wz: ...
IMU roll, pitch, yaw [rad]: ...
Encoder M1..M4 ticks: ...
Probe complete.
```

Then test the M2/M4 wiring gently, still with the robot lifted:

```bash
ros2 run yahboom_2wd_driver yahboom_motor_test \
  --serial-port /dev/myserial \
  --left-port 2 \
  --right-port 4 \
  --speed 15 \
  --duration 1.0
```

If the wheels do not both rotate forward, change `left_motor_sign` or `right_motor_sign` in:

```text
src/yahboom_2wd_bringup/config/yahboom_2wd.yaml
```

## Bringup on `yahboom1`

Use one terminal for the robot launch.

Terminal 1:

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

The launch should show that both nodes started:

```text
robot_state_publisher
yahboom_2wd_node
```

The Yahboom node should print something similar to:

```text
Yahboom 2WD interface started on /dev/myserial; command_mode=motion; left=M2, right=M4
Using initial IMU yaw offset for odom: ... rad
```

## Topic verification

Use a second terminal.

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 node list
ros2 topic list | grep robot1
ros2 topic info -v /robot1/cmd_vel
```

Expected important topics:

```text
/robot1/cmd_vel
/robot1/odom
/robot1/imu/data
/robot1/battery_state
/robot1/encoder_ticks
/robot1/diagnostics
```

Echo odometry:

```bash
ros2 topic echo /robot1/odom
```

Echo IMU:

```bash
ros2 topic echo /robot1/imu/data
```

## Motion tests

### Very gentle one-shot command

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 topic pub --once /robot1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

### Five-second smoke test

```bash
timeout 5s ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}" --rate 10
```

### Ten-second calibrated straight-line test

With `linear_cmd_scale:=1.7`, this test should move `yahboom1` approximately 1 m in the x direction:

```bash
timeout 10s ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10
```

Interpretation:

```text
Commanded nominal speed: 0.1 m/s
Command duration:        10 s
Expected distance:       1.0 m
Calibrated scale:        linear_cmd_scale = 1.7
```

The physical measurement should be made using a fixed point on the robot body before and after the test. For a straight-line test with little yaw drift, the exact point is less important than using the same point consistently.


## Odometry-feedback path-following tests

The path-following package should be added under:

```text
~/yahboom2wd_ws/src/yahboom_2wd_tests
```

After adding the package, rebuild the workspace:

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

The Yahboom bridge must already be running in Terminal 1 before starting a feedback path-following test.

### Single-robot feedback test progress

| Order | Scenario | Main objective | Current result on `yahboom1` | Status | Next action |
|---:|---|---|---|---|---|
| 1 | `straight` | Validate odometry-feedback tracking of a 1.0 m forward reference | After adding `odom_linear_scale = 1.5`, the robot traveled approximately 1.0 m physically for a 1.0 m reference | Passed | Keep as baseline calibration test |
| 2 | `pure_rotation` | Validate yaw sign, angular command behavior, and automatic stopping | After fixing yaw-only stopping and using slower settings, the robot turned approximately 90 degrees and stopped automatically | Passed | Keep as angular baseline test |
| 3a | `arc` left | Validate simultaneous `linear.x` and `angular.z` for a left quarter circle | Endpoint stayed within roughly 10 cm of the expected `(x, y) = (1.0, 1.0)` m target and the trajectory was smooth | Passed | Keep as left constant-curvature baseline |
| 3b | `arc` right | Check left/right symmetry for the same constant-curvature reference | Result looked similar to the left arc, with opposite rotation direction | Passed | Keep as right constant-curvature baseline |
| 4a | `circle` left | Validate sustained left constant-curvature tracking | Smooth full-circle trajectory; final displacement roughly within 15-20 cm, mainly y-axis offset | Passed with observation | Keep as sustained left-turn baseline |
| 4b | `circle` right | Validate sustained right constant-curvature tracking | Similar to the left circle, with opposite rotation direction and about 20 cm y-axis offset | Passed with observation | Keep as sustained right-turn baseline |
| 5 | `stop_and_go` | Validate repeated starts/stops and transient behavior | After fixing `ref_stop_and_go()`, the robot moved smoothly straight, stopped twice between the three move phases, and ended around `0.57 m` for a `0.54 m` reference | Passed | Keep as transient-response baseline |
| 6 | `sinusoidal` | Validate smooth left-right changing curvature | Robot traveled approximately 2 m in a smooth low-amplitude sinusoidal shape, with small fluctuations around the reference, final `dx ≈ 2.03 m`, `dy ≈ -0.02 m`, and automatic stopping | Passed | Single-robot suite for `robot1` complete |
| 7 | `robot2` calibration and validation | Repeat calibration and the same standard test suite on the second Yahboom 2WD robot | `robot2` has been assembled, calibrated, and has passed the same standard tests | Passed | Keep its own calibration values as a separate baseline |
| 8 | two-robot distributed MPC dry run | Validate namespaces, networking, time alignment, common world frame, ZeroMQ communication, command publishing, and two-robot MPC behavior | Dry run passed: VM sees both robots, `/dmpc/robot*/pose_world` confirms the measured common-frame initial poses, `/dmpc/robot*/u_world` is published, and `/robot*/cmd_vel` stays zero with `enable_motion:=false` | Passed | Use this as the safety gate before enabling motion |
| 9 | first two-robot motion-enabled DMPC test | Command both physical Yahboom robots from the VM coordinator using the validated common world frame | Not started | Next | Run a short low-speed test with `enable_motion:=true`, record a bag, and be ready to stop the coordinator |
| 10 | four-robot distributed MPC setup | Scale the final planner to four robots | Not started | Future | Order and commission `robot3` and `robot4` after the two-robot setup is stable |

The progress table should be updated after every bagged experiment. Keep the terminal command, bag folder name, physical observation, and plotted result together so that calibration decisions are traceable.

### Terminal 1: launch the Yahboom bridge

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

### Terminal 2: run the recommended first feedback test

Start with a 1 m straight-line reference at a conservative speed:

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=straight \
  linear_speed:=0.06 \
  distance:=1.0 \
  max_linear_speed:=0.09 \
  max_angular_speed:=0.40
```

This test reads `/robot1/odom`, computes tracking error relative to the straight reference path, and publishes corrected commands to `/robot1/cmd_vel`.

### Pure rotation test

Use this to validate yaw direction, yaw-rate sign, and angular velocity scaling:

Turn to LEFT:
```bash
ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=pure_rotation \
  angular_speed:=0.12 \
  rotation_angle:=1.5708 \
  turn_direction:=left \
  max_angular_speed:=0.25 \
  goal_tolerance_yaw:=0.025
```
Turn to RIGHT:
```bash
ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=pure_rotation \
  angular_speed:=0.12 \
  rotation_angle:=1.5708 \
  turn_direction:=right \
  max_angular_speed:=0.25 \
  goal_tolerance_yaw:=0.025
```

Expected nominal motion: approximately 90 degrees counterclockwise for `turn_direction:=left`, and approximately 90 degrees clockwise for `turn_direction:=right`.

Development note: the `pure_rotation` scenario must stop based on yaw error only. It should command `linear.x = 0.0` throughout the test. Small x/y odometry drift during in-place rotation should not prevent the test from finishing.

### Constant arc test

Use this to validate simultaneous forward and angular velocity commands. This is the first combined-motion test after `straight` and `pure_rotation`.

The left quarter-circle test has passed with the following conservative settings. The endpoint stayed within roughly 10 cm of the expected `(x, y) = (1.0, 1.0)` m target and the odometry trajectory was smooth:

```bash
ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=arc \
  linear_speed:=0.05 \
  radius:=1.0 \
  arc_angle:=1.5708 \
  turn_direction:=left \
  max_linear_speed:=0.075 \
  max_angular_speed:=0.20 \
  goal_tolerance_xy:=0.08 \
  goal_tolerance_yaw:=0.05
```

Expected nominal motion for the left test: a quarter-circle arc with radius 1 m. The final pose should be approximately 1 m forward, 1 m lateral to the left, and 90 degrees left relative to the start pose, in the local test frame.

The next test is the symmetric right quarter-circle. Use the same values and change only `turn_direction`:

```bash
ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=arc \
  linear_speed:=0.05 \
  radius:=1.0 \
  arc_angle:=1.5708 \
  turn_direction:=right \
  max_linear_speed:=0.075 \
  max_angular_speed:=0.20 \
  goal_tolerance_xy:=0.08 \
  goal_tolerance_yaw:=0.05
```

Expected nominal motion for the right test: a quarter-circle arc with radius 1 m, approximately 1 m forward, 1 m lateral to the right, and 90 degrees right relative to the start pose. Passing both left and right arcs gives confidence that the robot can handle constant-curvature commands symmetrically before running the full `circle` test.

### Circle test

Use this after the arc test to evaluate sustained constant-curvature tracking. The left and right circle tests have passed for first-run commissioning with the following conservative settings:

```bash
ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=circle \
  linear_speed:=0.05 \
  radius:=0.6 \
  turn_direction:=left \
  max_linear_speed:=0.075 \
  max_angular_speed:=0.20 \
  goal_tolerance_xy:=0.20 \
  goal_tolerance_yaw:=0.08
```

For the right circle, use the same values and change only:

```text
turn_direction = right
```

Expected nominal motion: one full circle, returning close to the start pose. In the first left and right circle tests, the final position error was approximately within 15-20 cm, mainly visible as a y-axis offset. This is acceptable at this commissioning stage because the trajectories were smooth, the rotation directions were correct, and no strong spiral or oscillation appeared.

#### Note on the observed circle y-axis offset

Do not split `odom_linear_scale` into global `odom_linear_scale_x` and `odom_linear_scale_y` for now. In the Yahboom bridge, the board-reported body-frame linear velocity is scaled and then rotated into the odom frame using the current yaw estimate. A final y-offset after a full circle is therefore more likely caused by accumulated heading/curvature error, wheel slip, caster behavior, floor friction, or small left/right drive asymmetry than by an independent global y-scale error.

Keep `odom_linear_scale = 1.5` as the baseline because it is validated by the straight-line feedback test. If circle accuracy later becomes critical, investigate yaw/curvature calibration, controller gains, repeated left/right averages, and eventually sensor fusion before adding separate x/y odometry scaling.

### Stop-and-go test

Use this to test repeated starts, stops, command timeout behavior, transient response, and whether the robot remains well behaved when high-level commands switch between motion and zero velocity.

The validated commissioning command is:

```bash
ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=stop_and_go \
  linear_speed:=0.06 \
  move_time:=3.0 \
  stop_time:=2.0 \
  cycles:=3 \
  max_linear_speed:=0.09 \
  max_angular_speed:=0.30 \
  goal_tolerance_xy:=0.08 \
  goal_tolerance_yaw:=0.06
```

Expected nominal motion: three short forward motion segments separated by full stops. With the values above, the reference forward distance is approximately:

```text
3 cycles * 3.0 s/cycle * 0.06 m/s = 0.54 m
```

Development note: an earlier version of `ref_stop_and_go()` accidentally added one extra move segment and produced a reference jump from about `0.54 m` to about `0.72 m`. This was fixed by clamping the completed cycles at the configured number of cycles.

After the fix, the stop-and-go test is considered passed for commissioning. The repeated run produced smooth straight motion, two visible intermediate stop phases, and final odometry around `0.57 m` with very small lateral drift. This is consistent with the expected `0.54 m` reference distance.

### Sinusoidal path test

Use this to test smooth changing-curvature commands. Run it after `straight`, `pure_rotation`, left/right `arc`, left/right `circle`, and `stop_and_go` behave correctly.

The validated commissioning command for `robot1` is:

```bash
ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=sinusoidal \
  linear_speed:=0.05 \
  amplitude:=0.10 \
  wavelength:=2.0 \
  path_length:=2.0 \
  max_linear_speed:=0.075 \
  max_angular_speed:=0.25 \
  goal_tolerance_xy:=0.12 \
  goal_tolerance_yaw:=0.10
```

Expected nominal motion: the robot moves forward about `2 m` while smoothly changing curvature left and right. With `amplitude = 0.10 m` and `wavelength = 2.0 m`, the reference lateral deviation is small, approximately between `-0.10 m` and `+0.10 m`.

The current implementation uses:

```text
y(x) = amplitude * sin(2*pi*x / wavelength)
```

Therefore, for `path_length = 2.0 m` and `wavelength = 2.0 m`, the path completes one sinusoidal wavelength and ends near `y = 0`. The initial and final path tangent are not exactly zero, so a small initial heading correction and a nonzero final heading reference are expected.

The `robot1` sinusoidal test is considered passed for commissioning. The robot traveled about 2 m in a smooth S-shaped trajectory, showed no unstable oscillation, and stopped automatically near the end. The bag summary reported approximately `final dx = 2.03 m`, `final dy = -0.02 m`, and `path length = 2.12 m`.

### Safety notes for path-following tests

- Start with low speeds: `0.05` to `0.08 m/s`.
- Keep at least 2 m of free space around the robot for arc, circle, and sinusoidal tests.
- Be ready to stop the path follower with `Ctrl+C`.
- If the robot turns the wrong way, stop the test and inspect yaw sign, `turn_direction`, and `angular_cmd_scale`.
- If `/robot1/path_test/tracking_error.y` grows instead of decreasing, stop and inspect the heading/yaw convention.

## ROS 2 bag recording

Use bag recording instead of terminal logs for calibration and debugging.

Start the robot launch in Terminal 1, then open Terminal 2 for bag recording.

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42

mkdir -p ~/yahboom2wd_ws/bags

ros2 bag record -s mcap \
  -o ~/yahboom2wd_ws/bags/robot1_forward_0p1_10s_$(date +%Y%m%d_%H%M%S) \
  /robot1/cmd_vel \
  /robot1/odom \
  /robot1/imu/data \
  /robot1/battery_state \
  /robot1/encoder_ticks \
  /robot1/diagnostics \
  /rosout \
  /tf \
  /tf_static
```

Open Terminal 3 for the command:

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42

timeout 10s ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10
```

After the robot stops, press `Ctrl+C` in the bag-recording terminal.

Check the bag:

```bash
ros2 bag info ~/yahboom2wd_ws/bags/<bag_folder_name>
```

Example:

```bash
LATEST_BAG=$(ls -td ~/yahboom2wd_ws/bags/robot1_forward_0p1_10s_* | head -1)
ros2 bag info "$LATEST_BAG"
```


## ROS 2 bag recording for feedback path-following tests

For feedback path-following tests, record the reference and tracking-error topics in addition to the base robot topics.

Start the Yahboom bridge in Terminal 1, then start the bag in Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42

mkdir -p ~/yahboom2wd_ws/bags

ros2 bag record -s mcap \
  -o ~/yahboom2wd_ws/bags/robot1_feedback_straight_$(date +%Y%m%d_%H%M%S) \
  /robot1/cmd_vel \
  /robot1/odom \
  /robot1/imu/data \
  /robot1/battery_state \
  /robot1/encoder_ticks \
  /robot1/diagnostics \
  /robot1/path_test/reference_pose \
  /robot1/path_test/tracking_error \
  /rosout \
  /tf \
  /tf_static
```

Then run the path follower in Terminal 3, for example:

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=straight \
  linear_speed:=0.06 \
  distance:=1.0 \
  max_linear_speed:=0.09 \
  max_angular_speed:=0.40
```

After the test finishes, stop the bag recorder with `Ctrl+C`.

For later tests, use descriptive bag names such as:

```text
robot1_feedback_straight_YYYYMMDD_HHMMSS
robot1_feedback_rotation_YYYYMMDD_HHMMSS
robot1_feedback_arc_YYYYMMDD_HHMMSS
robot1_feedback_circle_YYYYMMDD_HHMMSS
robot1_feedback_stopgo_YYYYMMDD_HHMMSS
robot1_feedback_sinusoidal_YYYYMMDD_HHMMSS
```

For sinusoidal tests, use for example:

```bash
ros2 bag record -s mcap \
  -o ~/yahboom2wd_ws/bags/robot1_feedback_sinusoidal_$(date +%Y%m%d_%H%M%S) \
  /robot1/cmd_vel \
  /robot1/odom \
  /robot1/imu/data \
  /robot1/battery_state \
  /robot1/encoder_ticks \
  /robot1/diagnostics \
  /robot1/path_test/reference_pose \
  /robot1/path_test/tracking_error \
  /rosout \
  /tf \
  /tf_static
```

## Plotting a recorded bag

The workspace now uses three complementary plotting tools under:

```text
~/yahboom2wd_ws/tools/
```

Use them at different levels of inspection:

```text
plot_yahboom_bag.py             -> single-robot plots for one namespace
plot_yahboom_team_bag.py        -> team-level static plots for robot1 + robot2
plot_yahboom_team_animation.py  -> team-level motion animation from the bag
```

The original `plot_yahboom_bag.py` remains useful for detailed per-robot inspection. The new team-level plotters are needed because a two-robot DMPC experiment cannot be evaluated only from individual robot plots. For formation and safety, the important quantities are global/common-frame team quantities such as inter-robot distance, minimum distance to `d_safe`, convergence to the target formation distance, and the relation between `/dmpc/robot*/u_world` and `/robot*/cmd_vel`.

### Single-robot plotting with `plot_yahboom_bag.py`

Use this tool when you want to inspect one robot individually, for example `/robot1/odom`, `/robot1/cmd_vel`, `/robot1/imu/data`, encoder ticks, battery, diagnostics, and rosout messages.

For any latest robot1 feedback bag, run:

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42

LATEST_BAG=$(find ~/yahboom2wd_ws/bags \
  -maxdepth 1 \
  -type d \
  -name "robot1_feedback_*" \
  -printf "%T@ %p\n" \
  | sort -nr \
  | head -1 \
  | cut -d' ' -f2-)

python3 ~/yahboom2wd_ws/tools/plot_yahboom_bag.py \
  --bag "$LATEST_BAG" \
  --namespace robot1
```

For a two-robot simulation or hardware bag, run the same tool once per robot namespace:

```bash
LATEST_BAG=$(ls -td ~/yahboom2wd_ws/bags/two_robot_dmpc_sim_* | head -1)

python3 ~/yahboom2wd_ws/tools/plot_yahboom_bag.py \
  --bag "$LATEST_BAG" \
  --namespace robot1 \
  --output-dir "$LATEST_BAG/plots_robot1"

python3 ~/yahboom2wd_ws/tools/plot_yahboom_bag.py \
  --bag "$LATEST_BAG" \
  --namespace robot2 \
  --output-dir "$LATEST_BAG/plots_robot2"
```

For a hardware two-robot bag, use:

```bash
LATEST_BAG=$(ls -td ~/yahboom2wd_ws/bags/two_robot_dmpc_motion_* | head -1)
```

The single-robot plotter writes figures and a summary file into the selected output directory. Useful outputs include:

```text
summary.txt
cmd_vel_timeseries.png
odom_xy.png
odom_pose_timeseries.png
odom_twist_timeseries.png
encoder_delta_ticks.png
imu_angular_velocity.png
```

Some hardware-only plots such as encoder ticks, battery voltage, IMU, or diagnostics may be missing in a simulation bag. That is normal because the simulator is intentionally focused on the ROS motion interface.

### Team-level static plotting with `plot_yahboom_team_bag.py`

Use this tool for the holistic DMPC evaluation of the team. It reads the common world-frame topics and team metrics from the same ROS 2 bag:

```text
/dmpc/robot1/pose_world
/dmpc/robot2/pose_world
/dmpc/robot1/u_world
/dmpc/robot2/u_world
/dmpc/two_robot/metrics
/dmpc/two_robot/safety_thresholds
/robot1/cmd_vel
/robot2/cmd_vel
```

It also uses `two_robot_dmpc_analysis.csv` if that file already exists in the bag directory. If the CSV is missing, it computes the most important pairwise metrics directly from `/dmpc/robot*/pose_world`.

For the latest simulation bag, run:

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

LATEST_BAG=$(ls -td ~/yahboom2wd_ws/bags/two_robot_dmpc_sim_* | head -1)

python3 ~/yahboom2wd_ws/tools/plot_yahboom_team_bag.py \
  --bag "$LATEST_BAG" \
  --namespaces robot1 robot2 \
  --storage-id auto \
  --d-safe 0.65 \
  --formation-margin 0.15 \
  --d-agent-enter 0.70 \
  --d-agent-exit 0.75
```

For the latest hardware motion-enabled bag, use:

```bash
LATEST_BAG=$(ls -td ~/yahboom2wd_ws/bags/two_robot_dmpc_motion_* | head -1)

python3 ~/yahboom2wd_ws/tools/plot_yahboom_team_bag.py \
  --bag "$LATEST_BAG" \
  --namespaces robot1 robot2 \
  --storage-id auto \
  --d-safe 0.65 \
  --formation-margin 0.15 \
  --d-agent-enter 0.70 \
  --d-agent-exit 0.75
```

The team plotter writes its results into:

```text
<bag_folder>/team_plots/
```

Typical outputs are:

```text
team_summary.txt
team_world_trajectories.png
team_world_position_timeseries.png
team_inter_robot_distance.png
team_safety_margin.png
team_formation_distance_error.png
team_u_world_timeseries.png
team_cmd_vel_timeseries.png
team_analysis_csv_distance_comparison.png
```

For the current two-robot configuration, the most important quantitative values are:

```text
d_safe              = 0.65 m
formation_margin    = 0.15 m
target pair distance = d_safe + formation_margin = 0.80 m
d_agent_enter       = 0.70 m
d_agent_exit        = 0.75 m
```

The team-level pass/fail criteria are:

```text
minimum inter-robot distance > d_safe
minimum safety margin = distance - d_safe > 0
final inter-robot distance close to 0.80 m
formation-distance error decreases or remains small
both robots receive bounded and smooth cmd_vel commands
both robots stop when the coordinator stops
```

### Team-level animation with `plot_yahboom_team_animation.py`

Use this tool to create a GIF of the team motion. It visualizes the robot positions in the common world frame, heading arrows, trajectory trails, the line between robots, and safety/formation metrics over time.

For the latest simulation bag:

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

LATEST_BAG=$(ls -td ~/yahboom2wd_ws/bags/two_robot_dmpc_sim_* | head -1)

python3 ~/yahboom2wd_ws/tools/plot_yahboom_team_animation.py \
  --bag "$LATEST_BAG" \
  --namespaces robot1 robot2 \
  --storage-id auto \
  --d-safe 0.65 \
  --formation-margin 0.15 \
  --fps 8 \
  --step 1 \
  --max-frames 600
```

For the latest hardware bag:

```bash
LATEST_BAG=$(ls -td ~/yahboom2wd_ws/bags/two_robot_dmpc_motion_* | head -1)

python3 ~/yahboom2wd_ws/tools/plot_yahboom_team_animation.py \
  --bag "$LATEST_BAG" \
  --namespaces robot1 robot2 \
  --storage-id auto \
  --d-safe 0.65 \
  --formation-margin 0.15 \
  --fps 8 \
  --step 1 \
  --max-frames 600
```

The animation is written to:

```text
<bag_folder>/team_animation/team_motion.gif
```

If the GIF becomes too large, increase `--step` or reduce `--max-frames`, for example:

```bash
python3 ~/yahboom2wd_ws/tools/plot_yahboom_team_animation.py \
  --bag "$LATEST_BAG" \
  --namespaces robot1 robot2 \
  --step 3 \
  --max-frames 300
```

### Recommended complete post-processing order for a two-robot run

After each two-robot simulation or hardware experiment, run:

```bash
# 1) Analyze quantitative pairwise safety/formation metrics.
ros2 run yahboom_2wd_dmpc_sim analyze_two_robot_bag \
  --bag "$LATEST_BAG" \
  --storage sqlite3 \
  --d-safe 0.65 \
  --formation-margin 0.15

# 2) Plot robot1 individually.
python3 ~/yahboom2wd_ws/tools/plot_yahboom_bag.py \
  --bag "$LATEST_BAG" \
  --namespace robot1 \
  --output-dir "$LATEST_BAG/plots_robot1"

# 3) Plot robot2 individually.
python3 ~/yahboom2wd_ws/tools/plot_yahboom_bag.py \
  --bag "$LATEST_BAG" \
  --namespace robot2 \
  --output-dir "$LATEST_BAG/plots_robot2"

# 4) Plot team-level DMPC behavior.
python3 ~/yahboom2wd_ws/tools/plot_yahboom_team_bag.py \
  --bag "$LATEST_BAG" \
  --namespaces robot1 robot2 \
  --d-safe 0.65 \
  --formation-margin 0.15 \
  --d-agent-enter 0.70 \
  --d-agent-exit 0.75

# 5) Create team-level animation.
python3 ~/yahboom2wd_ws/tools/plot_yahboom_team_animation.py \
  --bag "$LATEST_BAG" \
  --namespaces robot1 robot2 \
  --d-safe 0.65 \
  --formation-margin 0.15
```

Use the individual plots to debug each robot. Use the team plots and animation to decide whether the distributed MPC experiment achieved safe formation behavior.

## Development roadmap after `robot1`

The standard single-robot commissioning suite has now passed on `robot1`. The next development stage is not the distributed MPC immediately; it is the assembly, calibration, and validation of `robot2`.

Recommended staged strategy:

1. **Finish `robot1` baseline**: keep the current `robot1` calibration as the reference baseline.
2. **Assemble `robot2`**: build the second Yahboom 2WD robot and verify the mechanical assembly, M2/M4 wiring, battery, serial link, IMU, and encoder readings.
3. **Calibrate `robot2` independently**: do not assume that `robot2` will use exactly the same `linear_cmd_scale`, `angular_cmd_scale`, or `odom_linear_scale` as `robot1`. Start from the `robot1` values, but tune only from `robot2` bagged measurements.
4. **Run the full standard single-robot suite on `robot2`**: open-loop straight, feedback straight, pure rotation, left/right arc, left/right circle, stop-and-go, and sinusoidal.
5. **Freeze `robot1` and `robot2` baselines**: record each robot's final launch parameters and test evidence.
6. **Run a two-robot distributed MPC dry run**: validate namespaces, networking, time alignment, collision-free references, per-robot command publishing, and bag recording using only `robot1` and `robot2`.
7. **Order and commission `robot3` and `robot4` only after the two-robot distributed MPC setup is stable**: this de-risks the software and communication architecture before scaling to the intended four-robot experiment.

This staged approach is the recommended development strategy. The default/ideal distributed MPC algorithm may target four robots, but a two-robot implementation is the correct intermediate validation step while only two physical Yahboom robots are available.

## Two-robot staging and four-robot convention for distributed MPC



Run the same bringup on each RPi4, changing only `namespace`, hostname, and optionally calibration values.

```text
yahboom1 -> /robot1/cmd_vel, /robot1/odom
yahboom2 -> /robot2/cmd_vel, /robot2/odom
yahboom3 -> /robot3/cmd_vel, /robot3/odom
yahboom4 -> /robot4/cmd_vel, /robot4/odom
```

On the Ubuntu VM, the distributed MPC planner should publish namespaced `Twist` commands and subscribe to namespaced odometry:

```text
planner/controller_i -> /robot{i}/cmd_vel
/robot{i}/odom      -> planner/state estimator or coordinator
```

Use a shared `ROS_DOMAIN_ID`, ensure all machines are on the same LAN, and use chrony or NTP so logs and bags are time-aligned.

## Recommended commissioning sequence

### Per-robot commissioning sequence

Use this sequence for each physical robot. `robot1` has completed this sequence; repeat it for `robot2` after assembly.

1. Confirm `/dev/myserial` exists after plugging in MicroUSB.
2. Set `ROS_DOMAIN_ID=42` in every terminal.
3. Run `yahboom_serial_probe` and check firmware version, battery voltage, IMU values, and encoder ticks.
4. Lift the robot and run `yahboom_motor_test` in low PWM mode to verify M2/M4 signs.
5. Launch `command_mode:=motion`.
6. Verify `/robotX/cmd_vel` has one subscriber using `ros2 topic info -v /robotX/cmd_vel`.
7. Test `/robotX/cmd_vel` at `0.05 m/s`.
8. Record a bag during a 10-second open-loop straight-line test.
9. Plot the bag and compare physical distance, odometry, and encoder ticks.
10. Adjust `linear_cmd_scale`, `angular_cmd_scale`, `odom_linear_scale`, motor signs, or `wheel_separation` only after recording evidence.
11. Add and build `yahboom_2wd_tests` under `~/yahboom2wd_ws/src`.
12. Run the feedback `straight` scenario and record `/robotX/path_test/reference_pose` and `/robotX/path_test/tracking_error`.
13. Run `pure_rotation`, then left/right `arc`, then left/right `circle`, then `stop_and_go`, then `sinusoidal`.
14. Compare reference trajectory, odometry trajectory, lateral error, heading error, encoder ticks, and IMU yaw rate for each test.
15. Store the final per-robot calibration values in this README.

### Multi-robot development sequence

1. Complete `robot1` commissioning. This is done.
2. Assemble and complete `robot2` commissioning.
3. Run a two-robot ROS communication test with `/robot1/*` and `/robot2/*` active at the same time.
4. Run a two-robot open-loop command test where the two robots receive independent namespaced `cmd_vel` commands.
5. Run a two-robot distributed MPC dry run using safe low-speed references and large separation distances.
6. Only after the two-robot MPC setup is stable, order and commission `robot3` and `robot4`.
7. Scale the same namespace, calibration, logging, and MPC structure to the final four-robot experiment.

## Current known calibration for `yahboom1`

```text
wheel_radius       = 0.0325 m
wheel_separation   = 0.120 m
left_motor_port    = M2
right_motor_port   = M4
command_mode       = motion
linear_cmd_scale   = 1.7
angular_cmd_scale  = 1.0
odom_linear_scale  = 1.5
ROS_DOMAIN_ID      = 42
```

The values above are the current baseline for `yahboom1`. Do not change them unless a new bagged experiment clearly shows a repeatable error.

## Current known calibration for `robot2`

`robot2` has been assembled and has passed the same standard commissioning tests as `robot1`. Keep its calibration record separate from `robot1`, because two mechanically similar Yahboom robots can still need different command and odometry scaling values.

Fill the exact final values from the `robot2` calibration bags:

```text
wheel_radius       = 0.0325 m or measured robot2-specific value
wheel_separation   = 0.120 m or measured robot2-specific value
left_motor_port    = M2
right_motor_port   = M4
command_mode       = motion
linear_cmd_scale   = <robot2 calibrated value>
angular_cmd_scale  = <robot2 calibrated value>
odom_linear_scale  = <robot2 calibrated value>
ROS_DOMAIN_ID      = 42
```

Do not silently copy `robot1` calibration values into `robot2` unless the `robot2` open-loop straight, feedback straight, pure rotation, arc, circle, stop-and-go, and sinusoidal bags support that decision.

## Current recommended single-robot feedback test settings for `yahboom1`

Use the following bridge settings in Terminal 1:

```bash
ros2 launch yahboom_2wd_bringup yahboom_2wd.launch.py \
  namespace:=robot1 \
  serial_port:=/dev/myserial \
  command_mode:=motion \
  linear_cmd_scale:=1.7 \
  angular_cmd_scale:=1.0 \
  odom_linear_scale:=1.5
```

### Passed baseline tests

Straight-line feedback test:

```bash
ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=straight \
  linear_speed:=0.06 \
  distance:=1.0 \
  max_linear_speed:=0.09 \
  max_angular_speed:=0.40
```

Pure-rotation feedback test:

```bash
ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=pure_rotation \
  angular_speed:=0.12 \
  rotation_angle:=1.5708 \
  turn_direction:=left \
  max_angular_speed:=0.25 \
  goal_tolerance_yaw:=0.025
```

The right-turn version should use the same values with:

```text
turn_direction = right
```

Left constant-arc feedback test:

```bash
ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=arc \
  linear_speed:=0.05 \
  radius:=1.0 \
  arc_angle:=1.5708 \
  turn_direction:=left \
  max_linear_speed:=0.075 \
  max_angular_speed:=0.20 \
  goal_tolerance_xy:=0.08 \
  goal_tolerance_yaw:=0.05
```

Right constant-arc feedback test: same values with `turn_direction:=right`.

Left/right circle feedback tests:

```bash
ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=circle \
  linear_speed:=0.05 \
  radius:=0.6 \
  turn_direction:=left \
  max_linear_speed:=0.075 \
  max_angular_speed:=0.20 \
  goal_tolerance_xy:=0.20 \
  goal_tolerance_yaw:=0.08
```

Right circle: same values with `turn_direction:=right`.

Stop-and-go feedback test:

```bash
ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=stop_and_go \
  linear_speed:=0.06 \
  move_time:=3.0 \
  stop_time:=2.0 \
  cycles:=3 \
  max_linear_speed:=0.09 \
  max_angular_speed:=0.30 \
  goal_tolerance_xy:=0.08 \
  goal_tolerance_yaw:=0.06
```

After fixing `ref_stop_and_go()`, this test passed with smooth straight motion, two intermediate stop phases, and final odometry close to the expected `0.54 m` reference distance.

### Next step

The standard single-robot suite has passed on both `robot1` and `robot2`. The two-robot distributed MPC **dry run has also passed** with the corrected controller/coordinator nodes and the common world-frame initialization.

The verified dry-run setup used:

```text
robot1: x0 = 0.0 m, y0 = -0.7 m, yaw0 = 0.0 rad
robot2: x0 = 0.0 m, y0 = +0.7 m, yaw0 = 0.0 rad
```

The coordinator published the expected common-frame debug poses:

```text
/dmpc/robot1/pose_world -> frame_id = map, position = (0.0, -0.7, 0.0)
/dmpc/robot2/pose_world -> frame_id = map, position = (0.0, +0.7, 0.0)
```

and `/robot1/cmd_vel` and `/robot2/cmd_vel` stayed zero because `enable_motion:=false`.

The next step is therefore the **first short motion-enabled two-robot DMPC test**. Start conservatively, record a bag, and be ready to stop the coordinator with `Ctrl+C`.

Recommended first motion-enabled test:

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
  enable_motion:=true
```

Use a clear test area, keep the robots well separated, and stop the test after a short first run of about 10-20 seconds. If the motion is smooth and the bag confirms correct `/dmpc/robot*/pose_world`, `/dmpc/robot*/u_world`, and `/robot*/cmd_vel`, then repeat with the default commissioning values.

Only after the two-robot distributed MPC pipeline is stable should the project scale to `robot3` and `robot4`.


## Two-robot distributed MPC development stage

The single-robot commissioning suite has now been completed for `robot1`, and `robot2` has also been assembled, calibrated, and validated with the same standard tests. The next development stage is therefore a **two-robot distributed MPC experiment** with:

```text
robot1 -> /robot1/odom, /robot1/cmd_vel
robot2 -> /robot2/odom, /robot2/cmd_vel
```

The final research target remains a four-robot distributed MPC setup, but the correct intermediate milestone is a two-robot hardware test. This reduces hardware risk and makes network, timing, odometry-frame, and command-conversion issues easier to isolate before purchasing and commissioning `robot3` and `robot4`.

### Development strategy

Use the following staged plan:

| Stage | Hardware | Goal | Status |
|---:|---|---|---|
| 1 | `robot1` only | Calibrate Yahboom ROS 2 interface and pass all standard single-robot path tests | Complete |
| 2 | `robot2` only | Assemble, calibrate, and validate the second Yahboom 2WD robot with the same test suite | Complete |
| 3 | `robot1` + `robot2` | Run the distributed MPC stack in a two-robot star topology | Next |
| 4 | `robot1` + `robot2` + simulated agents | Optionally test mixed hardware/SIL scaling behavior | Optional |
| 5 | `robot1` ... `robot4` | Order and commission two more robots only after the two-robot hardware stack is stable | Future |

### Two-robot star topology

The first hardware distributed MPC test should use a star topology:

```text
robot1 Raspberry Pi  <---- ZeroMQ ----\
                                       \
                                        Ubuntu VM coordinator
                                       /
robot2 Raspberry Pi  <---- ZeroMQ ----/
```

The robots do not communicate directly with each other. Each robot runs its local MPC/safety controller node. The Ubuntu VM coordinator gathers the current robot states from ROS 2 odometry, requests local MPC solutions over ZeroMQ, applies the safety layer, and publishes velocity commands back to the robot namespaces.

### ROS 2 / ZeroMQ interface package

Use the package:

```text
yahboom_2wd_dmpc
```

This package connects the already-validated Yahboom ROS 2 interface to the existing distributed MPC app.

Robot-side nodes:

```text
robot1: dmpc_controller_node --agent-id 1  -> ZMQ REP port 5601
robot2: dmpc_controller_node --agent-id 2  -> ZMQ REP port 5602
```

VM-side node:

```text
dmpc_coordinator_ros_node
```

The VM node subscribes to:

```text
/robot1/odom
/robot2/odom
```

and publishes:

```text
/robot1/cmd_vel
/robot2/cmd_vel
```

It also publishes debug world-frame commands:

```text
/dmpc/robot1/u_world
/dmpc/robot2/u_world
```


### Common world/map frame for the two-robot MPC

Each Yahboom bridge publishes a local odometry frame. Therefore, after startup it is normal for both robots to report approximately:

```text
robot1 local odom: x ≈ 0, y ≈ 0, yaw ≈ 0
robot2 local odom: x ≈ 0, y ≈ 0, yaw ≈ 0
```

This must not be passed directly to the MPC as if both poses were in one global frame. The coordinator must transform each local odometry into a shared world/map frame before building `r_all`.

For robot `i`, the measured initial world pose is:

```text
(x0_i, y0_i, yaw0_i)
```

and the coordinator computes:

```text
x_world   = x0_i + cos(yaw0_i) * x_local - sin(yaw0_i) * y_local
y_world   = y0_i + sin(yaw0_i) * x_local + cos(yaw0_i) * y_local
yaw_world = yaw0_i + yaw_local
```

For the first real test, mark a world origin and x-axis on the floor, place both robots at measured coordinates, align their headings with the marked axis, and only then start the robot bridges. The VM coordinator launch must include the measured initial poses, for example:

```bash
ros2 launch yahboom_2wd_dmpc two_robot_dmpc_coordinator.launch.py \
  robot1_controller_endpoint:=tcp://192.168.178.87:5601 \
  robot2_controller_endpoint:=tcp://192.168.178.94:5602 \
  robot1_initial_x:=0.0 \
  robot1_initial_y:=-0.45 \
  robot1_initial_yaw:=0.0 \
  robot2_initial_x:=0.0 \
  robot2_initial_y:=0.45 \
  robot2_initial_yaw:=0.0 \
  enable_motion:=false
```

This is the minimum requirement for meaningful inter-robot distance, formation, and collision-avoidance calculations. The coordinator publishes the transformed poses on:

```text
/dmpc/robot1/pose_world
/dmpc/robot2/pose_world
```

Check these topics before enabling motion. The local `/robot1/odom` and `/robot2/odom` topics may still start near `(0, 0, 0)`; that is normal. The MPC-relevant state is the transformed common-frame state shown on `/dmpc/robot*/pose_world`.

Longer experiments or stronger safety claims will eventually require external shared localization such as overhead camera/AprilTags, motion capture, UWB, or another global pose-correction method.

### First two-robot MPC test policy

Start with conservative real-robot parameters:

```text
model              = single_integrator
n_agents           = 2
graph              = complete
M_manual           = 5
u_bound            = 0.08
d_safe             = 0.65 m
formation_margin   = 0.15 m
max_linear_speed   = 0.07 m/s
max_angular_speed  = 0.35 rad/s
obstacles_enabled  = false
```

Start with `obstacles_enabled = false` to validate the communication, formation behavior, state feedback, and velocity-command conversion. Enable obstacle avoidance only after the basic two-robot formation run is repeatable and safe.


### Current DMPC package fixes and deployment notes

The first hardware deployment revealed and fixed the following issues:

| Area | Symptom | Fix |
|---|---|---|
| `rosdep` | `Cannot locate rosdep definition for [ament_python]` | Build still succeeds; use `--skip-keys "ament_python"` during `rosdep install` if needed |
| Python solver stack | NumPy 2.x with old ECOS wheel caused `_ARRAY_API not found` | Use `numpy<2`, `cvxpy<1.5`, `ecos<2.1`, `scs<3.3`, `osqp<0.7` on Raspberry Pi |
| Robot controller node | ROS 2 launch appended `--ros-args`, which argparse rejected | Use `parse_known_args()` in `dmpc_controller_node.py` |
| Robot controller node | `zmq.sleep()` did not exist in the installed pyzmq version | Use `time.sleep(0.02)` |
| VM coordinator launch | Two controller endpoints were concatenated into one string | Pass `robot1_controller_endpoint` and `robot2_controller_endpoint` as scalar parameters |
| VM coordinator node | `self.subscriptions` collided with a read-only `rclpy.node.Node` property | Use `self.odom_subscriptions` |
| MPC state frame | Both local odometries started at `(0, 0)` | Transform local odometry into a common world/map frame using measured initial poses |

Commit these fixes to the Git repository and pull the same version on `robot1`, `robot2`, and the Ubuntu VM. Avoid keeping different local package versions on different machines.

### Deployment order for the two-robot DMPC test

1. Start the calibrated Yahboom bridge on `robot1`.
2. Start the calibrated Yahboom bridge on `robot2`.
3. Start the local `dmpc_controller_node` on `robot1` with `agent_id:=1`.
4. Start the local `dmpc_controller_node` on `robot2` with `agent_id:=2`.
5. Mark a common world/map frame on the floor and measure `robot1` and `robot2` initial poses.
6. Start the Ubuntu VM coordinator with the measured `robot1_initial_*` and `robot2_initial_*` parameters and `enable_motion:=false`.
7. Confirm that the VM sees `/robot1/odom` and `/robot2/odom`.
8. Confirm that the VM can communicate with both ZMQ controller endpoints.
9. Inspect `/dmpc/robot1/pose_world` and `/dmpc/robot2/pose_world` and confirm that they match the measured initial world poses.
10. Inspect `/dmpc/robot1/u_world` and `/dmpc/robot2/u_world`.
11. Confirm that `/robot1/cmd_vel` and `/robot2/cmd_vel` remain zero in dry-run mode.
12. Switch to `enable_motion:=true` only in a clear test area and initially reduce `u_bound`, `max_linear_speed`, and `max_angular_speed`.
13. Record a bag containing both robots' odometry, commands, world poses, and DMPC debug topics.

Do not move to four physical robots until the two-robot stack runs repeatably and safely.
