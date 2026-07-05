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
yahboom2wd_ws/src/
├── yahboom_2wd_driver/
│   ├── yahboom_2wd_node.py        # cmd_vel <-> Rosmaster_Lib serial bridge
│   ├── serial_probe.py            # checks serial link, firmware version, sensors
│   ├── motor_test.py              # low-speed M2/M4 PWM test while robot is lifted
│   └── vendor/Rosmaster_Lib.py    # fallback copy of Yahboom driver library V3.3.9
├── yahboom_2wd_description/
│   └── urdf/yahboom_2wd.urdf.xacro
├── yahboom_2wd_bringup/
│   ├── launch/yahboom_2wd.launch.py
│   ├── config/yahboom_2wd.yaml
│   └── scripts/install_udev_yahboom.sh
└── yahboom_2wd_tests/
    ├── yahboom_2wd_tests/
    │   └── path_follower_node.py  # odometry-feedback path-following tests
    ├── launch/
    │   └── path_follower.launch.py
    ├── config/
    │   └── path_tests.yaml
    ├── package.xml
    ├── setup.py
    └── setup.cfg
```

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
- The next planned validation is the **sinusoidal** path test, because it checks smooth left-right changing curvature before moving toward MPC-style time-varying velocity commands.

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
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-matplotlib \
  git

sudo rosdep init 2>/dev/null || true
rosdep update

mkdir -p ~/yahboom2wd_ws/src
# Copy or clone these packages into ~/yahboom2wd_ws/src, including:
#   yahboom_2wd_driver
#   yahboom_2wd_description
#   yahboom_2wd_bringup
#   yahboom_2wd_tests

cd ~/yahboom2wd_ws
rosdep install --from-paths src -y --ignore-src --rosdistro humble
colcon build --symlink-install

source /opt/ros/humble/setup.bash
source install/setup.bash
```

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
| 6 | `sinusoidal` | Validate smooth left-right changing curvature | Not tested yet | Next | Run conservative low-amplitude sinusoidal test |

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

Start with a low-amplitude, conservative sinusoidal path:

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

The test passes if the robot follows a smooth S-shaped trajectory, alternates curvature without oscillatory instability, remains close to the reference path, and stops automatically near the end of the reference.

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

For the next sinusoidal test, use for example:

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

Keep `plot_yahboom_bag.py` on the Raspberry Pi:

```text
~/yahboom2wd_ws/tools/plot_yahboom_bag.py
```
### General pattern for any latest robot1 feedback bag:
For any feedback test scenario inclduing `straight`, `pure_rotation` etc., run:

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

The plotting script should write figures and a summary file into:

```text
<bag_folder>/plots/
```

Useful outputs for straight-line calibration:

```text
summary.txt
cmd_vel_timeseries.png
odom_xy.png
odom_twist_timeseries.png
encoder_delta_ticks.png
imu_angular_velocity.png
```

For feedback path-following bags, also inspect the reference and tracking-error signals. If the plotting script has not yet been extended for those topics, use `ros2 bag info`, `ros2 topic echo` during live tests, or extend the plotter to read:

```text
/robot1/path_test/reference_pose
/robot1/path_test/tracking_error
```

For calibration and path-following validation, compare:

```text
physical tape distance
integrated commanded distance
final odom displacement
reference trajectory versus odometry trajectory
lateral/cross-track error
heading error
encoder tick deltas of M2 and M4
average odom speed
```

## Four-robot convention for distributed MPC

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

1. Confirm `/dev/myserial` exists after plugging in MicroUSB.
2. Set `ROS_DOMAIN_ID=42` in every terminal.
3. Run `yahboom_serial_probe` and check firmware version, battery voltage, IMU values, and encoder ticks.
4. Lift the robot and run `yahboom_motor_test` in low PWM mode to verify M2/M4 signs.
5. Launch `command_mode:=motion`.
6. Verify `/robot1/cmd_vel` has one subscriber using `ros2 topic info -v /robot1/cmd_vel`.
7. Test `/robot1/cmd_vel` at `0.05 m/s`.
8. Record a bag during a 10-second open-loop straight-line test.
9. Plot the bag and compare physical distance, odometry, and encoder ticks.
10. Adjust `linear_cmd_scale`, `angular_cmd_scale`, `odom_linear_scale`, motor signs, or `wheel_separation` only after recording evidence.
11. Add and build `yahboom_2wd_tests` under `~/yahboom2wd_ws/src`.
12. Run the feedback `straight` scenario and record `/robot1/path_test/reference_pose` and `/robot1/path_test/tracking_error`.
13. Run `pure_rotation`, then left/right `arc`, then `circle`, then `stop_and_go`, then `sinusoidal`.
14. Compare reference trajectory, odometry trajectory, lateral error, heading error, encoder ticks, and IMU yaw rate for each test.
15. Repeat the validated checklist for all four robots.
16. Move to distributed MPC only after the single-robot feedback tests are repeatable and safe.

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

The values above are the current baseline for `yahboom1`. Do not change them for the sinusoidal test unless a new bagged experiment clearly shows a repeatable error.

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

### Next test to run

Sinusoidal feedback test:

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

Expected nominal behavior: the robot should move forward about `2 m` while performing a gentle left-right sinusoidal path. The lateral deviation should stay small, about `±0.10 m`, and the motion should look smooth rather than oscillatory or jerky. The robot should stop automatically near the end of the reference.

Proceed toward the distributed MPC planner only after the sinusoidal test is understood from bag data, because this is the first test with continuously changing curvature.
