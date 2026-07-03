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

The straight-line velocity command was empirically calibrated on `yahboom1`:

```text
linear_cmd_scale = 1.7
```

With this value, the command below moved the robot approximately 1 m in the x direction:

```bash
timeout 10s ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10
```

The command publishes `0.1 m/s` at 10 Hz for 10 seconds, so the expected nominal distance is:

```text
0.1 m/s * 10 s = 1.0 m
```

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
└── yahboom_2wd_bringup/
    ├── launch/yahboom_2wd.launch.py
    ├── config/yahboom_2wd.yaml
    └── scripts/install_udev_yahboom.sh
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

The current calibrated straight-line value for `yahboom1` is:

```text
linear_cmd_scale = 1.7
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
# Copy or clone these packages into ~/yahboom2wd_ws/src

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
  angular_cmd_scale:=1.0
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

## Plotting a recorded bag

Copy or keep `plot_yahboom_bag.py` on the Raspberry Pi, for example:

```text
~/yahboom2wd_ws/tools/plot_yahboom_bag.py
```

Run:

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42

LATEST_BAG=$(ls -td ~/yahboom2wd_ws/bags/robot1_forward_0p1_10s_* | head -1)

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

For calibration, compare:

```text
physical tape distance
integrated commanded distance
final odom displacement
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
8. Record a bag during a 10-second straight-line test.
9. Plot the bag and compare physical distance, odometry, and encoder ticks.
10. Adjust `linear_cmd_scale`, `angular_cmd_scale`, motor signs, or `wheel_separation` only after recording evidence.
11. Repeat for all four robots with the same checklist.

## Current known calibration for `yahboom1`

```text
wheel_radius       = 0.0325 m
wheel_separation   = 0.120 m
left_motor_port    = M2
right_motor_port   = M4
command_mode       = motion
linear_cmd_scale   = 1.7
angular_cmd_scale  = 1.0
ROS_DOMAIN_ID      = 42
```

