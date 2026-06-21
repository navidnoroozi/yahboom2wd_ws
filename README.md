# Yahboom 2WD ROS 2 interface for distributed MPC prototypes

This workspace contains a minimal ROS 2 interface for a custom 2WD Yahboom robot using:

- Yahboom lightweight aluminum alloy 2WD chassis
- L-type 520 encoder DC motors
- Yahboom ROS Robot Control Board V3.0 with STM32F103RCT6-compatible MCU
- Rosmaster Python driver library
- Raspberry Pi 4 running Ubuntu 22.04 and ROS 2 Humble

The design goal is to keep the factory STM32 firmware untouched and run only a small ROS 2 bridge on the Raspberry Pi.

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
- TF: `robot1/odom -> robot1/base_footprint`

## Command modes

### `motion` mode, recommended first

The driver calls:

```python
bot.set_car_motion(vx, 0.0, wz)
```

This keeps Yahboom's factory motion-control path active. Use this first because the uploaded driver library documents `set_pid_param()` as affecting `set_car_motion()`.

### `pwm_diff` mode, commissioning fallback

The driver computes differential wheel commands:

```text
v_left  = vx - wheel_separation/2 * wz
v_right = vx + wheel_separation/2 * wz
```

Then it maps the wheel speeds to `set_motor(M1, M2, M3, M4)` PWM percentages. The uploaded Yahboom driver library explicitly comments that `set_motor()` is PWM control and does not use encoder speed feedback. Use this mode only for M2/M4 sign and wiring tests unless you later confirm that your firmware exposes a true per-motor speed command.

## One-time installation on each Raspberry Pi

```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-serial python3-colcon-common-extensions python3-rosdep git
sudo rosdep init 2>/dev/null || true
rosdep update

mkdir -p ~/yahboom2wd_ws/src
# Copy or clone these packages into ~/yahboom2wd_ws/src
cd ~/yahboom2wd_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
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
source ~/yahboom2wd_ws/install/setup.bash
ros2 run yahboom_2wd_driver yahboom_serial_probe --serial-port /dev/myserial --car-type 4
```

Then test the M2/M4 wiring gently:

```bash
ros2 run yahboom_2wd_driver yahboom_motor_test \
  --serial-port /dev/myserial \
  --left-port 2 --right-port 4 \
  --speed 15 --duration 1.0
```

If the wheels do not both rotate forward, change `left_motor_sign` or `right_motor_sign` in `config/yahboom_2wd.yaml`.

## Bringup on `yahboom1`

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch yahboom_2wd_bringup yahboom_2wd.launch.py \
  namespace:=robot1 \
  serial_port:=/dev/myserial \
  command_mode:=motion
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic echo /robot1/odom
ros2 topic echo /robot1/imu/data
```

Gentle motion test:

```bash
ros2 topic pub --once /robot1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

Continuous 5-second test:

```bash
timeout 5s ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}" --rate 10
```

## Four-robot convention for distributed MPC

Run the same bringup on each RPi4, changing only `namespace` and hostname.

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
2. Run `yahboom_serial_probe` and check firmware version, battery voltage, IMU values, and encoder ticks.
3. Lift robot and run `yahboom_motor_test` in low PWM mode to verify M2/M4 signs.
4. Launch `command_mode:=motion` and test `/robot1/cmd_vel` at 0.05 m/s.
5. Record a short bag:

```bash
ros2 bag record -o ~/bags/yahboom1_smoke_test \
  /robot1/cmd_vel /robot1/odom /robot1/imu/data /robot1/encoder_ticks /robot1/battery_state /tf
```

6. Measure straight-line drift and distance with tape. Adjust `linear_cmd_scale`, `angular_cmd_scale`, motor signs, or `wheel_separation` only after recording evidence.
7. Repeat for all four robots with the same checklist.
