# yahboom_2wd_tests

Odometry-feedback path-following scenarios for the Yahboom 2WD ROS 2 interface.

The node subscribes to `/robot1/odom` and publishes `/robot1/cmd_vel` by default. It initializes each reference trajectory from the robot's current odometry pose, so place the robot at the desired start pose before starting the test.

## Supported scenarios

- `straight`
- `pure_rotation`
- `arc`
- `circle`
- `stop_and_go`
- `sinusoidal`

## Example

Terminal 1: launch the Yahboom bridge.

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch yahboom_2wd_bringup yahboom_2wd.launch.py \
  namespace:=robot1 \
  serial_port:=/dev/myserial \
  command_mode:=motion \
  linear_cmd_scale:=1.7 \
  angular_cmd_scale:=1.0
```

Terminal 2: run a feedback straight-line test.

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom2wd_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch yahboom_2wd_tests path_follower.launch.py \
  robot_namespace:=robot1 \
  scenario:=straight \
  linear_speed:=0.08 \
  distance:=1.0
```

The node also publishes:

- `/robot1/path_test/reference_pose`
- `/robot1/path_test/tracking_error`
