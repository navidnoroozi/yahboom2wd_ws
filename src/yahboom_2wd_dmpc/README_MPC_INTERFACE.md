# yahboom_2wd_dmpc

ROS 2 / ZeroMQ interface package for running the existing distributed MPC planning
algorithm on two commissioned Yahboom 2WD robots.

## Runtime topology

- `robot1` Raspberry Pi:
  - existing `yahboom_2wd_bringup` bridge publishes `/robot1/odom` and subscribes to `/robot1/cmd_vel`
  - `dmpc_controller_node --agent-id 1` binds a ZMQ REP socket on port `5601`

- `robot2` Raspberry Pi:
  - existing `yahboom_2wd_bringup` bridge publishes `/robot2/odom` and subscribes to `/robot2/cmd_vel`
  - `dmpc_controller_node --agent-id 2` binds a ZMQ REP socket on port `5602`

- Ubuntu VM:
  - `dmpc_coordinator_ros_node` subscribes to `/robot1/odom` and `/robot2/odom`
  - sends ZMQ `mpc_request` and `hybrid_request` messages to the two robot controllers
  - publishes `/robot1/cmd_vel` and `/robot2/cmd_vel`

This is a star topology: the two robots do not talk directly to each other. The
Ubuntu VM coordinator is the hub.

## Install

Copy this package into the same workspace as the existing Yahboom packages:

```bash
cd ~/yahboom2wd_ws/src
# copy yahboom_2wd_dmpc here

cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

Install Python dependencies if they are missing:

```bash
sudo apt install -y python3-zmq python3-numpy
python3 -m pip install --user cvxpy ecos
```

## Robot-side controller nodes

On `robot1`:

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch yahboom_2wd_dmpc robot_dmpc_controller.launch.py \
  agent_id:=1
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

## Existing Yahboom bridge nodes

On each robot, keep using the robot-specific calibrated values. Example for `robot1`:

```bash
ros2 launch yahboom_2wd_bringup yahboom_2wd.launch.py \
  namespace:=robot1 \
  serial_port:=/dev/myserial \
  command_mode:=motion \
  linear_cmd_scale:=1.7 \
  angular_cmd_scale:=1.0 \
  odom_linear_scale:=1.5
```

Use `namespace:=robot2` and the calibrated `robot2` values on the second robot.

## VM-side coordinator

First run in dry-run mode. It computes and logs commands but publishes zero
`cmd_vel` because `enable_motion:=false`.

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch yahboom_2wd_dmpc two_robot_dmpc_coordinator.launch.py \
  robot1_controller_endpoint:=tcp://<ROBOT1_IP>:5601 \
  robot2_controller_endpoint:=tcp://<ROBOT2_IP>:5602 \
  enable_motion:=false
```

After verifying that the coordinator receives both odometry topics and can talk
to both ZMQ controller nodes, run the same command with:

```bash
enable_motion:=true
```

## Conservative first real-robot parameters

The default launch uses:

```text
model = single_integrator
n_agents = 2
graph = complete
u_bound = 0.08
M_manual = 5
d_safe = 0.65 m
formation_margin = 0.15 m
max_linear_speed = 0.07 m/s
max_angular_speed = 0.35 rad/s
obstacles_enabled = false
```

This should be treated as a safe initial commissioning setup, not the final
research-tuned configuration.

## First two-robot test

1. Put the robots at least 1 m apart.
2. Disable obstacles initially: `obstacles_enabled:=false`.
3. Start the Yahboom bridge on both robots.
4. Start `dmpc_controller_node` on both robots.
5. Start the VM coordinator with `enable_motion:=false`.
6. Confirm `/dmpc/robot1/u_world` and `/dmpc/robot2/u_world` are being published.
7. Set `enable_motion:=true` and test at low speed in a clear area.
8. Record `/robot1/odom`, `/robot2/odom`, `/robot1/cmd_vel`, `/robot2/cmd_vel`, and `/dmpc/*`.

Only enable obstacle avoidance after the two-robot formation run is stable.
