# yahboom_2wd_dmpc_sim

Lightweight ROS 2 simulation counterpart for the two-robot Yahboom DMPC hardware interface.

The simulator is intentionally simple: it does not simulate motor electronics or floor contact. It simulates the ROS interface used by the real Yahboom robots:

```text
/robot1/cmd_vel -> simulated unicycle -> /robot1/odom
/robot2/cmd_vel -> simulated unicycle -> /robot2/odom
```

Both simulated robots publish local odometry that starts near `(0, 0, 0)`, just like the hardware. The existing `yahboom_2wd_dmpc` coordinator must still be launched with measured/selected initial world poses, for example:

```text
robot1_initial_y = -0.7
robot2_initial_y = +0.7
```

The coordinator then publishes the common-frame debug poses:

```text
/dmpc/robot1/pose_world
/dmpc/robot2/pose_world
```

## Installation

Copy both packages into the same workspace:

```bash
cd ~/yahboom2wd_ws/src
# copy/clone yahboom_2wd_dmpc
# copy/clone yahboom_2wd_dmpc_sim

cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src --rosdistro humble \
  --skip-keys "ament_python"
colcon build --symlink-install
source install/setup.bash
```

## Run the complete simulated two-robot DMPC stack

Stop any hardware DMPC controller nodes first, or use the simulator's default ports `5701` and `5702` to avoid collisions with hardware ports `5601` and `5602`.

```bash
cd ~/yahboom2wd_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

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

## Record a simulation bag

Run this in another terminal before or after launching the simulator stack:

```bash
mkdir -p ~/yahboom2wd_ws/bags

ros2 bag record \
  -o ~/yahboom2wd_ws/bags/two_robot_dmpc_sim_$(date +%Y%m%d_%H%M%S) \
  /robot1/odom \
  /robot2/odom \
  /robot1/cmd_vel \
  /robot2/cmd_vel \
  /dmpc/robot1/pose_world \
  /dmpc/robot2/pose_world \
  /dmpc/robot1/u_world \
  /dmpc/robot2/u_world \
  /dmpc/two_robot/metrics \
  /dmpc/two_robot/safety_thresholds \
  /robot1/diagnostics \
  /robot2/diagnostics \
  /robot1/imu/data \
  /robot2/imu/data \
  /robot1/encoder_ticks \
  /robot2/encoder_ticks \
  /rosout \
  /tf \
  /tf_static
```

Use `ros2 bag record -s sqlite3` if your VM does not have the MCAP plugin.

## Analyze a real or simulated bag

```bash
LATEST_BAG=$(ls -td ~/yahboom2wd_ws/bags/two_robot_dmpc_* | head -1)

ros2 run yahboom_2wd_dmpc_sim analyze_two_robot_bag \
  --bag "$LATEST_BAG" \
  --storage sqlite3 \
  --d-safe 0.65 \
  --formation-margin 0.15
```

The analyzer writes:

```text
<bag>/two_robot_dmpc_analysis.csv
<bag>/two_robot_dmpc_analysis.txt
```

Important metrics:

```text
current inter-robot distance
minimum inter-robot distance
minimum safety margin = distance - d_safe
target pair distance = d_safe + formation_margin for n_agents=2
final formation-distance error
```

## Quantitative meaning of the default two-robot formation

With:

```text
d_safe = 0.65 m
formation_margin = 0.15 m
n_agents = 2
formation_radius_override = 0.0
```

the desired pair distance is:

```text
d_form = d_safe + formation_margin = 0.80 m
```

The regular two-agent formation offsets are approximately:

```text
robot1 desired offset: (+0.40, 0.0) m
robot2 desired offset: (-0.40, 0.0) m
```

The safety hysteresis defaults are:

```text
d_agent_enter = 0.68 m
d_agent_exit  = 0.72 m
```

This is compatible with the formation because:

```text
d_safe < d_agent_enter < d_agent_exit < d_form
0.65   < 0.68          < 0.72         < 0.80
```

## Obstacle-avoidance simulation parameters

The simulation launch file now exposes the same circular-obstacle parameters as the hardware DMPC launch files:

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

The obstacle is defined in the coordinator's shared world/map frame. The practical inflated radius used by the safety layer is:

```text
inflated_radius = obstacle_radius + obstacle_margin
```

For the first 2 m x 3 m field simulation, use a small obstacle:

```text
obstacle_center_x = 1.0
obstacle_center_y = -0.33
obstacle_radius = 0.15
obstacle_margin = 0.10
obstacle_warning_radius = 0.35
d_obs_enter = 0.15
d_obs_exit = 0.25
tangential_waypoint_radius = 0.12
orbit_tangent_lookahead = 0.20
```

The simulation package records the same ROS topics as the hardware run, so bags can be analyzed with:

```bash
ros2 run yahboom_2wd_dmpc_sim analyze_two_robot_bag \
  --bag <bag_folder> \
  --storage sqlite3 \
  --d-safe 0.65 \
  --formation-margin 0.15
```

The analyzer now also summarizes `/dmpc/robot*/obstacle_metrics`, `/dmpc/two_robot/obstacle_thresholds`, and `/dmpc/two_robot/hold_state` when those topics exist in the bag.

## Verified obstacle-avoidance activation simulation

The obstacle-avoidance activation simulation has passed with the parameterized one-obstacle scenario.

Validated bag:

```text
/home/navid/yahboom2wd_ws/bags/two_robot_dmpc_motion_20260712_032738
```

Validated command parameters:

```text
robot1_initial_x = 1.0
robot1_initial_y = -0.7
robot2_initial_x = 1.0
robot2_initial_y = 0.7

max_linear_speed  = 0.02
max_angular_speed = 0.12
u_bound           = 0.02

d_safe = 0.65
formation_margin = 0.15
d_agent_enter = 0.68
d_agent_exit = 0.72

obstacles_enabled = true
obstacle_center_x = 1.0
obstacle_center_y = -0.33
obstacle_radius = 0.15
obstacle_margin = 0.10
d_obs_enter = 0.15
d_obs_exit = 0.25
obstacle_warning_radius = 0.35
tangential_waypoint_radius = 0.12
orbit_tangent_lookahead = 0.20
formation_hold_enabled = true
```

Analyzer result:

```text
target pair distance:                    0.8000 m
initial distance:                         1.4000 m
final distance:                           0.7623 m
final absolute formation-distance error:  0.0377 m
min distance:                             0.7091 m
minimum safety margin distance-d_safe:    0.0591 m

minimum inflated-obstacle clearance:      0.1200 m
robot1 obstacle-active samples:           24 / 594
robot2 obstacle-active samples:           15 / 594
hold active samples:                       8 / 594
```

This run is considered a real obstacle-avoidance simulation pass because obstacle activity was nonzero, the minimum obstacle clearance stayed positive, the inter-robot distance stayed above `d_safe`, and the final pair distance remained within about `4 cm` of the target formation distance.

The earlier `d_obs_enter=0.10`, `d_obs_exit=0.20`, `obstacle_warning_radius=0.25` run was a useful obstacle-enabled safety regression, but it was not a strong obstacle-activation test because no robot entered obstacle-active mode. The validated activation thresholds for the first hardware attempt are therefore:

```text
d_obs_enter = 0.15
d_obs_exit = 0.25
obstacle_warning_radius = 0.35
```

## Next validation step after the passed simulation

The next step is the hardware obstacle dry run and then a short reduced-speed hardware test. Keep the exact same obstacle geometry and thresholds as the passed simulation so that simulation and hardware remain comparable.

The first hardware dry run must use `enable_motion:=false`. Only after `/dmpc/robot*/pose_world`, `/dmpc/robot*/obstacle_metrics`, and zero `/robot*/cmd_vel` are verified should the 25-second motion-enabled test be run.

For hardware bag recording, include the obstacle and hold topics:

```text
/dmpc/robot1/obstacle_metrics
/dmpc/robot2/obstacle_metrics
/dmpc/two_robot/obstacle_thresholds
/dmpc/two_robot/hold_state
```

The 25-second hardware test passes only if:

```text
minimum inflated-obstacle clearance > 0.0 m
minimum inter-robot distance > 0.65 m
at least one robot has obstacle-active samples > 0
the robots stay inside the 2 m x 3 m field
both robots stop when the coordinator exits
```

Only after that passes should the same setup be repeated for 60 seconds.
