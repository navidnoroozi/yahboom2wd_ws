# Build fix notes

This archive fixes two first-build problems:

1. `yahboom_2wd_description` now installs `rviz` only if the directory exists.
   An empty `rviz/.gitkeep` is included so the directory survives Git/copy workflows.
2. Cached Python files were removed from the source tree.

On the Raspberry Pi, always source ROS before rosdep/colcon:

```bash
source /opt/ros/humble/setup.bash
cd ~/yahboom2wd_ws
rosdep install --from-paths src -y --ignore-src --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```
