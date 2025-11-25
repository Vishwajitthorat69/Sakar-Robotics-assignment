# ROS2 Mobile Manipulator - Full GitHub-ready Solution (A + B)

This repository contains a complete solution prepared for the ROS2 Mobile Manipulator assessment.
It includes two arm options so you can choose what runs best on your machine:

- **Option A**: Simple 3-DOF custom arm (stable, lightweight) — `urdf/arm_3dof.xacro`
- **Option B**: Doosan-style arm mounted on TurtleBot3 (realistic) — `urdf/doosan_mount.xacro`

The original assignment PDF is included in `docs/` for reference:
`/mnt/data/ROS2_Mobile_Manipulator_Assignment.pdf`

## Quick start (recommended)
1. Copy this folder into your ROS2 workspace: `~/ros2_ws/src/`
2. From workspace root:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --packages-select ros2_mobile_manipulator_full
   source install/setup.bash
   ```
3. Launch Gazebo & spawn robot (choose arm option via launch arg):
   ```bash
   ros2 launch ros2_mobile_manipulator_full spawn_robot.launch.py arm:="A"
   # or arm:="B" to use Doosan mount
   ```
4. In another terminal start mission control:
   ```bash
   ros2 launch ros2_mobile_manipulator_full mission_control.launch.py
   ```

## Contents
- `urdf/` - both arm URDFs and combined TB3 URDFs
- `launch/` - spawn, bringup, mission control launch files
- `src/` - C++ mission control and mock arm nodes
- `config/` - Nav2 and sensors placeholders
- `.github/workflows/` - GitHub Actions CI to run `colcon build`
- `docs/ROS2_Mobile_Manipulator_Assignment.pdf` - original assignment (local file included)

## Notes
- The package is prepared to work on ROS2 Jazzy (Ubuntu 24.04) or adapt it to Humble by changing package names in README.
- Option B (Doosan) is more realistic but heavier. If Gazebo is unstable, use Option A.


## Additional components added in this final version
- MoveIt2 minimal config and example pick/place node: `src/pick_place_moveit.cpp` and `moveit_config/`
- Nav2 bringup placeholder launch: `launch/nav2_bringup.launch.py` (replace with your nav2_bringup)
- Behavior Tree XML for mission sequencing: `behavior_tree/mission_bt.xml`
- SLAM Toolbox launch placeholder: `launch/slam_toolbox.launch.py`
- Teleop launch (keyboard): `launch/teleop.launch.py`
- RViz config placeholder: `rviz/mobile_manipulator.rviz`
- Demo world placeholder: `worlds/industrial_demo.world`
- Demo recording script: `scripts/make_demo_gif.sh` (uses ffmpeg)

### How to run everything (recommended order)
1. Source ROS2 and your workspace:
   ```bash
   source /opt/ros/jazzy/setup.bash   # or your ROS distro
   source ~/ros2_ws/install/setup.bash
   ```
2. Launch Gazebo + robot (choose arm A or B):
   ```bash
   ros2 launch ros2_mobile_manipulator_full spawn_robot.launch.py arm:=A
   ```
3. In new terminal, start SLAM (if you want to build a map):
   ```bash
   ros2 launch ros2_mobile_manipulator_full slam_toolbox.launch.py
   ```
4. Start Nav2 (use your nav2_bringup):
   ```bash
   ros2 launch ros2_mobile_manipulator_full nav2_bringup.launch.py
   ```
5. Start MoveIt2 (requires MoveIt2 installed and the robot_description param set):
   ```bash
   ros2 run moveit_ros_move_group move_group
   ros2 run ros2_mobile_manipulator_full pick_place_moveit
   ```
6. Start mission controller + mock arm or MoveIt pick/place:
   ```bash
   ros2 launch ros2_mobile_manipulator_full mission_control.launch.py
   ```
7. Teleoperate if needed:
   ```bash
   ros2 launch ros2_mobile_manipulator_full teleop.launch.py
   ```
8. Record demo GIF:
   ```bash
   ./scripts/make_demo_gif.sh
   ```
