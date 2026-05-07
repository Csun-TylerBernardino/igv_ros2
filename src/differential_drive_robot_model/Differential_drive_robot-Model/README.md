# Differential Drive Robot – ROS 2 Humble / Gazebo Classic 11

A minimal differential-drive robot simulation for **ROS 2 Humble** and
**Gazebo Classic 11** (`gazebo_ros_pkgs`).

---

## Prerequisites

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2
```

---

## Build

```bash
# Create (or reuse) your workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy or clone this project here so the three packages are under src/
# e.g. src/differential_drive_robot_description/
#      src/differential_drive_robot_gazebo/
#      src/differential_drive_robot_control/

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Launch options

### 1 – URDF viewer only (no Gazebo)
Quickly check the model geometry and TF tree with a slider to move the wheels.
```bash
ros2 launch differential_drive_robot_description rviz_visualize.launch.py
```

### 2 – Gazebo only (empty world)
```bash
ros2 launch differential_drive_robot_gazebo gazebo_visualize.launch.py
```

### 3 – Gazebo with custom world (recommended)
```bash
ros2 launch differential_drive_robot_gazebo differential_drive_robot_gazebo.launch.py
```

### 4 – Gazebo + RViz2 side-by-side
```bash
ros2 launch differential_drive_robot_gazebo differential_drive_robot_rviz_gazebo.launch.py
```

### 5 – Keyboard teleoperation (separate terminal)
```bash
# After any Gazebo launch is running:
ros2 launch differential_drive_robot_control teleop.launch.py
# or directly:
ros2 run differential_drive_robot_control teleop
```

Keys: `i`=forward, `,`=back, `j`/`l`=rotate, `k`/space=stop, `q`/`z`=speed up/down.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Gazebo window opens then disappears | Plugin not found | Run `gazebo --verbose` and check the terminal for `[Err]` lines. Ensure `ros-humble-gazebo-ros-pkgs` is installed and ROS is sourced. |
| Robot spawns underground / explodes | Wrong z_pose | Use `z_pose:=0.12` (default). The chassis origin is at wheel-axle height so 0.12 m puts the wheels just above ground. |
| `/odom` not publishing | Plugin not loaded | Check `gzserver` output for the diff-drive plugin loading message. |
| RViz shows "no TF data" | RSP not running or `use_sim_time` mismatch | Ensure `robot_state_publisher` is running and Gazebo's `/clock` topic is active. |
| Wheels spin but robot doesn't move | Caster friction too high | Should be fixed – casters are now zero-friction in the `.gazebo` file. |

---

## Package layout

```
Differential_drive_robot-Model/
├── differential_drive_robot_description/   # URDF/xacro + RViz config
│   ├── urdf/
│   │   ├── differential_drive_robot.xacro  # Main robot model
│   │   ├── differential_drive_robot.gazebo # Gazebo plugins + SDF friction
│   │   ├── xacro_variables.xacro           # All dimensions and masses
│   │   └── materials.xacro                 # RViz material colours
│   ├── rviz/urdf.rviz
│   └── launch/rviz_visualize.launch.py
├── differential_drive_robot_gazebo/        # Simulation launch + world
│   ├── launch/
│   │   ├── gazebo_visualize.launch.py              # Empty world
│   │   ├── gazebo_world_visualize.launch.py        # Custom world
│   │   ├── differential_drive_robot_gazebo.launch.py       # Top-level
│   │   └── differential_drive_robot_rviz_gazebo.launch.py  # + RViz
│   └── world/world.world
└── differential_drive_robot_control/       # Keyboard teleop
    ├── nodes/teleop.py
    └── launch/teleop.launch.py
```
