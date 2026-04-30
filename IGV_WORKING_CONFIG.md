# IGV Working Config Reference

## Main launch
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch igv_bringup real_lane_test.launch.py

---

## Current key files

### real lane launch
~/ros2_ws/src/igv_bringup/launch/real_lane_test.launch.py

### behavior manager
~/ros2_ws/src/igv_behavior/igv_behavior/behavior_manager.py

### segformer lane node
~/ros2_ws/src/igv_perception/igv_perception/segformer_lane_node.py

### simple test obstacle detector
~/ros2_ws/src/igv_perception/igv_perception/obstacle_detector_node.py

### cloud debug / filtered cloud / camera FOV trapezoid
~/ros2_ws/src/igv_perception/igv_perception/cloud_debug_filter_node.py

### robot URDF
~/ros2_ws/src/igv_description/urdf/igv.urdf

### RViz config
~/ros2_ws/src/igv_bringup/rviz/igv_default.rviz

### odom stub config
~/ros2_ws/src/igv_localization/config/odom_stub.yaml

---

## Current working capabilities

- ZED ROS2 camera input works
- SegFormer lane node runs from local model
- /lane_estimate publishes
- /lanes/point_cloud publishes from true ZED registered cloud
- simple obstacle detector works
- /obstacle_summary publishes
- /obstacles/box publishes
- /obstacles/points publishes
- /debug/filtered_cloud publishes
- /debug/camera_fov_ground publishes
- behavior manager states work:
  - LANE_KEEP
  - STOP
  - WAIT_FOR_CLEAR
  - SEARCH_LANE
- /cmd_vel publishes correctly
- odom stub publishes /odom
- odom -> base_link TF works

---

## Current behavior states

### LANE_KEEP
- lane visible and fresh
- no blocking obstacle

### STOP
- obstacle inside detector box

### WAIT_FOR_CLEAR
- obstacle just cleared
- short hold before moving again

### SEARCH_LANE
- lane invalid or stale

---

## Current obstacle detector idea

This is only the current simple test detector.

Input:
- /zed/zed_node/point_cloud/cloud_registered

Main outputs:
- /obstacle_summary
- /obstacles/box
- /obstacles/points

Important:
Future custom obstacle detector should keep publishing:
- topic: /obstacle_summary
- type: igv_interfaces/msg/ObstacleSummary

That way behavior code does not need to change.

---

## Current lane detector idea

Input:
- /zed/zed_node/rgb/color/rect/image

Main outputs:
- /lane_estimate
- /lanes/point_cloud

Important:
Current lane cloud uses true ZED registered point cloud extraction.

---

## Current odom setup

Node:
- odom_stub

Config file:
- ~/ros2_ws/src/igv_localization/config/odom_stub.yaml

Important:
This is still placeholder odometry.

---

## Important RViz topics

### full ZED cloud
/zed/zed_node/point_cloud/cloud_registered

### lane cloud
/lanes/point_cloud

### obstacle box
/obstacles/box

### obstacle points
/obstacles/points

### filtered debug cloud
/debug/filtered_cloud

### camera FOV ground trapezoid
/debug/camera_fov_ground

---

## Helpful test commands

### behavior state
ros2 topic echo /behavior_state

### command velocity
ros2 topic echo /cmd_vel

### lane estimate
ros2 topic echo /lane_estimate

### obstacle summary
ros2 topic echo /obstacle_summary

### lane cloud rate
ros2 topic hz /lanes/point_cloud

### obstacle points rate
ros2 topic hz /obstacles/points

### filtered cloud rate
ros2 topic hz /debug/filtered_cloud

### tf check
ros2 run tf2_ros tf2_echo odom base_link

---

## Rebuild cheat sheet

### if editing bringup or RViz
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select igv_bringup
source ~/ros2_ws/install/setup.bash

### if editing behavior manager
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select igv_behavior
source ~/ros2_ws/install/setup.bash

### if editing lane / obstacle / debug perception nodes
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select igv_perception
source ~/ros2_ws/install/setup.bash

### if editing URDF
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select igv_description
source ~/ros2_ws/install/setup.bash

### if editing localization
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select igv_localization
source ~/ros2_ws/install/setup.bash

---

## Notes
- Current obstacle detector is for testing/debugging only
- Future custom obstacle detector should replace the simple detector but keep the same output interface
- Current odom source is still placeholder
- RViz is currently considered usable enough for debugging
