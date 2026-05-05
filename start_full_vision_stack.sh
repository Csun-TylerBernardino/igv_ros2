#!/usr/bin/env bash
set -eo pipefail

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

cleanup() {
  echo
  echo "Stopping full vision stack..."

  if [[ -n "${COSTMAP_PID:-}" ]] && kill -0 "$COSTMAP_PID" 2>/dev/null; then
    kill "$COSTMAP_PID" 2>/dev/null || true
  fi

  if [[ -n "${MAIN_PID:-}" ]] && kill -0 "$MAIN_PID" 2>/dev/null; then
    kill "$MAIN_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting main vision stack..."
ros2 launch igv_bringup real_lane_test.launch.py &
MAIN_PID=$!

sleep 6

echo "Starting obstacle costmap..."
ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args --params-file ~/ros2_ws/src/igv_bringup/config/obstacle_layer_params.yaml &
COSTMAP_PID=$!

echo "Waiting for costmap node..."
for i in {1..15}; do
  if ros2 node list | grep -q "/costmap/costmap"; then
    break
  fi
  sleep 1
done

echo "Configuring costmap..."
for i in {1..10}; do
  if ros2 lifecycle set /costmap/costmap configure; then
    break
  fi
  sleep 1
done

sleep 1

echo "Activating costmap..."
for i in {1..10}; do
  if ros2 lifecycle set /costmap/costmap activate; then
    break
  fi
  sleep 1
done

echo
echo "Full vision stack is running."
echo "Main launch PID: $MAIN_PID"
echo "Costmap PID: $COSTMAP_PID"
echo "Press Ctrl+C in this terminal to stop everything."
echo

wait "$MAIN_PID"
