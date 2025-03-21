#!/bin/bash
set -e

# Source ROS2 environment setup files.
source /opt/ros/humble/setup.bash
source /home/ur16_ws/install/setup.bash

# Determine the host IP and set ROS_IP at runtime.
export ROS_IP=$(hostname -I | awk '{print $1}')
echo "ROS_IP set to ${ROS_IP}"

# Check if ROBOT_IP is provided as an environment variable.
if [ -z "$ROBOT_IP" ]; then
    echo "Error: ROBOT_IP environment variable is not set."
    echo "Please provide the robot IP when running the container (e.g., docker run -e ROBOT_IP=192.168.56.101 ...)."
    exit 1
fi

# Array to keep track of background process PIDs.
PIDS=()

# Cleanup function to kill all child processes.
cleanup() {
  echo "Caught termination signal. Shutting down processes..."
  for pid in "${PIDS[@]}"; do
    echo "Killing process $pid"
    kill -TERM "$pid" 2>/dev/null
  done
  wait
  exit 0
}

# Trap SIGINT and SIGTERM signals.
trap cleanup SIGINT SIGTERM

# Function to start all nodes.
start_nodes() {
  echo "Starting ur_robot_driver with robot_ip=${ROBOT_IP}..."
  ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur16e robot_ip:=${ROBOT_IP} launch_rviz:=true &
  DRIVER_PID=$!
  PIDS+=($DRIVER_PID)
  echo "ur_robot_driver started (PID: ${DRIVER_PID})."

  # Allow time for driver initialization.
  sleep 5

  echo "Starting static transform publisher for camara_link -> camera_link..."
  ros2 run tf2_ros static_transform_publisher \
    0.042 0.0 0.036 1.570 3.19 -0.02 camara_link camera_link &
  TF1_PID=$!
  PIDS+=($TF1_PID)
  echo "Static transform for camera started (PID: ${TF1_PID})."
  
  echo "Starting static transform publisher for tool0 -> laser..."
  ros2 run tf2_ros static_transform_publisher \
    -0.13 -0.03 0.04 0 1.57 3.14 tool0 laser &
  TF2_PID=$!
  PIDS+=($TF2_PID)
  echo "Static transform for laser started (PID: ${TF2_PID})."
  
  # Allow transforms to settle.
  sleep 2
  
  echo "Starting demo launch..."
  ros2 launch demo_cpp_ready demo_launch.py &
  DEMO_PID=$!
  PIDS+=($DEMO_PID)
  echo "Demo launch started (PID: ${DEMO_PID})."
  
  echo "All nodes have been launched."
  echo "To view real-time ROS2 logs in another terminal, run:"
  echo "docker logs -f <container_id>"
  
  # Wait indefinitely for the background processes.
  wait
}

# Start all nodes.
start_nodes
