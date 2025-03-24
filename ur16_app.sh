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

launch_app (){  
  echo $RMW_IMPLEMENTATION
  echo $ROS_DOMAIN_ID
  sleep 2
  ros2 launch demo_cpp_ready ur16_launch.py
}
# Launch the application.
launch_app