#!/bin/bash
set -e

# Source ROS2 environment setup files.
source /opt/ros/humble/setup.bash
source /home/ur16_ws/install/setup.bash

# Determine the host IP and set ROS_IP at runtime.
export ROS_IP=$(hostname -I | awk '{print $1}')
echo "ROS_IP set to ${ROS_IP}"

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


# Define the parameter file path.
PARAM_FILE="/config/params.yaml"

# Check if the external parameter file exists.
if [ ! -f "$PARAM_FILE" ]; then
    echo "Error: Parameter file not found at $PARAM_FILE"
    exit 1
fi
echo "Using parameter file: $PARAM_FILE"

# Launch the ROS2 system by passing the parameter file as a launch argument.
terminator -e "ros2 launch demo_cpp_ready ur16_launch.py params_file:=$PARAM_FILE"
