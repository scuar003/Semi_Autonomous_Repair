#!/bin/bash
set -e

# Function to start all nodes in order.
start_nodes() {
  # Prompt user for robot IP
  read -p "Enter the robot IP address: " ROBOT_IP
  echo "Starting ur_robot_driver with robot_ip=${ROBOT_IP}..."
  
  # Launch the ur_robot_driver with RViz enabled.
  ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur16e robot_ip:=${ROBOT_IP} launch_rviz:=true \
    > /tmp/ur_robot_driver.log 2>&1 &
  DRIVER_PID=$!
  echo "ur_robot_driver started (PID: ${DRIVER_PID}). Check /tmp/ur_robot_driver.log for details."

  # Allow time for driver initialization
  sleep 5

  # Launch static transform publisher for camara_link -> camera_link
  echo "Starting static transform publisher for camara_link -> camera_link..."
  ros2 run tf2_ros static_transform_publisher \
    0.042 0.0 0.036 1.570 3.19 -0.02 camara_link camera_link \
    > /tmp/tf_camara_link.log 2>&1 &
  TF1_PID=$!
  echo "Static transform for camera started (PID: ${TF1_PID}). Log: /tmp/tf_camara_link.log"
  
  # Launch static transform publisher for tool0 -> laser
  echo "Starting static transform publisher for tool0 -> laser..."
  ros2 run tf2_ros static_transform_publisher \
    -0.13 -0.03 0.04 0 1.57 3.14 tool0 laser \
    > /tmp/tf_laser.log 2>&1 &
  TF2_PID=$!
  echo "Static transform for laser started (PID: ${TF2_PID}). Log: /tmp/tf_laser.log"
  
  # Allow transforms to settle
  sleep 2
  
  # Launch the demo nodes
  echo "Starting demo launch..."
  ros2 launch demo_cpp_ready demo_launch.py \
    > /tmp/demo_launch.log 2>&1 &
  DEMO_PID=$!
  echo "Demo launch started (PID: ${DEMO_PID}). Log: /tmp/demo_launch.log"
  
  echo "All nodes have been launched. Press Ctrl+C to shut them down."
  
  # Wait indefinitely for the background processes.
  wait
}

# If you want to add a reset option, you could check for a parameter here.
# For now, we simply start the nodes.
start_nodes
