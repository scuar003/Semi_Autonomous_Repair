# UR16 Repair Platform Integration 

# Official ROS2 Humble base image
FROM ros:humble-ros-base 


# Working directory 

WORKDIR /root/ur16_ws


# Update package lists, install build tools and rviz2 

RUN apt-get update && apt-get install -y \
	python3-colcon-common-extensions \
	ros-humble-rviz2 \
	&& rm -rf /var/lib/apt/lists/*


# Build ROS2 workspace 

RUN  . /opt/ros/humble/setup.sh && colcon build

# default command: source the workspace setup and open a bash shell 
CMD ["/bin/bash", "-c", ". install/setup.bash && bash"]
