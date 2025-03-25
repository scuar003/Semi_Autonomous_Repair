FROM ros:humble-ros-base

# Set ROS_DISTRO and workspace environment variables
ENV ROS_DISTRO=humble
ENV WORKSPACE=/home/ur16_ws/
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=10

# Set working directory
WORKDIR ${WORKSPACE}

# Copy the combined repositories file into the container
COPY combined.repos ${WORKSPACE}/combined.repos

# Install essential system packages and clone repositories
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-rviz2 \
    ros-humble-rmw-cyclonedds-cpp* \
    ros-humble-cyclonedds* \
    python3-vcstool \
    libboost-all-dev \
    ros-humble-test-msgs \
    libcap-dev \
    ros-humble-filters \
    build-essential \
    cmake \
    git \
    libncurses-dev \ 
    python3-pip \
    terminator \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p ${WORKSPACE}src \
    && vcs import ${WORKSPACE}src < ${WORKSPACE}combined.repos

RUN pip3 install --upgrade pip && \
    pip3 install numpy open3d
# Clone, build, and install the ur_rtde library into ${WORKSPACE}libs_/
RUN mkdir -p ${WORKSPACE}libs_ && \
    cd ${WORKSPACE}libs_ && \
    git clone https://gitlab.com/sdurobotics/ur_rtde.git && \
    cd ur_rtde && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    make install



# Edit UR description and rviz config 
RUN git clone https://github.com/scuar003/UR16_repair_setup.git /tmp/UR16_repair_setup && \
    cp /tmp/UR16_repair_setup/lidar.STL ${WORKSPACE}/src/Universal_Robots_ROS2_Description/meshes/ur16e/visual/ && \
    cp /tmp/UR16_repair_setup/ur.urdf.xacro ${WORKSPACE}/src/Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro && \
    cp /tmp/UR16_repair_setup/view_robot.rviz ${WORKSPACE}/src/Universal_Robots_ROS2_Description/rviz/view_robot.rviz && \
    rm -rf /tmp/UR16_repair_setup

# Use rosdep to install any remaining system dependencies and build the workspace
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && rosdep update && \
    rosdep install --rosdistro humble --from-paths src --ignore-src -r -y && \
    colcon build

# Copy the startup script and make it executable
COPY ur16_app.sh /ur16_app.sh
RUN chmod +x /ur16_app.sh

# Optional: set a default (empty) ROBOT_IP environment variable.
# Users should pass the robot IP at runtime.
ENV ROBOT_IP=""


# Default command: launch the startup script.
ENTRYPOINT ["/ur16_app.sh"]
