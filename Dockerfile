FROM ros:humble-ros-base

# Set ROS_DISTRO and workspace environment variables
ENV ROS_DISTRO=humble
ENV WORKSPACE=/home/ur16_ws/

# Set working directory
WORKDIR ${WORKSPACE}

# Install essential system packages
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
    && rm -rf /var/lib/apt/lists/*

# Copy the combined repositories file into the container
COPY combined.repos ${WORKSPACE}/combined.repos

# Create the workspace source folder and import repositories
RUN mkdir -p src && vcs import src < ${WORKSPACE}/combined.repos

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


# Configure ROS2 comms 
RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc && \
    echo 'export ROS_DOMAIN_ID=10' >> ~/.bashrc


# Determines HOST IP and passes through to the docker
RUN HOST_IP=$(hostname -I | awk '{print $1}') && \
    export ROS_IP=${HOST_IP} && \
    echo "Setting ROS_IP to ${ROS_IP}"

#Edit UR description and rviz config 
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

COPY ur16_app.sh /ur16_app.sh
RUN chmod +x /ur16_app.sh




# Default command: source the workspace setup and open a bash shell.
CMD ["./ur16_app.sh"]
