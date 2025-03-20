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

# Use rosdep to install any remaining system dependencies and build the workspace
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && rosdep update && \
    rosdep install --rosdistro humble --from-paths src --ignore-src -r -y && \
    colcon build



# Default command: source the workspace setup and open a bash shell.
CMD ["/bin/bash", "-c", ". ${WORKSPACE}/install/setup.bash && exec bash"]
