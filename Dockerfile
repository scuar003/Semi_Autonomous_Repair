FROM ros:humble-ros-base

# Set ROS_DISTRO and workspace environment variables
ENV ROS_DISTRO=humble
ENV WORKSPACE=/home/ur16_ws/
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ROS_DOMAIN_ID=27
ENV ZENOH_ROUTER_CONFIG_URI=/home/ur16_ws/routerconfig.json5

# Set working directory
WORKDIR ${WORKSPACE}

# Copy the combined repositories file into the container
COPY combined.repos ${WORKSPACE}/combined.repos

# Install essential system packages and clone repositories
RUN rm -rf /var/lib/apt/lists/* && \
    sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirror.kku.ac.th/ubuntu/|g' /etc/apt/sources.list && \
    apt-get -o Acquire::AllowReleaseInfoChange=true update && \
    apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-rviz2 \
    ros-humble-rmw-zenoh-cpp* \
    ros-humble-zenoh* \
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

# Clone, build, and install the ur_rtde library into ${WORKSPACE}libs_/
# Clone, build, and install the ur_rtde library into ${WORKSPACE}libs_/
# --- clone, modernise pybind11, build & install ur_rtde ----------------------
    RUN mkdir -p ${WORKSPACE}libs_ && \
    cd ${WORKSPACE}libs_ && \
    git clone https://gitlab.com/sdurobotics/ur_rtde.git && \
    cd ur_rtde && \
    git submodule update --init --recursive && \
    # --- upgrade the pybind11 submodule so it knows modern CMake -------------
    cd pybind11 && git checkout v2.12.0 && cd .. && \
    # -------------------------------------------------------------------------
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_CPP_STANDARD=17 && \
    make -j$(nproc) && make install && ldconfig

# Upgrade CMake to >= 3.24 for Open3D compatibility
RUN apt-get update && apt-get install -y wget gnupg software-properties-common lsb-release && \
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | \
    gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && \
    apt-get install -y kitware-archive-keyring cmake && \
    rm -rf /var/lib/apt/lists/*

# Install Open3D C++ dependencies
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libtbb-dev \
    libflann-dev \
    libjsoncpp-dev \
    libpng-dev \
    libjpeg-dev \
    libtiff-dev \
    libglfw3-dev \
    libglew-dev \
    liblz4-dev \
    libopenblas-dev \
    libsuitesparse-dev \
    libx11-dev \
    libxi-dev \
    libxxf86vm-dev \
    libxcursor-dev \
    libxrandr-dev \
    libxinerama-dev \
    libgtk-3-dev \
    python3-dev \
    clang \
    libc++-dev \
    libc++abi-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install Open3D from source
RUN git clone --recursive https://github.com/isl-org/Open3D.git /tmp/Open3D && \
    cd /tmp/Open3D && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DBUILD_SHARED_LIBS=ON \
             -DBUILD_PYTHON_MODULE=OFF \
             -DBUILD_UNIT_TESTS=OFF \
             -DBUILD_EXAMPLES=OFF \
             -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j$(nproc) && \
    make install && \
    cd / && rm -rf /tmp/Open3D





# Edit UR description and rviz config 
RUN git clone https://github.com/scuar003/UR16_repair_setup.git /tmp/UR16_repair_setup && \
    cp /tmp/UR16_repair_setup/lidar.STL ${WORKSPACE}/src/Universal_Robots_ROS2_Description/meshes/ur16e/visual/ && \
    cp /tmp/UR16_repair_setup/ur.urdf.xacro ${WORKSPACE}/src/Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro && \
    cp /tmp/UR16_repair_setup/view_robot.rviz ${WORKSPACE}/src/Universal_Robots_ROS2_Description/rviz/view_robot.rviz && \
    cp /tmp/UR16_repair_setup/routerconfig.json5 ${WORKSPACE}/routerconfig.json5 && \
    rm -rf /tmp/UR16_repair_setup

# Use rosdep to install any remaining system dependencies and build the workspace
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && rosdep update && \
    rosdep install --rosdistro humble --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args -DCMAKE_PREFIX_PATH=/usr/local/lib/cmake/Open3D --packages-skip ur_rtde

# Copy the startup script and make it executable
COPY ur16_app.sh /ur16_app.sh
RUN chmod +x /ur16_app.sh

# Default command: launch the startup script.
ENTRYPOINT ["/ur16_app.sh"]
