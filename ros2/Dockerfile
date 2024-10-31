FROM ros:iron-ros-base

ENV ROS_USER=ros
ENV ROS_USER_HOME=/home/ros
RUN useradd -U -r ${ROS_USER} -m -d ${ROS_USER_HOME} -s /bin/bash && \
    usermod -G users ${ROS_USER}

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        lsb-release \
        rsync \
        software-properties-common \
        wget \
        apt-utils \
        gcc-11 \
        g++-11 \
        ros-${ROS_DISTRO}-tf2-sensor-msgs \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-mavros* \
        ros-${ROS_DISTRO}-vision-opencv \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-cyclonedds \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        libyaml-cpp-dev \
        libvulkan1 \
        vulkan-tools \
        clang-14 \
        clang++-14 \
        libc++-14-dev \
        libc++abi-14-dev \
        libexiv2-dev && \
    echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc &&\
    rm -rf /var/lib/apt/lists/* &&\
    apt-get clean

RUN cd ${ROS_USER_HOME} && \
    git clone --depth 1 --branch 10.1.1 https://github.com/fmtlib/fmt.git && \
    cd fmt && \
    mkdir build && \
    cd build && \
    cmake -DFMT_TEST=FALSE -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE .. && \
    make && \
    make install && \
    cd ${ROS_USER_HOME} && \
    rm -Rf fmt

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY --chown=${ROS_USER} ./ ${ROS_USER_HOME}/AirSim

WORKDIR ${ROS_USER_HOME}/AirSim

RUN ./setup.sh && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean
RUN ./build.sh --gcc

RUN chown -R ${ROS_USER}:${ROS_USER} ${ROS_USER_HOME}/AirSim
USER ${ROS_USER}

WORKDIR ${ROS_USER_HOME}/AirSim/ros2

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --event-handlers console_cohesion+

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${ROS_USER_HOME}/.bashrc
RUN echo "source ${ROS_USER_HOME}/AirSim/ros2/install/setup.bash" >> ${ROS_USER_HOME}/.bashrc

COPY --chown=${ROS_USER} ./ros2/entrypoint.sh ${ROS_USER_HOME}/AirSim/ros2/entrypoint.sh
ENTRYPOINT ["/bin/bash", "/home/ros/AirSim/ros2/entrypoint.sh"]
