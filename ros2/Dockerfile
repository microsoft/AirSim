FROM ros:galactic-ros-base

ENV ROS_USER=ros
ENV ROS_USER_HOME=/home/ros
RUN useradd -U -r ${ROS_USER} -m -d ${ROS_USER_HOME} -s /bin/bash && \
    usermod -G users ${ROS_USER}

RUN apt-get update &&\
    apt-get install -y\
    apt-utils \
    gcc-8 g++-8 \
    ros-$ROS_DISTRO-tf2-sensor-msgs ros-$ROS_DISTRO-tf2-geometry-msgs ros-$ROS_DISTRO-mavros* \
    ros-$ROS_DISTRO-vision-opencv ros-$ROS_DISTRO-image-transport \
    libyaml-cpp-dev &&\
    echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc &&\
    rm -rf /var/lib/apt/lists/* &&\
    apt-get clean

RUN git clone https://github.com/Microsoft/AirSim.git ${ROS_USER_HOME}/AirSim

WORKDIR ${ROS_USER_HOME}/AirSim

RUN ./setup.sh && ./build.sh

RUN rm -Rf ${ROS_USER_HOME}/AirSim/ros2/src
COPY src ${ROS_USER_HOME}/AirSim/ros2/src

RUN chown -R ${ROS_USER}:${ROS_USER} ${ROS_USER_HOME}
USER ${ROS_USER}

WORKDIR ${ROS_USER_HOME}/AirSim/ros2

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/galactic/setup.bash &&\
    colcon build --symlink-install --event-handlers console_direct+ 

RUN echo "source /opt/ros/galactic/setup.bash" >> ${ROS_USER_HOME}/.bashrc
RUN echo "source ${ROS_USER_HOME}/AirSim/ros2/install/setup.bash" >> ${ROS_USER_HOME}/.bashrc

COPY entrypoint.sh ${ROS_USER_HOME}/AirSim/ros2/entrypoint.sh
ENTRYPOINT ["/bin/bash", "/home/ros/AirSim/ros2/entrypoint.sh"]
