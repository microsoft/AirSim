source /opt/ros/iron/setup.bash
source /home/ros/AirSim/ros2/install/local_setup.bash

ROS2_LAUNCH_COMMAND=(ros2 launch airsim_ros_pkgs airsim_node.launch.py)

if [ "${1}" == "zv2_metadata" ]; then
    ROS2_LAUNCH_COMMAND+=("zv2_metadata:=True")
fi

${ROS2_LAUNCH_COMMAND[@]}
