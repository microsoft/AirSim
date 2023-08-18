import os
from launch import LaunchDescription
from launch_ros.actions import Node
current_directory = os.path.dirname(os.path.abspath(__file__))
parameters = [os.path.join(current_directory, os.pardir, 'param', 'pd_control_parameters.yaml')]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='airsim_ros_pkgs',
            executable='pd_position_controller_simple_topic_node',
            name='pid_position_node',
            output='screen',
            parameters=[parameters],
            remappings=[("~/drone_odometry", "/airsim_node/Chaser/odom_local_ned"),
                        ("~/local_goal_pose", "/los_keeper/local_goal_pose")]
        )
    ])