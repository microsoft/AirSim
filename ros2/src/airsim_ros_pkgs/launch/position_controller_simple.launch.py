import launch
from launch_ros.actions import Node


def generate_launch_description():
    ld = launch.LaunchDescription([
            Node(
                package='airsim_ros_pkgs',
                executable='pd_position_controller_simple_node',
                name='pid_position_node',
                output='screen',
                parameters=[{
                    'update_control_every_n_sec': 0.01,
                    
                    'kp_x': 0.30,
                    'kp_y': 0.30,
                    'kp_z': 0.30,
                    'kp_yaw': 0.30,
                    
                    'kd_x': 0.05,
                    'kd_y': 0.05,
                    'kd_z': 0.05,
                    'kd_yaw': 0.05,
                   
                    'reached_thresh_xyz': 0.1,
                    'reached_yaw_degrees': 5.0
                }
            ]
        )
    ])

    return ld
