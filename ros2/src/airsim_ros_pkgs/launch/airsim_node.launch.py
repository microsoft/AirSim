import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='output',
            default_value='log'
        ),
        launch.actions.DeclareLaunchArgument(
            name='publish_clock',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='is_vulkan',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='192.168.211.1' #ToDo - cahnge
        ),
        launch_ros.actions.Node(
            package='airsim_ros_pkgs',
            executable='airsim_node',
            name='airsim_node',
            output='screen', #ToDo-change to var
            parameters=[
                {
                    'is_vulkan': 'false'
                },
                {
                    'update_airsim_img_response_every_n_sec': '0.05'
                },
                {
                    'update_airsim_control_every_n_sec': '0.01'
                },
                {
                    'update_lidar_every_n_sec': '0.01'
                },
                {
                    'publish_clock': 'false' #launch.substitutions.LaunchConfiguration('publish_clock') #ToDo-change to var
                },
                {
                    'host_ip': '192.168.211.1'#launch.substitutions.LaunchConfiguration('host') #ToDo-change to var
                }
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        )#,
        # launch.actions.IncludeLaunchDescription(
        #     launch.launch_description_sources.PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory(
        #             'airsim_ros_pkgs'), 'launch/static_transforms.launch.py')
        #     )
        # )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
