import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('airsim_ros_pkgs')
    default_rviz_path = os.path.join(pkg_share, 'rviz/default.rviz')

    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_path]
        )
    ])
    return ld