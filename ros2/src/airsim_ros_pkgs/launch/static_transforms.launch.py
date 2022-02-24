
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ned_to_enu_pub',
            arguments=['0', '0', '0', '1.57', '0', '3.14', 'world_ned', 'world_enu']#, '100']
        )
    ])
    return ld
