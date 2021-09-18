
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription([

        DeclareLaunchArgument(
        "max_vel_vert_abs",
        default_value='10.0'),
        
        DeclareLaunchArgument(
        "max_vel_horz_abs",
        default_value='0.5'),

        DeclareLaunchArgument(
        "max_yaw_rate_degree",
        default_value='1.0'),

        DeclareLaunchArgument(
        "gimbal_front_center_max_pitch",
        default_value='40.0'),

        DeclareLaunchArgument(
        "gimbal_front_center_min_pitch",
        default_value='-130.0'),

        DeclareLaunchArgument(
        "gimbal_front_center_max_yaw",
        default_value='320.0'),

        DeclareLaunchArgument(
        "gimbal_front_center_min_yaw",
        default_value='-320.0'),

        DeclareLaunchArgument(
        "gimbal_front_center_min_yaw",
        default_value='20.0'),

        DeclareLaunchArgument(
        "gimbal_front_center_min_yaw",
        default_value='-20.0')

    ])

    return ld
