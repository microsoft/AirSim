import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    output = DeclareLaunchArgument(
        "output",
        default_value='log')

    publish_clock = DeclareLaunchArgument(
        "publish_clock",
        default_value='False')

    is_vulkan = DeclareLaunchArgument(
        "is_vulkan",
        default_value='True')

    host = DeclareLaunchArgument(
        "host",
        default_value='localhost')

    world_frame_id = DeclareLaunchArgument(
        "world_frame_id",
        default_value='map',
        description='World/map frame ID')

    odom_frame_id = DeclareLaunchArgument(
        "odom_frame_id",
        default_value='odom',
        description='Odometry frame ID')

    base_link_frame_id = DeclareLaunchArgument(
        "base_link_frame_id",
        default_value='base_link',
        description='Base link frame ID (drone center of gravity)')

    camera_link_frame_id = DeclareLaunchArgument(
        "camera_link_frame_id",
        default_value='camera_link',
        description='Camera link frame ID (camera mount point)')
  
    unite_imu_method = DeclareLaunchArgument(
        "unite_imu_method",
        default_value='0',
        description='IMU unification method: 0=none, 1=copy, 2=linear_interpolation')
  
    airsim_node = Node(
            package='airsim_ros_pkgs',
            executable='airsim_node',
            name='airsim_node',
            output='screen',
            parameters=[{
                'is_vulkan': True,
                # Image at 30Hz (0.0333s period)
                'update_airsim_img_response_every_n_sec': 0.0333,
                # Control loop rate
                'update_airsim_control_every_n_sec': 0.01,
                # Lidar at 100Hz
                'update_lidar_every_n_sec': 0.01,
                # IMU at 200Hz (0.005s period)
                'update_imu_every_n_sec': 0.005,
                'publish_clock': LaunchConfiguration('publish_clock'),
                'host_ip': LaunchConfiguration('host'),
                # TF frame IDs - configurable for integration with navigation stack
                'world_frame_id': LaunchConfiguration('world_frame_id'),
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'base_link_frame_id': LaunchConfiguration('base_link_frame_id'),
                'camera_link_frame_id': LaunchConfiguration('camera_link_frame_id'),
                'unite_imu_method': LaunchConfiguration('unite_imu_method')
            }])

    static_transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('airsim_ros_pkgs'), 'launch/static_transforms.launch.py')
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(output)
    ld.add_action(publish_clock)
    ld.add_action(is_vulkan)
    ld.add_action(host)
    ld.add_action(world_frame_id)
    ld.add_action(odom_frame_id)
    ld.add_action(base_link_frame_id)
    ld.add_action(camera_link_frame_id)
    ld.add_action(unite_imu_method)
  
    ld.add_action(static_transforms)
    ld.add_action(airsim_node)

    return ld
