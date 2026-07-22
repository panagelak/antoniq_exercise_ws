import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time == 'true'
        }],
        output='screen',
    )

    return [rviz2]


def generate_launch_description():
    bringup_share = get_package_share_directory('antoniq_bringup')
    default_rviz_config = os.path.join(bringup_share, 'rviz', 'waffle.rviz')

    declared_arguments = [
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use the Gazebo /clock topic as ROS time'),
        DeclareLaunchArgument('rviz_config',
                              default_value=default_rviz_config,
                              description='RViz2 config file to load'),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
