from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution


def generate_launch_description():
    pkg_config_path = FindPackageShare("antoniq_navigation")
    config_nav2 = PathJoinSubstitution([pkg_config_path, 'config', "nav2_params.yaml"])

    return LaunchDescription([
    # Lifecycle manager to manage the local costmap node
        Node(package='nav2_lifecycle_manager',
             executable='lifecycle_manager',
             name='lifecycle_manager_local_costmap',
             output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'autostart': True,
                 'node_names': ['local_costmap']
             }]),

    # Local costmap server
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            name='local_costmap',
            output='screen',
            parameters=[
                {
                    'use_sim_time': False
                },
    # Replace with your local costmap config path
                config_nav2
            ],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])
    ])
