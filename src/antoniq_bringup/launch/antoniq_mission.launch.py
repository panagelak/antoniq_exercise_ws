from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    row_count = int(LaunchConfiguration('row_count').perform(context))

    antoniq_mission_node = Node(
        package='antoniq_nodes',
        executable='antoniq_mission_node',
        name='antoniq_mission_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time == 'true',
            'row_count': row_count,
        }],
    )

    return [antoniq_mission_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use the Gazebo /clock topic as ROS time'),
        DeclareLaunchArgument(
            'row_count',
            default_value='4',
            description=
            'Number of crop-row corridors to traverse; must match the row_count the world and its boustrophedon_waypoint_* TF frames were generated with'
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
