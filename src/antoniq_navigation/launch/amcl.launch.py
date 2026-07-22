from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition
import sys
import os
from launch.substitutions import Command, FindExecutable
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_ros
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    # arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_config_path = FindPackageShare("antoniq_navigation")
    pkg_nav2_bringup = FindPackageShare("nav2_bringup")
    # param files
    amcl_file = LaunchConfiguration("amcl_file").perform(context)
    nav2_file = LaunchConfiguration("nav2_file").perform(context)
    map_file = LaunchConfiguration("map_file").perform(context)
    # configs
    config_map = PathJoinSubstitution([pkg_config_path, 'maps', map_file])
    config_amcl = PathJoinSubstitution([pkg_config_path, 'config', amcl_file])
    config_nav2 = PathJoinSubstitution([pkg_config_path, 'config', nav2_file])

    # localization
    localization = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'localization_launch.py'])),
            launch_arguments={
                'namespace': namespace,
                'map': config_map,
                'use_sim_time': use_sim_time,
                'params_file': config_amcl,
    #  "use_composition": "True",
    #  "use_respawn": "True"
            }.items()),
    ])
    configured_nav2_params = RewrittenYaml(source_file=config_nav2,
                                           root_key="",
                                           param_rewrites="",
                                           convert_types=True)
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2_bringup, "launch", "navigation_launch.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_nav2_params,
            "namespace": namespace,
            "autostart": "True",
    # "use_composition": "True",
    # "use_respawn": "True"
        }.items(),
    )
    return [localization]    #[localization, navigation2_cmd]


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              choices=['true', 'false'],
                              description='Use sim time'))
    declared_arguments.append(
        DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'))
    declared_arguments.append(DeclareLaunchArgument('amcl_file', default_value='amcl.yaml'))
    declared_arguments.append(DeclareLaunchArgument('nav2_file', default_value='nav2_params.yaml'))
    declared_arguments.append(DeclareLaunchArgument('map_file', default_value='tfmap.yaml'))
    declared_arguments.append(DeclareLaunchArgument("tf_prefix", default_value="", description=""))
    declared_arguments.append(
        DeclareLaunchArgument("tf_arm_prefix", default_value="", description=""))
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
