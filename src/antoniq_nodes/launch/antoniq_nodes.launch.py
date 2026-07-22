# Copyright 2026 Panagiotis Angelakis
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration('use_sim_time')

    task_id = {'task_id': int(LaunchConfiguration('task_id').perform(context))}
    mode = {'mode': LaunchConfiguration('mode').perform(context)}

    dir_config = PathJoinSubstitution([FindPackageShare('antoniq_demo'), 'config'])
    config_demo = ParameterFile([dir_config, '/demo_config.yaml'], allow_substs=True)

    main_demo_node = Node(
        package='antoniq_demo',
        executable='main_test_node',
        output='screen',
        name='antoniq_demo',
        parameters=[use_sim_time, task_id, mode, config_demo],
    )

    return [main_demo_node]


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
    ))    # false for standaloneZ
    declared_arguments.append(DeclareLaunchArgument(
        'task_id',
        default_value='-1',
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'mode',
        default_value='real',
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'namespace',
        default_value='/',
    ))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
