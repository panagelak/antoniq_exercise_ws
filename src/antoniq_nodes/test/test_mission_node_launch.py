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

import unittest

import launch
import launch_ros.actions

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

import pytest


def get_mission_node_action(row_count):
    return launch_ros.actions.Node(
        package='antoniq_nodes',
        executable='antoniq_mission_node',
        name='antoniq_mission_node',
        parameters=[{
            'row_count': row_count,
            'use_sim_time': False,
        }],
        output='screen',
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # No process declared here: antoniq_mission_node terminates on its own mid-test (see below),
    # so each test launches/tears down its own instance via launch_testing.tools.launch_process
    # instead of sharing one process across the launch's whole active-test phase.
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestMissionNodeInvalidRowCount(unittest.TestCase):

    def test_rejects_zero_row_count(self, launch_service, proc_info, proc_output):
        # row_count=0 -> waypoint_count=0 < 2, which runMission() rejects before it ever waits
        # on rotate_heading_server/obstacle_monitor/navigate_to_pose -- none of which this test
        # starts. That makes the node's fast-fail path exercisable as a self-contained launch
        # test with no nav2/gazebo/TF stack running alongside it.
        proc_action = get_mission_node_action(row_count=0)
        with launch_testing.tools.launch_process(launch_service, proc_action, proc_info,
                                                 proc_output):
            proc_info.assertWaitForStartup(process=proc_action, timeout=10)
            # RCLCPP_ERROR goes to stderr under the default rcutils logging config.
            proc_output.assertWaitFor('row_count must be >= 1 (got 0)',
                                      process=proc_action,
                                      timeout=10,
                                      stream='stderr')
            # main() returns right after logging that; wait for the process to actually exit on
            # its own rather than let this "with" block's teardown (SIGINT) race it and turn a
            # clean exit(1) into a signal-killed exit code.
            proc_info.assertWaitForShutdown(process=proc_action, timeout=10)
        launch_testing.asserts.assertExitCodes(proc_info,
                                               process=proc_action,
                                               allowable_exit_codes=[1])
