#!/usr/bin/env python3

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

import math
import threading

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class WorkstationTFManager(Node):

    def __init__(self):
        super().__init__('workstation_tf_manager')
        # world creation params, shared with antoniq_bringup's greenhouse world
        self.declare_parameter('row_count', 3.0)
        self.declare_parameter('row_length', 4.0)
        self.declare_parameter('row_width', 1.0)
        self.declare_parameter('wall_thickness', 0.3)

        self.row_count = int(self.get_parameter('row_count').get_parameter_value().double_value)
        self.row_length = self.get_parameter('row_length').get_parameter_value().double_value
        self.row_width = self.get_parameter('row_width').get_parameter_value().double_value
        self.wall_thickness = self.get_parameter(
            'wall_thickness').get_parameter_value().double_value
        self.tf_broadcaster = TransformBroadcaster(self)
        # self.waypoints is read by the broadcast timer and mutated by the flip service; both
        # run on this single callback group so rclpy.spin()'s (single-threaded) executor can
        # never dispatch them concurrently in the first place, and the lock makes that guarantee
        # explicit/defensive rather than implicit, in case this node is ever spun with a
        # MultiThreadedExecutor or the group split later.
        self.waypoints_lock = threading.Lock()
        self.waypoints_callback_group = MutuallyExclusiveCallbackGroup()
        self.waypoints = []
        self.publish_boustrophedon_waypoints()
        self.flip_service = self.create_service(Trigger,
                                                'flip_waypoint_orientations',
                                                self.flip_waypoint_orientations_cb,
                                                callback_group=self.waypoints_callback_group)
        # Broadcast on /tf (not /tf_static) on a timer instead of latching once: RViz's TF
        # display doesn't reliably re-render a *static* transform after its content changes
        # (frames renamed/re-oriented by flip_waypoint_orientations_cb), since static frames are
        # conventionally treated as set-once. Periodic re-publishing keeps every viewer/consumer
        # continuously in sync with self.waypoints's current state instead.
        self.broadcast_timer = self.create_timer(0.02,
                                                 self.broadcast_waypoints_cb,
                                                 callback_group=self.waypoints_callback_group)

    def publish_boustrophedon_waypoints(self):
        # Same row layout formulas as turtlebot_bringup.launch.py / greenhouse.sdf.xacro,
        # in world/SDF coordinates (world origin at the field's centre).
        row_pitch = self.row_width + self.wall_thickness
        field_width = self.row_count * row_pitch + self.wall_thickness
        y_start = -field_width / 2.0 + self.wall_thickness / 2.0
        row_0_y = y_start + row_pitch / 2.0

        # turtlebot_bringup.launch.py spawns the robot at (-row_length/2, row_0_y) in world
        # coordinates with no yaw offset. The diff-drive plugin's odom, and slam_toolbox's
        # map on top of it (map_start_pose defaults to [0,0,0]), both originate there with
        # the same orientation, so a world-frame point converts to the map frame by
        # subtracting the spawn pose.
        spawn_x = -self.row_length / 2.0
        spawn_y = row_0_y

        # Waypoints sit exactly at the row boundaries (world coordinates), not offset into
        # the headland.
        near_x_world = -self.row_length / 2.0
        far_x_world = self.row_length / 2.0

        # Boustrophedon (lawnmower) pattern: row 0 runs near->far like the robot's spawn
        # heading, row 1 runs far->near, row 2 back to near->far, and so on, so consecutive
        # rows always connect start-to-end without crossing back over the row just finished.
        # Each row's start waypoint faces into the row (the direction of travel down it) and
        # its end waypoint faces out of the row (continuing that same direction, out into the
        # headland) -- for a single straight row these are the same heading, so waypoints
        # alternate in yaw by row, not by waypoint: 1 into, 2 out (row 0, yaw 0), 3 into, 4 out
        # (row 1, yaw pi), and so on.
        waypoints = []
        waypoint_number = 1
        for row_index in range(self.row_count):
            row_y_world = row_0_y + row_index * row_pitch
            forward = row_index % 2 == 0
            start_x_world, end_x_world = ((near_x_world, far_x_world) if forward else
                                          (far_x_world, near_x_world))
            yaw = 0.0 if forward else math.pi

            for x_world in (start_x_world, end_x_world):
                map_pos = [x_world - spawn_x, row_y_world - spawn_y, 0.0]
                waypoints.append(
                    self.construct_transform('map', f'boustrophedon_waypoint_{waypoint_number}',
                                             map_pos, [0.0, 0.0, yaw]))
                waypoint_number += 1

        with self.waypoints_lock:
            self.waypoints = waypoints
        self.get_logger().info(
            f'Computed {len(waypoints)} boustrophedon waypoint frames for {self.row_count} rows')

    def broadcast_waypoints_cb(self):
        now = self.get_clock().now().to_msg()
        with self.waypoints_lock:
            for t in self.waypoints:
                t.header.stamp = now
            # Broadcast the same list object under the lock, not a copy: sendTransform() only
            # reads it to serialize the message before returning, so it's fine to keep this
            # (already-cheap, non-blocking) call inside the critical section too, and doing so
            # rules out a flip landing in between "read waypoints" and "publish".
            self.tf_broadcaster.sendTransform(self.waypoints)

    def flip_waypoint_orientations_cb(self, request, response):
        with self.waypoints_lock:
            for t in self.waypoints:
                _, _, yaw = euler_from_quaternion([
                    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
                    t.transform.rotation.w
                ])
                q = quaternion_from_euler(0.0, 0.0, yaw + math.pi)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

            # Swap each row's start/end frame NAMES too (waypoint_2k+1 <-> waypoint_2k+2 for
            # every row), not just their orientation. This is what lets antoniq_mission_node
            # keep using its plain even=straight-line/odd=turn indexing after an obstacle
            # recovery mirrors the rest of the mission: the physical point the robot just turned
            # to (still looked up by its pre-swap name before this call) becomes the new
            # odd/turn-arrival frame, and its row partner becomes the new even/straight-line
            # target -- with every later row's pair swapped the same way so the mirrored
            # direction stays consistent all the way to the end. Pairs already visited get
            # swapped too, but that's inert since they're never looked up again; calling this a
            # second time later in the mission swaps everything back, which is correct too since
            # two mirrors cancel out.
            for i in range(0, len(self.waypoints) - 1, 2):
                self.waypoints[i].child_frame_id, self.waypoints[i + 1].child_frame_id = (
                    self.waypoints[i + 1].child_frame_id, self.waypoints[i].child_frame_id)
            waypoint_count = len(self.waypoints)

        response.success = True
        response.message = (
            f'Flipped orientation and swapped start/end names for {waypoint_count} waypoints')
        self.get_logger().info(response.message)
        return response

    def construct_transform(self, parent_frame, child_frame, pos, euler_rot) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        q = quaternion_from_euler(float(euler_rot[0]), float(euler_rot[1]), float(euler_rot[2]))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t


def main(args=None):
    rclpy.init(args=args)
    node = WorkstationTFManager()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
