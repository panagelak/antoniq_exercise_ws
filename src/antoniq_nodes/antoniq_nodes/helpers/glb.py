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

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf_transformations import quaternion_from_euler

node = None


def constructTransform(parent_frame,
                       child_frame,
                       pos,
                       euler_rot,
                       use_degrees=False) -> TransformStamped:
    deg_to_rad = 0.0174532925
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = float(pos[0])
    t.transform.translation.y = float(pos[1])
    t.transform.translation.z = float(pos[2])
    ros_q = None
    if use_degrees:
        ros_q = quaternion_from_euler(
            float(euler_rot[0]) * deg_to_rad,
            float(euler_rot[1]) * deg_to_rad,
            float(euler_rot[2]) * deg_to_rad)
    else:
        ros_q = quaternion_from_euler(float(euler_rot[0]), float(euler_rot[1]),
                                      float(euler_rot[2]))
    t.transform.rotation.w = ros_q[3]
    t.transform.rotation.x = ros_q[0]
    t.transform.rotation.y = ros_q[1]
    t.transform.rotation.z = ros_q[2]
    return t


def constructTransformQuat(parent_frame, child_frame, pos, quat) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = float(pos[0])
    t.transform.translation.y = float(pos[1])
    t.transform.translation.z = float(pos[2])
    t.transform.rotation.x = float(quat[0])
    t.transform.rotation.y = float(quat[1])
    t.transform.rotation.z = float(quat[2])
    t.transform.rotation.w = float(quat[3])
    return t


def constructPose(header_frame, pos, euler_rot, use_degrees=False) -> PoseStamped:
    deg_to_rad = 0.0174532925
    p = PoseStamped()
    p.header.stamp = node.get_clock().now().to_msg()
    p.header.frame_id = header_frame
    p.pose.position.x = float(pos[0])
    p.pose.position.y = float(pos[1])
    p.pose.position.z = float(pos[2])
    ros_q = None
    if use_degrees:
        ros_q = quaternion_from_euler(
            float(euler_rot[0]) * deg_to_rad,
            float(euler_rot[1]) * deg_to_rad,
            float(euler_rot[2]) * deg_to_rad)
    else:
        ros_q = quaternion_from_euler(float(euler_rot[0]), float(euler_rot[1]),
                                      float(euler_rot[2]))
    p.pose.orientation.w = ros_q[3]
    p.pose.orientation.x = ros_q[0]
    p.pose.orientation.y = ros_q[1]
    p.pose.orientation.z = ros_q[2]
    return p
