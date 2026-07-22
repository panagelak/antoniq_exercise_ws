// Copyright 2026 Panagiotis Angelakis
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <string>

// Pure, ROS-free pieces of AntoniqMission::runMission()'s boustrophedon waypoint sequencing
// (see antoniq_mission_node.cpp), split out so they're unit-testable without an rclcpp::Node,
// TF buffer, or any of the action/service servers the mission depends on.
namespace antoniq_nodes
{

inline int waypointCount(int row_count)
{
  return row_count * 2;
}

// runMission() requires waypoint_count >= 2, i.e. at least one row.
inline bool isValidRowCount(int row_count)
{
  return waypointCount(row_count) >= 2;
}

// Even waypoints (2, 4, 6, ...) sit at the far end of a row, so the leg arriving there runs the
// length of the row -- entering it -- and uses the straight-line planner with obstacle
// monitoring. Odd waypoints (3, 5, 7, ...) sit across the headland, so that leg turns into the
// next row instead, via the regular costmap/recovery-aware planner.
inline bool isWithinRowLeg(int waypoint_index)
{
  return waypoint_index % 2 == 0;
}

// Row index (0-based) a given waypoint belongs to.
inline int rowForWaypoint(int waypoint_index)
{
  return (waypoint_index - 1) / 2;
}

inline std::string waypointFrameName(int waypoint_index)
{
  return "boustrophedon_waypoint_" + std::to_string(waypoint_index);
}

// Human-readable counterpart to the row/waypoint_index/waypoint_count fields published
// alongside it on MissionStatus, for display purposes (e.g. the antoniq_mission_status_rqt
// plugin). row is 0-based (see rowForWaypoint()); this reports it 1-based to match row_count.
inline std::string describeMissionStatus(int row, int row_count, int waypoint_index,
                                         int waypoint_count)
{
  std::string action = isWithinRowLeg(waypoint_index) ? "Entering" : "Turning into";
  return action + " row " + std::to_string(row + 1) + "/" + std::to_string(row_count) +
         " (waypoint " + std::to_string(waypoint_index) + "/" + std::to_string(waypoint_count) +
         ")";
}

}  // namespace antoniq_nodes
