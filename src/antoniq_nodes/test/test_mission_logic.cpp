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

#include <gtest/gtest.h>

#include "antoniq_nodes/mission_logic.hpp"

using antoniq_nodes::describeMissionStatus;
using antoniq_nodes::isValidRowCount;
using antoniq_nodes::isWithinRowLeg;
using antoniq_nodes::rowForWaypoint;
using antoniq_nodes::waypointCount;
using antoniq_nodes::waypointFrameName;

TEST(MissionLogic, WaypointCountIsTwicePerRow)
{
  EXPECT_EQ(waypointCount(1), 2);
  EXPECT_EQ(waypointCount(4), 8);
  EXPECT_EQ(waypointCount(0), 0);
}

TEST(MissionLogic, RowCountMustYieldAtLeastOneRow)
{
  EXPECT_FALSE(isValidRowCount(0));
  EXPECT_FALSE(isValidRowCount(-1));
  EXPECT_TRUE(isValidRowCount(1));
  EXPECT_TRUE(isValidRowCount(4));
}

// Mirrors the even=within-row-leg/odd=turn-leg alternation runMission() drives off of: legs
// landing on an even waypoint run the length of a row (straight-line planner + obstacle
// monitor), legs landing on an odd waypoint turn through the headland into the next row.
TEST(MissionLogic, EvenWaypointsAreWithinRowLegs)
{
  EXPECT_TRUE(isWithinRowLeg(2));
  EXPECT_TRUE(isWithinRowLeg(4));
  EXPECT_TRUE(isWithinRowLeg(100));
}

TEST(MissionLogic, OddWaypointsAreTurnLegs)
{
  EXPECT_FALSE(isWithinRowLeg(3));
  EXPECT_FALSE(isWithinRowLeg(5));
  EXPECT_FALSE(isWithinRowLeg(101));
}

// Waypoints 1,2 belong to row 0; 3,4 to row 1; 5,6 to row 2; and so on.
TEST(MissionLogic, RowForWaypointGroupsWaypointsInPairs)
{
  EXPECT_EQ(rowForWaypoint(1), 0);
  EXPECT_EQ(rowForWaypoint(2), 0);
  EXPECT_EQ(rowForWaypoint(3), 1);
  EXPECT_EQ(rowForWaypoint(4), 1);
  EXPECT_EQ(rowForWaypoint(5), 2);
  EXPECT_EQ(rowForWaypoint(6), 2);
}

TEST(MissionLogic, WaypointFrameNameMatchesWorkstationTfManagerConvention)
{
  EXPECT_EQ(waypointFrameName(1), "boustrophedon_waypoint_1");
  EXPECT_EQ(waypointFrameName(12), "boustrophedon_waypoint_12");
}

// Full mission over 3 rows: walks the sequence runMission() would, from the first waypoint it
// actually navigates to (2) through the last one (row_count * 2), and checks that leg-type,
// row, and frame name for every waypoint index line up the way runMission() expects them to.
TEST(MissionLogic, FullMissionSequenceForThreeRows)
{
  const int row_count = 3;
  ASSERT_TRUE(isValidRowCount(row_count));
  const int waypoint_count = waypointCount(row_count);
  ASSERT_EQ(waypoint_count, 6);

  const struct {
    int waypoint_index;
    bool within_row_leg;
    int row;
  } expected[] = {
      {2, true, 0}, {3, false, 1}, {4, true, 1}, {5, false, 2}, {6, true, 2},
  };

  for (const auto &step : expected) {
    EXPECT_EQ(isWithinRowLeg(step.waypoint_index), step.within_row_leg)
        << "waypoint " << step.waypoint_index;
    EXPECT_EQ(rowForWaypoint(step.waypoint_index), step.row) << "waypoint " << step.waypoint_index;
    EXPECT_EQ(waypointFrameName(step.waypoint_index),
              "boustrophedon_waypoint_" + std::to_string(step.waypoint_index));
  }
}

TEST(MissionLogic, DescribeMissionStatusForWithinRowLeg)
{
  // waypoint 2 -> row 0 (0-based), reported as row 1/3, within-row leg.
  EXPECT_EQ(
      describeMissionStatus(/*row=*/0, /*row_count=*/3, /*waypoint_index=*/2, /*waypoint_count=*/6),
      "Entering row 1/3 (waypoint 2/6)");
}

TEST(MissionLogic, DescribeMissionStatusForTurnLeg)
{
  EXPECT_EQ(
      describeMissionStatus(/*row=*/1, /*row_count=*/3, /*waypoint_index=*/3, /*waypoint_count=*/6),
      "Turning into row 2/3 (waypoint 3/6)");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
