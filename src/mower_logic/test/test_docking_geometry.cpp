/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <gtest/gtest.h>
#include <mower_map/docking_geometry.h>

#include "mower_logic/docking_config.h"

namespace {
const double kPi = std::acos(-1.0);
const double kTolerance = 1e-10;

void ExpectHeading(double actual, double expected) {
  EXPECT_NEAR(std::sin(actual), std::sin(expected), kTolerance);
  EXPECT_NEAR(std::cos(actual), std::cos(expected), kTolerance);
}

double ForwardDisplacement(const docking::Pose2D& from, const docking::Pose2D& to) {
  return (to.x - from.x) * std::cos(from.yaw) + (to.y - from.y) * std::sin(from.yaw);
}

TEST(DockPose, ReversePreservesPositionAndFlipsHeading) {
  for (double yaw : {0.0, kPi / 2, -kPi / 2, kPi - 0.01, -kPi + 0.01}) {
    const docking::Pose2D recorded{12.0, -3.0, yaw};
    const auto forward = docking::DockedPose(recorded, false);
    const auto reverse = docking::DockedPose(recorded, true);
    EXPECT_DOUBLE_EQ(reverse.x, recorded.x);
    EXPECT_DOUBLE_EQ(reverse.y, recorded.y);
    ExpectHeading(forward.yaw, yaw);
    ExpectHeading(reverse.yaw, yaw + kPi);
    EXPECT_LE(std::abs(reverse.yaw), kPi);
  }
}

TEST(DockingApproach, SameWorldLineWithOppositeBodyHeading) {
  const docking::Pose2D dock{4.0, 2.0, 0.0};
  const auto forward = docking::StraightDockingPath(dock, false, -1.5, 0.0);
  const auto reverse = docking::StraightDockingPath(dock, true, -1.5, 0.0);
  ASSERT_EQ(forward.size(), reverse.size());
  EXPECT_DOUBLE_EQ(reverse.front().x, 2.5);
  EXPECT_DOUBLE_EQ(reverse.back().x, 4.0);
  for (size_t i = 0; i < reverse.size(); ++i) {
    EXPECT_DOUBLE_EQ(reverse[i].x, forward[i].x);
    EXPECT_DOUBLE_EQ(reverse[i].y, 2.0);
    ExpectHeading(reverse[i].yaw, kPi);
    if (i > 0) {
      EXPECT_LT(ForwardDisplacement(reverse[i - 1], reverse[i]), 0.0);
      EXPECT_GT(ForwardDisplacement(forward[i - 1], forward[i]), 0.0);
    }
  }
}

TEST(DockingApproach, DiagonalHasExactEndpointsAndNoOvershoot) {
  const docking::Pose2D dock{3.0, -4.0, kPi / 4};
  const double distance = 1.237;
  const auto path = docking::StraightDockingPath(dock, true, -distance, 0.0);
  ASSERT_GE(path.size(), 3u);
  EXPECT_NEAR(path.front().x, dock.x - distance / std::sqrt(2.0), kTolerance);
  EXPECT_NEAR(path.front().y, dock.y - distance / std::sqrt(2.0), kTolerance);
  EXPECT_DOUBLE_EQ(path.back().x, dock.x);
  EXPECT_DOUBLE_EQ(path.back().y, dock.y);
  for (const auto& pose : path) {
    EXPECT_LE(pose.x, dock.x);
    EXPECT_LE(pose.y, dock.y);
    ExpectHeading(pose.yaw, -3.0 * kPi / 4);
  }
}

TEST(DockingInsertion, BacksTowardStationAlongRecordedHeading) {
  const docking::Pose2D dock{-2.0, 3.0, kPi / 2};
  const auto path = docking::StraightDockingPath(dock, true, 0.0, 0.537);
  ASSERT_GE(path.size(), 3u);
  EXPECT_NEAR(path.front().y, 3.0, kTolerance);
  EXPECT_NEAR(path.back().x, -2.0, kTolerance);
  EXPECT_NEAR(path.back().y, 3.537, kTolerance);
  EXPECT_LT(ForwardDisplacement(path.front(), path.back()), 0.0);
}

TEST(DockingPath, ShortNonzeroSegmentsHaveAtLeastThreeDistinctPoses) {
  for (bool reverse : {false, true}) {
    for (double distance : {0.001, 0.05, 0.1, 0.2, 1.237}) {
      const auto path = docking::StraightDockingPath({0.0, 0.0, 0.0}, reverse, 0.0, distance);
      ASSERT_GE(path.size(), 3u);
      EXPECT_DOUBLE_EQ(path.front().x, 0.0);
      EXPECT_DOUBLE_EQ(path.back().x, distance);
      for (size_t i = 1; i < path.size(); ++i) {
        EXPECT_GT(path[i].x, path[i - 1].x);
        EXPECT_LE(path[i].x - path[i - 1].x, 0.1 + kTolerance);
      }
    }
  }
}

TEST(DockingPath, ZeroLengthDoesNotSubmitAPath) {
  EXPECT_TRUE(docking::StraightDockingPath({0.0, 0.0, 0.0}, false, 0.0, 0.0).empty());
  EXPECT_TRUE(docking::StraightDockingPath({0.0, 0.0, 0.0}, true, -1.0, -1.0).empty());
}

TEST(Undocking, ForwardDockExitsBackwardAndReverseDockExitsForward) {
  const auto forward = docking::UndockingPath({2.0, 3.0, 0.0}, false, 1.2, 0.0, 0.0, true);
  const auto reverse = docking::UndockingPath({2.0, 3.0, kPi}, true, 1.2, 0.0, 0.0, true);
  ASSERT_EQ(forward.size(), 4u);
  ASSERT_EQ(reverse.size(), forward.size());
  EXPECT_NEAR(reverse.back().x, 0.8, kTolerance);
  EXPECT_NEAR(reverse.back().y, 3.0, kTolerance);
  EXPECT_GT(ForwardDisplacement(reverse.front(), reverse.back()), 0.0);
  EXPECT_LT(ForwardDisplacement(forward.front(), forward.back()), 0.0);
}

TEST(Undocking, RetryStartsFromActualPoseWithoutRotatingIt) {
  const docking::Pose2D actual{1.2, -0.4, kPi - 0.15};
  const auto path = docking::UndockingPath(actual, true, 0.6, 0.0, 0.0, true);
  ASSERT_GE(path.size(), 3u);
  EXPECT_DOUBLE_EQ(path.front().x, actual.x);
  EXPECT_DOUBLE_EQ(path.front().y, actual.y);
  ExpectHeading(path.front().yaw, actual.yaw);
  EXPECT_GT(ForwardDisplacement(actual, path.back()), 0.0);
}

TEST(Undocking, AngleSignPreservesWorldExitPathInBothModes) {
  for (double yaw : {0.0, kPi / 2, kPi - 0.01}) {
    for (double angle : {-kPi / 4, kPi / 4}) {
      for (bool curve : {false, true}) {
        const auto forward = docking::UndockingPath({2.0, 3.0, yaw}, false, 0.7, 1.3, angle, curve);
        const auto reverse = docking::UndockingPath({2.0, 3.0, yaw + kPi}, true, 0.7, 1.3, angle, curve);
        ASSERT_EQ(forward.size(), reverse.size());
        for (size_t i = 0; i < reverse.size(); ++i) {
          EXPECT_NEAR(reverse[i].x, forward[i].x, kTolerance);
          EXPECT_NEAR(reverse[i].y, forward[i].y, kTolerance);
          ExpectHeading(reverse[i].yaw, forward[i].yaw + kPi);
        }
      }
    }
  }
}

TEST(Undocking, CurvedAndStraightSecondStagesHaveKnownEndpoints) {
  const auto straight = docking::UndockingPath({0.0, 0.0, kPi}, true, 1.0, 2.0, kPi / 2, false);
  ASSERT_EQ(straight.size(), 14u);
  EXPECT_NEAR(straight.back().x, -1.0, kTolerance);
  EXPECT_NEAR(straight.back().y, -2.0, kTolerance);
  const auto curved = docking::UndockingPath({0.0, 0.0, kPi}, true, 1.0, 2.0, kPi / 2, true);
  ASSERT_EQ(curved.size(), 14u);
  EXPECT_LT(curved.back().x, -1.0);
  EXPECT_LT(curved.back().y, 0.0);
  EXPECT_GT(curved.back().y, -2.0);
  ExpectHeading(curved.back().yaw, -kPi / 2);
}

TEST(Undocking, ZeroSegmentsAreSkipped) {
  EXPECT_TRUE(docking::UndockingPath({0.0, 0.0, 0.0}, true, 0.0, 0.0, 0.3, true).empty());
  const auto straight = docking::UndockingPath({0.0, 0.0, 0.0}, true, 0.03, 0.0, 0.3, true);
  ASSERT_EQ(straight.size(), 4u);
  ExpectHeading(straight.back().yaw, 0.0);
  const auto angled = docking::UndockingPath({0.0, 0.0, 0.0}, true, 0.0, 0.03, 0.3, true);
  EXPECT_EQ(angled.size(), 11u);
}

TEST(DockingConfiguration, OnlyReversePerimeterCombinationIsUnsupported) {
  for (int perimeter : {-2, -1, 0, 1, 2}) {
    EXPECT_TRUE(docking::ConfigurationSupported(false, perimeter));
    EXPECT_EQ(docking::ConfigurationSupported(true, perimeter), perimeter == 0);
  }
}

TEST(DockingConfiguration, DirectionIsLatchedOnFirstCallback) {
  for (bool initial : {false, true}) {
    docking::StartupDirection direction;
    bool requested = initial;
    EXPECT_TRUE(direction.Apply(requested));
    EXPECT_EQ(requested, initial);
    requested = !initial;
    EXPECT_FALSE(direction.Apply(requested));
    EXPECT_EQ(requested, initial);
    EXPECT_TRUE(direction.Apply(requested));
  }
}

}  // namespace
