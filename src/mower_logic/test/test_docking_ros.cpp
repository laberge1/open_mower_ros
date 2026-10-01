/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <gtest/gtest.h>

#include "mower_logic/docking.h"

TEST(DockingRos, PathPreservesFrameTimestampAndHeight) {
  geometry_msgs::PoseStamped dock;
  dock.header.frame_id = "map";
  dock.header.stamp = ros::Time(123, 456);
  dock.pose.position.z = 0.7;
  dock.pose.orientation.w = 1.0;
  const auto path = docking::RosPath(dock, docking::StraightDockingPath({2.0, 3.0, 0.0}, true, -1.0, 0.0));
  ASSERT_GE(path.poses.size(), 3u);
  EXPECT_EQ(path.header.frame_id, dock.header.frame_id);
  EXPECT_EQ(path.header.stamp, dock.header.stamp);
  for (const auto& pose : path.poses) {
    EXPECT_EQ(pose.header.frame_id, dock.header.frame_id);
    EXPECT_EQ(pose.header.stamp, dock.header.stamp);
    EXPECT_DOUBLE_EQ(pose.pose.position.z, 0.7);
    const auto planar = docking::PlanarPose(pose.pose);
    EXPECT_NEAR(std::cos(planar.yaw), -1.0, 1e-10);
    EXPECT_NEAR(std::sin(planar.yaw), 0.0, 1e-10);
  }
}

TEST(DockingRos, DockedPoseFlipsBodyHeadingWithoutChangingPosition) {
  geometry_msgs::Pose pose;
  pose.position.x = 1.2;
  pose.position.y = -3.4;
  pose.position.z = 0.7;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 3.13);
  pose.orientation = tf2::toMsg(q);
  const auto forward = docking::DockedPose(pose, false);
  EXPECT_EQ(forward, pose);
  const auto reverse = docking::DockedPose(pose, true);
  EXPECT_EQ(reverse.position, pose.position);
  EXPECT_NEAR(std::cos(tf2::getYaw(reverse.orientation)), -std::cos(3.13), 1e-10);
  EXPECT_NEAR(std::sin(tf2::getYaw(reverse.orientation)), -std::sin(3.13), 1e-10);
}

TEST(DockingRos, EmptyGeometryProducesNoPoses) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  const auto path = docking::RosPath(pose, {});
  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(path.header.frame_id, "map");
}
