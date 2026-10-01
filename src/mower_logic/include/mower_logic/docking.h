/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef MOWER_LOGIC_DOCKING_H
#define MOWER_LOGIC_DOCKING_H

#include <geometry_msgs/PoseStamped.h>
#include <mower_map/docking_geometry.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace docking {

inline Pose2D PlanarPose(const geometry_msgs::Pose& pose) {
  return {pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)};
}

inline geometry_msgs::Pose WithPlanarPose(geometry_msgs::Pose pose, const Pose2D& planar) {
  pose.position.x = planar.x;
  pose.position.y = planar.y;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, planar.yaw);
  pose.orientation = tf2::toMsg(orientation);
  return pose;
}

inline geometry_msgs::Pose DockedPose(const geometry_msgs::Pose& pose, bool reverse) {
  // Preserve existing forward-mode pose resets exactly.
  return reverse ? WithPlanarPose(pose, DockedPose(PlanarPose(pose), true)) : pose;
}

inline nav_msgs::Path RosPath(const geometry_msgs::PoseStamped& reference, const std::vector<Pose2D>& poses) {
  nav_msgs::Path path;
  path.header = reference.header;
  for (const auto& pose : poses) {
    auto stamped = reference;
    stamped.pose = WithPlanarPose(reference.pose, pose);
    path.poses.push_back(stamped);
  }
  return path;
}

}  // namespace docking

#endif
