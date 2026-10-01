/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef MOWER_MAP_DOCKING_GEOMETRY_H
#define MOWER_MAP_DOCKING_GEOMETRY_H

#include <algorithm>
#include <cmath>
#include <vector>

namespace docking {

struct Pose2D {
  double x;
  double y;
  double yaw;
};

// The map heading describes travel into the station, independent of the robot's facing direction.
inline double RobotDockHeading(double recorded_heading, bool reverse) {
  const double heading = recorded_heading + (reverse ? std::acos(-1.0) : 0.0);
  return std::atan2(std::sin(heading), std::cos(heading));
}

inline Pose2D DockedPose(const Pose2D& recorded_pose, bool reverse) {
  return {recorded_pose.x, recorded_pose.y, RobotDockHeading(recorded_pose.yaw, reverse)};
}

inline Pose2D PoseOnDockingLine(const Pose2D& recorded_pose, bool reverse, double offset) {
  auto pose = DockedPose(recorded_pose, reverse);
  pose.x += std::cos(recorded_pose.yaw) * offset;
  pose.y += std::sin(recorded_pose.yaw) * offset;
  return pose;
}

// Offsets are measured along the recorded heading, not the robot heading. FTC needs at least three poses.
inline std::vector<Pose2D> StraightDockingPath(const Pose2D& recorded_pose, bool reverse, double start_offset,
                                               double end_offset) {
  const double distance = std::abs(end_offset - start_offset);
  if (distance == 0.0) return {};
  const int segments = std::max(2, static_cast<int>(std::ceil(distance * 10.0)));
  std::vector<Pose2D> path;
  path.reserve(segments + 1);
  for (int i = 0; i <= segments; ++i) {
    const double offset =
        i == segments ? end_offset : start_offset + (end_offset - start_offset) * (static_cast<double>(i) / segments);
    path.push_back(PoseOnDockingLine(recorded_pose, reverse, offset));
  }
  return path;
}

// Start from the actual body pose, including during a failed docking attempt.
inline std::vector<Pose2D> UndockingPath(const Pose2D& start, bool reverse, double straight_distance,
                                         double angled_distance, double angle, bool use_curve) {
  if (straight_distance == 0.0 && angled_distance == 0.0) return {};
  const double direction = reverse ? 1.0 : -1.0;
  std::vector<Pose2D> path{start};
  auto pose = start;
  if (straight_distance > 0.0) {
    const int segments = 3;
    for (int i = 1; i <= segments; ++i) {
      pose.x = start.x + direction * std::cos(start.yaw) * straight_distance * i / segments;
      pose.y = start.y + direction * std::sin(start.yaw) * straight_distance * i / segments;
      path.push_back(pose);
    }
  }
  if (angled_distance > 0.0) {
    const int segments = 10;
    for (int i = 1; i <= segments; ++i) {
      // Keep the angle sign: a pi heading rotation and a travel reversal preserve the same world-space exit.
      pose.yaw = start.yaw + angle * (use_curve ? static_cast<double>(i) / segments : 1.0);
      pose.x += direction * std::cos(pose.yaw) * angled_distance / segments;
      pose.y += direction * std::sin(pose.yaw) * angled_distance / segments;
      path.push_back(pose);
    }
  }
  return path;
}

}  // namespace docking

#endif
