// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#include "UndockingBehavior.h"

#include <mower_msgs/Power.h>

#include "mower_logic/docking.h"
#include "mower_logic/docking_config.h"

extern ros::ServiceClient dockingPointClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;
extern xbot_msgs::AbsolutePose getPose();
extern mower_msgs::Status getStatus();
extern mower_msgs::Power getPower();

extern void setRobotPose(geometry_msgs::Pose& pose);
extern void stopMoving();
extern bool isGpsGood();
extern bool setGPS(bool enabled);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions);

UndockingBehavior UndockingBehavior::INSTANCE(&MowingBehavior::INSTANCE);
UndockingBehavior UndockingBehavior::RETRY_INSTANCE(&DockingBehavior::INSTANCE);

UndockingBehavior::UndockingBehavior() {
  xbot_msgs::ActionInfo abort_undocking_action;
  abort_undocking_action.action_id = "abort_undocking";
  abort_undocking_action.enabled = true;
  abort_undocking_action.action_name = "Stop Undocking";

  actions.clear();
  actions.push_back(abort_undocking_action);
}

std::string UndockingBehavior::state_name() {
  return "UNDOCKING";
}

Behavior* UndockingBehavior::execute() {
  if (!docking::ConfigurationSupported(config.docking_reverse, config.perimeter_signal)) {
    ROS_ERROR("Reverse docking is not supported with perimeter docking. Set perimeter_signal to 0.");
    stopMoving();
    publishMowerEvent("UNDOCKING_FAILED", json{{"reason", "unsupported_docking_configuration"}});
    return &IdleBehavior::INSTANCE;
  }
  if (aborted) {
    stopMoving();
    return &IdleBehavior::INSTANCE;
  }
  static bool rng_seeding_required = true;

  // get robot's current pose from odometry.
  xbot_msgs::AbsolutePose pose = getPose();
  mbf_msgs::ExePathGoal exePathGoal;

  ros::Time start_wait_time = ros::Time::now();
  ros::Rate loop_rate(100);
  while (ros::ok() && !aborted && (ros::Time::now() - start_wait_time) < ros::Duration(config.undocking_waiting_time)) {
    loop_rate.sleep();
  }

  if (aborted || !ros::ok()) {
    stopMoving();
    return &IdleBehavior::INSTANCE;
  }

  double angle;
  if (config.undock_fixed_angle) {
    angle = config.undock_angle * M_PI / 180.0;
    ROS_INFO_STREAM("Fixed angle undock: " << config.undock_angle);
  } else {
    // seed based on first undock time rather than boot so should be ok even without RTC
    if (rng_seeding_required) {
      srand(ros::Time::now().toSec());
      ROS_INFO_STREAM("Random angle undock: Seeded rand()");
      rng_seeding_required = false;
    }
    double random_number = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
    double random_angle_deg = abs(config.undock_angle) * random_number;
    ROS_INFO_STREAM("Random angle undock: " << random_angle_deg);
    angle = random_angle_deg * M_PI / 180.0;
  }

  geometry_msgs::PoseStamped start_pose;
  start_pose.pose = pose.pose.pose;
  start_pose.header = pose.header;
  exePathGoal.path =
      docking::RosPath(start_pose, docking::UndockingPath(docking::PlanarPose(start_pose.pose), config.docking_reverse,
                                                          config.undock_distance, config.undock_angled_distance, angle,
                                                          config.undock_use_curve));
  if (exePathGoal.path.poses.empty()) {
    ROS_ERROR("No undocking distance configured. Staying idle.");
    stopMoving();
    publishMowerEvent("UNDOCKING_FAILED", json{{"reason", "empty_undocking_path"}});
    return &IdleBehavior::INSTANCE;
  }

  exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
  exePathGoal.dist_tolerance = 0.1;
  exePathGoal.tolerance_from_action = true;
  exePathGoal.controller = "DockingFTCPlanner";

  auto result = sendGoalAndWaitUnlessAborted(mbfClientExePath, exePathGoal);

  if (aborted) {
    ROS_INFO_STREAM("Undocking aborted.");
    stopMoving();
    return &IdleBehavior::INSTANCE;
  }

  bool success = result.state_ == actionlib::SimpleClientGoalState::SUCCEEDED;

  // stop the bot for now
  stopMoving();

  if (!success) {
    ROS_ERROR_STREAM("Error during undock");
    publishMowerEvent("UNDOCKING_FAILED", json{{"reason", "path_failed"}});
    return &IdleBehavior::INSTANCE;
  }

  ROS_INFO_STREAM("Undock success. Waiting for GPS.");
  bool hasGps = waitForGPS();

  if (!hasGps) {
    if (aborted) {
      ROS_INFO_STREAM("Undocking aborted while waiting for GPS.");
    } else if (!ros::ok()) {
      ROS_INFO_STREAM("ROS shutdown while waiting for GPS.");
    } else {
      ROS_ERROR_STREAM("Could not get GPS.");
      publishMowerEvent("UNDOCKING_FAILED", json{{"reason", "no_gps"}});
    }
    return &IdleBehavior::INSTANCE;
  }

  // TODO return mow area
  publishMowerEvent("UNDOCKED");
  return nextBehavior;
}

void UndockingBehavior::enter() {
  reset();
  paused = aborted = false;

  // Get the docking pose in map
  mower_map::GetDockingPointSrv get_docking_point_srv;
  if (!dockingPointClient.call(get_docking_point_srv)) {
    ROS_ERROR("Cannot get the docking pose. Undocking cancelled.");
    aborted = true;
    return;
  }
  docking_pose_stamped.pose = docking::DockedPose(get_docking_point_srv.response.docking_pose, config.docking_reverse);
  docking_pose_stamped.header.frame_id = "map";
  docking_pose_stamped.header.stamp = ros::Time::now();

  // set the robot's position to the dock if we're actually docked
  if (docking::ConfigurationSupported(config.docking_reverse, config.perimeter_signal) &&
      (getPower().charge_voltage_adc > 5.0 || getPower().charge_voltage > 5.0)) {
    ROS_INFO_STREAM("Currently inside the docking station, we set the robot's pose to the docks pose.");
    setRobotPose(docking_pose_stamped.pose);
  }

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:undocking", actions);
}

void UndockingBehavior::exit() {
  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:undocking", actions);
}

void UndockingBehavior::reset() {
  gpsRequired = false;
}

bool UndockingBehavior::needs_gps() {
  return gpsRequired;
}

bool UndockingBehavior::mower_enabled() {
  // No mower during docking
  return false;
}

bool UndockingBehavior::waitForGPS() {
  gpsRequired = false;
  setGPS(true);
  ros::Rate odom_rate(1.0);
  while (ros::ok() && !aborted) {
    if (isGpsGood()) {
      ROS_INFO("Got good gps, let's go");
      break;
    } else {
      ROS_INFO_STREAM("waiting for gps. current accuracy: " << getPose().position_accuracy);
      odom_rate.sleep();
    }
  }
  if (!ros::ok() || aborted) {
    return false;
  }

  // wait additional time for odometry filters to converge
  ros::Rate r(ros::Duration(config.gps_wait_time, 0));
  r.sleep();

  gpsRequired = true;

  return true;
}

UndockingBehavior::UndockingBehavior(Behavior* next) {
  this->nextBehavior = next;
}

void UndockingBehavior::command_home() {
  this->abort();
}

void UndockingBehavior::command_start() {
}

void UndockingBehavior::command_s1() {
}

void UndockingBehavior::command_s2() {
}

bool UndockingBehavior::redirect_joystick() {
  return false;
}

uint8_t UndockingBehavior::get_sub_state() {
  return 2;
}

uint8_t UndockingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void UndockingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:undocking/abort_undocking") {
    ROS_INFO_STREAM("Got abort undocking command");
    command_home();
  }
}
