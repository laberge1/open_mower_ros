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
#include "DockingBehavior.h"

#include <mower_msgs/Power.h>

#include "PerimeterDocking.h"
#include "mower_logic/docking.h"
#include "mower_logic/docking_config.h"
#include "mower_logic/utils.h"

extern ros::ServiceClient dockingPointClient;
extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;
extern mower_msgs::Status getStatus();
extern mower_msgs::Power getPower();

extern void stopMoving();
extern bool setGPS(bool enabled);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions);

DockingBehavior DockingBehavior::INSTANCE;

DockingBehavior::DockingBehavior() {
  xbot_msgs::ActionInfo abort_docking_action;
  abort_docking_action.action_id = "abort_docking";
  abort_docking_action.enabled = true;
  abort_docking_action.action_name = "Stop Docking";

  actions.clear();
  actions.push_back(abort_docking_action);
}

bool DockingBehavior::approach_docking_point() {
  ROS_INFO_STREAM("Calculating approach path");

  const auto recorded_pose = docking::PlanarPose(docking_pose_stamped.pose);

  // Navigate forward to the approach start and finish facing in the docking direction.
  {
    geometry_msgs::PoseStamped docking_approach_point = docking_pose_stamped;
    docking_approach_point.pose = docking::WithPlanarPose(
        docking_pose_stamped.pose,
        docking::PoseOnDockingLine(recorded_pose, config.docking_reverse, -config.docking_approach_distance));
    mbf_msgs::MoveBaseGoal moveBaseGoal;
    moveBaseGoal.target_pose = docking_approach_point;
    moveBaseGoal.controller = "FTCPlanner";

    auto result = sendGoalAndWaitUnlessAborted(mbfClient, moveBaseGoal);
    if (aborted || result.state_ != result.SUCCEEDED) {
      return false;
    }
  }

  {
    mbf_msgs::ExePathGoal exePathGoal;

    nav_msgs::Path path;

    ros::Time start_wait_time = ros::Time::now();
    ros::Rate loop_rate(100);
    while (ros::ok() && !aborted && (ros::Time::now() - start_wait_time) < ros::Duration(config.docking_waiting_time)) {
      loop_rate.sleep();
    }

    path = docking::RosPath(docking_pose_stamped, docking::StraightDockingPath(recorded_pose, config.docking_reverse,
                                                                               -config.docking_approach_distance, 0.0));
    if (path.poses.empty()) return !aborted && ros::ok();
    if (aborted || !ros::ok()) return false;

    exePathGoal.path = path;
    exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
    exePathGoal.dist_tolerance = 0.1;
    exePathGoal.tolerance_from_action = true;
    exePathGoal.controller = config.docking_reverse ? "DockingFTCPlanner" : "FTCPlanner";
    ROS_INFO_STREAM("Executing Docking Approach");

    auto approachResult = sendGoalAndWaitUnlessAborted(mbfClientExePath, exePathGoal);
    if (aborted || approachResult.state_ != approachResult.SUCCEEDED) {
      return false;
    }
  }

  return true;
}

bool DockingBehavior::dock_straight() {
  mbf_msgs::ExePathGoal exePathGoal;
  exePathGoal.path = docking::RosPath(
      docking_pose_stamped, docking::StraightDockingPath(docking::PlanarPose(docking_pose_stamped.pose),
                                                         config.docking_reverse, 0.0, config.docking_distance));
  if (aborted || !ros::ok()) return false;
  if (exePathGoal.path.poses.empty()) {
    stopMoving();
    const auto power = getPower();
    const bool docked = utils::GetFirstValid({power.charge_voltage_adc, power.charge_voltage}) > 5.0;
    if (docked) publishMowerEvent("DOCKED");
    return docked;
  }

  exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
  exePathGoal.dist_tolerance = 0.1;
  exePathGoal.tolerance_from_action = true;
  exePathGoal.controller = "DockingFTCPlanner";

  mbfClientExePath->sendGoal(exePathGoal);

  bool dockingSuccess = false;
  bool waitingForResult = true;

  ros::Rate r(10);

  // we can assume the last_state is current since we have a security timer
  while (ros::ok() && waitingForResult) {
    r.sleep();

    const auto last_status = getStatus();
    const auto last_power = getPower();
    auto mbfState = mbfClientExePath->getState();

    if (aborted) {
      ROS_INFO_STREAM("Docking aborted.");
      mbfClientExePath->cancelGoal();
      stopMoving();
      dockingSuccess = false;
      break;
    }

    // Use first valid sensor
    const float last_charge_v = utils::GetFirstValid({last_power.charge_voltage_adc, last_power.charge_voltage});

    switch (mbfState.state_) {
      case actionlib::SimpleClientGoalState::ACTIVE:
      case actionlib::SimpleClientGoalState::PENDING:
        // currently moving. Cancel as soon as we're in the station
        if (last_charge_v > 5.0) {
          ROS_INFO_STREAM("Got a voltage of " << last_charge_v << " V. Cancelling docking.");
          const auto contact_deadline = ros::Time::now() + ros::Duration(config.docking_extra_time);
          while (ros::ok() && !aborted && ros::Time::now() < contact_deadline) r.sleep();
          mbfClientExePath->cancelGoal();
          stopMoving();
          dockingSuccess = !aborted && ros::ok();
          if (dockingSuccess) publishMowerEvent("DOCKED");
          waitingForResult = false;
        }
        break;
      case actionlib::SimpleClientGoalState::SUCCEEDED:
        // we stopped moving because the path has ended. check, if we have docked successfully
        ROS_INFO_STREAM("Docking stopped, because we reached end pose. Voltage was " << last_charge_v << " V.");
        if (last_charge_v > 5.0) {
          mbfClientExePath->cancelGoal();
          dockingSuccess = true;
          stopMoving();
          publishMowerEvent("DOCKED");
        }
        waitingForResult = false;
        break;
      default:
        ROS_WARN_STREAM("Some error during path execution. Docking failed. status value was: " << mbfState.state_);
        waitingForResult = false;
        stopMoving();
        break;
    }
  }

  // to be safe if the planner sent additional commands after cancel
  stopMoving();

  return dockingSuccess;
}

std::string DockingBehavior::state_name() {
  return "DOCKING";
}

Behavior* DockingBehavior::execute() {
  if (!docking::ConfigurationSupported(config.docking_reverse, config.perimeter_signal)) {
    ROS_ERROR("Reverse docking is not supported with perimeter docking. Set perimeter_signal to 0.");
    stopMoving();
    publishMowerEvent("DOCKING_FAILED", json{{"reason", "unsupported_docking_configuration"}});
    return &IdleBehavior::INSTANCE;
  }
  if (aborted) {
    stopMoving();
    return &IdleBehavior::INSTANCE;
  }
  // Check if already docked (e.g. carried to base during emergency) and skip
  if (getPower().charge_voltage > 5.0 || getPower().charge_voltage_adc > 5.0) {
    ROS_INFO_STREAM("Already inside docking station, going directly to idle.");
    stopMoving();
    publishMowerEvent("DOCKED");
    return &IdleBehavior::DOCKED_INSTANCE;
  }

  while (!isGPSGood) {
    if (aborted) {
      ROS_INFO_STREAM("Docking aborted.");
      stopMoving();
      return &IdleBehavior::INSTANCE;
    }

    ROS_WARN_STREAM("Waiting for good GPS");
    ros::Duration(1.0).sleep();
  }

  bool approachSuccess = approach_docking_point();

  if (aborted) {
    ROS_INFO_STREAM("Docking aborted.");
    stopMoving();
    return &IdleBehavior::INSTANCE;
  }

  if (!approachSuccess) {
    ROS_ERROR("Error during docking approach.");

    retryCount++;
    if (retryCount <= config.docking_retry_count) {
      ROS_ERROR("Retrying docking approach");
      publishMowerEvent("DOCKING_RETRY", json{{"reason", "approach_failed"}, {"attempts", retryCount}});
      return &DockingBehavior::INSTANCE;
    }

    ROS_ERROR("Giving up on docking");
    publishMowerEvent("DOCKING_FAILED", json{{"reason", "approach_failed"}, {"attempts", retryCount}});
    return &IdleBehavior::INSTANCE;
  }

  // Reset retryCount here would cause an infinite loop
  // reset();

  // Disable GPS
  inApproachMode = false;
  setGPS(false);

  if (PerimeterSearchBehavior::configured(config)) return &PerimeterSearchBehavior::INSTANCE;

  bool docked = dock_straight();

  if (aborted) {
    ROS_INFO_STREAM("Docking aborted.");
    stopMoving();
    return &IdleBehavior::INSTANCE;
  }

  if (!docked) {
    ROS_ERROR("Error during docking.");

    retryCount++;
    if (retryCount <= config.docking_retry_count && !aborted) {
      ROS_ERROR_STREAM("Retrying docking. Try " << retryCount << " / " << config.docking_retry_count);
      publishMowerEvent("DOCKING_RETRY", json{{"reason", "dock_failed"}, {"attempts", retryCount}});
      return &UndockingBehavior::RETRY_INSTANCE;
    }

    ROS_ERROR("Giving up on docking");
    publishMowerEvent("DOCKING_FAILED", json{{"reason", "dock_failed"}, {"attempts", retryCount}});
    // Reset retryCount
    reset();
    return &IdleBehavior::INSTANCE;
  }

  // Reset retryCount
  reset();

  return &IdleBehavior::DOCKED_INSTANCE;
}

void DockingBehavior::enter() {
  paused = aborted = false;
  // start with target approach and then dock later
  inApproachMode = true;

  // Get the docking pose in map
  mower_map::GetDockingPointSrv get_docking_point_srv;
  if (!dockingPointClient.call(get_docking_point_srv)) {
    ROS_ERROR("Cannot get the docking pose. Docking cancelled.");
    aborted = true;
    return;
  }
  docking_pose_stamped.pose = get_docking_point_srv.response.docking_pose;
  docking_pose_stamped.header.frame_id = "map";
  docking_pose_stamped.header.stamp = ros::Time::now();

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:docking", actions);
}

void DockingBehavior::exit() {
  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:docking", actions);
}

void DockingBehavior::reset() {
  retryCount = 0;
}

bool DockingBehavior::needs_gps() {
  // we only need GPS if we're in approach mode
  return inApproachMode;
}

bool DockingBehavior::mower_enabled() {
  // No mower during docking
  return false;
}

void DockingBehavior::command_home() {
}

void DockingBehavior::command_start() {
  this->abort();
}

void DockingBehavior::command_s1() {
}

void DockingBehavior::command_s2() {
}

bool DockingBehavior::redirect_joystick() {
  return false;
}

uint8_t DockingBehavior::get_sub_state() {
  return 1;
}

uint8_t DockingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void DockingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:docking/abort_docking") {
    ROS_INFO_STREAM("Got abort docking command");
    command_start();
  }
}
