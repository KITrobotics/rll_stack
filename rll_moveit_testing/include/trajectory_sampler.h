/*
 * This file is part of the Robot Learning Lab stack at KIT
 *
 * TODO: clarify license and authorship with Udacity and their Pick and Place project
 */

#ifndef RLL_TRAJECTORY_SAMPLER_H
#define RLL_TRAJECTORY_SAMPLER_H

#include <ros/ros.h>

#include <iiwa_ros.h>

#include <moveit/move_group_interface/move_group_interface.h>

class TrajectorySampler
{
 public:
  explicit TrajectorySampler(ros::NodeHandle nh);
  ~TrajectorySampler();
  
 private:
  const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success_plan;

  iiwa_ros::iiwaRos my_iiwa;

  void moveToPose(const geometry_msgs::Pose target_pose);

};

#endif  // RLL_TRAJECTORY_SAMPLER_H
