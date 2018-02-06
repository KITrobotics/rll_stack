/*
 * This file is part of the Robot Learning Lab stack
 *
 * Copyright (C) 2018 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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

	bool getTargets(geometry_msgs::Pose *target_1, geometry_msgs::Pose *target_2);
	void runTrajectory();

};

#endif  // RLL_TRAJECTORY_SAMPLER_H

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
