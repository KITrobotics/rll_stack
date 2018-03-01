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
#include <iiwa_ros/iiwa_ros.h>

#include <rll_worker/JobEnv.h>
#include <moveit/move_group_interface/move_group_interface.h>

class TrajectorySampler
{
public:
	explicit TrajectorySampler(ros::NodeHandle nh);
	bool run_job(rll_worker::JobEnv::Request &req, rll_worker::JobEnv::Response &resp);
	bool idle(rll_worker::JobEnv::Request &req, rll_worker::JobEnv::Response &resp);
	void resetToHome(bool info = true);

	~TrajectorySampler();
  
private:
	const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group;
	geometry_msgs::Pose target_1, target_2;

	iiwa_ros::iiwaRos my_iiwa;

	bool getTargets();
	void runTrajectory(bool info = true);
};

#endif  // RLL_TRAJECTORY_SAMPLER_H

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
