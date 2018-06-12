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

#ifndef RLL_MOVE_IFACE_H
#define RLL_MOVE_IFACE_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <rll_msgs/JobEnv.h>
#include <rll_msgs/PickPlace.h>
#include <rll_msgs/MoveLin.h>
#include <rll_msgs/MoveJoints.h>
#include <schunk_gripper_egl90/MoveGrip.h>
#include <schunk_gripper_egl90/MovePos.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_wrapper/moveit_wrapper.h>

class MoveIface
{
public:
	explicit MoveIface();
	bool run_job(rll_msgs::JobEnv::Request &req,
		     rll_msgs::JobEnv::Response &resp);
	bool idle(rll_msgs::JobEnv::Request &req,
		  rll_msgs::JobEnv::Response &resp);
	bool pick_place(rll_msgs::PickPlace::Request &req,
			rll_msgs::PickPlace::Response &resp);
	bool move_lin(rll_msgs::MoveLin::Request &req,
		      rll_msgs::MoveLin::Response &resp);
	bool move_joints(rll_msgs::MoveJoints::Request &req,
			 rll_msgs::MoveJoints::Response &resp);
	void resetToHome(bool info = true);
	void close_gripper();
	void open_gripper();

	~MoveIface();
  
private:
	const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group;

	MoveItWrapper moveit_wrapper;

	bool runTrajectory(bool info = true);
	int gripper_reference_motion();
	int gripper_move_grip(float speed, float current);
	int gripper_stop();
	int gripper_move_pos(float pos);
	int gripper_acknowledge();
};

#endif  // RLL_MOVE_IFACE_H

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
