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

#include <trajectory_sampler.h>

TrajectorySampler::TrajectorySampler(ros::NodeHandle nh)
	: move_group(PLANNING_GROUP)
{
	bool target_set;
	geometry_msgs::Pose target_1, target_2;

	my_iiwa.init();

	// configure planner
	move_group.setPlannerId("RRTConnectkConfigDefault");
	move_group.setPlanningTime(10.0);

	// slow down movement of the robot
	move_group.setMaxVelocityScalingFactor(0.1);

	while (ros::ok()) {
		if (my_iiwa.getRobotIsConnected()) {

			ROS_INFO("Moving to home");
			move_group.setStartStateToCurrentState();
			move_group.setNamedTarget("home_bow");
			runTrajectory();

			ros::param::get("target_pos_set", target_set);
			if (target_set && getTargets(&target_1, &target_2)) {
				ROS_INFO("Moving to target 1");
				move_group.setStartStateToCurrentState();
				move_group.setPoseTarget(target_1);
				runTrajectory();

				ROS_INFO("Moving to target 2");
				move_group.setStartStateToCurrentState();
				move_group.setPoseTarget(target_2);
				runTrajectory();

				// reset after one run
				ros::param::set("target_pos_set", false);
			} else {
				ROS_WARN("No target set or unable to get target");
				ros::Duration(5.0).sleep(); // 5 seconds
			}
		} else {
			ROS_WARN_STREAM("Robot is not connected...");
			ros::Duration(5.0).sleep(); // 5 seconds
		}
	}
}

bool TrajectorySampler::getTargets(geometry_msgs::Pose *target_1, geometry_msgs::Pose *target_2)
{
	bool got_targets = false;

	ros::param::get("/target_1_spawn_location/x", target_1->position.x);
	ros::param::get("/target_1_spawn_location/y", target_1->position.y);
	ros::param::get("/target_1_spawn_location/z", target_1->position.z);
	ros::param::get("/target_2_spawn_location/x", target_2->position.x);
	ros::param::get("/target_2_spawn_location/y", target_2->position.y);
	ros::param::get("/target_2_spawn_location/z", target_2->position.z);

	target_1->orientation.x = 0.0;
	target_1->orientation.y = 1.0;
	target_1->orientation.z = 0.0;
	target_1->orientation.w = 0.0;
	target_2->orientation.x = 0.0;
	target_2->orientation.y = 1.0;
	target_2->orientation.z = 0.0;
	target_2->orientation.w = 0.0;

	got_targets = true;

	return got_targets;
}

void TrajectorySampler::runTrajectory()
{
	success_plan = move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	ROS_INFO("Planning result: %s",
		 success_plan ? "SUCCEEDED" : "FAILED");

	if (success_plan) {
		success_plan = move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
		ROS_INFO("Plan execution result: %s",
			 success_plan ? "SUCCEEDED" : "FAILED");

		// stay a little a the target
		ros::Duration(3.0).sleep();
	} else {
		ROS_WARN("Not executing because planning failed");
	}
}

TrajectorySampler::~TrajectorySampler() {}

int main (int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "trajectory_sampler");
	ros::NodeHandle nh;

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	TrajectorySampler plan_sampler(nh);

	return 0;
}

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
