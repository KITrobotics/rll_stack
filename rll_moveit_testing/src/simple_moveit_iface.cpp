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

#include <simple_moveit_iface.h>

TrajectorySampler::TrajectorySampler(ros::NodeHandle nh)
	: move_group(PLANNING_GROUP)
{
	// configure planner
	move_group.setPlannerId(PLANNING_GROUP+"[RRTConnectkConfigDefault]");
	move_group.setPlanningTime(10.0);
	// slow down movement of the robot
	move_group.setMaxVelocityScalingFactor(0.1);

	my_iiwa.init();
}

bool TrajectorySampler::run_job(rll_worker::JobEnv::Request &req,
				rll_worker::JobEnv::Response &resp)
{
	ROS_INFO("got job running request");
	bool target_set;

	if (my_iiwa.getRobotIsConnected()) {
		ros::param::get("target_pos_set", target_set);
		if (target_set && getTargets()) {
			// move three times between the targets
			for(int i=1; i<=3; ++i) {
				ROS_INFO("Moving to target 1: x %.2f, y %.2f, z %.2f", target_1.position.x, target_1.position.y,
					 target_1.position.z);
				move_group.setStartStateToCurrentState();
				move_group.setPoseTarget(target_1);
				runTrajectory();
				close_gripper();

				ROS_INFO("Moving to target 2: x %.2f, y %.2f, z %.2f", target_2.position.x, target_2.position.y,
					 target_2.position.z);
				move_group.setStartStateToCurrentState();
				move_group.setPoseTarget(target_2);
				runTrajectory();
				open_gripper();
			}

			resetToHome();
			open_gripper();
			// reset after one run
			ros::param::set("target_pos_set", false);
			resp.job.status = rll_worker::JobStatus::SUCCESS;
		} else {
			ROS_WARN("No target set or unable to get target");
			resp.job.status = rll_worker::JobStatus::FAILURE;
		}
	} else {
		ROS_WARN_STREAM("Robot is not connected...");
		resp.job.status = rll_worker::JobStatus::INTERNAL_ERROR;
	}

	return true;
}

bool TrajectorySampler::idle(rll_worker::JobEnv::Request &req,
			     rll_worker::JobEnv::Response &resp)
{
	// Send home position when idling
	// This ensures that the brakes are not activated and the control cycle keeps running.
	// If we the don't do this, the robot won't move when a trajectory is sent and the brakes are active.
	if (my_iiwa.getRobotIsConnected()) {
			resetToHome(false);
			resp.job.status = rll_worker::JobStatus::SUCCESS;
	} else {
		ROS_WARN_STREAM("Robot is not connected...");
		resp.job.status = rll_worker::JobStatus::INTERNAL_ERROR;
	}

	return true;
}

bool TrajectorySampler::pick_place(rll_moveit_testing::PickPlace::Request &req,
				   rll_moveit_testing::PickPlace::Response &resp)
{
	bool success;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	std::vector<geometry_msgs::Pose> waypoints_to;
	std::vector<geometry_msgs::Pose> waypoints_away;
	moveit_msgs::RobotTrajectory trajectory;
	const double eef_step = 0.05;
	const double jump_threshold = 1000.0;
	
	// if (my_iiwa.getRobotIsConnected()) {
		ROS_INFO("Moving above target");
		move_group.setStartStateToCurrentState();
		waypoints_to.push_back(req.pose_above);
		waypoints_to.push_back(req.pose_grip);
		move_group.setPoseTarget(req.pose_grip);
		move_group.computeCartesianPath(waypoints_to, eef_step, jump_threshold, trajectory);
		my_plan.trajectory_= trajectory;

		move_group.execute(my_plan);
		// if (!success) {
		// 	resp.success = false;
		// 	return true;
		// }

		if (req.gripper_close)
			close_gripper();
		else
			open_gripper();

		ROS_INFO("Moving above grip position");
		move_group.setStartStateToCurrentState();
		waypoints_away.push_back(req.pose_above);
		move_group.setPoseTarget(req.pose_above);
		move_group.computeCartesianPath(waypoints_away, eef_step, jump_threshold, trajectory);
		my_plan.trajectory_= trajectory;

		move_group.execute(my_plan);
		// if (!success) {
		// 	resp.success = false;
		// 	return true;
		// }

	// } else {
	// 	ROS_WARN_STREAM("Robot is not connected...");
	// 	resp.success = false;
	// }

	resp.success = true;
	return true;
}


bool TrajectorySampler::getTargets()
{
	bool got_targets = false;

	ros::param::get("target_1_spawn_location/x", target_1.position.x);
	ros::param::get("target_1_spawn_location/y", target_1.position.y);
	ros::param::get("target_1_spawn_location/z", target_1.position.z);
	ros::param::get("target_2_spawn_location/x", target_2.position.x);
	ros::param::get("target_2_spawn_location/y", target_2.position.y);
	ros::param::get("target_2_spawn_location/z", target_2.position.z);

	target_1.orientation.x = 0.0;
	target_1.orientation.y = 1.0;
	target_1.orientation.z = 0.0;
	target_1.orientation.w = 0.0;
	target_2.orientation.x = 0.0;
	target_2.orientation.y = 1.0;
	target_2.orientation.z = 0.0;
	target_2.orientation.w = 0.0;

	got_targets = true;

	return got_targets;
}

bool TrajectorySampler::runTrajectory(bool info)
{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success_plan;

	success_plan = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (info)
		ROS_INFO("Planning result: %s",
			 success_plan ? "SUCCEEDED" : "FAILED");

	if (success_plan) {
		success_plan = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if (info)
			ROS_INFO("Plan execution result: %s",
				 success_plan ? "SUCCEEDED" : "FAILED");

		// stay a little a the target
		ros::Duration(1.0).sleep();
	} else {
		ROS_WARN("Not executing because planning failed");
		return false;
	}

	return true;
}

void TrajectorySampler::close_gripper()
{
	move_grip(-80.0, 0.3);
	acknowledge();
}

void TrajectorySampler::open_gripper()
{
	move_grip(80.0, 0.3);
	acknowledge();
}

int TrajectorySampler::move_grip(float sp, float cu)
{
	ros::NodeHandle n_move_grip;
	ros::ServiceClient c_move_grip = n_move_grip.serviceClient<schunk_gripper_egl90::MoveGrip>("gripper_egl_90/move_grip");
	schunk_gripper_egl90::MoveGrip srv;
	srv.request.speed = sp;
	srv.request.current = cu;
	if (c_move_grip.call(srv)) {
	  ROS_INFO("Moving the gripper fingers.");
	} else {
	  ROS_ERROR("Failed to call service move_grip");
	  return 1;
	}
}

int TrajectorySampler::acknowledge()
{
	ros::NodeHandle n_ack;
	ros::ServiceClient c_ack = n_ack.serviceClient<std_srvs::Trigger>("gripper_egl_90/acknowledge");
	std_srvs::Trigger srv;
	if (c_ack.call(srv))
	{
	  ROS_INFO("gripper: Acknowledging");
	}
	else
	{
	  ROS_ERROR("Failed to call service acknowledge");
	  return 1;
	}
}

void TrajectorySampler::resetToHome(bool info)
{
	if (info)
		ROS_INFO("Moving to home");

	move_group.setStartStateToCurrentState();
	move_group.setNamedTarget("home_bow");
	runTrajectory(info);
}

TrajectorySampler::~TrajectorySampler() {}

int main (int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "trajectory_sampler");
	ros::NodeHandle nh;

	// ROS spinner
	ros::AsyncSpinner spinner(0);
	spinner.start();

	TrajectorySampler plan_sampler(nh);

	plan_sampler.resetToHome();
	plan_sampler.open_gripper();

	ros::ServiceServer service_job = nh.advertiseService("job_env", &TrajectorySampler::run_job, &plan_sampler);
	ros::ServiceServer service_idle = nh.advertiseService("job_idle", &TrajectorySampler::idle, &plan_sampler);
	ros::ServiceServer pick_place = nh.advertiseService("pick_place", &TrajectorySampler::pick_place, &plan_sampler);
	ROS_INFO("Trajectory Sampler started");

	ros::waitForShutdown();

	return 0;
}

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
