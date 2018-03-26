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

double cartesian_velocity = 0.25; //in m/s
double cartesian_acceleration = 0.5; //in m/s²

TrajectorySampler::TrajectorySampler(ros::NodeHandle nh)
	: move_group(PLANNING_GROUP), moveit_wrapper(PLANNING_GROUP)
{
	// configure planner
	move_group.setPlannerId("RRTConnectkConfigDefault");
	move_group.setPlanningTime(2.0);
	move_group.setPoseReferenceFrame("world");
	// slow down movement of the robot
	move_group.setMaxVelocityScalingFactor(0.5);

	std::string ee_link;
	std::string ns = ros::this_node::getNamespace();
	ROS_INFO("starting in ns %s", ns.c_str());
	if (ns == "//iiwa_1")
		ee_link = "iiwa_1_gripper_link_ee";
	else if (ns == "//iiwa_2")
		ee_link = "iiwa_2_gripper_link_ee";
	else
		ee_link = "iiwa_gripper_link_ee";
	move_group.setEndEffectorLink(ee_link);
	moveit_wrapper.setTCP(ee_link);

	my_iiwa.init();

	// do initial home reset already here, because if TCP is in a too upright
	// position, setting the path parameters fails
	resetToHome();

	my_iiwa.getPathParametersService().setJointRelativeVelocity(1.0);
	my_iiwa.getPathParametersService().setJointRelativeAcceleration(1.0);
	my_iiwa.getPathParametersService().setOverrideJointAcceleration(3.5);

	gripper_reference_motion();
	open_gripper();
}

bool TrajectorySampler::run_job(rll_worker::JobEnv::Request &req,
				rll_worker::JobEnv::Response &resp)
{
	ROS_INFO("got job running request");

	return true;
}

bool TrajectorySampler::idle(rll_worker::JobEnv::Request &req,
			     rll_worker::JobEnv::Response &resp)
{
	// Send home position when idling
	// This ensures that the brakes are not activated and the control cycle keeps running.
	// If we don't do this, the robot won't move when a trajectory is sent and the brakes are active.
	if (my_iiwa.getRobotIsConnected()) {
		resetToHome(false);
		open_gripper();
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
	std::vector<geometry_msgs::Pose> waypoints_grip;
	std::vector<geometry_msgs::Pose> waypoints_away;
	moveit_msgs::RobotTrajectory trajectory;
	const double eef_step = 0.001;
	const double jump_threshold = 1000.0;

	ROS_INFO("Moving above target");
	move_group.setStartStateToCurrentState();
	waypoints_to.push_back(req.pose_above);
	double achieved = move_group.computeCartesianPath(waypoints_to,
							  eef_step, jump_threshold, trajectory);
	if (achieved < 1 && achieved > 0) {
		ROS_ERROR("only achieved to compute %f of the requested path", achieved);
		resp.success = false;
		return true;
	} else if (achieved <= 0) {
		ROS_ERROR("path planning completely failed");
		resp.success = false;
		return true;
	}

	auto error = moveit_wrapper.parametrize_cartesian_time(trajectory, cartesian_velocity, cartesian_acceleration);
	if (!error) {
		ROS_ERROR("cartesian time parametrization failed");
		resp.success = false;
		return true;
	}

	my_plan.trajectory_= trajectory;

	success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path execution failed");
		resp.success = false;
		return true;
	}

	ROS_INFO("Moving to grip position");
	move_group.setStartStateToCurrentState();
	waypoints_grip.push_back(req.pose_grip);
	achieved = move_group.computeCartesianPath(waypoints_grip,
						   eef_step, jump_threshold, trajectory);
	if (achieved < 1 && achieved > 0) {
		ROS_ERROR("only achieved to compute %f of the requested path", achieved);
		resp.success = false;
		return true;
	} else if (achieved <= 0) {
		ROS_ERROR("path planning completely failed");
		resp.success = false;
		return true;
	}

	error = moveit_wrapper.parametrize_cartesian_time(trajectory, cartesian_velocity, cartesian_acceleration);
	if (!error) {
		ROS_ERROR("cartesian time parametrization failed");
		resp.success = false;
		return true;
	}

	my_plan.trajectory_= trajectory;

	success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path execution failed");
		resp.success = false;
		return true;
	}



	if (req.gripper_close)
		close_gripper();
	else
		open_gripper();

	ROS_INFO("Moving above grip position");
	move_group.setStartStateToCurrentState();
	waypoints_away.push_back(req.pose_above);
	move_group.computeCartesianPath(waypoints_away, eef_step, jump_threshold, trajectory);

	error = moveit_wrapper.parametrize_cartesian_time(trajectory, cartesian_velocity, cartesian_acceleration);
	if (!error) {
		ROS_ERROR("cartesian time parametrization failed");
		resp.success = false;
		return true;
	}

	my_plan.trajectory_= trajectory;

	success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path execution failed");
		resp.success = false;
		return true;
	}

	resp.success = true;
	return true;
}

bool TrajectorySampler::move_lin(rll_moveit_testing::MoveLin::Request &req,
				 rll_moveit_testing::MoveLin::Response &resp)
{
	bool success;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	std::vector<geometry_msgs::Pose> waypoints;
	moveit_msgs::RobotTrajectory trajectory;
	const double eef_step = 0.001;
	const double jump_threshold = 1000.0;

	ROS_INFO("Lin motion requested");
	move_group.setStartStateToCurrentState();
	waypoints.push_back(req.pose);
	double achieved = move_group.computeCartesianPath(waypoints,
							  eef_step, jump_threshold, trajectory);
	if (achieved < 1 && achieved > 0) {
		ROS_ERROR("only achieved to compute %f of the requested path", achieved);
		resp.success = false;
		return true;
	} else if (achieved <= 0) {
		ROS_ERROR("path planning completely failed");
		resp.success = false;
		return true;
	}

	auto error = moveit_wrapper.parametrize_cartesian_time(trajectory, cartesian_velocity, cartesian_acceleration);
	if (!error) {
		ROS_ERROR("cartesian time parametrization failed");
		resp.success = false;
		return true;
	}

	my_plan.trajectory_= trajectory;

	success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path execution failed");
		resp.success = false;
		return true;
	}

	resp.success = true;
	return true;
}

bool TrajectorySampler::move_joints(rll_moveit_testing::MoveJoints::Request &req,
				    rll_moveit_testing::MoveJoints::Response &resp)
{
	bool success;
	std::vector<double> joints;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	ROS_INFO("Joint motion requested");

	move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), joints);
	joints[0] = req.joint_1;
	joints[1] = req.joint_2;
	joints[2] = req.joint_3;
	joints[3] = req.joint_4;
	joints[4] = req.joint_5;
	joints[5] = req.joint_6;
	joints[6] = req.joint_7;

	move_group.setStartStateToCurrentState();
	success = move_group.setJointValueTarget(joints);
	if (!success) {
		ROS_ERROR("requested joint values out of range");
		resp.success = false;
		return true;
	}

	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path planning failed");
		resp.success = false;
		return true;
	}

	auto error = moveit_wrapper.parametrize_time(my_plan.trajectory_, 1, 1);
	if (!error) {
		ROS_ERROR("cartesian time parametrization failed");
		resp.success = false;
		return true;
	}

	success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path execution failed");
		resp.success = false;
		return true;
	}

	resp.success = true;
	return true;
}

bool TrajectorySampler::runTrajectory(bool info)
{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success_plan;

	success_plan = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (info)
		ROS_INFO("Planning result: %s",
			 success_plan ? "SUCCEEDED" : "FAILED");

	auto error = moveit_wrapper.parametrize_time(my_plan.trajectory_, 1, 1);
	if (!error) {
		ROS_ERROR("cartesian time parametrization failed");
		return false;
	}

	if (success_plan) {
		success_plan = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if (info)
			ROS_INFO("Plan execution result: %s",
				 success_plan ? "SUCCEEDED" : "FAILED");
	} else {
		ROS_WARN("Not executing because planning failed");
		return false;
	}

	return true;
}

void TrajectorySampler::close_gripper()
{
	gripper_move_grip(-80.0, 0.25);
	gripper_acknowledge();
}

void TrajectorySampler::open_gripper()
{
	gripper_move_pos(81.0);
	gripper_acknowledge();
}

int TrajectorySampler::gripper_reference_motion()
{
	ros::NodeHandle n_ref_mot;
	ros::ServiceClient c_ref_mot = n_ref_mot.serviceClient<std_srvs::Trigger>("gripper_egl_90/reference_motion");
	std_srvs::Trigger srv;
	if (c_ref_mot.call(srv))
	{
		ROS_DEBUG("gripper: moving to reference position...");
	}
	else
	{
		ROS_ERROR("gripper: failed to call service reference motion");
		return 1;
	}
}

int TrajectorySampler::gripper_move_grip(float speed, float current)
{
	ros::NodeHandle n_move_grip;
	ros::ServiceClient c_move_grip = n_move_grip.serviceClient<schunk_gripper_egl90::MoveGrip>("gripper_egl_90/move_grip");
	schunk_gripper_egl90::MoveGrip srv;
	srv.request.speed = speed;
	srv.request.current = current;
	if (c_move_grip.call(srv)) {
		ROS_INFO("Moving the gripper fingers...");
	} else {
		ROS_ERROR("gripper: ailed to call service move_grip");
		return 1;
	}
}

int TrajectorySampler::gripper_stop()
{
	ros::NodeHandle n_stop;
	ros::ServiceClient c_stop = n_stop.serviceClient<std_srvs::Trigger>("gripper_egl_90/stop");
	std_srvs::Trigger srv;
	if (c_stop.call(srv)) {
		ROS_INFO("Stopping gripper...");
	} else {
		ROS_ERROR("gripper: failed to call service stop");
		return 1;
	}
}

int TrajectorySampler::gripper_move_pos(float pos)
{
	ros::NodeHandle n_move_pos;
	ros::ServiceClient c_move_pos = n_move_pos.serviceClient<schunk_gripper_egl90::MovePos>("gripper_egl_90/move_pos");
	schunk_gripper_egl90::MovePos srv;
	srv.request.position = pos;
	if (c_move_pos.call(srv))
	{
		ROS_INFO("gripper: moving to an specific position...");
	}
	else
	{
		ROS_ERROR("gripper: failed to call service move_pos");
		return 1;
	}
}

int TrajectorySampler::gripper_acknowledge()
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
		ROS_ERROR("gripper: failed to call service acknowledge");
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

	ros::ServiceServer service_job = nh.advertiseService("job_env", &TrajectorySampler::run_job, &plan_sampler);
	ros::ServiceServer service_idle = nh.advertiseService("job_idle", &TrajectorySampler::idle, &plan_sampler);
	ros::ServiceServer pick_place = nh.advertiseService("pick_place", &TrajectorySampler::pick_place, &plan_sampler);
	ros::ServiceServer move_lin = nh.advertiseService("move_lin", &TrajectorySampler::move_lin, &plan_sampler);
	ros::ServiceServer move_joints = nh.advertiseService("move_joints", &TrajectorySampler::move_joints, &plan_sampler);
	ROS_INFO("Simple Moveit Interface started");

	ros::waitForShutdown();

	return 0;
}

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
