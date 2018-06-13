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

#include <rll_move_real/move_iface.h>

double cartesian_velocity = 0.4; //in m/s
double cartesian_acceleration = 1; //in m/s²

RLLMoveIfaceReal::RLLMoveIfaceReal()
	: moveit_wrapper(MANIP_PLANNING_GROUP)
{
	moveit_wrapper.setTCP(manip_move_group.getEndEffectorLink());
}

bool RLLMoveIfaceReal::modify_ptp_trajectory(moveit_msgs::RobotTrajectory &trajectory)
{
	auto success = moveit_wrapper.parametrize_time(trajectory, 1, 1);
	if (!success) {
		ROS_ERROR("time parametrization failed");
		return false;
	}

	return true;
}

bool RLLMoveIfaceReal::modify_lin_trajectory(moveit_msgs::RobotTrajectory &trajectory)
{
	auto success = moveit_wrapper.parametrize_cartesian_time(trajectory, cartesian_velocity, cartesian_acceleration);
	if (!success) {
		ROS_ERROR("cartesian time parametrization failed");
		return false;
	}

	return true;
}

bool RLLMoveIfaceReal::close_gripper()
{
	ROS_INFO("Closing the real gripper");

	gripper_move_grip(-80.0, 0.3);
	gripper_acknowledge();

	return true;
}

bool RLLMoveIfaceReal::open_gripper()
{
	ROS_INFO("Opening the real gripper");

	gripper_move_pos(75.0);
	gripper_acknowledge();

	return true;
}

int RLLMoveIfaceReal::gripper_reference_motion()
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

int RLLMoveIfaceReal::gripper_move_grip(float speed, float current)
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

int RLLMoveIfaceReal::gripper_stop()
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

int RLLMoveIfaceReal::gripper_move_pos(float pos)
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

int RLLMoveIfaceReal::gripper_acknowledge()
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

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
