/*
 * This file is part of the Robot Learning Lab SDK
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_iface");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	RLLMoveIfaceReal move_iface;

	ros::ServiceServer service_job = nh.advertiseService("job_env", &RLLMoveIfaceReal::run_job, (RLLMoveIface*) &move_iface);
	ros::ServiceServer service_idle = nh.advertiseService("job_idle", &RLLMoveIfaceReal::idle, (RLLMoveIface*) &move_iface);
	ros::ServiceServer pick_place = nh.advertiseService("pick_place", &RLLMoveIfaceReal::pick_place, (RLLMoveIface*) &move_iface);
	ros::ServiceServer move_lin = nh.advertiseService("move_lin", &RLLMoveIfaceReal::move_lin, (RLLMoveIface*) &move_iface);
	ros::ServiceServer move_joints = nh.advertiseService("move_joints", &RLLMoveIfaceReal::move_joints, (RLLMoveIface*) &move_iface);

	ROS_INFO("RLL Move Real started");

	ros::waitForShutdown();

	return 0;
}

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
