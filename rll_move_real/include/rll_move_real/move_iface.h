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

#ifndef RLL_MOVE_IFACE_REAL_H
#define RLL_MOVE_IFACE_REAL_H

#include <std_srvs/Trigger.h>

#include <rll_move/move_iface.h>
#include <schunk_gripper_egl90/MoveGrip.h>
#include <schunk_gripper_egl90/MovePos.h>
#include <moveit_wrapper/moveit_wrapper.h>

class RLLMoveIfaceReal: public RLLMoveIface
{
public:
	explicit RLLMoveIfaceReal();

	bool close_gripper() override;
	bool open_gripper() override;

private:
	MoveItWrapper moveit_wrapper;

	bool modify_lin_trajectory(moveit_msgs::RobotTrajectory &trajectory) override;
	bool modify_ptp_trajectory(moveit_msgs::RobotTrajectory &trajectory) override;
	int gripper_reference_motion();
	int gripper_move_grip(float speed, float current);
	int gripper_stop();
	int gripper_move_pos(float pos);
	int gripper_acknowledge();
};

#endif  // RLL_MOVE_IFACE_REAL_H

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
