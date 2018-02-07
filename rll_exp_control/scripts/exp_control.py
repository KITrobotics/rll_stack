#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab stack
#
# Copyright (C) 2018 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import rospy
import docker

def start_exp():
    client = docker.from_env()

    ns = rospy.get_namespace()
    git_url = "https://gitlab.ipr.kit.edu/rll/moveit_testing_sender.git"

    command_string = "bash -c \"catkin_init_workspace && export ROS_NAMESPACE=" + ns + " && git clone " \
              + git_url + " src/moveit_testing_sender"\
              + " && catkin build && source devel/setup.bash && rosrun moveit_testing_sender target_spawn.py\""

    rospy.loginfo("command string: %s", command_string)

    # TODO: don't grant full access to host network and restrict
    #       resources (CPU, memory etc.)
    #       may also need to detach in order to kill container if it runs too long
    #       capture logs from container (from git clone, catkin build, etc.)
    client.containers.run("rll_exp_env:v1", network_mode="host",command=command_string)

if __name__ == '__main__':
    rospy.init_node('exp_control')
    start_exp()
