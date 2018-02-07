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

import docker

def start_exp():
    client = docker.from_env()

    # TODO: don't grant full access to host network and restrict
    # resources (CPU, memory etc.)
    client.containers.run("rll_exp_env:v1", network_mode="host",
                          command='bash -c "catkin_init_workspace && export ROS_NAMESPACE=iiwa_1 && git clone https://gitlab.ipr.kit.edu/rll/moveit_testing_sender.git src/moveit_testing_sender && catkin build && source devel/setup.bash && rosrun moveit_testing_sender target_spawn.py"')

if __name__ == '__main__':
    start_exp()
