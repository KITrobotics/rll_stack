#!/bin/bash
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

set -e

GIT_URL=$1 
ROS_PKG=$2
ROS_LAUNCH_FILE=$3
ROS_NS=$4

echo -e "\n\n\t-------------------------------------------------------------------------------"
echo -e "\t-------------------------------------------------------------------------------"
echo -e "\tgit clone $GIT_URL"
echo -e "\t-------------------------------------------------------------------------------"
echo -e "\t-------------------------------------------------------------------------------\n"
git clone --progress --verbose $GIT_URL src/$ROS_PKG

echo -e "\n\n\t------------------------------------------------------------------------------"
echo -e "\t------------------------------------------------------------------------------"
echo -e "\tcatkin build $ROS_PKG"
echo -e "\t------------------------------------------------------------------------------"
echo -e "\t------------------------------------------------------------------------------\n"
catkin build $ROS_PKG

source devel/setup.bash

echo -e "\n\n\t------------------------------------------------------------------------------"
echo -e "\t------------------------------------------------------------------------------"
echo -e "\troslaunch $ROS_PKG $ROS_LAUNCH_FILE robot:=$ROS_NS"
echo -e "\t------------------------------------------------------------------------------"
echo -e "\t------------------------------------------------------------------------------\n"
roslaunch $ROS_PKG $ROS_LAUNCH_FILE robot:=$ROS_NS
