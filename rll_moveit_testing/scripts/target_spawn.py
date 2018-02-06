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

def update_spawn_location():

    # set target positions
    rospy.set_param('target_1_spawn_location', {'x': 0.5, 'y': -0.3, 'z': 0.2})
    rospy.set_param('target_2_spawn_location', {'x': 0.3, 'y': 0.3, 'z': 0.4})

    # start execution
    rospy.set_param('target_pos_set', True)
    
if __name__ == '__main__':
    update_spawn_location()
