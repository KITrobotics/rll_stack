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
from pymongo import MongoClient
from os.path import expanduser

def run_job(jobs_collection, dClient, ns):
    rospy.loginfo("starting new job")

    git_url = "https://gitlab.ipr.kit.edu/rll/moveit_testing_sender.git"
    job_id = "test"

    command_string =  "./run_exp.sh " + git_url + " " + ns
    rospy.loginfo("command string: %s", command_string)

    # TODO: don't grant full access to host network and restrict
    #       resources (CPU, memory, disc space etc.)
    #       may also need to detach in order to be able to kill container if it runs too long
    job_logs = dClient.containers.run("rll_exp_env:v1", network_mode="host",command=command_string, stderr=True)

    rospy.loginfo("\n\ncontainer logs:\n\n%s", job_logs)
    log_file = expanduser("~/ros-job-logs/") + job_id + ".log"
    log_ptr = open(log_file, "w")
    log_ptr.write(job_logs)
    log_ptr.close()

if __name__ == '__main__':
    rospy.init_node('job_worker')

    ns = rospy.get_namespace()
    jobs_collection = MongoClient().rll_test.jobs
    dClient = docker.from_env()

    while not rospy.is_shutdown():
        run_job(jobs_collection, dClient, ns)
