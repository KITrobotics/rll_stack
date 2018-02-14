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
import pymongo
from os.path import expanduser
import datetime

from rll_worker.srv import *
from rll_worker.msg import *

def run_job(jobs_collection, dClient, ns):
    # rospy.loginfo("searching for new job in namespace '%s'", ns)

    # TODO: use indexing
    job = jobs_collection.find_one_and_update({"status": "submitted"},
                                              {"$set": {"status": "running",
                                                        "ns": ns,
                                                        "job_start": datetime.datetime.now()}},
                                              sort=[("created", pymongo.ASCENDING)])

    if job == None:
        # rospy.loginfo("no job in queue")
        rospy.sleep(1.)
        return

    job_id = job["_id"]
    git_url = job["git_url"]

    rospy.loginfo("got job with id '%s', Git URL '%s' and create date %s", job_id, git_url,
                  str(job["created"]))

    command_string =  "./run_exp.sh " + git_url + " " + ns
    rospy.loginfo("command string: %s", command_string)

    try:
        # TODO: don't grant full access to host network and restrict
        #       resources (CPU, memory, disc space etc.)
        #       may also need to detach in order to be able to kill container if it runs too long
        job_logs = dClient.containers.run("rll_exp_env:v1", network_mode="host",command=command_string,
                                      stderr=True)
    except:
        rospy.logerr("failed to run container")
        # TODO: try to provide more details for this case (logs etc.?)
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "finished",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "failed"}})
        return

    rospy.wait_for_service("job_env")
    try:
        job_env = rospy.ServiceProxy('job_env', JobEnv)
        resp = job_env(True)
        rospy.loginfo("successfully run job environment with job status %d", resp.job.status)
    except rospy.ServiceException, e:
        rospy.loginfo("service call failed: %s", e)
        # reset the job and run it later again
        # TODO: be more transparent about this by using a different status code
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "submitted"}})
        return

    rospy.loginfo("\n\ncontainer logs:\n\n%s", job_logs)
    log_file = expanduser("~/ros-job-logs/") + str(job_id) + ".log"
    log_ptr = open(log_file, "w")
    log_ptr.write(job_logs)
    log_ptr.close()

    # TODO: also set status
    jobs_collection.find_one_and_update({"_id": job_id},
                                        {"$set": {"status": "finished",
                                                  "job_end": datetime.datetime.now(),
                                                  "job_result": job_result_codes_to_string(resp.job.status)}})

    rospy.loginfo("finished job with id '%s' in namespace '%s'", job_id, ns)

def job_result_codes_to_string(status):
    job_codes = {JobStatus.SUCCESS: "success", JobStatus.FAILURE: "failure",
                 JobStatus.INTERNAL_ERROR: "internal error"}
    return job_codes.get(status, "unknown")

if __name__ == '__main__':
    rospy.init_node('job_worker')

    ns = rospy.get_namespace()
    jobs_collection = pymongo.MongoClient().rll_test.jobs
    dClient = docker.from_env()

    while not rospy.is_shutdown():
        run_job(jobs_collection, dClient, ns)
