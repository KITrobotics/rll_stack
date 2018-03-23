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
import rospkg
import yaml

import docker
import pymongo
import datetime
import time

from rll_worker.srv import *
from rll_worker.msg import *

# the iiwas activate their brakes around 45 minutes of inactivity
# work around this by sending idle at least every iiwa_timeout
iiwa_timeout = 0.3 * 3600
idle_start = time.time()

def run_job(jobs_collection, dClient, ns):
    # rospy.loginfo("searching for new job in namespace '%s'", ns)
    global idle_start
    job_idle = rospy.ServiceProxy("job_idle", JobEnv)

    # TODO: use indexing
    job = jobs_collection.find_one_and_update({"status": "submitted"},
                                              {"$set": {"status": "running",
                                                        "ns": ns,
                                                        "job_start": datetime.datetime.now()}},
                                              sort=[("created", pymongo.ASCENDING)])

    if job == None:
        # rospy.loginfo("no job in queue")

        if (time.time() - idle_start) > iiwa_timeout:
            rospy.wait_for_service("job_idle")
            try:
                resp = job_idle(True)
            except rospy.ServiceException, e:
                rospy.loginfo("service call failed: %s", e)
            idle_start = time.time()
        else:
            rospy.sleep(0.1)
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
        # TODO: be more transparent about this by using a different status code or by improving
        #       the job run script
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "submitted"}})
        return

    rospy.loginfo("\n\ncontainer logs:\n\n%s", job_logs)
    log_file = rll_settings["logs_save_dir"] + "/" + str(job_id) + ".log"
    log_ptr = open(log_file, "w")
    log_ptr.write(job_logs)
    log_ptr.close()
    rospy.loginfo("wrote logs to disk")

    jobs_collection.find_one_and_update({"_id": job_id},
                                        {"$set": {"status": "finished",
                                                  "job_end": datetime.datetime.now(),
                                                  "job_result": job_result_codes_to_string(resp.job.status)}})

    try:
        resp = job_idle(True)
    except rospy.ServiceException, e:
        rospy.loginfo("service call failed: %s", e)

    rospy.loginfo("finished job with id '%s' in namespace '%s'", job_id, ns)

    # reset time counter
    idle_start = time.time()

def job_result_codes_to_string(status):
    job_codes = {JobStatus.SUCCESS: "success", JobStatus.FAILURE: "failure",
                 JobStatus.INTERNAL_ERROR: "internal error"}
    return job_codes.get(status, "unknown")

if __name__ == '__main__':
    rospy.init_node('job_worker')

    ns = rospy.get_namespace()

    # settings
    rospack = rospkg.RosPack()
    config_path = rospack.get_path('rll_description') + "/config/rll.yaml"
    with open(config_path, 'r') as doc:
        rll_settings = yaml.load(doc)
        db_name = rll_settings["db_name"]
        rospy.loginfo("using database %s", db_name)

    db_client = pymongo.MongoClient()
    jobs_collection = db_client[db_name].jobs

    dClient = docker.from_env()

    while not rospy.is_shutdown():
        run_job(jobs_collection, dClient, ns)
