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
from rll_worker.srv import *
from rll_worker.msg import *

import docker
import pymongo
from git import Repo
import datetime
import time
import traceback
import re
from os import path, makedirs, remove
from shutil import rmtree

# the iiwas activate their brakes around 45 minutes of inactivity
# work around this by sending idle at least every iiwa_timeout
iiwa_timeout = 0.3 * 3600
idle_start = time.time()

def job_loop(jobs_collection, dClient, ns):
    # rospy.loginfo("searching for new job in namespace '%s'", ns)
    global idle_start

    # TODO: use indexing
    job = jobs_collection.find_one_and_update({"status": "submitted"},
                                              {"$set": {"status": "running",
                                                        "ns": ns,
                                                        "job_start": datetime.datetime.now()}},
                                              sort=[("created", pymongo.ASCENDING)])

    if job == None:
        job_idling()
        return

    job_id = job["_id"]
    git_url = job["git_url"]
    git_tag = job["git_tag"]
    # TODO: for now same as package name
    project = job["project"]
    username = job["username"]

    rospy.loginfo("got job with id '%s' for project '%s', Git URL '%s', Git Tag '%s' and create date %s", job_id,
                  project, git_url, git_tag, str(job["created"]))

    success = run_job(git_url, git_tag, username, job_id, project)
    if not success:
        rospy.loginfo("running job failed, returning")
        return
    else:
        rospy.loginfo("running job succeeded")

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
                                            {"$set": {"status": "submitted",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "job env not available"}})
        return

    jobs_collection.find_one_and_update({"_id": job_id},
                                        {"$set": {"status": "finished",
                                                  "job_end": datetime.datetime.now(),
                                                  "job_result": job_result_codes_to_string(resp.job.status)}})

    # reset robot and environment
    try:
        resp = job_idle(True)
    except rospy.ServiceException, e:
        rospy.loginfo("service call failed: %s", e)

    rospy.loginfo("finished job with id '%s' in namespace '%s'", job_id, ns)

    # reset time counter
    idle_start = time.time()


def job_idling():
    global idle_start

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


def run_job(git_url, git_tag, username, job_id, project):
    try:
        # TODO: don't grant full access to host network
        container = dClient.containers.create("rll_exp_env:v2", network_mode="host",
                                              detach=True, tty=True,
                                              nano_cpus=int(1e9), # limit to one CPU
                                              mem_limit="1g", # limit RAM to 1GB
                                              memswap_limit="1g", # limit RAM+SWAP to 1GB
                                              storage_opt={"size": "10G"}) # limit disk space to 10GB
    except:
        rospy.logerr("failed to create container:\n%s", traceback.format_exc())
        # reset job for rerun
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "submitted",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "creating container failed"}})
        return False

    # TODO: handle clone failures
    success = get_exp_code(git_url, git_tag, username, job_id, project, container)
    if not success:
        return False

    cmd_result = container.exec_run("catkin build --no-status", stdin=True, tty=True)
    write_logs(job_id, cmd_result[1], "build")
    if cmd_result[0] != 0:
        rospy.logerr("building project failed")

        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "finished",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "building project failed"}})
        finish_container(container)
        return False;

    # TODO: read from config file
    launch_file = "move_sender.launch"
    launch_cmd = "roslaunch --disable-title " + project + " " + launch_file + " robot:=" + ns
    # TODO: we need to detach here (detach=True)
    cmd_result = container.exec_run("bash -c \"source devel/setup.bash && " + launch_cmd + "\"" , stdin=True)
    write_logs(job_id, cmd_result[1], "launch")
    if cmd_result[0] != 0:
        rospy.logerr("launching project failed")

        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "finished",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "launching project failed"}})
        finish_container(container)
        return False;

    finish_container(container)
    return True


def get_exp_code(git_url, git_tag, username, job_id, project, container):
    repo_clone_dir = rll_settings["git_archive_dir"] + "/" + username + "/" + project
    repo_archive_file = repo_clone_dir + ".tar"
    ws_repo_path = "/home/rll_user/ws/src/" + project

    repo = Repo.clone_from(git_url, repo_clone_dir, branch=git_tag) # TODO: --single-branch for not cloning all branches
    if repo.__class__ is not Repo:
        rospy.logerr("cloning project repo failed")
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "finished",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "fetching project code failed"}})
        return False

    with open(repo_archive_file, 'wb') as fwp:
        repo.archive(fwp)

    with open(repo_archive_file, 'rb') as frp:
        container.start()
        container.exec_run("mkdir " + ws_repo_path)
        container.put_archive(ws_repo_path, frp)
        rospy.loginfo("uploaded project to container")

    remove(repo_archive_file)
    # TODO: create a proper archive and don't delete the repo
    rmtree(repo_clone_dir, ignore_errors=True)

    return True


def finish_container(container):
    container.stop()
    container.remove()

def write_logs(job_id, log_str, log_type):
    rospy.loginfo("\n%s logs:\n\n%s", log_type, log_str)

    # https://stackoverflow.com/questions/14693701/how-can-i-remove-the-ansi-escape-sequences-from-a-string-in-python
    # TODO: as an alternative, save without ANSI codes removal and use ansi2html for formatting in browser
    ansi_escape = re.compile(r'\x1B\[[0-?]*[ -/]*[@-~]')
    log_str = ansi_escape.sub('', log_str)

    log_folder = rll_settings["logs_save_dir"] + "/" + str(job_id)
    if not path.exists(log_folder):
        makedirs(log_folder)
    log_file = log_folder + "/" + log_type  + ".log"
    log_ptr = open(log_file, "w")
    log_ptr.write(log_str)
    log_ptr.close()
    rospy.loginfo("wrote %s logs to disk", log_type)


def job_result_codes_to_string(status):
    job_codes = {JobStatus.SUCCESS: "success", JobStatus.FAILURE: "failure",
                 JobStatus.INTERNAL_ERROR: "internal error"}
    return job_codes.get(status, "unknown")


if __name__ == '__main__':
    rospy.init_node('job_worker')

    ns = rospy.get_namespace()

    # settings
    rospack = rospkg.RosPack()
    config_path = rospack.get_path('rll_config') + "/config/rll.yaml"
    with open(config_path, 'r') as doc:
        rll_settings = yaml.load(doc)
        db_name = rll_settings["db_name"]
        rospy.loginfo("using database %s", db_name)

    db_client = pymongo.MongoClient()
    jobs_collection = db_client[db_name].jobs

    dClient = docker.from_env()
    job_idle = rospy.ServiceProxy("job_idle", JobEnv)

    while not rospy.is_shutdown():
        job_loop(jobs_collection, dClient, ns)
