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
from rll_msgs.srv import JobEnv
from rll_msgs.msg import JobStatus

import docker
import pymongo
from git import Repo
import datetime
import time
import traceback
import re
from os import path, makedirs, remove
from shutil import rmtree
import sys
import datetime
import tarfile
from StringIO import StringIO

environment_containers = []

def job_loop(jobs_collection, dClient, ns):
    # rospy.loginfo("searching for new job in namespace '%s'", ns)
    global idle_start

    # TODO: use indexing
    job = jobs_collection.find_one_and_update({"project": project_name, "status": "submitted"},
                                              {"$set": {"status": "running",
                                                        "ns": ns,
                                                        "job_start": datetime.datetime.now()}},
                                              sort=[("created", pymongo.ASCENDING)])

    if job == None:
        rospy.sleep(0.1)
        return

    job_id = job["_id"]
    git_url = job["git_url"]
    git_tag = job["git_tag"]
    project = job["project"]
    username = job["username"]

    rospy.loginfo("got job with id '%s' for project '%s', Git URL '%s', Git Tag '%s' and create date %s", job_id,
                  project, git_url, git_tag, str(job["created"]))

    success, client_container = start_job(git_url, git_tag, username, job_id, project)
    if not success:
        rospy.loginfo("starting job failed, returning")
        return

    rospy.loginfo("running job env");
    try:
        rospy.wait_for_service("job_env", timeout=2.0)
    except:
        # reset the job and run it later again
        # TODO: be more transparent about this by using a different status code
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "submitted",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "job env not available"}})
        rospy.logfatal("Job env service is not available, please investigate!")
        sys.exit(1)

    try:
        job_env = rospy.ServiceProxy('job_env', JobEnv)
        resp = job_env(True)
        rospy.loginfo("successfully run job environment with job status %s", job_result_codes_to_string(resp.job.status))
    except rospy.ServiceException, e:
        rospy.logfatal("service call failed: %s, please investigate!", e)
        # reset the job and run it later again
        # TODO: be more transparent about this by using a different status code
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "submitted",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "job env not available"}})
        sys.exit(1)

    get_client_log(job_id, client_container)
    finish_container(client_container)

    jobs_collection.find_one_and_update({"_id": job_id},
                                        {"$set": {"status": "finished",
                                                  "job_end": datetime.datetime.now(),
                                                  "job_result": job_result_codes_to_string(resp.job.status)}})

    if (resp.job.status == JobStatus.INTERNAL_ERROR):
        rospy.logfatal("Internal error happened when running job environment, please investigate!")
        sys.exit(1)

    # reset robot and environment
    try:
        resp = job_idle(True)
    except rospy.ServiceException, e:
        rospy.loginfo("idle service call failed: %s", e)

    if (resp.job.status == JobStatus.INTERNAL_ERROR):
        rospy.logfatal("Internal error happened when running idle service, please investigate!")
        sys.exit(1)

    rospy.loginfo("finished job with id '%s' in namespace '%s'", job_id, ns)


def start_job(git_url, git_tag, username, job_id, project):
    try:
        #client container
        date = get_time_string()
        cc_name = "cc_" + date
        # terminal command to remove all containers from image "rll_exp_env:v1":
        # docker rm $(docker stop $(docker ps -a -q --filter ancestor=rll_exp_env:v1 --format="{{.ID}}"))
        client_container = create_container(cc_name, False)

    except:
        rospy.logerr("failed to create container:\n%s", traceback.format_exc())
        # reset job for rerun
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "submitted",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "creating container failed"}})
        return False

    # TODO: handle clone failures
    success = get_exp_code(git_url, git_tag, username, job_id, project, client_container)
    if not success:
        return False

    rospy.loginfo("building project")
    cmd_result = client_container.exec_run("catkin build --no-status", stdin=True, tty=True)
    write_logs(job_id, cmd_result[1], "build")
    if cmd_result[0] != 0:
        rospy.logerr("building project failed")

        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "finished",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "building project failed"}})
        finish_container(client_container)
        return False;

    launch_file = project_settings["launch_client"]
    launch_cmd = "roslaunch --disable-title " + project + " " + launch_file + " robot:=" + ns
    rospy.loginfo("running launch command %s", launch_cmd)
    # PYTHONUNBUFFERED needs to be disabled to ensure that data is piped before the app finishes and when we are detached
    client_container.exec_run("bash -c \"source devel/setup.bash && " + launch_cmd + " &> /tmp/client.log\"",
                              tty=True, detach=True, environment={"PYTHONUNBUFFERED": "0",
                                                                  "ROS_MASTER_URI": "http://" + ic_name + ":11311"})

    return True, client_container


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
        # wait a moment to make sure that the container is started
        rospy.sleep(0.2)
        container.exec_run("mkdir " + ws_repo_path)
        container.put_archive(ws_repo_path, frp)
        rospy.loginfo("uploaded project to container")

    remove(repo_archive_file)
    # TODO: create a proper archive and don't delete the repo
    rmtree(repo_clone_dir, ignore_errors=True)

    return True


def create_container(container_name, all_ports_open):

    return dClient.containers.create("rll_exp_env:v1", network=net_name,
                                     publish_all_ports=all_ports_open,
                                     detach=True, tty=True,
                                     nano_cpus=int(1e9), # limit to one CPU
                                     mem_limit="1g", # limit RAM to 1GB
                                     memswap_limit="1g", # limit RAM+SWAP to 1GB
                                     storage_opt={"size": "10G"}, # limit disk space to 10GB
                                     name = container_name)

def finish_container(container):
    container.stop()
    container.remove()

def get_client_log(job_id, container):
    log_folder = path.join(rll_settings["logs_save_dir"],str(job_id))
    log_file = path.join(log_folder, "client.log")
    size_counter = 0

    container.exec_run("chown root: /tmp/client.log", user="root")
    strm, status = container.get_archive("/tmp/client.log")
    with open(log_file, 'w') as outfile:
        for d in strm:
            size_counter += 1
            # chunks are 2MB and we limit log size to 10MB
            # so counter shouldn't be bigger than 5, but for some reason,
            # only this value is right
            if size_counter > 320:
                rospy.logwarn("Maximum log size exeeded")
                outfile.write("\nError: maximum log size exeeded\n")
                return
            outfile.write(d)

def write_logs(job_id, log_str, log_type):
    rospy.loginfo("\n%s logs:\n\n%s", log_type, log_str)

    # https://stackoverflow.com/questions/14693701/how-can-i-remove-the-ansi-escape-sequences-from-a-string-in-python
    # TODO: as an alternative, save without ANSI codes removal and use ansi2html for formatting in browser
    ansi_escape = re.compile(r'\x1B\[[0-?]*[ -/]*[@-~]')
    log_str = ansi_escape.sub('', log_str)

    log_folder = path.join(rll_settings["logs_save_dir"],str(job_id))
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


def setup_environment_container(dClient):

        dClient.networks.create(net_name);

        iface_container = create_container(ic_name, True)

        iface_container.start()
        rospy.loginfo("Started interface container: " + ic_name)
        environment_containers.append(iface_container)

        iface_container.exec_run("bash -c \"source devel/setup.bash && roscore\"", tty=True, detach=True, environment={"ROS_HOSTNAME": ic_name})
        
        # TODO: make this work
        # TODO: the iface container log can be cleared with "truncate -s 0 /tmp/iface.log"

        # launch_file = project_settings["launch_iface"]
        # launch_cmd = "roslaunch --disable-title " + project_name + " " + launch_file + " robot:=" + ns
        # ic_result = interface_container.exec_run("bash -c \"source devel/setup.bash && " + launch_cmd + "\"",
        #                                          stdin=True, detach=True, tty=True)


def on_shutdown_call():
    print "shutdown call received! Trying to shutdown environment containers"
    for cont in environment_containers:
        finish_container(cont)

def get_time_string():
        cur_time = time.time()
        value = datetime.datetime.fromtimestamp(cur_time)
        return value.strftime('%d_%H-%M-%S')


if __name__ == '__main__':
    rospy.init_node('job_worker')

    # register shutdown call
    rospy.on_shutdown(on_shutdown_call)

    ns = rospy.get_namespace()

    # settings
    rospack = rospkg.RosPack()
    config_path = rospack.get_path('rll_config') + "/config/rll.yaml"
    with open(config_path, 'r') as doc:
        rll_settings = yaml.load(doc)
        db_name = rll_settings["db_name"]
        rospy.loginfo("using database %s", db_name)

    project_name = rospy.get_param("~project")
    rospy.loginfo("processing project %s", project_name)
    try:
        project_settings = rll_settings["project_settings"][project_name]
    except:
        rospy.logfatal("failed to retrieve project settings")
        sys.exit(1)

    db_client = pymongo.MongoClient()
    jobs_collection = db_client[db_name].jobs

    job_idle = rospy.ServiceProxy("job_idle", JobEnv)

    dClient = docker.from_env()
    # TODO: us ns for naming this
    date = get_time_string()
    ic_name = "ic_" + date
    net_name = "testing" + date
    setup_environment_container(dClient)

    while not rospy.is_shutdown():
        job_loop(jobs_collection, dClient, ns)
