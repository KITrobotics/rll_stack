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
import rosgraph
import yaml
import actionlib
from rll_msgs.msg import *

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
import socket

environment_containers = []
topic_sync_names = {}

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

    available = job_env.wait_for_server(rospy.Duration.from_sec(2.0))
    if not available:
        # reset the job and run it later again
        # TODO: be more transparent about this by using a different status code
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "submitted",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "job env not available"}})
        rospy.logfatal("Job env action server is not available, please investigate!")
        sys.exit(1)

    job_env_goal = JobEnvGoal()
    job_env.send_goal(job_env_goal)
    rospy.loginfo("running job env")

    # TODO: handle this better
    rospy.sleep(0.5)
    sync_to_host_master()

    # TODO: have a timeout here and maybe request feedback
    job_env.wait_for_result()
    resp = job_env.get_result()
    rospy.loginfo("successfully run job environment with job status %s\n", job_result_codes_to_string(resp.job.status))

    get_client_log(job_id, client_container)
    get_iface_log(job_id, iface_container)
    unregister_client()
    finish_container(client_container)

    jobs_collection.find_one_and_update({"_id": job_id},
                                        {"$set": {"status": "finished",
                                                  "job_end": datetime.datetime.now(),
                                                  "job_result": job_result_codes_to_string(resp.job.status)}})

    if (resp.job.status == JobStatus.INTERNAL_ERROR):
        rospy.logfatal("Internal error happened when running job environment, please investigate (job ID %s)!", job_id)
        sys.exit(1)

    # reset robot and environment
    job_idle_goal = JobEnvGoal()
    job_idle.send_goal(job_idle_goal)
    job_idle.wait_for_result()
    resp = job_idle.get_result()
    if (resp.job.status == JobStatus.INTERNAL_ERROR):
        rospy.logfatal("Internal error happened when running idle service, please investigate!")
        sys.exit(1)

    rospy.loginfo("finished job with id '%s' in namespace '%s'\n", job_id, ns)


def start_job(git_url, git_tag, username, job_id, project):
    try:
        #client container
        cc_name = "cc_" + ns.replace("/", "")
        # terminal command to remove all containers from image "rll-base":
        # docker rm $(docker stop $(docker ps -a -q --filter ancestor=rll-base --format="{{.ID}}"))
        client_container = create_container(cc_name, "rll-base", False)

    except:
        rospy.logfatal("failed to create container:\n%s", traceback.format_exc())
        # reset job for rerun
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "submitted",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "creating container failed"}})

        sys.exit(1)

    # TODO: handle clone failures
    success = get_exp_code(git_url, git_tag, username, job_id, project, client_container)
    if not success:
        return False, client_container

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
        return False, client_container

    launch_file = project_settings["launch_client"]
    launch_cmd = "roslaunch --disable-title " + project + " " + launch_file + " robot:=" + ns
    client_container.reload()
    client_container_ip = client_container.attrs["NetworkSettings"]["Networks"][net_name]["IPAddress"]

    rospy.loginfo("running launch command %s", launch_cmd)
    # PYTHONUNBUFFERED needs to be disabled to ensure that data is piped before the app finishes and when we are detached
    # stdbuf -o0 does the same for C/C++ apps
    client_container.exec_run("bash -c \"source devel/setup.bash && stdbuf -o0 " + launch_cmd + " &> /tmp/client.log\"",
                              tty=True, detach=True, environment={"PYTHONUNBUFFERED": "0",
                                                                  "ROS_IP": client_container_ip,
                                                                  "ROS_MASTER_URI": "http://" + ic_name + ":11311"})
    # giving the client a little time to start
    # TODO: handle this better with syncing between masters
    rospy.sleep(2)

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
        rospy.sleep(1)
        container.exec_run("mkdir " + ws_repo_path)
        container.put_archive(ws_repo_path, frp)
        rospy.loginfo("uploaded project to container")

    remove(repo_archive_file)
    # TODO: create a proper archive and don't delete the repo
    rmtree(repo_clone_dir, ignore_errors=True)

    return True


def create_container(container_name, image_name, all_ports_open):

    return dClient.containers.create(image_name, network=net_name,
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

def get_iface_log(job_id, container):
    log_folder = path.join(rll_settings["logs_save_dir"],str(job_id))
    log_file = path.join(log_folder, "iface.log")

    container.exec_run("cp /tmp/iface.log /tmp/iface.log.bak", user="root")
    container.exec_run("chown root: /tmp/iface.log.bak", user="root")
    strm, status = container.get_archive("/tmp/iface.log.bak")
    with open(log_file, 'w') as outfile:
        for d in strm:
            outfile.write(d)

    container.exec_run("rm /tmp/iface.log.bak", user="root")
    container.exec_run("truncate -s 0 /tmp/iface.log")

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
    iface_container = create_container(ic_name, project_settings["iface_docker_image"], True)

    iface_container.start()
    rospy.loginfo("Started interface container: " + ic_name)
    environment_containers.append(iface_container)

    iface_container.reload()
    iface_container_ip = iface_container.attrs["NetworkSettings"]["Networks"][net_name]["IPAddress"]
    rospy.loginfo("iface container ip: " + iface_container_ip)

    # start ROS master
    iface_container.exec_run("bash -c \"source devel/setup.bash && roscore\"", tty=True, detach=True, environment={"ROS_HOSTNAME": ic_name})

    # TODO: the iface container log can be cleared with "truncate -s 0 /tmp/iface.log"

    launch_file = project_settings["launch_iface"]
    launch_cmd = "roslaunch --disable-title " + project_settings["iface_project"] + " " + launch_file + " robot:=" + ns + " headless:=true"
    rospy.loginfo("running iface launch command %s", launch_cmd)
    ic_result = iface_container.exec_run("bash -c \"source devel/setup.bash && stdbuf -o0 " + launch_cmd + " &> /tmp/iface.log\"",
                                         tty=True, detach=True, environment={"PYTHONUNBUFFERED": "0", "ROS_IP": iface_container_ip,
                                                                             "ROS_MASTER_URI": "http://" + host_ip + ":11311"})

    rospy.loginfo("waiting for the client master and the iface to come up")
    rospy.sleep(10)
    sync_to_client_master(iface_container_ip)

    return iface_container

def unregister_client():
    host_master_uri = "http://" + host_ip + ":11311"
    host_master = rosgraph.Master(topic_sync_names["host"], master_uri=host_master_uri)
    iface_container_ip = iface_container.attrs["NetworkSettings"]["Networks"][net_name]["IPAddress"]
    client_master_uri = "http://" + iface_container_ip + ":11311"
    client_master = rosgraph.Master(topic_sync_names["client"], master_uri=client_master_uri)

    for action_name in project_settings["sync_actions_to_host"]:
        action_topics = []
        try:
            action_node = client_master.lookupNode(project_settings["action_publisher"][action_name])
        except:
            continue
        action_topics.append(ns + action_name + "/cancel")
        action_topics.append(ns + action_name + "/feedback")
        action_topics.append(ns + action_name + "/goal")
        action_topics.append(ns + action_name + "/result")
        action_topics.append(ns + action_name + "/status")
        for action_topic in action_topics:
            num_host_pub = host_master.unregisterPublisher(action_topic, action_node)
            num_host_sub = host_master.unregisterSubscriber(action_topic, action_node)
            num_client_pub = client_master.unregisterPublisher(action_topic, action_node)
            num_client_sub = client_master.unregisterSubscriber(action_topic, action_node)
            # rospy.loginfo("num_host_pub %d num_host_sub %d num_client_pub %d num_client_sub %d", num_host_pub, num_host_sub, num_client_pub, num_client_sub)
            # rospy.loginfo("unregistered publisher %s with node uri %s", action_topic, action_node)

def cleanup_host_master():
    host_master_uri = "http://" + host_ip + ":11311"
    host_master = rosgraph.Master(rospy.get_name(), master_uri=host_master_uri)

    for srv_name in project_settings["sync_services_to_client"]:
        srv_full_name = ns + srv_name
        try:
            srv_uri = host_master.lookupService(srv_full_name)
            rospy.loginfo("cleaning up %s", srv_uri)
            if srv_uri:
                host_master.unregisterService(srv_full_name, srv_uri)
        except:
            pass

def sync_to_host_master():
    iface_container_ip = iface_container.attrs["NetworkSettings"]["Networks"][net_name]["IPAddress"]
    topic_sync_names["client"] = rosgraph.names.anonymous_name('master_sync')
    client_master_uri = "http://" + iface_container_ip + ":11311"
    client_master = rosgraph.Master(topic_sync_names["client"], master_uri=client_master_uri)
    topic_sync_names["host"] = rosgraph.names.anonymous_name('master_sync')
    host_master_uri = "http://" + host_ip + ":11311"
    host_master = rosgraph.Master(topic_sync_names["host"], master_uri=host_master_uri)
    iface_node_uri = host_master.lookupNode(project_settings["iface_node"])

    for action_name in project_settings["sync_actions_to_host"]:
        action_topics_hp = []
        action_topics_cp = []
        try:
            action_node = client_master.lookupNode(project_settings["action_publisher"][action_name])
        except:
            rospy.logerr("client node %s not online", project_settings["action_publisher"][action_name])
            continue
        action_topics_hp.append(ns + action_name + "/cancel")
        action_topics_hp.append(ns + action_name + "/goal")
        action_topics_cp.append(ns + action_name + "/feedback")
        action_topics_cp.append(ns + action_name + "/result")
        action_topics_cp.append(ns + action_name + "/status")
        for action_topic in action_topics_cp:
            action_topic_type = []
            for topic, topic_type in client_master.getTopicTypes():
                if topic == action_topic:
                    action_topic_type = topic_type
            if not action_topic_type:
                rospy.logerr("topic type for topic %s not found", action_topic)
                return False
            host_master.registerPublisher(action_topic, action_topic_type, action_node)
            client_master.registerSubscriber(action_topic, action_topic_type, iface_node_uri)
            # rospy.loginfo("registered client publisher %s with type %s and node uri %s", action_topic, action_topic_type, action_node)
        for action_topic in action_topics_hp:
            action_topic_type = []
            for topic, topic_type in client_master.getTopicTypes():
                if topic == action_topic:
                    action_topic_type = topic_type
            if not action_topic_type:
                rospy.logerr("topic type for topic %s not found", action_topic)
                return False
            host_master.registerSubscriber(action_topic, action_topic_type, action_node)
            client_master.registerPublisher(action_topic, action_topic_type, iface_node_uri)
            # rospy.loginfo("registered host publisher %s with type %s and node uri %s", action_topic, action_topic_type, iface_node_uri)
    return True

def sync_to_client_master(iface_container_ip):
    client_master_uri = "http://" + iface_container_ip + ":11311"
    anon_name_client = rosgraph.names.anonymous_name('master_sync')
    client_master = rosgraph.Master(anon_name_client, master_uri=client_master_uri)
    host_master_uri = "http://" + host_ip + ":11311"
    anon_name_host = rosgraph.names.anonymous_name('master_sync')
    host_master = rosgraph.Master(anon_name_host, master_uri=host_master_uri)

    for srv_name in project_settings["sync_services_to_client"]:
        srv_full_name = ns + srv_name
        srv_uri = host_master.lookupService(srv_full_name)
        if not srv_uri:
            rospy.logfatal(srv_name + " service not found")
            sys.exit(1)
        else:
            fake_api = 'http://%s:0' % host_ip
            rospy.loginfo("trying to sync srv %s with uri %s and fake api %s to client master %s",
                          srv_name, srv_uri, fake_api, client_master_uri)
            client_master.registerService(srv_name, srv_uri, fake_api)

def on_shutdown_call():
    rospy.loginfo("shutdown call received! Trying to shutdown environment containers")
    for cont in environment_containers:
        finish_container(cont)

    docker_network.remove()

def get_host_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    host_ip = s.getsockname()[0]
    s.close
    return host_ip

if __name__ == '__main__':
    rospy.init_node('job_worker')

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

    host_ip = get_host_ip()
    cleanup_host_master()

    dClient = docker.from_env()
    ic_name = "ic_" + ns.replace("/", "")
    net_name = "bridge_" + ns.replace("/", "")
    docker_network = dClient.networks.create(net_name)
    iface_container = setup_environment_container(dClient)

    job_env = actionlib.SimpleActionClient('job_env', JobEnvAction)
    job_idle = actionlib.SimpleActionClient('job_idle', JobEnvAction)

    rospy.loginfo("ready to process jobs")
    while not rospy.is_shutdown():
        job_loop(jobs_collection, dClient, ns)
