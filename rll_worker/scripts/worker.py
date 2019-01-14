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
from axis_camera.msg import VideostreamAction, VideostreamGoal

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
from urllib2 import urlopen, URLError
from tarfile import is_tarfile

REMOVE_CONTAINERS_ON_SHUTDOWN = True

environment_containers = []
topic_sync_names = {}

def job_loop():
    global REMOVE_CONTAINERS_ON_SHUTDOWN

    job = find_new_job()
    if job == None:
        rospy.sleep(0.1)
        return

    job_id = job["_id"]
    submit_type = job["submit_type"]
    rospy.loginfo("got job with id '%s' with submit type '%s' and create date %s", job_id, submit_type,
                  str(job["created"]))

    if manual_mode:
        rospy.loginfo("Got new job, do you want to continue processing? [Y/n]")
        choice = raw_input().lower()
        if choice == "n":
            reset_job(job_id, "manual job processing aborted")
            rospy.loginfo("manual job processing aborted, exiting...")
            sys.exit()

    success, client_container = start_job(job, job_id, submit_type)
    if not success:
        rospy.loginfo("starting job failed, returning")
        return

    available = job_env.wait_for_server(rospy.Duration.from_sec(2.0))
    if not available:
        reset_job(job_id, "job env not available")
        rospy.logfatal("Job env action server is not available, please investigate!")
        sys.exit(1)

    if run_mode == "real":
        start_recording(job_id)
    job_env_goal = JobEnvGoal()
    job_env.send_goal(job_env_goal)
    rospy.loginfo("running job env")

    # TODO: handle this better
    rospy.sleep(0.5)
    sync_to_host_master()

    # TODO: have a timeout here and maybe request feedback
    job_env.wait_for_result()
    resp = job_env.get_result()
    result_string = job_result_codes_to_string(resp.job.status)
    rospy.loginfo("successfully run job environment with job status %s\n", result_string)

    if run_mode == "real":
        stop_recording()
    get_client_log(job_id, client_container)
    get_iface_log(job_id, iface_container)
    unregister_client()
    finish_container(client_container)

    if result_string == "sim success":
        finished_status = "waiting for real"
    else:
        finished_status = "finished"

    job_extra_data = {}
    if resp.job_data:
        rospy.loginfo("extra job data:")
        for element in resp.job_data:
            rospy.loginfo("%s: %f", element.description, element.value)
            job_extra_data[element.description] = element.value
    else:
        job_extra_data = None

    jobs_collection.find_one_and_update({"_id": job_id},
                                        {"$set": {"status": finished_status,
                                                  "job_end": datetime.datetime.now(),
                                                  "job_result": result_string,
                                                  "job_data": job_extra_data}})

    if resp.job.status == JobStatus.INTERNAL_ERROR:
        REMOVE_CONTAINERS_ON_SHUTDOWN = False
        rospy.logfatal("Internal error happened when running job environment, please investigate (job ID %s)!", job_id)
        sys.exit(1)

    # reset robot and environment
    job_idle_goal = JobEnvGoal()
    job_idle.send_goal(job_idle_goal)
    job_idle.wait_for_result()
    resp = job_idle.get_result()
    if resp.job.status == JobStatus.INTERNAL_ERROR:
        REMOVE_CONTAINERS_ON_SHUTDOWN = False
        rospy.logfatal("Internal error happened when running idle service, please investigate (job ID %s)!", job_id)
        sys.exit(1)

    rospy.loginfo("finished job with id '%s' in namespace '%s'\n", job_id, ns)

def find_new_job():
    if rospy.has_param("~shutdown"):
        shutdown = rospy.get_param("~shutdown")
        if shutdown:
            rospy.loginfo("requested shutdown of worker")
            sys.exit()

    if run_mode == "real" and sim_check == True:
        search_status = "waiting for real"
    else:
        search_status = "submitted"

    # TODO: use indexing
    return jobs_collection.find_one_and_update({"project": project_name, "status": search_status},
                                               {"$set": {"status": "processing " + run_mode + " started",
                                                         "ns": ns,
                                                         "job_start": datetime.datetime.now()}},
                                               sort=[("created", pymongo.ASCENDING)])

def start_job(job, job_id, submit_type):
    try:
        #client container
        # terminal command to remove all containers from image "rll-base":
        # docker rm $(docker stop $(docker ps -a -q --filter ancestor=rll-base --format="{{.ID}}"))
        client_container = create_client_container(cc_name, "rll-base")
    except:
        rospy.logfatal("failed to create container:\n%s", traceback.format_exc())
        reset_job(job_id, "creating container failed")
        sys.exit(1)

    jobs_collection.find_one_and_update({"_id": job_id}, {"$set": {"status": "downloading code"}})
    # TODO: handle clone failures
    success = get_exp_code(job, job_id, submit_type, client_container)
    if not success:
        finish_container(client_container)
        return False, client_container

    rospy.loginfo("building project")
    jobs_collection.find_one_and_update({"_id": job_id}, {"$set": {"status": "building"}})
    try:
        cmd_result = client_container.exec_run("catkin build --no-status " + job["project"], stdin=True, tty=True)
    except:
        rospy.logerr("Docker API error when building:\n%s", traceback.format_exc())
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "finished",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "building project failed"}})
        finish_container(client_container)
        return False, client_container

    write_build_log(job_id, cmd_result[1])
    if cmd_result[0] != 0:
        rospy.logerr("building project failed")

        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "finished",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "building project failed"}})
        finish_container(client_container)
        return False, client_container

    jobs_collection.find_one_and_update({"_id": job_id}, {"$set": {"status": "running " + run_mode}})
    launch_file = project_settings["launch_client"]
    launch_cmd = "roslaunch --disable-title " + job["project"] + " " + launch_file + " robot:=" + ns
    client_container.reload()
    client_container_ip = client_container.attrs["NetworkSettings"]["Networks"][net_name]["IPAddress"]

    rospy.loginfo("running launch command %s", launch_cmd)
    # PYTHONUNBUFFERED needs to be disabled to ensure that data is piped before the app finishes and when we are detached
    # stdbuf -o0 does the same for C/C++ apps
    try:
        client_container.exec_run("bash -c \"source devel/setup.bash && stdbuf -o0 " + launch_cmd + " &> /tmp/client.log\"",
                                  tty=True, detach=True, environment={"PYTHONUNBUFFERED": "0",
                                                                      "ROS_IP": client_container_ip,
                                                                      "ROS_MASTER_URI": "http://" + ic_name + ":11311"})
    except:
        rospy.logerr("Docker API error when launching project:\n%s", traceback.format_exc())
        # TODO: rather reset job? May be entirely our fault and not the client ones
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "finished",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "launching project failed"}})
        finish_container(client_container)
        return False, client_container

    # giving the client a little time to start
    # TODO: handle this better with syncing between masters
    rospy.sleep(2)

    return True, client_container


def get_exp_code(job, job_id, submit_type, container):
    project = job["project"]
    ws_repo_path = path.join("/home/rll_user/ws/src/", project)

    if submit_type == "git":
        repo_clone_dir = path.join(rll_settings["git_archive_dir"], job["username"], project)
        code_archive_file = repo_clone_dir + ".tar"

        # TODO: --single-branch for not cloning all branches
        repo = Repo.clone_from(job["git_url"], repo_clone_dir, branch=job["git_tag"])
        if repo.__class__ is not Repo:
            rospy.logerr("cloning project repo failed")
            jobs_collection.find_one_and_update({"_id": job_id},
                                                {"$set": {"status": "finished",
                                                          "job_end": datetime.datetime.now(),
                                                          "job_result": "fetching project code failed"}})
            return False
        with open(code_archive_file, 'wb') as fwp:
            repo.archive(fwp)

    elif submit_type == "tar":
        # TODO: check into Git repo for deduplication and compression
        code_archive_dir = path.join(rll_settings["tar_archive_dir"],str(job_id))
        code_archive_file = path.join(code_archive_dir, "code.tar")

        # if this is the case, then the code is already there
        if not (run_mode == "real" and sim_check == True):
            try:
                response = urlopen(job["tar_url"])
            except URLError as e:
                if hasattr(e, "reason"):
                    rospy.logerr("failed to reach server for tar archive download: %s", e.reason)
                elif hasattr(e, "code"):
                    rospy.logerr("server error when downloading tar archive: %s", e.code)

                jobs_collection.find_one_and_update({"_id": job_id},
                                                    {"$set": {"status": "finished",
                                                              "job_end": datetime.datetime.now(),
                                                              "job_result": "fetching project code failed"}})
                return False

            if not path.exists(code_archive_dir):
                makedirs(code_archive_dir)
            with open(code_archive_file, 'wb') as fwp:
                try:
                    fwp.write(response.read())
                except:
                    rospy.logerr("read error when downloading tar archive")
                    jobs_collection.find_one_and_update({"_id": job_id},
                                                        {"$set": {"status": "finished",
                                                                  "job_end": datetime.datetime.now(),
                                                                  "job_result": "fetching project code failed"}})
                    return False

    if not is_tarfile(code_archive_file):
        jobs_collection.find_one_and_update({"_id": job_id},
                                            {"$set": {"status": "finished",
                                                      "job_end": datetime.datetime.now(),
                                                      "job_result": "fetching project code failed"}})
        rospy.logerr("code archive is not a tar file")
        return False

    with open(code_archive_file, 'rb') as frp:
        container.start()
        # TODO: figure out if this is a bug in docker-py or Docker
        # wait a moment to make sure that the container is started
        rospy.sleep(1)
        container.exec_run("mkdir " + ws_repo_path)
        rospy.sleep(0.1)
        try:
            container.put_archive(ws_repo_path, frp)
        except:
            rospy.logfatal("failed to upload project to container:\n%s", traceback.format_exc())
            reset_job(job_id, "failed to upload project to container")
            sys.exit(1)

        rospy.loginfo("uploaded project to container")

    if submit_type == "git":
        remove(code_archive_file)
        # TODO: create a proper archive and don't delete the repo
        rmtree(repo_clone_dir, ignore_errors=True)

    return True


def create_client_container(container_name, image_name):

    return dClient.containers.create(image_name, network=net_name,
                                     publish_all_ports=False,
                                     detach=True, tty=True,
                                     nano_cpus=int(1e9), # limit to one CPU
                                     mem_limit="1g", # limit RAM to 1GB
                                     memswap_limit="1g", # limit RAM+SWAP to 1GB
                                     storage_opt={"size": "10G"}, # limit disk space to 10GB
                                     name = container_name)

def reset_job(job_id, reason):
    if run_mode == "real":
        reset_status = "waiting for real"
    else:
        reset_status = "submitted"
    # TODO: be more transparent about this by using a dedicated status code
    jobs_collection.find_one_and_update({"_id": job_id},
                                        {"$set": {"status": reset_status,
                                                  "job_end": datetime.datetime.now(),
                                                  "job_result": reason}})

def finish_container(container):
    container.stop()
    container.remove()

def get_client_log(job_id, container):
    log_folder = path.join(job_data_save_dir, str(job_id))
    log_file = path.join(log_folder, run_mode + "-client.log")
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
    global REMOVE_CONTAINERS_ON_SHUTDOWN
    log_folder = path.join(job_data_save_dir, str(job_id))
    log_file = path.join(log_folder, run_mode + "-iface.log")

    container.exec_run("cp /tmp/iface.log /tmp/iface.log.bak", user="root")
    # TODO: test again if we really need to sleep here
    rospy.sleep(0.1)
    container.exec_run("chown root: /tmp/iface.log.bak", user="root")
    rospy.sleep(0.1)
    try:
        strm, status = container.get_archive("/tmp/iface.log.bak")
        with open(log_file, 'w') as outfile:
            for d in strm:
                outfile.write(d)
    except:
        rospy.logfatal("failed to retrieve iface log from container:\n%s", traceback.format_exc())
        reset_job(job_id, "failed to retrieve log from container")
        REMOVE_CONTAINERS_ON_SHUTDOWN = False
        sys.exit(1)

    container.exec_run("rm /tmp/iface.log.bak", user="root")
    container.exec_run("truncate -s 0 /tmp/iface.log")

def write_build_log(job_id, log_str):
    rospy.loginfo("build logs:\n\n%s", log_str)

    # if it should be saved with ANSI code removal
    # https://stackoverflow.com/questions/14693701/how-can-i-remove-the-ansi-escape-sequences-from-a-string-in-python
    # ansi_escape = re.compile(r'\x1B\[[0-?]*[ -/]*[@-~]')
    # log_str = ansi_escape.sub('', log_str)

    log_folder = path.join(job_data_save_dir, str(job_id))
    if not path.exists(log_folder):
        makedirs(log_folder)
    log_file = path.join(log_folder, "build.log")
    log_ptr = open(log_file, "w")
    log_ptr.write(log_str)
    log_ptr.close()
    rospy.loginfo("wrote build logs to disk")


def job_result_codes_to_string(status):
    job_codes = {JobStatus.SUCCESS: run_mode + " success", JobStatus.FAILURE: run_mode + " failure",
                 JobStatus.INTERNAL_ERROR: run_mode + " internal error"}
    return job_codes.get(status, "unknown")


def setup_environment_container(dClient):
    if run_mode == "real":
        iface_image = project_settings["iface_docker_image"] + "-real"
    else:
        iface_image = project_settings["iface_docker_image"]
    rospy.loginfo("using docker image '%s' for iface container", iface_image)
    iface_container = dClient.containers.create(iface_image,
                                                network=net_name,
                                                publish_all_ports=True,
                                                detach=True, tty=True,
                                                name = ic_name)
    iface_container.start()
    rospy.loginfo("Started interface container: " + ic_name)
    environment_containers.append(iface_container)

    iface_container.reload()
    iface_container_ip = iface_container.attrs["NetworkSettings"]["Networks"][net_name]["IPAddress"]
    rospy.loginfo("iface container ip: " + iface_container_ip)

    # start ROS master
    iface_container.exec_run("bash -c \"source devel/setup.bash && roscore\"", tty=True, detach=True, environment={"ROS_HOSTNAME": ic_name})

    launch_file = project_settings["launch_iface"]
    launch_cmd = "roslaunch --disable-title " + project_settings["iface_project"] + " " + launch_file + " robot:=" + ns + " headless:=true"
    rospy.loginfo("running iface launch command %s", launch_cmd)
    ic_result = iface_container.exec_run("bash -c \"source devel/setup.bash && stdbuf -o0 " + launch_cmd + " &> /tmp/iface.log\"",
                                         tty=True, detach=True, environment={"PYTHONUNBUFFERED": "0", "ROS_IP": iface_container_ip,
                                                                             "ROS_MASTER_URI": "http://" + host_ip + ":11311"})

    rospy.loginfo("waiting for the client master and the iface to come up")
    if run_mode == "sim":
        rospy.sleep(10)
    else:
        rospy.sleep(20)
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

    if rospy.has_param("~shutdown"):
        rospy.delete_param("~shutdown")

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
    global REMOVE_CONTAINERS_ON_SHUTDOWN
    client_master_uri = "http://" + iface_container_ip + ":11311"
    anon_name_client = rosgraph.names.anonymous_name('master_sync')
    client_master = rosgraph.Master(anon_name_client, master_uri=client_master_uri)
    host_master_uri = "http://" + host_ip + ":11311"
    anon_name_host = rosgraph.names.anonymous_name('master_sync')
    host_master = rosgraph.Master(anon_name_host, master_uri=host_master_uri)

    for srv_name in project_settings["sync_services_to_client"]:
        srv_full_name = ns + srv_name
        try:
            srv_uri = host_master.lookupService(srv_full_name)
        except:
            rospy.logfatal(srv_full_name + " service not found")
            REMOVE_CONTAINERS_ON_SHUTDOWN = False
            sys.exit(1)
        else:
            fake_api = 'http://%s:0' % host_ip
            rospy.loginfo("trying to sync srv %s with uri %s and fake api %s to client master %s",
                          srv_name, srv_uri, fake_api, client_master_uri)
            client_master.registerService(srv_name, srv_uri, fake_api)

def check_unfinished_jobs():
    job = jobs_collection.find_one({"project": project_name, "ns": ns,
                                    "$and": [{"status": {"$ne": "finished"}},
                                             {"status": {"$ne": "submitted"}},
                                             {"status": {"$ne": "waiting for real"}}]})
    if job:
        rospy.logfatal("found a job that is marked running in this namespace")
        rospy.logfatal("job data: id '%s', status '%s'", job["_id"], job["status"])
        sys.exit(1)

def start_recording(job_id):
    available = video_client.wait_for_server(rospy.Duration.from_sec(2.0))
    if not available:
        reset_job(job_id, "video server not available")
        rospy.logfatal("video server not available")
        sys.exit(1)

    goal = VideostreamGoal()
    goal.base_path = job_data_save_dir
    goal.video_folder = str(job_id)
    video_client.send_goal(goal)

def stop_recording():
    video_client.cancel_goal()

def on_shutdown_call():
    rospy.loginfo("shutdown call received! Trying to shutdown containers...")
    if REMOVE_CONTAINERS_ON_SHUTDOWN:
        try:
            cc = dClient.containers.get(cc_name)
            finish_container(cc)
        except:
            pass
        for cont in environment_containers:
            finish_container(cont)
        docker_network.remove()
    else:
        rospy.loginfo("not shutting down any containers")

def get_host_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    host_ip = s.getsockname()[0]
    s.close
    return host_ip

def docker_cleanup():
    rospy.loginfo("pruning Docker containers and networks...")
    dClient.containers.prune()
    dClient.networks.prune()

if __name__ == '__main__':
    rospy.init_node('job_worker')

    rospy.on_shutdown(on_shutdown_call)

    ns = rospy.get_namespace()

    # settings
    rospack = rospkg.RosPack()
    config_path = rospack.get_path('rll_common') + "/config/rll.yaml"
    with open(config_path, 'r') as doc:
        rll_settings = yaml.load(doc)

    project_name = rospy.get_param("~project")
    run_mode = rospy.get_param("~mode")
    sim_check = rospy.get_param("~sim_check")
    production_mode = rospy.get_param("~production")
    manual_mode = rospy.get_param("~manual")
    rospy.loginfo("processing project %s in %s mode", project_name, run_mode)
    if manual_mode:
        rospy.loginfo("manual processing of jobs enabled")
    try:
        project_settings = rll_settings["project_settings"][project_name]
    except:
        rospy.logfatal("failed to retrieve project settings")
        sys.exit(1)

    if production_mode:
        rospy.logwarn("started worker in production mode. Type 'yes' if you really want to continue!")
        typed = raw_input().lower()
        if typed != "yes":
            rospy.loginfo("aborting...")
            sys.exit(0)
        db_user = rll_settings["production_db_user"]
        db_pw = rll_settings["production_db_pw"]
        db_host = rll_settings["production_db_host"]
        db_name = rll_settings["production_db_name"]
        db_client_string = "mongodb://" + db_user + ":" + db_pw + "@" + db_host + "/" + db_name
        db_client = pymongo.MongoClient(db_client_string)
        job_data_save_dir = rll_settings["production_job_data_save_dir"]
    else:
        db_name = rll_settings["test_db_name"]
        db_client = pymongo.MongoClient()
        job_data_save_dir = rll_settings["test_job_data_save_dir"]
    rospy.loginfo("using database %s", db_name)
    jobs_collection = db_client[db_name].jobs

    host_ip = get_host_ip()
    cleanup_host_master()

    dClient = docker.from_env()
    cc_name = "cc_" + ns.replace("/", "")
    ic_name = "ic_" + ns.replace("/", "")
    net_name = "bridge_" + ns.replace("/", "")
    docker_cleanup()
    docker_network = dClient.networks.create(net_name, internal=True)
    check_unfinished_jobs()
    iface_container = setup_environment_container(dClient)

    job_env = actionlib.SimpleActionClient('job_env', JobEnvAction)
    job_idle = actionlib.SimpleActionClient('job_idle', JobEnvAction)
    if run_mode == "real":
        video_client = actionlib.SimpleActionClient('videostreamserver', VideostreamAction)
        available = video_client.wait_for_server(rospy.Duration.from_sec(2.0))
        if not available:
            rospy.logfatal("video server not available")
            sys.exit(1)

    rospy.loginfo("ready to process jobs")
    while not rospy.is_shutdown():
        job_loop()
