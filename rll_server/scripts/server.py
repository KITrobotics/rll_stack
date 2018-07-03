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

import tornado.ioloop
import tornado.web
from tornado.escape import json_encode
import motor.motor_tornado
from bson.objectid import ObjectId

import git
import signal
import datetime
import re

app_closing = False

def signal_handler(signum, frame):
    global app_closing
    app_closing = True

def try_exit():
    global app_closing
    if app_closing:
        tornado.ioloop.IOLoop.instance().stop()

class JobsHandler(tornado.web.RequestHandler):
    def initialize(self, db, rll_settings):
        self.jobs_collection = db.jobs
        self.rll_settings = rll_settings

    def set_default_headers(self):
        # TODO: maybe do this more fine-grained?
        self.set_header("Access-Control-Allow-Origin", "*")

    @tornado.web.asynchronous
    def get(self):
        operation = self.get_argument("op")

        if operation == "status":
            self._handle_job_status()
        elif operation == "logs":
            self._handle_logs_req()
        else:
            raise tornado.web.HTTPError(400)

    @tornado.web.asynchronous
    def post(self):
        operation = self.get_argument("op")

        if operation == "submit":
            self._handle_submit()
        else:
            raise tornado.web.HTTPError(400)

    def _handle_job_status(self):
        job_id = self.get_argument("job")

        rospy.loginfo("got status request for job id '%s'", job_id)

        try:
            job_obj_id = ObjectId(job_id)
        except:
            raise tornado.web.HTTPError(400)

        self.jobs_collection.find_one({"_id": ObjectId(job_id)}, callback=self._db_job_status_cb)

    def _handle_logs_req(self):
        job_id = self.get_argument("job")

        rospy.loginfo("got log request for job id '%s'", job_id)

        try:
            job_obj_id = ObjectId(job_id)
        except:
            raise tornado.web.HTTPError(400)

        self.jobs_collection.find_one({"_id": ObjectId(job_id), "status": "finished"}, callback=self._db_job_logs_cb)

    def _handle_submit(self):
        username = self.get_argument("username")
        secret = self.get_argument("secret")
        project = self.get_argument("project")
        git_url = self.get_argument("git_url")
        git_tag = self.get_argument("git_tag")

        # TODO: better retrieve this from db
        if not secret == self.rll_settings["secret"]:
            rospy.logwarn("authentication error")
            raise tornado.web.HTTPError(401)

        if not project in self.rll_settings["projects"]:
            rospy.logwarn("unknown project name in submission")
            raise tornado.web.HTTPError(400)

        if not re.match("^[A-Za-z0-9_-]+$", username):
            rospy.logwarn("username invalid")
            response = {"status": "error", "error": "Username invalid"}
            self.write(json_encode(response))
            self.finish()
            return

        rospy.loginfo("got a new submission with username '%s', Git repo URL '%s' and tag '%s' for project '%s'",
                      username, git_url, git_tag, project)

        # check if there is already a job in the queue for this user
        # TODO: try to solve this with indexing
        self.jobs_collection.find_one({"username": username, "project": project,
                                       "status": {"$in": ["submitted", "running"]}},
                                 callback=self._db_open_job_cb)

    def _db_open_job_cb(self, job, error):
        rospy.loginfo("db open job callback with job %s and error %s", job, error)

        if error:
            raise tornado.web.HTTPError(500, error)
        elif not job == None:
            response = {"status": "error", "error": "User has a job in the queue"}
            self.write(json_encode(response))
            self.finish()
            return

        git_url = self.get_argument("git_url")
        git_tag = self.get_argument("git_tag")
        if not self._valid_git_url(git_url, git_tag):
            return

        username = self.get_argument("username")
        project = self.get_argument("project")
        job = {"username": username, "project": project, "git_url": git_url, "git_tag": git_tag,
               "status": "submitted", "created": datetime.datetime.now()}
        self.jobs_collection.insert_one(job, callback=self._db_job_insert_cb)

    def _valid_git_url(self, git_url, git_tag):
        g = git.cmd.Git()
        try:
            tag = g.ls_remote(git_url, "refs/tags/" + git_tag)
        except:
            response = {"status": "error", "error": "Git URL invalid"}
            self.write(json_encode(response))
            self.finish()
            return False

        if tag == "":
            response = {"status": "error", "error": "Git tag not found"}
            self.write(json_encode(response))
            self.finish()
            return False

        return True

    def _db_job_status_cb(self, job, error):
        rospy.loginfo("job status db callback with job %s and error %s", job, error)

        if error:
            result = {"status": "error", "error": "No job with this ID"}
        else:
            result = self._build_status_resp(job)

        self.write(json_encode(result))
        self.finish()

    def _db_job_logs_cb(self, job, error):
        rospy.loginfo("job log db callback with job %s and error %s", job, error)
        if error:
            result = {"status": "error", "error": "No finished job with this ID"}
        else:
            # TODO: also return build log
            result = {"status": "success", "log_url": self.rll_settings["logs_base_url"] + "/" + str(job["_id"]) + "/launch.log"}

        self.write(json_encode(result))
        self.finish()

    def _build_status_resp(self, job):
        job_status = job["status"]
        result = {"status": "success", "job_status": job_status}

        if job_status == "finished":
            result["job_result"] = job["job_result"]
        elif job_status == "running":
            result["cam_url"] = self._ns_to_cam(job["ns"])

        return result

    def _db_job_insert_cb(self, result, error):
        rospy.loginfo("job insert db callback with result %s and error %s", result, error)

        if error:
            raise tornado.web.HTTPError(500, error)
        else:
            result = {"status": "success", "job_id": str(result.inserted_id)}
            self.write(json_encode(result))
            self.finish()

    def _ns_to_cam(self, ns):
        cam_mapping = {"/iiwa_1/": self.rll_settings["cams_base_url"] + "/stream1/mjpg/video.mjpg",
                       "/iiwa_2/": self.rll_settings["cams_base_url"] + "/stream2/mjpg/video.mjpg"}
        return cam_mapping.get(ns, "unknown")

if __name__ == '__main__':
    rospy.init_node('rll_server')
    signal.signal(signal.SIGINT, signal_handler)

    # settings
    rospack = rospkg.RosPack()
    config_path = rospack.get_path('rll_config') + "/config/rll.yaml"
    with open(config_path, 'r') as doc:
        rll_settings = yaml.load(doc)
        db_name = rll_settings["db_name"]
        rospy.loginfo("using database %s", db_name)

    db_client = motor.motor_tornado.MotorClient()
    db = db_client[db_name]

    app = tornado.web.Application([(r"/jobs", JobsHandler, dict(db=db, rll_settings=rll_settings))],
                                  debug = True)
    app.listen(8888)
    tornado.ioloop.PeriodicCallback(try_exit, 100).start()
    tornado.ioloop.IOLoop.current().start()
