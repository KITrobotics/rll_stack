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
import tornado.gen
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
        elif operation == "data_urls":
            self._handle_data_urls_req()
        else:
            raise tornado.web.HTTPError(400)

    @tornado.web.asynchronous
    def post(self):
        operation = self.get_argument("op")

        if operation == "submit_git" or operation == "submit_tar":
            self._handle_submit(operation)
        else:
            raise tornado.web.HTTPError(400)

    def _handle_job_status(self):
        job_id = self.get_argument("job")

        try:
            job_obj_id = ObjectId(job_id)
        except:
            # TODO: provide error message
            raise tornado.web.HTTPError(400)

        rospy.loginfo("got status request for job id '%s'", job_id)

        self.jobs_collection.find_one({"_id": ObjectId(job_id)}, callback=self._db_job_status_cb)

    def _handle_data_urls_req(self):
        job_id = self.get_argument("job")

        rospy.loginfo("got data request for job id '%s'", job_id)

        try:
            job_obj_id = ObjectId(job_id)
        except:
            raise tornado.web.HTTPError(400)

        self.jobs_collection.find_one({"_id": ObjectId(job_id)}, callback=self._db_job_data_urls_cb)

    def _handle_submit(self, operation):
        username = self.get_argument("username")
        secret = self.get_argument("secret")
        project = self.get_argument("project")

        if operation == "submit_git":
            git_url = self.get_argument("git_url")
            git_tag = self.get_argument("git_tag")
        elif operation == "submit_tar":
            tar_url = self.get_argument("tar_url")

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

        if operation == "submit_git":
            rospy.loginfo("got a new submission with username '%s', Git repo URL '%s' and tag '%s' for project '%s'",
                          username, git_url, git_tag, project)
        elif operation == "submit_tar":
            rospy.loginfo("got a new submission with username '%s', Tar archive URL '%s' for project '%s'",
                          username, tar_url, project)

        self._handle_new_submission(operation, username, project)

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

    @tornado.gen.coroutine
    def _db_job_status_cb(self, job, error):
        rospy.loginfo("job status db callback with job %s and error %s", job, error)
        if error or not job:
            result = {"status": "error", "error": "No job with this ID"}
            self.write(json_encode(result))
            self.finish()
            return

        if job["status"] == "submitted" or job["status"] == "waiting for real":
            future = self.jobs_collection.find({"status": job["status"], "created": {"$lte": job["created"]}}).count()
            try:
                position = yield future
                result = self._build_status_resp(job)
                result["position"] = position
            except Exception, e:
                raise tornado.web.HTTPError(500, e)
        else:
            result = self._build_status_resp(job)
            result["position"] = -1

        self.write(json_encode(result))
        self.finish()

    def _db_job_data_urls_cb(self, job, error):
        rospy.loginfo("job data db callback with job %s and error %s", job, error)
        if error:
            result = {"status": "error", "error": "No job with this ID"}
        else:
            result = {"status": "success", "build_log_url": self.rll_settings["logs_base_url"] + "/" + str(job["_id"]) + "/build.log",
                      "video_url": self.rll_settings["logs_base_url"] + "/" + str(job["_id"]) + "/video.mp4",
                      "sim_run_client_log_url": self.rll_settings["logs_base_url"] + "/" + str(job["_id"]) + "/sim-client.log",
                      "sim_run_iface_log_url": self.rll_settings["logs_base_url"] + "/" + str(job["_id"]) + "/sim-iface.log",
                      "real_run_client_log_url": self.rll_settings["logs_base_url"] + "/" + str(job["_id"]) + "/real-client.log",
                      "real_run_iface_log_url": self.rll_settings["logs_base_url"] + "/" + str(job["_id"]) + "/real-iface.log"}

        self.write(json_encode(result))
        self.finish()

    def _add_job_position(self,result,error):
        rospy.loginfo("_add_job_position callback with resutl %s and error %s", result, error)

        if error:
            raise tornado.web.HTTPError(500, error)
        else:
            rospy.loginfo(result)
            result = {"status": "success", "job_id": str(result.inserted_id),"position":str(result)}
            self.write(json_encode(result))
            self.finish()

    def _build_status_resp(self, job):
        job_status = job["status"]
        result = {"status": "success", "job_status": job_status}

        if job_status == "finished" or job_status == "waiting for real":
            result["job_result"] = job["job_result"]
            if "job_data" in job:
                result["job_data"] = job["job_data"]
            else:
                result["job_data"] = None
        elif job_status == "running real":
            result["cam_url"] = self._ns_to_cam(job["ns"])

        return result

    def _db_job_insert_cb(self, result, error):
        rospy.loginfo("job insert db callback with result %s and error %s", result, error)

        if error:
            rospy.loginfo(error)
            raise tornado.web.HTTPError(500, error)
        else:
            result = {"status": "success", "job_id": str(result.inserted_id)}
            self.write(json_encode(result))
            self.finish()

    def _ns_to_cam(self, ns):
        cam_mapping = {"/iiwa_1/": self.rll_settings["cams_base_url"] + "/stream1/mjpg/video.mjpg",
                       "/iiwa_2/": self.rll_settings["cams_base_url"] + "/stream2/mjpg/video.mjpg"}
        return cam_mapping.get(ns, "unknown")

    @tornado.gen.coroutine
    def _update_submission(self, result, error):

        if error:
            rospy.loginfo(error)
            raise tornado.web.HTTPError(500, error)
        else:
            rospy.loginfo("Update submission performed:\n Matched: %d, Modified: %d\n Raw: %s"%(result.matched_count, result.modified_count,result.raw_result))
            # see: http://api.mongodb.com/python/3.6.1/api/pymongo/results.html#pymongo.results.UpdateResult
            if result.matched_count and not result.modified_count:
                # job was found, but nothing modified
                result = {"status": "error", "error": "Nothing was modified"}
            elif result.matched_count and result.modified_count:
                # job was found and updated
                future = self.jobs_collection.find_one({"status":"submitted",
                                                        "username": self.get_argument("username"),
                                                        "project": self.get_argument("project")})
                res = yield future
                # TODO: use new flag to make clear that it's modified
                result = {"status": "success", "job_id": str(res["_id"])}
            else:
                 raise tornado.web.HTTPError(500, "Unexpected Error")

            self.write(json_encode(result))
            self.finish()


    @tornado.gen.coroutine
    def _handle_new_submission(self, operation, username, project):
        try:
            # check if max queue size is reached
            future = self.jobs_collection.find({"status":"submitted"}).count()
            submission_count = yield future
            max_sub_queue = rll_settings["sub_queue_limit"]
            if submission_count > max_sub_queue:
                response = {"status": "error", "error": "Queue at max size"}
                self.write(json_encode(response))
                self.finish()
                return

            rospy.loginfo("Length of submission queue is %d. Max is %d" % (submission_count, max_sub_queue))

            # then check if user has running job
            future_run_job = self.jobs_collection.find_one({"$and": [{"status": {"$ne": "finished"}},
                                                                     {"status": {"$ne": "submitted"}}],
                                                            "project": project,
                                                            "username": username})
            job = yield future_run_job
            if not job == None:
                rospy.loginfo("Found running job for user.")
                response = {"status": "error", "error": "User has a running job in the queue"}
                self.write(json_encode(response))
                self.finish()
                return
            rospy.loginfo("Found no running job for user.")

            # check for existing submission
            future_existing_sub = self.jobs_collection.find_one({"status": "submitted",
                                                                 "username": username,
                                                                 "project": project})
            result = yield future_existing_sub
            if result:
                # submission found: Update it
                if operation == "submit_git":
                    self.jobs_collection.update_one({"username": username,
                                                     "project": project,
                                                     "status": "submitted"},
                                                    update = {"$set": {"git_tag": self.get_argument("git_tag")}},
                                                    callback=self._update_submission, upsert=False)
                elif operation == "submit_tar":
                    self.jobs_collection.update_one({"username": username,
                                                     "project": project,
                                                     "status": "submitted"},
                                                    update = {"$set": {"tar_url": self.get_argument("tar_url")}},
                                                    callback=self._update_submission, upsert=False)
            else:
                # no submission for user and project found, create new one
                if operation == "submit_git":
                    git_url = self.get_argument("git_url")
                    git_tag = self.get_argument("git_tag")
                    if not self._valid_git_url(git_url, git_tag):
                        return
                    new_job = {"username": username, "project": project, "submit_type": "git",
                               "git_url": git_url, "git_tag": git_tag,
                               "status": "submitted", "created": datetime.datetime.now()}
                elif operation == "submit_tar":
                    new_job = {"username": username, "project": project, "submit_type": "tar",
                               "tar_url": self.get_argument("tar_url"),
                               "status": "submitted", "created": datetime.datetime.now()}
                self.jobs_collection.insert_one(new_job, callback=self._db_job_insert_cb)

        except Exception, e:
                 raise tornado.web.HTTPError(500, e)


if __name__ == '__main__':
    rospy.init_node('rll_server')
    signal.signal(signal.SIGINT, signal_handler)

    # settings
    rospack = rospkg.RosPack()
    config_path = rospack.get_path('rll_common') + "/config/rll.yaml"
    with open(config_path, 'r') as doc:
        rll_settings = yaml.load(doc)

    production_mode = rospy.get_param("~production")
    if production_mode:
        rospy.logwarn("started server in production mode. Press enter if you really want to continue!")
        raw_input()
        db_user = rll_settings["production_db_user"]
        db_pw = rll_settings["production_db_pw"]
        db_host = rll_settings["production_db_host"]
        db_name = rll_settings["production_db_name"]
        db_client_string = "mongodb://" + db_user + ":" + db_pw + "@" + db_host + "/" + db_name
        db_client = motor.motor_tornado.MotorClient(db_client_string)
    else:
        db_name = rll_settings["test_db_name"]
        db_client = motor.motor_tornado.MotorClient()

    db = db_client[db_name]
    rospy.loginfo("using database %s", db_name)

    app = tornado.web.Application([(r"/jobs", JobsHandler, dict(db=db, rll_settings=rll_settings))],
                                  debug = True)
    app.listen(8888)
    tornado.ioloop.PeriodicCallback(try_exit, 100).start()
    tornado.ioloop.IOLoop.current().start()
