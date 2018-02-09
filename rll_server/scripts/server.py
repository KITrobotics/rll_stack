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

import tornado.ioloop
import tornado.web
import motor.motor_tornado
from bson.objectid import ObjectId

import git
import signal
import datetime

app_closing = False

def signal_handler(signum, frame):
    global app_closing
    app_closing = True

def try_exit():
    global app_closing
    if app_closing:
        tornado.ioloop.IOLoop.instance().stop()

class JobsHandler(tornado.web.RequestHandler):
    @tornado.web.asynchronous
    def get(self):
        operation = self.get_argument("op")

        if operation == "status":
            self._handle_job_status()
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
        jobs_collection = self.settings['db'].jobs

        rospy.loginfo("got status request for job id '%s'", job_id)

        try:
            job_obj_id = ObjectId(job_id)
        except:
            raise tornado.web.HTTPError(400)

        jobs_collection.find_one({"_id": ObjectId(job_id)}, callback=self._db_job_status_cb)

    def _handle_submit(self):
        username = self.get_argument("username")
        git_url = self.get_argument("git_url")
        jobs_collection = self.settings['db'].jobs

        rospy.loginfo("got a new submission with username '%s' and Git repo URL '%s'", username, git_url)

        # check if there is already a job in the queue for this user
        jobs_collection.find_one({"username": username, "status": "submitted"},
                                 callback=self._db_open_job_cb)

    def _db_open_job_cb(self, job, error):
        rospy.loginfo("db open job callback with job %s and error %s", job, error)

        if error:
            raise tornado.web.HTTPError(500, error)
        elif not job == None:
            response = {"status": "error", "error": "User has a job in the queue"}
            self.write(response)
            self.finish()
            return

        git_url = self.get_argument("git_url")
        if not self._valid_git_url(git_url):
            return

        jobs_collection = self.settings['db'].jobs
        username = self.get_argument("username")
        job = {"username": username, "git_url": git_url,
               "status": "submitted", "created": datetime.datetime.now()}
        jobs_collection.insert_one(job, callback=self._db_job_insert_cb)

    def _valid_git_url(self, git_url):
        g = git.cmd.Git()
        try:
            g.ls_remote(git_url)
        except:
            response = {"status": "error", "error": "Git URL invalid"}
            self.write(response)
            self.finish()
            return False
        return True

    def _db_job_status_cb(self, job, error):
        rospy.loginfo("job status db callback with job %s and error %s", job, error)

        if error:
            raise tornado.web.HTTPError(500, error)
        else:
            result = {"status": "success", "job_status": job["status"]}
            self.write(result)
            self.finish()

    def _db_job_insert_cb(self, result, error):
        rospy.loginfo("job insert db callback with result %s and error %s", result, error)

        if error:
            raise tornado.web.HTTPError(500, error)
        else:
            result = {"status": "success", "job_id": str(result.inserted_id)}
            self.write(result)
            self.finish()

if __name__ == '__main__':
    rospy.init_node('rll_server')
    signal.signal(signal.SIGINT, signal_handler)

    db = motor.motor_tornado.MotorClient().rll_test

    app = tornado.web.Application([(r"/jobs", JobsHandler)], db=db,
                                  debug = True)
    app.listen(8888)
    tornado.ioloop.PeriodicCallback(try_exit, 100).start()
    tornado.ioloop.IOLoop.current().start()
