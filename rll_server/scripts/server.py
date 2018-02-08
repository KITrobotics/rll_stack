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
import signal

is_closing = False

def signal_handler(signum, frame):
    global is_closing
    is_closing = True

def try_exit():
    global is_closing
    if is_closing:
        # clean up here
        tornado.ioloop.IOLoop.instance().stop()

class JobsHandler(tornado.web.RequestHandler):
    def post(self):
        operation = self.get_argument("op")

        if operation == "submit":
            username = self.get_argument("username")
            git_url = self.get_argument("git_url")

            rospy.loginfo("got a new submission with username '%s' and Git repo URL '%s'", username, git_url)
        elif operation == "status":
            job_id = self.get_argument("job")

            rospy.loginfo("got status request for job id '%s'", job_id)

if __name__ == '__main__':
    rospy.init_node('rll_server')
    signal.signal(signal.SIGINT, signal_handler)

    db = motor.motor_tornado.MotorClient().test_database

    app = tornado.web.Application([(r"/jobs", JobsHandler)], db=db,
                                  debug = True)
    app.listen(8888)
    tornado.ioloop.PeriodicCallback(try_exit, 100).start() 
    tornado.ioloop.IOLoop.current().start()
