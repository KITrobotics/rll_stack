#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab stack
#
# Copyright (C) 2018 Raphael Wirth <uwefu@student.kit.edu>
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

PKG='rll_test'

import sys
import unittest
import roslaunch
import requests
import string
import random
import rospkg
import yaml
import os
from pymongo import MongoClient
from bson import json_util


class test_Submission(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(test_Submission, self).__init__(*args, **kwargs)

        # settings
        rospack = rospkg.RosPack()
        config_path = os.path.join(rospack.get_path('rll_common'),"config","rll.yaml")
        with open(config_path, 'r') as doc:
          rll_settings = yaml.load(doc)

        self.PASSWORD = rll_settings['secret']
        print("Using password from rll_common")
        api_base_url = rll_settings['api_base_url']
        self.JOBS_API_URL = api_base_url + "jobs"
        print("Using %s as JOBS_API_URL from rll_common" % self.JOBS_API_URL)
        self.SYSTEM_API_URL = api_base_url + "system/job_stats"
        print("Using %s as SYSTEM_API_URL from rll_common" % self.SYSTEM_API_URL)
        self.MAX_QUEUE = rll_settings['sub_queue_limit']
        print("Using %d as MAX_QUEUE from rll_common" % self.MAX_QUEUE)
        self.TEST_DB_NAME = rll_settings["test_db_name"]
        print("Using %s as TEST_DB_NAME from rll_common" % self.TEST_DB_NAME)

    def test_tag_exists(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        print("api url in method %s" % self.JOBS_API_URL)
        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}

        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success")

    def test_tag_not_exists(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])

        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.01'}


        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "error")

    def test_password_wrong(self):
        payload = {'op': 'submit_git', 'username': "bla",'secret':'wrong_pw','project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}

        res = requests.post(self.JOBS_API_URL,data=payload)

        res_txt = res.text
        print(res_txt)
        self.assertTrue("HTTPError: HTTP 401: Unauthorized" in res_txt)

    def test_missing_arg_tag(self):
        payload = {'op': 'submit_git', 'username': "bla",'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git'} #Gittag is missing

        res = requests.post(self.JOBS_API_URL,data=payload)

        res_txt = res.text
        print(res_txt)
        self.assertTrue("MissingArgumentError: HTTP 400: Bad Request (Missing argument git_tag)" in res_txt)

    def test_missing_arg_username(self):
        payload = {'op': 'submit_git','secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git', 'git_tag': 'bla'} #Gittag is missing

        res = requests.post(self.JOBS_API_URL,data=payload)

        res_txt = res.text
        print(res_txt)
        self.assertTrue("MissingArgumentError: HTTP 400: Bad Request (Missing argument username)" in res_txt)

    def test_double_submission(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}

        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success") #First submission with username is ok

        res2 = requests.post(self.JOBS_API_URL,data=payload)
        res_json2 = res2.json()
        print(res_json2)
        res_status2 = res_json2["status"]
        res_error2 = res_json2["error"]
        self.assertEqual(res_status2, "error") #Second submission with username is error
        self.assertEqual(res_error2, "Nothing was modified")

    def test_tag_update(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}


        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success") #First submission with username is ok

        #change new git_tag
        new_git_tag = 'v0.2'
        payload['git_tag']=new_git_tag
        res2 = requests.post(self.JOBS_API_URL,data=payload)
        res_json2 = res2.json()
        print(res_json2)
        res_status2 = res_json2["status"]
        res_jobid = res_json2["job_id"]
        self.assertEqual(res_status2, "success") #Second submission with username is error

        #Think about checking new tag --> status operation would need to return more infromation for that


    def test_job_positions(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}


        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        res_jobid = res_json["job_id"]
        self.assertEqual(res_status, "success") #First submission with username is ok

        #check position of first submission
        payload_status1 = {'op': 'status', 'job': res_jobid}

        res_status_json = requests.get(self.JOBS_API_URL,data=payload_status1).json()
        position1 = res_status_json['position']

        random_name2 = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        payload2 = {'op': 'submit_git', 'username': random_name2,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}

        res = requests.post(self.JOBS_API_URL,data=payload2)
        res_json2 = res.json()
        res_status2 = res_json2["status"]
        res_jobid2 = res_json2["job_id"]
        self.assertEqual(res_status2, "success") #Second submission with username is ok

        #check position of second submission
        payload_status2 = {'op': 'status', 'job':res_jobid2}

        res_status2_json = requests.get(self.JOBS_API_URL,data=payload_status2).json()
        position2 = res_status2_json['position']

        self.assertEqual(position1+1,position2)


    def test_systems_stats(self):
        client = MongoClient('mongodb://localhost:27017/')
        db = client[self.TEST_DB_NAME]
        collection = db.jobs

        #Clear collection
        collection.drop()

        #Load collection from json file
        test_entries = []
        rospack = rospkg.RosPack()
        test_db = rospack.get_path('rll_test') + "/data/test_job_stats_db.json"
        for line in open(test_db, 'r'):
            test_entries.append(json_util.loads(line))

        result = collection.insert_many(test_entries)

        ##request job stats
        sysstats_status_request = {'job_info_type': 'status', 'project': 'test_projects'}
        res_status_json = requests.get(self.SYSTEM_API_URL,data=sysstats_status_request).json()

        ##request job results
        sysstats_result_request = {'job_info_type': 'result', 'project': 'test_projects'}
        res_result_json = requests.get(self.SYSTEM_API_URL,data=sysstats_result_request).json()

        correct_status = json_util.loads('{"running real": 0, "submitted": 0, "running sim": 0, "processing sim started": 0, "finished": 14, "processing real started": 0, "total": 53, "waiting for real": 39, "downloading code": 0}')

        correct_result = json_util.loads('{"sim internal error": 0, "launching project failed": 0, "real failure": 0, "real success": 0, "sim success": 46, "total real": 0, "sim failure": 3, "building project failed": 4, "fetching project code failed": 0, "unknown": 0, "total": 53, "real internal error": 0}')

        status_compare = True if correct_status==res_status_json else False
        result_compare = True if correct_result==res_result_json else False

        self.assertTrue(status_compare)
        self.assertTrue(result_compare)



    def test_zqueue_maximum(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}


        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        res_jobid = res_json["job_id"]
        self.assertEqual(res_status, "success") #First submission with username is ok

        #check position of first submission
        payload_status1 = {'op': 'status', 'job': res_jobid}

        res_status_json = requests.get(self.JOBS_API_URL,data=payload_status1).json()
        position1 = res_status_json['position']

        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        #Fill up queue with submissions until limit is reached
        for pos in range(position1,self.MAX_QUEUE+1):
            sub_username = random_name + 'queue_maximum_test'+str(pos)
            payload = {'op': 'submit_git', 'username': sub_username,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.2'}

            res = requests.post(self.JOBS_API_URL,data=payload)
            res_json2 = res.json()
            res_status2 = res_json2["status"]
            res_jobid2 = res_json2["job_id"]
            self.assertEqual(res_status2, "success")

        #If queue is full, try another submission and check if it fails with message queue is full
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}

        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        res_error = res_json["error"]
        self.assertEqual(res_status, "error")
        self.assertEqual(res_error,"Queue at max size")

        #After filling queue to max, clear collection database again
        client = MongoClient('mongodb://localhost:27017/')
        db = client[self.TEST_DB_NAME]
        collection = db.jobs

        #Clear collection
        collection.drop()


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_Submission', test_Submission)
