#!/usr/bin/env python
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



class test_Submission(unittest.TestCase):
    PASSWORD = '' #will be set in setUpClass from rll_common
    JOBS_API_URL = ''#will be set in setUpClass from rll_common
    MAX_QUEUE = 0#will be set in setUpClass from rll_common
    @classmethod
    def setUpClass(cls):
        
        # settings
        rospack = rospkg.RosPack()
        config_path = os.path.join(rospack.get_path('rll_common'),"config","rll.yaml")
        with open(config_path, 'r') as doc:
          rll_settings = yaml.load(doc)
        
        cls.PASSWORD = rll_settings['secret']
        print("Using password from rll_common")
        api_base_url = rll_settings['api_base_url']
        cls.JOBS_API_URL = api_base_url + "jobs"
        print("Using %s as JOBS_API_URL from rll_common" % cls.JOBS_API_URL)
        cls.MAX_QUEUE = rll_settings['sub_queue_limit']
        print("Using %d as MAX_QUEUE from rll_common" % cls.MAX_QUEUE)
        
        
    def test_tag_exists(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        print("api url in method %s" % self.JOBS_API_URL)
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}
        
        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success")
        
    def test_tag_not_exists(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])

        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.01'}
        

        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "error")
        
    def test_password_wrong(self):
        payload = {'op': 'submit', 'username': "bla",'secret':'wrong_pw','project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}
        
        res = requests.post(self.JOBS_API_URL,data=payload)
        
        res_txt = res.text
        print(res_txt)
        self.assertTrue("HTTPError: HTTP 401: Unauthorized" in res_txt)
        
    def test_missing_arg_tag(self):
        payload = {'op': 'submit', 'username': "bla",'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git'} #Gittag is missing
        
        res = requests.post(self.JOBS_API_URL,data=payload)
        
        res_txt = res.text
        print(res_txt)
        self.assertTrue("MissingArgumentError: HTTP 400: Bad Request (Missing argument git_tag)" in res_txt)
        
    def test_missing_arg_username(self):
        payload = {'op': 'submit','secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git', 'git_tag': 'bla'} #Gittag is missing
        
        res = requests.post(self.JOBS_API_URL,data=payload)
        
        res_txt = res.text
        print(res_txt)
        self.assertTrue("MissingArgumentError: HTTP 400: Bad Request (Missing argument username)" in res_txt)
        
    def test_double_submission(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}
        
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
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}
        
        
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
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}
        
        
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
        payload2 = {'op': 'submit', 'username': random_name2,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}
        
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
        
    def test_zqueue_maximum(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}
        
        
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
        
        #Fill up queue with submissions until limit is reached
        for pos in range(position1,self.MAX_QUEUE+1):
            sub_username = 'queue_maximum_test'+str(pos)
            payload = {'op': 'submit', 'username': sub_username,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.2'}
        
            res = requests.post(self.JOBS_API_URL,data=payload)
            res_json2 = res.json()
            res_status2 = res_json2["status"]
            res_jobid2 = res_json2["job_id"]
            self.assertEqual(res_status2, "success")
        
        #If queue is full, try another submission and check if it fails with message queue is full
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}
        
        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        res_error = res_json["error"]
        self.assertEqual(res_status, "error")
        self.assertEqual(res_error,"Max number of submission in queue is reached.")
        
        
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_Submission', test_Submission)
 
