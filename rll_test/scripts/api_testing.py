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
    PASSWORD = '' #will be set in setUpClass from rll_config
    JOBS_API_URL = ''#will be set in setUpClass from rll_config
        
    @classmethod
    def setUpClass(cls):
        
        # settings
        rospack = rospkg.RosPack()
        config_path = os.path.join(rospack.get_path('rll_config'),"config","rll.yaml")
        with open(config_path, 'r') as doc:
          rll_settings = yaml.load(doc)
        
        cls.PASSWORD = rll_settings['secret']
        print("Using password from rll_config")
        api_base_url = rll_settings['api_base_url']
        cls.JOBS_API_URL = api_base_url + "jobs"
        print("Using %s as JOBS_API_URL from rll_config" % cls.JOBS_API_URL)
        
        
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
        self.assertEqual(res_error2, "User has a job in the queue")
        
        
        
        
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_Submission', test_Submission)
 
