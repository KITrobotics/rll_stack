#!/usr/bin/env python
PKG='rll_test'

import sys
import unittest
import roslaunch
import requests
import string
import random

PASSWORD = 'password' #RLL password for submission
URL = 'http://localhost:8888/jobs'

class test_Submission(unittest.TestCase):
        
        
    def test_tag_exists(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        
        payload = {'op': 'submit', 'username': random_name,'secret':PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}
        
        res = requests.post(URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success")
        
    def test_tag_not_exists(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])

        payload = {'op': 'submit', 'username': random_name,'secret':PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.01'}
        

        res = requests.post(URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "error")
        
    def test_password_wrong(self):
        payload = {'op': 'submit', 'username': "bla",'secret':'wrong_pw','project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}
        
        res = requests.post(URL,data=payload)
        
        res_txt = res.text
        print(res_txt)
        self.assertTrue("HTTPError: HTTP 401: Unauthorized" in res_txt)
        
    def test_missing_arg_tag(self):
        payload = {'op': 'submit', 'username': "bla",'secret':PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git'} #Gittag is missing
        
        res = requests.post(URL,data=payload)
        
        res_txt = res.text
        print(res_txt)
        self.assertTrue("MissingArgumentError: HTTP 400: Bad Request (Missing argument git_tag)" in res_txt)
        
    def test_missing_arg_username(self):
        payload = {'op': 'submit','secret':PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git', 'git_tag': 'bla'} #Gittag is missing
        
        res = requests.post(URL,data=payload)
        
        res_txt = res.text
        print(res_txt)
        self.assertTrue("MissingArgumentError: HTTP 400: Bad Request (Missing argument username)" in res_txt)
        
    def test_double_submission(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        payload = {'op': 'submit', 'username': random_name,'secret':PASSWORD,'project':'tower_of_hanoi','git_url':'https://gitlab.ipr.kit.edu/rll/tower_of_hanoi.git','git_tag':'v0.1'}
        
        res = requests.post(URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success") #First submission with username is ok
        
        res2 = requests.post(URL,data=payload)
        res_json2 = res2.json()
        print(res_json2)
        res_status2 = res_json2["status"]
        res_error2 = res_json2["error"]
        self.assertEqual(res_status2, "error") #Second submission with username is error
        self.assertEqual(res_error2, "User has a job in the queue")
        
        
        
        
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_Submission', test_Submission)
 
