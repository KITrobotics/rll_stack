#!/usr/bin/env python
PKG='rll'

import sys
import unittest
import roslaunch
import requests
import string
import random
import rospkg
import yaml
import time
from os import path
import re
import rospkg

class test_Worker(unittest.TestCase):
    PASSWORD = '' #will be set in setUpClass from rll_common
    JOBS_API_URL = ''#will be set in setUpClass from rll_common
    
    @classmethod
    def setUpClass(cls):
        
        # settings
        rospack = rospkg.RosPack()
        config_path = path.join(rospack.get_path('rll_common'),"config","rll.yaml")
        with open(config_path, 'r') as doc:
          rll_settings = yaml.load(doc)
        
        cls.PASSWORD = rll_settings['secret']
        print("Using password from rll_common")
        api_base_url = rll_settings['api_base_url']
        cls.JOBS_API_URL = api_base_url + "jobs"
        print("Using %s as JOBS_API_URL from rll_common" % cls.JOBS_API_URL)

    def test_build_success(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
    
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'tower_hanoi_working'}
    
        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print("Submit result: ",res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success") #Make sure submission successfull
    
    
        timeout = 30 #[seconds]
    
        job_id = res_json["job_id"] #Get new submitted job id from response
        start_time = time.time()
        while time.time() < start_time + timeout:
            print("wait for timeout")
            time.sleep(5)
    
        log_folder = path.join(rll_settings["job_data_save_dir"],job_id)
        print("Path for logs: "+log_folder)
        print(str(path.exists(log_folder)))
        self.assertTrue(path.exists(log_folder),"Check for log_folder")
    
        build_log = path.join(log_folder,"build.log")
        found = re.search(re.escape("[build]")+" Summary: All \d+ packages succeeded!",open(build_log).read())
        self.assertTrue(found,'Assert that build successfull string in build logfile found')
    
    
    def test_build_fail(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
    
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'build_failure'}
    
        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print("Submit result: ",res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success") #Make sure submission successfull
    
        build_timeout = 100 #[seconds]
    
        job_id = res_json["job_id"] #Get new submitted job id from response
        start_time = time.time()
        job_status = ""
        job_result = ""
        job_finished = False
        while time.time() < start_time + build_timeout:
            poll_result = requests.get("http://localhost:8888/jobs",{'op':'status','job':job_id})
            print "Poll result:"+poll_result.text
            poll_json = poll_result.json()
            job_status = poll_json["job_status"]
            if job_status == "finished":
                print "Job status is now finished"
                job_finished = True
                job_result = poll_json["job_result"]
                print "Job result: "+job_result
                break
            time.sleep(5)
    
        if job_finished:
            log_folder = path.join(rll_settings["job_data_save_dir"],job_id)
            print("Path for logs: "+log_folder)
            self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")
    
            self.assertEqual(job_result,'building project failed')
    
    
    def test_job_success(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
    
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'tower_hanoi_working'}
    
        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print("Submit result: ",res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success") #Make sure submission successfull
    
        timeout = 100 #[seconds]
    
        job_id = res_json["job_id"] #Get new submitted job id from response
        start_time = time.time()
        job_status = ""
        job_result = ""
        job_finished = False
        while time.time() < start_time + timeout:
            poll_result = requests.get("http://localhost:8888/jobs",{'op':'status','job':job_id})
            print "Poll result:"+poll_result.text
            poll_json = poll_result.json()
            job_status = poll_json["job_status"]
            if job_status == "finished":
                print "Job status is now finished"
                job_finished = True
                job_result = poll_json["job_result"]
                print "Job result: "+job_result
                break;
            time.sleep(5)
    
        self.assertTrue(job_finished,"Assert that job_result is set to finished")
        if job_finished:
            log_folder = path.join(rll_settings["job_data_save_dir"],job_id)
            print("Path for logs: "+log_folder)
            self.assertTrue(path.exists(log_folder),"Check for log_folder")
    
            self.assertEqual(job_result,"success")
    
    
    def test_storage_fail(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
    
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'storage_fail'}
    
        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        job_id = res_json["job_id"]
        self.assertEqual(res_status, "success")
    
        timeout = 30 #[seconds]
    
        job_id = res_json["job_id"] #Get new submitted job id from response
        start_time = time.time()
        while time.time() < start_time + timeout:
            print("wait for timeout")
            time.sleep(5)
    
        log_folder = path.join(rll_settings["job_data_save_dir"],job_id)
        print("Path for logs: "+log_folder)
        print(str(path.exists(log_folder)))
        self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")
    
        build_log = path.join(log_folder,"build.log")
        found = re.search(re.escape("[build]")+" Summary: All \d+ packages succeeded!",open(build_log).read())
        self.assertTrue(found,'Assert that build successfull string in build logfile found')
        
        start_time = time.time()
        while time.time() < start_time + timeout:
            print("wait for launch log")
            time.sleep(5)
    
        launch_log = path.join(log_folder,"launch.log")
        search_string = "MemoryError"
        success = search_string in open(build_log).read()
    
        self.assertTrue(success,'Assert that MemoryError string was found in logfile')
    
    def test_infinity_loop(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
    
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'infinity_loop'}
    
        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        job_id = res_json["job_id"]
        self.assertEqual(res_status, "success")
    
        build_timeout = 30 #[seconds]
    
        job_id = res_json["job_id"] #Get new submitted job id from response
        start_time = time.time()
        while time.time() < start_time + build_timeout:
            print("wait for build to finish")
            time.sleep(5)
    
        log_folder = path.join(rll_settings["job_data_save_dir"],job_id)
        print("Path for logs: "+log_folder)
        print(str(path.exists(log_folder)))
        self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")
    
        build_log = path.join(log_folder,"build.log")
        found = re.search(re.escape("[build]")+" Summary: All \d+ packages succeeded!",open(build_log).read())
        self.assertTrue(found,'build successfull string found')
    
    
        execution_timeout = 303 #[seconds]
        start_time = time.time()
        job_status = ""
        job_result = ""
        job_finished = False
        while time.time() < start_time + execution_timeout:
            poll_result = requests.get("http://localhost:8888/jobs",{'op':'status','job':job_id})
            print "Poll result:"+poll_result.text
            poll_json = poll_result.json()
            job_status = poll_json["job_status"]
            if job_status == "finished":
                print "Job status is now finished"
                job_finished = True
                job_result = poll_json["job_result"]
                print "Job result: "+job_result
                break;
            time.sleep(5)
    
        self.assertTrue(job_finished,"Job is finished")
        if job_finished:
            log_folder = path.join(rll_settings["job_data_save_dir"],job_id)
            print("Path for logs: "+log_folder)
            self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")
            self.assertEqual(job_result,"execution timeout")

            
    def test_available_topics_params_services(self):
        #whitelist must be found
        #blacklist must not be found
        
        topics_whitelist = ["/this/topic/must/exist","this/as/well"]
        topics_blacklist = ["these/topics/must/not/exist","/iiwa_2/move_group/goal"]

        params_whitelist = []
        params_blacklist = []

        services_whitelist = []
        services_blacklist = []
        
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'topics_params_services'}
        
        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print("Submit result: ",res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success") #Make sure submission successfull
        
        
        timeout = 30 #[seconds]
        
        job_id = res_json["job_id"] #Get new submitted job id from response
        start_time = time.time()
        while time.time() < start_time + timeout:
            print("wait for timeout")
            time.sleep(5)
        
        log_folder = path.join(rll_settings["job_data_save_dir"],job_id)
        print("Path for logs: "+log_folder)
        print(str(path.exists(log_folder)))
        self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")
        
        build_log = path.join(log_folder,"build.log")
        found = re.search(re.escape("[build]")+" Summary: All \d+ packages succeeded!",open(build_log).read())
        self.assertTrue(found,'build successfull string found')
        
        timeout = 300 #[seconds]
        job_id = res_json["job_id"] #Get new submitted job id from response
        start_time = time.time()
        job_status = ""
        job_result = ""
        job_finished = False
        while time.time() < start_time + timeout:
            poll_result = requests.get("http://localhost:8888/jobs",{'op':'status','job':job_id})
            print "Poll result:"+poll_result.text
            poll_json = poll_result.json()
            job_status = poll_json["job_status"]
            if job_status == "finished":
                print "Job status is now finished"
                job_finished = True
                job_result = poll_json["job_result"]
                print "Job result: "+job_result
                break;
            time.sleep(5)
        
        self.assertTrue(job_finished,"Job is finished")
        if job_finished:            
            self.assertEqual(job_result,"success")
            
        launch_log = path.join(log_folder,"launch.log")

        #Check launch log-file for topics
        whitelist_check_topics = True
        for topic in topics_whitelist:
            found = topic in open(launch_log).read()
            print("check for topic "+topic+": "+str(found))
            if not found:
                whitelist_check_topics = False
                print("didnt find topic "+topic+" from whitelist")
        
        self.assertTrue(whitelist_check_topics,"Whitelist check succeded")
        
        blacklist_check_topics = True
        for topic in topics_blacklist:
            found = topic in open(launch_log).read()
            if found:
                blacklist_check_topics = False
                print("found topic "+topic+" from blacklist")
        
        self.assertTrue(blacklist_check_topics,"blacklist check succeded")

        #Check launch log-file for params
        whitelist_check_params = True
        for param in params_whitelist:
            found = topic in open(launch_log).read()
            if not found:
                whitelist_check_params = False
                print("didnt find topic " + param + " from whitelist")

        self.assertTrue(whitelist_check_params, "Whitelist check succeded")

        blacklist_check_params = True
        for param in params_blacklist:
            found = topic in open(launch_log).read()
            if found:
                blacklist_check_params = False
                print("found topic " + param + " from blacklist")

        self.assertTrue(blacklist_check_params, "blacklist check succeded")

        whitelist_check_services = True
        for service in services_whitelist:
            found = topic in open(launch_log).read()
            if not found:
                whitelist_check_services = False
                print("didnt find topic " + service + " from whitelist")

        self.assertTrue(whitelist_check_services, "Whitelist check succeded")

        blacklist_check_services = True
        for service in services_blacklist:
            found = topic in open(launch_log).read()
            if found:
                blacklist_check_services = False
                print("found topic " + service + " from blacklist")

        self.assertTrue(blacklist_check_services, "blacklist check succeded")
        
                    
    def test_check_internet_connection(self):
        internet_available_string = "internet connection available"
        
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'internet_check'}
        
        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print("Submit result: ",res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success") #Make sure submission successfull
        
        
        timeout = 30 #[seconds]
        
        job_id = res_json["job_id"] #Get new submitted job id from response
        start_time = time.time()
        while time.time() < start_time + timeout:
            print("wait for timeout")
            time.sleep(5)
        
        log_folder = path.join(rll_settings["job_data_save_dir"],job_id)
        print("Path for logs: "+log_folder)
        print(str(path.exists(log_folder)))
        self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")
        
        build_log = path.join(log_folder,"build.log")
        found = re.search(re.escape("[build]")+" Summary: All \d+ packages succeeded!",open(build_log).read())
        self.assertTrue(found,'Assert that build successfull string in build logfile found')
        
        timeout = 300 #[seconds]
        job_id = res_json["job_id"] #Get new submitted job id from response
        start_time = time.time()
        job_status = ""
        job_result = ""
        job_finished = False
        while time.time() < start_time + timeout:
            poll_result = requests.get("http://localhost:8888/jobs",{'op':'status','job':job_id})
            print "Poll result:"+poll_result.text
            poll_json = poll_result.json()
            job_status = poll_json["job_status"]
            if job_status == "finished":
                print "Job status is now finished"
                job_finished = True
                job_result = poll_json["job_result"]
                print "Job result: "+job_result
                break;
            time.sleep(5)
        
        self.assertTrue(job_finished,"Job is finished")
        if job_finished:            
            self.assertEqual(job_result,"success")
        
        timeout = 10 #[seconds]
        start_time = time.time()
        while time.time() < start_time + timeout:
            print("wait for launch file")
            time.sleep(5)
            
        launch_log = path.join(log_folder,"launch.log")
        internet_avail = internet_available_string in open(launch_log).read()
        print("Result of find: "+str(internet_avail))
        
        #Check aunch log-file for topics
        self.assertFalse(internet_avail,"Assert that Internet is not available")
        
    def test_cpu_load(self):
        
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])
        
        payload = {'op': 'submit', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'cpu_load'}
        
        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print("Submit result: ",res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success") #Make sure submission successfull
        
        
        timeout = 30 #[seconds]
        
        job_id = res_json["job_id"] #Get new submitted job id from response
        start_time = time.time()
        while time.time() < start_time + timeout:
            print("wait for timeout")
            time.sleep(5)
        
        log_folder = path.join(rll_settings["job_data_save_dir"],job_id)
        print("Path for logs: "+log_folder)
        print(str(path.exists(log_folder)))
        self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")
        
        build_log = path.join(log_folder,"build.log")
        found = re.search(re.escape("[build]")+" Summary: All \d+ packages succeeded!",open(build_log).read())
        self.assertTrue(found,'Assert that build successfull string in build logfile found')
        
        timeout = 305 #[seconds]
        job_id = res_json["job_id"] #Get new submitted job id from response
        start_time = time.time()
        job_status = ""
        job_result = ""
        job_finished = False
        while time.time() < start_time + timeout:
            poll_result = requests.get("http://localhost:8888/jobs",{'op':'status','job':job_id})
            print "Poll result:"+poll_result.text
            poll_json = poll_result.json()
            job_status = poll_json["job_status"]
            if job_status == "finished":
                print "Job status is now finished"
                job_finished = True
                job_result = poll_json["job_result"]
                print "Job result: "+job_result
                break;
            time.sleep(5)
        
        self.assertTrue(job_finished,"Job is finished")
        if job_finished:            
            self.assertEqual(job_result,"success")
        
        timeout = 10 #[seconds]
        start_time = time.time()
        while time.time() < start_time + timeout:
            print("wait for launch file")
            time.sleep(5)
            
        launch_log = path.join(log_folder,"launch.log")
        self.assertTrue(path.exists(launch_log),"Assert that launch file exists")
        
        

        
if __name__ == '__main__':
    import rosunit
    
    # settings
    rospack = rospkg.RosPack()
    config_path = rospack.get_path('rll_common') + "/config/rll.yaml"
    with open(config_path, 'r') as doc:
        rll_settings = yaml.load(doc)

    
    rosunit.unitrun(PKG, 'test_Worker', test_Worker)
 
