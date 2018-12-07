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

    def __init__(self, *args, **kwargs):
        super(test_Worker, self).__init__(*args, **kwargs)

        # settings
        rospack = rospkg.RosPack()
        config_path = path.join(rospack.get_path('rll_common'),"config","rll.yaml")
        with open(config_path, 'r') as doc:
          rll_settings = yaml.load(doc)

        self.PASSWORD = rll_settings['secret']
        print("Using password from rll_common")
        api_base_url = rll_settings['api_base_url']
        self.JOBS_API_URL = api_base_url + "jobs"
        print("Using %s as JOBS_API_URL from rll_common" % self.JOBS_API_URL)

    def test_build_success(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])

        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'tower_hanoi_working',}

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

        log_folder = path.join(rll_settings["test_job_data_save_dir"],job_id)
        print("Path for logs: "+log_folder)
        print(str(path.exists(log_folder)))
        self.assertTrue(path.exists(log_folder),"Check for log_folder")

        build_log = path.join(log_folder,"build.log")
        found = re.search(re.escape("[build]")+" Summary: All \d+ packages succeeded!",self.escape_ansi(open(build_log).read()))
        self.assertTrue(found,'Assert that build successfull string in build logfile found')


    def test_build_fail(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])

        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'build_failure'}

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
            log_folder = path.join(rll_settings["test_job_data_save_dir"],job_id)
            print("Path for logs: "+log_folder)
            self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")

            self.assertEqual(job_result,'building project failed')


    def test_job_success_git(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])

        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'tower_hanoi_working'}

        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print("Submit result: ",res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success") #Make sure submission successfull

        timeout = 200 #[seconds]

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
            if job_status == "waiting for real":
                print "Job status is now finished"
                job_finished = True
                job_result = poll_json["job_result"]
                print "Job result: "+job_result
                break;
            time.sleep(5)

        self.assertTrue(job_finished,"Assert that job_result is set to waiting for real")
        if job_finished:
            log_folder = path.join(rll_settings["test_job_data_save_dir"],job_id)
            print("Path for logs: "+log_folder)
            self.assertTrue(path.exists(log_folder),"Check for log_folder")

            self.assertEqual(job_result,"sim success")


    def test_storage_fail(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])

        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'storage_fail'}

        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print(res_json)
        res_status = res_json["status"]
        job_id = res_json["job_id"]
        self.assertEqual(res_status, "success")

        timeout = 60 #[seconds]

        job_id = res_json["job_id"] #Get new submitted job id from response
        start_time = time.time()
        while time.time() < start_time + timeout:
            print("wait for timeout")
            time.sleep(5)

        start_time = time.time()
        while time.time() < start_time + timeout:
            print("wait for launch log")
            time.sleep(5)


        log_folder = path.join(rll_settings["test_job_data_save_dir"],job_id)
        print("Path for logs: "+log_folder)
        print(str(path.exists(log_folder)))
        self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")

        build_log = path.join(log_folder,"build.log")
        found = re.search(re.escape("[build]")+" Summary: All \d+ packages succeeded!",self.escape_ansi(open(build_log).read()))
        self.assertTrue(found,'Assert that build successfull string in build logfile found')


        launch_log = path.join(log_folder,"sim-client.log")
        search_string = "No space left on device"
        success = search_string in self.escape_ansi(open(launch_log).read())

        self.assertTrue(success,'Assert that no space is left on device')


    def test_available_topics_params_services(self):
        #whitelist must be found
        #blacklist must not be found

        topics_whitelist = ["/iiwa/move_client/result"]
        topics_blacklist = ["/iiwa/job_env","/iiwa/job_idle","/gazebo"]

        params_whitelist = []
        params_blacklist = []

        services_whitelist = ["/iiwa/move_joints","/iiwa/move_lin","/iiwa/pick_place"]
        services_blacklist = ["/gazebo"]

        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])

        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'topics_params_services'}

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
            if job_status == "waiting for real":
                print "Job status is now finished"
                job_finished = True
                job_result = poll_json["job_result"]
                print "Job result: "+job_result
                break;
            time.sleep(5)

        self.assertTrue(job_finished,"Job is finished")
        if job_finished:
            self.assertEqual(job_result,"sim success")

        log_folder = path.join(rll_settings["test_job_data_save_dir"],job_id)
        print("Path for logs: "+log_folder)
        print(str(path.exists(log_folder)))
        self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")

        build_log = path.join(log_folder,"build.log")
        found = re.search(re.escape("[build]")+" Summary: All \d+ packages succeeded!",self.escape_ansi(open(build_log).read()))
        self.assertTrue(found,'build successfull string found')

        launch_log = path.join(log_folder,"sim-client.log")
        #Log file example:
        #Topics:

        #[['/iiwa/move_client/goal', 'rll_msgs/DefaultMoveIfaceActionGoal'], ['/iiwa/move_client/result', 'rll_msgs/DefaultMoveIfaceActionResult'], ['/rosout', 'rosgraph_msgs/Log'], ['/rosout_agg', 'rosgraph_msgs/Log'], ['/iiwa/move_client/cancel', 'actionlib_msgs/GoalID'], ['/iiwa/move_client/feedback', 'rll_msgs/DefaultMoveIfaceActionFeedback'], ['/iiwa/move_client/status', 'actionlib_msgs/GoalStatusArray']]
        #Params:

        #/rosdistro
        #/roslaunch/uris/host_172_18_0_3__43285
        #/roslaunch/uris/host_172_18_0_3__45441
        #/roslaunch/uris/host_ic_iiwa__43499
        #/rosversion
        #/run_id

        #Services:

        #/iiwa/move_joints
        #/iiwa/move_lin
        #/iiwa/pick_place
        #/iiwa/test_project/get_loggers
        #/iiwa/test_project/set_logger_level
        #/rosout/get_loggers
        #/rosout/set_logger_level

        topics_log = ""
        services_log = ""
        params_log = ""
        last_keyword = ""
        for line in open(launch_log).read().splitlines():
            print line
            if "Topics:" in line:
                last_keyword = "Topics:"
                continue
            if "Services:" in line:
                last_keyword = "Services:"
                continue
            if "Params:" in line:
                last_keyword = "Params:"
                continue
            if last_keyword is "Topics:":
                topics_log = topics_log + line
                continue
            if last_keyword is "Services:":
                services_log = services_log + line +"\n"
                continue
            if last_keyword is "Params:":
                params_log =  params_log + line +"\n"
                continue


        print("Found Topics:\n"+topics_log)
        print("Found Services:\n"+services_log)
        print("Found Params:\n"+params_log)


        #Check launch log-file for topics
        whitelist_check_topics = True
        for topic in topics_whitelist:
            found = topic in topics_log
            print("check for topic "+topic+": "+str(found))
            if not found:
                whitelist_check_topics = False
                print("didnt find topic "+topic+" from whitelist")

        self.assertTrue(whitelist_check_topics,"Whitelist check succeded")

        blacklist_check_topics = True
        for topic in topics_blacklist:
            found = topic in topics_log
            if found:
                blacklist_check_topics = False
                print("found topic "+topic+" from blacklist")

        self.assertTrue(blacklist_check_topics,"blacklist check succeded")

        #Check launch log-file for params
        whitelist_check_params = True
        for param in params_whitelist:
            found = param in params_log
            if not found:
                whitelist_check_params = False
                print("didnt find topic " + param + " from whitelist")

        self.assertTrue(whitelist_check_params, "Whitelist check succeded")

        blacklist_check_params = True
        for param in params_blacklist:
            found = param in params_log
            if found:
                blacklist_check_params = False
                print("found topic " + param + " from blacklist")

        self.assertTrue(blacklist_check_params, "blacklist check succeded")

        whitelist_check_services = True
        for service in services_whitelist:
            found = service in services_log
            if not found:
                whitelist_check_services = False
                print("didnt find topic " + service + " from whitelist")

        self.assertTrue(whitelist_check_services, "Whitelist check succeded")

        blacklist_check_services = True
        for service in services_blacklist:
            found = topic in services_log
            if found:
                blacklist_check_services = False
                print("found topic " + service + " from blacklist")

        self.assertTrue(blacklist_check_services, "blacklist check succeded")


    def test_check_internet_connection(self):
        internet_available_string = "internet connection available"

        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])

        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'internet_check'}

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
            if job_status == "waiting for real":
                print "Job status is now finished"
                job_finished = True
                job_result = poll_json["job_result"]
                print "Job result: "+job_result
                break;
            time.sleep(5)

        self.assertTrue(job_finished,"Job is finished")
        if job_finished:
            self.assertEqual(job_result,"sim success")

        log_folder = path.join(rll_settings["test_job_data_save_dir"],job_id)
        print("Path for logs: "+log_folder)
        print(str(path.exists(log_folder)))
        self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")

        build_log = path.join(log_folder,"build.log")
        found = re.search(re.escape("[build]")+" Summary: All \d+ packages succeeded!",self.escape_ansi(open(build_log).read()))
        self.assertTrue(found,'Assert that build successfull string in build logfile found')

        timeout = 10 #[seconds]
        start_time = time.time()
        while time.time() < start_time + timeout:
            print("wait for launch file")
            time.sleep(5)

        launch_log = path.join(log_folder,"sim-client.log")
        internet_avail = internet_available_string in open(launch_log).read()
        print("Result of find: "+str(internet_avail))

        #Check aunch log-file for topics
        self.assertFalse(internet_avail,"Assert that Internet is not available")

    def test_cpu_load(self):

        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])

        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'cpu_load'}

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
            if job_status == "waiting for real":
                print "Job status is now finished"
                job_finished = True
                job_result = poll_json["job_result"]
                print "Job result: "+job_result
                break;
            time.sleep(5)

        self.assertTrue(job_finished,"Job is finished")
        if job_finished:
            self.assertEqual(job_result,"sim success")

        log_folder = path.join(rll_settings["test_job_data_save_dir"],job_id)
        print("Path for logs: "+log_folder)
        print(str(path.exists(log_folder)))
        self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")

        build_log = path.join(log_folder,"build.log")
        found = re.search(re.escape("[build]")+" Summary: All \d+ packages succeeded!",self.escape_ansi(open(build_log).read()))
        self.assertTrue(found,'Assert that build successfull string in build logfile found')

        timeout = 10 #[seconds]
        start_time = time.time()
        while time.time() < start_time + timeout:
            print("wait for launch file")
            time.sleep(5)

        launch_log = path.join(log_folder,"sim-client.log")
        self.assertTrue(path.exists(launch_log),"Assert that launch file exists")



    def test_infinity_loop(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])

        payload = {'op': 'submit_git', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','git_url':'https://gitlab.ipr.kit.edu/rll/test_projects.git','git_tag':'infinity_loop'}

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


        execution_timeout = 490 #[seconds]
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
            log_folder = path.join(rll_settings["test_job_data_save_dir"],job_id)
            print("Path for logs: "+log_folder)
            self.assertTrue(path.exists(log_folder),"Assert that log_folder exists")
            self.assertEqual(job_result,"sim failure")

    def test_job_success_tar(self):
        random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(6)])

        payload = {'op': 'submit_tar', 'username': random_name,'secret':self.PASSWORD,'project':'test_projects','tar_url':'https://gitlab.ipr.kit.edu/rll/test_projects/raw/master/test_data/tower_hanoi_working.tar'}

        res = requests.post(self.JOBS_API_URL,data=payload)
        res_json = res.json()
        print("Submit result: ",res_json)
        res_status = res_json["status"]
        self.assertEqual(res_status, "success") #Make sure submission successfull

        timeout = 200 #[seconds]

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
            if job_status == "waiting for real":
                print "Job status is now finished"
                job_finished = True
                job_result = poll_json["job_result"]
                print "Job result: "+job_result
                break;
            time.sleep(5)

        self.assertTrue(job_finished,"Assert that job_result is set to waiting for real")
        if job_finished:
            log_folder = path.join(rll_settings["test_job_data_save_dir"],job_id)
            print("Path for logs: "+log_folder)
            self.assertTrue(path.exists(log_folder),"Check for log_folder")

            code_file = path.join(log_folder,"code.tar")
            print("Path for tar file: "+code_file)
            self.assertTrue(path.exists(code_file),"Check for tar file")

            self.assertEqual(job_result,"sim success")


    def escape_ansi(self,line):
        ansi_escape = re.compile(r'(\x9B|\x1B\[)[0-?]*[ -/]*[@-~]')
        return ansi_escape.sub('', line)



if __name__ == '__main__':
    import rosunit

    # settings
    rospack = rospkg.RosPack()
    config_path = rospack.get_path('rll_common') + "/config/rll.yaml"
    with open(config_path, 'r') as doc:
        rll_settings = yaml.load(doc)


    rosunit.unitrun(PKG, 'test_Worker', test_Worker)
