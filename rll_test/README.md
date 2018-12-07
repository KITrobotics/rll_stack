 
# Robot Learning Lab Stack Tests


## Test explanation

#### api\_testing.py: 
Only tests submissions for now.

   - test_tag_exists: 
   Submits a existing tag
   - test_tag_not_exists: 
   Submits a non existing tag
   - test_password_wrong: 
   Submits with wrong password
   - test_missing_arg_tag: 
   Submits a request without git\_tag
   - test_missing_arg_username: 
   Submits a request without username
   - test_double_submission: 
   Submits two jobs with same username right after another

#### worker\_testing.py: 
Tests builds of worker

  - test_build_success: 
  Run a successfull build (running container, cloning, building): Check build.log if "[build] Summary: All (n) packages succeeded!" is found. No further status checks.
   - test_build_fail: 
   Run a test where build fails (running container, cloning, building fails): Check job_status = "finished" and job_result = "building project failed". Failure is forced by missing character in package.xml
   - test_job_success: 
   Run a test with a job that finishes(running container, cloning, building,executing): Check for success simply by check job_status: status = success. Additionally checks if log folder exists
   - test_storage_fail: Run a test and write a 11GB file to disk : Check if keyword "MemoryError" occurs in launch log
   - test_infinity_loop: 
   Just run a while loop forever, after certain timeout (by now 303 seconds) job_status is checkd for finished and job_result for "execution timeout". By now checking of build logs fails, needs to be investigated
  - test\_available\_topics\_params\_services: 
  Contains a black and whitelist for services, params and topics. These need to be definded in order to check for topics/params/services that must be available and topics/params/services that must not be enabled. By now, whitelist and blacklist are empty and need to be definded **tbd**
  - test\_check\_internet\_connection:
  Checks for internet connection by trying to reach server from google.com 
  - test\_cpu\_load:
  Generates cpu load for the time specified in the project. By now 300 seconds of 100% load on all available cores.
  
## How to run tests

Prerequisites:

The job collection in the test database needs to be empty.

Run tests directly:

   `cd rll_test/scripts`  

   `python api_testing.py`
    or
   `python worker_testing.py`

   Using rostest:   
   `rostest rll_test test_api.launch` runs only api tests
   `rostest rll_test test_worker.launch` runs only worker tests   
   `rostest rll_test test_all.launch`   runs all tests

   **_In order to use rostest, make sure the python files containing the test code are marked as executable_**

## Evalute tests

Right at the execution you see the overview of the test results, something like:

    `[ROSUNIT] Outputting test results to /home/wirth/.ros/test_results/rll/rosunit-Test_submission.xml
    `[Testcase: test_password_wrong] ... ok
    `[Testcase: test_tag_exists] ... ok
    `[Testcase: test_tag_not_exists] ... ok
    `-------------------------------------------------------------
    `SUMMARY:
    ` * RESULT: SUCCESS
    ` * TESTS: 3
    ` * ERRORS: 0 []
    ` * FAILURES: 0 []   

As you can see, results are also written in the file which is mentioned in the output. There you can see further information like the output of the print statements.
