import tornado.web
from tornado.escape import json_encode
import rospy

JOB_STATUS_SUBMITTED = "submitted"
JOB_STATUS_SIM_START =  "processing sim started"
JOB_STATUS_REAL_START =  "processing real started"
JOB_STATUS_DOWNLOAD = "downloading code"
JOB_STATUS_BUILDING = "building"
JOB_STATUS_SIM_RUN = "running sim"
JOB_STATUS_WAITING_REAL = "waiting for real"
JOB_STATUS_REAL_RUN = "running real"
JOB_STATUS_FINISHED = "finished"

JOB_RESULT_SIM_SUCCESS = "sim success"
JOB_RESULT_REAL_SUCCESS= "real success"
JOB_RESULT_REAL_FAILURE= "real failure"
JOB_RESULT_SIM_FAILURE= "sim failure"
JOB_RESULT_SIM_INTERNAL_ERROR= "sim internal error"
JOB_RESULT_REAL_INTERNAL_ERROR = "real internal error"
JOB_RESULT_UNKNOWN= "unknown"
JOB_RESULT_FETCHING_PROJECT_CODE_FAILED= "fetching project code failed"
JOB_RESULT_LAUNCHING_PROJECT_FAILED= "launching project failed"
JOB_RESULT_BUILDING_PROJECT_FAILED= "building project failed"

job_status_codes = [JOB_STATUS_SUBMITTED, JOB_STATUS_SIM_START, JOB_STATUS_REAL_START,
                    JOB_STATUS_DOWNLOAD, JOB_STATUS_BUILDING, JOB_STATUS_SIM_RUN,
                    JOB_STATUS_WAITING_REAL, JOB_STATUS_REAL_RUN, JOB_STATUS_FINISHED]

job_result_codes = [JOB_RESULT_SIM_SUCCESS,JOB_RESULT_REAL_SUCCESS,JOB_RESULT_REAL_FAILURE,JOB_RESULT_SIM_FAILURE,
                    JOB_RESULT_SIM_INTERNAL_ERROR,JOB_RESULT_REAL_INTERNAL_ERROR,JOB_RESULT_UNKNOWN,JOB_RESULT_FETCHING_PROJECT_CODE_FAILED,
                    JOB_RESULT_LAUNCHING_PROJECT_FAILED,JOB_RESULT_BUILDING_PROJECT_FAILED]
    
class SystemJobStatsHandler(tornado.web.RequestHandler):
        
    def initialize(self, db, rll_settings):
        self.jobs_collection = db.jobs
        self.rll_settings = rll_settings
    
    @tornado.web.asynchronous
    def get(self):
        job_type = self.get_argument("job_info_type")
        project = self.get_argument("project")
        
        if not project in self.rll_settings["projects"]:
            rospy.logwarn("unknown project name in submission")
            raise tornado.web.HTTPError(400)
        
        if job_type == "result":
            response = self.__handle_job_result_stats(project)
        elif job_type == "status":
            response = self.__handle_job_status_stats(project)
        else:
            raise tornado.web.HTTPError(400)
    
    @tornado.gen.coroutine
    def __handle_job_result_stats(self,project):
        response = {}
        
        future_total = self.jobs_collection.find({"project": project}).count()
        future_success_sim = self.jobs_collection.find({"project": project,"job_result": JOB_RESULT_SIM_SUCCESS}).count()
        future_success_real = self.jobs_collection.find({"project": project,"job_result": JOB_RESULT_REAL_SUCCESS}).count()
        future_failure_sim = self.jobs_collection.find({"project": project,"job_result": JOB_RESULT_SIM_FAILURE}).count()
        future_failure_real = self.jobs_collection.find({"project": project,"job_result": JOB_RESULT_REAL_FAILURE}).count()
        future_sim_internal_error = self.jobs_collection.find({"project": project,"job_result": JOB_RESULT_SIM_INTERNAL_ERROR}).count()
        future_real_internal_error = self.jobs_collection.find({"project": project,"job_result": JOB_RESULT_REAL_INTERNAL_ERROR}).count()
        future_unknown = self.jobs_collection.find({"project": project,"job_result": JOB_RESULT_UNKNOWN}).count()
        future_fetching_code_failed = self.jobs_collection.find({"project": project,"job_result": JOB_RESULT_FETCHING_PROJECT_CODE_FAILED}).count()
        future_launching_failed = self.jobs_collection.find({"project": project,"job_result": JOB_RESULT_LAUNCHING_PROJECT_FAILED}).count()
        future_building_failed = self.jobs_collection.find({"project": project,"job_result": JOB_RESULT_BUILDING_PROJECT_FAILED}).count()
        
        try:
            total = yield future_total
            response["total"] = total
            
            sucess_sim = yield future_success_sim
            response[JOB_RESULT_SIM_SUCCESS] = sucess_sim
            
            success_real = yield future_success_real
            response[JOB_RESULT_REAL_SUCCESS] = success_real
            
            failure_sim = yield future_failure_sim
            response[JOB_RESULT_SIM_FAILURE] = failure_sim
            
            failure_real = yield future_failure_real
            response[JOB_RESULT_REAL_FAILURE] = failure_real
            
            internal_sim_error = yield future_sim_internal_error
            response[JOB_RESULT_SIM_INTERNAL_ERROR] = internal_sim_error
            
            internal_real_error = yield future_real_internal_error
            response[JOB_RESULT_REAL_INTERNAL_ERROR] = internal_real_error
            
            unknown = yield future_unknown
            response[JOB_RESULT_UNKNOWN] = unknown
            
            fetching_code_failed = yield future_fetching_code_failed
            response[JOB_RESULT_FETCHING_PROJECT_CODE_FAILED] = fetching_code_failed
            
            launching_failed = yield future_launching_failed
            response[JOB_RESULT_LAUNCHING_PROJECT_FAILED] = launching_failed
            
            building_failed = yield future_building_failed
            response[JOB_RESULT_BUILDING_PROJECT_FAILED] = building_failed
            
            #success real + failure real
            response["total real"] = success_real+failure_real
        except Exception, e:
            raise tornado.web.HTTPError(500, e)

        self.write(json_encode(response))
        self.finish()
    
    
    @tornado.gen.coroutine
    def __handle_job_status_stats(self,project):
        
        response = {}
        
        future_total = self.jobs_collection.find({"project": project}).count()
        future_subm = self.jobs_collection.find({"project": project,"status": JOB_STATUS_SUBMITTED}).count()
        future_sim_start = self.jobs_collection.find({"project": project,"status": JOB_STATUS_SIM_START}).count()
        future_real_start = self.jobs_collection.find({"project": project,"status": JOB_STATUS_REAL_START}).count()
        future_download = self.jobs_collection.find({"project": project,"status": JOB_STATUS_DOWNLOAD}).count()
        future_sim_run = self.jobs_collection.find({"project": project,"status": JOB_STATUS_SIM_RUN}).count()
        future_wait_real = self.jobs_collection.find({"project": project,"status": JOB_STATUS_WAITING_REAL}).count()
        future_real_run = self.jobs_collection.find({"project": project,"status": JOB_STATUS_REAL_RUN}).count()
        future_finished = self.jobs_collection.find({"project": project,"status": JOB_STATUS_FINISHED}).count()
        try:
            total = yield future_total
            response["total"] = total
            
            subm = yield future_subm
            response[JOB_STATUS_SUBMITTED] = subm
            
            sim_start = yield future_sim_start
            response[JOB_STATUS_SIM_START] = sim_start
            
            real_start = yield future_real_start
            response[JOB_STATUS_REAL_START] = real_start
            
            download = yield future_download
            response[JOB_STATUS_DOWNLOAD] = download
            
            sim_run = yield future_sim_run
            response[JOB_STATUS_SIM_RUN] = sim_run
            
            wait_real = yield future_wait_real
            response[JOB_STATUS_WAITING_REAL] = wait_real
            
            real_run = yield future_real_run
            response[JOB_STATUS_REAL_RUN] = real_run
            
            finished = yield future_finished
            response[JOB_STATUS_FINISHED] = finished

        except Exception, e:
            raise tornado.web.HTTPError(500, e)

        self.write(json_encode(response))
        self.finish()