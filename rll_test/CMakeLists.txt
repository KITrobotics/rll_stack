cmake_minimum_required(VERSION 2.8.3)
project(rll_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rll_test
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

install(FILES
  data/test_job_stats_db.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
