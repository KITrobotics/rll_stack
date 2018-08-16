cmake_minimum_required(VERSION 2.8.3)
project(rll_common)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rll_config
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  config/rll.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
