# environment to run experimental code

FROM ros:kinetic-ros-core-xenial
MAINTAINER Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>

ARG ros_master_ip

RUN groupadd -r rll_user && useradd --no-log-init -r -g rll_user rll_user
USER rll_user

env ROS_MASTER_URI=http://${ros_master_ip}:11311
#env ROS_HOSTNAME=exp_container
#env ROS_IP=${ros_master_ip}

WORKDIR /home/rll_user/ws
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash"
