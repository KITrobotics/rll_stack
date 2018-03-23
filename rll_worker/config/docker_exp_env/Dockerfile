# environment to run the experiment's code

FROM ros:kinetic-ros-base-xenial

ARG ros_master_ip

RUN apt-get update && apt-get install -y python-catkin-tools ros-kinetic-tf-conversions

RUN useradd -ms /bin/bash rll_user
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/rll_user/.bashrc

USER rll_user

env ROS_MASTER_URI=http://${ros_master_ip}:11311
#env ROS_HOSTNAME=exp_container
#env ROS_IP=${ros_master_ip}

RUN mkdir -p /home/rll_user/ws/src/
COPY ./run_exp.sh /home/rll_user/ws

WORKDIR /home/rll_user/ws
