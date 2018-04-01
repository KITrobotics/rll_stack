# environment to run the experiment's code

FROM ros:kinetic-ros-base-xenial

ARG ros_master_ip

RUN apt-get update && apt-get install -y python-catkin-tools ros-kinetic-tf-conversions

RUN useradd -ms /bin/bash rll_user
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/rll_user/.bashrc

USER rll_user

RUN HOME=/home/rll_user rosdep update

env ROS_MASTER_URI=http://${ros_master_ip}:11311
#env ROS_HOSTNAME=exp_container
#env ROS_IP=${ros_master_ip}

RUN mkdir -p /home/rll_user/ws/src/
COPY ./run_exp.sh /home/rll_user/ws

WORKDIR /home/rll_user/ws

RUN . /opt/ros/kinetic/setup.bash; catkin_init_workspace; git clone https://gitlab.ipr.kit.edu/rll/rll_sdk.git src/rll_sdk; \
    catkin build
