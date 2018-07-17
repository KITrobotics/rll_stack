# base image that includes the RLL SDK

FROM osrf/ros:kinetic-desktop-full-xenial

# Get latest Gazebo stable release
RUN apt-get update && apt-get -y install wget python-catkin-tools nano
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update && apt-get -y dist-upgrade

RUN useradd -ms /bin/bash rll_user
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/rll_user/.bashrc

USER rll_user

RUN HOME=/home/rll_user rosdep update

RUN mkdir -p /home/rll_user/ws/src/

WORKDIR /home/rll_user/ws

RUN . /opt/ros/kinetic/setup.bash; catkin_init_workspace; git clone https://gitlab.ipr.kit.edu/rll/rll_sdk.git src/rll_sdk

# install dependencies as root
USER root
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

# build
USER rll_user
RUN . /opt/ros/kinetic/setup.bash; catkin build
