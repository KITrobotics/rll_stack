FROM rll-base

COPY schunk_gripper_egl90 /home/rll_user/ws/src/schunk_gripper_egl90
COPY moveit /home/rll_user/ws/src/moveit
COPY moveit_wrapper /home/rll_user/ws/src/moveit_wrapper
COPY rll_moving_robots /home/rll_user/ws/src/rll_moving_robots

# install dependencies as root
USER root
RUN rosdep install --from-paths src --ignore-src -r -y

# build
USER rll_user
RUN . /opt/ros/kinetic/setup.bash; catkin build
