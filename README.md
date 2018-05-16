# Robot Learning Lab Stack

## Quick Installation

1. Create a workspace and clone the core repositories:

    `mkdir -p rll_ws/src && cd rll_ws`   
    `catkin_init_workspace`   
    `cd src`   
    `git clone https://gitlab.ipr.kit.edu/rll/rll_stack.git`   
    `git clone https://gitlab.ipr.kit.edu/rll/schunk_gripper_egl90.git`   
    `git clone https://gitlab.ipr.kit.edu/rll/moveit_wrapper.git`   
    `git clone https://gitlab.ipr.kit.edu/rll/moveit.git`   
    `git clone https://gitlab.ipr.kit.edu/rll/iiwa_fri_hw_interface.git`   
    `cd ..`

2. Install dependencies:

    `rosdep install --from-paths src --ignore-src -r -y`

3. Build the workspace:

    `catkin build`

4. Source the workspace:

    `source devel/setup.bash`

## Dependencies

The server and worker need recent versions of the Docker and Motor Python libraries.
Both of them can be installed using ```pip```.
