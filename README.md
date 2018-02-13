# Robot Learning Lab Stack

## Quick Installation

1. Create a workspace and clone the core repositories:

    `mkdir -p rll_ws/src && cd rll_ws`   
    `catkin_init_workspace`   
    `git clone https://gitlab.ipr.kit.edu/rll/rll_stack.git src/rll_stack`   
    `git clone https://gitlab.ipr.kit.edu/rll/iiwa_stack.git src/iiwa_stack`

2. Install dependencies:

    `rosdep install --from-paths src --ignore-src -r -y`

3. Build the workspace:

    `catkin build`

4. Source the workspace:

    `source devel/setup.bash`
