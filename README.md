# Robot Learning Lab Stack

Job processing pipeline for the Robot Learning Lab at KIT.

## Components

* RESTful web API for submitting jobs and retrieving job data
    * For an interactive documentation, see the [RLL API doc page](https://rll-apidoc.ipr.kit.edu/)
    * Includes an example web frontend page
    * Server runs on port 8888 by default
* Worker for processing jobs
    * Uses Docker to isolate client code; see the [config directory](rll_worker/config/docker) for the Docker setups of the example projects
    * Can be configured to use a simulation or the real robot
* Test framework with a collection of unit tests for the server and the worker

## Quick Installation in a ROS Workspace

1. Create a workspace and clone the core repositories:

    `mkdir -p rll_ws/src && cd rll_ws`   
    `catkin_init_workspace`   
    `cd src`   
    `git clone https://github.com/KITrobotics/rll_stack.git`   
    `cd ..`

2. Install dependencies:

    `rosdep install --from-paths src --ignore-src -r -y`

3. Build the workspace:

    `catkin build`

4. Source the workspace:

    `source devel/setup.bash`

## Additional Dependencies

The server and worker need recent versions of the Docker and Motor Python libraries.
Both of them are not available from `rosdep`, but they  can be installed using `pip`.

A MongoDB database needs to be accessible.

## Configuration

Database access is configured in the [common config file](rll_common/config/rll.yaml). The directories for storing job data need to be set there, too. You also need a webserver that has access to the job data directory and can serve the files. We recommend configuring Apache for this purpose. You can also use Apache as a proxy in front of the RLL server and the webcams.

The server and worker can be launched in production or test mode. The mode is set with the optional parameter `production` (default: `false`) for the launch file.

Additionally, the worker can be launched with the following arguments:

* `robot`: Namespace the worker should be started in; default: `iiwa`
* `project`: Name of the project the worker should process; default: `greetings`
* `manual`: If `true`, the next job is only executed when `Enter` is pressed; default: `false`
* `mode`: `sim` for simulation, `real` for the real robot; default: `sim`
* `sim_check`: if mode is `real`, the previous check in simulation can be disabled when setting to `false`; default: `true`
