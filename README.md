# SUAS ROS 

This document has instructions to get you started. Reach each section carefully.

## Setup (All Platforms)

If you have a Windows, Mac or Linux on your desktop you can use Docker and Development Containers in vscode to do local development. However this container is currently only validated on Ubuntu. Windows has an issue where the GUI cannot be enabled and Mac is untested.


1. [Install vscode](https://code.visualstudio.com/)
1. [Install Docker Desktop](https://www.docker.com/products/docker-desktop/) 
    * Docker command line is sufficient on Linux
1. Follow the [Dev Containers Tutorial](https://code.visualstudio.com/docs/devcontainers/tutorial)
1. Follow the linked instructions on how to [clone a repository locally](https://code.visualstudio.com/docs/sourcecontrol/intro-to-git#_open-a-git-repository), login to GitHub and when prompted, put in the repository URL:

    ```
    https://github.com/CabrilloRoboticsClub/SUAS_ros.git    
    ```
1. You will also need to grab the recursive repositories:
    ```
    git submodule update --init --recursive
    ```
1. And (for Ubuntu) enable gui passthrough:
    ```
    xhost +local:
    ```

1. You will be prompted to re-open the folder in a **development container**. Select yes. Confirm that you have the devconainer open by looking at the bottom left corner of VScode. It should look like the picture below:

    ![](doc/dev-container.png)

1. Create a Terminal in your vscode window with the `Terminal -> New Terminal` menu. 

## Quick Start 

Here are some steps to test your repository. These steps work on my setup at home. The simulation stack is very heavy and may require a reasonably powerful GPU. **Run this command in the dev container terminal.**

1. Build the software in your dev container: 

    ```
    make
    ```

1. Source the `setup.bash` file after the build is complete.

    ```
    source install/setup.bash
    ```

1. Launch a simulation: 

    ```
    ros2 launch ardupilot_gz_bringup iris_runway.launch.py
    ```

1. Control the simulation with a flight controller. There are a number of programs available. I have had success with [QGroundControl](https://qgroundcontrol.com/). 

## Tutorials 

Ardupilot has official tutorials here:

* [Install ROS2](https://ardupilot.org/dev/docs/ros2.html)
* [ROS 2 with SITL in Gazebo](https://ardupilot.org/dev/docs/ros2-gazebo.html)
* [ROS 2 waypoint goal inerface](https://ardupilot.org/dev/docs/ros2-waypoint-goal-interface.html)
* [Cartographer SLAM with ROS 2 in SITL](https://ardupilot.org/dev/docs/ros2-cartographer-slam.html)

None of the tutorial's setup steps are necessary. The development container is fully setup. 
