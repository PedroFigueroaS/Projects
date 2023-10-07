# ROS PROJECT Guide to Compile and run the project
GITHUB LINK OF THE PROJECT: https://github.com/PedroFigueroaS/Projects.git

Follow the instructions in the README.md file in order to set up the package in a ros workspace.

In particular, the repository is organized as follows:
- The `src` contains the folder `rosbot_description`, the main folder where all the scripts are contained


## Running the project

In the `rosbot_description` folder, in order to execute the simulation it is necessary to type the next command line in a terminal:

```console
roslaunch rosbot_description rosbot.launch
```

This command allows to launch all the nodes involved in the simulation as well as the gazebo environment.

## Modifying the project

The main node that allows the robot to perform the wall following task is a program written in python. In order to modify it, go to the following address:

```
rosbot_description/src/rosbot_description/scripts/
```
Inside the `scripts` folder, open the file `rosbot_project.py`

## Troubleshooting after launching the run executable

It is possible that after compiling the package in the ROS workspace, the package still doesn't appear, for that it is necessary to instantiate the terminal with the following code: 

```console
source /opt/ros/<distro>/setup.bash
```
In which <distro> is the current ROS version, the project has been programmed in ROS noetic




