# AMR22-FP3 
# Safe robot navigation in crowds with TIAGo
The aim of this project is to develop a safe navigation framework for the TIAGo robot moving in a human crowd.
Our approach is based on the paper of Vulcano et al., where a sensor-based scheme is presented. This scheme consists of two modules, the Crowd Prediction and Motion Generation modules, which run sequentially during every sampling interval. Our setup is implemented in Python using ROS and to validate our implementation multiple experiments are performed on Gazebo in scenarios of different complexity.

This project was developed for The Autonomous and Mobile Robotics Course 2022/2023 for the Master on Artificial Intelligence and Robotics, Sapienza University of Rome by:
- *[Antonio Purificato](https://github.com/antoniopurificato)* [@**antoniopurificato**]
- *[Flavio Galasso](https://github.com/redmodder)* [@**FlavioGalasso**]
- *[Salvatore Falciglia](https://github.com/falciglia)* [@**falciglia**]

For all the details about the theory or the implementative choices you can refer to the [report](https://drive.google.com/file/d/1_frmXtmcYm_d3X3t86k6Nnk6FJ9AVsak/view?usp=sharing) or the [slides](https://docs.google.com/presentation/d/1zWcRXxjLLXlvheeO4xfYQoIa5k-DK6qY/edit?usp=sharing&ouid=117663982450336711859&rtpof=true&sd=true);
## Installation
To make sure everything is working properly make sure you have Ubuntu 18.04 with
ROS Melodic. Install catkin_tools, create a catkin workspace and clone this
repository in the `src` folder. Make sure you are compiling in *Release* mode
by properly setting your catkin workspace:
```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Build your code by running the following command:
```bash
catkin build
```

## Setup
Install the required packages for Python 2.7
```bash
pip install -r requirements.txt
```

## Usage
To run the Gazebo simulation:
```bash
roslaunch crowd_navigation_gazebo tiago_gazebo.launch public_sim:=true end_effector:=pal-gripper world:=WORLD
```
where `WORLD` is one of the worlds in the folder `crowd_navigation_gazebo/worlds` (e.g. `5_humans` or `10_humans`).

Remember that if you run regulation experiments (as `5_humans`, `10_humans` or `15_humans`) you have to go to `crowd_navigation_core/src/CommonVars.py` and change `TASK_TYPE=‘REG’` and also modify `NUM_ACTORS` depending on the number of actors in your simulation.
If you use all the other trajectory tracking experiments you have to change `TASK_TYPE=‘8’`. In this file you can also change some other parameters as the number of clusters `K` or you can toggle the printing of debug information from the different modules in your terminal.

To run the crowd prediction module:
```bash
roslaunch crowd_navigation_core crowd_prediction.launch
```

To run the *Debugger*:
```bash
roslaunch crowd_navigation_core debug_tool.launch
```

To run the motion generation module:
```bash
roslaunch crowd_navigation_core motion_generation.launch
```
