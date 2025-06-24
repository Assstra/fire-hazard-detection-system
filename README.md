# Fire hazard prevention system

The goal of this project is to identify an early stage fire in a bulding or outside in zone without internet connection. This project is done by Adrien, Rémi and Matéo for the purpose of our intership at OIT: Osaka's Institute of Technology.

## Robot

The robot logic is available in the `/robot/src` folder. As it works with ROS noetic and rviz, only the scripts are included. To run the robot in a simulation, you'll need to have set up ROS and catkin, as well as [KMiyawaki/oit_navigation_minibot_light_01](https://github.com/KMiyawaki/oit_navigation_minibot_light_01) within the catkin folder.

A makefile is available to quickly iterate over the work. From within the `/robot` folder:
1. Start the simulation with `make start` (pay attention to the map used, as it may be unavailable)
2. Run your script with `make run`
