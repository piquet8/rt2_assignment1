# First Assignment of the Research Track 2 course 

In this assignment, starting from the initial package which you can find in the repository https://github.com/CarmineD8/rt2_assignment1 two different versions have been created which you can find in the [action](https://github.com/piquet8/rt2_assignment1/tree/action) branch and the [ros2](https://github.com/piquet8/rt2_assignment1/tree/ros2) branch. In addition, a [Vrep scene](https://github.com/piquet8/rt2_assignment1/blob/main/coppeliascene.ttt) containing a robot interacting with the simulation was implemented in the [main](https://github.com/piquet8/rt2_assignment1/tree/main) branch.  

## Packages required
- [Ros1_bridge](https://github.com/ros2/ros1_bridge) allows the communication among ROS and ROS2 nodes
- [SimExtRos](https://github.com/CoppeliaRobotics/simExtROS2) is the ROS2 Interface plugin for CoppeliaSiam
- [Coppeliasim](https://coppeliarobotics.com/downloads) is the simulator that allows to inregrate Vrep with ROS

## Description of the main nodes
### Scripts Folder
- [user_interface.py](https://github.com/piquet8/rt2_assignment1/blob/main/scripts/user_interface.py) implents the user interface, if the user inserts '1' the robot starts to move instead inserting the value '0' the robot stop
- [go_to_point.py](https://github.com/piquet8/rt2_assignment1/blob/main/scripts/go_to_point.py) implents a service to move and drive the robot among different target 



The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment.
To launch the node, please run:
```
roslaunch rt2_assignment1 sim.launch
```

