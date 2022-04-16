# First Assignment of the Research Track 2 course 

In this assignment, starting from the initial package which you can find in the repository https://github.com/CarmineD8/rt2_assignment1 two different versions have been created which you can find in the [action](https://github.com/piquet8/rt2_assignment1/tree/action) branch and the [ros2](https://github.com/piquet8/rt2_assignment1/tree/ros2) branch. In addition, a [Vrep scene](https://github.com/piquet8/rt2_assignment1/blob/main/coppeliascene.ttt) containing a robot interacting with the simulation was implemented in the [main](https://github.com/piquet8/rt2_assignment1/tree/main) branch.  

## Packages required
- [Ros1_bridge](https://github.com/ros2/ros1_bridge) allows the communication among ROS and ROS2 nodes
- [SimExtRos](https://github.com/CoppeliaRobotics/simExtROS2) is the ROS2 Interface plugin for CoppeliaSiam
- [Coppeliasim](https://coppeliarobotics.com/downloads) is the simulator that allows to inregrate Vrep with ROS
    
## Description of the main branch
### Launch folder
- [sim.launch](https://github.com/piquet8/rt2_assignment1/blob/main/launch/sim.launch) launch file for starting the four nodes and the simulation 
- [sim2.launch](https://github.com/piquet8/rt2_assignment1/blob/main/launch/sim2.launch) Launch file for starting the same things of the [sim.launch](https://github.com/piquet8/rt2_assignment1/blob/main/launch/sim.launch) without the *state_machine.cpp* and the *position_service.cpp*. We used this launcher in the interaction between ROS and ROS2.
- [coppelia.launch](https://github.com/piquet8/rt2_assignment1/blob/main/launch/coppelia.launch) Launch file for starting the four nodes interacting with Vrep
### Scripts folder
- [user_interface.py](https://github.com/piquet8/rt2_assignment1/blob/main/scripts/user_interface.py) implents the user interface, if the user inserts '1' the robot starts to move instead inserting the value '0' the robot stop
- [go_to_point.py](https://github.com/piquet8/rt2_assignment1/blob/main/scripts/go_to_point.py) implents a service to move and drive the robot among different target 
### Src folder
- [state_machine.cpp](https://github.com/piquet8/rt2_assignment1/blob/main/src/state_machine.cpp) is implemented as a service server, the activation is meanaged by the start boolean variable retrieved in the `ui_client` by the [user_interface](https://github.com/piquet8/rt2_assignment1/blob/main/scripts/user_interface.py). This node also implements a service client for both: `/go_to_point` and `/position_server`
- [position_service.cpp](https://github.com/piquet8/rt2_assignment1/blob/main/src/position_service.cpp) is implemented ad a service server node, it replies with a random values for x,y and theta where x and y are found among a certain range of values
### SRV folder
- [Command.srv](https://github.com/piquet8/rt2_assignment1/blob/main/srv/Command.srv) is composed by:
  - as request a string 
  - as reply a boolean
- [Position.srv](https://github.com/piquet8/rt2_assignment1/blob/main/srv/Position.srv) is composed by:
  -  as request a position to reach
  -  as reply a boolean
- [RandomPosition.srv](https://github.com/piquet8/rt2_assignment1/blob/main/srv/RandomPosition.srv) is composed by:
  -  as request min and max values of x and y
  -  as reply random target position (x,y and theta)
 


The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment.
To launch the node, please run:
```
roslaunch rt2_assignment1 sim.launch
```

