# First Assignment of the *Research Track 2 course* 
In this assignment, starting from the initial package which you can find in the repository https://github.com/CarmineD8/rt2_assignment1 two different versions have been created which you can find in the [action](https://github.com/piquet8/rt2_assignment1/tree/action) branch and the [ros2](https://github.com/piquet8/rt2_assignment1/tree/ros2) branch. In addition, a [Vrep scene](https://github.com/piquet8/rt2_assignment1/blob/main/coppeliascene.ttt) containing a robot interacting with the simulation was implemented in the [main](https://github.com/piquet8/rt2_assignment1/tree/main) branch. 
## The 'action' branch
This branch should contain the same packages in ROS, the main differnce is in the go_to_point node modelled as a ROS action server. In this way the robot FSM node should now implement the possibility fot the user of cancel the goal at any time. If you are interest about that you can check it in the [action branch](https://github.com/piquet8/rt2_assignment1/tree/action) 
## The 'ros2' branch
In the branch ros2, the cpp nodes are written for ROS2, as components, in this way using the [ros1_bridge](https://github.com/ros2/ros1_bridge), they can be interact with the ROS nodes and with the simulation in Gazebo. To check it you can see it here [ros2 branch](https://github.com/piquet8/rt2_assignment1/tree/ros2) 
## Vrep scene
In this branch we can also find the scene [coppeliascene.ttt](https://github.com/piquet8/rt2_assignment1/blob/main/coppeliascene.ttt) that implements a robot simulation on Coppeliasim. In particular there is a robot_pioneer_ctrl placed in the simulation environment, its movement is controlled by a ROS node of the rt2_assignment1 packages. To communicate whit the robot a publisher `/odomtopic` has been decleared in the robot script, it communicates the actual position of the robot that will be exploit by the subscriber of the `/odom` topic in the node [go_to_point.py](https://github.com/piquet8/rt2_assignment1/blob/main/scripts/go_to_point.py). To set the actuator velocity instead a subscriber to the `/cmd_vel` topic has been introduced by means twist message published against to the [go_to_point.py](https://github.com/piquet8/rt2_assignment1/blob/main/scripts/go_to_point.py) script.
## Packages required
- [Ros1_bridge](https://github.com/ros2/ros1_bridge) allows the communication among ROS and ROS2 nodes
- [SimExtRos](https://github.com/CoppeliaRobotics/simExtROS2) is the ROS2 Interface plugin for CoppeliaSiam
- [Coppeliasim](https://www.coppeliarobotics.com/) is the simulator that allows to inregrate Vrep with ROS
## Description of the main branch
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
### Launch folder
- [sim.launch](https://github.com/piquet8/rt2_assignment1/blob/main/launch/sim.launch) launch file for starting the four nodes and the simulation 
- [sim2.launch](https://github.com/piquet8/rt2_assignment1/blob/main/launch/sim2.launch) Launch file for starting the same things of the [sim.launch](https://github.com/piquet8/rt2_assignment1/blob/main/launch/sim.launch) without the *state_machine.cpp* and the *position_service.cpp*. We used this launcher in the interaction between ROS and ROS2.
- [coppelia.launch](https://github.com/piquet8/rt2_assignment1/blob/main/launch/coppelia.launch) Launch file for starting the four nodes interacting with Vrep
## How to run
To launch the simulation, please:
1. Download Coppeliasim from here https://coppeliarobotics.com/downloads
2. Launch the ROS master
```
roscore &
```
3. Then in the second shell, to start the Coppelia environment, please run:
```
./coppeliasim.sh
```
4. Now you can load the scene [coppeliascene.ttt](https://github.com/piquet8/rt2_assignment1/blob/main/coppeliascene.ttt) (Menu bar --> File --> Open Scene)
6. Now we are ready, open a new terminal and launch
```
roslaunch rt2_assignment1 coppelia.launch 
```


