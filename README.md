# First Assignment of the *Research Track 2 course* - Action branch
## Packages required
In this branch the [Actionlib package](http://wiki.ros.org/actionlib) it used to create:
- servers able to execute long-running tasks
- clinets interacting with them are defined through an .action structure defined in ROS and placed inside the [action](https://github.com/piquet8/rt2_assignment1/tree/action/action) folder of the rt2_assignment1 package. The action messages are generated automatically from the ReachGoal.action file. This file defines:
  - the type and format of the goal
  - the result
  - the feedback topics for the action
## Branch description
The action branch contain the same package of the main branch, but with the go_to_point node modelled as a ROS action server. In this way, the robot FSM node implements mechanisms for possibly cancelling the goal, when the related user command is received in any time.
### Action folder
It contains the instruction for defining the expected action. 
- the `goal` is composed by three flaot32 value represent the target position for the robot (x, y and theta)
- the `result` is composed by a simple boolean value
- the `feedback` is composed by three flaot32 value represent the updated position of the robot (x, y and theta)
### Scripts folder
- [go_to_point.py](https://github.com/piquet8/rt2_assignment1/blob/action/scripts/go_to_point.py) implents a service to move and drive the robot among different target 
- [user_interface.py](https://github.com/piquet8/rt2_assignment1/blob/action/scripts/user_interface.py) implements the interface for the user, when the user inserts '1', the command client calls the state machine (which the start string), instead in the user inserts '0' the goal is cancelled and the robot is stopped in the actual position (to stop the robot we used a twist messsage)
### Src folder
- [poistion_service.cpp](https://github.com/piquet8/rt2_assignment1/blob/action/src/position_service.cpp) is implemented ad a service server node, it replies with a random values for x,y and theta where x and y are found among a certain range of values
- [state_machine.cpp](https://github.com/piquet8/rt2_assignment1/blob/action/src/state_machine.cpp) this node interacts with the [user_interface.py](https://github.com/piquet8/rt2_assignment1/blob/action/scripts/user_interface.py), in particular there is a boolena value that becomes true and then call the [poistion_service.cpp](https://github.com/piquet8/rt2_assignment1/blob/action/src/position_service.cpp) which gives the random goal position takes from the [RandomPosition.srv](https://github.com/piquet8/rt2_assignment1/blob/action/srv/RandomPosition.srv) custom service. Then sends the random position as the action server goal and waits that the robot reaches the target 
### Srv folder 
The srv are the same described previously, you can check the descripton in the [main branch](https://github.com/piquet8/rt2_assignment1) 
- [Command.srv](https://github.com/piquet8/rt2_assignment1/blob/action/srv/Command.srv)
- [Position.srv](https://github.com/piquet8/rt2_assignment1/blob/action/srv/Position.srv)
- [RandomPosition.srv](https://github.com/piquet8/rt2_assignment1/blob/action/srv/RandomPosition.srv)
### Launch folder
- Here there is the [sim.launch](https://github.com/piquet8/rt2_assignment1/blob/action/launch/sim.launch) that allows to starting the simulation
## Rqt-graph
Here we can see a dynamic graph showing what's going on in the system, you can see more graphs [here](https://github.com/piquet8/rt2_assignment1/tree/action/rqt_graph):

![Rqt-graph](https://github.com/piquet8/rt2_assignment1/blob/action/rqt_graph/rqt2.jpeg)
## Documentation 
Here you can find the documentation about the code
## How to run the code
To launch the nodes and the simulation, open a terminal and please run:
```
roslaunch rt2_assignment1 sim.launch
```

