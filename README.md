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
- [go_to_point.py](https://github.com/piquet8/rt2_assignment1/blob/action/scripts/go_to_point.py)
- [user_interface.py](https://github.com/piquet8/rt2_assignment1/blob/action/scripts/user_interface.py)
### Src folder
- [poistion_service.cpp](https://github.com/piquet8/rt2_assignment1/blob/action/src/position_service.cpp)
- [state_machine.cpp](https://github.com/piquet8/rt2_assignment1/blob/action/src/state_machine.cpp)
### Srv folder 
- [Command.srv](https://github.com/piquet8/rt2_assignment1/blob/action/srv/Command.srv)
- [Position.srv](https://github.com/piquet8/rt2_assignment1/blob/action/srv/Position.srv)
- [RandomPosition.srv](https://github.com/piquet8/rt2_assignment1/blob/action/srv/RandomPosition.srv)
### Launch folder
- [sim.launch](https://github.com/piquet8/rt2_assignment1/blob/action/launch/sim.launch)
