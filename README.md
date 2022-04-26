# First Assignment of the *Research Track 2 course* - Ros2 branch
## Packages required
- `rt2_assignment1` [main branch](https://github.com/piquet8/rt2_assignment1) in a ros workspace (in the `/root` folder, source ros.sh)
- `rt2_assignment1` [ros2 branch](https://github.com/piquet8/rt2_assignment1/tree/ros2) in a ros2 workspace (in the `/root` folder, source ros2.sh)
- `ros1_bridge` ([info])(https://github.com/ros2/ros1_bridge)) in a ros12 workspace (in thr `/root` folder, source ros12.sh)
## Description of the branch
### Src folder
- [position_service.cpp](https://github.com/piquet8/rt2_assignment1/blob/ros2/src/position_service.cpp): it contains the position server. It's designed to be compatible with `ROS2`, as components, so by using the `ros1_bridge`, they can be interface with the ros nodes and the simulation in Gazebo. The `go_to_point` has been implemented as a service. I have also created and implemented: 
  - a launch file to start the container manager and the components
  - a script to launch all required nodes and the simulation
- [state_machine.cpp](https://github.com/piquet8/rt2_assignment1/blob/ros2/src/state_machine.cpp): it contains the Robot FSM
### Srv folder 
The srv are the same described previously, you can check the descripton in the [main branch](https://github.com/piquet8/rt2_assignment1) 
- [Command.srv](https://github.com/piquet8/rt2_assignment1/blob/action/srv/Command.srv)
- [Position.srv](https://github.com/piquet8/rt2_assignment1/blob/action/srv/Position.srv)
- [RandomPosition.srv](https://github.com/piquet8/rt2_assignment1/blob/action/srv/RandomPosition.srv)
### Mapping rules
We nedd to create a [mapping_rules.yaml](https://github.com/piquet8/rt2_assignment1/blob/ros2/mapping_rules.yaml) file in in the ros2 package for compiling the bridge 
## Rqt-graph
## How to launch
To run the project, please follow these steps:
1. Create three .sh files in your /root directory: the first one, `ros.sh`, should contain these lines:
```
#!/bin/bash
source /root/my_ros/devel/setup.bash
```
The second one, `ros2.sh`, should contain:
```
#!/bin/bash
source /root/my_ros2/install/setup.bash
```
The third one, `ros12.sh`, should contain:
```
#!/bin/bash
source /root/my_ros/devel/setup.bash
source /root/my_ros2/install/local_setup.bash
```
2. Now the configuration should be ready, open a new shell and run:
```
source ros.sh
```
3. Here, in the same shell, start the simulation with this line:
```
roslaunch rt2_assignment1 sim2.launch
```
4. Now open a new terminal and insert:
```
source ros12.sh
```
5. Then to start the bridge, in the same terminal, run:
```
  ros2 run ros1_bridge dynamic_bridge
```
6. Finally in a new terminal insert:
```
source ros2.sh
```
7. and then you can launch:
```
  ros2 launch rt2_assignment1 ros2_launch.py  
```
