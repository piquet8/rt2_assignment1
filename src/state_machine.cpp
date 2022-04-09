/** @package rt2_assignment1
*
*@file state_machine.cpp
*@brief This node implements the state machine
*@author Gianluca Piquet
*@version 0.1
*@date 03/04/2022
*
*
*
*@details
*
* Subsribes to: <BR>
* [None]
*
* Publishes to: <BR>
* [None]
*
* Clients: <BR>
* /position_server
*
* Services: <BR>
* /user_interface
*
* Action Client: <BR>
* /go_to_point
* 
* Description: <BR>
*
*In this node we have: communication with the user interface, which allows the movement of the robot to be 
*triggered if the user enters the value 1 using a Boolean value; communication as a request to the 
*position_service node of a random position to be reached by the robot; communication of the target position 
*to the action server and waiting to receive this position
*
*/


#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"

// action library used from implementing simple actions
#include <actionlib/client/simple_action_client.h>
//defines the possible goal states
#include <actionlib/client/terminal_state.h>
//include action messages generated from ReachGoal action
#include <rt2_assignment1/ReachGoalAction.h>


bool start = false;

/**
*@brief This function implents the callback of the service /user_interface for the server
*@param req is the request received from the client of the user_interface.py the field command in the srv 
*file
*@return a boolean value
*
*This function is used to manage the value of the variable start: start is initialised to True if the 
*command received consists of a string 'start', otherwise it is initialised to False
*
*/

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

/**
*@brief This is the main function
* 
*@return the value 0
*
* In this function it's implemented:
* -# the node 'state machine'
* -# the service for the /user_interface
* -# the client for the /position_server
* -# the action client for the go_to_point
* -# the custom messages RandomPosition 
* -# the custom messages ReachGoalGoal
*
*When the start variable is set to True, the RandomPosition service is called and we wait for the action 
*server to start. Once started, the goal is set with the ranodm position that was received and sent to the 
*action server. Finally, a timer checks whether the action is finished or not.
*
*
*/

int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   //define the action client
   actionlib::SimpleActionClient<rt2_assignment1::ReachGoalAction> ac("go_to_point", true);
   rt2_assignment1::RandomPosition rp;
   //a goal message is created
   rt2_assignment1::ReachGoalGoal goal;
   
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   
      
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_rp.call(rp);
   		ROS_INFO("Waiting for action server to start....");
   		//wait for the action server to start before continuing
        ac.waitForServer();
   		
   		goal.x = rp.response.x;
   		goal.y = rp.response.y;
   		goal.theta = rp.response.theta;
   		std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " << goal.theta << std::endl;
   	ac.sendGoal(goal);
        
        //the action client waits for the goal to finish before continuing, the timeout is set to 100 seconds, this means that after 100 seconds 
        //the function will return with false if the goal has not finished
   		bool finished_before_timeout = ac.waitForResult(ros::Duration(100.0));
   		
   		if (finished_before_timeout)
   		{
   			ROS_INFO("ACTION FINISHED: Position Reached");
   		}
   		else
   			ROS_INFO("ACTION DOESN'T FINISHED: the time is over");
   	}
   }
   return 0;
}
