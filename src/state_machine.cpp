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

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}


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
        
        //the action client waits for the goal to finish before continuing, the timeout is set
        //to 100 seconds, this means that after 100 seconds the function will return with false
        //if the goal has not finished
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
