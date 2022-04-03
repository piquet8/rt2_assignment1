## @package rt2_assignment1
# \file user_interface.py 
# \brief This node implements an user interface for the communication with the costumer
# \author Gianluca Piquet
# \version 0.1
# \date 03/04/2022
#
# \details
#
# Subsribes to: <BR>
# [None]
#
# Publishes to: <BR>
# /cmd_vel
#
# Clients: <BR>
# /position_server
#
# Services: <BR>
# /user_interface
#
# Action Client: <BR>
# /go_to_point
#
# Description: <BR>
#
# This node implements the user interface, the user is prompted as input to enter 1 if they want to start the 
#robot and 0 if they want to stop it.
#

import rospy
import time
from rt2_assignment1.srv import Command
# this line imports the actionlib library 
import actionlib
# this line imports the generated messages
import rt2_assignment1.msg	
from geometry_msgs.msg import Twist

## 
# This is the main function
#
# ui_client: implements the service client of user_interface type that has as argument the Command service #to start/stop robot behaviours according to user's control  
#
# client: implements the action client of the go_to_point Action and has as argument the message of type #ReachGoal action that we can find in the rt2assignment1 package
#
# pub: it's the publsiher variable of the cmd_vel, it publishes a twist message to stop the robot setting #the velocities equal to zero. 
#
# x: stores the input value of the user

def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    # create the action client
    action_client = actionlib.SimpleActionClient('go_to_point', rt2_assignment1.msg.ReachGoalAction)
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            # here implement the behavior if the user ask to cancel the action
            action_client.cancel_all_goals()
            twist_msg = Twist()
            twist_msg.linear.x = 0	
            twist_msg.linear.y = 0
            twist_msg.angular.z = 0
            pub_.publish(twist_msg)
            
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
