#! /usr/bin/env python

"""
.. module:: user_interface
   :platform: Unix
   :synopsis: Python module for the user Interface
   
.. moduleauthor:: Gianluca Piquet gianlucapiquet8@gmail.com

This node implements an user interface, through which the user can 
insert 1 to make the robot move and press 0 to stop it.

Service:
   /user_interface

"""



import rospy
import time
from rt2_assignment1.srv import Command
# this line imports the actionlib library 
import actionlib
# this line imports the generated messages
import rt2_assignment1.msg	
from geometry_msgs.msg import Twist 

def main():
    """
	
    This function initializes the ROS node and waits for the 
    user to insert *start* or *stop* to control the robot, by relying 
    on the`rospy <http://wiki.ros.org/rospy/>`_ module.
	
    The user message is passed to the service ``user_interface``, 
    advertised by :mod:`go_to_point`.
	
    """
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
