#! /usr/bin/env python
"""
.. module:: go_to_point
    :platform: Unix
    :synopsis: Python module for move to robot to the target
   
.. moduleauthor:: Gianluca Piquet gianlucapiquet8@gmail.com

This ROS node implements the movement of the robot within the simulation

Subscribes to:
	/odom topic where the simulator publishes the robot position

Publishes to:
	/cmd_vel the desired robot position

Service :
	/go_to_point to start the robot motion

"""




import rt2_assignment1.msg
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_assignment1.srv import Position
import math
# imports the actionlib library used for implementing simple actions
import actionlib

# robot state variables
position_ = Point()
"""Point: current robot position
"""
yaw_ = 0
"""Pose: current robot orientation
"""
position_ = 0
"""Float: current robot angle
"""
state_ = 0
"""Int: current state of server
"""
pub_ = None
""" it publishes a twist messages on the /cmd_vel topic
"""

# parameters for control
yaw_precision_ = math.pi / 9  
"""Float: yaw acc +/- 20 degree allowed
"""
yaw_precision_2_ = math.pi / 90  
"""Float: tight yaw +/- 2 degree allowed
"""
dist_precision_ = 0.1
"""Float: linear distance allowed
"""

kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

def clbk_odom(msg):
    '''
    Callback: clbk_odom
    
    Args:
      msg(Twist): data retrived by c,d_vel topic
    Returns:
      None
    '''
    
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


class ReachGoalAction(object):
	"""
	The ReachGoalAction class implements the Goal action
	
	"""
	_feedback = rt2_assignment1.msg.ReachGoalFeedback()
	_result = rt2_assignment1.msg.ReachGoalResult()
	_goal = rt2_assignment1.msg.ReachGoalGoal()

	def __init__(self,name):
		'''Constructor method
		'''
		self._action_name = name
		# here a simple action server is created
		self._as = actionlib.SimpleActionServer(self._action_name, rt2_assignment1.msg.ReachGoalAction,execute_cb=self.execute_cb,auto_start=False)
		self._as.start()
	
	# this is the execute callback function that we'll run everytime a new goal is received
	def execute_cb(self,goal):
		'''
		Callback: execute_cb
		
		This function takes as argument the goal variable whose value is provided from the action client 
		in the state_machine.cpp. Then the feedback of the action message is constantly updated as the 
		current pose of the robot. The desired_position is set as the goal of the action. The change_state
		function is given with argument zero so that the robot can start fixing its own yaw before proceeding by reaching the goal
		
		Args:	
		  goal(Float): the robot position goal
		Returns:
		  goal(float)
		  
		'''
		global position_, yaw_precision_, yaw_, state_, pub_
 	       
		r = rospy.Rate(1)
		success = True
        
 	       # initialising feedback
		self._feedback.up_x = position_.x
		self._feedback.up_y = position_.y
		self._feedback.up_theta = yaw_

        

        	# start executing the action
		desired_position = Point()
		desired_position.x = goal.x
		desired_position.y = goal.y
		des_yaw = goal.theta
		self.change_state(0)
		
		while True:
            		if self._as.is_preempt_requested():
                		rospy.loginfo('%s: Preempted' % self._action_name)
                		self._as.set_preempted()
                		success = False
                		break
            	# updating feedback (run-time)
            		self._feedback.up_x = position_.x
            		self._feedback.up_y = position_.y
            		self._feedback.up_theta = yaw_
            	# publish the feedback
            		self._as.publish_feedback(self._feedback)
            		if state_ == 0:
                		self.fix_yaw(desired_position)
            		elif state_ == 1:
                		self.go_straight_ahead(desired_position)
            		elif state_ == 2:
                		self.fix_final_yaw(des_yaw)
            		elif state_ == 3:
                		self.done()
                		break
 		
	def change_state(self,state):
		
		''' 
		Change_state function
		
		This function updates the state
		
		Args:
		  state(int): the state of the robot
		Returns:
		  None
		'''
		global state_
		state_ = state
		print ('State changed: [%s]' % state_)


	def normalize_angle(self, angle):    
		'''
		Function for normalizing the angle between -pi and pi.	
		Args:
			angle(Float): the input angle		
		Returns:
			angle(Float): the normalized angle.
		
		'''
		if(math.fabs(angle) > math.pi):
			angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
		return angle

	def fix_yaw(self,des_pos):
		'''
		Fix_yawl function
		
		This function elaborates the robot orientation between x and y and sets the angular velocity
		for achieving the desired robot position
		
		Args:
		  des_pos(Point): x and y desired
		Returns:
		  None
		'''
		desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
		err_yaw = self.normalize_angle(desired_yaw - yaw_)
		rospy.loginfo(err_yaw)
		twist_msg = Twist()
		if math.fabs(err_yaw) > yaw_precision_2_:
        		twist_msg.angular.z = kp_a*err_yaw
        		if twist_msg.angular.z > ub_a:
            			twist_msg.angular.z = ub_a
        		elif twist_msg.angular.z < lb_a:
            			twist_msg.angular.z = lb_a
		pub_.publish(twist_msg)
    # state change conditions
		if math.fabs(err_yaw) <= yaw_precision_2_:
        		self.change_state(1)


	def go_straight_ahead(self,des_pos):
		'''
		Go_straight_ahead function
		
		This function elaborates the robot orientation between x and y to reach the target point
		
		Args:
		  des_pos(Point) x and y desired
		Returns:
		  None
		'''
		desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
		err_yaw = desired_yaw - yaw_
		err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
		err_yaw = self.normalize_angle(desired_yaw - yaw_)
		rospy.loginfo(err_yaw)
   		
		if err_pos > dist_precision_:
        		twist_msg = Twist()
        		twist_msg.linear.x = 0.3
        		if twist_msg.linear.x > ub_d:
            			twist_msg.linear.x = ub_d
		        twist_msg.angular.z = kp_a*err_yaw
		        pub_.publish(twist_msg)
		else:
        		self.change_state(2)
    # state change conditions
		if math.fabs(err_yaw) > yaw_precision_:
        		self.change_state(0)
        		

	def fix_final_yaw(self,des_yaw):
		'''
		Fix_final_yawl function
		
		This fucntion compute the error between the desired orientation and the current one
		
		Args:
		  des_yawl(Float): epected orientation
		Returns:
		  None
		'''
		err_yaw = self.normalize_angle(des_yaw - yaw_)
		rospy.loginfo(err_yaw)
		twist_msg = Twist()
		if math.fabs(err_yaw) > yaw_precision_2_:
        		twist_msg.angular.z = kp_a*err_yaw
        		if twist_msg.angular.z > ub_a:
            			twist_msg.angular.z = ub_a
        		elif twist_msg.angular.z < lb_a:
            			twist_msg.angular.z = lb_a
		pub_.publish(twist_msg)
    	# state change conditions
		if math.fabs(err_yaw) <= yaw_precision_2_:
        		self.change_state(3)
        
	def done(self):
		'''
		Done function
		
		This function sets the velocities to zero and marks the goal target as succeeded 
		
		Args:
		  None
		Returns:
		  None
		'''
		twist_msg = Twist()
		twist_msg.linear.x = 0
		twist_msg.angular.z = 0
		pub_.publish(twist_msg)
		# once the action is done, the action server notifies the action client that the goal is complete by callung set_succeeded
		self._result.ok = True
		rospy.loginfo('Position Reached')
		self._as.set_succeeded(self._result)
    
def main():
    """
    Main function
    
    This function initialises all the pub/sub and the server for the :class:'ReachGoalAction'
    
    Args:
      None
    Returns:
      None
    """
    global pub_
    rospy.init_node('go_to_point')
    #creates the action server
    action_server = ReachGoalAction('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    rospy.spin()

if __name__ == '__main__':
    main()
