#! /usr/bin/env python

# this line imports the generated messages, the action specification generats several messages
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
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

def clbk_odom(msg):
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
	_feedback = rt2_assignment1.msg.ReachGoalFeedback()
	_result = rt2_assignment1.msg.ReachGoalResult()
	_goal = rt2_assignment1.msg.ReachGoalGoal()

	def __init__(self,name):
		self._action_name = name
		# here a simple action server is created
		self._as = actionlib.SimpleActionServer(self._action_name, rt2_assignment1.msg.ReachGoalAction,execute_cb=self.execute_cb,auto_start=False)
		self._as.start()
	
	# this is the execute callback function that we'll run everytime a new goal is received
	def execute_cb(self,goal):
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
    		global state_
    		state_ = state
    		print ('State changed: [%s]' % state_)


	def normalize_angle(self,angle):
    		if(math.fabs(angle) > math.pi):
        		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    		return angle

	def fix_yaw(self,des_pos):
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
        #print ('Yaw error: [%s]' % err_yaw)
        		self.change_state(1)


	def go_straight_ahead(self,des_pos):
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

        #print ('Position error: [%s]' % err_pos)
        		self.change_state(2)
    # state change conditions
   		if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        		self.change_state(0)
        		

	def fix_final_yaw(self,des_yaw):
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
        #print ('Yaw error: [%s]' % err_yaw)
        		self.change_state(3)
        
	def done(self):
    		twist_msg = Twist()
    		twist_msg.linear.x = 0
    		twist_msg.angular.z = 0
    		pub_.publish(twist_msg)
    		self._result.ok = True
    		rospy.loginfo('Position Reached')
    		self._as.set_succeeded(self._result)
    
def main():
    global pub_
    rospy.init_node('go_to_point')
    action_server = ReachGoalAction('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    rospy.spin()

if __name__ == '__main__':
    main()
