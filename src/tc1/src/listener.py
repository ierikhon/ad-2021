#! /usr/bin/python
import rospy
import time
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan, fabs, pi, sqrt


class FollowController:
	def __init__(self):
		self.my_x = 0
		self.my_y = 0
		self.my_rad = 0

		rospy.init_node('follow_controller')
		rospy.Subscriber('turtle1/pose', Pose, self.callback1)
		rospy.Subscriber('turtle2/pose', Pose, self.callback2)
		self.pub2 = rospy.Publisher('turtle2/cmd_vel', Twist, queue_size=2)

	def callback1(self, msg):
		d_x = self.my_x - msg.x
		d_y = self.my_y - msg.y
		distance = sqrt(d_x**2 + d_y**2)

		msg_to_send = Twist()
		msg_to_send.linear.x = 1
		if distance < 0.5:
			msg_to_send.linear.x = 0
			msg_to_send.angular.z = 0
		else:
			new_angle = atan(d_y/d_x)
			if d_x > 0:
				new_angle = new_angle if d_y > 0 else 2*pi - fabs(new_angle)
			else:
				new_angle = new_angle + pi if d_y < 0 else pi - fabs(new_angle)
			msg_to_send.angular.z = new_angle - self.my_rad

		self.pub2.publish(msg_to_send)

	def callback2(self, msg):
		self.my_x = msg.x
		self.my_y = msg.y
		self.my_rad = msg.theta + pi


FollowController()
rospy.spin()

