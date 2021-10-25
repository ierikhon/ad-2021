#! /usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan


class ScanFilter:
	def __init__(self):
		rospy.init_node('scan_filter')
		rospy.Subscriber('/base_scan', LaserScan, self.filter)
		self.pub = rospy.Publisher('/base_scan_filtered', LaserScan, queue_size=1)

	def filter(self, msg):
		self.pub.publish(msg)


ScanFilter()
rospy.spin()
